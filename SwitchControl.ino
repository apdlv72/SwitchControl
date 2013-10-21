/*
 Copyright (C) Artur Pogoda de la Vega
 Arduino sketch to send DHCP requests to a remote IP and reset a device (e.g. swicth) if that fails too often.
 An interval can be defined (office hours, e.g. 9:00 to 17:00) when there shoud be no automatic reset.
 Time is synchronized from an NTP server.
 Circuit: Relay attached to pin 12
 */
 
#include "SwitchControl.h"

#include <SPI.h>         
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Dhcp.h>

#ifdef WITH_TESTPING
#include <ICMPPing.h>
#endif
#include <avr/eeprom.h>

#define PPRINT(TEXT)   showPgmString(PSTR(TEXT))
#define PPRINTLN(TEXT) showPgmStringLn(PSTR(TEXT))
#define MAX(A,B) ((A)>(B)?(A):(B))

static const uint16_t MAGIC    = 4711;
static const uint8_t PIN_LED   = 13;
static const uint8_t PIN_RELAY = 12;

// local port to listen for UDP packets
static const unsigned int localPort = 2390;      

static const s_config DEFAULT_CONFIG =
{
  magic          : MAGIC,
  // coffee, coffee. 2nd coffee will be overwritten with randoms on 1st time power on.
  mac            : { 0xC0, 0xFF, 0xEE, 0xC0, 0xFF, 0xEE}, 
  mode           : MODE_DHCP,
  retries        :  3,
  timeout        :  4,
  waitTime       : 15,
  pingAddr       : { 10,2,0,1 },
  timeServer     : { 10,2,0,1 },  
  fetchTime      : true,
  timezone       :  2,
  officeStart    :  9,
  officeEnd      : 19,
  officeEndFr    : 17,
  fixedIp        : { 0,0,0,0 }
};

static const int NTP_PACKET_SIZE =  48; // NTP time stamp is in the first 48 bytes of the message
#ifdef WITH_TESTPING
static const int UDP_PACKET_SIZE = 256; // max. length of a UDP packet
#else
static const int UDP_PACKET_SIZE = 0;
#endif

static const int MAX_PACKET_SIZE = MAX(NTP_PACKET_SIZE,UDP_PACKET_SIZE);

// buffer shared for NTP and UDP packets
#if defined(WITH_NTP) || defined(WITH_TESTPING)
static char packetBuffer[MAX_PACKET_SIZE];
#endif

// a string to hold incoming serial data (commands)
static char   input_buffer[22]= { 0 };  // longest cmd: "J=aa.bb.cc.dd.ee.ff\0" (20 characters)
static char * inputPos  = input_buffer;
static char * inputEnd  = input_buffer+sizeof(input_buffer)-1;
boolean       input_complete = false;

#ifdef WITH_TESTPING
static SOCKET pingSocket = 0;
#endif

static boolean wasReset        = false;
static boolean wasPaused       = false;
#ifdef WITH_NTP
static boolean wasNightlyReset = false;
#endif

static uint16_t errors   = 0;

static s_time time_utc = { valid : false,  updated : 0 };
#ifdef WITH_TZ      
static s_time time_loc = { valid : false,  updated : 0 };
#else
#define time_loc time_utc
#endif

static byte localAddr[4] = { 0, 0, 0, 0};

static s_config config = DEFAULT_CONFIG;

// place log in EEProm imediately after config
s_log * LOG = (s_log *)sizeof(s_config);

//unsigned long timeoutDhcp = 60000; // the default (60s)
const unsigned long timeoutDhcp =  8000; // shorter (8s)  
const unsigned long timeoutNTP  =  1000;

static DhcpClass dhcp;


static void inputClear()
{
  // clear the string:
  *(inputPos=input_buffer) = 0;
  input_complete = false;
}

static void println()
{
  Serial.print("\r\n");
}

void showPgmStringLn(PGM_P s)
{
  showPgmString(s);
  println();
}

void showPgmString (PGM_P s)
{
  char c;
  while ((c = pgm_read_byte(s++)) != 0)
  {
    Serial.print(c);
  }
}

void serialEvent()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c=='\n' || c=='\r')
    {
      *inputPos = 0; // strip CR at end for convenience
      input_complete = true;
      return;
    }
    else if (inputPos<inputEnd)
    {
      *(inputPos++) = c;
      if (inputPos<inputEnd)
      {
        *inputPos = 0;
      }
    }
  }
}

char toUpper(char c)
{
  if ('a'<=c && c<='z') c='A'+(c-'a');
  return c;
}

#ifdef WITH_HELP
static boolean showHelp()
{
  PPRINTLN(
    #ifdef WITH_SETMAC
    "H:J=mac  set own mac addr\r\n"
    #endif
    "H:I=ip   set remote IP to check\r\n"
    "H:L=ip   set local IP (disable: 0.0.0.0)\r\n"
    "H:V      run check now\r\n"
    #ifdef WITH_NTP
    "H:N=ip   remote timeserver IP\r\n"
    "H:F=1|0  do fetch time|don't\r\n"
    "H:U      fetch time now\r\n"
    #ifdef WITH_TZ    
    "H:Z=2    timezone\r\n"
    #endif    
    #endif
    "H:T=4    request timeout\r\n"
    "H:W=10   wait time between checks\r\n"
    "H:R=10   retries\r\n"
    "H:P=1|0  pause on|off\r\n"
    #ifdef WITH_TESTPING
    "H:M=1|2  ping|dhcp mode\r\n"
    #endif    
    #ifdef WITH_NTP    
    "H:S=9    office start hour\r\n"
    "H:E=19   set office end\r\n"
    "H:Y=17   same for fridays\r\n"
    #endif
    "H:C      show config\r\n"
    "H:B=4711 reboot uC\r\n"
    "H:X=4711 factory reset uC\r\n"
    "H:Q=4711 reset switch now\r\n"
    "H:H      help");
  return true;
}
#endif

static boolean parseInt(const char * str, int &i, int min, int max)
{
  if (1==sscanf(str, "%i", &i))
  {
    return (min<=i) && (i<=max);
  }
  return false;
}

static boolean parseIP(const char * str, byte addr[4])
{
  int i[4];
  if (4==sscanf(str, "%i.%i.%i.%i", i, i+1, i+2, i+3))
  {
    for (byte n=0; n<4; n++) addr[n] = (byte)(i[n]);
    return true;
  }
  return false;
}

#ifdef WITH_SETMAC
boolean parseMAC(const char * str, byte addr[6])
{
  int i[6];
  if (6==sscanf(str, "%x:%x:%x:%x:%x:%x", i, i+1, i+2, i+3, i+4, i+5))
  {
    for (byte n=0; n<6; n++) addr[n] = (byte)(i[n]);
    return true;
  }
  return false;
}
#endif

#ifdef WITH_RNDMAC
uint8_t createRandom()
{
  int analogPin = 3;
  uint8_t res = analogRead(analogPin);
  for (int i=0; i<8; i++)
  {
    res = (res<<1) | (1 & analogRead(analogPin));
  }    
  return res;
}
#endif

static void handleCommand()
{
  if (input_complete)
  {
    char          first  = toUpper(input_buffer[0]);
    char          delim  = input_buffer[1];
    const char  * arg    = &input_buffer[2];
    boolean       valid  = false;
    boolean       save   = false;
    boolean       check  = false;
    #ifdef WITH_NTP
    boolean       update = false;
    #endif
    boolean       reset  = false;
    int           i;

    if ('\r'==first || '\n'==first)
    {
      // ignore empty lines
      valid = true; 
    }
    #ifdef WITH_HELP    
    else if ('H'==first || '?'==first)
    {
      valid = showHelp();
    }
    #endif    
    else if ('C'==first)
    {
      valid = configDump();
    }
    #ifdef WITH_NTP
    else if ('U'==first)
    {
      valid = update = true;
    }
    #endif
    else if ('V'==first)
    {
      valid = check = true;
    }
    else if ('='!=delim)
    {
      valid = false;
    }    
    #ifdef WITH_TESTPING
    else if ('I'==first)
    {
      if ((valid=parseIP(arg,config.pingAddr)))
      {
        save = true;
      }
    }
    #endif    
    else if ('L'==first)
    {
      if ((valid=parseIP(arg,config.fixedIp)))
      {
        save = true;
      }
    }
    #ifdef WITH_SETMAC
    else if ('J'==first)
    {
      if ((valid=parseMAC(arg,config.mac)))
      {
        save = true;
      }
    }
    #endif    
    else if ('N'==first)
    {
      if ((valid=parseIP(arg,config.timeServer)))
      {
        save = true;
      }
    }
    else if ('F'==first)
    {
      if ((valid=parseInt(arg, i, 0, 1)))
      {
        config.fetchTime = (i==1);
      }
    }
    else if ('P'==first)
    {
      if ((valid=parseInt(arg, i, 0, 1)))
      {
        wasPaused = (i==1);
      }
    }
    else if ('T'==first)
    {
      if ((valid=parseInt(arg, i, 1, 180)))
      {
        config.timeout = i;
        save = true;
      }
    }
    else if ('W'==first)
    {
      if ((valid=parseInt(arg, i, 1, 180)))
      {
        config.waitTime = i;
        save = true;
      }
    }
#ifdef WITH_TZ    
    else if ('Z'==first)
    {
      if ((valid=parseInt(arg, i, -14, 14)))
      {
        config.timezone = i;
        save = true;
      }
    }
#endif    
    else if ('S'==first)
    {
      if ((valid=parseInt(arg, i, 0, 24)))
      {
        config.officeStart = i;
        save = true;
      }
    }
    else if ('E'==first)
    {
      if ((valid=parseInt(arg, i, 0, 24)))
      {
        config.officeEnd = i;
        save = true;
      }
    }
    else if ('Y'==first)
    {
      if ((valid=parseInt(arg, i, 0, 24)))
      {
        config.officeEndFr = i;
        save = true;
      }
    }
    else if ('R'==first)
    {
      if ((valid=parseInt(arg, i, 1, 100)))
      {
        config.retries = i;
        save = true;
      }
    }
    else if ('M'==first)
    {
      if ((valid=parseInt(arg, i, 1, 2)))
      {
        config.mode = i;
        save = true;
      }
    }
    else if ('X'==first || 'B'==first || 'Q'==first)
    {
        // argument "4711" deals as a confirmation against accidental reset/reboot
      if ((valid=parseInt(arg, i, 4711, 4711)))
      {
        if ('X'==first)
        {
          // save with invalid magic
          configSave(config.magic+1);      
          PPRINTLN("I:*RESET*\r\n");      
        }
        else if ('B'==first)
        {
          PPRINTLN("I:*REBOOT*\r\n");
        }
        
        if ('Q'==first)
        {
          reset = true;
        }
        else  
        {
          // give serial a chance to flush
          delay(500); 
          asm volatile ("jmp 0");
        }
      }
    }

    if (valid)
    {
      PPRINTLN("CMD:OK");
    }
    else
    {
      PPRINTLN("CMD:INVAL");
    }
    
    inputClear();
    
    // make sure we called inputClear, since there may be "mydelay" calls inside 
    // the following functions which will cause in infinite recursion otherwise
    if (save)
    {
      configSave(MAGIC);
    }
    else if (check)
    {
      checkConnection();
    }    
    #ifdef WITH_NTP
    else if (update)
    {
      timeUpdate(true);
    }
    #endif
    else if (reset)
    {
       resetSwitch();
    }
      
  }
}

static void eepromRead(byte * addr, byte * dest, uint16_t len)
{
  for (uint16_t i=0; i<len; i++, addr++, dest++)
  {
    *dest = eeprom_read_byte(addr);
  }
}

void eepromWrite(byte * src, byte * addr, uint16_t len)
{
  for (uint16_t i=0; i<len; i++, addr++, src++)
  {
    byte target = *src;
    byte currnt = eeprom_read_byte(addr);
    // avoid unneccessary write cycles (limited to 100k)
    if (target!=currnt)
    {
      eeprom_write_byte((unsigned char *) addr, target);
    }
  }
}

static boolean configDump()
{
  PPRINT("C:features: ");
  #ifdef WITH_TESTPING
  PPRINT(" PING");
  #endif
  #ifdef WITH_HELP
  PPRINT(" HELP");
  #endif
  #ifdef WITH_SETMAC
  PPRINT(" SETMAC");
  #endif
  #ifdef WITH_RNDMAC
  PPRINT(" RNDMAC");
  #endif
  #ifdef WITH_TZ
  PPRINT(" TZ");
  #endif
  #ifdef WITH_NTP
  PPRINT(" NTP");
  #endif
  println();
  
  PPRINT("C:localAddr: "); dumpIP(localAddr);         println();
  PPRINT("C:mac:       "); dumpMac(config.mac); println();
  PPRINT("C:mode:      "); Serial.print(config.mode);       println();;
  PPRINT("C:timeout:   "); Serial.print(config.timeout);    println();;
  PPRINT("C:waitTime:  "); Serial.print(config.waitTime);   println();;
  PPRINT("C:retries:   "); Serial.print(config.retries);    println();;
  #ifdef WITH_TESTPING
  PPRINT("C:pingAddr:  "); dumpIP(config.pingAddr);   println();
  #endif  
  #ifdef WITH_NTP
  PPRINT("C:timeServer:"); dumpIP(config.timeServer); println();
  PPRINT("C:fetchTime: "); Serial.print(config.fetchTime);  println();;
  #ifdef WITH_TZ  
  PPRINT("C:timezone:  "); Serial.print(config.timezone);   println();;
  #endif  
  PPRINT("C:office:    "); Serial.print(config.officeStart); PPRINT("-"); Serial.print(config.officeEnd); PPRINT(" (FR: "); Serial.print(config.officeEndFr); PPRINTLN(")");
  #endif
  return true;
}

static void configLoad()
{
  eepromRead(0, (byte*)&config, sizeof(config));
  if (config.magic!=MAGIC)
  {
    config = DEFAULT_CONFIG;
    #ifdef WITH_RNDMAC    
    // fill the last 3 octets of the mac with random values
    for (uint8_t i=3; i<6; i++)
    {
      for (uint8_t trial=0; trial<10; trial++)
      {
        uint8_t r = createRandom();        
        if (r>0) 
        {
          config.mac[i]=r;
          break;
        }
      }
    }
    #endif    
    eepromWrite((byte*)&config, 0, sizeof(config));
  }
  #ifndef WITH_TESTPING
  config.mode = MODE_DHCP;  
  #endif
  #ifndef WITH_NTP
  config.fetchTime = false;
  #endif
  configDump();
}

static void configSave(uint16_t magic)
{
  config.magic = magic;
  eepromWrite((byte*)&config, 0, sizeof(config));
  PPRINTLN("D:CFG SAVED");
}

static void mydelay(long ms)
{
  uint32_t end = millis()+ms;
  do 
  {
    delay(1);
    serialEvent();
    handleCommand();
  }
  while (millis()<end);
}

#ifdef WITH_NTP
// send an NTP request to the time server at the given address 
//static unsigned long sendNTPpacket(EthernetUDP& Udp, IPAddress& address)
void sendNTPpacket(EthernetUDP& Udp, byte address[4])
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE); 

  // Initialize values needed to form NTP request(see URL above for details on the packets)
  packetBuffer[ 0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[ 1] = 0;     // Stratum, or type of clock
  packetBuffer[ 2] = 6;     // Polling Interval
  packetBuffer[ 3] = 0xEC;  // Peer Clock Precision

  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49; 
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:         
  Udp.beginPacket(address, 123); // NTP requests are to port 123
  Udp.write((byte*)packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();   
  //return 0;
}
#endif

void dumpIP(byte addr[4])
{
  for (byte i=0; i<4; i++) 
  {
    Serial.print(addr[i], DEC);
    if (i<3) PPRINT("."); 
  }
}

void dumpMac(byte addr[6])
{
  for (byte i=0; i<6; i++) 
  {
    byte b = addr[i];
    if (b<=0x0f) PPRINT("0");
    Serial.print(addr[i], HEX);
    if (i<5) PPRINT(":"); 
  }
}

void printMillis(long l)
{
  Serial.print(l/1000); PPRINT("."); Serial.print(l%1000); 
}

boolean haveFixedIp()
{
  return config.fixedIp[0] && config.fixedIp[1] && config.fixedIp[2] && config.fixedIp[3];
}

void setupEther()
{
  // start Ethernet
  digitalWrite(PIN_LED, HIGH);
  IPAddress ip(0, 0, 0, 0);    

  int rc;
  PPRINTLN("S:BEGIN");
  uint32_t start, end;
  
  #ifdef WITH_DHCP
  if (!haveFixedIp())
  {
    Ethernet.begin(config.mac, ip);    
    // for some reason, the very first DHCP request fails reproducibly, thus do one
    // extra before the actual retry loop below with a very short tiomeout
    dhcp.beginWithDHCP(config.mac, 100);        
    
    do
    {
      PPRINT("S:DHCP?("); printMillis(timeoutDhcp); PPRINTLN(")");
  
      start = millis();
      rc = dhcp.beginWithDHCP(config.mac, timeoutDhcp);        
      end = millis();
      if (0==rc) 
      {
        PPRINTLN("E:DHCP");
        for (int i=0; i<20; i++)
        {
          digitalWrite(PIN_LED, i%2);
          mydelay(100);
        }
      }
      else
      {
        Ethernet.begin(config.mac, dhcp.getLocalIp());    
      }
    }
    while (0==rc);
  
    // save in global variable
    memcpy(localAddr, &(dhcp.getLocalIp()[0]), 4);
  
    // print your local IP address:
    PPRINT("S:DHCP:("); printMillis(end-start); PPRINT("):"); dumpIP(localAddr); println();
    digitalWrite(PIN_LED, LOW);    
    return;
  }
  #endif
  
  while (!haveFixedIp())
  {
    mydelay(1000);
    if (haveFixedIp())
    {
      #ifndef WITH_DHCP
      memcpy(localAddr, config.fixedIp, 4);
      start = millis();
      Ethernet.begin(config.mac, config.fixedIp);    
      end  = millis();
      PPRINT("S:ETHER:("); printMillis(end-start); PPRINT("):"); dumpIP(localAddr); println();
      #endif
    }
    else
    {
      PPRINT("E:NOFIXEDIP");
    }
  }
  digitalWrite(PIN_LED, LOW);    
}

static void dumpDayOfWeek(int dow)
{  
  switch(dow)
  {
    case 0 : PPRINT("SU"); return;
    case 1 : PPRINT("MO"); return;
    case 2 : PPRINT("TU"); return;
    case 3 : PPRINT("WE"); return;
    case 4 : PPRINT("TH"); return;
    case 5 : PPRINT("FR"); return;
    case 6 : PPRINT("SA"); return;
  }
}

void print00(uint8_t h)
{
  Serial.print(h<10 ? 0 : h/10); Serial.print(h%10);
}

void timeDump(s_time & t)
{
  if (!t.valid)
  {
    return;
  }
  uint8_t h = t.hours;
  uint8_t m = t.minutes;
  uint8_t s = t.seconds;    
  PPRINT("["); dumpDayOfWeek(t.dow); PPRINT(" "); print00(h); PPRINT(":"); print00(m); PPRINT(":"); print00(s); PPRINT("]");
}

void epochToHMS(unsigned long epoch, s_time &t)
{
    const long secsPerDay = 86400L;
    t.dow     = (epoch  / secsPerDay + 4) % 7;
    t.hours   = (epoch  % secsPerDay) / 3600;
    t.minutes = (epoch  % 3600) / 60;
    t.seconds = epoch % 60;
    t.valid   = true;
    t.epoch   = epoch;
    t.updated = millis();
}

#ifdef WITH_NTP
boolean timeUpdate(boolean force)
{
  if (!config.fetchTime) return false;
  
  uint32_t now  = millis();
  uint32_t diff = (now-time_utc.updated)/1000; // seconds
  
  boolean valid = time_utc.valid;

  // if we have already a valid time, synchronize ever hour.
  // if this is the first time call, do update
  // if not, retry every 10 seconds
  if (force || (valid && diff>(60*60)) || (!valid && 0==time_utc.updated) || (!valid && diff>10))  
  {  
    // regardless of whether the request will be successfull or not, set timestamp for the above condition:
    time_utc.updated = millis();
    
    PPRINTLN("I:NTP?");    
    uint32_t start = millis();
    EthernetUDP Udp;
    Udp.begin(localPort);
    
    sendNTPpacket(Udp, config.timeServer); // send an NTP packet to a time server
    //PPRINTLN("D:UDP SENT");
    mydelay(timeoutNTP);
   
    //PPRINTLN("D:UDP PARSE");  
    //Serial.println( Udp.parsePacket() );
    if ( Udp.parsePacket() ) 
    { 
        //PPRINTLN("D:UDP PARSED"); 
        // We've received a packet, read the data from it
        Udp.read(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer
        uint32_t end = millis();
        
        //the timestamp starts at byte 40 of the received packet and is four bytes,
        // or two words, long. First, esxtract the two words:
    
        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord  = word(packetBuffer[42], packetBuffer[43]);  
        // combine the four bytes (two words) into a long integer
        // this is NTP time (seconds since Jan 1 1900):
        unsigned long secsSince1900 = highWord << 16 | lowWord;  
        //PPRINT("Seconds since Jan 1 1900 = " );
        //Serial.println(secsSince1900);               
    
        // now convert NTP time into everyday time:
        // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
        const unsigned long seventyYears = 2208988800UL;     
        // subtract seventy years: yields unix time
        unsigned long epoch = secsSince1900 - seventyYears;  
        //PPRINT("unix timestamp: "); Serial.print(epoch); println();;
            
        // this will also set the valid flag:
        epochToHMS(epoch, time_utc);
        PPRINT("I:NTP:("); printMillis(end-start); PPRINT("):"); timeDump(time_utc); println();
    }
    else
    {
      uint32_t end = millis();
      PPRINT("E:NTP("); printMillis(end-start); PPRINTLN(")"); 
    }      
      
    // release any resources being used by this EthernetUDP instance 
    Udp.stop();
  }

#ifdef WITH_TZ      
  extrapolateTime();
#endif  
  return true;
}
#endif

#ifdef WITH_TZ
void extrapolateTime()
{
  if (!time_utc.valid) 
  {
    return;
  }

  uint32_t now  = millis();
  uint32_t last = time_utc.updated;
  uint32_t diff; // seconds
  
  // handle possible overflow of the 32bit millis counter (after approx. 49 days):
  if (last>now)
  {
    // 42949672950=2^32-1
    diff = (4294967295UL-last)+now; 
  }
  else
  {
    diff = (now-time_utc.updated);
  }
  
  // convert to seconds:
  diff/=1000;
  epochToHMS(time_utc.epoch+diff+3600*config.timezone, time_loc);    
}
#endif

static boolean checkConnection()
{
  boolean success = false;

  PPRINT("T"); timeDump(time_loc); PPRINT(":");
  uint32_t start = millis();
  
  #ifdef WITH_TESTPING  
  if (MODE_PING==config.mode)
  {
      PPRINTLN("PING?");
      ICMPPing ping(pingSocket);
      success = ping(config.timeout, config.pingAddr, packetBuffer);
      uint32_t end = millis();
      #ifdef WITH_TZ      
      extrapolateTime();
      #endif
      PPRINT("T"); timeDump(time_loc); PPRINT(":PING:("); printMillis(end-start); PPRINT("):"); Serial.print(packetBuffer); println();
  }
  #endif
  
  #ifdef WITH_TESTDHCP
  if (MODE_DHCP==config.mode)
  {
      unsigned long to = 1000UL*config.timeout;
      PPRINT("DHCP?("); printMillis(to); PPRINTLN(")");
      success = dhcp.beginWithDHCP(config.mac, to)!=0;              
      if (success)
      {        
        uint32_t end = millis();
        #ifdef WITH_TZ      
        extrapolateTime();
        #endif        
        PPRINT("T"); timeDump(time_loc); PPRINT(":DHCP:("); printMillis(end-start); PPRINT("):"); 
        
        for (byte b=0; b<4; b++) 
        {
          // print the value of each byte of the IP address:
          Serial.print(dhcp.getLocalIp()[b], DEC);
          if (b<3) PPRINT(".");
        }
        println();
      }
  }  
  #endif
  
  return success;
}

void resetSwitch()
{
  // power off for two seconds
  PPRINTLN("I:RESETTING");
  digitalWrite(PIN_RELAY, LOW);
  mydelay(3000);
  digitalWrite(PIN_RELAY, HIGH);
  wasReset = true;
}

void setup() 
{
  pinMode(PIN_LED,   OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH);

  Serial.begin(9600);
  PPRINTLN("\r\nI:SwitchControl V0.3");
  configLoad();
  setupEther();
}

boolean resetBlocked()
{
  // fetching the time was disabled -> always allow
  if (!config.fetchTime) return false;
  
  // no valid time received until now -> do not reset 
  if (!time_loc.valid) return true;
  
  // before start of office -> allow
  if (time_loc.hours<config.officeStart) return false;
  
  // distinguish different days of a week
  switch (time_loc.dow)
  {
    // always allow on weekends
    case DAY_SUN:
    case DAY_SAT:
      return false;
    case DAY_FRI:
      // block if before end of office on fridays
      return (time_loc.hours<config.officeEndFr);
  }

  // block if before end of office on normal days
  return (time_loc.hours<config.officeEnd);
}

void loop()
{
  digitalWrite(PIN_LED, HIGH);    
  
  #ifdef WITH_NTP
  timeUpdate(false); 
  #endif
  
  if (wasPaused)
  {
    mydelay(100);
    digitalWrite(PIN_LED, LOW);  
    PPRINTLN("I:PAUSED");
    mydelay(100);
    return;
  }

  boolean success = checkConnection();  
  if (!success)
  {
    errors++;
    // avoid overflow:
    if (errors>1000) errors=1000; 
    PPRINT("FAIL:"); Serial.print(errors); println();;
  }
  else
  {
    uint32_t now = millis();
    PPRINT("OK:"); printMillis(now); 
    #ifdef WITH_NTP
    if (wasNightlyReset) PPRINT(" (NIGHTLYRST)");
    #endif
    println();

    errors = 0;
    if (wasReset)
    {
      wasReset = false;
      PPRINTLN("I:CONBACK");
    }
  }

  if (errors>=config.retries)
  {
    PPRINT("W:ERRLIMIT("); Serial.print(errors); PPRINTLN(")");
    if (!wasReset)
    {
      PPRINTLN("E:CONDOWN");
      
      // if fetching the time was disabled or we are outside office hours, do reset
      if (!resetBlocked())
      {
        resetSwitch();
      }
      else
      {
        PPRINTLN("W:RSTBLCK");
      }
      
      if (config.mode!=MODE_DHCP)
      {
        setupEther();
      }
    }
    mydelay(500);
  }
  
  #ifdef WITH_NTP
  if (config.fetchTime && time_loc.valid && time_loc.hours<1 && time_loc.minutes<15)
  {
    if (!wasNightlyReset)
    {      
      PPRINTLN("I:NIGHTLYRSTNOW");
      resetSwitch();
      wasNightlyReset = true;
    }
  }
  else
  {
    wasNightlyReset = false;
  }
  #endif

  digitalWrite(PIN_LED, LOW);  
  
  if (success)
  {
    // mydelay will handle command, so config.waitTime can change.
    // this prevent from lockup when user entered too high a value for wait time
    for (uint16_t w=0; w<config.waitTime && !wasPaused;  w++)
    {
      mydelay(1000);      
    }
  }
  else
  {
    mydelay(1000);
  }
}



