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

#include <ICMPPing.h>
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
  pingAddr       : { 10, 2, 0, 1 },
  timeServer     : { 10, 2, 0, 1 },
  fetchTime      : true,
  timezone       :  2,
  officeStart    :  9,
  officeEnd      : 19,
  officeEndFr    : 17
};

static const int NTP_PACKET_SIZE =  48; // NTP time stamp is in the first 48 bytes of the message
static const int UDP_PACKET_SIZE = 256; // max. length of a UDP packet
static const int MAX_PACKET_SIZE = MAX(NTP_PACKET_SIZE,UDP_PACKET_SIZE);

// buffer shared for NTP and UDP packets
static char packetBuffer[MAX_PACKET_SIZE];

// a string to hold incoming serial data (commands)
static char   input_buffer[22]= { 0 };  // longest cmd: "J=aa.bb.cc.dd.ee.ff\0" (20 characters)
static char * inputPos  = input_buffer;
static char * inputEnd  = input_buffer+sizeof(input_buffer)-1;
boolean       input_complete = false;

static SOCKET pingSocket = 0;

static boolean wasReset        = false;
static boolean wasPaused       = false;
static boolean wasNightlyReset = false;

static uint16_t errors   = 0;

static s_time time_utc = { valid : false,  updated : 0 };
static s_time time_loc = { valid : false,  updated : 0 };

static byte localAddr[4] = { 0, 0, 0, 0};

static s_config config = DEFAULT_CONFIG;

//unsigned long timeoutDhcp = 60000; // the default (60s)
const unsigned long   timeoutDhcp =  8000; // shorter (8s)  

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

static boolean showHelp()
{
  PPRINTLN(
    "H:J=mac  own mac addr\r\n"
    "H:I=ip   remote IP to ping\r\n"
    "H:N=ip   remote timeserver IP\r\n"
    "H:F=1|0  do fetch time|don't\r\n"
    "H:T=4    request timeout\r\n"
    "H:W=10   wait time between checks\r\n"
    "H:Z=2    timezone\r\n"
    "H:R=10   retries\r\n"
    "H:P=1|0  pause on|off\r\n"
    "H:M=1|2  ping|dhcp mode\r\n"
    "H:S=9    office start hour\r\n"
    "H:E=19   set office end\r\n"
    "H:Y=17   same for fridays\r\n"
    "H:C      show config\r\n"
    "H:U      update time now\r\n"
    "H:B=4711 reboot uC\r\n"
    "H:X=4711 factory reset uC\r\n"
    "H:Q=4711 reset switch now\r\n"
    "H:H      help");
  return true;
}

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
  int a,b,c,d;
  if (4==sscanf(str, "%i.%i.%i.%i", &a, &b, &c, &d))
  {
    addr[0] = a;
    addr[1] = b;
    addr[2] = c;
    addr[3] = d;
    return true;
  }
  return false;
}

static boolean parseMAC(const char * str, byte addr[6])
{
  int a,b,c,d,e,f;
  if (6==sscanf(str, "%x:%x:%x:%x:%x:%x", &a, &b, &c, &d, &e, &f))
  {
    addr[0] = (byte)a;
    addr[1] = (byte)b;
    addr[2] = (byte)c;
    addr[3] = (byte)d;
    addr[4] = (byte)e;
    addr[5] = (byte)f;
    return true;
  }
  return false;
}

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

static void handleCommand()
{
  if (input_complete)
  {
    char          first  = toUpper(input_buffer[0]);
    char          delim  = input_buffer[1];
    const char  * arg    = &input_buffer[2];
    boolean       valid  = false;
    boolean       save   = false;
    boolean       update = false;
    boolean       reset  = false;
    int           i;

    if ('\r'==first || '\n'==first)
    {
      // ignore empty lines
      valid = true; 
    }
    else if ('H'==first || '?'==first)
    {
      valid = showHelp();
    }
    else if ('C'==first)
    {
      valid = configDump();
    }
    else if ('U'==first)
    {
      valid = update = true;
    }
    else if ('='!=delim)
    {
      valid = false;
    }    
    else if ('I'==first)
    {
      if ((valid=parseIP(arg,config.pingAddr)))
      {
        save = true;
      }
    }
    else if ('J'==first)
    {
      if ((valid=parseMAC(arg,config.mac)))
      {
        save = true;
      }
    }
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
    else if ('Z'==first)
    {
      if ((valid=parseInt(arg, i, -14, 14)))
      {
        config.timezone = i;
        save = true;
      }
    }
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
    else if (update)
    {
      timeUpdate(true);
    }
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
  PPRINT("C:localAddr: "); dumpIP(localAddr);         println();
  PPRINT("C:mac:       "); dumpMac(config.mac); println();
  PPRINT("C:mode:      "); Serial.print(config.mode);       println();;
  PPRINT("C:timeout:   "); Serial.print(config.timeout);    println();;
  PPRINT("C:waitTime:  "); Serial.print(config.waitTime);   println();;
  PPRINT("C:retries:   "); Serial.print(config.retries);    println();;
  PPRINT("C:pingAddr:  "); dumpIP(config.pingAddr);   println();
  PPRINT("C:timeServer:"); dumpIP(config.timeServer); println();
  PPRINT("C:fetchTime: "); Serial.print(config.fetchTime);  println();;
  PPRINT("C:timezone:  "); Serial.print(config.timezone);   println();;
  PPRINT("C:office:    "); Serial.print(config.officeStart); PPRINT("-"); Serial.print(config.officeEnd); PPRINT(" (FR: "); Serial.print(config.officeEndFr); PPRINTLN(")");
  return true;
}

static void configLoad()
{
  eepromRead(0, (byte*)&config, sizeof(config));
  if (config.magic!=MAGIC)
  {
    config = DEFAULT_CONFIG;
    
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
    
    eepromWrite((byte*)&config, 0, sizeof(config));
  }
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

// send an NTP request to the time server at the given address 
//static unsigned long sendNTPpacket(EthernetUDP& Udp, IPAddress& address)
static void sendNTPpacket(EthernetUDP& Udp, byte address[4])
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

/*
int EthernetClass::begin(uint8_t *mac_address)
{
  static DhcpClass s_dhcp;
  _dhcp = &s_dhcp;

  // Initialise the basic info
  W5100.init();
  W5100.setMACAddress(mac_address);
  W5100.setIPAddress(IPAddress(0,0,0,0).raw_address());

  // Now try to get our config info from a DHCP server
  int ret = _dhcp->beginWithDHCP(mac_address);
  if(ret == 1)
  {
    // We've successfully found a DHCP server and got our configuration info, so set things
    // accordingly
    W5100.setIPAddress(_dhcp->getLocalIp().raw_address());
    W5100.setGatewayIp(_dhcp->getGatewayIp().raw_address());
    W5100.setSubnetMask(_dhcp->getSubnetMask().raw_address());
    _dnsServerAddress = _dhcp->getDnsServerIp();
  }

  return ret;
}
*/

void printMillis(long l)
{
  Serial.print(l/1000); PPRINT("."); Serial.print(l%1000); 
}

void setupEther()
{
  // start Ethernet
  digitalWrite(PIN_LED, HIGH);
  IPAddress ip(0, 0, 0, 0);    

  int rc;
  PPRINTLN("S:BEGIN");
  Ethernet.begin(config.mac, ip);    
  // for some reason, the very first DHCP request fails reproducibly, thus do one
  // extra before the actual retry loop below with a very short tiomeout
  dhcp.beginWithDHCP(config.mac, 100);        
  
  do
  {
    PPRINT("S:DHCP?("); printMillis(timeoutDhcp); PPRINTLN(")");

    rc = dhcp.beginWithDHCP(config.mac, timeoutDhcp);        
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
  
  // savi in global variable
  memcpy(localAddr, &(dhcp.getLocalIp()[0]), 4);

  // print your local IP address:
  PPRINT("S:DHCP: ");
  dumpIP(localAddr);
  println();

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
    //PPRINT("?");
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

boolean timeUpdate(boolean force)
{
  if (!config.fetchTime) return false;
  
  uint32_t now  = millis();
  uint32_t diff = (now-time_utc.updated)/1000; // seconds
  
//  PPRINT("timeUpdate: diff="); Serial.println(diff);
//  PPRINT("timeUpdate: updated="); Serial.println(time_utc.updated);
  
  boolean valid = time_utc.valid;

  // if we have already a valid time, synchronize ever hour.
  // if this is the first time call, do update
  // if not, retry every 10 seconds
  if (force || (valid && diff>(60*60)) || (!valid && 0==time_utc.updated) || (!valid && diff>10))  
  {  
    // regardless of whether the request will be successfull or not, set timestamp for the above condition:
    time_utc.updated = millis();
    
    PPRINTLN("I:NTP?");    
    EthernetUDP Udp;
    Udp.begin(localPort);
    
    sendNTPpacket(Udp, config.timeServer); // send an NTP packet to a time server
    //PPRINTLN("D:UDP SENT");
    mydelay(1000);
   
    //PPRINTLN("D:UDP PARSE");  
    //Serial.println( Udp.parsePacket() );
    if ( Udp.parsePacket() ) 
    { 
        //PPRINTLN("D:UDP PARSED"); 
        // We've received a packet, read the data from it
        Udp.read(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer
    
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
        PPRINT("I:NTP:");        
        timeDump(time_utc);
        println();
      }
      
      // eelease any resources being used by this EthernetUDP instance 
      Udp.stop();
  }

  extrapolateTime();
  return true;
}

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

static boolean checkConnection()
{
  boolean success = false;

  PPRINT("T"); timeDump(time_loc); PPRINT(":");
  
  if (MODE_PING==config.mode)
  {
      PPRINTLN("PING?");
      ICMPPing ping(pingSocket);
      success = ping(config.timeout, config.pingAddr, packetBuffer);
      
      extrapolateTime();
      PPRINT("T"); timeDump(time_loc); PPRINT(":PING:"); Serial.print(packetBuffer); println();
  }
  else     
  {
      unsigned long to = 1000UL*config.timeout;
      PPRINT("DHCP?("); printMillis(to); PPRINTLN(")");
      success = dhcp.beginWithDHCP(config.mac, to)!=0;              
      if (success)
      {        
        extrapolateTime();
        PPRINT("T"); timeDump(time_loc); PPRINT(":DHCP:");
        
        for (byte b=0; b<4; b++) 
        {
          // print the value of each byte of the IP address:
          Serial.print(dhcp.getLocalIp()[b], DEC);
          if (b<3) PPRINT(".");
        }
        println();
      }
  }
  
  
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
  timeUpdate(false); 
  
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
    if (wasNightlyReset) PPRINT(" (NIGHTLYRST)");
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



