/*
 Ping Example 
 This example repeatedly sends ICMP pings and sends the result over the serial port.
 Circuit: * Ethernet shield attached to pins 10, 11, 12, 13 
 */

#include "SwitchControl.h"

#include <SPI.h>         
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ICMPPing.h>
#include <avr/eeprom.h>

#define PPRINT(TEXT)   showPgmString(PSTR(TEXT))
#define PPRINTLN(TEXT) showPgmStringLn(PSTR(TEXT))

static const uint16_t MAGIC  = 4712;
static const uint8_t PIN_LED   = 13;
static const uint8_t PIN_RELAY = 12;

//static const uint8_t BUSINESS_START =  9;
//static const uint8_t BUSINESS_END   = 17;

static char      input_buffer[16]= { 0 };         // a string to hold incoming serial data
static char    * inputPos  = input_buffer;
static char    * inputEnd  = input_buffer+sizeof(input_buffer)-1;
boolean          input_complete = false;

// address for ethernet shield (coffee, coffee)
static byte mac[] = { 0xC0, 0xFF, 0xEE, 0xC0, 0xFF, 0xEE}; 

static SOCKET pingSocket = 0;

static char udpBuffer [256];

static boolean   wasReset = false;
static boolean   paused   = false;
static int       errors   = 0;

static uint8_t MODE_PING = 1;
static uint8_t MODE_DHCP = 2;

static s_time time_utc = { valid : false,  updated : 0 };
static s_time time_loc = { valid : false,  updated : 0 };

//static int timezone = +2; 

static boolean dailyReset = false;

const s_config config_default =
{
  magic          :  MAGIC,
  ping_ip        : { 10, 2, 0, 1 },
  retries        : 3,
  timeout        : 4,
  mode           : MODE_DHCP,
  timeServer     : IPAddress(10, 2, 0, 1),
  timezone       :  2,
  business_start :  9,
  business_end   : 19
  
};

s_config config = config_default;

// An EthernetUDP instance to let us send and receive packets over UDP
unsigned int localPort = 2390;      // local port to listen for UDP packets

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte      packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets 


void inputClear()
{
  // clear the string:
  *(inputPos=input_buffer) = 0;
  input_complete = false;
}

void showPgmStringLn(PGM_P s)
{
  showPgmString(s);
  PPRINT("\r\n");
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
  int a=0;
  while (Serial.available())
  {
    a=1;
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
  PPRINTLN("H: I=10.2.0.1 - set ping IP");
  PPRINTLN("H: N=10.2.0.1 - set NTP timeserver addr.");
  PPRINTLN("H: T=4        - set timouet");
  PPRINTLN("H: Z=4        - set timozone");
  PPRINTLN("H: R=10       - set retries");
  PPRINTLN("H: P=1|0      - pause on/off");
  PPRINTLN("H: M=1|2      - mode (1 ping, 2 dhcp)");
  PPRINTLN("H: C          - show config");
  PPRINTLN("H: X          - reset uC");
  PPRINTLN("H: H          - show help");
  return true;
}

static void handleCommand()
{
  if (input_complete)
  {
//    PPRINT("CMD: "); 
//    Serial.print((const char*)input_buffer); PPRINT("\r\n");

    char first = toUpper(input_buffer[0]);
    char delim = input_buffer[1];
    char * arg = &input_buffer[2];
    boolean valid = false;

//    PPRINT("first: "); Serial.print(first);  PPRINT("\r\n");
//    PPRINT("delim: "); Serial.print(delim);  PPRINT("\r\n");
//    PPRINT("arg: "); Serial.print(arg);      PPRINT("\r\n");

    if ('\r'==first || '\n'==first)
    {
      // ignore empty lines
      valid = true; 
    }
    else if ('H'==first)
    {
      valid = showHelp();
    }
    else if ('X'==first)
    {
      asm volatile ("jmp 0");
    }
    else if ('C'==first)
    {
      valid = configDump();
    }
    else if ('='!=delim)
    {
      //PPRINT("wrong delimiter\n");
      valid = false;
    }    
    else if ('I'==first)
    {
      int a,b,c,d;
      int n = sscanf(arg, "%i.%i.%i.%i", &a, &b, &c, &d);
      if ((valid=(4==n)))
      {
        config.ping_ip[0] = a;
        config.ping_ip[1] = b;
        config.ping_ip[2] = c;
        config.ping_ip[3] = d;
      }
    }
    else if ('N'==first)
    {
      int a,b,c,d;
      int n = sscanf(arg, "%i.%i.%i.%i", &a, &b, &c, &d);
      if ((valid=(4==n)))
      {
        config.timeServer = IPAddress(a,b,c,d);
      }
    }
    else if ('P'==first)
    {
      int p;
      int n = sscanf(arg, "%i", &p);
      if ((valid=(1==n)))
      {
        paused = p!=0;
      }
    }
    else if ('T'==first)
    {
      int t;
      int n = sscanf(arg, "%i", &t);
      if ((valid=(1==n)))
      {
        config.timeout = t;
      }
    }
    else if ('Z'==first)
    {
      int z;
      int n = sscanf(arg, "%i", &z);
      if ((valid=(1==n)))
      {
        config.timezone = z;
      }
    }
    else if ('R'==first)
    {
      int r;
      int n = sscanf(arg, "%i", &r);
      if ((valid=(1==n)))
      {
        config.retries = r;
      }
    }
    else if ('M'==first)
    {
      int m;
      int n = sscanf(arg, "%i", &m);
      if ((valid=(1==n)))
      {
        config.mode = m;
      }
    }

    if (valid)
    {
      configSave();
    }
    else
    {
      PPRINTLN("INVAL CMD");
    }

    inputClear();
  }
}

void eepromRead(byte * addr, byte * dest, uint16_t len)
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


boolean configDump()
{
  PPRINT("C: mac:        "); 
  for (byte i=0; i<6; i++)
  {
    Serial.print(mac[i],16); Serial.print(i==5 ? "\r\n" : ":");
  }

  PPRINT("C: ping_ip:    "); 
  Serial.print(config.ping_ip[0]); PPRINT(".");
  Serial.print(config.ping_ip[1]); PPRINT(".");
  Serial.print(config.ping_ip[2]); PPRINT(".");
  Serial.print(config.ping_ip[3]); PPRINTLN("");

  PPRINT("C: timeserver: "); 
  Serial.print(config.timeServer[0]); PPRINT(".");
  Serial.print(config.timeServer[1]); PPRINT(".");
  Serial.print(config.timeServer[2]); PPRINT(".");
  Serial.print(config.timeServer[3]); PPRINTLN("");

  PPRINT("C: timezone:   "); Serial.print(config.timezone);   PPRINTLN("");
  PPRINT("C: timeout:    "); Serial.print(config.timeout);    PPRINTLN("");
  PPRINT("C: retries:    "); Serial.print(config.retries);    PPRINTLN("");
  PPRINT("C: mode:       "); Serial.print(config.mode);       PPRINTLN("");
  PPRINT("C: business:   "); Serial.print(config.business_start); PPRINT(" - "); Serial.print(config.business_end); PPRINTLN("");
  return true;
}

void configLoad()
{
  eepromRead(0, (byte*)&config, sizeof(config));
  if (config.magic!=MAGIC)
  {
    //PPRINTLN("I: Magic invalid, performing 1st time init:");
    config = config_default;
    eepromWrite((byte*)&config, 0, sizeof(config));
  }
  else
  {
    //PPRINTLN("I: Magic valid, config loaded:");
  }
  configDump();
}


void configSave()
{
  config.magic = MAGIC;
  eepromWrite((byte*)&config, 0, sizeof(config));
  PPRINTLN("D: config saved");
}


void mydelay(long ms)
{
  long end = millis()+ms;
  do 
  {
    delay(1);
    serialEvent();
    handleCommand();
  }
  while (millis()<end);
}


// send an NTP request to the time server at the given address 
unsigned long sendNTPpacket(EthernetUDP& Udp, IPAddress& address)
{
  //Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE); 
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49; 
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  
  //Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:         
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  //Serial.println("4");
  Udp.write(packetBuffer,NTP_PACKET_SIZE);
  //Serial.println("5");
  Udp.endPacket(); 
  //Serial.println("6");
  return 0;
}


void setupEther()
{
  // start Ethernet
  digitalWrite(PIN_LED, HIGH);
  PPRINTLN("SETUP: Request IP ..");
  while (Ethernet.begin(mac) == 0) 
  {
    // no point in carrying on, so do nothing forevermore:
    PPRINTLN("E: DHCP failed");
    for (int i=0; i<20; i++)
    {
      digitalWrite(PIN_LED, i%2);
      mydelay(100);
    }
    
    PPRINTLN("SETUP: Retry DHCP..");
  }

  // print your local IP address:
  PPRINT("SETUP: IP: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) 
  {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    PPRINT("."); 
  }
  PPRINTLN("");

  digitalWrite(PIN_LED, LOW);
}

void dumpDayOfWeek(int dow)
{  
  switch(dow)
  {
    case 0 : PPRINT("Su"); return;
    case 1 : PPRINT("Mo"); return;
    case 2 : PPRINT("Tu"); return;
    case 3 : PPRINT("We"); return;
    case 4 : PPRINT("Th"); return;
    case 5 : PPRINT("Fr"); return;
    case 6 : PPRINT("Sa"); return;
  }
}

void timeDump(s_time & t)
{
    int h = t.hours;
    int m = t.minutes;
    int s = t.seconds;    
    dumpDayOfWeek(t.dow); 
    PPRINT(", ");
    Serial.print(h<10 ? 0 : h/10); Serial.print(h%10); PPRINT(":");
    Serial.print(m<10 ? 0 : m/10); Serial.print(m%10); PPRINT(":");
    Serial.print(s<10 ? 0 : s/10); Serial.print(s%10); 
}

void epochToHMS(unsigned long epoch, s_time & t)
{
    const long secsPerDay = 86400L;
    t.dow     = (epoch  / 86400L + 4) % 7;
    t.hours   = (epoch  % 86400L) / 3600;
    t.minutes = (epoch  % 3600) / 60;
    t.seconds = epoch %60;
    t.valid   = true;
    t.epoch   = epoch;
    t.updated = millis();
}


void timeUpdate()
{
  uint32_t now  = millis();
  uint32_t diff = (now-time_utc.updated)/1000; // seconds
  
  //PPRINTLN("timeUpdate: diff="); Serial.println(diff);
  
  // current time is valid and less than 60 minutes old
  if (0==time_utc.updated || diff>(60*60))  
  {  
    PPRINTLN("D: Request NTP time");
    // assume the worst case:  
    time_utc.updated = millis();
    time_utc.valid   = false;
    //PPRINTLN("D: time invalidated");
    
    EthernetUDP Udp;
    Udp.begin(localPort);
    
    //PPRINTLN("D: Getting time...");
    sendNTPpacket(Udp, config.timeServer); // send an NTP packet to a time server
    //PPRINT("D: UDP packet sent\r\n");
    mydelay(1000);
   
    //PPRINTLN("D: Parsing packet");  
    //Serial.println( Udp.parsePacket() );
    if ( Udp.parsePacket() ) 
    { 
        //PPRINTLN("D: UDP packet received"); 
        // We've received a packet, read the data from it
        Udp.read(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer
    
        //the timestamp starts at byte 40 of the received packet and is four bytes,
        // or two words, long. First, esxtract the two words:
    
        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);  
        // combine the four bytes (two words) into a long integer
        // this is NTP time (seconds since Jan 1 1900):
        unsigned long secsSince1900 = highWord << 16 | lowWord;  
        //PPRINT("Seconds since Jan 1 1900 = " );
        Serial.println(secsSince1900);               
    
        // now convert NTP time into everyday time:
        // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
        const unsigned long seventyYears = 2208988800UL;     
        // subtract seventy years: yields unix time
        unsigned long epoch = secsSince1900 - seventyYears;  
        //PPRINT("unix timestamp: "); Serial.print(epoch); PPRINTLN("");
            
        epochToHMS(epoch, time_utc);
        //PPRINT("D: time updated\r\n");        
      }
      
      /* Release any resources being used by this EthernetUDP instance */
      Udp.stop();
    }

  extrapolateTime();
  return;
}


void extrapolateTime()
{
  uint32_t now  = millis();
  uint32_t diff = (now-time_utc.updated)/1000; // seconds
  epochToHMS(time_utc.epoch+diff+3600*config.timezone, time_loc);    
}

void setup() 
{
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH);

  Serial.begin(9600);
  PPRINTLN("\r\nI: SwitchControl V0.2");

  configLoad();
  setupEther();
}


boolean checkConnection()
{
  boolean success = false;

  PPRINT("T["); timeDump(time_loc); PPRINT("]: ");
  
  if (MODE_PING==config.mode)
  {
      PPRINTLN("PING?");
      ICMPPing ping(pingSocket);
      success = ping(config.timeout, config.ping_ip, udpBuffer);
      
      extrapolateTime();
      PPRINT("T["); timeDump(time_loc); PPRINT("]: PING: "); Serial.print(udpBuffer); 
  }
  else     
  {
      PPRINTLN("DHCP?");
      success = (Ethernet.begin(mac) == 0) ? false : true;
      if (success)
      {        
        extrapolateTime();
        PPRINT("T["); timeDump(time_loc); PPRINT("]: DHCP: ");
        
        for (byte b = 0; b < 4; b++) 
        {
          // print the value of each byte of the IP address:
          Serial.print(Ethernet.localIP()[b], DEC);
          Serial.print(3==b ? "" : "."); 
        }
      }
  }
  
  PPRINTLN("");
  return success;
}


void resetSwitch()
{
  // power off for two seconds
  PPRINTLN("I: Resetting switch");
  digitalWrite(PIN_RELAY, LOW);
  mydelay(3000);
  digitalWrite(PIN_RELAY, HIGH);
  wasReset = true;
}

void loop()
{
  digitalWrite(PIN_LED, HIGH);
    
  timeUpdate(); 
  //PPRINT("TIME: UTC: "); timeDump(time_utc); PPRINT("\r\n"); 
  //PPRINT("TIME: "); timeDump(time_loc); PPRINTLN(""); 
  
  if (paused)
  {
    mydelay(100);
    digitalWrite(PIN_LED, LOW);  
    PPRINTLN("I: Paused");
    mydelay(100);
    return;
  }

  boolean success = checkConnection();
  
//  ICMPPing ping(pingSocket);  
//  boolean success = ping(timeout, config.ping_ip, buffer);
//  PPRINT("I: "); Serial.print(buffer); PPRINT("\r\n");

  if (!success)
  {
    errors++;
    if (errors>1000) errors=1000; // prevents from overflow
    PPRINT("W: failed, "); Serial.print(errors); PPRINTLN(" errors");
  }
  else
  {
    if (dailyReset)
    {
      PPRINTLN("OK: dailyRest=true") ;
    }
    else
    {
      PPRINTLN("OK:");
    }
    errors = 0;
    if (wasReset)
    {
      wasReset = false;
      PPRINTLN("I: connection up again!");
    }
  }

  if (errors>=config.retries)
  {
    PPRINT("W: too many errors ("); Serial.print(errors); PPRINTLN(")");
    if (!wasReset)
    {
      PPRINTLN("E: Connection down ...");
      
      if (time_loc.hours<config.business_start || time_loc.hours>config.business_end)
      {
        resetSwitch();
      }
      else
      {
        PPRINTLN("W: No reset (still in business hours)");
      }
      
      if (config.mode!=MODE_DHCP)
      {
        setupEther();
      }
    }
    mydelay(500);
  }
  
  if (time_loc.hours<1)
  {
    if (!dailyReset)
    {      
      PPRINTLN("I: Performing daily reset");
      resetSwitch();
      dailyReset = true;
      wasReset   = true;
    }
  }
  else
  {
    dailyReset = false;
  }

  if (success)
  {
    digitalWrite(PIN_LED, LOW);  
    mydelay(5000);
  }
  else
  {
    digitalWrite(PIN_LED, LOW);  
    mydelay(1000);
  }
}













