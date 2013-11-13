/*
 Copyright (C) Artur Pogoda de la Vega
 Arduino sketch to send DHCP requests to a remote IP and reset a device (e.g. swicth) if that fails too often.
 An interval can be defined (office hours, e.g. 9:00 to 17:00) when there shoud be no automatic reset.
 Time is synchronized from an NTP server.
 Circuit: Ethernet shield, relay attached to pin 12, HC-05 module connected to RX,TX (configured to 56700,N,0)
 */

#include "SwitchControl.h"

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Dhcp.h>
#include "Syslog.h"


#ifdef WITH_TESTPING
#include <ICMPPing.h>
#endif
#include <avr/eeprom.h>

#define PPRINT(TEXT)   showPgmString(PSTR(TEXT))
#define PPRINTLN(TEXT) showPgmStringLn(PSTR(TEXT))
#define MAX(A,B) ((A)>(B)?(A):(B))

static const uint16_t MAGIC    = 0x1147;
static const uint8_t PIN_LED   = 13;
static const uint8_t PIN_RELAY = 12;
static const uint8_t PIN_RESET =  7;

// local port to listen for UDP packets
static const unsigned int localPort = 2390;

static const s_config DEFAULT_CONFIG =
{
magic          :   MAGIC,  // coffee, coffee. 2nd coffee will be overwritten with randoms on 1st time power on.
mac            :   { 0xC0, 0xFF, 0xEE, 0xC0, 0xFF, 0xEE },
mode           :   MODE_DHCP,
retries        :   4,
timeoutPingS   :   2, // seconds
timeoutDhcpMs  :   500, // ms
waitTime       :   10,
pingAddr       :   { 10, 2, 0, 1 },
timeServer     :   { 10, 2, 0, 1 },
fetchTime      :   true,
timezone       :   1,
officeStart    :   9,
officeEnd      :   19,
officeEndFr    :   17,
fixedIp        :   {  0, 0, 0, 0 }  ,
syslogIP       :   { 10, 2, 0,21 }  ,
httpIP         :   { 10, 2, 0,21 }  ,
httpPort       :   80,
httpPath       :   "/switch/log.php",
lastReset      :   { isValid : 0 },
lastReboot     :   { isValid : 0 },
lastChange     :   { isValid : 0 },
lastStart      :   { isValid : 0 },
totalFailures  :   0,
doNightlyReset :   false  
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
static char   input_buffer[22]= { 
  0 };  // longest cmd: "J=aa.bb.cc.dd.ee.ff\0" (20 characters)
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
static uint32_t totalFiluresSaveTime = 0;

static s_time time_utc = { 
valid : 
false,  updated : 
  0 };
#ifdef WITH_TZ
static s_time time_loc = { 
valid : 
false,  updated : 
  0 };
#else
#define time_loc time_utc
#endif

static byte localAddr[4] = { 
  0, 0, 0, 0};

static s_config config = DEFAULT_CONFIG;

// place log in EEProm imediately after config
//s_log * LOG = (s_log *)sizeof(s_config);
const int LOG_SIZE = 10;
s_log logs[LOG_SIZE];

typedef struct { 
  char c[130-sizeof(logs)]; 
} 
fail_on_excess;


void logs_add(const char * text)
{
  unsigned long timestamp = 0;
  boolean isNTP = false;

  if (time_loc.valid)
  {
    timestamp = time_loc.epoch;
    isNTP = true;
  }
  else
  {
    timestamp = millis();
  }

  int found =0;
  for (int i=0; i<LOG_SIZE; i++)
  {
    s_log &l = logs[i];
    if (!l.isUsed || l.isLast)
    {
      found = i;
      break;

    }
  }

  s_log &curr = logs[ found            ];
  s_log &next = logs[(found+1)%LOG_SIZE];

  curr.isUsed = 1;
  curr.isLast = 0;
  curr.isNTP  = isNTP;
  curr.timestamp = timestamp;
  strncpy(curr.text, text, sizeof(curr.text));

  next.isLast = 1;
}


void logs_print_one(int i, s_log &l)
{
  PPRINT("LOG: "); 
  print00(i); 
  PPRINT(" ");
  if (l.isNTP)
  {
    s_time t;
    epochToHMS(l.timestamp, t);
    timeDump(t);
  }
  else
  {
    PPRINT(" ["); 
    printMillis(l.timestamp); 
    PPRINT("] ");
  }

  for (uint16_t i=0; i<sizeof(l.text); i++)
  {
    char c = l.text[i];
    if (0==c) break;
    Serial.print(c);
  }
  PPRINT("\r\n");
}


boolean logs_show()
{
  PPRINTLN("LOG: start");
  int start = 0;
  for (int i=0; i<LOG_SIZE; i++)
  {
    s_log &l = logs[i];
    if (l.isLast)
    {
      start = i;
      break;
    }
  }

  for (int i=0; i<LOG_SIZE; i++)
  {
    s_log &l = logs[(i+start)%LOG_SIZE];
    if (l.isUsed)
    {
      logs_print_one(i+1, l);
    }
  }
  return true;
  PPRINTLN("LOG: end");
}


//unsigned long timeoutDhcp = 60000; // the default (60s)
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
    if (c==0x1b) // || c==0x01 || c==0x00 || c=='#') // handle stk500 sync attempt
    {      
      Serial.end();
      //Serial.println("RST");
      asm volatile ("jmp 0");
    }
    else if (c=='\n' || c=='\r')
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
    "H:I=ip   set remote Ip to check\r\n"
    "H:L=ip   set Local ip (disable: 0.0.0.0)\r\n"
    "H:V      run check now\r\n"
#ifdef WITH_NTP
    "H:N=ip   remote Ntp timeserver IP\r\n"
    "H:F=1|0  do Fetch time|don't\r\n"
    "H:U      fetch time now\r\n"
#ifdef WITH_TZ
    "H:Z=2    timeZone\r\n"
#endif
#endif
    "H:T=4    ping Timeout (s)\r\n"
    "H:D=4    Dhcp timeout (ms)\r\n"
    "H:W=10   Wait time between checks\r\n"
    "H:R=10   Retries\r\n"
    "H:P=1|0  Pause on|off\r\n"
#ifdef WITH_TESTPING
    "H:M=1|2  ping|dhcp Mode\r\n"
#endif
#ifdef WITH_NTP
    "H:S=9    office Start hour\r\n"
    "H:E=19   office End hour\r\n"
    "H:Y=17   same for fridaYs\r\n"
    "H:@      toggle reset @ midnight\r\n"
#endif
    "H:C      show Config\r\n"
    "H:G      show loGs\r\n"
    "H:B=4711 reBoot uC\r\n"
    "H:X=4711 factory reset uC\r\n"
    "H:Q=4711 reset switch now\r\n"
#ifdef WITH_SYSLOG
    "H:K      syslog host\r\n"
#endif
#ifdef WITH_HTTPLOG
    "H:H=ip   Http server ip for logging\r\n"
    "H:O=port http server pOrt\r\n"
    "H:A=path http server pAth\r\n"
#endif
    "H:?      help");
  return true;
}
#endif

#ifdef WITH_SYSLOG
void reportSyslog(int dhcpms, int pingms, const char * msg)
{
  if (!isIPSet(config.syslogIP))
  {
    //PPRINTLN("D:reportSyslog: no ip");
    return;
  }

  Syslog.setLoghost(config.syslogIP);
  if (msg)
  {
    PPRINT("SLOG: msg="); 
    Serial.println(msg);
    Syslog.logger(LOG_USER, LOG_NOTICE, "SwCtrl", msg);
  }

  if (dhcpms>-2)
  {    
    char buf[32]= { 
      "dhcp="     };
    itoa(dhcpms, buf+5, 10);
    PPRINT("SLOG: dhcpms="); 
    Serial.println(dhcpms, DEC);
    Syslog.logger(LOG_USER, dhcpms<0 ? LOG_ALERT : LOG_NOTICE, "SwCtrl", buf);
  }

  if (pingms>-2)
  {
    char buf[32]= { 
      "ping="     };
    itoa(pingms, buf+5, 10);
    PPRINT("SLOG: pingms="); 
    Serial.println(pingms, DEC);
    Syslog.logger(LOG_USER, pingms<0 ? LOG_ALERT : LOG_NOTICE, "SwCtrl", buf);
  }
}
#endif

#ifdef WITH_HTTPLOG
void print00(EthernetClient &cl, int i)
{
  cl.print(i/10); 
  cl.print(i%10);
}

void reportHttp(int dhcpms, int pingms, const char * msg)
{
  if (!isIPSet(config.httpIP) || config.httpPort<1)
  {
    return;
  }

  EthernetClient p;
  p.connect(config.httpIP, config.httpPort);

  p.print("GET ");   
  p.print(config.httpPath);
  p.print("?time="); 
  print00(p,time_loc.hours); 
  p.print("."); 
  print00(p,time_loc.minutes); 
  p.print("."); 
  print00(p,time_loc.seconds);
  if (dhcpms>-2)
  {
    p.print("&dhcpms="); 
    p.print(dhcpms);
  }
  if (dhcpms>-2)
  {
    p.print("&pingms="); 
    p.print(pingms);
  }
  if (msg)
  {
    p.print("&m=");
    for (const char * c=msg; *c; c++)
    {
      if (' '==*c)
      {
        p.print('+');
      }
      else if ('+'==*c)
      {
        p.print("%2b");
      }
      else if (('A'<=*c && *c<='Z') || ('a'<=*c && *c<='a') || ('0'<=*c && *c<='9'))
      {
        p.print(*c);
      }
      else
      {
        p.print('%'); 
        p.print((*c)/16); 
        p.print((*c)%16);
      }
    }
  }
  p.print(" HTTP/1.1\r\n");
  p.print("Host: localhost\r\n"); // avoid nasty "client sent HTTP/1.1 request without hostname" warning
  p.print("Connection: close\r\n");
  p.print("User-Agent: Arduino SwCtrl\r\n");
  p.print("\r\n");
  delay(50);
  p.stop();
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
    else if ('?'==first)
    {
      valid = showHelp();
    }
#endif
    else if ('C'==first)
    {
      valid = configDump();
    }
    else if ('G'==first)
    {
      valid = logs_show();
    }
    else if ('@'==first)
    {
      config.doNightlyReset = config.doNightlyReset ? false : true;
      configSave(config.magic);
      valid = true;
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
#ifdef WITH_SYSLOG
    else if ('K'==first)
    {
      if ((valid=parseIP(arg,config.syslogIP)))
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
      // 180 secs should be more than sufficient
      if ((valid=parseInt(arg, i, 1, 180))) 
      {
        config.timeoutPingS = i;
        save = true;
      }
    }
    else if ('D'==first)
    {
      // 180 secs should be more than sufficient
      if ((valid=parseInt(arg, i, 1, 60000))) 
      {
        config.timeoutDhcpMs = i;
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
    else if ('O'==first)
    {
      if ((valid=parseInt(arg, i, 1, 65535)))
      {
        config.httpPort = i;
        save = true;
      }
    }
    else if ('A'==first)
    {
      strncpy(config.httpPath, arg, sizeof(config.httpPath));
      config.httpPath[sizeof(config.httpPath)-1] = 0;
      save = true;
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
          event_save(config.lastReboot);
          configSave(MAGIC);

          // give serial a bit more a chance to flush
          //delay(500);
          doReset();
          //asm volatile ("jmp 0");
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
      event_save(config.lastChange);
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
      logs_add("RSTUSR");
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
#ifdef WITH_SYSLOG
  PPRINT(" SYSLOG");
#endif
#ifdef WITH_HTTPLOG
  PPRINT(" HTTPLOG");
#endif

  println();

  PPRINT("C:baudRate:     "); 
  Serial.println(BAUD_RATE); 
  println();
  PPRINT("C:localAddr:    "); 
  dumpIP(localAddr);         
  println();
  PPRINT("C:fixedIp:      "); 
  dumpIP(config.fixedIp);    
  println();
  PPRINT("C:mac:          "); 
  dumpMac(config.mac);       
  println();
  PPRINT("C:mode:         "); 
  Serial.print(config.mode);       
  println();
  ;
  PPRINT("C:timeoutPing:  "); 
  Serial.print(config.timeoutPingS);  
  PPRINTLN(" s");
  PPRINT("C:timeoutDHCP:  "); 
  Serial.print(config.timeoutDhcpMs); 
  PPRINTLN(" ms");
  PPRINT("C:waitTime:     "); 
  Serial.print(config.waitTime);   
  println();
  ;
  PPRINT("C:retries:      " ); 
  Serial.print(config.retries);    
  println();
  ;
#ifdef WITH_TESTPING
  PPRINT("C:pingAddr:     "); 
  dumpIP(config.pingAddr);   
  println();
#endif
#ifdef WITH_NTP
  PPRINT("C:timeServer:   "); 
  dumpIP(config.timeServer); 
  println();
  PPRINT("C:fetchTime:    "); 
  Serial.print(config.fetchTime ? "yes" : "no");  
  println();
  ;
#ifdef WITH_TZ
  PPRINT("C:timezone:     "); 
  Serial.print(config.timezone);   
  println();
  ;
#endif
  PPRINT("C:office:       "); 
  Serial.print(config.officeStart); 
  PPRINT("-"); 
  Serial.print(config.officeEnd); 
  PPRINT(" (FR: "); 
  Serial.print(config.officeEndFr); 
  PPRINTLN(")");
#endif
#ifdef WITH_NTP
  PPRINT("C:nightlyReset: "); Serial.println(config.doNightlyReset); 
#endif
#ifdef WITH_SYSLOG
  PPRINT("C:syslogIp:     "); 
  dumpIP(config.syslogIP); 
  println();
#endif
#ifdef WITH_HTTPLOG
  PPRINT("C:httpLog:      http://"); 
  dumpIP(config.httpIP); 
  PPRINT(":"); 
  Serial.print(config.httpPort); 
  Serial.println(config.httpPath);
#endif

  PPRINT("C:lastReset:    "); 
  dumpEventTime(config.lastReset);    
  PPRINT("\r\n");
  PPRINT("C:lastReboot:   "); 
  dumpEventTime(config.lastReboot);   
  PPRINT("\r\n");
  PPRINT("C:lastChange:   "); 
  dumpEventTime(config.lastChange);   
  PPRINT("\r\n");
  PPRINT("C:lastStart:    "); 
  dumpEventTime(config.lastStart);    
  PPRINT("\r\n");
  PPRINT("C:totalFailures:"); 
  Serial.print(config.totalFailures); 
  PPRINT("\r\n");

  return true;
}


void dumpEventTime(s_event_time &et)
{
  if (et.isValid)
  {
    if (et.isNTP)
    {
      s_time t;
      epochToHMS(et.timestamp, t);
      timeDump(t);
    }
    else
    {
      PPRINT("["); 
      printMillis(et.timestamp); 
      PPRINT("]");
    }
  }
  else
  {
    PPRINT("never");
  }
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
  //configDump();
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
  Serial.print(l/1000); 
  PPRINT("."); 
  Serial.print(l%1000);
}

boolean isIPSet(byte ip[4])
{
  return ip[0] || ip[1]>0 || ip[2]>0 || ip[3]>0;
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
  if (!isIPSet(config.fixedIp))
  {
    Ethernet.begin(config.mac, ip);
    // for some reason, the very first DHCP request fails reproducibly, thus do one
    // extra before the actual retry loop below with a very short tiomeout
    dhcp.beginWithDHCP(config.mac, 100);

    do
    {
      PPRINT("S:DHCP?("); 
      printMillis(config.timeoutDhcpMs); 
      PPRINTLN(")");

      start = millis();
      rc = dhcp.beginWithDHCP(config.mac, config.timeoutDhcpMs);
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
    unsigned long ms = end-start;
    PPRINT("S:DHCP:("); 
    printMillis(ms); 
    PPRINT("):"); 
    dumpIP(localAddr); 
    println();

#ifdef WITH_SYSLOG
    reportSyslog(ms, -2, "setup_dhcp_complete");
#endif

#ifdef WITH_HTTPLOG
    reportHttp(ms, -2, "setup_dhcp_complete");
#endif

    digitalWrite(PIN_LED, LOW);
    return;
  }
#endif

  while (!isIPSet(config.fixedIp))
  {
    mydelay(1000);
    if (isIPSet(config.fixedIp))
    {
#ifndef WITH_DHCP
      memcpy(localAddr, config.fixedIp, 4);
      start = millis();
      Ethernet.begin(config.mac, config.fixedIp);
      end  = millis();
      unsigned long = end-start;
      PPRINT("S:ETHER:("); 
      printMillis(ms); 
      PPRINT("):"); 
      dumpIP(localAddr); 
      println();

#ifdef WITH_SYSLOG
      reportSyslog(-2, ms, "setup_fix_complete");
#endif

#ifdef WITH_HTTPLOG
      reportHttp(-2, ms, "setup_fix_complete");
#endif

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
  case 0 : 
    PPRINT("SU"); 
    return;
  case 1 : 
    PPRINT("MO"); 
    return;
  case 2 : 
    PPRINT("TU"); 
    return;
  case 3 : 
    PPRINT("WE"); 
    return;
  case 4 : 
    PPRINT("TH"); 
    return;
  case 5 : 
    PPRINT("FR"); 
    return;
  case 6 : 
    PPRINT("SA"); 
    return;
  }
}

void print00(uint8_t h)
{
  Serial.print(h<10 ? 0 : h/10); 
  Serial.print(h%10);
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
  PPRINT("["); 
  dumpDayOfWeek(t.dow); 
  PPRINT(" "); 
  print00(h); 
  PPRINT(":"); 
  print00(m); 
  PPRINT(":"); 
  print00(s); 
  PPRINT("]");
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
boolean wasNTPreceivedOnce = false;

boolean timeUpdate(boolean force)
{
  if (!config.fetchTime) return false;

  uint32_t now  = millis();
  uint32_t diff = (now-time_utc.updated)/1000; // seconds

  boolean valid = time_utc.valid;
  boolean gotTimeNow = false;
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
      PPRINT("I:NTP:("); 
      printMillis(end-start); 
      PPRINT("):"); 
      timeDump(time_utc); 
      println();
      gotTimeNow = true;
    }
    else
    {
      uint32_t end = millis();
      PPRINT("E:NTP("); 
      printMillis(end-start); 
      PPRINTLN(")");
    }

    // release any resources being used by this EthernetUDP instance
    Udp.stop();
  }

#ifdef WITH_TZ
  extrapolateTime();
#endif

  if (!wasNTPreceivedOnce && gotTimeNow)
  {
    wasNTPreceivedOnce = true;
    event_save(config.lastStart);
    configSave(MAGIC);          
  }


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

  PPRINT("T"); 
  timeDump(time_loc); 
  PPRINT(":");
  uint32_t start = millis();

#ifdef WITH_TESTPING
  if (MODE_PING==config.mode)
  {
    PPRINTLN("PING?");
    ICMPPing ping(pingSocket);
    success = ping(config.timeoutPingS, config.pingAddr, packetBuffer);
    uint32_t end = millis();
#ifdef WITH_TZ
    extrapolateTime();
#endif
    unsigned long ms = end-start;
    PPRINT("T"); 
    timeDump(time_loc); 
    PPRINT(":PING:("); 
    printMillis(ms); 
    PPRINT("):"); 
    Serial.print(packetBuffer); 
    println();

#ifdef WITH_HTTPLOG
    reportHttp(-2, ms, packetBuffer);
#endif

#ifdef WITH_SYSLOG
    reportSyslog(-2, ms, NULL);
#endif
  }
#endif

#ifdef WITH_TESTDHCP
  if (MODE_DHCP==config.mode)
  {
    PPRINT("DHCP?("); 
    printMillis(config.timeoutDhcpMs); 
    PPRINTLN(")");
    success = dhcp.beginWithDHCP(config.mac, config.timeoutDhcpMs)!=0;
    if (success)
    {
      uint32_t end = millis();
#ifdef WITH_TZ
      extrapolateTime();
#endif

      unsigned long ms = end-start;
      PPRINT("T"); 
      timeDump(time_loc); 
      PPRINT(":DHCP:("); 
      printMillis(ms); 
      PPRINT("):");

      for (byte b=0; b<4; b++)
      {
        // print the value of each byte of the IP address:
        Serial.print(dhcp.getLocalIp()[b], DEC);
        if (b<3) PPRINT(".");
      }
      println();

#ifdef WITH_HTTPLOG
      reportHttp(ms, -2, NULL);
#endif

#ifdef WITH_SYSLOG
      reportSyslog(ms, -2, NULL);
#endif
    }
  }
#endif

  return success;
}


void event_save(s_event_time &et)
{
  et.isValid = true;
  if (time_loc.valid)
  {
    et.isNTP = true;
    et.timestamp = time_loc.epoch;
  }
  else
  {
    et.isNTP = false;
    et.timestamp = millis();
  }
}

void resetSwitch()
{
  // power off for two seconds
  PPRINTLN("I:RESETTING");
  digitalWrite(PIN_RELAY, LOW);

  // use the necessary delay to save info to eeprom
  event_save(config.lastReset);
  configSave(MAGIC);

  mydelay(3000);
  digitalWrite(PIN_RELAY, HIGH);
  wasReset = true;
}

//stk500_private.h
#define Cmnd_STK_GET_SYNC          0x30  // '0'      
#define Sync_CRC_EOP               0x20  // 'SPACE'
#define Resp_STK_INSYNC            0x14  // ' '


void doReset()
{
  digitalWrite(PIN_RESET, LOW);     
  delay(100);
  for (;;);
}  

void checkForSTK500()
{
  char last=0, curr=0;

  for (uint32_t i=0; i<180*1000L; i++)
  {
    digitalWrite(PIN_LED, 1==((i/200)%8));    

    if (Serial.available())
    {
      last = curr;
      curr = (char)Serial.read();      
      // wiring.c for arduino mega sends:
      // [1b] . [01] . [00] . [01] . [0e] . [01] . [14] 
      if (0x1b==curr || 0x01==curr || 0x00==curr || 0x01==curr || 0x0e==curr || 0x14==curr || '#'==curr) 
      { 
        digitalWrite(PIN_LED, LOW);
        delay(3700); 
        doReset();
        Serial.print("@@@@@@");
      }
    }
    else
    {
      delayMicroseconds(20);   
    }
  }  
}

void setup()
{
  digitalWrite(PIN_RESET, HIGH);
  pinMode(PIN_RESET, OUTPUT);     


  Serial.begin(BAUD_RATE);
  pinMode(PIN_LED,OUTPUT);

  checkForSTK500();  
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH);

  PPRINTLN("\r\nI:SwitchControl V0.5");

  logs_add("SETUPSTRT");
  configLoad();
  setupEther();
  logs_add("SETUPCMPL");
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

//void loop()
//{
//  digitalWrite(PIN_LED, LOW);
//  mydelay(100);
//  digitalWrite(PIN_LED, HIGH);
//  mydelay(100);
//}

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
    uint32_t now = millis();
    PPRINT("I:PAUSED:"); printMillis(now); println();
    mydelay(100);
    return;
  }

  boolean success = checkConnection();
  if (!success)
  {
    errors++;
    config.totalFailures++;

    const long sixHours = 6L*3600*1000;
    uint32_t now = millis();
    if (totalFiluresSaveTime+sixHours<now)
    {
      configSave(MAGIC);
      totalFiluresSaveTime = now;
    }

    // avoid overflow:
    if (errors>1000) errors=1000;
    PPRINT("FAIL:"); 
    Serial.print(errors); 
    PPRINT(",TF=");  
    Serial.print(config.totalFailures);
    println();
    ;
  }
  else
  {
    uint32_t now = millis();
    PPRINT("OK:");  printMillis(now); 
    PPRINT(",TF="); Serial.print(config.totalFailures);
    #ifdef WITH_NTP
    if (wasNightlyReset) 
    {
      PPRINT(" (NIGHTLYRST)");
    }
    #endif
    println();

    errors = 0;
    if (wasReset)
    {
      wasReset = false;
      PPRINTLN("I:CONBACK");
      logs_add("CONBACK");

    }
  }

  if (errors>=config.retries)
  {
    PPRINT("W:ERRLIMIT("); 
    Serial.print(errors); 
    PPRINTLN(")");
    if (!wasReset)
    {
      PPRINTLN("E:CONDOWN");

      // if fetching the time was disabled or we are outside office hours, do reset
      if (!resetBlocked())
      {
        logs_add("RSTERR");
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
  if (config.doNightlyReset && config.fetchTime && time_loc.valid && time_loc.hours<1 && time_loc.minutes<15)
  {
    if (!wasNightlyReset)
    {
      PPRINTLN("I:NIGHTLYRSTNOW");
      logs_add("RSTSCHD");
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





