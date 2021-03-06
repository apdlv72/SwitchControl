#ifndef SwitchControl_h
#define SwitchControl_h

#include <Arduino.h>         
#include <Ethernet.h>

// no ping functionality needed
//#define WITH_TESTPING 
#define WITH_HELP
#define WITH_SETMAC
//#define WITH_RNDMAC
#define WITH_TZ
#define WITH_NTP
#define WITH_TESTDHCP
#define WITH_DHCP
// logging via http can block quite long thus diabled by default
//#define WITH_HTTPLOG
#define WITH_SYSLOG

//#define BAUD_RATE 57600
#define BAUD_RATE 115200

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
#ifdef PROGMEM
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))
#endif


#ifdef WITH_TESTPING
static const uint8_t MODE_PING = 1;
#endif
static const uint8_t MODE_DHCP = 2;

typedef struct
{
	byte     isUsed     : 1;
	byte     isNTP      : 1;
	byte     isLast     : 1;
	uint32_t timestamp;
	char     text[8];
} s_log;


typedef struct
{
	  byte isValid : 1;
	  byte isNTP   : 1;
	  unsigned long timestamp;
} s_event_time;


// Configuration. everything that goes to EEProm is inside this struct.
// Regardless whether any of the WITH_* features have been compiled in, 
// all settings will always be incldued 
typedef struct  
{
  // magic to check whether EEProm was initialized
  uint16_t  magic;
  // address for ethernet shield (coffee, coffee)
  byte      mac[6]; // = { 0xC0, 0xFF, 0xEE, 0xC0, 0xFF, 0xEE}; 
  // testing mode: MODE_PING/MODE_DHCP
  uint8_t   mode;
  // number of trials before network is assumed to be down
  uint16_t  retries;
  // timeout for ping requests (seconds)
  uint8_t   timeoutPingS;
  // timeout for DHCP requests (milliseconds)
  uint32_t  timeoutDhcpMs;
  // time between two (successful) checks
  uint16_t  waitTime;
  // the remote address to ping
  byte      pingAddr[4];
  // the timeserver to query
  byte      timeServer[4];
  // 1 if a timeserver should be queried
  boolean   fetchTime;
  // TZ: +2 = CEST
  int8_t    timezone;
  // hour when office office starts 
  uint8_t   officeStart;
  // hour when it ends
  uint8_t   officeEnd;
  // same on fridays
  uint8_t   officeEndFr; 
  // fixed IP when not using (not compiled in) DHCP 
  byte      fixedIp[4]; 
 // ip addr of syslog server 
  byte      syslogIP[4];
  // the remote address to ping
  byte      httpIP[4];
  uint16_t  httpPort;
  char      httpPath[64];
  // saves time when last reset (of external switch) occured
  s_event_time lastReset;
  // saves time when uC was rebotted last time (upon command)
  s_event_time lastReboot;
  // saves time  time of last config change
  s_event_time lastChange;
  // saves time of last startup of uC
  s_event_time lastStart;
  // total number of negative test outcomes
  uint16_t     totalFailures;
  // flag whether to reset the swicth at midnight
  boolean      doNightlyReset;
} 
s_config;


// This is meant to break with an error should the EEProm capacity be exceeded:
struct FailOnEEPromExceess { int c[E2END-sizeof(s_config)]; };

const uint8_t DAY_SUN = 0;
const uint8_t DAY_MON = 1;
const uint8_t DAY_TUE = 2;
const uint8_t DAY_WED = 3;
const uint8_t DAY_THU = 4;
const uint8_t DAY_FRI = 5;
const uint8_t DAY_SAT = 6;

typedef struct
{
  boolean  valid;
  // "system" time (millis())
  uint32_t updated; 
  // day of week, <=DAY_SAT
  uint8_t  dow;
  uint8_t  hours;
  uint8_t  minutes;
  uint8_t  seconds;  
  // unix timestamp epoche
  unsigned long epoch;
} 
s_time;

#endif


