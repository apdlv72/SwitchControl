#ifndef SwitchControl_h
#define SwitchControl_h

#include <Arduino.h>         
#include <Ethernet.h>

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
#ifdef PROGMEM
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))
#endif


static const uint8_t MODE_PING = 1;
static const uint8_t MODE_DHCP = 2;

// configuration. everything that goes to EEProm is inside this struct:
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
  // timeout (for ping requests)
  uint16_t  timeout;
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

