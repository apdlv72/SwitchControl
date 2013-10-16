#ifndef SwitchControl_h
#define SwitchControl_h

#include <Arduino.h>         
#include <Ethernet.h>

typedef struct
{
  boolean  valid;
  uint32_t updated; // "system" time (millis())
  uint8_t  dow;
  uint8_t  hours;
  uint8_t  minutes;
  uint8_t  seconds;  
  unsigned long epoch;
} 
s_time;

typedef struct  
{
  uint16_t magic;
  byte     ping_ip[4];
  int      retries;
  int      timeout;
  uint8_t  mode;
  IPAddress timeServer;
  int8_t   timezone;
  int8_t   business_start;
  int8_t   business_end;
} 
s_config;

#endif

