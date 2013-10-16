/*
 Ping Example 
 This example repeatedly sends ICMP pings and sends the result over the serial port.
 Circuit: * Ethernet shield attached to pins 10, 11, 12, 13 
 */

#include <SPI.h>         
#include <Ethernet.h>
#include <ICMPPing.h>
#include <avr/eeprom.h>

static char      input_buffer[16]= { 0 };         // a string to hold incoming serial data
static char    * inputPos  = input_buffer;
static char    * inputEnd  = input_buffer+sizeof(input_buffer)-1;
boolean          input_complete = false;

// address for ethernet shield (coffee, coffee)
byte mac[] = { 0xC0, 0xFF, 0xEE, 0xC0, 0xFF, 0xEE}; 

SOCKET pingSocket = 0;

const uint8_t PIN_LED   = 13;
const uint8_t PIN_RELAY = 12;

char buffer [256];

const int timeout = 4;
boolean wasReset = false;
boolean paused = false;
int errors = 0;

const uint16_t MAGIC = 4711;

uint8_t MODE_PING = 1;
uint8_t MODE_DHCP = 2;


typedef struct  
{
  uint16_t magic;
  byte     ping_ip[4];
  int      retries;
  int      timeout;
  uint8_t  mode;
} 
s_config;


const s_config config_default =
{
  magic   :  MAGIC,
  ping_ip : { 10, 2, 0, 1 },
  retries : 4,
  timeout : 4,
  mode    : 1
};

s_config config = config_default;


void inputClear()
{
  // clear the string:
  *(inputPos=input_buffer) = 0;
  input_complete = false;
}

#define PPRINT(TEXT) showPgmString(PSTR(TEXT))


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
  PPRINT("H: Commands:\r\n");
  PPRINT("H: I=10.2.0.1 - set ping IP\r\n");
  PPRINT("H: T=4        - set timouet (s)\r\n");
  PPRINT("H: R=10       - set retries\r\n");
  PPRINT("H: P=1|0      - pause on/off\r\n");
  PPRINT("H: M=1|2      - mode (1 ping, 2 dhcp)\r\n");
  PPRINT("H: C          - show config\r\n");
  PPRINT("H: H          - show help\r\n");
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
      PPRINT("INVALID COMMAND\r\n");
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
  PPRINT("C: mac:     "); 
  for (byte i=0; i<6; i++)
  {
    Serial.print(mac[i],16); Serial.print(i==5 ? "\r\n" : ":");
  }
  PPRINT("C: ping_ip: "); 
  Serial.print(config.ping_ip[0]); PPRINT(".");
  Serial.print(config.ping_ip[1]); PPRINT(".");
  Serial.print(config.ping_ip[2]); PPRINT(".");
  Serial.print(config.ping_ip[3]); PPRINT("\r\n");
  PPRINT("C: timeout: "); Serial.print(config.timeout); PPRINT("\r\n");
  PPRINT("C: retries: "); Serial.print(config.retries); PPRINT("\r\n");
  PPRINT("C: mode:    "); Serial.print(config.mode);    PPRINT("\r\n");
  return true;
}

void configLoad()
{
  eepromRead(0, (byte*)&config, sizeof(config));
  if (config.magic!=MAGIC)
  {
    PPRINT("I: Magic invalid, performing 1st time init:\r\n");
    config = config_default;
    eepromWrite((byte*)&config, 0, sizeof(config));
  }
  else
  {
    PPRINT("I: Magic valid, config loaded:\r\n");
  }
  configDump();
}


void configSave()
{
  config.magic = MAGIC;
  eepromWrite((byte*)&config, 0, sizeof(config));
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


void setupEther()
{
  // start Ethernet
  digitalWrite(PIN_LED, HIGH);
  PPRINT("SETUP: Sending DHCP request....\r\n");
  while (Ethernet.begin(mac) == 0) 
  {
    // no point in carrying on, so do nothing forevermore:
    for(;;) 
    {
      PPRINT("E: Failed to configure Ethernet using DHCP\r\n");
      for (int i=0; i<10; i++)
      {
        digitalWrite(PIN_LED, i%2);
        mydelay(100);
      }
      
      PPRINT("SETUP: Resending DHCP request....\r\n");
    }
  }

  // print your local IP address:
  PPRINT("SETUP: IP: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) 
  {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    PPRINT("."); 
  }
  PPRINT("\r\n");

  digitalWrite(PIN_LED, LOW);
}


void setup() 
{
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH);

  Serial.begin(9600);
  PPRINT("\r\nI: Switch monitor V0.1\r\n");

  configLoad();
  setupEther();
}


boolean checkConnection()
{
  boolean success = false;
  if (MODE_PING==config.mode)
  {
      PPRINT("T: Sending PING request\r\n");
      ICMPPing ping(pingSocket);
      success = ping(timeout, config.ping_ip, buffer);
      PPRINT("T: PING: "); Serial.print(buffer); PPRINT("\r\n");
  }
  else     
  {
      PPRINT("T: Sending DHCP request\r\n");
      success = (Ethernet.begin(mac) == 0) ? false : true;
      if (success)
      {
        PPRINT("T: DHCP: "); 
        for (byte b = 0; b < 4; b++) 
        {
          // print the value of each byte of the IP address:
          Serial.print(Ethernet.localIP()[b], DEC);
          PPRINT("."); 
        }
        PPRINT("\r\n");
      }
  }
  return success;
}


void loop()
{
  digitalWrite(PIN_LED, HIGH);
  
  if (paused)
  {
    mydelay(100);
    digitalWrite(PIN_LED, LOW);  
    PPRINT("I: Paused\r\n");
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
    PPRINT("E: test failed, errors: "); Serial.print(errors); PPRINT("\r\n");
  }
  else
  {
    PPRINT("I: OK\r\n");
    errors = 0;
    if (wasReset)
    {
      wasReset = false;
      PPRINT("I: connection up again!\r\n");
    }
  }

  if (errors>=config.retries)
  {
    PPRINT("I: too many errors ("); Serial.print(errors); PPRINT(")\r\n");
    if (!wasReset)
    {
      PPRINT("I: Connection down ...\r\n");
      
      // power off for two seconds
      PPRINT("I: Resetting switch\r\n");
      digitalWrite(PIN_RELAY, LOW);
      mydelay(2000);
      digitalWrite(PIN_RELAY, HIGH);
      wasReset = true;
      
      if (config.mode!=MODE_DHCP)
      {
        setupEther();
      }
    }
    mydelay(500);
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












