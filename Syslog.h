// from: https://code.google.com/p/ardusyslog/source/browse/trunk/Syslog/examples/SyslogClient/SyslogClient.pde
/*
  Syslog.h - An Arduino library for a sending Syslog messages.
  Copyright (C) 2011 Markus Heller <heller@relix.de>
  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Syslog_h
#define Syslog_h

#include <inttypes.h>

#define SYSLOG_DEFAULT_PORT 514

// log levels
#define LOG_EMERG       0   // system unusable 
#define LOG_ALERT       1   // action necessary 
#define LOG_CRIT        2   // critical  
#define LOG_ERR         3   // error  
#define LOG_WARNING     4   // warning  
#define LOG_NOTICE      5   // notice 
#define LOG_INFO        6   // info 
#define LOG_DEBUG       7   // debug message 

// facilities 
#define LOG_KERN        0  // kernel  
#define LOG_USER        1  // user level
#define LOG_MAIL        2  // mail
#define LOG_DAEMON      3  // daemons 
#define LOG_AUTH        4  // authorization
#define LOG_SYSLOG      5  // syslogd internal
#define LOG_LPR         6  // line printer 
#define LOG_NEWS        7  // network 
#define LOG_UUCP        8  // uucp
#define LOG_CRON        9  // clockd
#define LOG_AUTHPRIV   10  // sec/auth messages
#define LOG_FTP        11  // ftpd

// other codes reserved (what for?)
#define LOG_LOCAL0      (16) // reserved
#define LOG_LOCAL1      (17) 
#define LOG_LOCAL2      (18) 
#define LOG_LOCAL3      (19) 
#define LOG_LOCAL4      (20) 
#define LOG_LOCAL5      (21) 
#define LOG_LOCAL6      (22) 
#define LOG_LOCAL7      (23) 


class SyslogClass {
public:
    void setLoghost(uint8_t *);
    void setOwnHostname(int n);
    void logger(uint8_t, uint8_t, const char[], const char[]);
    void logger(uint8_t, uint8_t, const char[], String&);
private:
    uint8_t * ip_syslogserver;
    char my_own_hostname[256];
};

extern SyslogClass Syslog;

#endif
