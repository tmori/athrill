#ifndef _T_SYSLOG_H_
#define _T_SYSLOG_H_

#include <stdarg.h>
#include <stdio.h>

#define LOG_ERROR 1
#define LOG_WARNING 1
#define LOG_NOTICE 1

static inline void syslog(int flag, const char* fmt, ...)
{
	  va_list arg_ptr;
	  static char format[1024];

	  sprintf(format, "%s\n", fmt);
	  va_start(arg_ptr, fmt);
	  vprintf(format, arg_ptr);
	  va_end(arg_ptr);
	  return;
}

#endif
