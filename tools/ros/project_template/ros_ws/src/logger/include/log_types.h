#ifndef _LOG_TYPES_H_
#define _LOG_TYPES_H_

#include <sys/types.h>

typedef enum {
    LOG_INFO = 0,
    LOG_WARN,
    LOG_ERROR,
    LOG_DEBUG,
    LOG_LEVEL_NUM, /* do not use */
} LogLevelType;
#define LOG_ALL LOG_LEVEL_NUM

#endif /* _LOG_TYPES_H_ */