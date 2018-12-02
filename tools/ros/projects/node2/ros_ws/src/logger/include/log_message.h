#ifndef _LOG_MESSAGE_H_
#define _LOG_MESSAGE_H_

#include <stdarg.h>
#include "log_types.h"

namespace athrill {
namespace ros {
namespace lib {

typedef struct {
    int size;
    char *message; /* do not free */
} LogMessageType;

class LogMessage {
public:
    LogMessage(int buffer_size);
    ~LogMessage(void);
    int init(void);
    void get(LogLevelType level, LogMessageType &msg, const char* fmt, va_list args);
private:
    int max_message_size_each_line;
    char *message;
    char *log_message;
};

}
}
}

#endif /* _LOG_MESSAGE_H_ */