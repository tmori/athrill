#ifndef _ATHRILL_ROS_LOGGER_H_
#define _ATHRILL_ROS_LOGGER_H_

#include <pthread.h>
#include "log_types.h"
#include "log_config_type.h"
#include "log_message.h"
#include "log_file.h"

using namespace athrill::ros::lib;

namespace athrill {
namespace ros {
namespace lib {

class Logger {
public:
    Logger(LoggerConfigType &config);
    ~Logger(void);
    int init(void);
    void log(LogLevelType level, const char* fmt, ...);

private:
    LoggerConfigType config;
    pthread_mutex_t mutex;
    LogMessage *message;
    LogFile *log_file[LOG_LEVEL_NUM + 1];
    LogFile *lock_file;

    int log_backup(void);
    void log_lock(void);
    void log_unlock(void);
    Logger(void);
};

}
}
}

#endif /* _ATHRILL_ROS_LOGGER_H_ */