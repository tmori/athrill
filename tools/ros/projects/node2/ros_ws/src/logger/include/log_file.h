#ifndef _LOG_FILE_H_
#define _LOG_FILE_H_

#include "log_config_type.h"
#include "log_message.h"

using namespace athrill::ros::lib;

namespace athrill {
namespace ros {
namespace lib {

class LogFile {
public:
    LogFile(LoggerFileType &config);
    ~LogFile(void);

    int open_lock(void);
    int open(LogLevelType level);
    int write(LogMessageType &msg);
    void close(void);
    size_t size(void);

    void lock(void);
    void unlock(void);

private:
    LoggerFileType config;
    size_t current_filesize;
    char path[LOGGER_MAX_PATHLEN];
    char backup_path[LOGGER_MAX_PATHLEN];
    int fd;
    LogLevelType loglevel;
    void backup(void);
};

}
}
}

#endif /* _LOG_FILE_H_ */