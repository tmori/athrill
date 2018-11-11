#ifndef _ATHRILL_ROS_LOGGER_H_
#define _ATHRILL_ROS_LOGGER_H_

#include <pthread.h>

namespace athrill {
namespace ros {
namespace lib {

typedef enum {
    LOG_INFO = 0,
    LOG_WARN,
    LOG_ERROR,
    LOG_DEBUG,
    LOG_ALL,
    LOG_LEVEL_NUM, /* do not use */
} LogLevelType;

typedef struct {
    int backup_num;
    const char *dest_folder;
    const char *prefix_logname;
} LoggerFileType;

typedef struct {
    /*
     * multi process sharing
     */
    bool multi_process_sharing;
    bool multi_thread_sharing;
    LogLevelType log_level;
    /*
     * unit: kb
     */
    int max_filesize;
    /*
     * max message size of each line
     */
    int max_message_size_each_line;
    LoggerFileType logfile;
} LoggerConfigType;

#define LOGGER_MAX_PATHLEN  4096
#define LOGGER_UNIT_SIZE    1024

class Logger {
public:

    Logger(LoggerConfigType &config);
    int init(void);
    void log(LogLevelType level, const char* fmt, ...);
    ~Logger(void);

private:
    pthread_mutex_t mutex;
    char *message;
    char *log_message;
    int fd;
    int lock_fd;
    size_t current_filesize;
    char path[LOGGER_MAX_PATHLEN];
    char lock_file[LOGGER_MAX_PATHLEN];
    char backup_file[LOGGER_MAX_PATHLEN];
    LoggerConfigType config;

    int log_open(void);
    void log_close(void);
    int log_backup(void);
    void log_lock(void);
    void log_unlock(void);
    Logger(void);
};

}
}
}

#endif /* _ATHRILL_ROS_LOGGER_H_ */