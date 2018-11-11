#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/file.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include "logger.h"

using namespace athrill::ros::lib;

static const char* log_level_string[LOG_LEVEL_NUM] = {
    "info",
    "warn",
    "error",
    "debug",
    "all",
};

Logger::Logger(LoggerConfigType &config) 
{
    this->config = config;
    this->message = NULL;
    this->fd = -1;
    this->lock_fd = -1;
    (void)pthread_mutex_init(&this->mutex, NULL);
    return;
}

int Logger::init(void)
{
    if (this->lock_fd != -1) {
        fprintf(stderr, "ERROR: already initialized\n");
        return -1;
    }
    if (this->fd != -1) {
        fprintf(stderr, "ERROR: already opened\n");
        return -1;
    }
    if (this->config.logfile.dest_folder == NULL) {
        fprintf(stderr, "ERROR: dest_folder is not set\n");
        return -1;
    }
    if (this->config.max_message_size_each_line <= 0) {
        fprintf(stderr, "ERROR: max_message_size_each_line is invalid(size must > 0)\n");
        return -1;
    }

    this->message = (char*)malloc(this->config.max_message_size_each_line);
    if (this->message == NULL) {
        fprintf(stderr, "ERROR: can not malloc message buffer\n");
        return -1;
    }
    this->log_message = (char*)malloc(this->config.max_message_size_each_line);
    if (this->log_message == NULL) {
        fprintf(stderr, "ERROR: can not malloc log message buffer\n");
        return -1;
    }
    snprintf(this->path, LOGGER_MAX_PATHLEN, "%s/%s_%s.log", 
        this->config.logfile.dest_folder, this->config.logfile.prefix_logname, 
        log_level_string[this->config.log_level]);

    snprintf(this->lock_file, LOGGER_MAX_PATHLEN, "%s/%s_%s.lock", 
        this->config.logfile.dest_folder, this->config.logfile.prefix_logname, 
        log_level_string[this->config.log_level]);

    /*
     * lock file open
     */
    this->lock_fd = open(this->path, O_RDWR|O_CREAT, 0644);
    if (this->lock_fd < 0) {
        fprintf(stderr, "ERROR: can not open file(%s) errno=%d\n", this->lock_file, errno);
        return -1;
    }

    return 0;
}

int Logger::log_open(void)
{
    if (this->fd >= 0) {
        this->log_close();
    }
    /*
     * log file open
     */
    this->fd = open(this->path, O_RDWR|O_CREAT|O_LARGEFILE|O_SYNC|O_APPEND, 0644);
    if (this->fd < 0) {
        fprintf(stderr, "ERROR: can not open file(%s) errno=%d\n", this->path, errno);
        return -1;
    }
    struct stat buf;
    int err = fstat(this->fd, &buf);
    if (err < 0) {
        fprintf(stderr, "ERROR: can not fstat file(%s) errno=%d\n", this->path, errno);
        return -1;
    }
    this->current_filesize = buf.st_size;
    return 0;
}

void Logger::log_lock(void)
{
    if (this->config.multi_thread_sharing == true) {
        pthread_mutex_lock(&this->mutex);
    }
    if (this->config.multi_process_sharing == true) {
        if (this->lock_fd >= 0) {
            (void)flock(this->lock_fd, LOCK_EX);
        }
    }
    return;
}

void Logger::log_unlock(void)
{
    if (this->config.multi_process_sharing == true) {
        if (this->lock_fd >= 0) {
            (void)flock(this->lock_fd, LOCK_UN);
        }
    }
    if (this->config.multi_thread_sharing == true) {
        pthread_mutex_unlock(&this->mutex);
    }
    return;
}

void Logger::log_close(void)
{
    if (this->fd >= 0) {
        close(this->fd);
        this->fd = -1;
    }
    return;
}

int Logger::log_backup(void)
{
    int backup_id = -1;
    for (int i = 0; i < this->config.logfile.backup_num; i++) {
        snprintf(this->backup_file, LOGGER_MAX_PATHLEN, "%s/%s_%s_log_%02d.bak", 
            this->config.logfile.dest_folder, this->config.logfile.prefix_logname, 
            log_level_string[this->config.log_level], i);
        struct stat buf;
        int err = stat(this->backup_file, &buf);
        if (err < 0) {
            backup_id = i;
            break;
        }
    }
    if (backup_id < 0) {
        fprintf(stderr, "ERROR: backup files exists so many(%d) that can not backup...\n", this->config.logfile.backup_num);
        (void)unlink(this->path);
        return -1;
    }
    (void)rename(this->path, this->backup_file);
    return 0;
}
void Logger::log(LogLevelType level, const char* fmt, ...)
{
    if (this->lock_fd < 0) {
        fprintf(stderr, "ERROR: not opened lock file.\n");
        return;
    }

    this->log_lock();
    {
        this->log_open();
        {
            va_list args;
            va_start(args, fmt);
            (void)vsnprintf(this->message, this->config.max_message_size_each_line, fmt, args);
            va_end(args);

            time_t t = time(NULL);
            struct timeval tm;
            (void)gettimeofday(&tm, NULL);
            char ctime_str[1024];
            (void)ctime_r(&t, ctime_str);
            for (int i = 0; i < strlen(ctime_str); i++) { 
                if (ctime_str[i] == '\n') {
                    ctime_str[i] = '\0'; 
                    break;
                }
            }

            int size = snprintf(this->log_message, this->config.max_message_size_each_line, 
                    "%s [%ld.%06ld] : %s : %s\n", ctime_str, tm.tv_sec, tm.tv_usec, log_level_string[level], this->message);
            if ((this->current_filesize + size) > (this->config.max_filesize * LOGGER_UNIT_SIZE)) {
                this->log_close();
                this->log_backup();
                this->log_open();
            }
            int err = lseek(this->fd, this->current_filesize, SEEK_SET);
            if (err == this->current_filesize) {
                err = write(this->fd, this->log_message, size);
                if (err != size) {
                    fprintf(stderr, "ERROR: can not write log data: errno=%d\n", errno);
                }
            } else {
                fprintf(stderr, "ERROR: can not write log data: errno=%d\n", errno);
            }
        }
        this->log_close();
    }
    this->log_unlock();
    return;
}

Logger::Logger(void) 
{
    return;
}

Logger::~Logger(void) 
{
    (void)pthread_mutex_destroy(&this->mutex);
    if (this->message != NULL) {
        free(this->message);
        this->message = NULL;
    }
    if (this->log_message != NULL) {
        free(this->log_message);
        this->log_message = NULL;
    }
    if (this->fd >= 0) {
        close(this->fd);
        this->fd = -1;
    }
    if (this->lock_fd >= 0) {
        close(this->lock_fd);
        this->lock_fd = -1;
    }
    return;
}