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


Logger::Logger(LoggerConfigType &config) 
{
    this->config = config;
    this->message = NULL;
    for (int i = 0; i <= LOG_LEVEL_NUM; i++) {
        this->log_file[i] = NULL;
    }
    this->lock_file = NULL;
    (void)pthread_mutex_init(&this->mutex, NULL);
    return;
}

int Logger::init(void)
{
    if (this->lock_file != NULL) {
        fprintf(stderr, "ERROR: already initialized\n");
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

    this->message = new LogMessage(this->config.max_message_size_each_line);
    if ((this->message == NULL) || (this->message->init() < 0)) {
        fprintf(stderr, "ERROR: can not malloc message buffer\n");
        return -1;
    }
    /*
     * log file
     */
    for (int i = 0; i <= LOG_LEVEL_NUM; i++) {
        if ((i == LOG_LEVEL_NUM) && (this->config.save_as_all_level_one_file == false)) {
            this->log_file[i] = NULL;
            break;
        }
        this->log_file[i] = new LogFile(this->config.logfile);
        if (this->log_file[i] == NULL) {
            fprintf(stderr, "ERROR: can not malloc log_file(%d)\n", i);
            return -1;
        }
    }

    /*
     * lock file open
     */
    this->lock_file = new LogFile(this->config.logfile);
    if ((this->lock_file == NULL) || (this->lock_file->open_lock() < 0)) {
        fprintf(stderr, "ERROR: can not open lock file\n");
        return -1;
    }

    return 0;
}

void Logger::log_lock(void)
{
    if (this->config.multi_thread_sharing == true) {
        pthread_mutex_lock(&this->mutex);
    }
    if (this->config.multi_process_sharing == true) {
        this->lock_file->lock();
    }
    return;
}

void Logger::log_unlock(void)
{
    if (this->config.multi_process_sharing == true) {
        this->lock_file->unlock();
    }
    if (this->config.multi_thread_sharing == true) {
        pthread_mutex_unlock(&this->mutex);
    }
    return;
}

void Logger::log(LogLevelType level, const char* fmt, ...)
{
    if (this->lock_file == NULL) {
        fprintf(stderr, "ERROR: not opened lock file.\n");
        return;
    }

    this->log_lock();
    {
        this->log_file[level]->open(level);
        {
            LogMessageType msg;
            va_list args;
            va_start(args, fmt);

            this->message->get(level, msg, fmt, args);
            (void)this->log_file[level]->write(msg);
            if (this->log_file[LOG_ALL] !=NULL) {
                if (this->log_file[LOG_ALL]->open(LOG_ALL) >= 0) {
                    (void)this->log_file[LOG_ALL]->write(msg);
                }
                this->log_file[LOG_ALL]->close();
            }
            va_end(args);
        }
        this->log_file[level]->close();
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
        delete this->message;
        this->message = NULL;
    }
    for (int i = 0; i <= LOG_LEVEL_NUM; i++) {
        if (this->log_file[i] != NULL) {
            delete this->log_file[i];
            this->log_file[i] = NULL;
        }
    }
    /*
     * lock file
     */
    if (this->lock_file != NULL) {
        delete this->lock_file;
        this->lock_file = NULL;
    }
    return;
}