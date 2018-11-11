#include "log_message.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

using namespace athrill::ros::lib;

static const char* log_level_string[LOG_LEVEL_NUM] = {
    "INFO",
    "WARN",
    "ERROR",
    "DEBUG",
};

LogMessage::LogMessage(int buffer_size)
{
    this->message = NULL;
    this->log_message = NULL;
    this->max_message_size_each_line = buffer_size;
    return;
}

int LogMessage::init(void)
{
    this->message = (char*)malloc(this->max_message_size_each_line);
    if (this->message == NULL) {
        fprintf(stderr, "ERROR: can not malloc message buffer\n");
        return -1;
    }
    this->log_message = (char*)malloc(this->max_message_size_each_line);
    if (this->log_message == NULL) {
        fprintf(stderr, "ERROR: can not malloc log message buffer\n");
        return -1;
    }
    return 0;
}

void LogMessage::get(LogLevelType level, LogMessageType &msg, const char *fmt, va_list args)
{
    (void)vsnprintf(this->message, this->max_message_size_each_line, fmt, args);

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

    msg.size = snprintf(this->log_message, this->max_message_size_each_line, 
            "%s [%ld.%06ld] : %s : %s\n", ctime_str, tm.tv_sec, tm.tv_usec, log_level_string[level], this->message);
    msg.message = this->log_message;
    return;
}

LogMessage::~LogMessage(void)
{
    if (this->message != NULL) {
        free(this->message);
        this->message = NULL;
    }
    if (this->log_message != NULL) {
        free(this->log_message);
        this->log_message = NULL;
    }
    return;
}