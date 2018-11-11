#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/file.h>

#include "log_file.h"
#include "log_message.h"

using namespace athrill::ros::lib;

static const char* log_level_string[LOG_LEVEL_NUM + 1] = {
    "info",
    "warn",
    "error",
    "debug",
    "all",
};


LogFile::LogFile(LoggerFileType &config)
{
    this->config = config;
    return;
}

int LogFile::open_lock(void)
{
    if (this->fd >= 0) {
        this->close();
    }

    snprintf(this->path, LOGGER_MAX_PATHLEN, "%s/lock/%s.lock", 
        this->config.dest_folder, this->config.prefix_logname);

    this->fd = ::open(this->path, O_RDWR|O_CREAT|O_LARGEFILE|O_SYNC|O_APPEND, 0644);
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

int LogFile::open(LogLevelType level)
{
    if (this->fd >= 0) {
        this->close();
    }
    this->loglevel = level;

    snprintf(this->path, LOGGER_MAX_PATHLEN, "%s/%s/%s_%s.log", 
        this->config.dest_folder, log_level_string[level],
        this->config.prefix_logname, log_level_string[level]);

    this->fd = ::open(this->path, O_RDWR|O_CREAT|O_LARGEFILE|O_SYNC|O_APPEND, 0644);
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
}

void LogFile::backup(void)
{
    int backup_id = -1;
    for (int i = 0; i < this->config.backup_num; i++) {
        snprintf(this->backup_path, LOGGER_MAX_PATHLEN, "%s/%s/%s_%s_log_%02d.bak", 
            this->config.dest_folder, log_level_string[this->loglevel],
            this->config.prefix_logname, log_level_string[this->loglevel], i);
        struct stat buf;
        int err = stat(this->backup_path, &buf);
        if (err < 0) {
            backup_id = i;
            break;
        }
    }
    if (backup_id < 0) {
        fprintf(stderr, "ERROR: backup files exists so many(%d) that can not backup...\n", this->config.backup_num);
        (void)unlink(this->path);
    }
    else {
        (void)rename(this->path, this->backup_path);
    }
}
size_t LogFile::size(void)
{
    return this->current_filesize;
}

int LogFile::write(LogMessageType &msg)
{
    if ((this->size() + msg.size) > (this->config.max_filesize * LOGGER_UNIT_SIZE)) {
        this->close();
        this->backup();
        this->open(this->loglevel);
    }

    int err = lseek(this->fd, this->current_filesize, SEEK_SET);
    if (err == this->current_filesize) {
        err = ::write(this->fd, msg.message, msg.size);
        if (err != msg.size) {
            fprintf(stderr, "ERROR: can not write log data: errno=%d\n", errno);
            return -1;
        }
    } else {
        fprintf(stderr, "ERROR: can not write log data: errno=%d\n", errno);
        return -1;
    }
    return 0;
}


void LogFile::close(void)
{
    if (this->fd >= 0) {
        ::close(this->fd);
        this->fd = -1;
    }
    return;
}

LogFile::~LogFile(void)
{
    this->close();
}


void LogFile::lock(void)
{
    if (this->fd >= 0) {
        (void)flock(this->fd, LOCK_EX);
    }
    return;
}

void LogFile::unlock(void)
{
    if (this->fd >= 0) {
        (void)flock(this->fd, LOCK_UN);
    }
    return;
}