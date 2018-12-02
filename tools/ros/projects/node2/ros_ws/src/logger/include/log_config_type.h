#ifndef _LOG_CONFIG_TYPE_H_
#define _LOG_CONFIG_TYPE_H_

#include "log_types.h"

typedef struct {
    /*
     * unit: kb
     */
    int max_filesize;
    int backup_num;
    const char *dest_folder;
    const char *prefix_logname;
} LoggerFileType;

typedef struct {
    bool save_as_all_level_one_file;
    bool multi_process_sharing;
    bool multi_thread_sharing;
    /*
     * max message size(byte) of each line 
     */
    int max_message_size_each_line;
    LoggerFileType logfile;
} LoggerConfigType;

#define LOGGER_MAX_PATHLEN  4096
#define LOGGER_UNIT_SIZE    1024

#endif /* _LOG_CONFIG_TYPE_H_ */
