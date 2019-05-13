#ifndef _TOOL_FILE_H_
#define _TOOL_FILE_H_

#include "tool_types.h"

typedef struct {
    const char *filepath;
    int fd;
    int filesize;
    char *mmap;
} ToolFileType;

extern ToolReturnType open_file(ToolFileType *fp);
extern ToolReturnType close_file(ToolFileType *fp);

#endif /* _TOOL_FILE_H_ */