#include "tool_file.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>

ToolReturnType open_file(ToolFileType *fp)
{
	struct stat statbuf;
	ToolReturnType err;

	fp->fd = open(fp->filepath, O_RDWR);
	if (fp->fd < 0) {
		return errno;
	}
	err = fstat(fp->fd, &statbuf);
	if (err < 0) {
		err = errno;
		goto errdone;
	}

	fp->filesize = statbuf.st_size;

	fp->mmap = mmap(NULL, fp->filesize, (PROT_READ|PROT_WRITE), MAP_SHARED, fp->fd, 0);
	if (fp->mmap != NULL) {
		err = errno;
		goto errdone;
	}
    return 0;

errdone:
	if (fp->fd > 0) {
		close(fp->fd);
		fp->fd = -1;
	}
	return err;
}

ToolReturnType close_file(ToolFileType *fp)
{
	if (fp->fd > 0) {
		close(fp->fd);
		fp->fd = -1;
	}
    return 0;
}
