#include "fifo.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

static int fifo_read(const char *path, FifoDataType *buffer, int off)
{
	int fd;
	int err;
	struct stat buf;

	fd = open(path, O_RDONLY|O_BINARY);
	if (fd < 0) {
		printf("file open error:%s\n", path);
		return -1;
	}
	err = fstat(fd, &buf);
	if (err < 0) {
		printf("fstat error:%s\n", path);
		return -1;
	}
	if (off >= buf.st_size) {
		return -1;
	}

	err = lseek(fd, off, SEEK_SET);
	if (err < 0) {
		printf("lseek error:%s\n", path);
		return -1;
	}

	err = read(fd, buffer, sizeof(FifoDataType));
	if (err < 0) {
		printf("read error:%s\n", path);
		return -1;
	}
	close(fd);
	return 0;
}

int main(int argc, const char* argv[])
{
	int err;
	int i;
	FifoDataType data;
	char *path;
	int off = 0;

	if (argc != 2) {
		printf("Usage: %s fifo\n", argv[0]);
		return 1;
	}
	path = (char*)argv[1];


	while (1) {
		err = fifo_read(path, &data, off);
		if (err < 0) {
			return 1;
		}
		printf("**************\n");
		printf("canid=0x%x\n", data.canid);
		printf("len=%u\n", data.len);
		for (i = 0; i < data.len; i++) {
			printf("data[%u]=%x\n", i, data.data[i]);
		}
		off += (8 + data.len);
	}

	return 0;
}
