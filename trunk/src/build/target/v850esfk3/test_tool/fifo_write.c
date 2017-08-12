#include "fifo.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

static void fifo_write(const char *path, char *buffer, int size)
{
	int fd;
	int err;
	struct stat buf;

	fd = open(path, O_WRONLY|O_BINARY);
	if (fd < 0) {
		printf("file open error:%s\n", path);
		exit(1);
	}
	err = fstat(fd, &buf);
	if (err < 0) {
		printf("fstat error:%s\n", path);
		exit(1);
	}

	err = lseek(fd, buf.st_size, SEEK_SET);
	if (err < 0) {
		printf("lseek error:%s\n", path);
		exit(1);
	}

	err = write(fd, buffer, size);
	if (err != size) {
		printf("write error:%s\n", path);
		exit(1);
	}
	close(fd);
	return;
}

int main(int argc, const char* argv[])
{
	int i;
	int len;
	FifoDataType data;
	char *path;
	char *endptr;
	long long ret64;

	if (argc != 4) {
		printf("Usage: %s fifo canid data\n", argv[0]);
		return 1;
	}
	path = (char*)argv[1];


	ret64 = strtoull(argv[2], &endptr, 16);
	if ((errno != 0) || (*endptr != '\0')) {
		printf("Usage: %s fifo canid data\n", argv[0]);
		return 1;
	}
	data.canid = (uint32)ret64;
	len = strlen(argv[3]);
	data.len = len;
	for (i = 0; i < len; i++) {
		data.data[i] = (uint8)argv[3][i];
	}

	printf("path=%s\n", path);
	printf("canid=0x%x\n", data.canid);
	printf("len=%u\n", data.len);
	for (i = 0; i < len; i++) {
		printf("data[%u]=%c\n", i, data.data[i]);
	}
	fifo_write(path, (char*)&data, 8 + len);

	return 0;
}
