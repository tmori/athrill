#include "concrete_executor/target/dbg_target_serial.h"
#include <stdio.h>

#define DBG_SERIAL_CHANNEL_NUM	4U
#define DBG_SERIAL_BUFFER_SIZE	1024U

typedef struct {
	uint32 count;
	uint32 write_off;
	uint32 read_off;
	uint8  buffer[DBG_SERIAL_BUFFER_SIZE];
} DbgSerialFifoType;

DbgSerialFifoType dbg_serial_fifo[DBG_SERIAL_CHANNEL_NUM];

Std_ReturnType dbg_serial_in(uint8 channel, uint8 data)
{
	DbgSerialFifoType *fifo;
	if (channel >= DBG_SERIAL_CHANNEL_NUM) {
		return STD_E_INVALID;
	}

	fifo = &dbg_serial_fifo[channel];
	if (fifo->count >= DBG_SERIAL_BUFFER_SIZE) {
		return STD_E_LIMIT;
	}
	fifo->buffer[fifo->write_off] = data;

	fifo->count++;
	fifo->write_off++;
	if (fifo->write_off >= DBG_SERIAL_BUFFER_SIZE) {
		fifo->write_off = 0U;
	}
	return STD_E_OK;
}


bool dbg_serial_getchar(uint8 channel, uint8 *data)
{
	DbgSerialFifoType *fifo;
	if (channel >= DBG_SERIAL_CHANNEL_NUM) {
		return FALSE;
	}

	fifo = &dbg_serial_fifo[channel];
	if (fifo->count == 0U) {
		return FALSE;
	}
	*data = fifo->buffer[fifo->read_off];

	fifo->count--;
	fifo->read_off++;
	if (fifo->read_off >= DBG_SERIAL_BUFFER_SIZE) {
		fifo->read_off = 0U;
	}
	return TRUE;
}

bool dbg_serial_putchar(uint8 channel, uint8 data)
{
	printf("%c", data);
	fflush(stdout);
	return TRUE;
}

/*
 * file
 */

typedef struct {
	int isset;
	char path[4096];
	int fd;
} SerialFileType;

typedef struct {
	SerialFileType 	file;
	int				read_off;
} SerialFileReaderType;

typedef struct {
	SerialFileType 	file;
	int				write_off;
} SerialFileWriterType;

typedef struct {
	SerialFileReaderType in;
	SerialFileWriterType out;
} SerialDeviceFileType;
static SerialDeviceFileType SerialDeviceFile = {
		.in.file.isset = -1,
		.in.file.fd = -1,
		.out.file.isset = -1,
		.out.file.fd = -1,
};

static Std_ReturnType file_write(SerialFileWriterType *wfile, char c);
static Std_ReturnType file_read(SerialFileReaderType *rfile, char *c);

bool dbg_serial_getchar_file(uint8 channel, uint8 *data)
{
	char c;
	if (file_read(&SerialDeviceFile.in, &c) != STD_E_OK) {
		return FALSE;
	}
	else {
		*data = c;
	}
	//printf("getchar=0x%x\n", c);
	//fflush(stdout);
	return TRUE;
}
bool dbg_serial_putchar_file(uint8 channel, uint8 data)
{
	char c = data;
	if (file_write(&SerialDeviceFile.out, c) != STD_E_OK) {
		return FALSE;
	}
	return TRUE;
}
/*
 * static
 */
#include "cpuemu_ops.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#define SERIAL_IN_FILENAME		"serial_out.txt"
#define SERIAL_OUT_FILENAME		"serial_in.txt"


static void file_pathset(SerialFileType *file, const char* filename, int filenamelen)
{
	Std_ReturnType ret;
	char *path;
	int pathlen = 0;
	if (file->isset >= 0) {
		return;
	}

	ret = cpuemu_get_devcfg_string("SERIAL_FILE_PATH", &path);
	if (ret != STD_E_OK) {
		printf("WARNING:can not find SERIAL_FILE_PATH on device_file\n");
	}

	pathlen = strlen(path);
	file->isset = 1;
	memset(file->path, 0, 4096);
	if (pathlen > 0) {
		memcpy(file->path, path, pathlen);
		file->path[pathlen] = '/';
		memcpy(&file->path[pathlen + 1], filename, filenamelen);
		if (file->path[0] == '/') {
			file->path[0] = file->path[1];
			file->path[1] = ':';
		}
	}
	else {
		memcpy(&file->path, filename, filenamelen);
	}
	return;
}

static void file_wopen(SerialFileWriterType *wfile)
{
	int err;
	struct stat buf;

	if (wfile->file.fd >= 0) {
		return;
	}
	file_pathset(&wfile->file, SERIAL_OUT_FILENAME, strlen(SERIAL_OUT_FILENAME));
	wfile->file.fd = open(wfile->file.path, O_WRONLY|O_BINARY);
	if (wfile->file.fd < 0) {
		printf("file open error:%s\n", wfile->file.path);
		exit(1);
	}
	err = fstat(wfile->file.fd, &buf);
	if (err < 0) {
		printf("fstat error:%s\n", wfile->file.path);
		exit(1);
	}
	wfile->write_off = buf.st_size;

	return;
}
static void file_ropen(SerialFileReaderType *rfile)
{
	int err;
	struct stat buf;

	if (rfile->file.fd >= 0) {
		return;
	}
	file_pathset(&rfile->file, SERIAL_IN_FILENAME, strlen(SERIAL_IN_FILENAME));
	rfile->file.fd = open(rfile->file.path, O_RDONLY|O_BINARY);
	if (rfile->file.fd < 0) {
		printf("file open error:%s errno=%d\n", rfile->file.path, errno);
		exit(1);
	}
	err = fstat(rfile->file.fd, &buf);
	if (err < 0) {
		printf("fstat error:%s\n", rfile->file.path);
		exit(1);
	}
	rfile->read_off = buf.st_size;

	return;
}

static Std_ReturnType file_read(SerialFileReaderType *rfile, char *c)
{
	int err;
	struct stat buf;

	file_ropen(rfile);

	err = fstat(rfile->file.fd, &buf);
	if (err < 0) {
		printf("fstat error:%s\n", rfile->file.path);
		exit(1);
	}

	if (rfile->read_off >= buf.st_size) {
		return STD_E_NOENT;
	}

	err = lseek(rfile->file.fd, rfile->read_off, SEEK_SET);
	if (err < 0) {
		printf("lseek error:%s\n", rfile->file.path);
		exit(1);
	}
	err = read(rfile->file.fd, c, 1);
	if (err < 0) {
		printf("read error:%s\n", rfile->file.path);
		exit(1);
	}
	rfile->read_off++;

	return STD_E_OK;
}

static Std_ReturnType file_write(SerialFileWriterType *wfile, char c)
{
	int err;
	struct stat buf;

	file_wopen(wfile);

	err = fstat(wfile->file.fd, &buf);
	if (err < 0) {
		printf("fstat error:%s\n", wfile->file.path);
		exit(1);
	}
	err = lseek(wfile->file.fd, wfile->write_off, SEEK_SET);
	if (err < 0) {
		printf("lseek error:%s\n", wfile->file.path);
		exit(1);
	}

	err = write(wfile->file.fd, &c, 1);
	if (err != 1) {
		printf("write error:%s\n", wfile->file.path);
		exit(1);
	}
	wfile->write_off++;

	err = close(wfile->file.fd);
	if (err < 0) {
		printf("lseek error:%s\n", wfile->file.path);
		exit(1);
	}
	wfile->file.fd = -1;
	return STD_E_OK;
}
