#include "dbg_log.h"
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

DbgExecOpBufferType DbgExecOpBuffer;
char dbg_tmp_logbuf[DBG_BUFP_LEN];
uint32 dbg_tmp_logbuflen;

void dbg_log_init(char *filepath)
{
	int fd;
	fd = open(filepath, O_CREAT | O_TRUNC |O_WRONLY | O_BINARY, 0777);
	if (fd < 0) {
		printf("debugger_exec_op_bufinit:open err=%d\n", errno);
		fflush(stdout);
		exit(1);
	}
	DbgExecOpBuffer.is_view_mode = FALSE;
	DbgExecOpBuffer.fd = fd;
	DbgExecOpBuffer.filepath = filepath;
	DbgExecOpBuffer.count = 0;
	return;
}

void dbg_log_sync(void)
{
	int i;
	int err;
	for (i = 0; i < DbgExecOpBuffer.count; i++) {
		err = write(DbgExecOpBuffer.fd, DbgExecOpBuffer.buf[i].p, DbgExecOpBuffer.buf[i].write_len);
		if (err <= 0) {
			printf("debugger_exec_op_bufsync:write err=%d\n", errno);
			fflush(stdout);
			exit(1);
		}
	}
	err = close(DbgExecOpBuffer.fd);
	if (err < 0) {
		printf("debugger_exec_op_bufsync:close err=%d\n", errno);
		fflush(stdout);
		exit(1);
	}
	int fd;
	fd = open(DbgExecOpBuffer.filepath, O_APPEND |O_WRONLY | O_BINARY);
	if (fd < 0) {
		printf("debugger_exec_op_bufsync:open err=%d\n", errno);
		fflush(stdout);
		exit(1);
	}
	DbgExecOpBuffer.fd = fd;
	DbgExecOpBuffer.count = 0;
	return;
}


