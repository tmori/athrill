#include "dbg_log.h"
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "target/target_os_api.h"

DbgExecOpBufferType DbgExecOpBuffer;
char dbg_tmp_logbuf[DBG_BUFP_LEN];
uint32 dbg_tmp_logbuflen;

void dbg_log_init(char *filepath)
{
	int fd;
	fd = target_os_api_open_ctw(filepath, 0777);
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
	fd = target_os_api_open_aw(DbgExecOpBuffer.filepath);
	if (fd < 0) {
		printf("debugger_exec_op_bufsync:open err=%d\n", errno);
		fflush(stdout);
		exit(1);
	}
	DbgExecOpBuffer.fd = fd;
	DbgExecOpBuffer.count = 0;
	return;
}


