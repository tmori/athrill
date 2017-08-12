#ifndef _DBG_LOG_H_
#define _DBG_LOG_H_

#define DEBUG_EMU
#ifdef DEBUG_EMU
#include "symbol_ops.h"
#include "cpu.h"
#include <stdio.h>
#include "std_types.h"

#define DBG_BUFP_LEN		(2048U)
#define DBG_BUFP_MAX_CNT	(8192U)

typedef struct {
	int fd;
	uint32 pc;
	char *funcname;
	uint32 funcid;
	uint32 funcaddr;
	uint32 funcoff;
	bool is_view_mode;
	bool can_print;
	char *filepath;
	int count;
	struct {
		uint32 write_len;
		char p[DBG_BUFP_LEN];
	} buf[DBG_BUFP_MAX_CNT];
} DbgExecOpBufferType;

extern DbgExecOpBufferType DbgExecOpBuffer;
extern uint32 dbg_tmp_logbuflen;
extern char dbg_tmp_logbuf[DBG_BUFP_LEN];
#define DBG_EXEC_OP_BUF()		(dbg_tmp_logbuf)
#define DBG_EXEC_OP_BUF_LEN()	(DBG_BUFP_LEN)
extern void dbg_log_init(char *filepath);
extern void dbg_log_sync(void);

static inline void dbg_log_set_view_mode(bool on)
{
	DbgExecOpBuffer.is_view_mode = on;
}

static inline bool dbg_log_is_view_mode(void)
{
	return DbgExecOpBuffer.is_view_mode;
}
static inline void dbg_log_set_print_mode(bool on)
{
	DbgExecOpBuffer.can_print = on;
}

static inline bool dbg_log_can_print(void)
{
	return DbgExecOpBuffer.can_print;
}
static inline void dbg_print_log(void)
{
	int err;
	err = symbol_pc2funcid(cpu_get_current_core_pc(), &DbgExecOpBuffer.funcaddr);
	if (err >= 0) {
		DbgExecOpBuffer.funcid = err;
		DbgExecOpBuffer.funcname = symbol_funcid2funcname(DbgExecOpBuffer.funcid);
		DbgExecOpBuffer.funcoff = cpu_get_current_core_pc() - DbgExecOpBuffer.funcaddr;
		DbgExecOpBuffer.buf[DbgExecOpBuffer.count].write_len = snprintf(
				DbgExecOpBuffer.buf[DbgExecOpBuffer.count].p,
				DBG_BUFP_LEN,
				"[DONE> pc=0x%x %s(+%x) %s",
				cpu_get_current_core_pc(),
				DbgExecOpBuffer.funcname,
				DbgExecOpBuffer.funcoff,
				dbg_tmp_logbuf);
	}
	else {
		DbgExecOpBuffer.buf[DbgExecOpBuffer.count].write_len = snprintf(
				DbgExecOpBuffer.buf[DbgExecOpBuffer.count].p,
				DBG_BUFP_LEN,
				"[DONE> pc=0x%x null(null) %s", cpu_get_current_core_pc(),
				dbg_tmp_logbuf);
	}
	if (dbg_log_can_print() == TRUE) {
		printf("%s", DbgExecOpBuffer.buf[DbgExecOpBuffer.count].p);
	}
	DbgExecOpBuffer.count++;
	if (DbgExecOpBuffer.count >= DBG_BUFP_MAX_CNT) {
		dbg_log_sync();
	}
	return;
}


#define DBG_PRINT(arg)	\
do { \
	if (dbg_log_is_view_mode() == TRUE) {	\
		dbg_tmp_logbuflen = snprintf	arg;	\
		dbg_print_log();	\
	}	\
} while (0)
#define DBG_LOG_SET_VIEW(on)	dbg_log_set_view_mode(on)
#define DBG_LOG_IS_VIEW_MODE()	dbg_log_is_view_mode()
#else
#define DBG_PRINT(arg)
#define DBG_LOG_SET_VIEW(on)
#define DBG_LOG_IS_VIEW_MODE()
#endif


#endif /* _DBG_LOG_H_ */
