#ifndef _DBG_STD_PARSER_H_
#define _DBG_STD_PARSER_H_

#include "dbg_executor.h"
#include "token.h"

typedef enum {
	DBG_CMD_BBREAK_SET,
	DBG_CMD_BBREAK_SET_SYMBOL,
	DBG_CMD_BREAK_SET_FILE_LINE,
	DBG_CMD_BREAK_INFO
} DbgCmdBreakType;
typedef struct {
	DbgCmdBreakType		type;
	uint32 				break_addr;
	TokenStringType		symbol;
	uint32				line;
} DbgCmdExecutorBreakType;
extern DbgCmdExecutorType *dbg_parse_break(DbgCmdExecutorType *arg, const TokenContainerType *token_container);

typedef enum {
	DBG_CMD_DELETE_ALL,
	DBG_CMD_DELETE_ONE
} DbgCmdDeleteType;
typedef struct {
	DbgCmdDeleteType 	type;
	uint32 				delete_break_no;
} DbgCmdExecutorDeleteType;
extern DbgCmdExecutorType *dbg_parse_delete(DbgCmdExecutorType *arg, const TokenContainerType *token_container);


typedef enum {
	DBG_CMD_CONT_ALL,
	DBG_CMD_CONT_CLOCKS
} DbgCmdContType;
typedef struct {
	DbgCmdContType 		type;
	uint32 				cont_clocks;
} DbgCmdExecutorContType;
extern DbgCmdExecutorType *dbg_parse_cont(DbgCmdExecutorType *arg, const TokenContainerType *token_container);
extern DbgCmdExecutorType *dbg_parse_elaps(DbgCmdExecutorType *arg, const TokenContainerType *token_container);
extern DbgCmdExecutorType *dbg_parse_next(DbgCmdExecutorType *arg, const TokenContainerType *token_container);
extern DbgCmdExecutorType *dbg_parse_return(DbgCmdExecutorType *arg, const TokenContainerType *token_container);
extern DbgCmdExecutorType *dbg_parse_view(DbgCmdExecutorType *arg, const TokenContainerType *token_container);
typedef enum {
	DBG_CMD_PRINT_SYMBOL,
	DBG_CMD_PRINT_ADDR,
	DBG_CMD_PRINT_ADDR_SIZE,
} DbgCmdDPrintType;
typedef struct {
	DbgCmdDPrintType	type;
	TokenStringType		symbol;
	uint32 				addr;
	uint32 				size;
} DbgCmdExecutorPrintType;
extern DbgCmdExecutorType *dbg_parse_print(DbgCmdExecutorType *arg, const TokenContainerType *token_container);
typedef struct {
	uint32 				addr;
	uint8 				value;
} DbgCmdExecutorMemsetType;
extern DbgCmdExecutorType *dbg_parse_memset(DbgCmdExecutorType *arg, const TokenContainerType *token_container);
extern DbgCmdExecutorType *dbg_parse_quit(DbgCmdExecutorType *arg, const TokenContainerType *token_container);
extern DbgCmdExecutorType *dbg_parse_exit(DbgCmdExecutorType *arg, const TokenContainerType *token_container);

typedef struct {
	uint32				channel;
	TokenStringType		input;
} DbgCmdExecutorSerialInType;
extern DbgCmdExecutorType *dbg_parse_serialin(DbgCmdExecutorType *arg, const TokenContainerType *token_container);

extern DbgCmdExecutorType *dbg_parse_info_cpu(DbgCmdExecutorType *arg, const TokenContainerType *token_container);


typedef struct {
	uint32 				bt_number;
} DbgCmdExecutorFuncTraceType;
extern DbgCmdExecutorType *dbg_parse_func_trace(DbgCmdExecutorType *arg, const TokenContainerType *token_container);

extern DbgCmdExecutorType *dbg_parse_back_trace(DbgCmdExecutorType *arg, const TokenContainerType *token_container);

extern DbgCmdExecutorType *dbg_parse_profile(DbgCmdExecutorType *arg, const TokenContainerType *token_container);

typedef enum {
	DBG_CMD_WATCH_SET,
	DBG_CMD_WATCH_SET_SYMBOL,
	DBG_CMD_WATCH_DELETE_ONE,
	DBG_CMD_WATCH_DELETE_ALL,
	DBG_CMD_WATCH_INFO
} DbgCmdDWatchDataType;
#define DBG_CMD_WATCH_TYPE_READ		0x01
#define DBG_CMD_WATCH_TYPE_WRITE	0x10
#define DBG_CMD_WATCH_TYPE_RW		(DBG_CMD_WATCH_TYPE_READ|DBG_CMD_WATCH_TYPE_WRITE)
typedef struct {
	DbgCmdDWatchDataType	type;
	uint8					watch_type;
	TokenStringType			symbol;
	uint32 					addr;
	uint32 					size;
	uint32					delno;
} DbgCmdExecutorWatchDataType;
extern DbgCmdExecutorType *dbg_parse_watch_data(DbgCmdExecutorType *arg, const TokenContainerType *token_container);


typedef struct {
	TokenStringType			symbol;
} DbgCmdExecutorDataAccessInfoType;
extern DbgCmdExecutorType *dbg_parse_data_access_info(DbgCmdExecutorType *arg, const TokenContainerType *token_container);


typedef struct {
	uint32 				intno;
} DbgCmdExecutorIntrType;
extern DbgCmdExecutorType *dbg_parse_intr(DbgCmdExecutorType *arg, const TokenContainerType *token_container);

extern DbgCmdExecutorType *dbg_parse_list(DbgCmdExecutorType *arg, const TokenContainerType *token_container);


#define DBG_CMD_ARG_TYPES_MAX	3U
typedef struct {
	uint32			cmd_num;
	struct {
		const TokenStringType *name;
		const TokenStringType *name_shortcut;
		uint32			opt_num;
		struct {
			const char*		description;
			const char*		semantics;
		} opts[DBG_CMD_ARG_TYPES_MAX];
	} cmd[DBG_CMD_STD_ID_NUM];
} DbgCmdHelpType;

typedef struct {
	const DbgCmdHelpType *arg;
} DbgCmdExecutorHelpType;
extern DbgCmdExecutorType *dbg_parse_help(DbgCmdExecutorType *arg, const TokenContainerType *token_container);
#endif /* _DBG_STD_PARSER_H_ */
