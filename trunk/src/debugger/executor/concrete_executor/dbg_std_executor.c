#include "dbg_executor.h"
#include "file_address_mapping.h"
#include "concrete_executor/dbg_std_executor.h"
#include "front/parser/concrete_parser/dbg_std_parser.h"
#include "cpu_control/dbg_cpu_control.h"
#include "cpu_control/dbg_cpu_thread_control.h"
#include "cpuemu_ops.h"
#include "dbg_log.h"
#include "assert.h"
#include "concrete_executor/target/dbg_target_serial.h"
#include "concrete_executor/util/dbg_print_data_type.h"
#include "symbol_ops.h"
#include "cui/cui_ops.h"
#include "dbg_target_cpu.h"
#include <stdio.h>
#include <windows.h>
#include <string.h>
#define SYMBOL_CANDIATE_NUM		10


void dbg_std_executor_parse_error(void *executor)
{
	 DbgCmdExecutorType *arg = (DbgCmdExecutorType *)executor;

	 printf("ERROR: %s :command not found\n", arg->original_str);
	 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "NG\n"));

	 return;
}

void dbg_std_executor_break(void *executor)
{
	 DbgCmdExecutorType *arg = (DbgCmdExecutorType *)executor;
	 DbgCmdExecutorBreakType *parsed_args = (DbgCmdExecutorBreakType *)(arg->parsed_args);
	 uint32 func_len;
	 uint32 addr;
	 uint32 size;

	 if (parsed_args->type == DBG_CMD_BBREAK_SET_SYMBOL) {
		 func_len = strlen((char*)parsed_args->symbol.str);
		 if (symbol_get_func((char*)parsed_args->symbol.str, func_len, &addr, &size) < 0) {
			 printf("ERROR: not found symbol %s\n", parsed_args->symbol.str);
			 symbol_print_func((char*)parsed_args->symbol.str, SYMBOL_CANDIATE_NUM);
			 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "NG\n"));
		 }
		 else {
			 if (cpuctrl_set_break(addr, BREAK_POINT_TYPE_FOREVER) == TRUE) {
				 printf("break %s 0x%x\n", parsed_args->symbol.str, addr);
				 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
			 }
			 else {
				 printf("ERROR: can not break %s\n", parsed_args->symbol.str);
				 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "NG\n"));
			 }
		 }
	 }
	 else if (parsed_args->type == DBG_CMD_BBREAK_SET) {
		 if (cpuctrl_set_break(parsed_args->break_addr, BREAK_POINT_TYPE_FOREVER) == TRUE) {
			 printf("break 0x%x\n", parsed_args->break_addr);
			 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
		 }
		 else {
			 printf("ERROR: can not break 0x%x\n", parsed_args->break_addr);
			 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "NG\n"));
		 }
	 }
	 else if (parsed_args->type == DBG_CMD_BREAK_SET_FILE_LINE) {
		 Std_ReturnType err;
		 KeyAddressType value;
		 err = file_address_mapping_get_addr((char*)parsed_args->symbol.str, parsed_args->line, &value);
		 if (err != STD_E_OK) {
			 ValueFileType candidate;
			 printf("ERROR: can not found break addr:%s %u\n", (char*)parsed_args->symbol.str, parsed_args->line);
			 err = file_address_mapping_get_candidate((char*)parsed_args->symbol.str, parsed_args->line, &candidate);
			 if (err == STD_E_OK) {
				 printf("candidate : %s %u\n", candidate.file, candidate.line);
			 }
		 }
		 else {
			 if (cpuctrl_set_break(value.addr, BREAK_POINT_TYPE_FOREVER) == TRUE) {
				 printf("break 0x%x\n", value.addr);
				 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
			 }
			 else {
				 printf("ERROR: can not break 0x%x\n", value.addr);
				 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "NG\n"));
			 }
		 }
	 }
	 else if (parsed_args->type == DBG_CMD_BREAK_INFO) {
		 uint32 i;
		 uint32 addr;
		 for (i = 0; i < DBG_CPU_CONTROL_BREAK_SETSIZE; i++) {
			 int funcid;
			 uint32 funcaddr;
			 if (cpuctrl_get_break(i, &addr) == TRUE) {
				 funcid = symbol_pc2funcid(addr, &funcaddr);
				 if (funcid >= 0) {
					 printf("[%u] 0x%x %s(+0x%x)\n", i, addr, symbol_funcid2funcname(funcid), addr - funcaddr);
				 }
				 else {
					 printf("[%u] 0x%x\n", i, addr);
				 }
			 }
		 }
		 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
	 }
	 return;
}
static char* watch_type_string(DataWatchPointEumType type)
{
	if (type == DATA_WATCH_POINT_TYPE_READ) {
		return "r";
	}
	if (type == DATA_WATCH_POINT_TYPE_WRITE) {
		return "w";
	}
	if (type == DATA_WATCH_POINT_TYPE_RW) {
		return "rw";
	}
	return "none";
}
static bool dbg_std_executor_watch_data_info(DbgCmdExecutorWatchDataType *arg)
{
	 uint32 i;
	 uint32 addr;
	 uint32 size;
	 DataWatchPointEumType type;

	 for (i = 0; i < DBG_CPU_CONTROL_WATCH_DATA_SETSIZE; i++) {
		 int glid;
		 uint32 gladdr;
		 if (cpuctrl_get_data_watch_point(i, &addr, &size, &type) == TRUE) {
			 glid = symbol_addr2glid(addr, &gladdr);
			 if (glid >= 0) {
				 printf("[%u] %-4s 0x%x %u %s(+0x%x)\n", i, watch_type_string(type), addr, size, symbol_glid2glname(glid), addr - gladdr);
			 }
			 else {
				 printf("[%u] %-4s 0x%x %u\n", i, watch_type_string(type), addr, size);
			 }
		 }
	 }
	return TRUE;
}
static bool dbg_std_executor_watch_data_delete(DbgCmdExecutorWatchDataType *arg)
{
	 if (arg->type == DBG_CMD_WATCH_DELETE_ONE) {
		 if (cpuctrl_del_data_watch_point(arg->delno) == FALSE) {
			 printf("ERROR: can not delete %u\n", arg->delno);
			 return FALSE;
		 }
	 }
	 else if (arg->type == DBG_CMD_WATCH_DELETE_ALL) {
		 cpuctrl_del_all_data_watch_points();
	 }

	 return TRUE;
}
static bool dbg_std_executor_watch_data_set(DbgCmdExecutorWatchDataType *arg)
{
	uint32 addr;
	uint32 size;
	DataWatchPointEumType type;

	if (arg->type == DBG_CMD_WATCH_SET_SYMBOL) {
		/*
		 * symbol指定の場合は，アドレス，サイズ変換する
		 */
		 if (symbol_get_gl((char*)arg->symbol.str, arg->symbol.len, &addr, &size) < 0) {
			 printf("ERROR: not found symbol %s\n", arg->symbol.str);
			 symbol_print_gl((char*)arg->symbol.str, SYMBOL_CANDIATE_NUM);
			 return FALSE;
		 }
	}
	else {
		addr = arg->addr;
		size = arg->size;
	}

	if ((arg->watch_type & DBG_CMD_WATCH_TYPE_RW) == DBG_CMD_WATCH_TYPE_READ) {
		type = DATA_WATCH_POINT_TYPE_READ;
	}
	else if ((arg->watch_type & DBG_CMD_WATCH_TYPE_RW) == DBG_CMD_WATCH_TYPE_WRITE) {
		type = DATA_WATCH_POINT_TYPE_WRITE;
	}
	else if ((arg->watch_type & DBG_CMD_WATCH_TYPE_RW) == DBG_CMD_WATCH_TYPE_RW) {
		type = DATA_WATCH_POINT_TYPE_RW;
	}
	else {
		return FALSE;
	}

	if (cpuctrl_set_data_watch(type, addr, size) == TRUE) {
		 printf("set watch point 0x%x %u\n", addr, size);
	 }
	 else {
		 printf("ERROR: can not set watch point 0x%x %u\n", addr, size);
		 return FALSE;
	 }
	return TRUE;
}

void dbg_std_executor_watch_data(void *executor)
{
	bool result = FALSE;
	 DbgCmdExecutorType *arg = (DbgCmdExecutorType *)executor;
	 DbgCmdExecutorWatchDataType *parsed_args = (DbgCmdExecutorWatchDataType *)(arg->parsed_args);

	 switch (parsed_args->type) {
	 case DBG_CMD_WATCH_SET:
	 case DBG_CMD_WATCH_SET_SYMBOL:
		 result = dbg_std_executor_watch_data_set(parsed_args);
		 break;
	 case DBG_CMD_WATCH_DELETE_ALL:
	 case DBG_CMD_WATCH_DELETE_ONE:
		 result = dbg_std_executor_watch_data_delete(parsed_args);
		 break;
	 case DBG_CMD_WATCH_INFO:
		 result = dbg_std_executor_watch_data_info(parsed_args);
		 break;
	 default:
		 break;
	 }

	 if (result == TRUE) {
		 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
	 }
	 else {
		 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "NG\n"));
	 }
	 return;
}


void dbg_std_executor_delete(void *executor)
{
	 DbgCmdExecutorType *arg = (DbgCmdExecutorType *)executor;
	 DbgCmdExecutorDeleteType *parsed_args = (DbgCmdExecutorDeleteType *)(arg->parsed_args);

	 if (parsed_args->type == DBG_CMD_DELETE_ONE) {
		 if (cpuctrl_del_break(parsed_args->delete_break_no) == FALSE) {
			 printf("ERROR: can not delete %u\n", parsed_args->delete_break_no);
			 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "NG\n"));
		 }
		 else {
			 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
		 }
	 }
	 else if (parsed_args->type == DBG_CMD_DELETE_ALL) {
		 cpuctrl_del_all_break(BREAK_POINT_TYPE_FOREVER);
		 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
	 }
	 return;
}

void dbg_std_executor_cont(void *executor)
{
	DbgCmdExecutorType *arg = (DbgCmdExecutorType *)executor;
	DbgCmdExecutorContType *parsed_args = (DbgCmdExecutorContType *)(arg->parsed_args);

	cpuctrl_set_debug_mode(FALSE);
	if (parsed_args->type == DBG_CMD_CONT_ALL) {
		cpuctrl_set_cont_clocks(FALSE, 0);
		cputhr_control_dbg_wakeup_cpu();
	}
	else {
		cpuctrl_set_cont_clocks(TRUE, parsed_args->cont_clocks);
		cputhr_control_dbg_wakeup_cpu_and_wait_for_cpu_stopped();
	}
	CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
	return;
}

void dbg_std_executor_intr(void *executor)
{
	DbgCmdExecutorType *arg = (DbgCmdExecutorType *)executor;
	DbgCmdExecutorIntrType *parsed_args = (DbgCmdExecutorIntrType *)(arg->parsed_args);

	(void)cpuemu_raise_intr(parsed_args->intno);

	CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
	return;
}

void dbg_std_executor_next(void *executor)
{
	bool org_view_mode = dbg_log_is_view_mode();
	cpuctrl_set_cont_clocks(FALSE, 0);

	cpuctrl_set_debug_mode(TRUE);
	dbg_log_set_view_mode(TRUE);
	dbg_log_set_print_mode(TRUE);

	cputhr_control_dbg_wakeup_cpu_and_wait_for_cpu_stopped();

	/*
	 * ここにsleepを入れないと，cont => q コマンド実行後，ログ出力されなくなるため，
	 * 一時的に本修正対応する．
	 */
	Sleep(50);
	dbg_log_set_print_mode(FALSE);
	dbg_log_set_view_mode(org_view_mode);
	return;
}

void dbg_std_executor_return(void *executor)
{
	uint32 retaddr;
	CoreIdType core_id;

	cpuctrl_set_cont_clocks(FALSE, 0);

	if (cpuctrl_get_current_debugged_core(&core_id) == TRUE) {
		retaddr = cpuemu_get_retaddr(core_id);
		 if (cpuctrl_set_break(retaddr, BREAK_POINT_TYPE_ONLY_ONCE) == TRUE) {
			 printf("break 0x%x\n", retaddr);
		 }
		 else {
			 printf("ERROR: can not break 0x%x\n", retaddr);
		 }
		cpuctrl_set_debug_mode(FALSE);
		cputhr_control_dbg_wakeup_cpu();
	}
	else {
		ASSERT(0);
	}

	return;
}

void dbg_std_executor_quit(void *executor)
{
	cpuctrl_set_debug_mode(TRUE);
	cputhr_control_dbg_waitfor_cpu_stopped();
}

void dbg_std_executor_list(void *executor)
{
	dbg_cpu_control_update_editor();
	return;
}

void dbg_std_executor_exit(void *executor)
{
	cpuctrl_set_debug_mode(TRUE);
	cputhr_control_dbg_waitfor_cpu_stopped();
	printf("Exit\n");
	CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
	exit(1);
	return;
}

void dbg_std_executor_elaps(void *executor)
{
	CpuEmuElapsType elaps;
	cpuemu_get_elaps(&elaps);
	printf("clock = cpu %I64u intc %I64u\n", elaps.total_clocks, elaps.intr_clocks);
	CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "%I64u\n", elaps.total_clocks));
	return;
}
void dbg_std_executor_view(void *executor)
{
	bool view_mode = dbg_log_is_view_mode();

	if (view_mode == TRUE) {
		dbg_log_set_view_mode(FALSE);
		printf("VIEW_MODE=OFF\n");
		CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "VIEW_MODE=OFF\n"));
	}
	else {
		dbg_log_set_view_mode(TRUE);
		printf("VIEW_MODE=ON\n");
		CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "VIEW_MODE=ON\n"));
	}
	return;
}

static void print_memory(uint32 vaddr, uint8 *top_addr, uint32 size)
{
	uint32 i;
	printf("size=%u byte\n", size);
	for (i = 0; i < size; i++) {
		printf("%4u 0x%x 0x%x\n", i, (vaddr + i), *(top_addr + i));
	}
	CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "%x\n", *top_addr));
	return;
}
static void print_memory_type(char *variable_name, uint32 vaddr, uint8 *top_addr, uint32 size)
{
	if (variable_name != NULL) {
		bool ret = print_variable_with_data_type(variable_name, vaddr, top_addr, size);
		if (ret == TRUE) {
			return;
		}
	}

	if (size == 2) {
		uint16 *data = (uint16*)top_addr;
		printf("size=%u byte\n", size);
		printf("0x%x 0x%x\n", (vaddr), *(data));
		CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "%x\n",  *(data)));
	}
	else if (size == 4) {
		uint32 *data = (uint32*)top_addr;
		printf("size=%u byte\n", size);
		printf("0x%x 0x%x\n", (vaddr), *(data));
		CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "%x\n",  *(data)));
	}
	else {
		print_memory(vaddr, top_addr, size);
	}
	return;
}

void dbg_std_executor_print(void *executor)
{
	 DbgCmdExecutorType *arg = (DbgCmdExecutorType *)executor;
	 DbgCmdExecutorPrintType *parsed_args = (DbgCmdExecutorPrintType *)(arg->parsed_args);
	 uint8 *data;
	 uint32 gl_len;
	 uint32 addr;
	 uint32 size;

	 if (parsed_args->type == DBG_CMD_PRINT_SYMBOL) {
		 gl_len = strlen((char*)parsed_args->symbol.str);
		 if (symbol_get_gl((char*)parsed_args->symbol.str, gl_len, &addr, &size) < 0) {
			 printf("ERROR: not found symbol %s\n", parsed_args->symbol.str);
			 symbol_print_gl((char*)parsed_args->symbol.str, SYMBOL_CANDIATE_NUM);
			 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "NG\n"));
		 }
		 else {
			 cpuemu_get_addr_pointer(addr, &data);
			 print_memory_type((char*)(parsed_args->symbol.str), addr, data, size);
		 }
	 }
	 else if (parsed_args->type == DBG_CMD_PRINT_ADDR) {
		 printf("ERROR: not supported:print addr(0x%x)\n", parsed_args->addr);
		 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "NG\n"));
	 }
	 else if (parsed_args->type == DBG_CMD_PRINT_ADDR_SIZE) {
		 cpuemu_get_addr_pointer(parsed_args->addr, &data);
		 print_memory_type(NULL, parsed_args->addr, data, parsed_args->size);
	 }
	 return;
}

void dbg_std_executor_memset(void *executor)
{
	Std_ReturnType ret;
	DbgCmdExecutorType *arg = (DbgCmdExecutorType *)executor;
	DbgCmdExecutorMemsetType *parsed_args = (DbgCmdExecutorMemsetType *)(arg->parsed_args);
	uint8 *data;
	uint32 addr;
	uint8 value;

	addr = parsed_args->addr;
	value = parsed_args->value;

	ret = cpuemu_get_addr_pointer(addr, &data);
	if (ret == STD_E_OK) {
		*data = value;
		CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
	} else {
		printf("ERROR:can not find addr:0x%x\n", addr);
		CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "NG\n"));
	}
	 return;
}
void dbg_std_executor_serialin(void *executor)
{
	uint32 i;
	Std_ReturnType err;
	DbgCmdExecutorType *arg = (DbgCmdExecutorType *)executor;
	DbgCmdExecutorSerialInType *parsed_args = (DbgCmdExecutorSerialInType *)(arg->parsed_args);

	for (i = 0; i < parsed_args->input.len; i++) {
		err = dbg_serial_in(parsed_args->channel, parsed_args->input.str[i]);
		if (err != STD_E_OK) {
			printf("ERROR:can not put serial data:%c\n", parsed_args->input.str[i]);
		}
	}

	return;
}

void dbg_std_executor_info_cpu(void *executor)
{
	dbg_target_print_cpu();
	CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
	return;
}

void dbg_std_executor_func_trace(void *executor)
{
	int i;
	uint32 funcid;
	uint32 funcpcoff;
	char *funcname;
	uint32 sp;
	uint32 glid;
	uint32 gladdr;
	char *stackp;
	DbgCmdExecutorType *arg = (DbgCmdExecutorType *)executor;
	DbgCmdExecutorFuncTraceType *parsed_args = (DbgCmdExecutorFuncTraceType *)(arg->parsed_args);

	for (i = (parsed_args->bt_number - 1); i >= 0; i--) {
		funcname = cpuctrl_get_func_log_trace_info(i, &funcpcoff, &funcid, &sp);
		if (funcname == NULL) {
			break;
		}
		glid = symbol_addr2glid(sp, &gladdr);
		stackp = symbol_glid2glname(glid);
		printf("<%-30s(0x%03x)> [%3u] <0x%03x> %s\n", stackp, sp - gladdr, i, funcpcoff, funcname);
	}
	CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));

	return;
}


void dbg_std_executor_data_access_info(void *executor)
{
	DbgCmdExecutorType *arg = (DbgCmdExecutorType *)executor;
	DbgCmdExecutorDataAccessInfoType *parsed_args = (DbgCmdExecutorDataAccessInfoType *)(arg->parsed_args);
	DataAccessInfoType *table;
	DataAccessInfoType *p;
	int i;
	int num = symbol_get_func_num();

	table = cpuctrl_get_func_access_info_table((const char*)parsed_args->symbol.str);
	if (table == NULL) {
		 printf("ERROR: not found symbol %s\n", parsed_args->symbol.str);
		 symbol_print_gl((char*)parsed_args->symbol.str, SYMBOL_CANDIATE_NUM);
		 CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "NG\n"));
		return;
	}


	printf("* %s\n", parsed_args->symbol.str);
	printf(" READ\n");
	for (i = 0; i < num; i++) {
		p = &table[i];
		if (p->read_access_num > 0) {
			printf("  - %s\n", symbol_funcid2funcname(i));
		}
	}
	printf(" WRITE\n");
	for (i = 0; i < num; i++) {
		p = &table[i];
		if (p->write_access_num > 0) {
			printf("  - %s\n", symbol_funcid2funcname(i));
		}
	}

	CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));

	return;
}
static void print_stack_data(uint32 addr)
{
	 uint32 funcaddr;
	 int funcid;
	 uint32 gladdr;
	 int glid;
	 uint32 *datap;
	 uint32 data;


	 cpuemu_get_addr_pointer(addr, (uint8**)&datap);
	 data = *datap;

	 funcid = symbol_pc2funcid(data, &funcaddr);
	 if (funcid >= 0) {
		printf("					0x%x 0x%x %s(+0x%x)", addr, data, symbol_funcid2funcname(funcid), data - funcaddr);
	 }
	 else {
		glid = symbol_addr2glid(data, &gladdr);
		if (glid >= 0) {
			printf("					0x%x 0x%x %s(+0x%x)", addr, data, symbol_glid2glname(glid), data - gladdr);
		}
		else {
			printf("					0x%x 0x%x", addr, data);
		}
	 }
	printf("\n");
	CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));
	return;
}
#define DBG_CMD_BACK_TRACE_SHOW_MAX		10
void dbg_std_executor_back_trace(void *executor)
{
	int i;
	uint32 prev_sp = -1;
	uint32 sp;
	Std_ReturnType err;
	uint32 gl_num = symbol_get_gl_num();
	int inx;

	for (inx = 0; inx < gl_num; inx++) {
		i = DBG_STACK_LOG_SIZE - 1;
		if (cpuctrl_get_stack_pointer(inx, 0, &sp) == STD_E_OK) {
			prev_sp = -1;
			printf("*************************************\n");
			while (i >= 0) {
				err = cpuctrl_get_stack_pointer(inx, i, &sp);
				if (err == STD_E_OK) {
					if (prev_sp != -1) {
						uint32 addr_inx;
						uint32 addr = prev_sp;
						uint32 num = (sp - prev_sp)/4;
						if (num > DBG_CMD_BACK_TRACE_SHOW_MAX) num = DBG_CMD_BACK_TRACE_SHOW_MAX;
						for (addr_inx = 0; addr_inx < num; addr_inx++) {
							print_stack_data(addr);
							addr += 4;
						}
					}
					prev_sp = sp;
					printf(" %-30s [%4u] 0x%08x\n", symbol_glid2glname(inx), i, sp);
					}
				i--;
			}
			printf("\n");
		}
	}
	CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));


	return;
}
void dbg_std_executor_profile(void *executor)
{
	uint32 funcnum;
	uint32 funcid;
	char *funcname;
	CpuProfileType profile;

	funcnum = symbol_get_func_num();
	printf("%-50s %-15s %-15s %-15s\n", "funcname", "call_num", "func_time", "total_time");
	for (funcid = 0; funcid < funcnum; funcid++) {
		cpuctrl_profile_get(funcid, &profile);
		if (profile.call_num == 0) {
			continue;
		}
		funcname = symbol_funcid2funcname(funcid);
		printf("%-50s %-15I64u %-15I64u %-15I64u\n",
				funcname, profile.call_num,
				profile.func_time/profile.call_num, profile.total_time/profile.call_num);
	}
	CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));

	return;
}
void dbg_std_executor_help(void *executor)
{
	int i;
	int j;
	DbgCmdExecutorType *arg = (DbgCmdExecutorType *)executor;
	DbgCmdExecutorHelpType *parsed_args = (DbgCmdExecutorHelpType *)(arg->parsed_args);

	printf("LIst of commands:\n");
	for (i = 0; i < parsed_args->arg->cmd_num; i++) {
		if (parsed_args->arg->cmd[i].name_shortcut != NULL) {
			printf(" * %s(%s):\n", parsed_args->arg->cmd[i].name->str, parsed_args->arg->cmd[i].name_shortcut->str);
		}
		else {
			printf(" * %s(%s):\n", parsed_args->arg->cmd[i].name->str, "-");
		}
		for (j = 0; j < parsed_args->arg->cmd[i].opt_num; j++) {
			printf("   %u) %s\n", j+1, parsed_args->arg->cmd[i].opts[j].semantics);
			printf("      %s\n", parsed_args->arg->cmd[i].opts[j].description);
		}
	}

	CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "OK\n"));

	return;
}
