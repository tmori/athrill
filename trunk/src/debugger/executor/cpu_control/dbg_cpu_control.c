#include "cpu_control/dbg_cpu_control.h"
#include "cpu_config.h"
#include "symbol_ops.h"
#include "assert.h"
#include "file.h"
#include "cpuemu_ops.h"
#include "std_errno.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
	bool 				is_set;
	BreakPointEumType	type;
	uint32 				addr;
} DbgCpuCtrlBreakPointType;

DbgCpuCtrlBreakPointType dbg_cpuctrl_break_points[DBG_CPU_CONTROL_BREAK_SETSIZE] = {
		{ TRUE, 0x00 },
};

typedef struct {
	bool					is_set;
	DataWatchPointEumType	type;
	uint32					addr;
	uint32					size;
} DbgCpuCtrlDataWatchType;
DbgCpuCtrlDataWatchType dbg_cpuctrl_data_watch_points[DBG_CPU_CONTROL_WATCH_DATA_SETSIZE];

bool dbg_cpuctrl_dbg_mode = TRUE;

typedef struct {
	bool is_stopped;
	CoreIdType core_id;
} DbgCpuStoppedCoreType;
DbgCpuStoppedCoreType dbg_cpu_stopped_core;

typedef struct {
	bool	is_timeout;
	uint64 	cont_clocks;
} DbgCpuContType;

static DbgCpuContType dbg_cpu_cont[CPU_CONFIG_CORE_NUM];


typedef struct {
	uint32	current;
	uint32	lognum;
	uint32	sp[DBG_FUNCLOG_TRACE_SIZE];
	uint32	funcid[DBG_FUNCLOG_TRACE_SIZE];
	uint32	funcoff[DBG_FUNCLOG_TRACE_SIZE];
	uint32	funcpc[DBG_FUNCLOG_TRACE_SIZE];
	char 	*funcname[DBG_FUNCLOG_TRACE_SIZE];
} DbgFuncLogTraceType;

static DbgFuncLogTraceType dbg_func_log_trace;

#include "file.h"
#include "file_address_mapping.h"
static FileType dbg_std_executor_file;

static char *search_filepath(char *dir, char *file)
{
	uint32 param_num;
	uint32 i;
	Std_ReturnType err;
	char *candiate_path;
	static char buffer[4096];
	static char parameter[4096];

	//printf("%s\n", dir);
	snprintf(buffer, sizeof(buffer), "%s/%s", dir, file);
	if (file_exist(buffer) == TRUE) {
		return buffer;
	}
	err = cpuemu_get_devcfg_value("EDITOR_SEARCH_PATH_NUM", &param_num);
	if (err != STD_E_OK) {
		return NULL;
	}
	//printf("param_num=%d\n", param_num);
	for (i = 0; i < param_num; i++) {
		snprintf(parameter, sizeof(parameter), "EDITOR_SEARCH_PATH_%d", i);
		err = cpuemu_get_devcfg_string(parameter, &candiate_path);
		if (err != STD_E_OK) {
			printf("not found param=%s\n", parameter);
			return NULL;
		}
		snprintf(buffer, sizeof(buffer), "%s/%s/%s", candiate_path, dir, file);
		if (candiate_path[0] == '/') {
			candiate_path[0] = candiate_path[1];
			candiate_path[1] = ':';
		}
		//printf("%s = %s %s\n", parameter, candiate_path, buffer);
		if (file_exist(buffer) == TRUE) {
			return buffer;
		}
	}
	return NULL;
}

void dbg_cpu_control_print_source(uint32 pc)
{
	Std_ReturnType err = STD_E_OK;
	ValueFileType value;
	err = file_address_mapping_get(pc, &value);
	if (err == STD_E_OK) {
		char *path = search_filepath(value.dir, value.file);
		if (path != NULL) {
			token_string_set(&dbg_std_executor_file.filepath, "./arg_sakura.txt");
			file_wopen(&dbg_std_executor_file);
			int len = snprintf((char*)dbg_std_executor_file.buffer,
					sizeof(dbg_std_executor_file.buffer),
					"-Y=%u %s\n", value.line, path);
			file_putline(&dbg_std_executor_file, (char*)dbg_std_executor_file.buffer, len);
			file_close(&dbg_std_executor_file);
			printf("[NEXT> pc=0x%x %s %u\n", pc, value.file, value.line);
			/*
			 * サクラエディタが常にフォーカスされてしまい，デバッグを阻害するため，コメントアウト
			 */
			//dbg_cpu_control_update_editor();
		}
	}
	return;
}
char *dbg_cpu_control_get_print_args(void)
{
	return (char*)dbg_std_executor_file.buffer;
}
void dbg_cpu_control_update_editor(void)
{
	char cmd[256];
	snprintf(cmd, sizeof(cmd), "sh sakura.sh");
	if (system(cmd) < 0) {
		printf("can not execute sakura\n");
	}
	return;
}


/*
 * データ×アクセス関数マッピング表
 */
static DataAccessInfoType *data_access_info;
static DataAccessInfoType **data_access_info_table_gl;
static uint32 current_funcid;

DataAccessInfoType *cpuctrl_get_func_access_info_table(const char* glname)
{
	uint32 addr;
	uint32 size;
	int glid;
	glid = symbol_get_gl((char*)glname, strlen(glname), &addr, &size);
	if (glid < 0) {
		return NULL;
	}
	return data_access_info_table_gl[glid];
}

void cpuctrl_set_func_log_trace(uint32 pc, uint32 sp)
{
	uint32 inx;
	uint32 next;
	uint32 funcpc;
	char *funcname;
	int funcid;

	funcid = symbol_pc2funcid(pc, &funcpc);
	if (funcid < 0) {
		current_funcid = -1;
		return;
	}
	current_funcid = funcid;
	funcname = symbol_funcid2funcname(funcid);

	if (dbg_func_log_trace.lognum > 0) {
		inx = dbg_func_log_trace.current;
		if (dbg_func_log_trace.funcpc[inx] == funcpc) {
			return;
		}
		next = inx + 1;
		if (next >= DBG_FUNCLOG_TRACE_SIZE) {
			next = 0;
		}
	} else {
		next = 0;
	}

	dbg_func_log_trace.current = next;
	dbg_func_log_trace.sp[next] = sp;
	dbg_func_log_trace.funcid[next] = funcid;
	dbg_func_log_trace.funcoff[next] = pc - funcpc;
	dbg_func_log_trace.funcname[next] = funcname;
	dbg_func_log_trace.funcpc[next] = funcpc;
	if (dbg_func_log_trace.lognum < DBG_FUNCLOG_TRACE_SIZE) {
		dbg_func_log_trace.lognum++;
	}
	return;
}

char *cpuctrl_get_func_log_trace_info(uint32 bt_number, uint32 *funcpcoff, uint32 *funcid, uint32 *sp)
{
	int off;
	if (bt_number >= DBG_FUNCLOG_TRACE_SIZE) {
		return NULL;
	}
	if (bt_number >= dbg_func_log_trace.lognum) {
		return NULL;
	}

	if (dbg_func_log_trace.current >= bt_number) {
		off = dbg_func_log_trace.current - bt_number;
	}
	else {
		off = DBG_FUNCLOG_TRACE_SIZE - (bt_number - dbg_func_log_trace.current);
	}
	*sp = dbg_func_log_trace.sp[off];
	*funcid = dbg_func_log_trace.funcid[off];
	*funcpcoff = dbg_func_log_trace.funcoff[off];
	return dbg_func_log_trace.funcname[off];

}

void cpuctrl_set_cont_clocks(bool is_timeout, uint64 cont_clocks)
{
	dbg_cpu_cont[dbg_cpu_stopped_core.core_id].is_timeout = is_timeout;
	dbg_cpu_cont[dbg_cpu_stopped_core.core_id].cont_clocks = cont_clocks;
	return;
}
bool cpuctrl_is_timeout_cont_clocks(CoreIdType core_id)
{
	if (dbg_cpu_cont[core_id].is_timeout == TRUE) {
		if (dbg_cpu_cont[core_id].cont_clocks > 1) {
			dbg_cpu_cont[core_id].cont_clocks--;
			return FALSE;
		}
		return TRUE;
	}
	else {
		return FALSE;

	}
}

void cpuctrl_set_current_debugged_core(CoreIdType core_id)
{
	dbg_cpu_stopped_core.is_stopped = TRUE;
	dbg_cpu_stopped_core.core_id = core_id;
	return;
}
bool cpuctrl_get_current_debugged_core(CoreIdType *core_id)
{
	if (dbg_cpu_stopped_core.is_stopped == TRUE) {
		*core_id = dbg_cpu_stopped_core.core_id;
		return TRUE;
	}
	return FALSE;
}
void cpuctrl_clr_current_debugged_core(void)
{
	dbg_cpu_stopped_core.is_stopped = FALSE;
	return;
}


static DbgCpuCtrlBreakPointType *search_free_break_point_space(void)
{
	uint32 i;
	for (i = 0; i < DBG_CPU_CONTROL_BREAK_SETSIZE; i++) {
		if (dbg_cpuctrl_break_points[i].is_set == FALSE) {
			return &dbg_cpuctrl_break_points[i];
		}
	}
	return NULL;
}

static DbgCpuCtrlBreakPointType *search_break_point(uint32 addr)
{
	uint32 i;
	for (i = 0; i < DBG_CPU_CONTROL_BREAK_SETSIZE; i++) {
		if (dbg_cpuctrl_break_points[i].is_set == FALSE) {
			continue;
		}
		else if (dbg_cpuctrl_break_points[i].addr == addr) {
			return &dbg_cpuctrl_break_points[i];
		}
	}
	return NULL;
}

static DbgCpuCtrlBreakPointType *search_break_point_with_type(uint32 addr, BreakPointEumType type)
{
	uint32 i;
	for (i = 0; i < DBG_CPU_CONTROL_BREAK_SETSIZE; i++) {
		if (dbg_cpuctrl_break_points[i].is_set == FALSE) {
			continue;
		}
		else if (dbg_cpuctrl_break_points[i].type != type) {
			continue;
		}
		else if (dbg_cpuctrl_break_points[i].addr == addr) {
			return &dbg_cpuctrl_break_points[i];
		}
	}
	return NULL;
}

bool cpuctrl_get_break(uint32 index, uint32 *addrp)
{
	if (index >= DBG_CPU_CONTROL_BREAK_SETSIZE) {
		return FALSE;
	}
	if (dbg_cpuctrl_break_points[index].is_set == FALSE) {
		return FALSE;
	}
	*addrp = dbg_cpuctrl_break_points[index].addr;
	return TRUE;
}

bool cpuctrl_is_break_point(uint32 addr)
{
	if (search_break_point(addr) != NULL) {
		return TRUE;
	}
	return FALSE;
}

bool cpuctrl_is_debug_mode(void)
{
	return (dbg_cpuctrl_dbg_mode == TRUE);
}

bool cpuctrl_set_break(uint32 addr, BreakPointEumType type)
{
	DbgCpuCtrlBreakPointType *bp;
	if (search_break_point_with_type(addr, type) != NULL) {
		return TRUE;
	}
	bp = search_free_break_point_space();
	if (bp != NULL) {
		bp->type = type;
		bp->is_set = TRUE;
		bp->addr = addr;
		return TRUE;
	}
	return FALSE;
}


bool cpuctrl_del_break(uint32 index)
{
	if (index > 0) {
		dbg_cpuctrl_break_points[index].is_set = FALSE;
		return TRUE;
	}
	return FALSE;
}
void cpuctrl_del_all_break(BreakPointEumType type)
{
	 uint32 i;
	 for (i = 0; i < DBG_CPU_CONTROL_BREAK_SETSIZE; i++) {
		 if (dbg_cpuctrl_break_points[i].type == type) {
			 cpuctrl_del_break(i);
		 }
	 }
	 return;
}
#define ACCESS_TYPE_READ	0x01
#define ACCESS_TYPE_WRITE	0x02
static void cpuctrl_set_access(uint32 access_type, uint32 access_addr, uint32 size)
{
	uint32 i;
	uint32 prev_glid = -1;
	uint32 glid;
	uint32 gladdr;
	DataAccessInfoType *access_infop;

	if (current_funcid == -1) {
		return;
	}
	for (i = 0; i < size; i++) {
		glid = symbol_addr2glid(access_addr + i, &gladdr);
		if (glid < 0) {
			continue;
		}
		if (glid == prev_glid) {
			continue;
		}
		access_infop = data_access_info_table_gl[glid];
		if (access_type == ACCESS_TYPE_READ) {
			access_infop[current_funcid].read_access_num++;
		}
		else {
			access_infop[current_funcid].write_access_num++;
		}
		prev_glid = glid;
	}
	return;
}


int cpuctrl_is_break_read_access(uint32 access_addr, uint32 size)
{
	uint32 i;
	uint32 watch_start;
	uint32 watch_end;
	uint32 access_end = access_addr + size;

	cpuctrl_set_access(ACCESS_TYPE_READ, access_addr, size);

	for (i = 0; i < DBG_CPU_CONTROL_WATCH_DATA_SETSIZE; i++) {
		if (dbg_cpuctrl_data_watch_points[i].is_set == FALSE) {
			continue;
		}
		if (dbg_cpuctrl_data_watch_points[i].type == DATA_WATCH_POINT_TYPE_WRITE) {
			continue;
		}
		watch_start = dbg_cpuctrl_data_watch_points[i].addr;
		watch_end = watch_start + dbg_cpuctrl_data_watch_points[i].size;
		if (access_end <= watch_start) {
			continue;
		}
		else if (access_addr >= watch_end) {
			continue;
		}
		return i;
	}
	return -1;
}

int cpuctrl_is_break_write_access(uint32 access_addr, uint32 size)
{
	uint32 i;
	uint32 watch_start;
	uint32 watch_end;
	uint32 access_end = access_addr + size;

	cpuctrl_set_access(ACCESS_TYPE_WRITE, access_addr, size);

	for (i = 0; i < DBG_CPU_CONTROL_WATCH_DATA_SETSIZE; i++) {
		if (dbg_cpuctrl_data_watch_points[i].is_set == FALSE) {
			continue;
		}
		if (dbg_cpuctrl_data_watch_points[i].type == DATA_WATCH_POINT_TYPE_READ) {
			continue;
		}
		watch_start = dbg_cpuctrl_data_watch_points[i].addr;
		watch_end = watch_start + dbg_cpuctrl_data_watch_points[i].size;
		if (access_end <= watch_start) {
			continue;
		}
		else if (access_addr >= watch_end) {
			continue;
		}
		return i;
	}
	return -1;
}
bool cpuctrl_get_data_watch_point(uint32 index, uint32 *addrp, uint32 *sizep, DataWatchPointEumType *type)
{
	if (index >= DBG_CPU_CONTROL_WATCH_DATA_SETSIZE) {
		return FALSE;
	}
	if (dbg_cpuctrl_data_watch_points[index].is_set == FALSE) {
		return FALSE;
	}
	*addrp = dbg_cpuctrl_data_watch_points[index].addr;
	*sizep = dbg_cpuctrl_data_watch_points[index].size;
	*type = dbg_cpuctrl_data_watch_points[index].type;
	return TRUE;
}

bool cpuctrl_set_data_watch(DataWatchPointEumType watch_type, uint32 addr, uint32 size)
{
	uint32 i;
	/*
	 * 既存のものを探し，上書きする
	 */
	for (i = 0; i < DBG_CPU_CONTROL_WATCH_DATA_SETSIZE; i++) {
		if (dbg_cpuctrl_data_watch_points[i].is_set == FALSE) {
			continue;
		}
		if (dbg_cpuctrl_data_watch_points[i].addr == addr) {
			dbg_cpuctrl_data_watch_points[i].type = watch_type;
			dbg_cpuctrl_data_watch_points[i].addr = addr;
			dbg_cpuctrl_data_watch_points[i].size = size;
			return TRUE;
		}
	}
	/*
	 * 既存のものがない場合は，新規設定する
	 */
	for (i = 0; i < DBG_CPU_CONTROL_WATCH_DATA_SETSIZE; i++) {
		if (dbg_cpuctrl_data_watch_points[i].is_set == FALSE) {
			dbg_cpuctrl_data_watch_points[i].is_set = TRUE;
			dbg_cpuctrl_data_watch_points[i].type = watch_type;
			dbg_cpuctrl_data_watch_points[i].addr = addr;
			dbg_cpuctrl_data_watch_points[i].size = size;
			return TRUE;
		}
	}
	return FALSE;
}
bool cpuctrl_del_data_watch_point(uint32 delno)
{
	if (delno >= DBG_CPU_CONTROL_WATCH_DATA_SETSIZE) {
		return FALSE;
	}
	dbg_cpuctrl_data_watch_points[delno].is_set = FALSE;
	return TRUE;
}
void cpuctrl_del_all_data_watch_points(void)
{
	uint32 i;

	for (i = 0; i < DBG_CPU_CONTROL_WATCH_DATA_SETSIZE; i++) {
		cpuctrl_del_data_watch_point(i);
	}
	return;
}


void cpuctrl_set_debug_mode(bool on)
{
	dbg_cpuctrl_dbg_mode = on;
	return;
}
void cpuctrl_set_force_break(void)
{
	cpuctrl_set_debug_mode(TRUE);

	return;
}

/*
 * profile機能
 */
static CpuProfileType *CpuProfile;


void cpuctrl_init(void)
{
	uint32 i;
	uint32 func_num = symbol_get_func_num();
	uint32 gl_num = symbol_get_gl_num();
	CpuProfile = malloc(func_num * sizeof(CpuProfileType));
	ASSERT(CpuProfile != NULL);
	memset(CpuProfile, 0, func_num * sizeof(CpuProfileType));

	data_access_info = malloc(func_num * gl_num * sizeof(DataAccessInfoType));
	ASSERT(data_access_info != NULL);
	memset(data_access_info, 0, func_num * gl_num * sizeof(DataAccessInfoType));

	data_access_info_table_gl = malloc(gl_num * sizeof(DataAccessInfoType *));
	for (i = 0; i < gl_num; i++) {
		data_access_info_table_gl[i] = &data_access_info[(func_num * i)];
	}

	return;
}
void cpuctrl_profile_collect(uint32 pc)
{
	int funcid;
	uint32 funcpc;
	uint32 funcaddr;
	CpuEmuElapsType elaps;

	funcid = symbol_pc2funcid(pc, &funcaddr);
	if (funcid < 0) {
		return;
	}
	cpuemu_get_elaps(&elaps);
	funcpc = symbol_funcid2funcaddr(funcid);

	if (pc == funcpc) {
		CpuProfile[funcid].call_num++;
		if (CpuProfile[funcid].last_time > 0) {
			CpuProfile[funcid].total_time +=
					(CpuProfile[funcid].last_time - CpuProfile[funcid].start_time);
		}
		CpuProfile[funcid].start_time = elaps.total_clocks;
	}
	CpuProfile[funcid].func_time++;
	CpuProfile[funcid].last_time = elaps.total_clocks;
	return;
}
void cpuctrl_profile_get(uint32 funcid, CpuProfileType *profile)
{
	*profile = CpuProfile[funcid];
	return;
}
/*
 * 関数フレーム記録
 */
typedef struct {
	uint32	current;
	uint32	lognum;
	uint32	sp[DBG_STACK_LOG_SIZE];
} DbgFuncFrameType;
static DbgFuncFrameType dbg_func_frame[DBG_STACK_NUM];

void cpuctrl_set_stack_pointer(uint32 sp)
{
	uint32 inx;
	uint32 next;
	uint32 prev;
	uint32 gladdr;
	int glid;

	glid = symbol_addr2glid(sp, &gladdr);
	if (glid < 0) {
		return;
	}
	if (dbg_func_frame[glid].lognum > 0) {
		inx = dbg_func_frame[glid].current;
		/*
		 * 同じスタックポインタの場合は終了
		 */
		if (dbg_func_frame[glid].sp[inx] == sp) {
			return;
		}
		/*
		 * 一個前のスタックポインタの場合は縮小する．
		 */
		for (prev = 0; prev < dbg_func_frame[glid].lognum; prev++) {
			if (dbg_func_frame[glid].sp[prev] == sp) {
				dbg_func_frame[glid].lognum = prev + 1;
				dbg_func_frame[glid].current = prev;
				return;
			}
		}
		/*
		 * 新しいスタックポインタの場合は追加する
		 */
		next = inx + 1;
		ASSERT(next <= DBG_STACK_LOG_SIZE);
	} else {
		next = 0;
	}
	dbg_func_frame[glid].current = next;
	dbg_func_frame[glid].sp[next] = sp;
	dbg_func_frame[glid].lognum++;
	return;
}

Std_ReturnType cpuctrl_get_stack_pointer(int glid, uint32 bt_number, uint32 *sp)
{
	if (dbg_func_frame[glid].lognum <= 1) {
		return STD_E_NOENT;
	}

	if (bt_number >= dbg_func_frame[glid].lognum) {
		return STD_E_NOENT;
	}
	*sp = dbg_func_frame[glid].sp[bt_number];
	return STD_E_OK;
}
