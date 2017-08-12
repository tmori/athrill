#include "cpu.h"
#include "bus.h"
#include "cpu_control/dbg_cpu_callback.h"
#include "cpu_control/dbg_cpu_thread_control.h"
#include "cpu_control/dbg_cpu_control.h"
#include "cpuemu_ops.h"
#include "cui/cui_ops.h"
#include "symbol_ops.h"

void dbg_notify_cpu_clock_supply_start(const TargetCoreType *core)
{
	bool need_stop = FALSE;
	uint32 pc = cpu_get_pc(core);

	if (cpuemu_cui_mode() == FALSE) {
		return;
	}

	/*
	 * cont timeout check
	 * break point check
	 * debug mode check
	 */
	if (cpuctrl_is_timeout_cont_clocks(cpu_get_core_id(core)) == TRUE) {
		need_stop = TRUE;
		printf("\nCONT TIMEOUT\n");
		//printf("[DBG>");
	}
	else if ((cpuctrl_is_break_point(pc) == TRUE)) {
		 uint32 funcaddr;
		 int funcid;
		 need_stop = TRUE;
		 funcid = symbol_pc2funcid(pc, &funcaddr);
		 if (funcid >= 0) {
			 printf("\nHIT break:0x%x %s(+0x%x)\n", pc, symbol_funcid2funcname(funcid), pc - funcaddr);
		 }
		 else {
			 printf("\nHIT break:0x%x\n", pc);
		 }

	}
	else if ((cpuctrl_is_debug_mode() == TRUE)) {
		need_stop = TRUE;
	}

	if (need_stop == TRUE) {
		dbg_cpu_control_print_source(pc);
		fflush(stdout);
		CUI_PRINTF((CPU_PRINT_BUF(), CPU_PRINT_BUF_LEN(), "core[%u].pc = %x\n", cpu_get_core_id(core), pc));
		cpuctrl_set_current_debugged_core(cpu_get_core_id(core));
		cpuctrl_set_debug_mode(TRUE);
		dbg_log_sync();
		/*
		 * return コマンド実行時の一時的なブレークポイントを削除する．
		 */
		cpuctrl_del_all_break(BREAK_POINT_TYPE_ONLY_ONCE);
		cputhr_control_cpu_wait();
	}
	return;
}

void dbg_notify_cpu_clock_supply_end(const TargetCoreType *core, const DbgCpuCallbackFuncEnableType *enable_dbg)
{
	uint32 pc = cpu_get_pc(core);
	uint32 sp = cpu_get_sp(core);
	BusAccessType type;
	uint32 size;
	uint32 access_addr;
	bool need_stop = FALSE;

	if (cpuemu_cui_mode() == FALSE) {
		return;
	}
	/*
	 * call callback
	 */

	if (enable_dbg->enable_ft == TRUE) {
		cpuctrl_set_func_log_trace(pc, sp);
	}
	if (enable_dbg->enable_prof == TRUE) {
		cpuctrl_profile_collect(pc);
	}
	if (enable_dbg->enable_bt == TRUE) {
		cpuctrl_set_stack_pointer(sp);
	}
	if (enable_dbg->enable_watch == TRUE) {
		/*
		 * data watch check
		 */
		while (TRUE) {
			int inx;
			Std_ReturnType err;
			err = bus_access_get_log(&type, &size, &access_addr);
			if (err != STD_E_OK) {
				break;
			}
			if (type == BUS_ACCESS_TYPE_READ) {
				inx = cpuctrl_is_break_read_access(access_addr, size);
				if (inx >= 0) {
					need_stop = TRUE;
					printf("\nHIT watch data : read access : [%u] 0x%x %u\n", inx, access_addr, size);
				}
			}
			else if (type == BUS_ACCESS_TYPE_WRITE) {
				inx = cpuctrl_is_break_write_access(access_addr, size);
				if (inx >= 0) {
					need_stop = TRUE;
					printf("\nHIT watch data : write access : [%u] 0x%x %u\n", inx, access_addr, size);
				}
			}
		}

		if (need_stop == TRUE) {
			cpuctrl_set_debug_mode(TRUE);
		}
	}

	return;
}
