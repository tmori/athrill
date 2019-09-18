#include "mros_types.h"
#include "mros_memory.h"
#include "mros_sys_config.h"
#include "kernel.h"

void mros_sys_config_init(void)
{
	return;
}


void usr_task_activation(void)
{
	mros_uint32 i;
	for (i = 0; i < MROS_USR_TASK_NUM; i++) {
		act_tsk(mros_usr_task_table[i]);
	}
}

