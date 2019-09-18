#include "mros_types.h"
#include "mros_memory.h"
#include "mros_usr_config.h"
#include "kernel_cfg.h"



/****************************************
 * USR OS TASK
 ****************************************/

mRosTaskIdType mros_usr_task_table[MROS_USR_TASK_NUM] = {
	USR_TASK1,
	USR_TASK2,
};
