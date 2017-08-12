#ifndef _DBG_EXECUTOR_H_
#define _DBG_EXECUTOR_H_

#include "dbg_interaction_std_ids.h"
#include "dbg_interaction_target_ids.h"
#include "std_types.h"


#define DBG_CMD_BUFFER_SIZE		(4096*256U)
typedef struct {
	DbgCmdStdIdType			std_id;
	DbgCmdTargetIdType		target_id;
	uint8	original_str[DBG_CMD_BUFFER_SIZE];
	uint8	parsed_args[DBG_CMD_BUFFER_SIZE];
	void (*run) (void *executor);
} DbgCmdExecutorType;

#endif /* _DBG_EXECUTOR_H_ */
