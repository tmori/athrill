#ifndef _MROS_USR_CONFIG_H_
#define _MROS_USR_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"
#include "mros_memory.h"
#include "mros_os.h"


/**************************************
 * TOPIC
 **************************************/

/*
 * num of max topics
 */
#define MROS_TOPIC_MAX_NUM						20U

/*
 * max name length of topic name
 */
#define MROS_TOPIC_NAME_MAXLEN					50U
/*
 * max name length of topic type name
 */
#define MROS_TOPIC_TYPENAME_MAXLEN				20U


/**************************************
 * TOPIC CONNECTOR
 **************************************/
/*
 * num of max publish topic connectors
 */
#define MROS_PUB_TOPIC_CONNECTOR_MAX_NUM		10U

/*
 * num of max subscribe topic connectors
 */
#define MROS_SUB_TOPIC_CONNECTOR_MAX_NUM		10U

/*
 * max queue length of outer connection topic data buffering
 */
#define MROS_OUTER_CONNECTOR_QUEUE_MAXLEN		1


/**************************************
 * NODE
 **************************************/
/*
 * num of max nodes
 */
#define MROS_NODE_MAX_NUM						10U
/*
 * max name length of node name
 */
#define MROS_NODE_NAME_MAXLEN					20U

/*
 * do not change for mROS inner data
 */
extern mRosMemoryManagerType ros_inner_topic_publisher_mempool;
extern mRosMemoryManagerType ros_outer_topic_publisher_mempool;

/****************************************
 * USR OS TASK
 ****************************************/
/*
 * num of user task
 *
 * following config parameter is an example.
 *
 * please change corresponding variable definitions
 * of mros_usr_config.c for changing these parameters.
 */
#define MROS_USR_TASK_NUM			1		// for athrill-thread
/*
 * do not change for mROS inner data
 */
extern mRosTaskIdType mros_usr_task_table[MROS_USR_TASK_NUM];

#ifdef __cplusplus
}
#endif

#endif /* _MROS_USR_CONFIG_H_ */

