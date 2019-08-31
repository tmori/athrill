#ifndef _ROS_CIMPL_H_
#define _ROS_CIMPL_H_

typedef struct {
	void *objp;
} mRosObjType;

typedef struct {
	mRosObjType 	*objp;
	int				topic_id;
} mRosPublisherType;

typedef struct {
	mRosObjType 	*objp;
	int				topic_id;
	void			*ptr;
} mRosSubscriberType;

extern void ros_init(int argc, char *argv, const char* node_name);
extern mRosSubscriberType *ros_topic_subscribe(const char* topic, int queue_size, void (*fp) (const char *));
extern mRosPublisherType *ros_topic_advertise(const char* topic, int queue_size);
extern int ros_topic_publish(mRosPublisherType* pub, void *data, int datalen);

typedef unsigned int mRosCallbackTopicIdType;
typedef unsigned int mRosCallbackDataLenType;
extern mRosCallbackTopicIdType ros_topic_callback_topic_id(void);
extern mRosCallbackDataLenType ros_topic_callback_datalen(void);
extern void ros_topic_callback_lock(void);
extern void ros_topic_callback_unlock(void);

extern void set_main_task(void);
extern void set_athrill_task(void);

#include "mros_log.h"

#endif /* _ROS_CIMPL_H_ */
