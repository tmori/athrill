#ifndef _ATHRILL_MROS_DEVICE_H_
#define _ATHRILL_MROS_DEVICE_H_

#include "ros_cimpl.h"

typedef struct {
	const char* topic_name;
	mRosPublisherType *pub;
} AthrillMrosDevPubReqType;

typedef struct {
	const char* topic_name;
	void (*callback) (const char *data, int datalen);
	mRosSubscriberType *sub;
} AthrillMrosDevSubReqType;

extern int athrill_mros_device_pub_register(AthrillMrosDevPubReqType *reqs, int req_num);
extern int athrill_mros_device_sub_register(AthrillMrosDevSubReqType *reqs, int req_num);

extern int athrill_mros_device_start(void);
extern int athrill_mros_device_sub_read(mRosSubscriberType *sub, char *datap, int datalen, int *res);

#endif /* _ATHRILL_MROS_DEVICE_H_ */
