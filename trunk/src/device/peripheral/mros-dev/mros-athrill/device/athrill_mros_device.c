#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include "athrill_mros_device.h"
#include "mros_os_config.h"

typedef struct {
	char 					*buffer;
	int						recvlen;
	int						buffer_len;
	mRosSubscriberType		*sub;
	void (*func) (const char *data, int datalen);
} mRosTopicSubBufferType;

static int mrostopic_sub_buffer_num;
static mRosTopicSubBufferType *mros_topic_sub_buffer;

static void *sub_buffer_realloc(mRosTopicSubBufferType *sub, int len)
{
	if (sub->buffer != NULL) {
		free(sub->buffer);
	}
	sub->buffer = malloc(len);
	if (sub->buffer == NULL) {
		return NULL;
	}
	sub->buffer_len = len;
	return sub->buffer;
}

static mRosTopicSubBufferType *sub_buffer_find(int topic_id)
{
	int i;
	for (i = 0; i < mrostopic_sub_buffer_num; i++) {
		if (mros_topic_sub_buffer[i].sub->topic_id == topic_id) {
			return &mros_topic_sub_buffer[i];
		}
	}
	return NULL;
}

static mRosTopicSubBufferType *get_subptr(int topic_id, mRosCallbackDataLenType len)
{
	mRosTopicSubBufferType *sub;
	if (mros_topic_sub_buffer == NULL) {
		return NULL;
	}

	sub = sub_buffer_find(topic_id);
	if (sub == NULL) {
		return NULL;
	}
	if (sub->buffer_len < len) {
		if (sub_buffer_realloc(sub, len) == NULL) {
			return NULL;
		}
	}
	sub->recvlen = len;

	return sub;
}

static void athrill_mros_callback(const char *msg)
{
	mRosCallbackTopicIdType topic_id;
	mRosCallbackDataLenType datalen;

	ros_topic_callback_lock();
	{
		topic_id = ros_topic_callback_topic_id();
		datalen = ros_topic_callback_datalen();

		mRosTopicSubBufferType *bp = get_subptr(topic_id, datalen + 1);
		memcpy(bp->buffer, msg, datalen + 1);
		bp->buffer[datalen] = '\0';
		printf("callback:topic_id=%u:datalen=%u: %s\n", topic_id, datalen, bp->buffer);
		if (bp->func != NULL) {
			bp->func(bp->buffer, datalen);
		}
	}
	ros_topic_callback_unlock();
	return;
}

typedef struct {
	struct {
		int req_num;
		AthrillMrosDevPubReqType *reqs;
	} pub;
	struct {
		int req_num;
		AthrillMrosDevSubReqType *reqs;
	} sub;
} AthrillMrosDeviceRegisterType;

static AthrillMrosDeviceRegisterType athrill_mros_device_register;

int athrill_mros_device_pub_register(AthrillMrosDevPubReqType *reqs, int req_num)
{
	athrill_mros_device_register.pub.req_num = req_num;
	athrill_mros_device_register.pub.reqs = reqs;
	return 0;
}

int athrill_mros_device_sub_register(AthrillMrosDevSubReqType *reqs, int req_num)
{
	athrill_mros_device_register.sub.req_num = req_num;
	athrill_mros_device_register.sub.reqs = reqs;
	return 0;
}

static int athrill_mros_device_pub_init(AthrillMrosDevPubReqType *reqs, int req_num)
{
	mRosPublisherType *pub_cobj;
	int i;

	for (i = 0; i < req_num; i++) {
		pub_cobj = ros_topic_advertise(reqs[i].topic_name, 1);
		if (pub_cobj == NULL) {
			return -1;
		}
		reqs[i].pub = pub_cobj;
	}
	return 0;
}

static int athrill_mros_device_sub_init(AthrillMrosDevSubReqType *reqs, int req_num)
{
	mRosSubscriberType *sub_cobj;
	int i;

	mros_topic_sub_buffer = malloc(sizeof(mRosTopicSubBufferType) * req_num);
	if (mros_topic_sub_buffer == NULL) {
		return -1;
	}
	mrostopic_sub_buffer_num = req_num;
	for (i = 0; i < req_num; i++) {
		sub_cobj = ros_topic_subscribe(reqs[i].topic_name, 1, athrill_mros_callback);
		if (sub_cobj == NULL) {
			return -1;
		}
		reqs[i].sub = sub_cobj;
		reqs[i].sub->ptr = &mros_topic_sub_buffer[i];
		mros_topic_sub_buffer[i].buffer = NULL;
		mros_topic_sub_buffer[i].buffer_len = 0;
		mros_topic_sub_buffer[i].sub = sub_cobj;
		mros_topic_sub_buffer[i].func = reqs[i].callback;
	}
	return 0;
}

static void *athrill_mros_device_main(void *arg)
{
	int err;
	set_main_task();
	main_task();
	ros_init(0, NULL, "athrill_node");

	err = athrill_mros_device_pub_init(athrill_mros_device_register.pub.reqs, athrill_mros_device_register.pub.req_num);
	if (err != 0) {
		printf("ERROR: athrill_mros_device_pub_init()\n");
		exit(1);
	}
	err = athrill_mros_device_sub_init(athrill_mros_device_register.sub.reqs, athrill_mros_device_register.sub.req_num);
	if (err != 0) {
		printf("ERROR: athrill_mros_device_pub_init()\n");
		exit(1);
	}
	while (1) {
		usleep(1000*100); /* 100msec */
		cyclic_handler(0);
	}

	return NULL;
}

int athrill_mros_device_start(void)
{
	pthread_t thread;

	return pthread_create(&thread, NULL, athrill_mros_device_main, NULL);
}

int athrill_mros_device_sub_read(mRosSubscriberType *sub, char *datap, int datalen, int *res)
{
	int ret = -1;
	int copylen = datalen;
	mRosTopicSubBufferType *bp = sub_buffer_find(sub->topic_id);
	if (bp == NULL) {
		return -1;
	}
	ros_topic_callback_lock();
	{
		if (bp->buffer != NULL) {
			if (datalen > bp->recvlen) {
				copylen = bp->recvlen;
			}
			memcpy(datap, bp->buffer, copylen);
			ret = 0;
		}
	}
	ros_topic_callback_unlock();
	*res = copylen;
	return ret;
}
