#include "dbg_can.h"
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <memory.h>
#include <windows.h>

#include <canlib.h>

static bool dbg_kavaser_can_init(uint32 ch);
static bool dbg_kavaser_can_recv(uint32 *ch, uint32 *canid, uint32 *ex_canid, uint8 *data, uint8 *dlc, uint8 *canid_type);
static bool dbg_kavaser_can_send(uint32 ch, uint32 can_id, uint8 *data, uint8 dlc, uint8 canid_type);

DeviceCanOpType dbg_can_ops = {
	dbg_kavaser_can_init,
	dbg_kavaser_can_recv,
	dbg_kavaser_can_send,
};

typedef struct {
	int channel;
	int baud;
	CanHandle handle;
	canStatus status;
} DbgKvaserType;

static DbgKvaserType dbg_kvaser = {
	0,
	BAUD_500K,
	0,
	canOK
};


static bool dbg_kavaser_can_init(uint32 ch)
{
	CanHandle handle;

	canInitializeLibrary();

	handle = canOpenChannel(dbg_kvaser.channel, canOPEN_EXCLUSIVE);
	if (handle < 0) {
		printf("dbg_kavaser_can_init:canOpenChannel err=%d\n", handle);
		return FALSE;
	}
	printf("dbg_kavaser_can_init:canOpenChannel OK:handle=%d\n", handle);

	dbg_kvaser.handle = handle;
	dbg_kvaser.status = canSetBusParams(handle, dbg_kvaser.baud, 0, 0, 0, 0, 0);
	if (dbg_kvaser.status != canOK) {
		printf("dbg_kavaser_can_init:canSetBusParams err=%d\n", dbg_kvaser.status);
		return FALSE;
	}
	printf("dbg_kavaser_can_init:canSetBusParams OK:baud=%d\n", dbg_kvaser.baud);
	dbg_kvaser.status = canBusOn(handle);
	if (dbg_kvaser.status != canOK) {
		printf("dbg_kavaser_can_init:canBusOn err=%d\n", dbg_kvaser.status);
		return FALSE;
	}
	printf("dbg_kavaser_can_init:canBusOn OK\n");

	return TRUE;
}
static bool dbg_kavaser_can_recv(uint32 *ch, uint32 *canid, uint32 *ex_canid, uint8 *data, uint8 *dlc, uint8 *canid_type)
{
	unsigned int flag = 0;
	unsigned int out_canid;
	unsigned int out_dlc;
	unsigned long time;
	static unsigned int count= 0;
	count++;
	if ((count % 10000) != 0) {
		return FALSE;
	}

	dbg_kvaser.status = canReadWait(dbg_kvaser.handle, (long*)&out_canid, (void*)data, &out_dlc, (unsigned int*)&flag, &time, 0);
	if (dbg_kvaser.status != canOK) {
		//printf("dbg_kavaser_can_recv:canReadWait err=%d\n", dbg_kvaser.status);
		return FALSE;
	}

	*ch = 1;
	*canid = out_canid;
	*ex_canid = 0xFFFFFFFF;//TODO
	*dlc = out_dlc;

	if (flag & canMSG_STD) {
		*canid_type = 0;
	}
	else{
		*canid_type = 1;
	}

#if 0
		printf("##RCV_CAN::ch=%u\n", *ch);
		printf("##RCV_CNA:canid=0x%x\n", *canid);
		printf("##RCV_CAN:ex_canid=0x%x\n", *ex_canid);
		printf("##RCV_CNA:dlc=%u\n", *dlc);
		printf("##RCV_CNA:dlc=%u\n", *canid_type);
		printf("##");
		for (i = 0; i < *dlc; i++) {
			printf("0x%02x ", data[i]);
		}
		printf("\n");

		fflush(stdout);
#endif
	return TRUE;
}

static bool dbg_kavaser_can_send(uint32 ch, uint32 can_id, uint8 *data, uint8 dlc, uint8 canid_type)
{
	int flag = 0;

	if (canid_type == 0) {
		flag &= canMSG_STD;
	}
	else {
		flag &= canMSG_EXT;
	}

	dbg_kvaser.status = canWriteWait(dbg_kvaser.handle, can_id, data, dlc, flag, -1);
    if (dbg_kvaser.status != canOK) {
		printf("dbg_kavaser_can_send:canWriteWait err=%d\n", dbg_kvaser.status);
		fflush(stdout);
		return FALSE;
    }
	//printf("dbg_kavaser_can_send:canWriteWait msg_id=0x%x err=%d\n", can_id, dbg_kvaser.status);
	//fflush(stdout);
	return TRUE;
}

