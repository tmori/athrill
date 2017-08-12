#include "dbg_can.h"
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

static bool dbg_stdio_can_init(uint32 ch);
static bool dbg_stdio_can_recv(uint32 *ch, uint32 *canid, uint32 *ex_canid, uint8 *data, uint8 *dlc, uint8 *canid_type);
static bool dbg_stdio_can_send(uint32 ch, uint32 can_id, uint8 *data, uint8 dlc, uint8 canid_type);

DeviceCanOpType dbg_can_ops = {
		dbg_stdio_can_init,
		dbg_stdio_can_recv,
		dbg_stdio_can_send,
};


typedef struct {
	bool is_rcv;
	uint32 ch;
	uint32 canid;
	uint32 ex_canid;
	uint8 data[8U];
	uint8 dlc;
} DbgCmdCanRcvDataType;

static DbgCmdCanRcvDataType dbg_can_rcvinfo;

#if 0 //TODO
void dbg_do_can_recvbuf(DbgCmdType *cmd)
{
	int fd;
	int err;
	char data[1024];
	DbgCmdCanRcvDataType *can_data;

	fd = open(".\\can.txt", O_RDONLY|O_BINARY);
	if (fd < 0) {
		printf("dbg_do_can_recvbuf:open err=%d\n", errno);
		fflush(stdout);
		return;
	}
	err = read(fd, &data, 1024);
	if (err <= 0) {
		printf("dbg_do_can_recvbuf:read err=%d\n", errno);
		fflush(stdout);
		return;
	}
	can_data = (DbgCmdCanRcvDataType *)data;

	dbg_can_rcvinfo = *can_data;
	close(fd);
	return;
}
#endif

static bool dbg_stdio_can_init(uint32 ch)
{
	return TRUE;
}


static bool dbg_stdio_can_recv(uint32 *ch, uint32 *canid, uint32 *ex_canid, uint8 *data, uint8 *dlc, uint8 *canid_type)
{
	int i;
	if (dbg_can_rcvinfo.is_rcv) {
		*ch = dbg_can_rcvinfo.ch;
		*canid = dbg_can_rcvinfo.canid;
		*ex_canid = dbg_can_rcvinfo.ex_canid;
		*dlc = dbg_can_rcvinfo.dlc;
		dbg_can_rcvinfo.is_rcv = FALSE;

		if (dbg_can_rcvinfo.ex_canid ==  0xFFFFFFFF) {
			*canid_type = 0;
		}
		else {
			*canid_type = 1;
		}

		printf("\n");
		printf("##RCV_CAN::ch=%u\n", *ch);
		printf("##RCV_CNA:canid=0x%x\n", *canid);
		printf("##RCV_CAN:ex_canid=0x%x\n", *ex_canid);
		printf("##RCV_CNA:dlc=%u\n", *dlc);
		printf("##RCV_CAN:canid_type=%u\n", *canid_type);
		printf("##");
		for (i = 0; i < *dlc; i++) {
			data[i] = dbg_can_rcvinfo.data[i];
			printf("0x%02x ", data[i]);
		}
		printf("\n");
		return TRUE;
	}
	return FALSE;
}
static bool dbg_stdio_can_send(uint32 ch, uint32 can_id, uint8 *data, uint8 dlc, uint8 canid_type)
{
#if 0
	uint8 i;
	printf("\n");
	printf("#########################CAN DEBUG INFO########################\n");
	printf("##ch=%u\n", ch);
	printf("##msg_id=%u\n", can_id);
	printf("##dlc=%u\n", dlc);
	printf("##canid_type=%u\n", canid_type);
	printf("##");
	for (i = 0U; i < dlc; i++) {
		printf("%u:0x%02x ", i, data[i]);
	}
	printf("\n");
	printf("#########################CAN DEBUG INFO########################\n");
	fflush(stdout);
#endif
	return TRUE;
}
