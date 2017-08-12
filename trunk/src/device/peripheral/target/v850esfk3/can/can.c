#include "inc/can.h"
#include "device.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define CAN_CHANNEL_NUM		5U
#define CAN_MSGBUF_NUM		32U

#define CAN_CHANNEL_ID_1	1U
#define CAN1_BASEADDR		0x03FEC700

static Std_ReturnType can_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data);
static Std_ReturnType can_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data);
static Std_ReturnType can_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data);
static Std_ReturnType can_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data);
static Std_ReturnType can_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data);
static Std_ReturnType can_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data);

MpuAddressRegionOperationType	can_memory_operation = {
		.get_data8 		= 	can_get_data8,
		.get_data16		=	can_get_data16,
		.get_data32		=	can_get_data32,

		.put_data8 		= 	can_put_data8,
		.put_data16		=	can_put_data16,
		.put_data32		=	can_put_data32,

		.get_pointer	= NULL
};

/*
 * TODO
 *
 * 供給クロック周波数を20MHzで，512Kbps通信毒度とした場合，
 * １バイトのデータ転送に要するクロック数を求めた
 *
 * 最終的には設定レジスタの値から導出すべき
 */
#define CAN_DATA_SEND_CLOCKS_PER_BYTE_512KBPS	(306U)


#define CAN_DATA_RCV_CLOCKS_PER_BYTE_512KBPS	(306U)
#define CAN_DATA_RCV_CLOCKS_INTR				(100U)

/*
 * CANモジュール割り込み許可レジスタ（ CnIE）
 */
#define CAN_ADDR_C1IE		0x03FEC656

#define CAN_READ_C1IE_CIE5	5U
#define CAN_READ_C1IE_CIE4	4U
#define CAN_READ_C1IE_CIE3	3U
#define CAN_READ_C1IE_CIE2	2U
#define CAN_READ_C1IE_CIE1	1U
#define CAN_READ_C1IE_CIE0	0U

#define CAN_WRITE_C1IE_CIE5_SET		13U
#define CAN_WRITE_C1IE_CIE4_SET		12U
#define CAN_WRITE_C1IE_CIE3_SET		11U
#define CAN_WRITE_C1IE_CIE2_SET		10U
#define CAN_WRITE_C1IE_CIE1_SET		9U
#define CAN_WRITE_C1IE_CIE0_SET		8U

#define CAN_WRITE_C1IE_CIE5_CLR		5U
#define CAN_WRITE_C1IE_CIE4_CLR		4U
#define CAN_WRITE_C1IE_CIE3_CLR		3U
#define CAN_WRITE_C1IE_CIE2_CLR		2U
#define CAN_WRITE_C1IE_CIE1_CLR		1U
#define CAN_WRITE_C1IE_CIE0_CLR		0U

/*
 * CANモジュール割り込みステータス・レジスタ（ CnINTS）
 */
#define CAN_ADDR_C1INTS		0x03FEC658

#define CAN_READ_C1INTS_CINTS5	5U
#define CAN_READ_C1INTS_CINTS4	4U
#define CAN_READ_C1INTS_CINTS3	3U
#define CAN_READ_C1INTS_CINTS2	2U
#define CAN_READ_C1INTS_CINTS1	1U
#define CAN_READ_C1INTS_CINTS0	0U

#define CAN_WRITE_C1INTS_CINTS5_CLR	5U
#define CAN_WRITE_C1INTS_CINTS4_CLR	4U
#define CAN_WRITE_C1INTS_CINTS3_CLR	3U
#define CAN_WRITE_C1INTS_CINTS2_CLR	2U
#define CAN_WRITE_C1INTS_CINTS1_CLR	1U
#define CAN_WRITE_C1INTS_CINTS0_CLR	0U

/*
 * CANモジュール制御レジスタ（ CnCTRL）
 */
#define CAN_ADDR_C1CTRL		0x03FEC650

#define CAN_READ_C1CTRL_RSTAT		9U
#define CAN_READ_C1CTRL_TSTAT		8U
#define CAN_READ_C1CTRL_CCERC		7U
#define CAN_READ_C1CTRL_AL			6U
#define CAN_READ_C1CTRL_VALID		5U
#define CAN_READ_C1CTRL_PSMODE1		4U
#define CAN_READ_C1CTRL_PSMODE0		3U
#define CAN_READ_C1CTRL_OPMODE2		2U
#define CAN_READ_C1CTRL_OPMODE1		1U
#define CAN_READ_C1CTRL_OPMODE0		0U


#define CAN_WRITE_C1CTRL_CCERC_SET		15U
#define CAN_WRITE_C1CTRL_AL_SET			14U
#define CAN_WRITE_C1CTRL_PSMODE1_SET	12U
#define CAN_WRITE_C1CTRL_PSMODE0_SET	11U
#define CAN_WRITE_C1CTRL_OPMODE2_SET	10U
#define CAN_WRITE_C1CTRL_OPMODE1_SET	9U
#define CAN_WRITE_C1CTRL_OPMODE0_SET	8U
#define CAN_WRITE_C1CTRL_AL_CLR			6U
#define CAN_WRITE_C1CTRL_VALID_CLR		5U
#define CAN_WRITE_C1CTRL_PSMODE1_CLR	4U
#define CAN_WRITE_C1CTRL_PSMODE0_CLR	3U
#define CAN_WRITE_C1CTRL_OPMODE2_CLR	2U
#define CAN_WRITE_C1CTRL_OPMODE1_CLR	1U
#define CAN_WRITE_C1CTRL_OPMODE0_CLR	0U

/*
 * CANメッセージ制御レジスタｍ
 */
#define CAN_ADDR_C1MCTRLm(m)		(CAN1_BASEADDR + ( ((m) * 0x20 ) + 0xE ) )


#define CAN_READ_C1MCTRLm_MUC	13U
#define CAN_READ_C1MCTRLm_MOW	4U
#define CAN_READ_C1MCTRLm_IE	3U
#define CAN_READ_C1MCTRLm_DN	2U
#define CAN_READ_C1MCTRLm_TRQ	1U
#define CAN_READ_C1MCTRLm_RDY	0U


#define CAN_WRITE_C1MCTRLm_IE_SET	11U
#define CAN_WRITE_C1MCTRLm_IE_CLR	3U
#define CAN_WRITE_C1MCTRLm_TRQ_SET	9U
#define CAN_WRITE_C1MCTRLm_TRQ_CLR	1U
#define CAN_WRITE_C1MCTRLm_RDY_SET	8U
#define CAN_WRITE_C1MCTRLm_RDY_CLR	0U
#define CAN_WRITE_C1MCTRLm_MOW_CLR	4U
#define CAN_WRITE_C1MCTRLm_DN_CLR	2U

/*
 * CANメッセージ・コンフィギュレーション・レジスタm
 */
#define CAN_ADDR_C1MCONFm(m)		(CAN1_BASEADDR + ( ((m) * 0x20 ) + 0x9 ) )

#define CAN_MSGBUF_IS_USED(data)			( ((data) & 0x01) != 0x00U )

#define CAN_MSGBUF_TYPE_SND(data)			( ((data) & 0x38) == 0x00U )
#define CAN_MSGBUF_TYPE_RCV_NOMASK(data)	( ((data) & 0x38) == 0x08U )
#define CAN_MSGBUF_TYPE_RCV_MASK1(data)		( ((data) & 0x38) == 0x10U )
#define CAN_MSGBUF_TYPE_RCV_MASK2(data)		( ((data) & 0x38) == 0x18U )
#define CAN_MSGBUF_TYPE_RCV_MASK3(data)		( ((data) & 0x38) == 0x20U )
#define CAN_MSGBUF_TYPE_RCV_MASK4(data)		( ((data) & 0x38) == 0x28U )

#define CAN_MSGBUF_IS_DATA_FRAME(data)		( ((data) & 0x40) == 0x00U )

#define CAN_MSGBUF_CAN_OVER_WRITE(data)		( ((data) & 0x80) != 0x00U )


/*
 * CANメッセージIDレジスタm
 */
#define CAN_ADDR_C1MIDLm(m)		(CAN1_BASEADDR + ( ((m) * 0x20 ) + 0xA ) )
#define CAN_ADDR_C1MIDHm(m)		(CAN1_BASEADDR + ( ((m) * 0x20 ) + 0xC ) )

/*
 * CANメッセージ・データ長レジスタｍ
 */
#define CAN_ADDR_C1MDLCm(m)		(CAN1_BASEADDR + ( ((m) * 0x20 ) + 0x8 ) )

/*
 * CANメッセージ・データ・バイト・レジスタ
 */
#define CAN_ADDR_C1MDATAxm(x, m)		(CAN1_BASEADDR + ( ((m) * 0x20 ) + (x) ) )

typedef enum {
	CANID_TYPE_NORMAL,
	CANID_TYPE_EXTEND
} CanIdType;

typedef struct {
	uint32 rcv_cnt;
	uint32 snd_cnt;
	uint16* ctrl;
	uint8* conf;
	uint8* dlc;
	uint16* idl;
	uint16* idh;
	uint8 *buffer;
	uint32						canid;
	CanIdType					canid_type;
} CanDeviceMsgBufferType;

/*
 *
 */
typedef enum {
	CAN_DEVICE_CHANNEL_STATE_NONE,
	CAN_DEVICE_CHANNEL_STATE_DOING,
	CAN_DEVICE_CHANNEL_STATE_INTR_WAITING,
} CanDeviceChannelStateType;

typedef struct {
	/*
	 * レジスタ情報
	 */
	uint16 						*ie;
	uint16 						*ints;

	CanDeviceChannelStateType 	snd_state;
	uint32 						snd_msgid;
	uint32						snd_wait_time;

	CanDeviceChannelStateType 	rcv_state;
	uint32 						rcv_msgid;
	uint32						rcv_wait_time;
	uint32						rcv_wait_time_intr;
	uint8						rcv_data[8U];
	uint32						rcv_canid;
	uint32						rcv_ex_canid;
	uint8						rcv_dlc;

	CanDeviceMsgBufferType msg[CAN_MSGBUF_NUM];
} CanDeviceChannelType;

typedef struct {
	CanDeviceChannelType channel[CAN_CHANNEL_NUM];
} CanDeviceModuleType;

typedef struct {
	CanDeviceModuleType module;
	DeviceCanOpType *ops;
} CanDeviceType;

static CanDeviceType CanDevice;
static MpuAddressRegionType *can_region;

void device_init_can(MpuAddressRegionType *region)
{
	uint32 msg_id;
	uint32* addr;
	uint32 off;
	bool result;

	can_region = region;

	CanDevice.module.channel[CAN_CHANNEL_ID_1].snd_state = CAN_DEVICE_CHANNEL_STATE_NONE;
	CanDevice.module.channel[CAN_CHANNEL_ID_1].rcv_state = CAN_DEVICE_CHANNEL_STATE_NONE;

	off = CAN_ADDR_C1IE - can_region->start;
	addr = (uint32*)&(can_region->data[off]);
	CanDevice.module.channel[CAN_CHANNEL_ID_1].ie = (uint16*)addr;

	off = CAN_ADDR_C1INTS - can_region->start;
	addr = (uint32*)&(can_region->data[off]);
	CanDevice.module.channel[CAN_CHANNEL_ID_1].ints = (uint16*)addr;


	for (msg_id = 0U; msg_id < CAN_MSGBUF_NUM; msg_id++) {
		CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].snd_cnt = 0U;
		CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].rcv_cnt = 0U;

		off = CAN_ADDR_C1MCTRLm(msg_id) - can_region->start;
		addr = (uint32*)&(can_region->data[off]);
		CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].ctrl = (uint16*)addr;

		off = CAN_ADDR_C1MDATAxm(0, msg_id) - can_region->start;
		addr = (uint32*)&(can_region->data[off]);
		CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].buffer = (uint8*)addr;

		off = CAN_ADDR_C1MCONFm(msg_id) - can_region->start;
		addr = (uint32*)&(can_region->data[off]);
		CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].conf = (uint8*)addr;

		off = CAN_ADDR_C1MDLCm(msg_id) - can_region->start;
		addr = (uint32*)&(can_region->data[off]);
		CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].dlc = (uint8*)addr;

		off = CAN_ADDR_C1MIDLm(msg_id) - can_region->start;
		addr = (uint32*)&(can_region->data[off]);
		CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].idl = (uint16*)addr;

		off = CAN_ADDR_C1MIDHm(msg_id) - can_region->start;
		addr = (uint32*)&(can_region->data[off]);
		CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].idh = (uint16*)addr;
	}

	/*
	 * CAN初期化
	 */
	result = dbg_can_ops.init(CAN_CHANNEL_ID_1);
	if (result == FALSE) {
		//printf("device_init_can:err\n");
	}
	return;
}
static uint32 get_canid(uint32 channel, uint32 msg_id)
{
	uint32 canid;
	/*
	 * 11bit
	 */
	canid = ((*CanDevice.module.channel[channel].msg[msg_id].idh) & 0x1FFC) >> 2U;

	//printf("get_canid=0x%x\n", canid);
	return canid;
}

static uint32 get_ex_canid(uint32 channel, uint32 msg_id)
{
	uint32 ex_canid;

	/*
	 * 1bit
	 */
	if (((*CanDevice.module.channel[channel].msg[msg_id].idh) & 0x8000) == 0x0U) {
		return DEVICE_CAN_RCV_EX_CANID_NONE;
	}

	/*
	 * 29bit
	 */
	ex_canid = ((*CanDevice.module.channel[channel].msg[msg_id].idh) & 0x1FFF) << 16U;
	ex_canid |= ((*CanDevice.module.channel[channel].msg[msg_id].idl));
	//printf("get_excanid=0x%x\n", ex_canid);
	return ex_canid;
}

static uint64 get_priopoint_snd_msg(uint32 channel, uint32 msg_id)
{
	uint64 idh;
	uint64 frame_type;
	uint64 id_type;
	uint64 idl;
	uint64 bufno = msg_id;//5bit
	uint64 point;

	/*
	 * 11bit
	 */
	idh = ((*CanDevice.module.channel[channel].msg[msg_id].idh) & 0x1FFC) >> 2U;

	point = idh;

	//1bit
	if (CAN_MSGBUF_IS_DATA_FRAME(*CanDevice.module.channel[channel].msg[msg_id].conf)) {
		frame_type = 0U;
	}
	else {
		frame_type = 1U;
	}

	point <<= 11U;
	point |= frame_type;

	/*
	 * 1bit
	 */
	if (((*CanDevice.module.channel[channel].msg[msg_id].idh) & 0x8000) == 0x0U) {
		//標準フォーマット
		id_type = 0U;
		CanDevice.module.channel[channel].msg[msg_id].canid_type = CANID_TYPE_NORMAL;
	}
	else {
		//拡張フォーマット
		id_type = 1U;
		CanDevice.module.channel[channel].msg[msg_id].canid_type = CANID_TYPE_EXTEND;
	}
	point <<= 1U;
	point |= id_type;

	/*
	 * 18bit
	 */
	idl = ((*CanDevice.module.channel[channel].msg[msg_id].idh) & 0x0003) << 16U;
	idl |= ((*CanDevice.module.channel[channel].msg[msg_id].idl));


	if (CanDevice.module.channel[channel].msg[msg_id].canid_type == CANID_TYPE_NORMAL) {
		/* ID18-28 */
		CanDevice.module.channel[channel].msg[msg_id].canid = ( (*CanDevice.module.channel[channel].msg[msg_id].idh) & 0x1FFC ) >> 2U;
	}
	else {
		uint32 tmph = ( (*CanDevice.module.channel[channel].msg[msg_id].idh) & 0x1FFF ) ; /* ID16-28 */
		uint32 tmpl = ( (*CanDevice.module.channel[channel].msg[msg_id].idl) & 0xFFFF ) ; /* ID0-15 */
		uint32 tmp = (tmph << 16) | tmpl;
		CanDevice.module.channel[channel].msg[msg_id].canid = tmp;
	}
	//printf("channel=%d msg_id=%d canid=0x%x\n", channel, msg_id,  CanDevice.module.channel[channel].msg[msg_id].canid);

	point <<= 18U;
	point |= idl;

	/*
	 * 5bit
	 */
	point <<= 5U;
	point |= bufno;


	return point;
}

static bool get_highest_prio_snd_msg(uint32 channel, uint32 *msg_idp)
{
	uint32 msg_id;
	uint8 data8;
	uint16 data16;
	uint32 candidate_id = 0U;
	uint32 candidate_num = 0U;
	uint32 candidate_msgid[CAN_MSGBUF_NUM];
	uint64 point;
	uint64 high_point = 0xFFFFFFFFFFFFFFFF;
	uint32 high_msgid = 0U;

	/*
	 * TRQビットがセット （ 1） されている送信バッファが存在するかチェックする
	 */
	for (msg_id = 0U; msg_id < CAN_MSGBUF_NUM; msg_id++) {
		data8 = *CanDevice.module.channel[channel].msg[msg_id].conf;
		if (!CAN_MSGBUF_IS_USED(data8)) {
			continue;
		}
		if (!CAN_MSGBUF_TYPE_SND(data8)) {
			continue;
		}
		data16 = *CanDevice.module.channel[channel].msg[msg_id].ctrl;
		if ((data16 & (1U << CAN_READ_C1MCTRLm_TRQ)) == 0U) {
			continue;
		}
		if ((data16 & (1U << CAN_READ_C1MCTRLm_RDY)) == 0U) {
			continue;
		}
		//printf("msg_id=%u data8=0x%x\n", msg_id, data8);
		candidate_msgid[candidate_num] = msg_id;
		candidate_num++;
	}

	if (candidate_num == 0U) {
		return FALSE;
	}

	/*
	 * 最高優先度の送信バッファを探す
	 */
	for (candidate_id = 0U; candidate_id < candidate_num; candidate_id++) {
		point = get_priopoint_snd_msg(channel, candidate_msgid[candidate_id]);
		//printf("point=0x%llx high_point=0x%llx\n", point, high_point);
		if (point < high_point) {
			high_point = point;
			high_msgid = candidate_msgid[candidate_id];
		}
	}

	*msg_idp = high_msgid;
	return TRUE;
}

static void send_can_data(uint32 channel,  uint32 msg_id)
{
	uint8 dlc = *CanDevice.module.channel[channel].msg[msg_id].dlc;
	uint32 canid = CanDevice.module.channel[channel].msg[msg_id].canid;
	uint8 canid_type = (uint8)CanDevice.module.channel[channel].msg[msg_id].canid_type;
	
	(void)CanDevice.ops->send(channel, canid, CanDevice.module.channel[channel].msg[msg_id].buffer, dlc, canid_type);

	//TRQ clr
	*(CanDevice.module.channel[channel].msg[msg_id].ctrl) &= ~(1U << CAN_READ_C1MCTRLm_TRQ);

	return;
}

static void device_supply_clock_can_snd(DeviceClockType *dev_clock)
{
	uint32 msg_id;

	//送信処理
	if (CanDevice.module.channel[CAN_CHANNEL_ID_1].snd_state == CAN_DEVICE_CHANNEL_STATE_DOING) {
		msg_id = CanDevice.module.channel[CAN_CHANNEL_ID_1].snd_msgid;
		CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].snd_cnt++;

		if (CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].snd_cnt >= CanDevice.module.channel[CAN_CHANNEL_ID_1].snd_wait_time) {
			send_can_data(CAN_CHANNEL_ID_1, msg_id);
			CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].snd_cnt = 0U;
			CanDevice.module.channel[CAN_CHANNEL_ID_1].snd_state = CAN_DEVICE_CHANNEL_STATE_NONE;
		}
		return;
	}

	//送信メッセージバッファ選択
	if (get_highest_prio_snd_msg(CAN_CHANNEL_ID_1, &msg_id) == FALSE) {
		return;
	}

	//CANデータ送信開始
	CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].snd_cnt = 0U;
	CanDevice.module.channel[CAN_CHANNEL_ID_1].snd_msgid = msg_id;

	//TODO 送信速度設定
	CanDevice.module.channel[CAN_CHANNEL_ID_1].snd_wait_time = CAN_DATA_SEND_CLOCKS_PER_BYTE_512KBPS * (*CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].dlc);
	CanDevice.module.channel[CAN_CHANNEL_ID_1].snd_state = CAN_DEVICE_CHANNEL_STATE_DOING;
	return;
}
static void recv_can_data_start(uint32 channel,  uint32 msg_id)
{
	//printf("recv_can_data_start:ch=%u msg_id=%u\n", channel, msg_id);
	//set DN
	//set MUC
	*(CanDevice.module.channel[channel].msg[msg_id].ctrl) |= (1U << CAN_READ_C1MCTRLm_DN);
	*(CanDevice.module.channel[channel].msg[msg_id].ctrl) |= (1U << CAN_READ_C1MCTRLm_MUC);

	return;
}
static void recv_can_data_end(uint32 channel,  uint32 msg_id)
{
	uint8 i;
	//printf("recv_can_data_end\n");

	//set DN
	//clear MUC
	*(CanDevice.module.channel[channel].msg[msg_id].ctrl) |= (1U << CAN_READ_C1MCTRLm_DN);
	*(CanDevice.module.channel[channel].msg[msg_id].ctrl) &= ~(1U << CAN_READ_C1MCTRLm_MUC);

	//DLC copy
	*(CanDevice.module.channel[channel].msg[msg_id].dlc) = CanDevice.module.channel[channel].rcv_dlc;

	//DATA copy
	for (i = 0U; i < CanDevice.module.channel[channel].rcv_dlc; i++) {
		CanDevice.module.channel[channel].msg[msg_id].buffer[i] = CanDevice.module.channel[channel].rcv_data[i];
		//printf("DATA copy data[%d] = %d\n", i, CanDevice.module.channel[channel].msg[msg_id].buffer[i]);
	}

	return;
}

static void recv_can_data_intr(uint32 channel,  uint32 msg_id)
{
	uint16 data;
	//printf("recv_can_data_intr:enter\n");


	data = *CanDevice.module.channel[channel].ie;
	if ((data & (1U << CAN_READ_C1IE_CIE1)) == 0U) {
		//printf("recv_can_data_intr:exit:not C1IE ie set\n");
		return;
	}
	data = *CanDevice.module.channel[channel].msg[msg_id].ctrl;
	if ((data & (1U << CAN_READ_C1MCTRLm_IE)) == 0U) {
		//printf("recv_can_data_intr:exit:not C1MCTRLm ie set\n");
		return;
	}


	//printf("recv_can_data_intr\n");

	//set CINTS1
	*(CanDevice.module.channel[channel].ints) |= (1U << CAN_READ_C1INTS_CINTS1);

	//受信割込みを上げる
	device_raise_int(INTNO_INTC1REC);

	return;
}
/*
 * マスクは未サポート
 */
static bool get_highest_prio_rcv_msg(uint32 channel, uint32 canid, uint32 ex_canid, uint8 dlc, CanIdType canid_type, uint32 *msg_idp)
{
	uint32 msg_id;
	uint8 data8;
	uint16 data16;
	uint32 candidate_id = 0U;
	uint32 candidate_num = 0U;
	uint32 candidate_msgid[CAN_MSGBUF_NUM];

	/*
	 * RDYビットがセット （ 1） されている受信バッファが存在するかチェックする
	 */
	for (msg_id = 0U; msg_id < CAN_MSGBUF_NUM; msg_id++) {
		data8 = *CanDevice.module.channel[channel].msg[msg_id].conf;
		if (!CAN_MSGBUF_IS_USED(data8)) {
			continue;
		}
		if (!CAN_MSGBUF_TYPE_RCV_NOMASK(data8)) {
			continue;
		}
		data16 = *CanDevice.module.channel[channel].msg[msg_id].ctrl;
		if ((data16 & (1U << CAN_READ_C1MCTRLm_RDY)) == 0U) {
			continue;
		}
		if ((data16 & (1U << CAN_READ_C1MCTRLm_DN)) == 0U) {
			//printf("msg_id=%u data8=0x%x\n", msg_id, data8);
			candidate_msgid[candidate_num] = msg_id;
			candidate_num++;
		}
		else if (CAN_MSGBUF_CAN_OVER_WRITE(data8)) {
			//printf("msg_id=%u data8=0x%x\n", msg_id, data8);
			candidate_msgid[candidate_num] = msg_id;
			candidate_num++;
		}
	}

	//printf("canid=0x%x candidate_num=%u\n", canid, candidate_num);
	if (candidate_num == 0U) {
		return FALSE;
	}

	/*
	 * 最高優先度の受信バッファを探す
	 */
	for (candidate_id = 0U; candidate_id < candidate_num; candidate_id++) {

		if (canid_type == CANID_TYPE_NORMAL) {
			if (get_canid(channel, candidate_msgid[candidate_id]) != canid) {
				continue;
			}
		}
		else {
			if (get_ex_canid(channel, candidate_msgid[candidate_id]) != ex_canid) {
				continue;
			}
		}
		*msg_idp = candidate_msgid[candidate_id];
		return TRUE;
	}
	return FALSE;
}
/*
 * 複数のメッセージ同時受信は未サポート
 */
static void device_supply_clock_can_rcv(DeviceClockType *dev_clock)
{
	uint32 msg_id;
	bool has_recv = FALSE;
	bool has_msgid = FALSE;
	uint32 channel;
	uint32 canid;
	uint32 ex_canid;
	uint8 data[8U];
	uint8 dlc;
	uint8 canid_type;
	uint8 i;


	//CANデータ受信処理
	if (CanDevice.module.channel[CAN_CHANNEL_ID_1].rcv_state == CAN_DEVICE_CHANNEL_STATE_DOING) {
		msg_id = CanDevice.module.channel[CAN_CHANNEL_ID_1].rcv_msgid;
		CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].rcv_cnt++;

		if (CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].rcv_cnt >= CanDevice.module.channel[CAN_CHANNEL_ID_1].rcv_wait_time) {
			recv_can_data_end(CAN_CHANNEL_ID_1, msg_id);

			CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].rcv_cnt = 0U;
			CanDevice.module.channel[CAN_CHANNEL_ID_1].rcv_state = CAN_DEVICE_CHANNEL_STATE_INTR_WAITING;
			CanDevice.module.channel[CAN_CHANNEL_ID_1].rcv_wait_time = CAN_DATA_RCV_CLOCKS_INTR;
		}


		return;
	}
	else if (CanDevice.module.channel[CAN_CHANNEL_ID_1].rcv_state == CAN_DEVICE_CHANNEL_STATE_INTR_WAITING) {
		msg_id = CanDevice.module.channel[CAN_CHANNEL_ID_1].rcv_msgid;
		CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].rcv_cnt++;

		if (CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].rcv_cnt >= CanDevice.module.channel[CAN_CHANNEL_ID_1].rcv_wait_time) {
			CanDevice.module.channel[CAN_CHANNEL_ID_1].msg[msg_id].rcv_cnt = 0U;
			CanDevice.module.channel[CAN_CHANNEL_ID_1].rcv_state = CAN_DEVICE_CHANNEL_STATE_NONE;
			CanDevice.module.channel[CAN_CHANNEL_ID_1].rcv_wait_time = 0U;

			recv_can_data_intr(CAN_CHANNEL_ID_1, msg_id);
		}

		return;
	}

	//CANデータ受信確認
	has_recv = CanDevice.ops->recv(&channel, &canid, &ex_canid, data, &dlc, &canid_type);
	if (has_recv == FALSE) {
		return;
	}
	CanDevice.module.channel[channel].rcv_canid = canid;
	CanDevice.module.channel[channel].rcv_ex_canid = ex_canid;
	CanDevice.module.channel[channel].rcv_dlc = dlc;
	for (i = 0U; i < dlc; i++) {
		CanDevice.module.channel[channel].rcv_data[i] = data[i];
	}


	//受信メッセージバッファ選択
	has_msgid = get_highest_prio_rcv_msg(channel, canid, ex_canid, dlc, (CanIdType)canid_type, &msg_id);
	if (has_msgid == FALSE) {
		//printf("device_supply_clock_can_rcv:has_msgid==FALSE:NOP\n");
		return;
	}

	//CANデータ受信開始
	CanDevice.module.channel[channel].msg[msg_id].rcv_cnt = 0U;
	CanDevice.module.channel[channel].rcv_msgid = msg_id;

	//受信速度設定
	CanDevice.module.channel[channel].rcv_wait_time = (CAN_DATA_RCV_CLOCKS_PER_BYTE_512KBPS * dlc);
	CanDevice.module.channel[channel].rcv_state = CAN_DEVICE_CHANNEL_STATE_DOING;

	recv_can_data_start(channel, msg_id);
	return;
}

void device_supply_clock_can(DeviceClockType *dev_clock)
{
	/*
	 * 送信処理
	 */
	device_supply_clock_can_snd(dev_clock);

	/*
	 * 受信処理
	 */
	device_supply_clock_can_rcv(dev_clock);

	return;
}


static void can_set_c1mctrlm(uint16 wdata, uint16 *rdata)
{
	/*
	 * Clear MOW
	 */
	if ((wdata & (1U << CAN_WRITE_C1MCTRLm_MOW_CLR)) != 0U) {
		*rdata &= ~(1U << CAN_READ_C1MCTRLm_MOW);
	}

	/*
	 * IE
	 */
	if (((wdata & (1U << CAN_WRITE_C1MCTRLm_IE_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1MCTRLm_IE_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1MCTRLm_IE);
	}
	else if (((wdata & (1U << CAN_WRITE_C1MCTRLm_IE_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1MCTRLm_IE_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1MCTRLm_IE);
	}
	else {
		/* nop */
	}

	/*
	 * clear DN
	 */
	if ((wdata & (1U << CAN_WRITE_C1MCTRLm_DN_CLR)) != 0U) {
		*rdata &= ~(1U << CAN_READ_C1MCTRLm_DN);
	}

	/*
	 * TRQ
	 */
	if (((wdata & (1U << CAN_WRITE_C1MCTRLm_TRQ_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1MCTRLm_TRQ_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1MCTRLm_TRQ);
	}
	else if (((wdata & (1U << CAN_WRITE_C1MCTRLm_TRQ_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1MCTRLm_TRQ_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1MCTRLm_TRQ);
	}
	else {
		/* nop */
	}

	/*
	 * RDY
	 */
	if (((wdata & (1U << CAN_WRITE_C1MCTRLm_RDY_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1MCTRLm_RDY_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1MCTRLm_RDY);
	}
	else if (((wdata & (1U << CAN_WRITE_C1MCTRLm_RDY_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1MCTRLm_RDY_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1MCTRLm_RDY);
	}
	else {
		/* nop */
	}

	return;
}
static void can_set_c1ctrl(uint16 wdata, uint16 *rdata)
{
	/*
	 * CCERC
	 */
	if ((wdata & (1U << CAN_WRITE_C1CTRL_CCERC_SET)) != 0U) {
		*rdata |= (1U << CAN_READ_C1CTRL_CCERC);
	}
	/*
	 * AL
	 */
	if (((wdata & (1U << CAN_WRITE_C1CTRL_AL_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1CTRL_AL_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1CTRL_AL);
	}
	else if (((wdata & (1U << CAN_WRITE_C1CTRL_AL_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1CTRL_AL_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1CTRL_AL);
	}
	else {
		/* nop */
	}
	/*
	 * VALID
	 */
	if ((wdata & (1U << CAN_WRITE_C1CTRL_VALID_CLR)) != 0U) {
		*rdata &= ~(1U << CAN_READ_C1CTRL_VALID);
	}
	/*
	 * PSMODE0
	 */
	if (((wdata & (1U << CAN_WRITE_C1CTRL_PSMODE0_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1CTRL_PSMODE0_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1CTRL_PSMODE0);
	}
	else if (((wdata & (1U << CAN_WRITE_C1CTRL_PSMODE0_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1CTRL_PSMODE0_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1CTRL_PSMODE0);
	}
	else {
		/* nop */
	}

	/*
	 * PSMODE1
	 */
	if (((wdata & (1U << CAN_WRITE_C1CTRL_PSMODE1_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1CTRL_PSMODE1_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1CTRL_PSMODE1);
	}
	else if (((wdata & (1U << CAN_WRITE_C1CTRL_PSMODE1_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1CTRL_PSMODE1_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1CTRL_PSMODE1);
	}
	else {
		/* nop */
	}
	/*
	 * OPMODE0
	 */
	if (((wdata & (1U << CAN_WRITE_C1CTRL_OPMODE0_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1CTRL_OPMODE0_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1CTRL_OPMODE0);
	}
	else if (((wdata & (1U << CAN_WRITE_C1CTRL_OPMODE0_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1CTRL_OPMODE0_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1CTRL_OPMODE0);
	}
	else {
		/* nop */
	}
	/*
	 * OPMODE1
	 */
	if (((wdata & (1U << CAN_WRITE_C1CTRL_OPMODE1_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1CTRL_OPMODE1_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1CTRL_OPMODE1);
	}
	else if (((wdata & (1U << CAN_WRITE_C1CTRL_OPMODE1_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1CTRL_OPMODE1_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1CTRL_OPMODE1);
	}
	else {
		/* nop */
	}

	/*
	 * OPMODE2
	 */
	if (((wdata & (1U << CAN_WRITE_C1CTRL_OPMODE2_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1CTRL_OPMODE2_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1CTRL_OPMODE2);
	}
	else if (((wdata & (1U << CAN_WRITE_C1CTRL_OPMODE2_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1CTRL_OPMODE2_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1CTRL_OPMODE2);
	}
	else {
		/* nop */
	}

	return;
}
static void can_set_c1ie(uint16 wdata, uint16 *rdata)
{
	/*
	 * CIE0
	 */
	if (((wdata & (1U << CAN_WRITE_C1IE_CIE0_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1IE_CIE0_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1IE_CIE0);
	}
	else if (((wdata & (1U << CAN_WRITE_C1IE_CIE0_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1IE_CIE0_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1IE_CIE0);
	}
	else {
		/* nop */
	}

	/*
	 * CIE1
	 */
	if (((wdata & (1U << CAN_WRITE_C1IE_CIE1_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1IE_CIE1_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1IE_CIE1);
	}
	else if (((wdata & (1U << CAN_WRITE_C1IE_CIE1_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1IE_CIE1_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1IE_CIE1);
	}
	else {
		/* nop */
	}
	/*
	 * CIE2
	 */
	if (((wdata & (1U << CAN_WRITE_C1IE_CIE2_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1IE_CIE2_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1IE_CIE2);
	}
	else if (((wdata & (1U << CAN_WRITE_C1IE_CIE2_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1IE_CIE2_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1IE_CIE2);
	}
	else {
		/* nop */
	}
	/*
	 * CIE3
	 */
	if (((wdata & (1U << CAN_WRITE_C1IE_CIE3_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1IE_CIE3_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1IE_CIE3);
	}
	else if (((wdata & (1U << CAN_WRITE_C1IE_CIE3_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1IE_CIE3_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1IE_CIE3);
	}
	else {
		/* nop */
	}
	/*
	 * CIE4
	 */
	if (((wdata & (1U << CAN_WRITE_C1IE_CIE4_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1IE_CIE4_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1IE_CIE4);
	}
	else if (((wdata & (1U << CAN_WRITE_C1IE_CIE4_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1IE_CIE4_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1IE_CIE4);
	}
	else {
		/* nop */
	}
	/*
	 * CIE5
	 */
	if (((wdata & (1U << CAN_WRITE_C1IE_CIE5_SET)) == 0U) && ((wdata & (1U << CAN_WRITE_C1IE_CIE5_CLR)) != 0U)) {
		*rdata &= ~(1U << CAN_READ_C1IE_CIE5);
	}
	else if (((wdata & (1U << CAN_WRITE_C1IE_CIE5_SET)) != 0U) && ((wdata & (1U << CAN_WRITE_C1IE_CIE5_CLR)) == 0U)) {
		*rdata |= (1U << CAN_READ_C1IE_CIE5);
	}
	else {
		/* nop */
	}


	return;
}
static void can_set_c1ints(uint16 wdata, uint16 *rdata)
{
	if ((wdata & (1U << CAN_WRITE_C1INTS_CINTS0_CLR)) != 0U) {
		*rdata &= ~(1U << CAN_READ_C1INTS_CINTS0);
	}
	if ((wdata & (1U << CAN_WRITE_C1INTS_CINTS1_CLR)) != 0U) {
		*rdata &= ~(1U << CAN_READ_C1INTS_CINTS1);
	}
	if ((wdata & (1U << CAN_WRITE_C1INTS_CINTS2_CLR)) != 0U) {
		*rdata &= ~(1U << CAN_READ_C1INTS_CINTS2);
	}
	if ((wdata & (1U << CAN_WRITE_C1INTS_CINTS3_CLR)) != 0U) {
		*rdata &= ~(1U << CAN_READ_C1INTS_CINTS3);
	}
	if ((wdata & (1U << CAN_WRITE_C1INTS_CINTS4_CLR)) != 0U) {
		*rdata &= ~(1U << CAN_READ_C1INTS_CINTS4);
	}
	if ((wdata & (1U << CAN_WRITE_C1INTS_CINTS5_CLR)) != 0U) {
		*rdata &= ~(1U << CAN_READ_C1INTS_CINTS5);
	}
	return;
}

static bool can_hook_update_reg16(uint32 regaddr, uint16 data)
{
	uint32 msg_id;
	uint32 off;
	uint16 *rdata;

	/*
	 * C1IE check
	 */
	if (regaddr == CAN_ADDR_C1IE) {
		off = regaddr - can_region->start;

		rdata = (uint16*)&can_region->data[off];
		can_set_c1ie(data, rdata);
		//printf("########### CAN_ADDR_C1IE=0x%x #############\n", *rdata);
		return TRUE;
	}
	/*
	 * C1INTS check
	 */
	if (regaddr == CAN_ADDR_C1INTS) {
		off = regaddr - can_region->start;

		rdata = (uint16*)&can_region->data[off];
		can_set_c1ints(data, rdata);
		return TRUE;
	}

	/*
	 * C1CTRL check
	 */
	if (regaddr == CAN_ADDR_C1CTRL) {
		off = regaddr - can_region->start;

		rdata = (uint16*)&can_region->data[off];
		can_set_c1ctrl(data, rdata);
		//printf("########### CAN_ADDR_C1CTRL=0x%x #############\n", *rdata);
		return TRUE;
	}

	/*
	 * C1MCTRLm check
	 */
	for (msg_id = 0U; msg_id < CAN_MSGBUF_NUM; msg_id++) {
		if (regaddr == CAN_ADDR_C1MCTRLm(msg_id)) {
			off = regaddr - can_region->start;

			rdata = (uint16*)&can_region->data[off];
			can_set_c1mctrlm(data, rdata);
			//printf("########### CAN_ADDR_C1MCTRL%d=0x%x #############\n", msg_id, *rdata);
			return TRUE;
		}
	}

	return FALSE;

}

void device_can_register_ops(void *can, DeviceCanOpType *ops)
{
	CanDevice.ops = ops;
	return;
}


static Std_ReturnType can_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint8*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType can_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint16*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType can_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint32*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType can_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data)
{
	uint32 off = (addr - region->start);
	*((uint8*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType can_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - region->start);
	if (can_hook_update_reg16(addr, data) == FALSE) {
		*((uint16*)(&region->data[off])) = data;
	}
	return STD_E_OK;
}
static Std_ReturnType can_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	uint32 off = (addr - region->start);
	*((uint32*)(&region->data[off])) = data;
	return STD_E_OK;
}

