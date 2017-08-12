#include <stdio.h>
#include "dbg_can.h"
#include "udp/udp_comm.h"
#include "cpuemu_ops.h"

static bool dbg_udp_can_init(uint32 ch);
static bool dbg_udp_can_recv(uint32 *channel, uint32 *canid, uint32 *ex_canid, uint8 *data, uint8 *dlc, uint8 *canid_type);
static bool dbg_udp_can_send(uint32 channel, uint32 can_id, uint8 *data, uint8 dlc, uint8 canid_type);

typedef struct{
	uint32 ch;
	uint32 canid;
	uint32 ex_canid;
	uint8 data[8];
	uint8 dlc;
	uint8 canid_type;
}DbgUdpCanDataType;

DeviceCanOpType dbg_can_ops = {
		dbg_udp_can_init,
		dbg_udp_can_recv,
		dbg_udp_can_send,
};

static UdpCommConfigType	dbg_udp_can_config;
static UdpCommType			dbg_udp_can_comm;
static uint32	dbg_udp_can_recv_cunt = 0;
static bool		dbg_udp_can_enabl = FALSE;

static bool dbg_udp_can_init(uint32 ch)
{
	Std_ReturnType err;

	/* サーバーポート番号取得 */
	err = cpuemu_get_devcfg_value("DEVICE_CONFIG_CAN_SERVER_PORTNO", (uint32*)(&(dbg_udp_can_config.server_port)));
	if (err != STD_E_OK) {
		return FALSE;
	}
	/* クライアントポート番号取得 */
	err = cpuemu_get_devcfg_value("DEVICE_CONFIG_CAN_CLIENT_PORTNO", (uint32*)(&(dbg_udp_can_config.client_port)));
	if (err != STD_E_OK) {
		return FALSE;
	}

	/* UDPソケットの生成 */
	err = udp_comm_create(&dbg_udp_can_config, &dbg_udp_can_comm);
	if (err != STD_E_OK) {
		return FALSE;
	}

	dbg_udp_can_enabl = TRUE;
	return TRUE;
}

static bool dbg_udp_can_recv(uint32 *channel, uint32 *canid, uint32 *ex_canid, uint8 *data, uint8 *dlc, uint8 *canid_type)
{
	Std_ReturnType err;
	uint8 i;
	DbgUdpCanDataType *recv_data;

	if (dbg_udp_can_enabl == FALSE) {
		return FALSE;
	}

	if (dbg_udp_can_recv_cunt >= 10000) {
		dbg_udp_can_recv_cunt = 0;
	}
	else {
		dbg_udp_can_recv_cunt++;
		return FALSE;
	}

	/* UDPデータ受信 */
	err = udp_comm_read(&dbg_udp_can_comm);
	if (err != STD_E_OK) {
		return FALSE;
	}

	/* 受信データをDbgUdpCanDataType型にキャスト */
	recv_data = (DbgUdpCanDataType*)&(dbg_udp_can_comm.read_data.buffer);

	/*　受信データ取得 チャネル　*/
	*channel = recv_data->ch;

	/*　受信データ取得 CANID */
	*canid = recv_data->canid;
	*ex_canid = recv_data->canid;

	/*　受信データ取得 データ長　*/
	*dlc = recv_data->dlc;

	/*　受信データ取得 データ */
	for ( i=0; i < recv_data->dlc; i++) {
		data[i] = recv_data->data[i];
	}

	/*　受信データ取得 フォーマットタイプ */
	*canid_type = recv_data->canid_type;

	return TRUE;
}


static bool dbg_udp_can_send(uint32 channel, uint32 can_id, uint8 *data, uint8 dlc, uint8 canid_type)
{
	Std_ReturnType err;
	DbgUdpCanDataType *send_data;
	uint8 i;

	if (dbg_udp_can_enabl == FALSE) {
		return FALSE;
	}

	send_data = (DbgUdpCanDataType*)(dbg_udp_can_comm.write_data.buffer);

	/* 送信データセット チャネル */
	send_data->ch = channel;
	/* 送信データセット CANID */
	send_data->canid = can_id;

	//TODO print
	//printf("UDP_CAN_SEND canid %x\n", send_data->canid);

	/* 送信データセット 送信データ */
	for ( i=0; i < dlc; i++) {
		send_data->data[i] = data[i];
	}

	/* 送信データセット 送信データ長 */
	send_data->dlc = dlc;

	/* 送信データセット フォーマットタイプ */
	send_data->canid_type = canid_type;

	//TODO print
	//printf("SEND CANID_TYPE = %d\n" ,canid_type);

	/* 送信データのサイズをセット */
	dbg_udp_can_comm.write_data.len = sizeof(DbgUdpCanDataType);

	/* データ送信 */
	err = udp_comm_write(&dbg_udp_can_comm);
	if (err != STD_E_OK) {
		return FALSE;
	}

	return TRUE;
}
