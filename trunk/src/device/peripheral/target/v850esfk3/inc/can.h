#ifndef _CAN_H_
#define _CAN_H_

#include "device.h"
#include "dbg_can.h"

#define DEVICE_CAN_RCV_EX_CANID_NONE	(0xFFFFFFFFU)
/*
 *  CAN受信完了割込み番号
 */
#define INTNO_INTC1REC	69U

extern void device_can_register_ops(void *can, DeviceCanOpType *ops);


#endif /* _CAN_H_ */
