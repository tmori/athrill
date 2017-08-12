#ifndef _DBG_CAN_H_
#define _DBG_CAN_H_

#include "device.h"
#include "dbg_log.h"

typedef struct {
	bool (*init) (uint32 ch);
	bool (*recv) (uint32 *ch, uint32 *canid,uint32 *ex_canid, uint8 *data, uint8 *dlc, uint8 *canid_type);
	bool (*send) (uint32 ch, uint32 can_id, uint8 *data, uint8 dlc, uint8 canid_type);
} DeviceCanOpType;

extern DeviceCanOpType dbg_can_ops;

#endif /* _DBG_CAN_H_ */
