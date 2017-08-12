#ifndef _DEVICE_EX_SERIAL_OPS_H_
#define _DEVICE_EX_SERIAL_OPS_H_

typedef struct {
	/*
	 * シリアル出力をCPUエミュレータから外部に出力する．
	 */
	bool (*putchar) (uint8 channel, uint8 data);
	/*
	 * シリアル入力をCPUエミュレータの外部から入力する．
	 */
	bool (*getchar) (uint8 channel, uint8 *data);
} DeviceExSerialOpType;
extern void device_ex_serial_register_ops(uint8 channel, DeviceExSerialOpType *ops);

#endif /* _DEVICE_EX_SERIAL_OPS_H_ */
