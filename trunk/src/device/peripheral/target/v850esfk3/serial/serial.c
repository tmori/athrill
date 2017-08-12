#include "inc/serial.h"
#include "device.h"
#include "std_errno.h"
#include "mpu_types.h"
#include "device_ex_serial_ops.h"
#include <stdio.h>

typedef struct {
	uint16 id;
	uint16 fd;
	uint32 last_raised_counter;
	uint32 count;
	uint32 bitrate;
	uint32 count_base;
	bool   is_send_data;
	uint8 send_data;
	DeviceExSerialOpType *ops;
} SerialDeviceType;

static SerialDeviceType SerialDevice[UDnChannelNum];
static void serial_set_str(bool enable);
static bool serial_isset_str_ssf(void);
static void serial_set_str_ssf(void);

static Std_ReturnType serial_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data);
static Std_ReturnType serial_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data);
static Std_ReturnType serial_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data);
static Std_ReturnType serial_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data);
static Std_ReturnType serial_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data);
static Std_ReturnType serial_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data);
static Std_ReturnType serial_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data);

MpuAddressRegionOperationType	serial_memory_operation = {
		.get_data8 		= 	serial_get_data8,
		.get_data16		=	serial_get_data16,
		.get_data32		=	serial_get_data32,

		.put_data8 		= 	serial_put_data8,
		.put_data16		=	serial_put_data16,
		.put_data32		=	serial_put_data32,

		.get_pointer	= serial_get_pointer,
};


#define CLOCK_PER_SEC	10000000U	/* 10MHz */
static MpuAddressRegionType *serial_region;

void device_init_serial(MpuAddressRegionType *region)
{
	int i = 0;

	for (i = 0; i < UDnChannelNum; i++) {
		SerialDevice[i].id = i;
//		SerialDevice[i].fd = 2;
		SerialDevice[i].fd = 1;
		SerialDevice[i].is_send_data = FALSE;
		SerialDevice[i].count = 0;
		SerialDevice[i].bitrate = 38400; /* bit/sec */
//		SerialDevice[i].count_base = CLOCK_PER_SEC / (SerialDevice[i].bitrate / 8);
		SerialDevice[i].count_base = 1;
		SerialDevice[i].ops = NULL;

		SerialDevice[i].last_raised_counter = 0;
	}
	serial_region = region;

	return;
}
void device_do_serial(SerialDeviceType *serial)
{
	uint8 data;
	bool ret;
	serial->count++;

	if (serial->ops == NULL) {
		return;
	}
	if ((serial->count % serial->count_base) != 0) {
		return;
	}
	/*
	 * 受信データチェック：存在している場合は，割り込みを上げる．
	 */
	if (serial_isset_str_ssf() == FALSE) {
		if (serial->last_raised_counter == 0U) {
			ret = serial->ops->getchar(serial->id, &data);
			if (ret == TRUE) {
				serial_set_str_ssf();
				//受信データをセットする．
				(void)serial_put_data8(serial_region, CPU_CONFIG_CORE_ID_0, (UDnRX(serial->id) & serial_region->mask), data);
				//受信割込みを上げる
				//printf("serial interrupt:%c\n", data);
				device_raise_int(INTNO_INTUD0R);
				serial->last_raised_counter = 1000U;
			}
		}
		else {
			serial->last_raised_counter--;
		}
	}

	/*
	 * 送信データチェック：存在している場合は，データ転送する．
	 */
	if (serial->is_send_data) {
		//送信割込みを上げる
		//TODO 送信割り込みｋを上げるとサンプルプログラムがエラー終了してしまうため，一旦，コメントアウトした．
		serial_set_str(FALSE);
		//device_raise_int(INTNO_INTUD0T);
		serial->is_send_data = FALSE;
	}


	return;
}

void device_supply_clock_serial(DeviceClockType *dev_clock)
{
#if 1
	device_do_serial(&SerialDevice[0U]);
#else
	int i = 0;

	for (i = 0; i < UDnChannelNum; i++) {
		if ((dev_clock->clock % SerialDevice[i].fd) != 0) {
			continue;
		}
		device_do_serial(&SerialDevice[i]);
	}
	return;
#endif
}


void device_ex_serial_register_ops(uint8 channel, DeviceExSerialOpType *ops)
{
	SerialDevice[channel].ops = ops;
	return;
}

static void serial_set_str(bool enable)
{
	uint8 str;
	(void)serial_get_data8(serial_region, CPU_CONFIG_CORE_ID_0, (UDnSTR(UDnCH0) & serial_region->mask), &str);
	if (enable) {
		str |= 0x80;
	}
	else {
		str &= ~0x80;
	}
	(void)serial_put_data8(serial_region, CPU_CONFIG_CORE_ID_0, (UDnSTR(UDnCH0) & serial_region->mask), str);
	return;
}
static bool serial_isset_str_ssf(void)
{
	uint8 str;
	(void)serial_get_data8(serial_region, CPU_CONFIG_CORE_ID_0, (UDnSTR(UDnCH0) & serial_region->mask), &str);
	return ((str & 0x10) == 0x10);
}
static void serial_set_str_ssf(void)
{
	uint8 str;
	(void)serial_get_data8(serial_region, CPU_CONFIG_CORE_ID_0, (UDnSTR(UDnCH0) & serial_region->mask), &str);
	str |= 0x10;
	(void)serial_put_data8(serial_region, CPU_CONFIG_CORE_ID_0, (UDnSTR(UDnCH0) & serial_region->mask), str);
	//printf("str=0x%x\n", str);
	return;
}

static Std_ReturnType serial_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint8*)(&region->data[off]));
	//if (addr ==  (UDnSTR(UDnCH0) & serial_region->mask))
	//	printf("serial_get_data8:str=0x%x\n", *data);
	return STD_E_OK;
}
static Std_ReturnType serial_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint16*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType serial_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint32*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType serial_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data)
{
	uint32 off = (addr - region->start);
	*((uint8*)(&region->data[off])) = data;

	//if (addr ==  (UDnSTR(UDnCH0) & serial_region->mask))
	//	printf("serial_put_data8:addr=0x%x str=0x%x\n", addr, data);

	if (addr == (UDnTX(UDnCH0) & region->mask)) {
		(void)SerialDevice[UDnCH0].ops->putchar(SerialDevice[UDnCH0].id, data);
		SerialDevice[UDnCH0].is_send_data = TRUE;
		//SerialDevice[UDnCH0].send_data = data;
		serial_set_str(TRUE);
		//printf("%c", data);
		//fflush(stdout);
	}
	return STD_E_OK;
}
static Std_ReturnType serial_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - region->start);
	*((uint16*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType serial_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	uint32 off = (addr - region->start);
	*((uint32*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType serial_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data)
{
	uint32 off = (addr - region->start);
	*data = &region->data[off];
	return STD_E_OK;
}

