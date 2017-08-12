#ifndef _BUS_H_
#define _BUS_H_

#include "std_types.h"
#include "std_errno.h"
#include "mpu_ops.h"
#include <stdio.h>

typedef enum {
	BUS_ACCESS_TYPE_NONE = 0,
	BUS_ACCESS_TYPE_READ,
	BUS_ACCESS_TYPE_WRITE,
} BusAccessType;
#define BUS_ACCESS_LOG_SIZE	128

extern void bus_access_set_log(BusAccessType type, uint32 size, uint32 access_addr);
extern Std_ReturnType bus_access_get_log(BusAccessType *type, uint32 *size, uint32 *access_addr);

/*
 * データ取得するための操作関数群
 */
static inline Std_ReturnType bus_get_data8(CoreIdType core_id, uint32 addr, uint8 *data)
{
	Std_ReturnType err;

	bus_access_set_log(BUS_ACCESS_TYPE_READ, 1U, addr);
	err = mpu_get_data8(core_id, addr, data);
	if (err != STD_E_OK) {
		printf("ERROR:can not load data:addr=0x%x size=4byte\n", addr);
	}
	return err;
}
static inline Std_ReturnType bus_get_data16(CoreIdType core_id, uint32 addr, uint16 *data)
{
	Std_ReturnType err;

	bus_access_set_log(BUS_ACCESS_TYPE_READ, 2U, addr);
	err = mpu_get_data16(core_id, addr, data);
	if (err != STD_E_OK) {
		printf("ERROR:can not load data:addr=0x%x size=4byte\n", addr);
	}
	return err;
}
static inline Std_ReturnType bus_get_data32(CoreIdType core_id, uint32 addr, uint32 *data)
{
	Std_ReturnType err;

	bus_access_set_log(BUS_ACCESS_TYPE_READ, 4U, addr);
	err = mpu_get_data32(core_id, addr, data);
	if (err != STD_E_OK) {
		printf("ERROR:can not load data:addr=0x%x size=4byte\n", addr);
	}
	return err;
}

/*
 * データ転送するための操作関数群
 */
static inline Std_ReturnType bus_put_data8(CoreIdType core_id, uint32 addr, uint8 data)
{
	bus_access_set_log(BUS_ACCESS_TYPE_WRITE, 1U, addr);
	return mpu_put_data8(core_id, addr, data);
}
static inline Std_ReturnType bus_put_data16(CoreIdType core_id, uint32 addr, uint16 data)
{
	bus_access_set_log(BUS_ACCESS_TYPE_WRITE, 2U, addr);
	return mpu_put_data16(core_id, addr, data);
}
static inline Std_ReturnType bus_put_data32(CoreIdType core_id, uint32 addr, uint32 data)
{
	bus_access_set_log(BUS_ACCESS_TYPE_WRITE, 4U, addr);
	return mpu_put_data32(core_id, addr, data);
}

/*
 * 注意：本APIは基本的には使用しないこと．
 *
 * 性能向上を目的として使用する場合は，
 * ROM/RAM領域のメモリに限り，当該領域のポインタ変数を本関数を使用して取得する．
 */
static inline Std_ReturnType bus_get_pointer(CoreIdType core_id, uint32 addr, uint8 **data)
{
	return mpu_get_pointer(core_id, addr, data);
}

#endif /* _BUS_H_ */
