#ifndef _DEVICE_IO_H_
#define _DEVICE_IO_H_

#include "types.h"

static inline void sil_wrb_mem(volatile uint8 *addr, uint8 data)
{
	(*(uint8*)addr) = data;
	return;
}
static inline uint8 sil_reb_mem(volatile uint8 *addr)
{
	return (*(uint8*)addr);
}

static inline void sil_wrh_mem(volatile uint16 *addr, uint16 data)
{
	(*(uint16*)addr) = data;
	return;
}
static inline uint16 sil_reh_mem(volatile uint16 *addr)
{
	return (*(uint16*)addr);
}

#endif /* _DEVICE_IO_H_ */