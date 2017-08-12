#include "bus.h"

typedef struct {
	BusAccessType		access_type;
	uint32				access_size;
	uint32 				access_addr;
} BusAccessLogType;

uint32			 bus_access_log_size  = 0;
BusAccessLogType bus_access_log[BUS_ACCESS_LOG_SIZE];

void bus_access_set_log(BusAccessType type, uint32 size, uint32 access_addr)
{
	if (type == BUS_ACCESS_TYPE_NONE) {
		bus_access_log_size = 0;
		return;
	}
	else if (bus_access_log_size >= BUS_ACCESS_LOG_SIZE) {
		return;
	}
	bus_access_log[bus_access_log_size].access_type = type;
	bus_access_log[bus_access_log_size].access_size = size;
	bus_access_log[bus_access_log_size].access_addr = access_addr;
	bus_access_log_size++;
	return;
}

Std_ReturnType bus_access_get_log(BusAccessType *type, uint32 *size, uint32 *access_addr)
{
	if (bus_access_log_size == 0) {
		return STD_E_NOENT;
	}
	bus_access_log_size--;
	*type = bus_access_log[bus_access_log_size].access_type;
	*size = bus_access_log[bus_access_log_size].access_size;
	*access_addr = bus_access_log[bus_access_log_size].access_addr;
	return STD_E_OK;
}
