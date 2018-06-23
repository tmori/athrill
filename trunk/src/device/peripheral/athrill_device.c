#include "athrill_device.h"
#include "mpu_ops.h"
#include "symbol_ops.h"
#include <string.h>

static uint32 athrill_device_func_call_addr = 0x0;

void device_init_athrill_device(void)
{
    uint32 addr;
    uint32 size;
    int err;

    err = symbol_get_gl("athrill_device_func_call", 
        strlen("athrill_device_func_call"), &addr, &size);
    if (err < 0) {
        return;
    }
    athrill_device_func_call_addr = addr;
    return;
}
void device_supply_clock_athrill_device(void)
{
    Std_ReturnType err;
    uint32 data;
	char cmd[256];

    if (athrill_device_func_call_addr == 0x0) {
        return;
    }

    err = mpu_get_data32(0U, athrill_device_func_call_addr, &data);
    if (err != 0) {
        return;
    }
    if (data == 0U) {
        return;
    }

	snprintf(cmd, sizeof(cmd), "athrill_extfunc.sh %u", data);
	if (system(cmd) < 0) {
		printf("can not execute athrill_extfunc.sh\n");
	}
    (void)mpu_put_data32(0U, athrill_device_func_call_addr, 0U);

    return;
}
