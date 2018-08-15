#include "kernel_service.h"
#include "test_serial.h"

#define KERNEL_TEST_DATA_NUM    5U
SrvUint32 kernel_test_data[KERNEL_TEST_DATA_NUM];
#define KERNEL_SERVICE_CALL_NUM 3U

ServiceReturnType kernel_get_data(SrvUint32 inx, SrvUint32 *ret_data)
{
    if (inx >= KERNEL_TEST_DATA_NUM) {
        return SERVICE_E_ERROR;
    }
    else if (ret_data == NULL) {
        return SERVICE_E_ERROR;
    }
    *ret_data = kernel_test_data[inx];
    return SERVICE_E_OK;
}

ServiceReturnType kernel_set_data(SrvUint32 inx, SrvUint32 data)
{
    if (inx >= KERNEL_TEST_DATA_NUM) {
        return SERVICE_E_ERROR;
    }
    kernel_test_data[inx] = data;
    return SERVICE_E_OK;
}
void kernel_printf(const char* p)
{
    if (p != NULL) {
        printf(p);
    }
}

void *svc_call_table[KERNEL_SERVICE_CALL_NUM] = {
    (void*)kernel_get_data,
    (void*)kernel_set_data,
    (void*)kernel_printf,
};
