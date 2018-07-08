#include "kernel_service.h"

extern ServiceReturnType svc_call_get_data(SrvUint32 inx, SrvUint32 *ret_data);
extern ServiceReturnType svc_call_set_data(SrvUint32 inx, SrvUint32 data);

#define KERNEL_TEST_DATA_NUM    5U
SrvUint32 kernel_test_data[KERNEL_TEST_DATA_NUM];

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
#if 0
ServiceReturnType svc_call_get_data(SrvUint32 inx, SrvUint32 *ret_data)
{
    //TODO
}

ServiceReturnType svc_call_set_data(SrvUint32 inx, SrvUint32 data)
{
    //TODO
}
#endif