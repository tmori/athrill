#include "kernel_service.h"

void kernel_task(void)
{
	SrvUint32 data;
	ServiceReturnType ret;

	ret = svc_get_data(0, &data);
	if (ret == SERVICE_E_OK) {
		data++;
		(void)svc_set_data(0, data);
	}
	return;
}
