#include "service_call.h"
//#include "kernel_service.h"
#include "test_reg.h"

unsigned char user_stack_data[USER_STACK_SIZE] __attribute__ ((section(".bss_noclr_user")));

void user_task(void)
{
	SrvUint32 data;
	ServiceReturnType ret;

	ret = svc_set_data(0, 324);
	if (ret == SERVICE_E_OK) {
		(void)svc_get_data(0, &data);
	}
	svc_printf("Hello User World!!\n");

	while (1) {
		;
	}

	return;
}
