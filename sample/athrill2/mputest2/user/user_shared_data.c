#include "service_call.h"
#include "test_reg.h"

unsigned char user_stack_data[USER_STACK_SIZE] __attribute__ ((section(".bss_noclr_user")));
volatile unsigned int user_shared_data __attribute__ ((section(".bss_noclr_user")));
KernelMpuTestNoType kernel_mputest_no  __attribute__ ((section(".bss_noclr_user")));

unsigned char user1_test_result[10] __attribute__ ((section(".bss_noclr_user")));
unsigned char user2_test_result[10] __attribute__ ((section(".bss_noclr_user")));
