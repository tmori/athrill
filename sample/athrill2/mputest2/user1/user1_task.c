#include "service_call.h"
#include "test_reg.h"

volatile unsigned int user1_task_data __attribute__ ((section(".bss_noclr_user1")));

void user1_task(void)
{
	switch (kernel_mputest_no) {
	case KernelMpuTestNo_0:
		/* Exception Test: write user2 */
		user1_test_result[kernel_mputest_no] = TRUE;
		user2_task_data = 10;
		user1_test_result[kernel_mputest_no] = FALSE;
		break;
	case KernelMpuTestNo_1:
		/* Exception Test: write kernel */
		user1_test_result[kernel_mputest_no] = TRUE;
		kernel_task_data = 10;
		user1_test_result[kernel_mputest_no] = FALSE;
		break;
	case KernelMpuTestNo_2:
		/* Exception Test: read user2 */
		user1_test_result[kernel_mputest_no] = TRUE;
		user1_task_data = user2_task_data;
		user1_test_result[kernel_mputest_no] = FALSE;
		break;
	case KernelMpuTestNo_3:
		/* Exception Test: read kernel */
		user1_test_result[kernel_mputest_no] = TRUE;
		user1_task_data = kernel_task_data;
		user1_test_result[kernel_mputest_no] = FALSE;
		break;
	case KernelMpuTestNo_4:
		/* Exception Test: exec user2 */
		user1_test_result[kernel_mputest_no] = TRUE;
		user2_internal_func();
		user1_test_result[kernel_mputest_no] = FALSE;
		break;
	case KernelMpuTestNo_5:
		/* Exception Test: exec kernel */
		user1_test_result[kernel_mputest_no] = TRUE;
		kernel_internal_func();
		user1_test_result[kernel_mputest_no] = FALSE;
		break;
	case KernelMpuTestNo_6:
		/* normal test: user1 read, write, exec */
		user1_internal_func();
		user1_test_result[kernel_mputest_no] = TRUE;
		user1_task_data = kernel_task_data; /* for end */
		user1_test_result[kernel_mputest_no] = FALSE;
		break;
	default:
		break;
	}
	return;
}

void user1_internal_func(void)
{
	user1_task_data++;
	user_shared_data++;
	return;
}