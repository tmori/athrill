#include "test_serial.h"
#include "section.h"
#include "interrupt.h"
#include <string.h>
#include "timer.h"
#include "service_call.h"
#include "test_reg.h"

unsigned char stack_data[STACK_SIZE] __attribute__ ((section(".bss_noclr_kernel")));
volatile unsigned int kernel_task_data __attribute__ ((section(".bss_noclr_kernel")));

extern void user1_start(void);
extern void user2_start(void);

static KernelMpuTestModeType kernel_mputest_mode;

void kernel_internal_func(void)
{
	user1_task_data = 999;
	user2_task_data = 999;
	user_shared_data = 999;
	kernel_task_data = 999;

	user1_internal_func();
	user2_internal_func();
	kernel_task_data++;

	return;
}

static void kernel_task(void)
{
	kernel_internal_func();
	ASSERT(user1_task_data == 1000);
	ASSERT(user2_task_data == 1000);
	ASSERT(kernel_task_data == 1000);
	ASSERT(user_shared_data == 1001);
	return;
}

static void do_user_test(void (*testfunc)(void))
{
	test_print_line(" START:KernelMpuTestNo_", kernel_mputest_no);
	switch (kernel_mputest_no) {
	case KernelMpuTestNo_0:
		/* Exception Test: write other user */
		printf(" Expect: EXCEPTION for do [other user<id>_task_data = 10]\n");
		break;
	case KernelMpuTestNo_1:
		/* Exception Test: write kernel */
		printf(" Expect: EXCEPTION for do [kernel_task_data = 10]\n");
		break;
	case KernelMpuTestNo_2:
		/* Exception Test: read user2 */
		printf(" Expect: EXCEPTION for do [other user<id>_task_data = user2_task_data]\n");
		break;
	case KernelMpuTestNo_3:
		/* Exception Test: read kernel */
		printf(" Expect: EXCEPTION for do [other user<id>_task_data = kernel_task_data]\n");
		break;
	case KernelMpuTestNo_4:
		/* Exception Test: exec user2 */
		printf(" Expect: EXCEPTION for do [other user2_internal_func()]\n");
		break;
	case KernelMpuTestNo_5:
		/* Exception Test: exec kernel */
		printf(" Expect: EXCEPTION for do [kernel_internal_func()]\n");
		break;
	case KernelMpuTestNo_6:
		/* normal test: user1 read, write, exec */
		printf(" Expect: not happen EXCEPTION for do [internal user read/write/exec]\n");
		break;
	default:
		break;
	}
	testfunc();
	return;
}

void kernel_mputest(void)
{
	switch (kernel_mputest_mode) {
	case KernelMpuTestMode_User1:
		do_user_test(user1_start);
		break;
	case KernelMpuTestMode_User2:
		do_user_test(user2_start);
		break;
	case KernelMpuTestMode_Kernel:
	default:
		break;
	}

	kernel_task();

	printf("****End: Kernel Test!\n");
	while (1) {
		;
	}
}

int main(void)
{
	int i;
	timer_init();

	kernel_mputest_mode = KernelMpuTestMode_User1;
	kernel_mputest_no = KernelMpuTestNo_0;

	user1_task_data = 99;
	user2_task_data = 99;
	kernel_task_data = 99;
	user_shared_data = 99;

	for (i = 0; i < KernelMpuTestNo_Num; i++) {
		user1_test_result[i] = FALSE;
		user2_test_result[i] = FALSE;
	}

	printf("\n****Start: Kernel Test:User1!****\n");
	kernel_mputest();
	while (1) {
		;
	}
}

void kernel_exception(uint32 fepc, uint32 fepsw)
{
	/* next test */
	if (kernel_mputest_no == KernelMpuTestNo_6) {
		kernel_mputest_no = KernelMpuTestNo_0;
		if (kernel_mputest_mode == KernelMpuTestMode_User1) {

			ASSERT(user1_test_result[KernelMpuTestNo_0] == TRUE);
			ASSERT(user1_test_result[KernelMpuTestNo_1] == TRUE);
			ASSERT(user1_test_result[KernelMpuTestNo_2] == TRUE);
			ASSERT(user1_test_result[KernelMpuTestNo_3] == TRUE);
			ASSERT(user1_test_result[KernelMpuTestNo_4] == TRUE);
			ASSERT(user1_test_result[KernelMpuTestNo_5] == TRUE);
			ASSERT(user1_test_result[KernelMpuTestNo_6] == TRUE);

			ASSERT(user1_task_data == 100);
			ASSERT(user_shared_data == 100);
			ASSERT(user2_task_data == 99);
			ASSERT(kernel_task_data == 99);

			printf("\n****Start: Kernel Test:User2!****\n");
		}
		else if (kernel_mputest_mode == KernelMpuTestMode_User2) {
			ASSERT(user2_test_result[KernelMpuTestNo_0] == TRUE);
			ASSERT(user2_test_result[KernelMpuTestNo_1] == TRUE);
			ASSERT(user2_test_result[KernelMpuTestNo_2] == TRUE);
			ASSERT(user2_test_result[KernelMpuTestNo_3] == TRUE);
			ASSERT(user2_test_result[KernelMpuTestNo_4] == TRUE);
			ASSERT(user2_test_result[KernelMpuTestNo_5] == TRUE);
			ASSERT(user2_test_result[KernelMpuTestNo_6] == TRUE);

			ASSERT(user1_task_data == 100);
			ASSERT(user_shared_data == 101);
			ASSERT(user2_task_data == 100);
			ASSERT(kernel_task_data == 99);
			printf("\n****Start: Kernel Test:Kernel!****\n");
		}
		kernel_mputest_mode++;
	}
	else {
		printf(" PASSED:Exception happened!!\n");
		kernel_mputest_no++;
	}
	kernel_mputest();

	/* not reached */
	while (1) {
		;
	}
}

void bss_clear(void)
{
	unsigned char *p = &_bss_kernel_start;
	unsigned char *e = &_bss_kernel_end;
	for (;p < e; p++) {
		*p = 0;
	}
	return;
}

void data_init(void)
{
	unsigned char *p_rom = &_idata_kernel_start;
	unsigned char *e_rom = &_idata_kernel_end;
	unsigned char *p_ram = &_data_kernel_start;

	for (;p_rom < e_rom; p_ram++, p_rom++) {
		*p_ram = *p_rom;
	}
}

