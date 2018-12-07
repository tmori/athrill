#include "test_serial.h"
#include "test_reg.h"
#include <string.h>
#include "athrill_syscall.h"
#include "TestClass.h"

using namespace Baremetal;

unsigned char stack_data[STACK_SIZE] __attribute__ ((section(".bss_noclr")));

sys_addr athrill_device_func_call __attribute__ ((section(".athrill_device_section")));

	TestClass a;

static void doTestClass(void)
{
	a.doTest();
}

int main(void)
{

	doTestClass();

	while (1) {
		;
	}
}

