#include "test_serial.h"
#include "test_reg.h"
#include <string.h>
#include "athrill_syscall.h"
#include "TestClass.h"
#include <math.h>
#include <string>

using namespace Baremetal;

unsigned char stack_data[STACK_SIZE] __attribute__ ((section(".bss_noclr")));

sys_addr athrill_device_func_call __attribute__ ((section(".athrill_device_section")));

static void doTestClassOp(TestClass &p)
{
	std::string a = "aa";
	a += "bb";
	p.doTest();
}

static void doTestClass(void)
{
	TestClass *obj = new TestClass();
	doTestClassOp(*obj);
	delete obj;
}

int main(void)
{
	doTestClass();
	while (1) {
		;
	}
}
