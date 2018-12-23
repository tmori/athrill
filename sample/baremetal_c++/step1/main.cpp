#include "test_serial.h"
#include "test_reg.h"
#include <string.h>
#include "athrill_syscall.h"
#include "TestClass.h"
#include <math.h>
#include <string>
#include <vector>
#include <stdlib.h>
#include "test_serial.h"

using namespace Baremetal;

unsigned char stack_data[STACK_SIZE] __attribute__ ((section(".bss_noclr")));

sys_addr athrill_device_func_call __attribute__ ((section(".athrill_device_section")));

static void doTestClassOp(TestClass &p)
{
	std::string a;
	a += "My name is ";
	a += "Takashi ";
	a += "Mori.\n";

	std::string b;
	b += "My favorite thing is ";
	b += "athrill ";
	b += "programing.\n";

	std::vector<std::string> v;

	v.push_back(a);
	v.push_back(b);

    for (size_t i = 0; i < v.size(); i++)
    {
		test_print((char*)v[i].c_str());
    }
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
	void *addr[4];

	addr[0] = malloc(31);
	addr[1] = malloc(32);
	addr[2] = malloc(33);
	addr[3] = malloc(8192);

	int i;
	for (i = 0; i < 4; i++) {
		free(addr[i]);
	}
	doTestClass();
	while (1) {
		;
	}
}
