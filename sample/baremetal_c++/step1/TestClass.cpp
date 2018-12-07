#include "TestClass.h"
#include "test_serial.h"

using namespace Baremetal;

TestClass::TestClass(void)
{
    printf("TestClass:constructor :enter\n");
}
TestClass::~TestClass(void)
{
    printf("TestClass:destructor :enter\n");
}

void TestClass::doTest(void)
{
    printf("TestClass:doTest :enter\n");
}
