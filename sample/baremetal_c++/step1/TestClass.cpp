#include "TestClass.h"
#include "test_serial.h"

using namespace Baremetal;

TestClass::TestClass(void)
{
    test_print("TestClass:constructor :enter\n");
}
TestClass::~TestClass(void)
{
    test_print("TestClass:destructor :enter\n");
}

void TestClass::doTest(void)
{
    test_print("TestClass:doTest :enter\n");
}
