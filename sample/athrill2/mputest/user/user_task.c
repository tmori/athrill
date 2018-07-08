static int test_data10;
static int test_data0 = 123;
static const int rodata = 123;

void user_task(void)
{
	test_data10 = 100;
	return;
}
