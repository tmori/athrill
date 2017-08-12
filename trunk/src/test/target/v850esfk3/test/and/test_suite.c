#include "test_check.h"

extern int do_test_and_1(void);
extern int do_test_and_2(void);
extern int do_test_and_3(void);
extern int do_test_and_4(void);
extern int do_test_and_5(void);
extern int do_test_and_6(void);
extern int do_test_and_7(void);
extern int do_test_and_8(void);
extern int do_test_and_9(void);
extern int do_test_and_10(void);
extern int do_test_and_11(void);
extern int do_test_and_12(void);
extern int do_test_and_13(void);
extern int do_test_and_14(void);
extern int do_test_and_15(void);
extern int do_test_and_16(void);

void test_suite(void)
{
    DO_TEST(do_test_and_1);
    DO_TEST(do_test_and_2);
    DO_TEST(do_test_and_3);
    DO_TEST(do_test_and_4);
    DO_TEST(do_test_and_5);
    DO_TEST(do_test_and_6);
    DO_TEST(do_test_and_7);
    DO_TEST(do_test_and_8);
    DO_TEST(do_test_and_9);
    DO_TEST(do_test_and_10);
    DO_TEST(do_test_and_11);
    DO_TEST(do_test_and_12);
    DO_TEST(do_test_and_13);
    DO_TEST(do_test_and_14);
    DO_TEST(do_test_and_15);
    DO_TEST(do_test_and_16);
	return;
}
