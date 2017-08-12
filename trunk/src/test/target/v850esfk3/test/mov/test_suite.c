#include "test_check.h"

struct test_structure {
    int a;
    short b;
};
struct test_structure test_variable;

extern int do_test_mov1_1(void);
extern int do_test_mov1_2(void);
extern int do_test_mov1_3(void);
extern int do_test_mov1_4(void);

extern int do_test_mov2_1(void);
extern int do_test_mov2_2(void);
extern int do_test_mov2_3(void);
extern int do_test_mov2_4(void);
extern int do_test_mov2_5(void);

extern int do_test_mov3_1(void);
extern int do_test_mov3_2(void);
extern int do_test_mov3_3(void);
extern int do_test_mov3_4(void);

void test_suite(void)
{

	DO_TEST(do_test_mov1_1);
	DO_TEST(do_test_mov1_2);
	DO_TEST(do_test_mov1_3);
	DO_TEST(do_test_mov1_4);

	DO_TEST(do_test_mov2_1);
	DO_TEST(do_test_mov2_2);
	DO_TEST(do_test_mov2_3);
	DO_TEST(do_test_mov2_4);
	DO_TEST(do_test_mov2_5);

	DO_TEST(do_test_mov3_1);
	DO_TEST(do_test_mov3_2);
	DO_TEST(do_test_mov3_3);
	DO_TEST(do_test_mov3_4);
	return;
}
