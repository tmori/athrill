#ifndef _LIST_TEST_H_
#define _LIST_TEST_H_
#include "test_serial.h"

extern void list_test_ERR_1(void);
extern void list_test_ERR_2(void);
extern void list_test_OK_1(void);
extern void list_test_OK_2(void);
extern void list_test_OK_3(void);


extern void test_print(const char *str);
#define ASSERT(expr)	\
do {	\
	if (expr) {	\
		test_print("PASSED : " #expr);	\
	}	\
	else {	\
		test_print("FAILED : " #expr);	\
	}	\
} while (0)

#endif /* _LIST_TEST_H_ */