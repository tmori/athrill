#ifndef _ASSERT_H_
#define _ASSERT_H_

#include <stdio.h>
#include <stdlib.h>

#define ASSERT(expr)	\
do {	\
	if (!(expr))	{	\
		printf("ASSERTION FAILED:%s:%s:%d:%s\n", __FILE__, __FUNCTION__, __LINE__, #expr);	\
		exit(1);	\
	}	\
} while (0)

#endif /* _ASSERT_H_ */
