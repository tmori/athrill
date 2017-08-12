#ifndef _STD_TYPES_H_
#define _STD_TYPES_H_

typedef char sint8;
typedef short sint16;
typedef int sint32;
typedef long long sint64;

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long long uint64;
typedef int	bool;

typedef uint32 Std_ReturnType;

typedef uint32 CoreIdType;


#ifndef NULL
#define NULL	((void*)0)
#endif

#ifndef TRUE
#define TRUE	(1U)
#endif

#ifndef FALSE
#define FALSE	(0U)
#endif

#ifndef UINT_C
#define UINT_C(val)		(val ## U)
#endif /* UINT_C */


#endif /* _STD_TYPES_H_ */
