#ifndef _TYPES_H_
#define _TYPES_H_

#define SERVICE_E_OK         (0)
#define SERVICE_E_ERROR      (-1)

typedef int ServiceReturnType;

typedef unsigned int SrvUint32;
typedef int SrvInt32;

#ifndef NULL
#define NULL    ((void*)0)
#endif

#ifndef FALSE
#define FALSE   0U
#endif

#ifndef TRUE
#define TRUE   1U
#endif

#endif /* _TYPES_H_ */