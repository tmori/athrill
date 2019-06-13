#ifndef TOPPERS_IOC_H
#define TOPPERS_IOC_H

#include "Os.h"
#include "Os_Lcfg.h"

/* IOC data management block */
typedef struct ioc_data_management_block_IOC_QUE {
	uint8	data1;
}IOCMB_IOC_QUE;

typedef struct ioc_data_management_block_IOC_DEQUE {
	uint8	data1;
	uint8	data2;
	uint8	data3;
}IOCMB_IOC_DEQUE;



/* IOC_QUE API */
extern Std_ReturnType IocSend_IOC_QUE_1(uint8 in1);
extern Std_ReturnType IocSend_IOC_QUE_0(uint8 in1);
extern Std_ReturnType IocReceive_IOC_QUE(uint8 *out1);
extern Std_ReturnType IocEmptyQueue_IOC_QUE(void);

/* IOC_DEQUE API */
extern Std_ReturnType IocWriteGroup_IOC_DEQUE(uint8 in1, uint8 in2, uint8 in3);
extern Std_ReturnType IocReadGroup_IOC_DEQUE(uint8 *out1, uint8 *out2, uint8 *out3);

#endif /* TOPPERS_IOC_H_ */

