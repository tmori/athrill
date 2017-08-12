#ifndef _TIMER_M_H_
#define _TIMER_M_H_

#include "device.h"

/*
 * TMM0コンペア・レジスタ 0（ TM0CMP0）
 */
#define MPU_TMM_ADDR_TM0CMP0				(0xFFFFF694U)

/*
 * TMM0制御レジスタ 0（ TM0CTL0）
 */
#define MPU_TMM_ADDR_TM0CTL0				(0xFFFFF690U)

#define MPU_TMM_ADDR_TM0CTL0_TM0CE			(7U)
#define MPU_TMM_ADDR_TM0CTL0_TM0CKS2		(2U)
#define MPU_TMM_ADDR_TM0CTL0_TM0CKS1		(1U)
#define MPU_TMM_ADDR_TM0CTL0_TM0CKS0		(0U)

#endif /* _TIMER_M_H_ */
