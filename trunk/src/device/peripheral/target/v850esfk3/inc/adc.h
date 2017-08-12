#ifndef _ADC_H_
#define _ADC_H_

#include "device.h"

/*
 * A/Dコンバータ
 */
#define MPU_ADC_ADA_NUM		(2U)
#define MPU_ADC_ADA0		(0U)
#define MPU_ADC_ADA1		(1U)

/*
 * A/Dコンバータ・モード・レジスタ 0（ ADAnM0）
 */
#define MPU_ADC_ADDR_AdAnM0_BASE			(0xFFFFF200U)
#define MPU_ADC_ADDR_AdAnM0(cntl)			(MPU_ADC_ADDR_AdAnM0_BASE + ((cntl) * 0x40))

#define MPU_ADC_ADDR_AdAnM0_ADAnCE			(7U)
#define MPU_ADC_ADDR_AdAnM0_ADAnPS			(6U)
#define MPU_ADC_ADDR_AdAnM0_ADAnMD1			(5U)
#define MPU_ADC_ADDR_AdAnM0_ADAnMD0			(4U)
#define MPU_ADC_ADDR_AdAnM0_ADAnETS1		(3U)
#define MPU_ADC_ADDR_AdAnM0_ADAnETS0		(2U)
#define MPU_ADC_ADDR_AdAnM0_ADAnTMD			(1U)
#define MPU_ADC_ADDR_AdAnM0_ADAnEF			(0U)

/*
 * A/Dコンバータ・モード・レジスタ 1（ ADAnM1）
 */
#define MPU_ADC_ADDR_AdAnM1_BASE			(0xFFFFF201U)
#define MPU_ADC_ADDR_AdAnM1(cntl)			(MPU_ADC_ADDR_AdAnM1_BASE + ((cntl) * 0x40))

/*
 * A/Dコンバータ・モード・レジスタ 2（ ADAnM2）
 */
#define MPU_ADC_ADDR_AdAnM2_BASE			(0xFFFFF203U)
#define MPU_ADC_ADDR_AdAnM2(cntl)			(MPU_ADC_ADDR_AdAnM2_BASE + ((cntl) * 0x40))

/*
 * A/Dコンバータ・チャネル指定レジスタ 0（ ADAnS）
 */
#define MPU_ADC_ADDR_ADAnS_BASE				(0xFFFFF202U)
#define MPU_ADC_ADDR_ADAnS(cntl)			(MPU_ADC_ADDR_AdAnS_BASE + ((cntl) * 0x40))
/*
 * A/D変換結果レジスタ m
 */
#define MPU_ADC_ADDR_ADAnCRm_BASE				(0xFFFFF210U)
#define MPU_ADC_ADDR_ADAnCRm(cntl,pchannel)		((MPU_ADC_ADDR_ADAnCRm_BASE + ((cntl) * 0x40)) + ((pchannel) * 2U))

typedef struct {
	bool (*recv) (uint8 ch, uint16 *data);
} DeviceAdcOpType;

extern void device_adc_register_ops(void *adc, DeviceAdcOpType *ops);


#endif /* _ADC_H_ */
