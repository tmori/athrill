#include "inc/adc.h"
#include <stdio.h>

typedef enum {
	ADC_MODE_STOP,
	ADC_MODE_RUN
} AdcModeType;

typedef enum {
	ADC_ADA0_CHANNEL_0 = 0,
	ADC_ADA0_CHANNEL_1,
	ADC_ADA0_CHANNEL_2,
	ADC_ADA0_CHANNEL_3,
	ADC_ADA0_CHANNEL_4,
	ADC_ADA0_CHANNEL_5,
	ADC_ADA0_CHANNEL_6,
	ADC_ADA0_CHANNEL_7,
	ADC_ADA0_CHANNEL_8,
	ADC_ADA0_CHANNEL_NUM
} AdcAda0ChannelType;

typedef enum {
	ADC_ADA1_CHANNEL_0 = 0,
	ADC_ADA1_CHANNEL_1,
	ADC_ADA1_CHANNEL_2,
	ADC_ADA1_CHANNEL_3,
	ADC_ADA1_CHANNEL_4,
	ADC_ADA1_CHANNEL_5,
	ADC_ADA1_CHANNEL_6,
	ADC_ADA1_CHANNEL_7,
	ADC_ADA1_CHANNEL_8,
	ADC_ADA1_CHANNEL_9,
	ADC_ADA1_CHANNEL_NUM
} AdcAda1ChannelType;

typedef struct {
	uint16 cnt;
	AdcModeType mode;
	uint16 intno;
	uint16 conv_interval_clock; /* clock */
	uint16* adc_data;
	uint8 adc_data_num;
	DeviceAdcOpType *ops;
} AdcDeviceType;

static AdcDeviceType AdcDevice[MPU_ADC_ADA_NUM];
static uint16 adc_ada0_data[ADC_ADA0_CHANNEL_NUM];
static uint16 adc_ada1_data[ADC_ADA1_CHANNEL_NUM];
static MpuAddressRegionType *adc_region;

void device_init_adc(MpuAddressRegionType *region)
{
	int i;
	int cntl;
	
	adc_region = region;

	//printf("AD init\n");

	AdcDevice[MPU_ADC_ADA0].adc_data = adc_ada0_data;
	AdcDevice[MPU_ADC_ADA1].adc_data = adc_ada1_data;
	AdcDevice[MPU_ADC_ADA0].adc_data_num = ADC_ADA0_CHANNEL_NUM;
	AdcDevice[MPU_ADC_ADA1].adc_data_num = ADC_ADA1_CHANNEL_NUM;
	AdcDevice[MPU_ADC_ADA0].intno = 42;
	AdcDevice[MPU_ADC_ADA1].intno = 112;

	for (cntl = 0; cntl < MPU_ADC_ADA_NUM; cntl++) {
		AdcDevice[cntl].cnt = 0;
		AdcDevice[cntl].mode = ADC_MODE_STOP;
		AdcDevice[cntl].conv_interval_clock = 160U;

		for (i = 0; i < AdcDevice[cntl].adc_data_num; i++) {
			AdcDevice[cntl].adc_data[i] = 0U;
		}
	}


	return;
}

void device_adc_register_ops(void *adc, DeviceAdcOpType *ops)
{
	int cntl;
	for (cntl = 0; cntl < MPU_ADC_ADA_NUM; cntl++) {
		AdcDevice[cntl].ops = ops;
	}
	return;
}

void device_supply_clock_adc(DeviceClockType *dev_clock)
{
	int i;
	uint8 data;
	uint8 cntl;

	for (cntl = 0; cntl < MPU_ADC_ADA_NUM; cntl++) {
		(void)device_io_read8(adc_region, MPU_ADC_ADDR_AdAnM0(cntl), &data);
		if ((data & (1U << MPU_ADC_ADDR_AdAnM0_ADAnCE)) == 0U) {
			/*
			 * AD変換動作停止
			 */
			AdcDevice[cntl].mode = ADC_MODE_STOP;
			continue;
		}
		if ((data & (1U << MPU_ADC_ADDR_AdAnM0_ADAnPS)) == 0U) {
			//printf("AD%d OFF\n",cntl);
			/*
			 * AD電源OFF
			 */
			AdcDevice[cntl].mode = ADC_MODE_STOP;
			continue;
		}
		if (AdcDevice[cntl].mode == ADC_MODE_STOP) {
			//printf("AD%d RUN START\n", cntl);
			AdcDevice[cntl].mode = ADC_MODE_RUN;
			(void)device_io_write8(adc_region, MPU_ADC_ADDR_AdAnM0(cntl), (data | MPU_ADC_ADDR_AdAnM0_ADAnEF));
		}
		if (AdcDevice[cntl].cnt < AdcDevice[cntl].conv_interval_clock) {
			//printf("AD%d cut %d\n", cntl, AdcDevice[cntl].cnt);
			AdcDevice[cntl].cnt++;
			continue;
		}


		if (AdcDevice[cntl].ops != NULL) {
			for (i = 0; i < AdcDevice[cntl].adc_data_num; i++) {
				AdcDevice[cntl].ops[cntl].recv(i, &(AdcDevice[cntl].adc_data[i]));
				(void)device_io_write16(adc_region, MPU_ADC_ADDR_ADAnCRm(cntl,i), (AdcDevice[cntl].adc_data[i] << 6));
				//printf("AD%d RUN DATA WRITE %d\n", cntl, AdcDevice[cntl].adc_data[i]);
			}
		}

		//printf("AD%d STOP\n", cntl);
		data &= ~(1U << MPU_ADC_ADDR_AdAnM0_ADAnEF);
		data &= ~(1U << MPU_ADC_ADDR_AdAnM0_ADAnCE);
		(void)device_io_write8(adc_region, MPU_ADC_ADDR_AdAnM0(cntl), data);
		AdcDevice[cntl].cnt = 0;

		device_raise_int(AdcDevice[cntl].intno);
	}
	return;
}
