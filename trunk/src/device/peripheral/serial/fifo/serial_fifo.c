#include "serial_fifo.h"
#include "assert.h"
#include "mpu_ops.h"
#include "cpuemu_ops.h"

static AthrillSerialFifoType athrill_serial_fifo[SERIAL_FIFO_MAX_CHANNEL_NUM];
static uint32 serial_fifo_base_addr = 0x0;

static char serial_fifo_param_buffer[256];
void athrill_device_init_serial_fifo(void)
{
	uint32 i;
	uint32 buffer_size;
	Std_ReturnType ret;

	ret = cpuemu_get_devcfg_value_hex("DEVICE_CONFIG_SERIAL_FILFO_BASE_ADDR\n", &serial_fifo_base_addr);
	if (ret != STD_E_OK) {
		return;
	}
	printf("DEVICE_CONFIG_SERIAL_FILFO_BASE_ADDR=0x%x", serial_fifo_base_addr);
	for (i = 0; i < SERIAL_FIFO_MAX_CHANNEL_NUM; i++) {
		memset(serial_fifo_param_buffer, 0, sizeof(serial_fifo_param_buffer));
		snprintf(serial_fifo_param_buffer, sizeof(serial_fifo_param_buffer), "DEVICE_CONFIG_SERIAL_FILFO_%d_SIZE", i);
		ret = cpuemu_get_devcfg_value("serial_fifo_param_buffer", &buffer_size);
		if (ret == STD_E_OK) {
			athrill_serial_fifo[i].rp = fifo_create(buffer_size);
			ASSERT(athrill_serial_fifo[i].rp != NULL);
			athrill_serial_fifo[i].wp = fifo_create(buffer_size);
			ASSERT(athrill_serial_fifo[i].wp != NULL);
			printf("%s=%u\n", serial_fifo_param_buffer, buffer_size);
		}
		else {
			athrill_serial_fifo[i].rp = NULL;
			athrill_serial_fifo[i].wp = NULL;
		}
	}
	return;
}

static void do_serial_fifo_cpu_read(uint32 channel)
{
	Std_ReturnType err;
	uint8 data;
	uint8 cmd;

	/*
	 * status
	 */
	if (FIFO_IS_EMPTY(athrill_serial_fifo[channel].rp)) {
		//no data found
		mpu_put_data8(0U, SERIAL_FIFO_READ_STATUS_ADDR(serial_fifo_base_addr, channel), SERIAL_FIFO_READ_STATUS_NO_DATA);
	}
	else {
		//found data
		mpu_put_data8(0U, SERIAL_FIFO_READ_STATUS_ADDR(serial_fifo_base_addr, channel), SERIAL_FIFO_READ_STATUS_DATA_IN);
	}
	/*
	 * check command and move data
	 */
	mpu_get_data8(0U, SERIAL_FIFO_READ_CMD_ADDR(serial_fifo_base_addr, channel), &cmd);
	if (cmd == SERIAL_FIFO_READ_CMD_MOVE) {
		err = fifo_get_char(athrill_serial_fifo[channel].rp, &data);
		if (err == STD_E_OK) {
			mpu_put_data8(0U, SERIAL_FIFO_READ_PTR_ADDR(serial_fifo_base_addr, channel), data);
		}
		mpu_put_data8(0U, SERIAL_FIFO_READ_CMD_ADDR(serial_fifo_base_addr, channel), SERIAL_FIFO_READ_CMD_NONE);
	}

	return;
}
static void do_serial_fifo_cpu_write(uint32 channel)
{
	uint8 data;
	uint8 cmd;
	/*
	 * status
	 */
	if (FIFO_IS_FULL(athrill_serial_fifo[channel].wp)) {
		mpu_put_data8(0U, SERIAL_FIFO_WRITE_STATUS_ADDR(serial_fifo_base_addr, channel), SERIAL_FIFO_WRITE_STATUS_CAN_DATA);
	}
	else {
		mpu_put_data8(0U, SERIAL_FIFO_WRITE_STATUS_ADDR(serial_fifo_base_addr, channel), SERIAL_FIFO_WRITE_STATUS_DATA_FULL);
	}
	/*
	 * check command and move data
	 */
	mpu_get_data8(0U, SERIAL_FIFO_WRITE_CMD_ADDR(serial_fifo_base_addr, channel), &cmd);
	if (cmd == SERIAL_FIFO_READ_CMD_MOVE) {
		mpu_get_data8(0U, SERIAL_FIFO_WRITE_PTR_ADDR(serial_fifo_base_addr, channel), &data);
		(void)fifo_put_char(athrill_serial_fifo[channel].wp, data);
		mpu_put_data8(0U, SERIAL_FIFO_WRITE_CMD_ADDR(serial_fifo_base_addr, channel), SERIAL_FIFO_WRITE_CMD_NONE);
	}
	return;
}

void athrill_device_supply_clock_serial_fifo(DeviceClockType *dev_clock)
{
	uint32 i;
	if (serial_fifo_base_addr == 0x0) {
		return;
	}
	for (i = 0; i < SERIAL_FIFO_MAX_CHANNEL_NUM; i++) {
		if (athrill_serial_fifo[i].rp == NULL) {
			continue;
		}
		do_serial_fifo_cpu_read(i);
		do_serial_fifo_cpu_write(i);
	}
	return;
}

void athrill_device_get_serial_fifo_buffer(uint32 channel, AthrillSerialFifoType **serial_fifop)
{
	*serial_fifop = NULL;
	if (serial_fifo_base_addr == 0x0) {
		return;
	}
	if (channel >= SERIAL_FIFO_MAX_CHANNEL_NUM) {
		return;
	}
	if (athrill_serial_fifo[channel].rp == NULL) {
		return;
	}
	*serial_fifop = &athrill_serial_fifo[channel];
	return;
}


