#include "inc/comm.h"
#include "intc.h"
#include "device.h"
#include "mpu_types.h"
#include "std_errno.h"
#include "cpuemu_ops.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


static Std_ReturnType comm_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data);
static Std_ReturnType comm_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data);
static Std_ReturnType comm_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data);
static Std_ReturnType comm_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data);
static Std_ReturnType comm_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data);
static Std_ReturnType comm_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data);

MpuAddressRegionOperationType	comm_memory_operation = {
		.get_data8 		= 	comm_get_data8,
		.get_data16		=	comm_get_data16,
		.get_data32		=	comm_get_data32,

		.put_data8 		= 	comm_put_data8,
		.put_data16		=	comm_put_data16,
		.put_data32		=	comm_put_data32,

		.get_pointer	= NULL
};


#define MAX_BUFFER_SIZE	4096
#define SYNC_COUNT		10000	//500 usec
typedef struct {
	char *path;
	uint32 fileoff;
	uint32 off;
	uint32 size;
	uint32 max;
	char buffer[MAX_BUFFER_SIZE];
} CpuEmuCommFileType;

typedef struct {
	CpuEmuCommFileType rx_fifo;
	CpuEmuCommFileType tx_fifo;
	uint32 count;
	uint32 sync_count;
} CpuEmuCommDevType;

static CpuEmuCommDevType CpuEmuCommDev;
static void tx_fifo_sync(void);
static void tx_fifo_write(uint32 data);
static void rx_fifo_sync(void);
static void rx_fifo_read(MpuAddressRegionType *region, uint32 *data);
static void rx_fifo_read_status(MpuAddressRegionType *region, uint32 *data);

static MpuAddressRegionType *comm_region;

void device_init_comm(MpuAddressRegionType *region)
{
	comm_region = region;

	CpuEmuCommDev.rx_fifo.path = (char*)cpuemu_get_comm_rx_fifo();
	CpuEmuCommDev.tx_fifo.path = (char*)cpuemu_get_comm_tx_fifo();

	CpuEmuCommDev.rx_fifo.max = MAX_BUFFER_SIZE;
	CpuEmuCommDev.tx_fifo.max = MAX_BUFFER_SIZE;

	CpuEmuCommDev.sync_count = SYNC_COUNT;

	return;
}


void device_supply_clock_comm(DeviceClockType *dev_clock)
{
	if (CpuEmuCommDev.tx_fifo.path == NULL) {
		return;
	}
	CpuEmuCommDev.count++;
	if (CpuEmuCommDev.count >= CpuEmuCommDev.sync_count) {
		//polling
		tx_fifo_sync();
		//rx_fifo_sync();
		CpuEmuCommDev.count = 0;
	}
	return;
}


/*--------------------- static ---------------- */

static void tx_fifo_sync(void)
{
	int fd;
	int err;

	fd = open(CpuEmuCommDev.tx_fifo.path, O_WRONLY|O_BINARY);
	if (fd < 0) {
		printf("file open error:%s\n", CpuEmuCommDev.tx_fifo.path);
		exit(1);
	}
	err = lseek(fd, CpuEmuCommDev.tx_fifo.fileoff, SEEK_SET);
	if (err < 0) {
		printf("lseek error:%s\n", CpuEmuCommDev.tx_fifo.path);
		exit(1);
	}

	err = write(fd, CpuEmuCommDev.tx_fifo.buffer, CpuEmuCommDev.tx_fifo.size);
	if (err != CpuEmuCommDev.tx_fifo.size) {
		printf("write error:%s\n", CpuEmuCommDev.tx_fifo.path);
		exit(1);
	}
	CpuEmuCommDev.tx_fifo.fileoff += err;
	CpuEmuCommDev.tx_fifo.off = 0;
	CpuEmuCommDev.tx_fifo.size = 0;

	close(fd);
	return;
}

static void rx_fifo_sync(void)
{
	int fd;
	int err;
	struct stat buf;

	fd = open(CpuEmuCommDev.rx_fifo.path, O_RDONLY|O_BINARY);
	if (fd < 0) {
		printf("file open error:%s\n", CpuEmuCommDev.rx_fifo.path);
		exit(1);
	}
	err = fstat(fd, &buf);
	if (err < 0) {
		printf("fstat error:%s\n", CpuEmuCommDev.rx_fifo.path);
		exit(1);
	}
	if (buf.st_size > CpuEmuCommDev.rx_fifo.fileoff) {
		err = lseek(fd, CpuEmuCommDev.rx_fifo.fileoff, SEEK_SET);
		if (err < 0) {
			printf("lseek error:%s\n", CpuEmuCommDev.rx_fifo.path);
			exit(1);
		}
		err = read(fd, CpuEmuCommDev.rx_fifo.buffer, CpuEmuCommDev.rx_fifo.max);
		if (err < 0) {
			printf("read error:%s\n", CpuEmuCommDev.rx_fifo.path);
			exit(1);
		}
		CpuEmuCommDev.rx_fifo.fileoff += err;
		CpuEmuCommDev.rx_fifo.off = 0;
		CpuEmuCommDev.rx_fifo.size = err;

	}
	close(fd);

	return;
}

static void tx_fifo_write(uint32 data)
{
	uint32 write_data = (0xFF & data);
	uint8 write_data8 = (uint8)write_data;

	if (CpuEmuCommDev.tx_fifo.path == NULL) {
		return;
	}

	if (CpuEmuCommDev.tx_fifo.size >= CpuEmuCommDev.tx_fifo.max) {
		tx_fifo_sync();
	}

	if (CpuEmuCommDev.tx_fifo.size < CpuEmuCommDev.tx_fifo.max) {
		CpuEmuCommDev.tx_fifo.buffer[CpuEmuCommDev.tx_fifo.off] = write_data8;
		CpuEmuCommDev.tx_fifo.off++;
		CpuEmuCommDev.tx_fifo.size++;
	}
	else {
		printf("ERROR:tx_fifo_write:empty buffer!!!\n");
	}
	return;
}

static void rx_fifo_read(MpuAddressRegionType *region, uint32 *data)
{
	uint8 data8 = 0;
	uint32 stat_data;
	uint32 off_stat = (CPU_EMU_COMM_FIFO_RX_STAT_ADDR & region->mask) - region->start;
	uint32 *stat_data_addr;

	if (CpuEmuCommDev.tx_fifo.path == NULL) {
		return;
	}

	if (CpuEmuCommDev.rx_fifo.off >= CpuEmuCommDev.rx_fifo.size) {
		rx_fifo_sync();
	}

	if (CpuEmuCommDev.rx_fifo.off < CpuEmuCommDev.rx_fifo.size) {
		data8 = CpuEmuCommDev.rx_fifo.buffer[CpuEmuCommDev.rx_fifo.off];
		*data = data8;
		CpuEmuCommDev.rx_fifo.off++;
	}

	if (CpuEmuCommDev.rx_fifo.off >= CpuEmuCommDev.rx_fifo.size) {
		stat_data = 0;
	}
	else {
		stat_data = 1;
	}
	stat_data_addr = (uint32*)&region->data[off_stat];
	*stat_data_addr = stat_data;

	return;
}

static void rx_fifo_read_status(MpuAddressRegionType *region, uint32 *data)
{
	uint32 stat_data;
	uint32 off_stat = (CPU_EMU_COMM_FIFO_RX_STAT_ADDR & region->mask) - region->start;
	uint32 *stat_data_addr;

	if (CpuEmuCommDev.tx_fifo.path == NULL) {
		*data = 0;
		return;
	}

	if (CpuEmuCommDev.rx_fifo.off >= CpuEmuCommDev.rx_fifo.size) {
		rx_fifo_sync();
	}

	if (CpuEmuCommDev.rx_fifo.off >= CpuEmuCommDev.rx_fifo.size) {
		stat_data = 0;
	}
	else {
		stat_data = 1;
	}

	stat_data_addr = (uint32*)&region->data[off_stat];
	*stat_data_addr = stat_data;
	*data = stat_data;
	return;
}


static Std_ReturnType comm_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint8*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType comm_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint16*)(&region->data[off]));
	return STD_E_OK;
}

static Std_ReturnType comm_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data)
{
	if (addr == (CPU_EMU_COMM_FIFO_RX_DATA_ADDR & region->mask)) {
		rx_fifo_read(region, data);
	}
	else if (addr == (CPU_EMU_COMM_FIFO_RX_STAT_ADDR & region->mask)) {
		rx_fifo_read_status(region, data);
	}
	return STD_E_OK;
}

static Std_ReturnType comm_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data)
{
	uint32 off = (addr - region->start);
	*((uint8*)(&region->data[off])) = data;

	return STD_E_OK;
}
static Std_ReturnType comm_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - region->start);
	*((uint16*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType comm_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	if (addr == (CPU_EMU_COMM_FIFO_TX_DATA_ADDR & region->mask)) {
		tx_fifo_write(data);
	}
	return STD_E_OK;
}

