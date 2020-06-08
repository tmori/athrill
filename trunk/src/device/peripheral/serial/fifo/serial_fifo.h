#ifndef _SERIAL_FIFO_H_
#define _SERIAL_FIFO_H_

#include "std_types.h"
#include "cpu.h"
#include "std_device_ops.h"
#include "comm_buffer.h"
#include "athrill_mpthread.h"

#define SERIAL_FIFO_MAX_CHANNEL_NUM					8U

#define SERIAL_FIFO_WRITE_STATUS_OFF				(SERIAL_FIFO_MAX_CHANNEL_NUM * 0U)
#define SERIAL_FIFO_WRITE_CMD_OFF					(SERIAL_FIFO_MAX_CHANNEL_NUM * 1U)
#define SERIAL_FIFO_WRITE_PTR_OFF					(SERIAL_FIFO_MAX_CHANNEL_NUM * 2U)
#define SERIAL_FIFO_READ_STATUS_OFF					(SERIAL_FIFO_MAX_CHANNEL_NUM * 3U)
#define SERIAL_FIFO_READ_CMD_OFF					(SERIAL_FIFO_MAX_CHANNEL_NUM * 4U)
#define SERIAL_FIFO_READ_PTR_OFF					(SERIAL_FIFO_MAX_CHANNEL_NUM * 5U)

/*
 * status:
 *  0x0: can write
 *  0x1: can not write(busy)
 */
#define SERIAL_FIFO_WRITE_STATUS_ADDR(base, ch)		((base) + SERIAL_FIFO_WRITE_STATUS_OFF + (ch))
/*
 * cmd:
 *  0x0: no cmd
 *  0x1: move one char data from fifo buffer
 */
#define SERIAL_FIFO_WRITE_CMD_ADDR(base, ch)		((base) + SERIAL_FIFO_WRITE_CMD_OFF + (ch))
#define SERIAL_FIFO_WRITE_PTR_ADDR(base, ch)		((base) + SERIAL_FIFO_WRITE_PTR_OFF    + (ch))

#define SERIAL_FIFO_WRITE_STATUS_CAN_DATA		0x0
#define SERIAL_FIFO_WRITE_STATUS_DATA_FULL		0x1

#define SERIAL_FIFO_WRITE_CMD_NONE				0x0
#define SERIAL_FIFO_WRITE_CMD_MOVE				0x1
#define SERIAL_FIFO_WRITE_CMD_TX				0x2

/*
 *  cpu write example:
 *  	while (res > 0) {
 *  		status = dev_read(status_addr);
 *  		if (status == 0x0) {
 *	 			dev_write(ptr_addr, data);
 *	 			dev_write(cmd_addr, 0x1);
 *	 			res--;
 * 			}
 * 			else { busy..
 * 				break;
 * 			}
 * 		}
 * 		dev_write(cmd_addr, 0x2);
 */


/*
 * status:
 *  0x0: can not read(no data)
 *  0x1: can read
 */
#define SERIAL_FIFO_READ_STATUS_ADDR(base, ch)		((base) + SERIAL_FIFO_READ_STATUS_OFF + (ch))
/*
 * cmd:
 *  0x0: no cmd
 *  0x1: move one char data from fifo buffer
 */
#define SERIAL_FIFO_READ_CMD_ADDR(base, ch)			((base) + SERIAL_FIFO_READ_CMD_OFF + (ch))
#define SERIAL_FIFO_READ_PTR_ADDR(base, ch)			((base) + SERIAL_FIFO_READ_PTR_OFF    + (ch))
#define SERIAL_FIFO_READ_STATUS_NO_DATA			0x0
#define SERIAL_FIFO_READ_STATUS_DATA_IN			0x1

#define SERIAL_FIFO_READ_CMD_NONE				0x0
#define SERIAL_FIFO_READ_CMD_MOVE				0x1

/*
 *  cpu read example:
 *  	while (1) {
 *  		status = dev_read(status_addr);
 *  		if (status == 0x1) {
 *	 			dev_write(cmd_addr, 0x1);
 *  			data = dev_read(ptr_addr);
 * 			}
 * 			else { no data..
 * 				break;
 * 			}
 * 		}
 */

extern void athrill_device_init_serial_fifo(void);
extern void athrill_device_supply_clock_serial_fifo(DeviceClockType *dev_clock);
#define SERIAL_FIFO_RD_BUFFER_LEN	16U
#define SERIAL_FIFO_WR_BUFFER_LEN	16U
typedef struct {
	/*
	 * read: cpu
	 * write: external device
	 */
	CommFifoBufferType 	rd;
	CommFifoBufferType	rd_dev_buffer;
	uint32				rd_intno;
	uint32				rd_intoff;
	uint32				rd_raise_delay_count;
	bool				rd_raise_intr;
	/*
	 * read: external device
	 * write: cpu
	 */
	CommFifoBufferType 	wr;
	CommFifoBufferType	wr_dev_buffer;
	uint32				wr_intno;
	uint32				wr_intoff;
	uint32				wr_raise_delay_count;
	bool				wr_raise_intr;

	/*
	 * for serial ext thread info
	 */
	MpthrIdType			rx_thread;
	char				*rx_serial_fifopath;
	MpthrIdType			tx_thread;
	char				*tx_serial_fifopath;
} AthrillSerialFifoType;
extern void athrill_device_get_serial_fifo_buffer(uint32 channel, AthrillSerialFifoType **serial_fifop);

#endif /* _SERIAL_FIFO_H_ */
