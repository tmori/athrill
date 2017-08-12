#ifndef _DBG_TARGET_SERIAL_H_
#define _DBG_TARGET_SERIAL_H_

#include "std_types.h"
#include "std_errno.h"

extern Std_ReturnType dbg_serial_in(uint8 channel, uint8 data);

extern bool dbg_serial_getchar(uint8 channel, uint8 *data);
extern bool dbg_serial_putchar(uint8 channel, uint8 data);

#endif /* _DBG_TARGET_SERIAL_H_ */
