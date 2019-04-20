#ifndef _DIGITAL_H_
#define _DIGITAL_H_

/*
 * digital bits addr(uint8)
 */
#define DIGITAL_REG_ADDR            ((volatile unsigned char *)0x07FF0000)

#define DIGITAL_SWITCH_START        0x01
#define DIGITAL_SWITCH_STOP         0x02
#define DIGITAL_SWITCH_DOOR         0x04
#define DIGITAL_SWITCH_UP1          0x08
#define DIGITAL_SWITCH_DOWN1        0x10
#define DIGITAL_SWITCH_UP2          0x20
#define DIGITAL_SWITCH_DOWN2        0x40

#endif /* _DIGITAL_H_ */