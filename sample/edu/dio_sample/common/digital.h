#ifndef _DIGITAL_H_
#define _DIGITAL_H_

/*
 * digital bits addr(uint8)
 */
#define DIGITAL_REG_ADDR            ((volatile unsigned char *)0x07FF0000)

#define DIGITAL_DSW1                0x01
#define DIGITAL_DSW2                0x02
#define DIGITAL_LED1                0x04
#define DIGITAL_LED2                0x08
#define DIGITAL_LED3                0x10

#endif /* _DIGITAL_H_ */