

#ifndef TOPPERS_V850ES_FK3_EMU_ENV_H
#define TOPPERS_V850ES_FK3_EMU_ENV_H

#include "v850_gcc/v850es_fk3.h"


#define PCLOCK			(20000000)


#define LED1_ADDRESS	PCT
#define LED1_BITPOS		6

/* シリアル関連定義 */
#define BAUD_38400_UA0CTL1	(0x01)
#define BAUD_38400_UA0CTL2	(0x82)

#define BAUD_19200_UA0CTL1	(0x02)
#define BAUD_19200_UA0CTL2	(0x82)

#define BAUD_9600_UA0CTL1	(0x03)
#define BAUD_9600_UA0CTL2	(0x82)

#endif	/* TOPPERS_CQ_V850_FK3_EMU_ENV_H */
