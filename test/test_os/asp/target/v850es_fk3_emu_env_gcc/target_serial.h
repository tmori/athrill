

#ifndef TOPPERS_TARGET_SERIAL_H
#define TOPPERS_TARGET_SERIAL_H

#include "v850es_fk3_emu_env.h"

#define SIO_RDY_SND		(1u)
#define SIO_RDY_RCV		(2u)



#define INHNO_SIO_TX	 45
#define INTNO_SIO_TX	 45
#define INHNO_SIO_RX	 44
#define INTNO_SIO_RX	 44
#define INTPRI_SIO		 (-4)
#define INTATR_SIO		 TA_NULL


#ifndef TOPPERS_MACRO_ONLY


typedef struct sio_port_control_block SIOPCB;



extern void sio_initialize(intptr_t exinf);


extern SIOPCB *sio_opn_por(ID siopid, intptr_t exinf);


extern void sio_cls_por(SIOPCB *p_siopcb);


extern void sio_tx_isr(intptr_t exinf);
extern void sio_rx_isr(intptr_t exinf);


extern bool_t sio_snd_chr(SIOPCB *siopcb, char c);


extern int_t sio_rcv_chr(SIOPCB *siopcb);


extern void sio_ena_cbr(SIOPCB *siopcb, uint_t cbrtn);


extern void sio_dis_cbr(SIOPCB *siopcb, uint_t cbrtn);


extern void sio_irdy_snd(intptr_t exinf);


extern void sio_irdy_rcv(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */


#endif /* TOPPERS_TARGET_SERIAL_H */
