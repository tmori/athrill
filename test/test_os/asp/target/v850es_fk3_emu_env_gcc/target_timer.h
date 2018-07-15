

#ifndef TARGET_TIMER_H
#define TARGET_TIMER_H

#include <sil.h>
#include "v850es_fk3_emu_env.h"


#define TIMER_CLOCK ((PCLOCK/4u) / 1000)


#define INHNO_TIMER 	38
#define INTNO_TIMER 	38
#define INTPRI_TIMER	(-6)
#define INTATR_TIMER	TA_NULL


typedef uint32_t    CLOCK;


#define TO_CLOCK(nume, deno)    ((CLOCK)(TIMER_CLOCK * (nume) / (deno)))
#define TO_USEC(clock)          (((SYSUTM) clock) * 1000U / TIMER_CLOCK)


#define MAX_CLOCK    ((CLOCK) 0xffffU)

#ifndef TOPPERS_MACRO_ONLY

extern void target_timer_initialize(intptr_t exinf);


extern void target_timer_terminate(intptr_t exinf);


#define target_timer_get_current()	(0)


Inline bool_t target_timer_probe_int(void)
{
	return x_probe_int(INTNO_TIMER);
}


extern void target_timer_handler(void);

#endif	/* TOPPERS_MACRO_ONLY */

#endif	/* TARGET_TIMER_H */
