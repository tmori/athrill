

#ifndef TARGET_SYSSVC_H
#define TARGET_SYSSVC_H


#include "v850es_fk3_emu_env.h"


#include "v850_gcc/prc_syssvc.h"


#ifdef TOPPERS_ENABLE_TRACE
#include "logtrace/trace_config.h"
#endif /* TOPPERS_ENABLE_TRACE */


#define TARGET_NAME    "V850ES-FK3-EmuEnv"


#ifdef PRC_COPYRIGHT
#define TARGET_COPYRIGHT	PRC_COPYRIGHT
#else
#define TARGET_COPYRIGHT	\
			"Copyright (C) 2016 by Eiwa System Management, Inc.\n"
#endif /* PRC_COPYRIGHT */


extern void	target_fput_log(char c);


#define TNUM_PORT		8
#define TNUM_SIOP		8


#define SIO_PORTID		1
#define LOGTASK_PORTID	SIO_PORTID

#endif	/* TARGET_SYSSVC_H */
