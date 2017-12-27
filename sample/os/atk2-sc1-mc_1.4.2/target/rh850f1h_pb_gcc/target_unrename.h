/* This file is generated from target_rename.def by genrename. */

/* This file is included only when target_rename.h has been included. */
#ifdef TOPPERS_TARGET_RENAME_H
#undef TOPPERS_TARGET_RENAME_H

/*
 *  target_config.c
 */
#undef target_hardware_initialize
#undef target_initialize
#undef target_exit
#undef target_ici_intno_table
#undef target_ici_intpri_table
#undef target_port_initialize
#undef target_fput_str
#undef target_clock_initialize

/*
 *  trace_dump.c
 */
#undef trace_dump

/*
 *  trace_config.c
 */
#undef log_dsp_enter
#undef log_dsp_leave

/*
 *  target_support.S
 */
#undef target_lmem_init


#ifdef TOPPERS_LABEL_ASM

/*
 *  target_config.c
 */
#undef _target_hardware_initialize
#undef _target_initialize
#undef _target_exit
#undef _target_ici_intno_table
#undef _target_ici_intpri_table
#undef _target_port_initialize
#undef _target_fput_str
#undef _target_clock_initialize

/*
 *  trace_dump.c
 */
#undef _trace_dump

/*
 *  trace_config.c
 */
#undef _log_dsp_enter
#undef _log_dsp_leave

/*
 *  target_support.S
 */
#undef _target_lmem_init


#endif /* TOPPERS_LABEL_ASM */

#include "prc_unrename.h"

#endif /* TOPPERS_TARGET_RENAME_H */
