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

/*
 *  trace_dump.c
 */
#undef trace_dump

/*
 *  trace_config.c
 */
#undef log_dsp_enter
#undef log_dsp_leave


#ifdef TOPPERS_LABEL_ASM

/*
 *  target_config.c
 */
#undef _target_hardware_initialize
#undef _target_initialize
#undef _target_exit

/*
 *  trace_dump.c
 */
#undef _trace_dump

/*
 *  trace_config.c
 */
#undef _log_dsp_enter
#undef _log_dsp_leave


#endif /* TOPPERS_LABEL_ASM */

#include "prc_unrename.h"

#endif /* TOPPERS_TARGET_RENAME_H */
