/* This file is generated from target_rename.def by genrename. */

#ifndef TOPPERS_TARGET_RENAME_H
#define TOPPERS_TARGET_RENAME_H

/*
 *  target_config.c
 */
#define target_hardware_initialize	kernel_target_hardware_initialize
#define target_initialize			kernel_target_initialize
#define target_exit					kernel_target_exit

/*
 *  trace_dump.c
 */
#define trace_dump					kernel_trace_dump

/*
 *  trace_config.c
 */
#define log_dsp_enter				kernel_log_dsp_enter
#define log_dsp_leave				kernel_log_dsp_leave


#ifdef TOPPERS_LABEL_ASM

/*
 *  target_config.c
 */
#define _target_hardware_initialize	_kernel_target_hardware_initialize
#define _target_initialize			_kernel_target_initialize
#define _target_exit				_kernel_target_exit

/*
 *  trace_dump.c
 */
#define _trace_dump					_kernel_trace_dump

/*
 *  trace_config.c
 */
#define _log_dsp_enter				_kernel_log_dsp_enter
#define _log_dsp_leave				_kernel_log_dsp_leave


#endif /* TOPPERS_LABEL_ASM */

#include "prc_rename.h"

#endif /* TOPPERS_TARGET_RENAME_H */
