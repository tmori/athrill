/* This file is generated from target_rename.def by genrename. */

#ifndef TOPPERS_TARGET_RENAME_H
#define TOPPERS_TARGET_RENAME_H


/*
 *  target_config.c
 */
#define hew_io_sim_putc				_kernel_hew_io_sim_putc
#define hew_io_sim_snd_chr			_kernel_hew_io_sim_snd_chr
#define hew_io_sim_rcv_chr			_kernel_hew_io_sim_rcv_chr
#define target_initialize			_kernel_target_initialize
#define target_exit					_kernel_target_exit
#define target_fput_initialize		_kernel_target_fput_initialize

/*
 *  target_serial.c
 */
#define sio_hardware_initialize		_kernel_sio_hardware_initialize
#define siopinib_table				_kernel_siopinib_table


#ifdef TOPPERS_LABEL_ASM


/*
 *  target_config.c
 */
#define _hew_io_sim_putc			__kernel_hew_io_sim_putc
#define _hew_io_sim_snd_chr			__kernel_hew_io_sim_snd_chr
#define _hew_io_sim_rcv_chr			__kernel_hew_io_sim_rcv_chr
#define _target_initialize			__kernel_target_initialize
#define _target_exit				__kernel_target_exit
#define _target_fput_initialize		__kernel_target_fput_initialize

/*
 *  target_serial.c
 */
#define _sio_hardware_initialize	__kernel_sio_hardware_initialize
#define _siopinib_table				__kernel_siopinib_table


#endif /* TOPPERS_LABEL_ASM */

#include "sh12a_gcc/prc_rename.h"

#endif /* TOPPERS_TARGET_RENAME_H */
