/* This file is generated from prc_rename.def by genrename. */

#ifndef TOPPERS_PRC_RENAME_H
#define TOPPERS_PRC_RENAME_H

#define start_dispatch				_kernel_start_dispatch
#define exit_and_dispatch			_kernel_exit_and_dispatch
#define call_exit_kernel			_kernel_call_exit_kernel
#define intnest						_kernel_intnest
#define current_intpri				_kernel_current_intpri
#define lock_flag					_kernel_lock_flag
#define default_int_handler			_kernel_default_int_handler
#define default_exc_handler			_kernel_default_exc_handler
#define start_r						_kernel_start_r
#define dispatch					_kernel_dispatch
#define start_dispatch				_kernel_start_dispatch
#define x_config_int				_kernel_x_config_int

#ifdef TOPPERS_LABEL_ASM

#define _start_dispatch				__kernel_start_dispatch
#define _exit_and_dispatch			__kernel_exit_and_dispatch
#define _call_exit_kernel			__kernel_call_exit_kernel
#define _intnest					__kernel_intnest
#define _current_intpri				__kernel_current_intpri
#define _lock_flag					__kernel_lock_flag
#define _default_int_handler		__kernel_default_int_handler
#define _default_exc_handler		__kernel_default_exc_handler
#define _start_r					__kernel_start_r
#define _dispatch					__kernel_dispatch
#define _start_dispatch				__kernel_start_dispatch
#define _x_config_int				__kernel_x_config_int

#endif /* TOPPERS_LABEL_ASM */


#endif /* TOPPERS_PRC_RENAME_H */
