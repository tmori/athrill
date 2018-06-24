/* This file is generated from prc_rename.def by genrename. */

/* This file is included only when prc_rename.h has been included. */
#ifdef TOPPERS_PRC_RENAME_H
#undef TOPPERS_PRC_RENAME_H

/*
 *  start.S
 */
#undef start
#undef hew_io_sim

/*
 *  prc_support.S
 */
#undef dispatch
#undef start_dispatch
#undef exit_and_dispatch
#undef call_exit_kernel
#undef start_r
#undef interrupt_entry
#undef non_kernel_int_entry
#undef exception_entry
#undef default_int_handler_entry
#undef default_exc_handler_entry
#undef iipm_disall

/*
 *  prc_config.c
 */
#undef prc_initialize
#undef prc_exit
#undef x_config_int
#undef default_int_handler
#undef default_exc_handler

#undef ipr_info_tbl
#undef lock_flag
#undef saved_iipm
#undef excnest_count

/*
 *  kernel_cfg.c
 */
#undef int_iipm_tbl

/*
 *  kernel_cfg_asm.S
 */
#undef vectors

/*
 *  trace_config.c
 */
#undef trace_sta_log
#undef trace_wri_log
#undef trace_rea_log
#undef trace_write_0
#undef trace_write_1
#undef trace_write_2
#undef trace_write_3

#undef trace_buffer
#undef trace_count
#undef trace_head
#undef trace_tail
#undef trace_lost

#ifdef TOPPERS_LABEL_ASM

/*
 *  start.S
 */
#undef _start
#undef _hew_io_sim

/*
 *  prc_support.S
 */
#undef _dispatch
#undef _start_dispatch
#undef _exit_and_dispatch
#undef _call_exit_kernel
#undef _start_r
#undef _interrupt_entry
#undef _non_kernel_int_entry
#undef _exception_entry
#undef _default_int_handler_entry
#undef _default_exc_handler_entry
#undef _iipm_disall

/*
 *  prc_config.c
 */
#undef _prc_initialize
#undef _prc_exit
#undef _x_config_int
#undef _default_int_handler
#undef _default_exc_handler

#undef _ipr_info_tbl
#undef _lock_flag
#undef _saved_iipm
#undef _excnest_count

/*
 *  kernel_cfg.c
 */
#undef _int_iipm_tbl

/*
 *  kernel_cfg_asm.S
 */
#undef _vectors

/*
 *  trace_config.c
 */
#undef _trace_sta_log
#undef _trace_wri_log
#undef _trace_rea_log
#undef _trace_write_0
#undef _trace_write_1
#undef _trace_write_2
#undef _trace_write_3

#undef _trace_buffer
#undef _trace_count
#undef _trace_head
#undef _trace_tail
#undef _trace_lost

#endif /* TOPPERS_LABEL_ASM */


#endif /* TOPPERS_PRC_RENAME_H */
