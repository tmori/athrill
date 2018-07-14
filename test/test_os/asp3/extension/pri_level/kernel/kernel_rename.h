/* This file is generated from kernel_rename.def by genrename. */

#ifndef TOPPERS_KERNEL_RENAME_H
#define TOPPERS_KERNEL_RENAME_H

/*
 *  startup.c
 */
#define kerflg						_kernel_kerflg
#define exit_kernel					_kernel_exit_kernel

/*
 *  task.c
 */
#define p_runtsk					_kernel_p_runtsk
#define p_schedtsk					_kernel_p_schedtsk
#define enadsp						_kernel_enadsp
#define dspflg						_kernel_dspflg
#define ready_queue					_kernel_ready_queue
#define ready_primap				_kernel_ready_primap
#define ready_primap1				_kernel_ready_primap1
#define ready_primap2				_kernel_ready_primap2
#define initialize_task				_kernel_initialize_task
#define search_schedtsk				_kernel_search_schedtsk
#define make_runnable				_kernel_make_runnable
#define make_non_runnable			_kernel_make_non_runnable
#define make_dormant				_kernel_make_dormant
#define make_active					_kernel_make_active
#define change_priority				_kernel_change_priority
#define rotate_ready_queue			_kernel_rotate_ready_queue
#define task_terminate				_kernel_task_terminate

/*
 *  taskhook.c
 */
#define mtxhook_check_ceilpri		_kernel_mtxhook_check_ceilpri
#define mtxhook_scan_ceilmtx		_kernel_mtxhook_scan_ceilmtx
#define mtxhook_release_all			_kernel_mtxhook_release_all

/*
 *  wait.c
 */
#define make_wait_tmout				_kernel_make_wait_tmout
#define wait_complete				_kernel_wait_complete
#define wait_tmout					_kernel_wait_tmout
#define wait_tmout_ok				_kernel_wait_tmout_ok
#define wobj_make_wait				_kernel_wobj_make_wait
#define wobj_make_wait_tmout		_kernel_wobj_make_wait_tmout
#define init_wait_queue				_kernel_init_wait_queue

/*
 *  time_event.c
 */
#define boundary_evttim				_kernel_boundary_evttim
#define current_evttim				_kernel_current_evttim
#define current_hrtcnt				_kernel_current_hrtcnt
#define monotonic_evttim			_kernel_monotonic_evttim
#define systim_offset				_kernel_systim_offset
#define in_signal_time				_kernel_in_signal_time
#define initialize_tmevt			_kernel_initialize_tmevt
#define tmevt_up					_kernel_tmevt_up
#define tmevt_down					_kernel_tmevt_down
#define update_current_evttim		_kernel_update_current_evttim
#define set_hrt_event				_kernel_set_hrt_event
#define tmevtb_register				_kernel_tmevtb_register
#define tmevtb_enqueue				_kernel_tmevtb_enqueue
#define tmevtb_dequeue				_kernel_tmevtb_dequeue
#define check_adjtim				_kernel_check_adjtim
#define tmevt_lefttim				_kernel_tmevt_lefttim
#define signal_time					_kernel_signal_time

/*
 *  semaphore.c
 */
#define initialize_semaphore		_kernel_initialize_semaphore

/*
 *  eventflag.c
 */
#define initialize_eventflag		_kernel_initialize_eventflag
#define check_flg_cond				_kernel_check_flg_cond

/*
 *  dataqueue.c
 */
#define initialize_dataqueue		_kernel_initialize_dataqueue
#define enqueue_data				_kernel_enqueue_data
#define force_enqueue_data			_kernel_force_enqueue_data
#define dequeue_data				_kernel_dequeue_data
#define send_data					_kernel_send_data
#define force_send_data				_kernel_force_send_data
#define receive_data				_kernel_receive_data

/*
 *  pridataq.c
 */
#define initialize_pridataq			_kernel_initialize_pridataq
#define enqueue_pridata				_kernel_enqueue_pridata
#define dequeue_pridata				_kernel_dequeue_pridata
#define send_pridata				_kernel_send_pridata
#define receive_pridata				_kernel_receive_pridata

/*
 *  mutex.c
 */
#define initialize_mutex			_kernel_initialize_mutex
#define mutex_check_ceilpri			_kernel_mutex_check_ceilpri
#define mutex_scan_ceilmtx			_kernel_mutex_scan_ceilmtx
#define mutex_drop_priority			_kernel_mutex_drop_priority
#define mutex_acquire				_kernel_mutex_acquire
#define mutex_release				_kernel_mutex_release
#define mutex_release_all			_kernel_mutex_release_all

/*
 *  mempfix.c
 */
#define initialize_mempfix			_kernel_initialize_mempfix
#define get_mpf_block				_kernel_get_mpf_block

/*
 *  cyclic.c
 */
#define initialize_cyclic			_kernel_initialize_cyclic
#define call_cyclic					_kernel_call_cyclic

/*
 *  alarm.c
 */
#define initialize_alarm			_kernel_initialize_alarm
#define call_alarm					_kernel_call_alarm

/*
 *  interrupt.c
 */
#define initialize_interrupt		_kernel_initialize_interrupt

/*
 *  exception.c
 */
#define initialize_exception		_kernel_initialize_exception

/*
 *  kernel_cfg.c
 */
#define initialize_object			_kernel_initialize_object
#define call_inirtn					_kernel_call_inirtn
#define call_terrtn					_kernel_call_terrtn
#define tmax_tskid					_kernel_tmax_tskid
#define tinib_table					_kernel_tinib_table
#define torder_table				_kernel_torder_table
#define tcb_table					_kernel_tcb_table
#define tmax_semid					_kernel_tmax_semid
#define seminib_table				_kernel_seminib_table
#define semcb_table					_kernel_semcb_table
#define tmax_flgid					_kernel_tmax_flgid
#define flginib_table				_kernel_flginib_table
#define flgcb_table					_kernel_flgcb_table
#define tmax_dtqid					_kernel_tmax_dtqid
#define dtqinib_table				_kernel_dtqinib_table
#define dtqcb_table					_kernel_dtqcb_table
#define tmax_pdqid					_kernel_tmax_pdqid
#define pdqinib_table				_kernel_pdqinib_table
#define pdqcb_table					_kernel_pdqcb_table
#define tmax_mtxid					_kernel_tmax_mtxid
#define mtxinib_table				_kernel_mtxinib_table
#define mtxcb_table					_kernel_mtxcb_table
#define tmax_mpfid					_kernel_tmax_mpfid
#define mpfinib_table				_kernel_mpfinib_table
#define mpfcb_table					_kernel_mpfcb_table
#define tmax_cycid					_kernel_tmax_cycid
#define cycinib_table				_kernel_cycinib_table
#define cyccb_table					_kernel_cyccb_table
#define tmax_almid					_kernel_tmax_almid
#define alminib_table				_kernel_alminib_table
#define almcb_table					_kernel_almcb_table
#define tnum_def_inhno				_kernel_tnum_def_inhno
#define inhinib_table				_kernel_inhinib_table
#define tnum_cfg_intno				_kernel_tnum_cfg_intno
#define intinib_table				_kernel_intinib_table
#define tnum_def_excno				_kernel_tnum_def_excno
#define excinib_table				_kernel_excinib_table
#define tmevt_heap					_kernel_tmevt_heap
#define istksz						_kernel_istksz
#define istk						_kernel_istk
#define istkpt						_kernel_istkpt


#include "target_rename.h"

#endif /* TOPPERS_KERNEL_RENAME_H */
