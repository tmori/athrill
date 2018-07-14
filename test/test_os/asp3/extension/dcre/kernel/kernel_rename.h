/* This file is generated from kernel_rename.def by genrename. */

#ifndef TOPPERS_KERNEL_RENAME_H
#define TOPPERS_KERNEL_RENAME_H

/*
 *  startup.c
 */
#define kerflg						_kernel_kerflg
#define exit_kernel					_kernel_exit_kernel
#define initialize_kmm				_kernel_initialize_kmm
#define kernel_malloc				_kernel_kernel_malloc
#define kernel_free					_kernel_kernel_free

/*
 *  task.c
 */
#define p_runtsk					_kernel_p_runtsk
#define p_schedtsk					_kernel_p_schedtsk
#define enadsp						_kernel_enadsp
#define dspflg						_kernel_dspflg
#define ready_queue					_kernel_ready_queue
#define ready_primap				_kernel_ready_primap
#define free_tcb					_kernel_free_tcb
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
#define free_semcb					_kernel_free_semcb
#define initialize_semaphore		_kernel_initialize_semaphore

/*
 *  eventflag.c
 */
#define free_flgcb					_kernel_free_flgcb
#define initialize_eventflag		_kernel_initialize_eventflag
#define check_flg_cond				_kernel_check_flg_cond

/*
 *  dataqueue.c
 */
#define free_dtqcb					_kernel_free_dtqcb
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
#define free_pdqcb					_kernel_free_pdqcb
#define initialize_pridataq			_kernel_initialize_pridataq
#define enqueue_pridata				_kernel_enqueue_pridata
#define dequeue_pridata				_kernel_dequeue_pridata
#define send_pridata				_kernel_send_pridata
#define receive_pridata				_kernel_receive_pridata

/*
 *  mutex.c
 */
#define free_mtxcb					_kernel_free_mtxcb
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
#define free_mpfcb					_kernel_free_mpfcb
#define initialize_mempfix			_kernel_initialize_mempfix
#define get_mpf_block				_kernel_get_mpf_block

/*
 *  time_manage.c
 */
#define check_nfyinfo				_kernel_check_nfyinfo
#define notify_handler				_kernel_notify_handler

/*
 *  cyclic.c
 */
#define free_cyccb					_kernel_free_cyccb
#define initialize_cyclic			_kernel_initialize_cyclic
#define call_cyclic					_kernel_call_cyclic

/*
 *  alarm.c
 */
#define free_almcb					_kernel_free_almcb
#define initialize_alarm			_kernel_initialize_alarm
#define call_alarm					_kernel_call_alarm

/*
 *  interrupt.c
 */
#define free_isrcb					_kernel_free_isrcb
#define initialize_isr				_kernel_initialize_isr
#define call_isr					_kernel_call_isr
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
#define tmax_stskid					_kernel_tmax_stskid
#define tinib_table					_kernel_tinib_table
#define atinib_table				_kernel_atinib_table
#define torder_table				_kernel_torder_table
#define tcb_table					_kernel_tcb_table
#define tmax_semid					_kernel_tmax_semid
#define tmax_ssemid					_kernel_tmax_ssemid
#define seminib_table				_kernel_seminib_table
#define aseminib_table				_kernel_aseminib_table
#define semcb_table					_kernel_semcb_table
#define tmax_flgid					_kernel_tmax_flgid
#define tmax_sflgid					_kernel_tmax_sflgid
#define flginib_table				_kernel_flginib_table
#define aflginib_table				_kernel_aflginib_table
#define flgcb_table					_kernel_flgcb_table
#define tmax_dtqid					_kernel_tmax_dtqid
#define tmax_sdtqid					_kernel_tmax_sdtqid
#define dtqinib_table				_kernel_dtqinib_table
#define adtqinib_table				_kernel_adtqinib_table
#define dtqcb_table					_kernel_dtqcb_table
#define tmax_pdqid					_kernel_tmax_pdqid
#define tmax_spdqid					_kernel_tmax_spdqid
#define pdqinib_table				_kernel_pdqinib_table
#define apdqinib_table				_kernel_apdqinib_table
#define pdqcb_table					_kernel_pdqcb_table
#define tmax_mtxid					_kernel_tmax_mtxid
#define tmax_smtxid					_kernel_tmax_smtxid
#define mtxinib_table				_kernel_mtxinib_table
#define amtxinib_table				_kernel_amtxinib_table
#define mtxcb_table					_kernel_mtxcb_table
#define tmax_mpfid					_kernel_tmax_mpfid
#define tmax_smpfid					_kernel_tmax_smpfid
#define mpfinib_table				_kernel_mpfinib_table
#define ampfinib_table				_kernel_ampfinib_table
#define mpfcb_table					_kernel_mpfcb_table
#define tmax_cycid					_kernel_tmax_cycid
#define tmax_scycid					_kernel_tmax_scycid
#define cycinib_table				_kernel_cycinib_table
#define acycinib_table				_kernel_acycinib_table
#define acyc_nfyinfo_table			_kernel_acyc_nfyinfo_table
#define cyccb_table					_kernel_cyccb_table
#define tmax_almid					_kernel_tmax_almid
#define tmax_salmid					_kernel_tmax_salmid
#define alminib_table				_kernel_alminib_table
#define aalminib_table				_kernel_aalminib_table
#define aalm_nfyinfo_table			_kernel_aalm_nfyinfo_table
#define almcb_table					_kernel_almcb_table
#define tnum_isr_queue				_kernel_tnum_isr_queue
#define isr_queue_list				_kernel_isr_queue_list
#define isr_queue_table				_kernel_isr_queue_table
#define tmax_isrid					_kernel_tmax_isrid
#define tmax_sisrid					_kernel_tmax_sisrid
#define isrinib_table				_kernel_isrinib_table
#define aisrinib_table				_kernel_aisrinib_table
#define isrorder_table				_kernel_isrorder_table
#define isrcb_table					_kernel_isrcb_table
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
#define kmmsz						_kernel_kmmsz
#define kmm							_kernel_kmm


#include "target_rename.h"

#endif /* TOPPERS_KERNEL_RENAME_H */
