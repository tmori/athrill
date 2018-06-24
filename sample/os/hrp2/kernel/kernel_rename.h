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
#define rundom						_kernel_rundom
#define p_ctxdom					_kernel_p_ctxdom
#define p_schedtsk					_kernel_p_schedtsk
#define reqflg						_kernel_reqflg
#define ipmflg						_kernel_ipmflg
#define disdsp						_kernel_disdsp
#define dspflg						_kernel_dspflg
#define ready_queue					_kernel_ready_queue
#define ready_primap				_kernel_ready_primap
#define initialize_task				_kernel_initialize_task
#define search_schedtsk				_kernel_search_schedtsk
#define make_runnable				_kernel_make_runnable
#define make_non_runnable			_kernel_make_non_runnable
#define make_dormant				_kernel_make_dormant
#define make_active					_kernel_make_active
#define change_priority				_kernel_change_priority
#define rotate_ready_queue			_kernel_rotate_ready_queue
#define call_texrtn_stask			_kernel_call_texrtn_stask
#define calltex_stask				_kernel_calltex_stask

/*
 *  memory.c
 */
#define initialize_memory			_kernel_initialize_memory
#define search_meminib				_kernel_search_meminib
#define probe_mem_write				_kernel_probe_mem_write
#define probe_mem_read				_kernel_probe_mem_read
#define probe_stack					_kernel_probe_stack
#define initialize_sections			_kernel_initialize_sections

/*
 *  wait.c
 */
#define make_wait_tmout				_kernel_make_wait_tmout
#define wait_complete				_kernel_wait_complete
#define wait_tmout					_kernel_wait_tmout
#define wait_tmout_ok				_kernel_wait_tmout_ok
#define wait_release				_kernel_wait_release
#define wobj_make_wait				_kernel_wobj_make_wait
#define wobj_make_wait_tmout		_kernel_wobj_make_wait_tmout
#define init_wait_queue				_kernel_init_wait_queue

/*
 *  time_event.c
 */
#define current_time				_kernel_current_time
#define min_time					_kernel_min_time
#define next_time					_kernel_next_time
#define next_subtime				_kernel_next_subtime
#define last_index					_kernel_last_index
#define initialize_tmevt			_kernel_initialize_tmevt
#define tmevt_up					_kernel_tmevt_up
#define tmevt_down					_kernel_tmevt_down
#define tmevtb_insert				_kernel_tmevtb_insert
#define tmevtb_delete				_kernel_tmevtb_delete
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
#define mtxhook_check_ceilpri		_kernel_mtxhook_check_ceilpri
#define mtxhook_scan_ceilmtx		_kernel_mtxhook_scan_ceilmtx
#define mtxhook_release_all			_kernel_mtxhook_release_all
#define initialize_mutex			_kernel_initialize_mutex
#define mutex_check_ceilpri			_kernel_mutex_check_ceilpri
#define mutex_scan_ceilmtx			_kernel_mutex_scan_ceilmtx
#define mutex_calc_priority			_kernel_mutex_calc_priority
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
#define call_cychdr					_kernel_call_cychdr

/*
 *  alarm.c
 */
#define initialize_alarm			_kernel_initialize_alarm
#define call_almhdr					_kernel_call_almhdr

/*
 *  overrun.c
 */
#define initialize_overrun			_kernel_initialize_overrun
#define ovrtimer_start				_kernel_ovrtimer_start
#define ovrtimer_stop				_kernel_ovrtimer_stop
#define call_ovrhdr					_kernel_call_ovrhdr

/*
 *  interrupt.c
 */
#define initialize_interrupt		_kernel_initialize_interrupt

/*
 *  exception.c
 */
#define initialize_exception		_kernel_initialize_exception

/*
 *  svc_table.c
 */
#define svc_table					_kernel_svc_table

/*
 *  kernel_cfg.c
 */
#define initialize_object			_kernel_initialize_object
#define call_inirtn					_kernel_call_inirtn
#define call_terrtn					_kernel_call_terrtn
#define tmax_domid					_kernel_tmax_domid
#define dominib_table				_kernel_dominib_table
#define dominib_kernel				_kernel_dominib_kernel
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
#define dtqcb_table					_kernel_dtqcb_table
#define dtqinib_table				_kernel_dtqinib_table
#define tmax_pdqid					_kernel_tmax_pdqid
#define pdqcb_table					_kernel_pdqcb_table
#define pdqinib_table				_kernel_pdqinib_table
#define tmax_mtxid					_kernel_tmax_mtxid
#define mtxcb_table					_kernel_mtxcb_table
#define mtxinib_table				_kernel_mtxinib_table
#define tmax_mpfid					_kernel_tmax_mpfid
#define mpfinib_table				_kernel_mpfinib_table
#define mpfcb_table					_kernel_mpfcb_table
#define tmax_cycid					_kernel_tmax_cycid
#define cycinib_table				_kernel_cycinib_table
#define cyccb_table					_kernel_cyccb_table
#define tmax_almid					_kernel_tmax_almid
#define alminib_table				_kernel_alminib_table
#define almcb_table					_kernel_almcb_table
#define ovrinib						_kernel_ovrinib
#define sysstat_acvct				_kernel_sysstat_acvct
#define tnum_meminib				_kernel_tnum_meminib
#define memtop_table				_kernel_memtop_table
#define meminib_table				_kernel_meminib_table
#define tnum_meminfo				_kernel_tnum_meminfo
#define meminfo_table				_kernel_meminfo_table
#define tnum_inhno					_kernel_tnum_inhno
#define inhinib_table				_kernel_inhinib_table
#define tnum_intno					_kernel_tnum_intno
#define intinib_table				_kernel_intinib_table
#define tnum_excno					_kernel_tnum_excno
#define excinib_table				_kernel_excinib_table
#define tmax_fncd					_kernel_tmax_fncd
#define svcinib_table				_kernel_svcinib_table
#define tmevt_heap					_kernel_tmevt_heap
#define istksz						_kernel_istksz
#define istk						_kernel_istk
#define istkpt						_kernel_istkpt
#define tnum_datasec				_kernel_tnum_datasec
#define datasecinib_table			_kernel_datasecinib_table
#define tnum_bsssec					_kernel_tnum_bsssec
#define bsssecinib_table			_kernel_bsssecinib_table


#ifdef TOPPERS_LABEL_ASM

/*
 *  startup.c
 */
#define _kerflg						__kernel_kerflg
#define _exit_kernel				__kernel_exit_kernel

/*
 *  task.c
 */
#define _p_runtsk					__kernel_p_runtsk
#define _rundom						__kernel_rundom
#define _p_ctxdom					__kernel_p_ctxdom
#define _p_schedtsk					__kernel_p_schedtsk
#define _reqflg						__kernel_reqflg
#define _ipmflg						__kernel_ipmflg
#define _disdsp						__kernel_disdsp
#define _dspflg						__kernel_dspflg
#define _ready_queue				__kernel_ready_queue
#define _ready_primap				__kernel_ready_primap
#define _initialize_task			__kernel_initialize_task
#define _search_schedtsk			__kernel_search_schedtsk
#define _make_runnable				__kernel_make_runnable
#define _make_non_runnable			__kernel_make_non_runnable
#define _make_dormant				__kernel_make_dormant
#define _make_active				__kernel_make_active
#define _change_priority			__kernel_change_priority
#define _rotate_ready_queue			__kernel_rotate_ready_queue
#define _call_texrtn_stask			__kernel_call_texrtn_stask
#define _calltex_stask				__kernel_calltex_stask

/*
 *  memory.c
 */
#define _initialize_memory			__kernel_initialize_memory
#define _search_meminib				__kernel_search_meminib
#define _probe_mem_write			__kernel_probe_mem_write
#define _probe_mem_read				__kernel_probe_mem_read
#define _probe_stack				__kernel_probe_stack
#define _initialize_sections		__kernel_initialize_sections

/*
 *  wait.c
 */
#define _make_wait_tmout			__kernel_make_wait_tmout
#define _wait_complete				__kernel_wait_complete
#define _wait_tmout					__kernel_wait_tmout
#define _wait_tmout_ok				__kernel_wait_tmout_ok
#define _wait_release				__kernel_wait_release
#define _wobj_make_wait				__kernel_wobj_make_wait
#define _wobj_make_wait_tmout		__kernel_wobj_make_wait_tmout
#define _init_wait_queue			__kernel_init_wait_queue

/*
 *  time_event.c
 */
#define _current_time				__kernel_current_time
#define _min_time					__kernel_min_time
#define _next_time					__kernel_next_time
#define _next_subtime				__kernel_next_subtime
#define _last_index					__kernel_last_index
#define _initialize_tmevt			__kernel_initialize_tmevt
#define _tmevt_up					__kernel_tmevt_up
#define _tmevt_down					__kernel_tmevt_down
#define _tmevtb_insert				__kernel_tmevtb_insert
#define _tmevtb_delete				__kernel_tmevtb_delete
#define _tmevt_lefttim				__kernel_tmevt_lefttim
#define _signal_time				__kernel_signal_time

/*
 *  semaphore.c
 */
#define _initialize_semaphore		__kernel_initialize_semaphore

/*
 *  eventflag.c
 */
#define _initialize_eventflag		__kernel_initialize_eventflag
#define _check_flg_cond				__kernel_check_flg_cond

/*
 *  dataqueue.c
 */
#define _initialize_dataqueue		__kernel_initialize_dataqueue
#define _enqueue_data				__kernel_enqueue_data
#define _force_enqueue_data			__kernel_force_enqueue_data
#define _dequeue_data				__kernel_dequeue_data
#define _send_data					__kernel_send_data
#define _force_send_data			__kernel_force_send_data
#define _receive_data				__kernel_receive_data

/*
 *  pridataq.c
 */
#define _initialize_pridataq		__kernel_initialize_pridataq
#define _enqueue_pridata			__kernel_enqueue_pridata
#define _dequeue_pridata			__kernel_dequeue_pridata
#define _send_pridata				__kernel_send_pridata
#define _receive_pridata			__kernel_receive_pridata

/*
 *  mutex.c
 */
#define _mtxhook_check_ceilpri		__kernel_mtxhook_check_ceilpri
#define _mtxhook_scan_ceilmtx		__kernel_mtxhook_scan_ceilmtx
#define _mtxhook_release_all		__kernel_mtxhook_release_all
#define _initialize_mutex			__kernel_initialize_mutex
#define _mutex_check_ceilpri		__kernel_mutex_check_ceilpri
#define _mutex_scan_ceilmtx			__kernel_mutex_scan_ceilmtx
#define _mutex_calc_priority		__kernel_mutex_calc_priority
#define _mutex_release				__kernel_mutex_release
#define _mutex_release_all			__kernel_mutex_release_all

/*
 *  mempfix.c
 */
#define _initialize_mempfix			__kernel_initialize_mempfix
#define _get_mpf_block				__kernel_get_mpf_block

/*
 *  cyclic.c
 */
#define _initialize_cyclic			__kernel_initialize_cyclic
#define _call_cychdr				__kernel_call_cychdr

/*
 *  alarm.c
 */
#define _initialize_alarm			__kernel_initialize_alarm
#define _call_almhdr				__kernel_call_almhdr

/*
 *  overrun.c
 */
#define _initialize_overrun			__kernel_initialize_overrun
#define _ovrtimer_start				__kernel_ovrtimer_start
#define _ovrtimer_stop				__kernel_ovrtimer_stop
#define _call_ovrhdr				__kernel_call_ovrhdr

/*
 *  interrupt.c
 */
#define _initialize_interrupt		__kernel_initialize_interrupt

/*
 *  exception.c
 */
#define _initialize_exception		__kernel_initialize_exception

/*
 *  svc_table.c
 */
#define _svc_table					__kernel_svc_table

/*
 *  kernel_cfg.c
 */
#define _initialize_object			__kernel_initialize_object
#define _call_inirtn				__kernel_call_inirtn
#define _call_terrtn				__kernel_call_terrtn
#define _tmax_domid					__kernel_tmax_domid
#define _dominib_table				__kernel_dominib_table
#define _dominib_kernel				__kernel_dominib_kernel
#define _tmax_tskid					__kernel_tmax_tskid
#define _tinib_table				__kernel_tinib_table
#define _torder_table				__kernel_torder_table
#define _tcb_table					__kernel_tcb_table
#define _tmax_semid					__kernel_tmax_semid
#define _seminib_table				__kernel_seminib_table
#define _semcb_table				__kernel_semcb_table
#define _tmax_flgid					__kernel_tmax_flgid
#define _flginib_table				__kernel_flginib_table
#define _flgcb_table				__kernel_flgcb_table
#define _tmax_dtqid					__kernel_tmax_dtqid
#define _dtqcb_table				__kernel_dtqcb_table
#define _dtqinib_table				__kernel_dtqinib_table
#define _tmax_pdqid					__kernel_tmax_pdqid
#define _pdqcb_table				__kernel_pdqcb_table
#define _pdqinib_table				__kernel_pdqinib_table
#define _tmax_mtxid					__kernel_tmax_mtxid
#define _mtxcb_table				__kernel_mtxcb_table
#define _mtxinib_table				__kernel_mtxinib_table
#define _tmax_mpfid					__kernel_tmax_mpfid
#define _mpfinib_table				__kernel_mpfinib_table
#define _mpfcb_table				__kernel_mpfcb_table
#define _tmax_cycid					__kernel_tmax_cycid
#define _cycinib_table				__kernel_cycinib_table
#define _cyccb_table				__kernel_cyccb_table
#define _tmax_almid					__kernel_tmax_almid
#define _alminib_table				__kernel_alminib_table
#define _almcb_table				__kernel_almcb_table
#define _ovrinib					__kernel_ovrinib
#define _sysstat_acvct				__kernel_sysstat_acvct
#define _tnum_meminib				__kernel_tnum_meminib
#define _memtop_table				__kernel_memtop_table
#define _meminib_table				__kernel_meminib_table
#define _tnum_meminfo				__kernel_tnum_meminfo
#define _meminfo_table				__kernel_meminfo_table
#define _tnum_inhno					__kernel_tnum_inhno
#define _inhinib_table				__kernel_inhinib_table
#define _tnum_intno					__kernel_tnum_intno
#define _intinib_table				__kernel_intinib_table
#define _tnum_excno					__kernel_tnum_excno
#define _excinib_table				__kernel_excinib_table
#define _tmax_fncd					__kernel_tmax_fncd
#define _svcinib_table				__kernel_svcinib_table
#define _tmevt_heap					__kernel_tmevt_heap
#define _istksz						__kernel_istksz
#define _istk						__kernel_istk
#define _istkpt						__kernel_istkpt
#define _tnum_datasec				__kernel_tnum_datasec
#define _datasecinib_table			__kernel_datasecinib_table
#define _tnum_bsssec				__kernel_tnum_bsssec
#define _bsssecinib_table			__kernel_bsssecinib_table


#endif /* TOPPERS_LABEL_ASM */

#include "target_rename.h"

#endif /* TOPPERS_KERNEL_RENAME_H */
