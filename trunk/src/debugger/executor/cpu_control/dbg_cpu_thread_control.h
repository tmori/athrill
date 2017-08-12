#ifndef _DBG_CPU_THREAD_CONTROL_H_
#define _DBG_CPU_THREAD_CONTROL_H_

extern void cputhr_control_init(void);
extern void cputhr_control_start(void *(*cpu_run) (void *));

/*
 * for debugger
 */
extern void cputhr_control_dbg_waitfor_cpu_stopped(void);
extern void cputhr_control_dbg_wakeup_cpu_and_wait_for_cpu_stopped(void);
extern void cputhr_control_dbg_wait(void);
extern void cputhr_control_dbg_wakeup_cpu(void);

/*
 * for cpu
 */
extern void cputhr_control_cpu_wait(void);
extern void cputhr_control_cpu_wakeup_dbg(void);


#endif /* _DBG_CPU_THREAD_CONTROL_H_ */
