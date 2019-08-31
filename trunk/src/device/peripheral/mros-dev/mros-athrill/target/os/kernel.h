#ifndef _KERNEL_H_
#define _KERNEL_H_

#include <string.h>
#include <stdlib.h>
#include <pthread.h>
typedef int ID;
typedef int PRI;

extern int slp_tsk(void);
extern int act_tsk(ID);
extern int get_tid(ID *p_tskid);
extern int chg_pri(ID tskid, PRI tskpri);
extern int iwup_tsk(ID tskid);
extern int get_pri(ID tskid, PRI *p_tskpri);
extern int dly_tsk(int dlytim);
extern int wup_tsk(ID tskid);

extern pthread_mutex_t mutex_lock;
extern pthread_cond_t cond_wait;

#include "kernel_cfg.h"


typedef struct {
	int tskid;
	int lockCount;
} OsSaveLockType;
extern void os_save_unlock(OsSaveLockType *save);
extern void os_restore_lock(OsSaveLockType *save);
extern void os_lock_recursive(void);
extern void os_unlock_recursive(void);

#define TMAX_TPRI 1

#endif
