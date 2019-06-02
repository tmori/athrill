#ifndef _ATHRILL_MPTHREAD_H_
#define _ATHRILL_MPTHREAD_H_

#include "std_types.h"
#include "std_errno.h"

typedef uint32 MpthrIdType;
typedef struct {
    Std_ReturnType (*do_init) (MpthrIdType id);
    Std_ReturnType (*do_proc) (MpthrIdType id);
} MpthrOperationType;

/*
 * Manager api
 */
extern Std_ReturnType mpthread_init(void);
extern Std_ReturnType mpthread_register(MpthrIdType *id, MpthrOperationType *op);

/*
 * Thread api
 */
typedef enum {
    MPTHR_STATUS_INITIALIZING = 0,
    MPTHR_STATUS_RUNNING,
    MPTHR_STATUS_WAITING,
} MpthrStatusType;
extern void mpthread_lock(MpthrIdType id);
extern void mpthread_unlock(MpthrIdType id);
extern MpthrStatusType mpthread_get_status(MpthrIdType id);
extern Std_ReturnType mpthread_start_proc(MpthrIdType id);
extern Std_ReturnType mpthread_stop_proc(MpthrIdType id);

#endif /* _ATHRILL_MPTHREAD_H_ */