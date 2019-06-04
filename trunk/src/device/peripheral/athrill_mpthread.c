#include "athrill_mpthread.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include "assert.h"
#include <time.h>

typedef struct {
    MpthrIdType         id;
    uint32              timeout;
    MpthrStatusType     status;
    MpthrOperationType  *op;
	pthread_t           thread;
    pthread_mutex_t     mutex;
    pthread_cond_t      cond;
} MpthrInfoType;

static uint32 mpthread_num = 0;
static pthread_mutex_t     mpthread_mutex = PTHREAD_MUTEX_INITIALIZER;
static MpthrInfoType *mpthread_info = NULL;


static void *mpthread_run(void *arg)
{
    MpthrInfoType *infop = ((MpthrInfoType*)arg);
    MpthrIdType id = infop->id;
    struct timespec abstime;

    mpthread_info[id].op->do_init(id);

    mpthread_lock(id);
    while (TRUE) {
        if (mpthread_info[id].status != MPTHR_STATUS_RUNNING) {
            if (mpthread_info[id].timeout == 0) {
                pthread_cond_wait(&mpthread_info[id].cond, &mpthread_info[id].mutex);
            }
            else {
                abstime.tv_sec = 0;
                abstime.tv_nsec = mpthread_info[id].timeout * 1000;
                pthread_cond_timedwait(&mpthread_info[id].cond, &mpthread_info[id].mutex, &abstime);
            }
            continue;
        }
        mpthread_unlock(id);

        mpthread_info[id].op->do_proc(id);

        mpthread_lock(id);
    }
    return NULL;
}

/*
 * Manager api
 */
Std_ReturnType mpthread_init(void)
{
    //nothing to do
    return STD_E_OK;
}
Std_ReturnType mpthread_register(MpthrIdType *id, MpthrOperationType *op)
{
    MpthrIdType new_id;
    
    pthread_mutex_lock(&mpthread_mutex);
    new_id = mpthread_num;
    mpthread_num++;
    MpthrInfoType *p = realloc(mpthread_info, sizeof(MpthrInfoType) * mpthread_num);
    ASSERT(p != NULL);
    mpthread_info = p;
    pthread_mutex_unlock(&mpthread_mutex);

    mpthread_info[new_id].id = new_id;
    mpthread_info[new_id].timeout = 0;
    mpthread_info[new_id].status = MPTHR_STATUS_INITIALIZING;
    mpthread_info[new_id].op = op;

    pthread_mutex_init(&mpthread_info[new_id].mutex, NULL);
    pthread_cond_init(&mpthread_info[new_id].cond, NULL);
    pthread_create(&mpthread_info[new_id].thread , NULL , mpthread_run , (void*)&mpthread_info[new_id]);

    *id = new_id;
    return STD_E_OK;
}

/*
 * Thread api
 */
void mpthread_lock(MpthrIdType id)
{
    if (id >= mpthread_num) {
        return;
    }
    pthread_mutex_lock(&mpthread_info[id].mutex);
    return;
}

void mpthread_unlock(MpthrIdType id)
{
    if (id >= mpthread_num) {
        return;
    }
    pthread_mutex_unlock(&mpthread_info[id].mutex);
    return;
}

MpthrStatusType mpthread_get_status(MpthrIdType id)
{
    if (id >= mpthread_num) {
        return STD_E_INVALID;
    }
    return mpthread_info[id].status;
}

Std_ReturnType mpthread_start_proc(MpthrIdType id)
{
    if (id >= mpthread_num) {
        return STD_E_INVALID;
    }
    mpthread_lock(id);
    mpthread_info[id].status = MPTHR_STATUS_RUNNING;
    pthread_cond_signal(&mpthread_info[id].cond);
    mpthread_unlock(id);
    return STD_E_OK;
}
Std_ReturnType mpthread_wait_proc(MpthrIdType id)
{
    if (id >= mpthread_num) {
        return STD_E_INVALID;
    }
    mpthread_lock(id);
    mpthread_info[id].timeout = 0;
    mpthread_info[id].status = MPTHR_STATUS_WAITING;
    mpthread_unlock(id);
    return STD_E_OK;
}
Std_ReturnType mpthread_timedwait_proc(MpthrIdType id, sint32 timeout)
{
    if (id >= mpthread_num) {
        return STD_E_INVALID;
    }
    mpthread_lock(id);
    mpthread_info[id].timeout = timeout;
    mpthread_info[id].status = MPTHR_STATUS_WAITING;
    mpthread_unlock(id);
    return STD_E_OK;
}