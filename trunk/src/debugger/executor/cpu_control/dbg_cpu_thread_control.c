#include "cpu_control/dbg_cpu_thread_control.h"
#include "std_types.h"
#include<windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>


typedef enum {
	THREAD_STATE_RUNNING,
	THREAD_STATE_WAIT,
} DbgCpuThrStateType;
static pthread_mutex_t dbg_mutex;
static pthread_cond_t dbg_cv;
static pthread_cond_t cpu_cv;
static volatile DbgCpuThrStateType dbgthr_state = THREAD_STATE_RUNNING;
static volatile DbgCpuThrStateType cputhr_state = THREAD_STATE_WAIT;

void cputhr_control_init(void)
{
	pthread_mutex_init(&dbg_mutex, NULL);
	pthread_cond_init(&dbg_cv, NULL);
	pthread_cond_init(&cpu_cv, NULL);
	return;

}
#if 0 /* for test */
#include "cpu.h"
#include "cpu_control/dbg_cpu_callback.h"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

static void *cpuemu_thread_run(void* arg)
{
	TargetCoreType core;


	core.core_id = 0;
	core.reg.pc = 0;
	while (TRUE) {
		DBG_PRINT((DBG_EXEC_OP_BUF(), DBG_EXEC_OP_BUF_LEN(), "pc=0x%x\n", core.reg.pc));
		fflush(stdout);

		dbg_notify_cpu_clock_supply_start(&core);

		Sleep(100);

		core.reg.pc++;
	}
	return NULL;
}
#endif
void cputhr_control_start(void *(*cpu_run) (void *))
{
	pthread_t thread;
	cputhr_state = THREAD_STATE_RUNNING;
	pthread_create(&thread , NULL , cpu_run , NULL);
}


void cputhr_control_cpu_wait(void)
{
	pthread_mutex_lock(&dbg_mutex);
	cputhr_state = THREAD_STATE_WAIT;
	pthread_cond_wait(&cpu_cv, &dbg_mutex);
	cputhr_state = THREAD_STATE_RUNNING;
	pthread_mutex_unlock(&dbg_mutex);
	return;
}
void cputhr_control_dbg_wait(void)
{
	pthread_mutex_lock(&dbg_mutex);
	dbgthr_state = THREAD_STATE_WAIT;
	pthread_cond_wait(&dbg_cv, &dbg_mutex);
	dbgthr_state = THREAD_STATE_RUNNING;
	pthread_mutex_unlock(&dbg_mutex);
	return;
}

void cputhr_control_dbg_wakeup_cpu_and_wait_for_cpu_stopped(void)
{
	pthread_mutex_lock(&dbg_mutex);
	if (cputhr_state == THREAD_STATE_WAIT) {
		cputhr_state = THREAD_STATE_RUNNING;
		pthread_cond_signal(&cpu_cv);
	}
	while (cputhr_state == THREAD_STATE_RUNNING) {
		pthread_mutex_unlock(&dbg_mutex);

		Sleep(50);

		pthread_mutex_lock(&dbg_mutex);
	}
	pthread_mutex_unlock(&dbg_mutex);
	return;
}

void cputhr_control_dbg_waitfor_cpu_stopped(void)
{
	pthread_mutex_lock(&dbg_mutex);
	while (cputhr_state == THREAD_STATE_RUNNING) {
		pthread_mutex_unlock(&dbg_mutex);

		Sleep(50);

		pthread_mutex_lock(&dbg_mutex);
	}
	pthread_mutex_unlock(&dbg_mutex);
	return;
}

void cputhr_control_dbg_wakeup_cpu(void)
{
	pthread_mutex_lock(&dbg_mutex);
	pthread_cond_signal(&cpu_cv);
	pthread_mutex_unlock(&dbg_mutex);
}
void cputhr_control_cpu_wakeup_dbg(void)
{
	pthread_mutex_lock(&dbg_mutex);
	pthread_cond_signal(&dbg_cv);
	pthread_mutex_unlock(&dbg_mutex);
}
