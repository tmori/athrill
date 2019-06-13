/* Os_Lcfg.c */
#include "kernel/kernel_int.h"
#include "Os_Lcfg.h"
#include "Ioc.h"


#ifndef TOPPERS_EMPTY_LABEL
#define TOPPERS_EMPTY_LABEL(x, y) x y[0]
#endif

/*
 *  Default Definitions of Trace Log Macros
 */

#ifndef LOG_ISR_ENTER
#define LOG_ISR_ENTER(isrid)
#endif /* LOG_ISR_ENTER */

#ifndef LOG_ISR_LEAVE
#define LOG_ISR_LEAVE(isrid)
#endif /* LOG_ISR_LEAVE */

/*
 *  Include Directives (#include)
 */

#include "sample2.h"
#include "sample1.h"
#include "target_serial.h"
#include "target_hw_counter.h"
#include "target_mem.h"

const AlarmType					tnum_alarm				= TNUM_ALARM;
const CounterType				tnum_counter			= TNUM_COUNTER;
const CounterType				tnum_hardcounter		= TNUM_HARDCOUNTER;
const ISRType					tnum_isr2				= TNUM_ISR2;
const ResourceType				tnum_stdresource		= TNUM_STD_RESOURCE;
const TaskType					tnum_task				= TNUM_TASK;
const TaskType					tnum_exttask			= TNUM_EXTTASK;
const AppModeType				tnum_appmode			= TNUM_APP_MODE;
const ScheduleTableType			tnum_scheduletable		= TNUM_SCHEDULETABLE;
const ScheduleTableType			tnum_implscheduletable	= TNUM_IMPLSCHEDULETABLE;
const TrustedFunctionIndexType	tnum_tfn				= TNUM_TFN;
const ApplicationType			tnum_osap				= TNUM_OSAP;


/****** Object TASK ******/

StackType _kernel_sstack_Task3[COUNT_STK_T(ROUND_STK_T(512U))];
StackType _kernel_sstack_Task2[COUNT_STK_T(ROUND_STK_T(512U))];
StackType _kernel_sstack_Task12[COUNT_STK_T(ROUND_STK_T(512U))];
StackType _kernel_sstack_Task11[COUNT_STK_T(ROUND_STK_T(512U))];
StackType _kernel_sstack_Task10[COUNT_STK_T(ROUND_STK_T(512U))];
StackType _kernel_sstack_MainTask[COUNT_STK_T((512U) + (512U))];
StackType _kernel_sstack_restart_NT_osap2[COUNT_STK_T(ROUND_STK_T(1024U))];
StackType _kernel_sstack_restart_NT_osap1[COUNT_STK_T(ROUND_STK_T(1024U))];
StackType _kernel_shared_sstack_1[COUNT_STK_T((512U) + (512U))];
StackType _kernel_shared_sstack_4[COUNT_STK_T(512U)];
StackType _kernel_shared_sstack_6[COUNT_STK_T(512U)];
StackType _kernel_shared_sstack_8[COUNT_STK_T(512U)];
StackType _kernel_shared_sstack_9[COUNT_STK_T((512U) + (512U))];
StackType _kernel_shared_sstack_15[COUNT_STK_T(512U)];

StackType _kernel_ustack_Task3[COUNT_STK_T(512U)] __attribute__((section(".user_stack.Task3")));
StackType _kernel_ustack_Task2[COUNT_STK_T(512U)] __attribute__((section(".user_stack.Task2")));
StackType _kernel_ustack_Task12[COUNT_STK_T(512U)] __attribute__((section(".user_stack.Task12")));
StackType _kernel_ustack_Task11[COUNT_STK_T(512U)] __attribute__((section(".user_stack.Task11")));
StackType _kernel_ustack_Task10[COUNT_STK_T(512U)] __attribute__((section(".user_stack.Task10")));
OSAPCB osapcb_table[TNUM_OSAP];
StackType _kernel_shared_ustack_4[COUNT_STK_T(ROUND_STK_T(512U))] __attribute__((section(".shared_user_stack.4")));
StackType _kernel_shared_ustack_6[COUNT_STK_T(ROUND_STK_T(512U))] __attribute__((section(".shared_user_stack.6")));
StackType _kernel_shared_ustack_8[COUNT_STK_T(ROUND_STK_T(768U))] __attribute__((section(".shared_user_stack.8")));
StackType _kernel_shared_ustack_9[COUNT_STK_T(ROUND_STK_T(512U))] __attribute__((section(".shared_user_stack.9")));
StackType _kernel_shared_ustack_15[COUNT_STK_T(ROUND_STK_T(512U))] __attribute__((section(".shared_user_stack.15")));

TCB tcb_table[TNUM_TASK_INC_RT];


/****** Object COUNTER ******/

const CNTINIB cntinib_table[TNUM_COUNTER] = {
	{ 536870911U, (536870911U * 2U) + 1U, 10U, 4000U, &(osapcb_table[TRUSTED_OSAP_for_MAIN_HW_COUNTER]), 0x00000000U },
	{ 99U, (99U * 2U) + 1U, 10U, 10U, &(osapcb_table[NT_osap2]), 0x00000001U },
	{ 99U, (99U * 2U) + 1U, 10U, 10U, &(osapcb_table[NT_osap1]), 0x00000002U },
	{ 99U, (99U * 2U) + 1U, 10U, 10U, &(osapcb_table[NT_osap1]), 0x00000002U },
	{ 99U, (99U * 2U) + 1U, 10U, 10U, &(osapcb_table[NT_osap1]), 0x00000002U },
	{ 99U, (99U * 2U) + 1U, 10U, 10U, &(osapcb_table[NT_osap1]), 0x00000002U }
};

CNTCB cntcb_table[TNUM_COUNTER];
const HWCNTINIB hwcntinib_table[TNUM_HARDCOUNTER] = 
{
	{
		&init_hwcounter_MAIN_HW_COUNTER,
		&start_hwcounter_MAIN_HW_COUNTER,
		&stop_hwcounter_MAIN_HW_COUNTER,
		&set_hwcounter_MAIN_HW_COUNTER,
		&get_hwcounter_MAIN_HW_COUNTER,
		&cancel_hwcounter_MAIN_HW_COUNTER,
		&trigger_hwcounter_MAIN_HW_COUNTER,
		&int_clear_hwcounter_MAIN_HW_COUNTER,
		&int_cancel_hwcounter_MAIN_HW_COUNTER,
		&increment_hwcounter_MAIN_HW_COUNTER,
		125U			/* 0.000000125 * 1000000000 */ 
	}
};


/****** Object ALARM ******/

static void
_activate_alarm_1(void);
static void
_activate_alarm_1(void)
{
	(void) activate_task_action(&(osapcb_table[NT_osap1]), Task1);
}

static void
_setevent_alarm_2(void);
static void
_setevent_alarm_2(void)
{
	(void) set_event_action(&(osapcb_table[NT_osap1]), MainTask, MainEvt);
}

static void
_activate_alarm_3(void);
static void
_activate_alarm_3(void)
{
	(void) activate_task_action(&(osapcb_table[NT_osap2]), Task4);
}

static void
_setevent_alarm_4(void);
static void
_setevent_alarm_4(void)
{
	(void) set_event_action(&(osapcb_table[NT_osap1]), Task3, T3Evt);
}

static void
_setevent_alarm_5(void);
static void
_setevent_alarm_5(void)
{
	(void) set_event_action(&(osapcb_table[NT_osap1]), MainTask, MainEvt);
}

static void
_incrementcounter_alarm_6(void);
static void
_incrementcounter_alarm_6(void)
{
	(void) incr_counter_action(&(osapcb_table[NT_osap1]), SampleCnt3);
}

const ALMINIB alminib_table[TNUM_ALARM] = {
	{ &(cntcb_table[MAIN_HW_COUNTER]), &_activate_alarm_1, 0x00000000U, 0U, 0U, 0U, &(osapcb_table[NT_osap1]), 0x00000002U },
	{ &(cntcb_table[MAIN_HW_COUNTER]), &_setevent_alarm_2, 0x00000000U, 0U, 0U, 0U, &(osapcb_table[NT_osap1]), 0x00000002U },
	{ &(cntcb_table[ActOtherOSAPTskArmCnt]), &_activate_alarm_3, 0x00000007U, 5U, 0U, 3U, &(osapcb_table[NT_osap2]), 0x00000001U },
	{ &(cntcb_table[MAIN_HW_COUNTER]), &_setevent_alarm_4, 0x00000000U, 0U, 0U, 0U, &(osapcb_table[NT_osap1]), 0x00000002U },
	{ &(cntcb_table[SampleCnt3]), &_setevent_alarm_5, 0x00000007U, 10U, 10U, 5U, &(osapcb_table[NT_osap1]), 0x00000002U },
	{ &(cntcb_table[SampleCnt2]), &_incrementcounter_alarm_6, 0x00000007U, 10U, 10U, 6U, &(osapcb_table[NT_osap1]), 0x00000002U }
};

ALMCB	almcb_table[TNUM_ALARM];

/****** Object SCHEDULETABLE ******/

/* Object SCHEDULETABLE(scheduletable2) */

static void
_expire_scheduletable_1_0(void);
static void
_expire_scheduletable_1_0(void)
{
	(void) activate_task_action(&(osapcb_table[NT_osap1]), Task11);
	(void) activate_task_action(&(osapcb_table[NT_osap1]), Task12);
}
static void
_expire_scheduletable_1_1(void);
static void
_expire_scheduletable_1_1(void)
{
	(void) set_event_action(&(osapcb_table[NT_osap1]), Task12, T12Evt);
}
static void
_expire_scheduletable_1_2(void);
static void
_expire_scheduletable_1_2(void)
{
	(void) set_event_action(&(osapcb_table[NT_osap1]), Task11, T11Evt);
}

static const SCHTBLEXPPTCB schtblexppt_table_1[3] = {
	{ 20U, &_expire_scheduletable_1_0 },
	{ 30U, &_expire_scheduletable_1_1 },
	{ 40U, &_expire_scheduletable_1_2 }
};

/* Object SCHEDULETABLE(scheduletable1) */

static void
_expire_scheduletable_2_0(void);
static void
_expire_scheduletable_2_0(void)
{
	(void) activate_task_action(&(osapcb_table[NT_osap1]), Task12);
	(void) activate_task_action(&(osapcb_table[NT_osap1]), Task11);
	(void) activate_task_action(&(osapcb_table[NT_osap1]), Task10);
}
static void
_expire_scheduletable_2_1(void);
static void
_expire_scheduletable_2_1(void)
{
	(void) set_event_action(&(osapcb_table[NT_osap1]), Task12, T12Evt);
	(void) set_event_action(&(osapcb_table[NT_osap1]), Task11, T11Evt);
	(void) set_event_action(&(osapcb_table[NT_osap1]), Task10, T10Evt);
}

static const SCHTBLEXPPTCB schtblexppt_table_2[2] = {
	{ 10U, &_expire_scheduletable_2_0 },
	{ 20U, &_expire_scheduletable_2_1 }
};


const SCHTBLINIB schtblinib_table[TNUM_SCHEDULETABLE] = {
	{ &(cntcb_table[SchtblSampleCnt]), 50U, 0x00000000U, 0U, 0U, schtblexppt_table_1, TRUE, 3U, &(osapcb_table[NT_osap1]), 0x00000002U },
	{ &(cntcb_table[SchtblSampleCnt]), 60U, 0x00000000U, 0U, 0U, schtblexppt_table_2, TRUE, 2U, &(osapcb_table[NT_osap1]), 0x00000002U }
};

SCHTBLCB schtblcb_table[TNUM_SCHEDULETABLE];


/****** Object RESOURCE ******/

const RESINIB resinib_table[TNUM_STD_RESOURCE] = {
	{ 3, 0x00000002U },
	{ 6, 0x00000002U }
};

RESCB rescb_table[TNUM_STD_RESOURCE];


void
object_initialize(void)
{
	osap_initialize();
	interrupt_initialize();
	resource_initialize();
	task_initialize();
	/* initialize the Os-genereted Restart Task */
	tcb_table[21].p_tinib = &(tinib_table[21]);
	/* initialize the Os-genereted Restart Task */
	tcb_table[22].p_tinib = &(tinib_table[22]);
 	counter_initialize();
	alarm_initialize();
	schtbl_initialize();
	ioc_initialize();
}


void
object_terminate(void)
{
	counter_terminate();
}


/*
 *  Interrupt Management Functions
 */

void
_kernel_inthdr_80(void)
{
	i_begin_int(80U);
	LOG_ISR_ENTER(C2ISR_for_MAIN_HW_COUNTER);
	ISRNAME(C2ISR_for_MAIN_HW_COUNTER)();
	LOG_ISR_LEAVE(C2ISR_for_MAIN_HW_COUNTER);
	i_end_int(80U);
}
void
_kernel_inthdr_35(void)
{
	i_begin_int(35U);
	LOG_ISR_ENTER(RxHwSerialInt);
	ISRNAME(RxHwSerialInt)();
	LOG_ISR_LEAVE(RxHwSerialInt);
	i_end_int(35U);
}

/* HardWare Counter Interrupt Handler(C2ISR) */
ISR(C2ISR_for_MAIN_HW_COUNTER)
{
	notify_hardware_counter(MAIN_HW_COUNTER);
}

/*
 *  Stack Area for Non-task Context
 */

#define TNUM_INTNO	UINT_C(2)
const InterruptNumberType tnum_intno = TNUM_INTNO;

const INTINIB intinib_table[TNUM_INTNO] = {
	{ (80U),ENABLE, (-2), 0x600U},
	{ (35U),ENABLE, (-2), 0x450U}
};


/****** Object ISR ******/


const ISRINIB isrinib_table[TNUM_ISR2] = {
	{
		&(intinib_table[C2ISR_for_MAIN_HW_COUNTER]), &(osapcb_table[TRUSTED_OSAP_for_MAIN_HW_COUNTER]), 0x00000000U
	},
	{
		&(intinib_table[RxHwSerialInt]), &(osapcb_table[TRUSTED_OSAP_for_RxHwSerialInt]), 0x00000000U
	}
};

ISRCB isrcb_table[TNUM_ISR2];


/******** Trusted Function ********/
const TFINIB tfinib_table[TNUM_TFN] = {
	{ &TRUSTED_actioctask4, 32U },
	{ &TRUSTED_tfnt4, 32U },
	{ &TRUSTED_tfnt3, 32U },
	{ &TRUSTED_tfnt2, 32U },
	{ &TRUSTED_tfnt1, 32U },
	{ &TRUSTED_actioctask3, 32U }
};

static StackType _kernel_ostack[COUNT_STK_T(0x700U)];
#define TOPPERS_OSTKSZ		ROUND_STK_T(0x700U)
#define TOPPERS_OSTK		(_kernel_ostack)

const MemorySizeType	_ostksz = TOPPERS_OSTKSZ;
StackType * const		_ostk = (StackType *) TOPPERS_OSTK;

#ifdef TOPPERS_OSTKPT
StackType * const	_ostkpt = TOPPERS_OSTKPT(TOPPERS_OSTK, TOPPERS_OSTKSZ);
#endif /* TOPPERS_OSTKPT */


/****** Object IOC ******/
const IocType	tnum_ioc = TNUM_IOC;
const IocType	tnum_queueioc = TNUM_QUEUEIOC;

/*
 *  IOC Initialize Value
 */
/* IOC_QUE */
/* No initialize value because queue communication */

/* IOC_DEQUE */
IOCMB_IOC_DEQUE ioc_inival_IOC_DEQUE = {
	0U,
	10U,
	20U
};


void *ioc_inival_table[TNUM_IOC] = {
	NULL,
	(void *)&ioc_inival_IOC_DEQUE
};

IOCCB ioccb_table[TNUM_QUEUEIOC];

IOCMB_IOC_QUE iocmb_table_IOC_QUE[3];
IOCMB_IOC_DEQUE iocmb_table_IOC_DEQUE[1];

const IOCINIB iocinib_table[TNUM_IOC] = {
	{
		3U,
		sizeof(IOCMB_IOC_QUE),
		alignof(IOCMB_IOC_QUE),
#ifdef CFG_USE_ERRORHOOK
		FALSE,
#endif /* CFG_USE_ERRORHOOK */
		&osapcb_table[NT_osap2],
		&iocmb_table_IOC_QUE
	},
	{
		0U,
		sizeof(IOCMB_IOC_DEQUE),
		alignof(IOCMB_IOC_DEQUE),
#ifdef CFG_USE_ERRORHOOK
		TRUE,
#endif /* CFG_USE_ERRORHOOK */
		&osapcb_table[KT_osap2],
		&iocmb_table_IOC_DEQUE
	}
};

const IOCWRPINIB iocwrpinib_table[TNUM_IOC_WRAPPER] = {
	{
		&iocinib_table[IOC_DEQUE],
		&osapcb_table[KT_osap1],
	0U
	},
	{
		&iocinib_table[IOC_QUE],
		&osapcb_table[KT_osap1],
		1U
	},
	{
		&iocinib_table[IOC_QUE],
		&osapcb_table[NT_osap1],
		0U
	}
};


#ifdef TOPPERS_ENABLE_TRACE
const char8 *
atk2_appid_str(AppModeType id)
{
	const char8	*appid_str;
	switch (id) {
	case AppMode3:
		appid_str = "AppMode3";
		break;
	case AppMode2:
		appid_str = "AppMode2";
		break;
	case AppMode1:
		appid_str = "AppMode1";
		break;
	default:
		appid_str = "";
		break;
	}
	return(appid_str);
}
const char8 *
atk2_tskid_str(TaskType id)
{
	const char8	*tskid_str;
	switch (id) {
	case Task3:
		tskid_str = "Task3";
		break;
	case Task2:
		tskid_str = "Task2";
		break;
	case Task12:
		tskid_str = "Task12";
		break;
	case Task11:
		tskid_str = "Task11";
		break;
	case Task10:
		tskid_str = "Task10";
		break;
	case MainTask:
		tskid_str = "MainTask";
		break;
	case Task9:
		tskid_str = "Task9";
		break;
	case Task8:
		tskid_str = "Task8";
		break;
	case Task5:
		tskid_str = "Task5";
		break;
	case Task4:
		tskid_str = "Task4";
		break;
	case Task1:
		tskid_str = "Task1";
		break;
	case Task7:
		tskid_str = "Task7";
		break;
	case Task6:
		tskid_str = "Task6";
		break;
	case IocTask4:
		tskid_str = "IocTask4";
		break;
	case IocTask3:
		tskid_str = "IocTask3";
		break;
	case IocTask2:
		tskid_str = "IocTask2";
		break;
	case IocTask1:
		tskid_str = "IocTask1";
		break;
	case Task14:
		tskid_str = "Task14";
		break;
	case Task13:
		tskid_str = "Task13";
		break;
	case HighPriorityTask:
		tskid_str = "HighPriorityTask";
		break;
	case NonPriTask:
		tskid_str = "NonPriTask";
		break;
	case INVALID_TASK:
		tskid_str = "INVALID_TASK";
		break;
	default:
		tskid_str = "";
		break;
	}
	return(tskid_str);
}

const char8 *
atk2_isrid_str(ISRType id)
{
	const char8	*isrid_str;
	switch (id) {
	case C2ISR_for_MAIN_HW_COUNTER:
		isrid_str = "C2ISR_for_MAIN_HW_COUNTER";
		break;
	case RxHwSerialInt:
		isrid_str = "RxHwSerialInt";
		break;
	case INVALID_ISR:
		isrid_str = "INVALID_ISR";
		break;
	default:
		isrid_str = "";
		break;
	}
	return(isrid_str);
}

const char8 *
atk2_cntid_str(CounterType id)
{
	const char8	*cntid_str;
	switch (id) {
	case MAIN_HW_COUNTER:
		cntid_str = "MAIN_HW_COUNTER";
		break;
	case ActOtherOSAPTskArmCnt:
		cntid_str = "ActOtherOSAPTskArmCnt";
		break;
	case SampleCnt:
		cntid_str = "SampleCnt";
		break;
	case SampleCnt3:
		cntid_str = "SampleCnt3";
		break;
	case SampleCnt2:
		cntid_str = "SampleCnt2";
		break;
	case SchtblSampleCnt:
		cntid_str = "SchtblSampleCnt";
		break;
	default:
		cntid_str = "";
		break;
	}
	return(cntid_str);
}

const char8 *
atk2_almid_str(AlarmType id)
{
	const char8	*almid_str;
	switch (id) {
	case ActTskArm:
		almid_str = "ActTskArm";
		break;
	case MainCycArm:
		almid_str = "MainCycArm";
		break;
	case ActOtherOSAPTskArm:
		almid_str = "ActOtherOSAPTskArm";
		break;
	case SetEvtArm:
		almid_str = "SetEvtArm";
		break;
	case SampleAlm2:
		almid_str = "SampleAlm2";
		break;
	case SampleAlm1:
		almid_str = "SampleAlm1";
		break;
	default:
		almid_str = "";
		break;
	}
	return(almid_str);
}

const char8 *
atk2_resid_str(ResourceType id)
{
	const char8	*resid_str;
	switch (id) {
	case CntRes:
		resid_str = "CntRes";
		break;
	case TskLevelRes:
		resid_str = "TskLevelRes";
		break;
	case GroupRes:
		resid_str = "GroupRes";
		break;
	default:
		resid_str = "";
		break;
	}
	return(resid_str);
}

const char8 *
atk2_schtblid_str(ScheduleTableType id)
{
	const char8	*schtblid_str;
	switch (id) {
	case scheduletable2:
		schtblid_str = "scheduletable2";
		break;
	case scheduletable1:
		schtblid_str = "scheduletable1";
		break;
	default:
		schtblid_str = "";
		break;
	}
	return(schtblid_str);
}

const char8 *
atk2_evtid_str(TaskType task, EventMaskType event)
{
	const char8	*evtid_str;
	switch (task) {
	case Task3:
		switch (event) {
		case T3Evt:
			evtid_str = "T3Evt";
			break;
		default:
			evtid_str = NULL;
			break;
		}
		break;
	case Task2:
		switch (event) {
		case T2Evt:
			evtid_str = "T2Evt";
			break;
		default:
			evtid_str = NULL;
			break;
		}
		break;
	case Task12:
		switch (event) {
		case T12Evt:
			evtid_str = "T12Evt";
			break;
		default:
			evtid_str = NULL;
			break;
		}
		break;
	case Task11:
		switch (event) {
		case T11Evt:
			evtid_str = "T11Evt";
			break;
		default:
			evtid_str = NULL;
			break;
		}
		break;
	case Task10:
		switch (event) {
		case T10Evt:
			evtid_str = "T10Evt";
			break;
		default:
			evtid_str = NULL;
			break;
		}
		break;
	case MainTask:
		switch (event) {
		case MainEvt:
			evtid_str = "MainEvt";
			break;
		default:
			evtid_str = NULL;
			break;
		}
		break;
	case Task9:
		evtid_str = NULL;
		break;
	case Task8:
		evtid_str = NULL;
		break;
	case Task5:
		evtid_str = NULL;
		break;
	case Task4:
		evtid_str = NULL;
		break;
	case Task1:
		evtid_str = NULL;
		break;
	case Task7:
		evtid_str = NULL;
		break;
	case Task6:
		evtid_str = NULL;
		break;
	case IocTask4:
		evtid_str = NULL;
		break;
	case IocTask3:
		evtid_str = NULL;
		break;
	case IocTask2:
		evtid_str = NULL;
		break;
	case IocTask1:
		evtid_str = NULL;
		break;
	case Task14:
		evtid_str = NULL;
		break;
	case Task13:
		evtid_str = NULL;
		break;
	case HighPriorityTask:
		evtid_str = NULL;
		break;
	case NonPriTask:
		evtid_str = NULL;
		break;
	default:
		evtid_str = NULL;
		break;
	}
	if (evtid_str == NULL) {
		if (event == T11Evt) {
			evtid_str = "T11Evt";
		}
		if (event == T12Evt) {
			evtid_str = "T12Evt";
		}
		if (event == T2Evt) {
			evtid_str = "T2Evt";
		}
		if (event == MainEvt) {
			evtid_str = "MainEvt";
		}
		if (event == T3Evt) {
			evtid_str = "T3Evt";
		}
		if (event == T10Evt) {
			evtid_str = "T10Evt";
		}
	}
	return(evtid_str);
}

const char8 *
atk2_osapid_str(ApplicationType id)
{
	const char8	*osapid_str;
	switch (id) {
	case NT_osap2:
		osapid_str = "NT_osap2";
		break;
	case NT_osap1:
		osapid_str = "NT_osap1";
		break;
	case KT_osap2:
		osapid_str = "KT_osap2";
		break;
	case KT_osap1:
		osapid_str = "KT_osap1";
		break;
	case TRUSTED_OSAP_for_MAIN_HW_COUNTER:
		osapid_str = "TRUSTED_OSAP_for_MAIN_HW_COUNTER";
		break;
	case TRUSTED_OSAP_for_RxHwSerialInt:
		osapid_str = "TRUSTED_OSAP_for_RxHwSerialInt";
		break;
	case TRUSTED_OSAP_for_NON_USE_PE_ROM:
		osapid_str = "TRUSTED_OSAP_for_NON_USE_PE_ROM";
		break;
	case INVALID_OSAPPLICATION:
		osapid_str = "INVALID_OSAPPLICATION";
		break;
	default:
		osapid_str = "";
		break;
	}
	return(osapid_str);
}

const char8 *
atk2_iocid_str(IocType id)
{
	const char8	*iocid_str;
	switch (id) {
	case IOC_QUE:
		iocid_str = "IOC_QUE";
		break;
	case IOC_DEQUE:
		iocid_str = "IOC_DEQUE";
		break;
	default:
		iocid_str = "";
		break;
	}
	return(iocid_str);
}

const char8 *
atk2_tfnid_str(TrustedFunctionIndexType id)
{
	const char8	*tfnid_str;
	switch (id) {
	case actioctask4:
		tfnid_str = "actioctask4";
		break;
	case tfnt4:
		tfnid_str = "tfnt4";
		break;
	case tfnt3:
		tfnid_str = "tfnt3";
		break;
	case tfnt2:
		tfnid_str = "tfnt2";
		break;
	case tfnt1:
		tfnid_str = "tfnt1";
		break;
	case actioctask3:
		tfnid_str = "actioctask3";
		break;
	default:
		tfnid_str = "";
		break;
	}
	return(tfnid_str);
}
#endif /* TOPPERS_ENABLE_TRACE */
											
const uint16 pmr_isr2_mask = 0xc000;
const uint16 pmr_isr1_mask = 0x3fff;
const FunctionRefType isr_tbl[TNUM_INT] = {
	default_int_handler,	/* 0 */
	default_int_handler,	/* 1 */
	default_int_handler,	/* 2 */
	default_int_handler,	/* 3 */
	default_int_handler,	/* 4 */
	default_int_handler,	/* 5 */
	default_int_handler,	/* 6 */
	default_int_handler,	/* 7 */
	default_int_handler,	/* 8 */
	default_int_handler,	/* 9 */
	default_int_handler,	/* 10 */
	default_int_handler,	/* 11 */
	default_int_handler,	/* 12 */
	default_int_handler,	/* 13 */
	default_int_handler,	/* 14 */
	default_int_handler,	/* 15 */
	default_int_handler,	/* 16 */
	default_int_handler,	/* 17 */
	default_int_handler,	/* 18 */
	default_int_handler,	/* 19 */
	default_int_handler,	/* 20 */
	default_int_handler,	/* 21 */
	default_int_handler,	/* 22 */
	default_int_handler,	/* 23 */
	default_int_handler,	/* 24 */
	default_int_handler,	/* 25 */
	default_int_handler,	/* 26 */
	default_int_handler,	/* 27 */
	default_int_handler,	/* 28 */
	default_int_handler,	/* 29 */
	default_int_handler,	/* 30 */
	default_int_handler,	/* 31 */
	default_int_handler,	/* 32 */
	default_int_handler,	/* 33 */
	default_int_handler,	/* 34 */
	_kernel_inthdr_35,	/* 35 */
	default_int_handler,	/* 36 */
	default_int_handler,	/* 37 */
	default_int_handler,	/* 38 */
	default_int_handler,	/* 39 */
	default_int_handler,	/* 40 */
	default_int_handler,	/* 41 */
	default_int_handler,	/* 42 */
	default_int_handler,	/* 43 */
	default_int_handler,	/* 44 */
	default_int_handler,	/* 45 */
	default_int_handler,	/* 46 */
	default_int_handler,	/* 47 */
	default_int_handler,	/* 48 */
	default_int_handler,	/* 49 */
	default_int_handler,	/* 50 */
	default_int_handler,	/* 51 */
	default_int_handler,	/* 52 */
	default_int_handler,	/* 53 */
	default_int_handler,	/* 54 */
	default_int_handler,	/* 55 */
	default_int_handler,	/* 56 */
	default_int_handler,	/* 57 */
	default_int_handler,	/* 58 */
	default_int_handler,	/* 59 */
	default_int_handler,	/* 60 */
	default_int_handler,	/* 61 */
	default_int_handler,	/* 62 */
	default_int_handler,	/* 63 */
	default_int_handler,	/* 64 */
	default_int_handler,	/* 65 */
	default_int_handler,	/* 66 */
	default_int_handler,	/* 67 */
	default_int_handler,	/* 68 */
	default_int_handler,	/* 69 */
	default_int_handler,	/* 70 */
	default_int_handler,	/* 71 */
	default_int_handler,	/* 72 */
	default_int_handler,	/* 73 */
	default_int_handler,	/* 74 */
	default_int_handler,	/* 75 */
	default_int_handler,	/* 76 */
	default_int_handler,	/* 77 */
	default_int_handler,	/* 78 */
	default_int_handler,	/* 79 */
	_kernel_inthdr_80,	/* 80 */
	default_int_handler,	/* 81 */
	default_int_handler,	/* 82 */
	default_int_handler,	/* 83 */
	default_int_handler,	/* 84 */
	default_int_handler,	/* 85 */
	default_int_handler,	/* 86 */
	default_int_handler,	/* 87 */
	default_int_handler,	/* 88 */
	default_int_handler,	/* 89 */
	default_int_handler,	/* 90 */
	default_int_handler,	/* 91 */
	default_int_handler,	/* 92 */
	default_int_handler,	/* 93 */
	default_int_handler,	/* 94 */
	default_int_handler,	/* 95 */
	default_int_handler,	/* 96 */
	default_int_handler,	/* 97 */
	default_int_handler,	/* 98 */
	default_int_handler,	/* 99 */
	default_int_handler,	/* 100 */
	default_int_handler,	/* 101 */
	default_int_handler,	/* 102 */
	default_int_handler,	/* 103 */
	default_int_handler,	/* 104 */
	default_int_handler,	/* 105 */
	default_int_handler,	/* 106 */
	default_int_handler,	/* 107 */
	default_int_handler,	/* 108 */
	default_int_handler,	/* 109 */
	default_int_handler,	/* 110 */
	default_int_handler,	/* 111 */
	default_int_handler,	/* 112 */
	default_int_handler,	/* 113 */
	default_int_handler,	/* 114 */
	default_int_handler,	/* 115 */
	default_int_handler,	/* 116 */
	default_int_handler,	/* 117 */
	default_int_handler,	/* 118 */
	default_int_handler,	/* 119 */
	default_int_handler,	/* 120 */
	default_int_handler,	/* 121 */
	default_int_handler,	/* 122 */
	default_int_handler,	/* 123 */
	default_int_handler,	/* 124 */
	default_int_handler,	/* 125 */
	default_int_handler,	/* 126 */
	default_int_handler,	/* 127 */
	default_int_handler,	/* 128 */
	default_int_handler,	/* 129 */
	default_int_handler,	/* 130 */
	default_int_handler,	/* 131 */
	default_int_handler,	/* 132 */
	default_int_handler,	/* 133 */
	default_int_handler,	/* 134 */
	default_int_handler,	/* 135 */
	default_int_handler,	/* 136 */
	default_int_handler,	/* 137 */
	default_int_handler,	/* 138 */
	default_int_handler,	/* 139 */
	default_int_handler,	/* 140 */
	default_int_handler,	/* 141 */
	default_int_handler,	/* 142 */
	default_int_handler,	/* 143 */
	default_int_handler,	/* 144 */
	default_int_handler,	/* 145 */
	default_int_handler,	/* 146 */
	default_int_handler,	/* 147 */
	default_int_handler,	/* 148 */
	default_int_handler,	/* 149 */
	default_int_handler,	/* 150 */
	default_int_handler,	/* 151 */
	default_int_handler,	/* 152 */
	default_int_handler,	/* 153 */
	default_int_handler,	/* 154 */
	default_int_handler,	/* 155 */
	default_int_handler,	/* 156 */
	default_int_handler,	/* 157 */
	default_int_handler,	/* 158 */
	default_int_handler,	/* 159 */
	default_int_handler,	/* 160 */
	default_int_handler,	/* 161 */
	default_int_handler,	/* 162 */
	default_int_handler,	/* 163 */
	default_int_handler,	/* 164 */
	default_int_handler,	/* 165 */
	default_int_handler,	/* 166 */
	default_int_handler,	/* 167 */
	default_int_handler,	/* 168 */
	default_int_handler,	/* 169 */
	default_int_handler,	/* 170 */
	default_int_handler,	/* 171 */
	default_int_handler,	/* 172 */
	default_int_handler,	/* 173 */
	default_int_handler,	/* 174 */
	default_int_handler,	/* 175 */
	default_int_handler,	/* 176 */
	default_int_handler,	/* 177 */
	default_int_handler,	/* 178 */
	default_int_handler,	/* 179 */
	default_int_handler,	/* 180 */
	default_int_handler,	/* 181 */
	default_int_handler,	/* 182 */
	default_int_handler,	/* 183 */
	default_int_handler,	/* 184 */
	default_int_handler,	/* 185 */
	default_int_handler,	/* 186 */
	default_int_handler,	/* 187 */
	default_int_handler,	/* 188 */
	default_int_handler,	/* 189 */
	default_int_handler,	/* 190 */
	default_int_handler,	/* 191 */
	default_int_handler,	/* 192 */
	default_int_handler,	/* 193 */
	default_int_handler,	/* 194 */
	default_int_handler,	/* 195 */
	default_int_handler,	/* 196 */
	default_int_handler,	/* 197 */
	default_int_handler,	/* 198 */
	default_int_handler,	/* 199 */
	default_int_handler,	/* 200 */
	default_int_handler,	/* 201 */
	default_int_handler,	/* 202 */
	default_int_handler,	/* 203 */
	default_int_handler,	/* 204 */
	default_int_handler,	/* 205 */
	default_int_handler,	/* 206 */
	default_int_handler,	/* 207 */
	default_int_handler,	/* 208 */
	default_int_handler,	/* 209 */
	default_int_handler,	/* 210 */
	default_int_handler,	/* 211 */
	default_int_handler,	/* 212 */
	default_int_handler,	/* 213 */
	default_int_handler,	/* 214 */
	default_int_handler,	/* 215 */
	default_int_handler,	/* 216 */
	default_int_handler,	/* 217 */
	default_int_handler,	/* 218 */
	default_int_handler,	/* 219 */
	default_int_handler,	/* 220 */
	default_int_handler,	/* 221 */
	default_int_handler,	/* 222 */
	default_int_handler,	/* 223 */
	default_int_handler,	/* 224 */
	default_int_handler,	/* 225 */
	default_int_handler,	/* 226 */
	default_int_handler,	/* 227 */
	default_int_handler,	/* 228 */
	default_int_handler,	/* 229 */
	default_int_handler,	/* 230 */
	default_int_handler,	/* 231 */
	default_int_handler,	/* 232 */
	default_int_handler,	/* 233 */
	default_int_handler,	/* 234 */
	default_int_handler,	/* 235 */
	default_int_handler,	/* 236 */
	default_int_handler,	/* 237 */
	default_int_handler,	/* 238 */
	default_int_handler,	/* 239 */
	default_int_handler,	/* 240 */
	default_int_handler,	/* 241 */
	default_int_handler,	/* 242 */
	default_int_handler,	/* 243 */
	default_int_handler,	/* 244 */
	default_int_handler,	/* 245 */
	default_int_handler,	/* 246 */
	default_int_handler,	/* 247 */
	default_int_handler,	/* 248 */
	default_int_handler,	/* 249 */
	default_int_handler,	/* 250 */
	default_int_handler,	/* 251 */
	default_int_handler,	/* 252 */
	default_int_handler,	/* 253 */
	default_int_handler,	/* 254 */
	default_int_handler,	/* 255 */
	default_int_handler,	/* 256 */
	default_int_handler,	/* 257 */
	default_int_handler,	/* 258 */
	default_int_handler,	/* 259 */
	default_int_handler,	/* 260 */
	default_int_handler,	/* 261 */
	default_int_handler,	/* 262 */
	default_int_handler,	/* 263 */
	default_int_handler,	/* 264 */
	default_int_handler,	/* 265 */
	default_int_handler,	/* 266 */
	default_int_handler,	/* 267 */
	default_int_handler,	/* 268 */
	default_int_handler,	/* 269 */
	default_int_handler,	/* 270 */
	default_int_handler,	/* 271 */
	default_int_handler,	/* 272 */
	default_int_handler,	/* 273 */
	default_int_handler,	/* 274 */
	default_int_handler,	/* 275 */
	default_int_handler,	/* 276 */
	default_int_handler,	/* 277 */
	default_int_handler,	/* 278 */
	default_int_handler,	/* 279 */
	default_int_handler,	/* 280 */
	default_int_handler,	/* 281 */
	default_int_handler,	/* 282 */
	default_int_handler,	/* 283 */
	default_int_handler,	/* 284 */
	default_int_handler,	/* 285 */
	default_int_handler,	/* 286 */
	default_int_handler,	/* 287 */
	default_int_handler,	/* 288 */
	default_int_handler,	/* 289 */
	default_int_handler,	/* 290 */
	default_int_handler,	/* 291 */
	default_int_handler,	/* 292 */
	default_int_handler,	/* 293 */
	default_int_handler,	/* 294 */
	default_int_handler,	/* 295 */
	default_int_handler,	/* 296 */
	default_int_handler,	/* 297 */
	default_int_handler,	/* 298 */
	default_int_handler,	/* 299 */
	default_int_handler,	/* 300 */
	default_int_handler,	/* 301 */
	default_int_handler,	/* 302 */
	default_int_handler,	/* 303 */
	default_int_handler,	/* 304 */
	default_int_handler,	/* 305 */
	default_int_handler,	/* 306 */
	default_int_handler,	/* 307 */
	default_int_handler,	/* 308 */
	default_int_handler,	/* 309 */
	default_int_handler,	/* 310 */
	default_int_handler,	/* 311 */
	default_int_handler,	/* 312 */
	default_int_handler,	/* 313 */
	default_int_handler,	/* 314 */
	default_int_handler,	/* 315 */
	default_int_handler,	/* 316 */
	default_int_handler,	/* 317 */
	default_int_handler,	/* 318 */
	default_int_handler,	/* 319 */
	default_int_handler,	/* 320 */
	default_int_handler,	/* 321 */
	default_int_handler,	/* 322 */
	default_int_handler,	/* 323 */
	default_int_handler,	/* 324 */
	default_int_handler,	/* 325 */
	default_int_handler,	/* 326 */
	default_int_handler,	/* 327 */
	default_int_handler,	/* 328 */
	default_int_handler,	/* 329 */
	default_int_handler,	/* 330 */
	default_int_handler,	/* 331 */
	default_int_handler,	/* 332 */
	default_int_handler,	/* 333 */
	default_int_handler,	/* 334 */
	default_int_handler,	/* 335 */
	default_int_handler,	/* 336 */
	default_int_handler,	/* 337 */
	default_int_handler,	/* 338 */
	default_int_handler,	/* 339 */
	default_int_handler,	/* 340 */
	default_int_handler,	/* 341 */
	default_int_handler,	/* 342 */
	default_int_handler,	/* 343 */
	default_int_handler,	/* 344 */
	default_int_handler,	/* 345 */
	default_int_handler,	/* 346 */
	default_int_handler,	/* 347 */
	default_int_handler,	/* 348 */
	default_int_handler,	/* 349 */
	default_int_handler	/* 350 */
};

ISRCB *const isr_p_isrcb_tbl[TNUM_INT] = {
	NULL,	/* 0 */
	NULL,	/* 1 */
	NULL,	/* 2 */
	NULL,	/* 3 */
	NULL,	/* 4 */
	NULL,	/* 5 */
	NULL,	/* 6 */
	NULL,	/* 7 */
	NULL,	/* 8 */
	NULL,	/* 9 */
	NULL,	/* 10 */
	NULL,	/* 11 */
	NULL,	/* 12 */
	NULL,	/* 13 */
	NULL,	/* 14 */
	NULL,	/* 15 */
	NULL,	/* 16 */
	NULL,	/* 17 */
	NULL,	/* 18 */
	NULL,	/* 19 */
	NULL,	/* 20 */
	NULL,	/* 21 */
	NULL,	/* 22 */
	NULL,	/* 23 */
	NULL,	/* 24 */
	NULL,	/* 25 */
	NULL,	/* 26 */
	NULL,	/* 27 */
	NULL,	/* 28 */
	NULL,	/* 29 */
	NULL,	/* 30 */
	NULL,	/* 31 */
	NULL,	/* 32 */
	NULL,	/* 33 */
	NULL,	/* 34 */
	&(isrcb_table[1]),	/* 35 */
	NULL,	/* 36 */
	NULL,	/* 37 */
	NULL,	/* 38 */
	NULL,	/* 39 */
	NULL,	/* 40 */
	NULL,	/* 41 */
	NULL,	/* 42 */
	NULL,	/* 43 */
	NULL,	/* 44 */
	NULL,	/* 45 */
	NULL,	/* 46 */
	NULL,	/* 47 */
	NULL,	/* 48 */
	NULL,	/* 49 */
	NULL,	/* 50 */
	NULL,	/* 51 */
	NULL,	/* 52 */
	NULL,	/* 53 */
	NULL,	/* 54 */
	NULL,	/* 55 */
	NULL,	/* 56 */
	NULL,	/* 57 */
	NULL,	/* 58 */
	NULL,	/* 59 */
	NULL,	/* 60 */
	NULL,	/* 61 */
	NULL,	/* 62 */
	NULL,	/* 63 */
	NULL,	/* 64 */
	NULL,	/* 65 */
	NULL,	/* 66 */
	NULL,	/* 67 */
	NULL,	/* 68 */
	NULL,	/* 69 */
	NULL,	/* 70 */
	NULL,	/* 71 */
	NULL,	/* 72 */
	NULL,	/* 73 */
	NULL,	/* 74 */
	NULL,	/* 75 */
	NULL,	/* 76 */
	NULL,	/* 77 */
	NULL,	/* 78 */
	NULL,	/* 79 */
	&(isrcb_table[0]),	/* 80 */
	NULL,	/* 81 */
	NULL,	/* 82 */
	NULL,	/* 83 */
	NULL,	/* 84 */
	NULL,	/* 85 */
	NULL,	/* 86 */
	NULL,	/* 87 */
	NULL,	/* 88 */
	NULL,	/* 89 */
	NULL,	/* 90 */
	NULL,	/* 91 */
	NULL,	/* 92 */
	NULL,	/* 93 */
	NULL,	/* 94 */
	NULL,	/* 95 */
	NULL,	/* 96 */
	NULL,	/* 97 */
	NULL,	/* 98 */
	NULL,	/* 99 */
	NULL,	/* 100 */
	NULL,	/* 101 */
	NULL,	/* 102 */
	NULL,	/* 103 */
	NULL,	/* 104 */
	NULL,	/* 105 */
	NULL,	/* 106 */
	NULL,	/* 107 */
	NULL,	/* 108 */
	NULL,	/* 109 */
	NULL,	/* 110 */
	NULL,	/* 111 */
	NULL,	/* 112 */
	NULL,	/* 113 */
	NULL,	/* 114 */
	NULL,	/* 115 */
	NULL,	/* 116 */
	NULL,	/* 117 */
	NULL,	/* 118 */
	NULL,	/* 119 */
	NULL,	/* 120 */
	NULL,	/* 121 */
	NULL,	/* 122 */
	NULL,	/* 123 */
	NULL,	/* 124 */
	NULL,	/* 125 */
	NULL,	/* 126 */
	NULL,	/* 127 */
	NULL,	/* 128 */
	NULL,	/* 129 */
	NULL,	/* 130 */
	NULL,	/* 131 */
	NULL,	/* 132 */
	NULL,	/* 133 */
	NULL,	/* 134 */
	NULL,	/* 135 */
	NULL,	/* 136 */
	NULL,	/* 137 */
	NULL,	/* 138 */
	NULL,	/* 139 */
	NULL,	/* 140 */
	NULL,	/* 141 */
	NULL,	/* 142 */
	NULL,	/* 143 */
	NULL,	/* 144 */
	NULL,	/* 145 */
	NULL,	/* 146 */
	NULL,	/* 147 */
	NULL,	/* 148 */
	NULL,	/* 149 */
	NULL,	/* 150 */
	NULL,	/* 151 */
	NULL,	/* 152 */
	NULL,	/* 153 */
	NULL,	/* 154 */
	NULL,	/* 155 */
	NULL,	/* 156 */
	NULL,	/* 157 */
	NULL,	/* 158 */
	NULL,	/* 159 */
	NULL,	/* 160 */
	NULL,	/* 161 */
	NULL,	/* 162 */
	NULL,	/* 163 */
	NULL,	/* 164 */
	NULL,	/* 165 */
	NULL,	/* 166 */
	NULL,	/* 167 */
	NULL,	/* 168 */
	NULL,	/* 169 */
	NULL,	/* 170 */
	NULL,	/* 171 */
	NULL,	/* 172 */
	NULL,	/* 173 */
	NULL,	/* 174 */
	NULL,	/* 175 */
	NULL,	/* 176 */
	NULL,	/* 177 */
	NULL,	/* 178 */
	NULL,	/* 179 */
	NULL,	/* 180 */
	NULL,	/* 181 */
	NULL,	/* 182 */
	NULL,	/* 183 */
	NULL,	/* 184 */
	NULL,	/* 185 */
	NULL,	/* 186 */
	NULL,	/* 187 */
	NULL,	/* 188 */
	NULL,	/* 189 */
	NULL,	/* 190 */
	NULL,	/* 191 */
	NULL,	/* 192 */
	NULL,	/* 193 */
	NULL,	/* 194 */
	NULL,	/* 195 */
	NULL,	/* 196 */
	NULL,	/* 197 */
	NULL,	/* 198 */
	NULL,	/* 199 */
	NULL,	/* 200 */
	NULL,	/* 201 */
	NULL,	/* 202 */
	NULL,	/* 203 */
	NULL,	/* 204 */
	NULL,	/* 205 */
	NULL,	/* 206 */
	NULL,	/* 207 */
	NULL,	/* 208 */
	NULL,	/* 209 */
	NULL,	/* 210 */
	NULL,	/* 211 */
	NULL,	/* 212 */
	NULL,	/* 213 */
	NULL,	/* 214 */
	NULL,	/* 215 */
	NULL,	/* 216 */
	NULL,	/* 217 */
	NULL,	/* 218 */
	NULL,	/* 219 */
	NULL,	/* 220 */
	NULL,	/* 221 */
	NULL,	/* 222 */
	NULL,	/* 223 */
	NULL,	/* 224 */
	NULL,	/* 225 */
	NULL,	/* 226 */
	NULL,	/* 227 */
	NULL,	/* 228 */
	NULL,	/* 229 */
	NULL,	/* 230 */
	NULL,	/* 231 */
	NULL,	/* 232 */
	NULL,	/* 233 */
	NULL,	/* 234 */
	NULL,	/* 235 */
	NULL,	/* 236 */
	NULL,	/* 237 */
	NULL,	/* 238 */
	NULL,	/* 239 */
	NULL,	/* 240 */
	NULL,	/* 241 */
	NULL,	/* 242 */
	NULL,	/* 243 */
	NULL,	/* 244 */
	NULL,	/* 245 */
	NULL,	/* 246 */
	NULL,	/* 247 */
	NULL,	/* 248 */
	NULL,	/* 249 */
	NULL,	/* 250 */
	NULL,	/* 251 */
	NULL,	/* 252 */
	NULL,	/* 253 */
	NULL,	/* 254 */
	NULL,	/* 255 */
	NULL,	/* 256 */
	NULL,	/* 257 */
	NULL,	/* 258 */
	NULL,	/* 259 */
	NULL,	/* 260 */
	NULL,	/* 261 */
	NULL,	/* 262 */
	NULL,	/* 263 */
	NULL,	/* 264 */
	NULL,	/* 265 */
	NULL,	/* 266 */
	NULL,	/* 267 */
	NULL,	/* 268 */
	NULL,	/* 269 */
	NULL,	/* 270 */
	NULL,	/* 271 */
	NULL,	/* 272 */
	NULL,	/* 273 */
	NULL,	/* 274 */
	NULL,	/* 275 */
	NULL,	/* 276 */
	NULL,	/* 277 */
	NULL,	/* 278 */
	NULL,	/* 279 */
	NULL,	/* 280 */
	NULL,	/* 281 */
	NULL,	/* 282 */
	NULL,	/* 283 */
	NULL,	/* 284 */
	NULL,	/* 285 */
	NULL,	/* 286 */
	NULL,	/* 287 */
	NULL,	/* 288 */
	NULL,	/* 289 */
	NULL,	/* 290 */
	NULL,	/* 291 */
	NULL,	/* 292 */
	NULL,	/* 293 */
	NULL,	/* 294 */
	NULL,	/* 295 */
	NULL,	/* 296 */
	NULL,	/* 297 */
	NULL,	/* 298 */
	NULL,	/* 299 */
	NULL,	/* 300 */
	NULL,	/* 301 */
	NULL,	/* 302 */
	NULL,	/* 303 */
	NULL,	/* 304 */
	NULL,	/* 305 */
	NULL,	/* 306 */
	NULL,	/* 307 */
	NULL,	/* 308 */
	NULL,	/* 309 */
	NULL,	/* 310 */
	NULL,	/* 311 */
	NULL,	/* 312 */
	NULL,	/* 313 */
	NULL,	/* 314 */
	NULL,	/* 315 */
	NULL,	/* 316 */
	NULL,	/* 317 */
	NULL,	/* 318 */
	NULL,	/* 319 */
	NULL,	/* 320 */
	NULL,	/* 321 */
	NULL,	/* 322 */
	NULL,	/* 323 */
	NULL,	/* 324 */
	NULL,	/* 325 */
	NULL,	/* 326 */
	NULL,	/* 327 */
	NULL,	/* 328 */
	NULL,	/* 329 */
	NULL,	/* 330 */
	NULL,	/* 331 */
	NULL,	/* 332 */
	NULL,	/* 333 */
	NULL,	/* 334 */
	NULL,	/* 335 */
	NULL,	/* 336 */
	NULL,	/* 337 */
	NULL,	/* 338 */
	NULL,	/* 339 */
	NULL,	/* 340 */
	NULL,	/* 341 */
	NULL,	/* 342 */
	NULL,	/* 343 */
	NULL,	/* 344 */
	NULL,	/* 345 */
	NULL,	/* 346 */
	NULL,	/* 347 */
	NULL,	/* 348 */
	NULL,	/* 349 */
	NULL	/* 350 */
};


extern uint8* shared_meminib_table[];

/*LOCAL_INLINE */void
mpu_shared_area_initialize(void)
{
	uint32 mpur;

	/*
	 * MPU15
	 */
	mpur = (uint32)shared_meminib_table[0];
	LDSR_REG(28, 7, (uint32)mpur);
	mpur = (uint32)shared_meminib_table[1];
	LDSR_REG(29, 7, (uint32)mpur);
	mpur = (uint32)shared_meminib_table[2];
	LDSR_REG(30, 7, (uint32)mpur);
	/*
	 * MPU14
	 */
	mpur = (uint32)shared_meminib_table[3];
	LDSR_REG(24, 7, (uint32)mpur);
	mpur = (uint32)shared_meminib_table[4];
	LDSR_REG(25, 7, (uint32)mpur);
	mpur = (uint32)shared_meminib_table[5];
	LDSR_REG(26, 7, (uint32)mpur);
	/*
	 * MPU13
	 */
	mpur = (uint32)shared_meminib_table[6];
	LDSR_REG(20, 7, (uint32)mpur);
	mpur = (uint32)shared_meminib_table[7];
	LDSR_REG(21, 7, (uint32)mpur);
	mpur = (uint32)shared_meminib_table[8];
	LDSR_REG(22, 7, (uint32)mpur);
	/*
	 * 共有リード/専用ライト領域の専用領域
	 * MPU1
	 */
	LDSR_REG(6, 6, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UW | MPAT_UR))
	/*
	 * 共有リード/専用ライト領域の専用領域(sdata)
	 * MPU2
	 */
	LDSR_REG(10, 6, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UW | MPAT_UR))
	/*
	 * 自保護ドメイン専用のrom領域
	 * MPU3
	 */
	LDSR_REG(14, 6, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UR))
	/*
	 * 自保護ドメイン専用のrosdata領域
	 * MPU4
	 */
	LDSR_REG(18, 6, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UR))
	/*
	 * 自保護ドメイン専用のRWX領域
	 * MPU5
	 */
	LDSR_REG(22, 6, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UW | MPAT_UR))
	/*
	 * 自保護ドメイン専用のRWX領域(sdata)
	 * MPU6
	 */
	LDSR_REG(26, 6, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UW | MPAT_UR))
}

extern void interrupt(void);
const uint32 __attribute__((aligned(512))) intbp_tbl[TNUM_INT] = {
	(uint32)&interrupt,	/* 0 */
	(uint32)&interrupt,	/* 1 */
	(uint32)&interrupt,	/* 2 */
	(uint32)&interrupt,	/* 3 */
	(uint32)&interrupt,	/* 4 */
	(uint32)&interrupt,	/* 5 */
	(uint32)&interrupt,	/* 6 */
	(uint32)&interrupt,	/* 7 */
	(uint32)&interrupt,	/* 8 */
	(uint32)&interrupt,	/* 9 */
	(uint32)&interrupt,	/* 10 */
	(uint32)&interrupt,	/* 11 */
	(uint32)&interrupt,	/* 12 */
	(uint32)&interrupt,	/* 13 */
	(uint32)&interrupt,	/* 14 */
	(uint32)&interrupt,	/* 15 */
	(uint32)&interrupt,	/* 16 */
	(uint32)&interrupt,	/* 17 */
	(uint32)&interrupt,	/* 18 */
	(uint32)&interrupt,	/* 19 */
	(uint32)&interrupt,	/* 20 */
	(uint32)&interrupt,	/* 21 */
	(uint32)&interrupt,	/* 22 */
	(uint32)&interrupt,	/* 23 */
	(uint32)&interrupt,	/* 24 */
	(uint32)&interrupt,	/* 25 */
	(uint32)&interrupt,	/* 26 */
	(uint32)&interrupt,	/* 27 */
	(uint32)&interrupt,	/* 28 */
	(uint32)&interrupt,	/* 29 */
	(uint32)&interrupt,	/* 30 */
	(uint32)&interrupt,	/* 31 */
	(uint32)&interrupt,	/* 32 */
	(uint32)&interrupt,	/* 33 */
	(uint32)&interrupt,	/* 34 */
	(uint32)&interrupt,	/* 35 */
	(uint32)&interrupt,	/* 36 */
	(uint32)&interrupt,	/* 37 */
	(uint32)&interrupt,	/* 38 */
	(uint32)&interrupt,	/* 39 */
	(uint32)&interrupt,	/* 40 */
	(uint32)&interrupt,	/* 41 */
	(uint32)&interrupt,	/* 42 */
	(uint32)&interrupt,	/* 43 */
	(uint32)&interrupt,	/* 44 */
	(uint32)&interrupt,	/* 45 */
	(uint32)&interrupt,	/* 46 */
	(uint32)&interrupt,	/* 47 */
	(uint32)&interrupt,	/* 48 */
	(uint32)&interrupt,	/* 49 */
	(uint32)&interrupt,	/* 50 */
	(uint32)&interrupt,	/* 51 */
	(uint32)&interrupt,	/* 52 */
	(uint32)&interrupt,	/* 53 */
	(uint32)&interrupt,	/* 54 */
	(uint32)&interrupt,	/* 55 */
	(uint32)&interrupt,	/* 56 */
	(uint32)&interrupt,	/* 57 */
	(uint32)&interrupt,	/* 58 */
	(uint32)&interrupt,	/* 59 */
	(uint32)&interrupt,	/* 60 */
	(uint32)&interrupt,	/* 61 */
	(uint32)&interrupt,	/* 62 */
	(uint32)&interrupt,	/* 63 */
	(uint32)&interrupt,	/* 64 */
	(uint32)&interrupt,	/* 65 */
	(uint32)&interrupt,	/* 66 */
	(uint32)&interrupt,	/* 67 */
	(uint32)&interrupt,	/* 68 */
	(uint32)&interrupt,	/* 69 */
	(uint32)&interrupt,	/* 70 */
	(uint32)&interrupt,	/* 71 */
	(uint32)&interrupt,	/* 72 */
	(uint32)&interrupt,	/* 73 */
	(uint32)&interrupt,	/* 74 */
	(uint32)&interrupt,	/* 75 */
	(uint32)&interrupt,	/* 76 */
	(uint32)&interrupt,	/* 77 */
	(uint32)&interrupt,	/* 78 */
	(uint32)&interrupt,	/* 79 */
	(uint32)&interrupt,	/* 80 */
	(uint32)&interrupt,	/* 81 */
	(uint32)&interrupt,	/* 82 */
	(uint32)&interrupt,	/* 83 */
	(uint32)&interrupt,	/* 84 */
	(uint32)&interrupt,	/* 85 */
	(uint32)&interrupt,	/* 86 */
	(uint32)&interrupt,	/* 87 */
	(uint32)&interrupt,	/* 88 */
	(uint32)&interrupt,	/* 89 */
	(uint32)&interrupt,	/* 90 */
	(uint32)&interrupt,	/* 91 */
	(uint32)&interrupt,	/* 92 */
	(uint32)&interrupt,	/* 93 */
	(uint32)&interrupt,	/* 94 */
	(uint32)&interrupt,	/* 95 */
	(uint32)&interrupt,	/* 96 */
	(uint32)&interrupt,	/* 97 */
	(uint32)&interrupt,	/* 98 */
	(uint32)&interrupt,	/* 99 */
	(uint32)&interrupt,	/* 100 */
	(uint32)&interrupt,	/* 101 */
	(uint32)&interrupt,	/* 102 */
	(uint32)&interrupt,	/* 103 */
	(uint32)&interrupt,	/* 104 */
	(uint32)&interrupt,	/* 105 */
	(uint32)&interrupt,	/* 106 */
	(uint32)&interrupt,	/* 107 */
	(uint32)&interrupt,	/* 108 */
	(uint32)&interrupt,	/* 109 */
	(uint32)&interrupt,	/* 110 */
	(uint32)&interrupt,	/* 111 */
	(uint32)&interrupt,	/* 112 */
	(uint32)&interrupt,	/* 113 */
	(uint32)&interrupt,	/* 114 */
	(uint32)&interrupt,	/* 115 */
	(uint32)&interrupt,	/* 116 */
	(uint32)&interrupt,	/* 117 */
	(uint32)&interrupt,	/* 118 */
	(uint32)&interrupt,	/* 119 */
	(uint32)&interrupt,	/* 120 */
	(uint32)&interrupt,	/* 121 */
	(uint32)&interrupt,	/* 122 */
	(uint32)&interrupt,	/* 123 */
	(uint32)&interrupt,	/* 124 */
	(uint32)&interrupt,	/* 125 */
	(uint32)&interrupt,	/* 126 */
	(uint32)&interrupt,	/* 127 */
	(uint32)&interrupt,	/* 128 */
	(uint32)&interrupt,	/* 129 */
	(uint32)&interrupt,	/* 130 */
	(uint32)&interrupt,	/* 131 */
	(uint32)&interrupt,	/* 132 */
	(uint32)&interrupt,	/* 133 */
	(uint32)&interrupt,	/* 134 */
	(uint32)&interrupt,	/* 135 */
	(uint32)&interrupt,	/* 136 */
	(uint32)&interrupt,	/* 137 */
	(uint32)&interrupt,	/* 138 */
	(uint32)&interrupt,	/* 139 */
	(uint32)&interrupt,	/* 140 */
	(uint32)&interrupt,	/* 141 */
	(uint32)&interrupt,	/* 142 */
	(uint32)&interrupt,	/* 143 */
	(uint32)&interrupt,	/* 144 */
	(uint32)&interrupt,	/* 145 */
	(uint32)&interrupt,	/* 146 */
	(uint32)&interrupt,	/* 147 */
	(uint32)&interrupt,	/* 148 */
	(uint32)&interrupt,	/* 149 */
	(uint32)&interrupt,	/* 150 */
	(uint32)&interrupt,	/* 151 */
	(uint32)&interrupt,	/* 152 */
	(uint32)&interrupt,	/* 153 */
	(uint32)&interrupt,	/* 154 */
	(uint32)&interrupt,	/* 155 */
	(uint32)&interrupt,	/* 156 */
	(uint32)&interrupt,	/* 157 */
	(uint32)&interrupt,	/* 158 */
	(uint32)&interrupt,	/* 159 */
	(uint32)&interrupt,	/* 160 */
	(uint32)&interrupt,	/* 161 */
	(uint32)&interrupt,	/* 162 */
	(uint32)&interrupt,	/* 163 */
	(uint32)&interrupt,	/* 164 */
	(uint32)&interrupt,	/* 165 */
	(uint32)&interrupt,	/* 166 */
	(uint32)&interrupt,	/* 167 */
	(uint32)&interrupt,	/* 168 */
	(uint32)&interrupt,	/* 169 */
	(uint32)&interrupt,	/* 170 */
	(uint32)&interrupt,	/* 171 */
	(uint32)&interrupt,	/* 172 */
	(uint32)&interrupt,	/* 173 */
	(uint32)&interrupt,	/* 174 */
	(uint32)&interrupt,	/* 175 */
	(uint32)&interrupt,	/* 176 */
	(uint32)&interrupt,	/* 177 */
	(uint32)&interrupt,	/* 178 */
	(uint32)&interrupt,	/* 179 */
	(uint32)&interrupt,	/* 180 */
	(uint32)&interrupt,	/* 181 */
	(uint32)&interrupt,	/* 182 */
	(uint32)&interrupt,	/* 183 */
	(uint32)&interrupt,	/* 184 */
	(uint32)&interrupt,	/* 185 */
	(uint32)&interrupt,	/* 186 */
	(uint32)&interrupt,	/* 187 */
	(uint32)&interrupt,	/* 188 */
	(uint32)&interrupt,	/* 189 */
	(uint32)&interrupt,	/* 190 */
	(uint32)&interrupt,	/* 191 */
	(uint32)&interrupt,	/* 192 */
	(uint32)&interrupt,	/* 193 */
	(uint32)&interrupt,	/* 194 */
	(uint32)&interrupt,	/* 195 */
	(uint32)&interrupt,	/* 196 */
	(uint32)&interrupt,	/* 197 */
	(uint32)&interrupt,	/* 198 */
	(uint32)&interrupt,	/* 199 */
	(uint32)&interrupt,	/* 200 */
	(uint32)&interrupt,	/* 201 */
	(uint32)&interrupt,	/* 202 */
	(uint32)&interrupt,	/* 203 */
	(uint32)&interrupt,	/* 204 */
	(uint32)&interrupt,	/* 205 */
	(uint32)&interrupt,	/* 206 */
	(uint32)&interrupt,	/* 207 */
	(uint32)&interrupt,	/* 208 */
	(uint32)&interrupt,	/* 209 */
	(uint32)&interrupt,	/* 210 */
	(uint32)&interrupt,	/* 211 */
	(uint32)&interrupt,	/* 212 */
	(uint32)&interrupt,	/* 213 */
	(uint32)&interrupt,	/* 214 */
	(uint32)&interrupt,	/* 215 */
	(uint32)&interrupt,	/* 216 */
	(uint32)&interrupt,	/* 217 */
	(uint32)&interrupt,	/* 218 */
	(uint32)&interrupt,	/* 219 */
	(uint32)&interrupt,	/* 220 */
	(uint32)&interrupt,	/* 221 */
	(uint32)&interrupt,	/* 222 */
	(uint32)&interrupt,	/* 223 */
	(uint32)&interrupt,	/* 224 */
	(uint32)&interrupt,	/* 225 */
	(uint32)&interrupt,	/* 226 */
	(uint32)&interrupt,	/* 227 */
	(uint32)&interrupt,	/* 228 */
	(uint32)&interrupt,	/* 229 */
	(uint32)&interrupt,	/* 230 */
	(uint32)&interrupt,	/* 231 */
	(uint32)&interrupt,	/* 232 */
	(uint32)&interrupt,	/* 233 */
	(uint32)&interrupt,	/* 234 */
	(uint32)&interrupt,	/* 235 */
	(uint32)&interrupt,	/* 236 */
	(uint32)&interrupt,	/* 237 */
	(uint32)&interrupt,	/* 238 */
	(uint32)&interrupt,	/* 239 */
	(uint32)&interrupt,	/* 240 */
	(uint32)&interrupt,	/* 241 */
	(uint32)&interrupt,	/* 242 */
	(uint32)&interrupt,	/* 243 */
	(uint32)&interrupt,	/* 244 */
	(uint32)&interrupt,	/* 245 */
	(uint32)&interrupt,	/* 246 */
	(uint32)&interrupt,	/* 247 */
	(uint32)&interrupt,	/* 248 */
	(uint32)&interrupt,	/* 249 */
	(uint32)&interrupt,	/* 250 */
	(uint32)&interrupt,	/* 251 */
	(uint32)&interrupt,	/* 252 */
	(uint32)&interrupt,	/* 253 */
	(uint32)&interrupt,	/* 254 */
	(uint32)&interrupt,	/* 255 */
	(uint32)&interrupt,	/* 256 */
	(uint32)&interrupt,	/* 257 */
	(uint32)&interrupt,	/* 258 */
	(uint32)&interrupt,	/* 259 */
	(uint32)&interrupt,	/* 260 */
	(uint32)&interrupt,	/* 261 */
	(uint32)&interrupt,	/* 262 */
	(uint32)&interrupt,	/* 263 */
	(uint32)&interrupt,	/* 264 */
	(uint32)&interrupt,	/* 265 */
	(uint32)&interrupt,	/* 266 */
	(uint32)&interrupt,	/* 267 */
	(uint32)&interrupt,	/* 268 */
	(uint32)&interrupt,	/* 269 */
	(uint32)&interrupt,	/* 270 */
	(uint32)&interrupt,	/* 271 */
	(uint32)&interrupt,	/* 272 */
	(uint32)&interrupt,	/* 273 */
	(uint32)&interrupt,	/* 274 */
	(uint32)&interrupt,	/* 275 */
	(uint32)&interrupt,	/* 276 */
	(uint32)&interrupt,	/* 277 */
	(uint32)&interrupt,	/* 278 */
	(uint32)&interrupt,	/* 279 */
	(uint32)&interrupt,	/* 280 */
	(uint32)&interrupt,	/* 281 */
	(uint32)&interrupt,	/* 282 */
	(uint32)&interrupt,	/* 283 */
	(uint32)&interrupt,	/* 284 */
	(uint32)&interrupt,	/* 285 */
	(uint32)&interrupt,	/* 286 */
	(uint32)&interrupt,	/* 287 */
	(uint32)&interrupt,	/* 288 */
	(uint32)&interrupt,	/* 289 */
	(uint32)&interrupt,	/* 290 */
	(uint32)&interrupt,	/* 291 */
	(uint32)&interrupt,	/* 292 */
	(uint32)&interrupt,	/* 293 */
	(uint32)&interrupt,	/* 294 */
	(uint32)&interrupt,	/* 295 */
	(uint32)&interrupt,	/* 296 */
	(uint32)&interrupt,	/* 297 */
	(uint32)&interrupt,	/* 298 */
	(uint32)&interrupt,	/* 299 */
	(uint32)&interrupt,	/* 300 */
	(uint32)&interrupt,	/* 301 */
	(uint32)&interrupt,	/* 302 */
	(uint32)&interrupt,	/* 303 */
	(uint32)&interrupt,	/* 304 */
	(uint32)&interrupt,	/* 305 */
	(uint32)&interrupt,	/* 306 */
	(uint32)&interrupt,	/* 307 */
	(uint32)&interrupt,	/* 308 */
	(uint32)&interrupt,	/* 309 */
	(uint32)&interrupt,	/* 310 */
	(uint32)&interrupt,	/* 311 */
	(uint32)&interrupt,	/* 312 */
	(uint32)&interrupt,	/* 313 */
	(uint32)&interrupt,	/* 314 */
	(uint32)&interrupt,	/* 315 */
	(uint32)&interrupt,	/* 316 */
	(uint32)&interrupt,	/* 317 */
	(uint32)&interrupt,	/* 318 */
	(uint32)&interrupt,	/* 319 */
	(uint32)&interrupt,	/* 320 */
	(uint32)&interrupt,	/* 321 */
	(uint32)&interrupt,	/* 322 */
	(uint32)&interrupt,	/* 323 */
	(uint32)&interrupt,	/* 324 */
	(uint32)&interrupt,	/* 325 */
	(uint32)&interrupt,	/* 326 */
	(uint32)&interrupt,	/* 327 */
	(uint32)&interrupt,	/* 328 */
	(uint32)&interrupt,	/* 329 */
	(uint32)&interrupt,	/* 330 */
	(uint32)&interrupt,	/* 331 */
	(uint32)&interrupt,	/* 332 */
	(uint32)&interrupt,	/* 333 */
	(uint32)&interrupt,	/* 334 */
	(uint32)&interrupt,	/* 335 */
	(uint32)&interrupt,	/* 336 */
	(uint32)&interrupt,	/* 337 */
	(uint32)&interrupt,	/* 338 */
	(uint32)&interrupt,	/* 339 */
	(uint32)&interrupt,	/* 340 */
	(uint32)&interrupt,	/* 341 */
	(uint32)&interrupt,	/* 342 */
	(uint32)&interrupt,	/* 343 */
	(uint32)&interrupt,	/* 344 */
	(uint32)&interrupt,	/* 345 */
	(uint32)&interrupt,	/* 346 */
	(uint32)&interrupt,	/* 347 */
	(uint32)&interrupt,	/* 348 */
	(uint32)&interrupt,	/* 349 */
	(uint32)&interrupt	/* 350 */
};

