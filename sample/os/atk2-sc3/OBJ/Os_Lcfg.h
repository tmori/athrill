/* Os_Lcfg.h */
#ifndef TOPPERS_OS_LCFG_H
#define TOPPERS_OS_LCFG_H

#define TNUM_ALARM				UINT_C(6)
#define TNUM_COUNTER			UINT_C(6)
#define TNUM_HARDCOUNTER		UINT_C(1)
#define TNUM_ISR2				UINT_C(2)
#define TNUM_STD_RESOURCE		UINT_C(2)
#define TNUM_TASK				UINT_C(21)
#define TNUM_TASK_INC_RT		UINT_C(23)
#define TNUM_EXTTASK			UINT_C(6)
#define TNUM_APP_MODE			UINT_C(3)
#define TNUM_SCHEDULETABLE		UINT_C(2)
#define TNUM_IMPLSCHEDULETABLE	UINT_C(0)
#define	TNUM_TFN				UINT_C(6)
#define TNUM_OSAP				UINT_C(7)

/*
 *  Default Definitions of Trace Log Macros
 */

#ifndef TOPPERS_ENABLE_TRACE
#ifndef LOG_USER_MARK
#define LOG_USER_MARK(str)
#endif /* LOG_USER_MARK */
#endif /* TOPPERS_ENABLE_TRACE */

/****** Object TASK ******/

#define Task3	UINT_C(0)
#define Task2	UINT_C(1)
#define Task12	UINT_C(2)
#define Task11	UINT_C(3)
#define Task10	UINT_C(4)
#define MainTask	UINT_C(5)
#define Task9	UINT_C(6)
#define Task8	UINT_C(7)
#define Task5	UINT_C(8)
#define Task4	UINT_C(9)
#define Task1	UINT_C(10)
#define Task7	UINT_C(11)
#define Task6	UINT_C(12)
#define IocTask4	UINT_C(13)
#define IocTask3	UINT_C(14)
#define IocTask2	UINT_C(15)
#define IocTask1	UINT_C(16)
#define Task14	UINT_C(17)
#define Task13	UINT_C(18)
#define HighPriorityTask	UINT_C(19)
#define NonPriTask	UINT_C(20)

/****** Object COUNTER ******/

#define MAIN_HW_COUNTER	UINT_C(0)
#define ActOtherOSAPTskArmCnt	UINT_C(1)
#define SampleCnt	UINT_C(2)
#define SampleCnt3	UINT_C(3)
#define SampleCnt2	UINT_C(4)
#define SchtblSampleCnt	UINT_C(5)

#define OSMAXALLOWEDVALUE_MAIN_HW_COUNTER	((TickType) 536870911)
#define OSTICKSPERBASE_MAIN_HW_COUNTER	((TickType) 10)
#define OSMINCYCLE_MAIN_HW_COUNTER	((TickType) 4000)
#define OSMAXALLOWEDVALUE_ActOtherOSAPTskArmCnt	((TickType) 99)
#define OSTICKSPERBASE_ActOtherOSAPTskArmCnt	((TickType) 10)
#define OSMINCYCLE_ActOtherOSAPTskArmCnt	((TickType) 10)
#define OSMAXALLOWEDVALUE_SampleCnt	((TickType) 99)
#define OSTICKSPERBASE_SampleCnt	((TickType) 10)
#define OSMINCYCLE_SampleCnt	((TickType) 10)
#define OSMAXALLOWEDVALUE_SampleCnt3	((TickType) 99)
#define OSTICKSPERBASE_SampleCnt3	((TickType) 10)
#define OSMINCYCLE_SampleCnt3	((TickType) 10)
#define OSMAXALLOWEDVALUE_SampleCnt2	((TickType) 99)
#define OSTICKSPERBASE_SampleCnt2	((TickType) 10)
#define OSMINCYCLE_SampleCnt2	((TickType) 10)
#define OSMAXALLOWEDVALUE_SchtblSampleCnt	((TickType) 99)
#define OSTICKSPERBASE_SchtblSampleCnt	((TickType) 10)
#define OSMINCYCLE_SchtblSampleCnt	((TickType) 10)

#define OS_TICKS2SEC_MAIN_HW_COUNTER(tick)	(((PhysicalTimeType)125U) * (tick) / 1000000000U)	/* (0.000000125 * 1000000000) * (tick) / 1000000000 */
#define OS_TICKS2MS_MAIN_HW_COUNTER(tick)	(((PhysicalTimeType)125U) * (tick) / 1000000U)		/* (0.000000125 * 1000000000) * (tick) / 1000000 */
#define OS_TICKS2US_MAIN_HW_COUNTER(tick)	(((PhysicalTimeType)125U) * (tick) / 1000U)			/* (0.000000125 * 1000000000) * (tick) / 1000 */
#define OS_TICKS2NS_MAIN_HW_COUNTER(tick)	(((PhysicalTimeType)125U) * (tick))					/* (0.000000125 * 1000000000) * (tick) */

/****** Object ALARM ******/

#define ActTskArm	UINT_C(0)
#define MainCycArm	UINT_C(1)
#define ActOtherOSAPTskArm	UINT_C(2)
#define SetEvtArm	UINT_C(3)
#define SampleAlm2	UINT_C(4)
#define SampleAlm1	UINT_C(5)

/****** Object SCHEDULETABLE ******/

#define scheduletable2	UINT_C(0)
#define scheduletable1	UINT_C(1)

/****** Object RESOURCE ******/

#define CntRes	UINT_C(0)
#define TskLevelRes	UINT_C(1)
#define GroupRes	UINT_C(2)

/****** Object ISR ******/

#define C2ISR_for_MAIN_HW_COUNTER	UINT_C(0)
#define RxHwSerialInt	UINT_C(1)

/****** Object APPMODE ******/

#define AppMode3	UINT_C(0)
#define AppMode2	UINT_C(1)
#define AppMode1	UINT_C(2)

/****** Object EVENT ******/
#define T11Evt	UINT_C(0x00000001)
#define T12Evt	UINT_C(0x00000001)
#define T2Evt	UINT_C(0x00000001)
#define MainEvt	UINT_C(0x00000001)
#define T3Evt	UINT_C(0x00010000)
#define T10Evt	UINT_C(0x00000001)

/****** Object Trusted Function ******/

#define actioctask4	UINT_C(0)
#define tfnt4	UINT_C(1)
#define tfnt3	UINT_C(2)
#define tfnt2	UINT_C(3)
#define tfnt1	UINT_C(4)
#define actioctask3	UINT_C(5)

/****** Object OSApplication ******/
#define NT_osap2 UINT_C(0)
#define NT_osap1 UINT_C(1)
#define KT_osap2 UINT_C(2)
#define KT_osap1 UINT_C(3)
#define TRUSTED_OSAP_for_MAIN_HW_COUNTER UINT_C(4)
#define TRUSTED_OSAP_for_RxHwSerialInt UINT_C(5)
#define TRUSTED_OSAP_for_NON_USE_PE_ROM UINT_C(6)



/****** Object IOC ******/
/* IOC ID */
#define IOC_QUE	UINT_C(0)
#define IOC_DEQUE	UINT_C(1)

/* Wrapper ID */
#define IOC_WRAPPER_0	UINT_C(0)
#define IOC_WRAPPER_1	UINT_C(1)
#define IOC_WRAPPER_2	UINT_C(2)

#define TNUM_IOC			UINT_C(2)
#define TNUM_QUEUEIOC		UINT_C(1)
#define TNUM_IOC_WRAPPER	UINT_C(3)

#ifndef TOPPERS_MACRO_ONLY

/*
 *  Interrupt Management Functions
 */

extern void _kernel_inthdr_80(void);
extern void _kernel_inthdr_35(void);
extern ISR(C2ISR_for_MAIN_HW_COUNTER);
extern ISR(RxHwSerialInt);

#ifdef TOPPERS_ENABLE_TRACE
extern const char8 *atk2_appid_str(AppModeType id);
extern const char8 *atk2_tskid_str(TaskType id);
extern const char8 *atk2_isrid_str(ISRType id);
extern const char8 *atk2_cntid_str(CounterType id);
extern const char8 *atk2_almid_str(AlarmType id);
extern const char8 *atk2_resid_str(ResourceType id);
extern const char8 *atk2_schtblid_str(ScheduleTableType id);
extern const char8 *atk2_evtid_str(TaskType task, EventMaskType event);
extern const char8 *atk2_osapid_str(ApplicationType id);
extern const char8 *atk2_iocid_str(IocType id);
extern const char8 *atk2_tfnid_str(TrustedFunctionIndexType id);
#endif /* TOPPERS_ENABLE_TRACE */

extern void init_hwcounter_MAIN_HW_COUNTER(TickType maxval, TimeType nspertick);
extern void start_hwcounter_MAIN_HW_COUNTER(void);
extern void stop_hwcounter_MAIN_HW_COUNTER(void);
extern void set_hwcounter_MAIN_HW_COUNTER(TickType exprtick);
extern TickType get_hwcounter_MAIN_HW_COUNTER(void);
extern void cancel_hwcounter_MAIN_HW_COUNTER(void);
extern void trigger_hwcounter_MAIN_HW_COUNTER(void);
extern void int_clear_hwcounter_MAIN_HW_COUNTER(void);
extern void int_cancel_hwcounter_MAIN_HW_COUNTER(void);
extern void increment_hwcounter_MAIN_HW_COUNTER(void);

/******** Trusted Function ********/
extern TRUSTEDFUNCTION(TRUSTED_actioctask4, FunctionIndex, FunctionParams);
extern TRUSTEDFUNCTION(TRUSTED_tfnt4, FunctionIndex, FunctionParams);
extern TRUSTEDFUNCTION(TRUSTED_tfnt3, FunctionIndex, FunctionParams);
extern TRUSTEDFUNCTION(TRUSTED_tfnt2, FunctionIndex, FunctionParams);
extern TRUSTEDFUNCTION(TRUSTED_tfnt1, FunctionIndex, FunctionParams);
extern TRUSTEDFUNCTION(TRUSTED_actioctask3, FunctionIndex, FunctionParams);

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_OS_LCFG_H */

