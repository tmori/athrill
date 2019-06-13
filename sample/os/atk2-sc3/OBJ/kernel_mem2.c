#include "kernel_int.h"
#include "Os_Lcfg.h"
#ifndef TOPPERS_EMPTY_LABEL
#define TOPPERS_EMPTY_LABEL(x, y) x y[0]
#endif
/*
 *  Include Directives (#include)
 */

#include "sample2.h"
#include "sample1.h"
#include "target_serial.h"
#include "target_hw_counter.h"
#include "target_mem.h"

const uint32 tnum_meminib = 17U;

void *const memtop_table[17] = {
	(void *)(stack_1)  /* 2, 2, TACP_KERNEL */
};

const MEMINIB meminib_table[17] = {{ TA_NULL, 0U, 0U, 0U }};

/*
 *	Data Section Management Functions
 */

#define TNUM_DATASEC	0

const uint32 tnum_datasec = TNUM_DATASEC;

//TOPPERS_EMPTY_LABEL(const DATASECINIB, datasecinib_table);


/*
 *	BSS Section Management Functions
 */

#define TNUM_BSSSEC	11

const uint32 tnum_bsssec = TNUM_BSSSEC;

const BSSSECINIB bsssecinib_table[TNUM_BSSSEC] = {{0U, 0U}};


extern const uint32 tnum_shared_mem;
extern uint8 * const shared_meminib_table[];

extern const uint32 __attribute__((section(".rodata_kernel"), aligned(4))) tnum_datasec;
extern const DATASECINIB __attribute__((section(".rodata_kernel"), aligned(4))) datasecinib_table[];
extern const uint32 __attribute__((section(".rodata_kernel"), aligned(4))) tnum_bsssec;
extern const BSSSECINIB __attribute__((section(".rodata_kernel"), aligned(4))) bsssecinib_table[];
extern StackType _kernel_sstack_Task3[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_ustack_Task3[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_sstack_Task2[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_ustack_Task2[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_sstack_Task12[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_ustack_Task12[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_sstack_Task11[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_ustack_Task11[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_sstack_Task10[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_ustack_Task10[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_sstack_MainTask[COUNT_STK_T(ROUND_STK_T((512U) + (512U)))];
extern StackType _kernel_sstack_restart_NT_osap2[COUNT_STK_T(ROUND_STK_T(1024U))];
extern StackType _kernel_sstack_restart_NT_osap1[COUNT_STK_T(ROUND_STK_T(1024U))];
extern StackType _kernel_shared_sstack_1[COUNT_STK_T(ROUND_STK_T((512U) + (512U)))];
extern StackType _kernel_shared_ustack_4[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_shared_sstack_4[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_shared_ustack_6[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_shared_sstack_6[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_shared_ustack_8[COUNT_STK_T(ROUND_STK_T(768U))];
extern StackType _kernel_shared_sstack_8[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_shared_ustack_9[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_shared_sstack_9[COUNT_STK_T(ROUND_STK_T((512U) + (512U)))];
extern StackType _kernel_shared_ustack_15[COUNT_STK_T(ROUND_STK_T(512U))];
extern StackType _kernel_shared_sstack_15[COUNT_STK_T(ROUND_STK_T(512U))];

extern TASK(Task3);
extern TASK(Task2);
extern TASK(Task12);
extern TASK(Task11);
extern TASK(Task10);
extern TASK(MainTask);
extern TASK(Task9);
extern TASK(Task8);
extern TASK(Task5);
extern TASK(Task4);
extern TASK(Task1);
extern TASK(Task7);
extern TASK(Task6);
extern TASK(IocTask4);
extern TASK(IocTask3);
extern TASK(IocTask2);
extern TASK(IocTask1);
extern TASK(Task14);
extern TASK(Task13);
extern TASK(HighPriorityTask);
extern TASK(NonPriTask);

extern TCB tcb_table[];

extern OSAPCB osapcb_table[];

/* Task Initialization Block */
const TINIB tinib_table[TNUM_TASK_INC_RT] = {
	{
		&TASKNAME(Task3),
		{
			ROUND_STK_T(512U),
			((void *)((uint8 *)(_kernel_sstack_Task3) + (ROUND_STK_T(512U)))),
			ROUND_STK_T(512U),
 			((void *)((uint8 *)(_kernel_ustack_Task3) + (ROUND_STK_T(512U)))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000002U,
		3,
		3,
		(1U) - 1U,
		0x00000001U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(Task2),
		{
			ROUND_STK_T(512U),
			((void *)((uint8 *)(_kernel_sstack_Task2) + (ROUND_STK_T(512U)))),
			ROUND_STK_T(512U),
 			((void *)((uint8 *)(_kernel_ustack_Task2) + (ROUND_STK_T(512U)))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000003U,
		8,
		8,
		(1U) - 1U,
		0x00000000U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(Task12),
		{
			ROUND_STK_T(512U),
			((void *)((uint8 *)(_kernel_sstack_Task12) + (ROUND_STK_T(512U)))),
			ROUND_STK_T(512U),
 			((void *)((uint8 *)(_kernel_ustack_Task12) + (ROUND_STK_T(512U)))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000002U,
		2,
		0,
		(1U) - 1U,
		0x00000000U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(Task11),
		{
			ROUND_STK_T(512U),
			((void *)((uint8 *)(_kernel_sstack_Task11) + (ROUND_STK_T(512U)))),
			ROUND_STK_T(512U),
 			((void *)((uint8 *)(_kernel_ustack_Task11) + (ROUND_STK_T(512U)))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000002U,
		2,
		0,
		(1U) - 1U,
		0x00000000U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(Task10),
		{
			ROUND_STK_T(512U),
			((void *)((uint8 *)(_kernel_sstack_Task10) + (ROUND_STK_T(512U)))),
			ROUND_STK_T(512U),
 			((void *)((uint8 *)(_kernel_ustack_Task10) + (ROUND_STK_T(512U)))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000002U,
		2,
		0,
		(1U) - 1U,
		0x00000000U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(MainTask),
		{
			ROUND_STK_T((512U) + (512U)),
			((void *)((uint8 *)(_kernel_sstack_MainTask) + (ROUND_STK_T((512U) + (512U))))),
			0,
			0,
		},
		&(osapcb_table[KT_osap1]),
		0x00000002U,
		1,
		0,
		(1U) - 1U,
		0x00000007U,
		{
			0,
			0,
		},
	},
	{
		&TASKNAME(Task9),
		{
			ROUND_STK_T(512U),
			((void *)((uint8 *)(_kernel_shared_sstack_8) + (ROUND_STK_T(512U)))),
			768U,
 			((void *)((uint8 *)(_kernel_shared_ustack_8) + (768U))),
		},
		&(osapcb_table[NT_osap2]),
		0x00000001U,
		7,
		7,
		(5U) - 1U,
		0x00000000U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(Task8),
		{
			ROUND_STK_T(512U),
			((void *)((uint8 *)(_kernel_shared_sstack_8) + (ROUND_STK_T(512U)))),
			768U,
 			((void *)((uint8 *)(_kernel_shared_ustack_8) + (768U))),
		},
		&(osapcb_table[NT_osap2]),
		0x00000001U,
		7,
		7,
		(5U) - 1U,
		0x00000000U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(Task5),
		{
			ROUND_STK_T((512U) + (512U)),
			((void *)((uint8 *)(_kernel_shared_sstack_9) + (ROUND_STK_T((512U) + (512U))))),
			512U,
 			((void *)((uint8 *)(_kernel_shared_ustack_9) + (512U))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000002U,
		6,
		6,
		(5U) - 1U,
		0x00000000U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(Task4),
		{
			ROUND_STK_T(512U),
			((void *)((uint8 *)(_kernel_shared_sstack_6) + (ROUND_STK_T(512U)))),
			512U,
 			((void *)((uint8 *)(_kernel_shared_ustack_6) + (512U))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000003U,
		9,
		6,
		(5U) - 1U,
		0x00000000U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(Task1),
		{
			ROUND_STK_T(512U),
			((void *)((uint8 *)(_kernel_shared_sstack_4) + (ROUND_STK_T(512U)))),
			512U,
 			((void *)((uint8 *)(_kernel_shared_ustack_4) + (512U))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000002U,
		11,
		11,
		(8U) - 1U,
		0x00000002U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(Task7),
		{
			512U,
			((void *)((uint8 *)(stack_00) + (512U))),
			0,
			0,
		},
		&(osapcb_table[KT_osap1]),
		0x00000002U,
		7,
		7,
		(5U) - 1U,
		0x00000000U,
		{
			0,
			0,
		},
	},
	{
		&TASKNAME(Task6),
		{
			512U,
			((void *)((uint8 *)(stack_00) + (512U))),
			0,
			0,
		},
		&(osapcb_table[KT_osap1]),
		0x00000000U,
		6,
		6,
		(5U) - 1U,
		0x00000000U,
		{
			0,
			0,
		},
	},
	{
		&TASKNAME(IocTask4),
		{
			ROUND_STK_T((512U) + (512U)),
			((void *)((uint8 *)(_kernel_shared_sstack_9) + (ROUND_STK_T((512U) + (512U))))),
			0,
			0,
		},
		&(osapcb_table[KT_osap2]),
		0x00000000U,
		6,
		6,
		(1U) - 1U,
		0x00000000U,
		{
			0,
			0,
		},
	},
	{
		&TASKNAME(IocTask3),
		{
			ROUND_STK_T((512U) + (512U)),
			((void *)((uint8 *)(_kernel_shared_sstack_9) + (ROUND_STK_T((512U) + (512U))))),
			0,
			0,
		},
		&(osapcb_table[KT_osap1]),
		0x00000000U,
		6,
		6,
		(1U) - 1U,
		0x00000000U,
		{
			0,
			0,
		},
	},
	{
		&TASKNAME(IocTask2),
		{
			ROUND_STK_T((512U) + (512U)),
			((void *)((uint8 *)(_kernel_shared_sstack_9) + (ROUND_STK_T((512U) + (512U))))),
			512U,
 			((void *)((uint8 *)(_kernel_shared_ustack_9) + (512U))),
		},
		&(osapcb_table[NT_osap2]),
		0x00000003U,
		6,
		6,
		(1U) - 1U,
		0x00000000U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(IocTask1),
		{
			ROUND_STK_T((512U) + (512U)),
			((void *)((uint8 *)(_kernel_shared_sstack_9) + (ROUND_STK_T((512U) + (512U))))),
			512U,
 			((void *)((uint8 *)(_kernel_shared_ustack_9) + (512U))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000003U,
		6,
		6,
		(1U) - 1U,
		0x00000000U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(Task14),
		{
			512U,
			((void *)((uint8 *)(stack_2) + (512U))),
			512U,
 			((void *)((uint8 *)(stack_1) + (512U))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000002U,
		7,
		7,
		(5U) - 1U,
		0x00000000U,
		{
			(uint8 *)stack_1,
			(uint8 *)((uint32)stack_1 + 512U),
		},
	},
	{
		&TASKNAME(Task13),
		{
			512U,
			((void *)((uint8 *)(stack_2) + (512U))),
			512U,
 			((void *)((uint8 *)(stack_1) + (512U))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000002U,
		6,
		6,
		(5U) - 1U,
		0x00000000U,
		{
			(uint8 *)stack_1,
			(uint8 *)((uint32)stack_1 + 512U),
		},
	},
	{
		&TASKNAME(HighPriorityTask),
		{
			ROUND_STK_T(512U),
			((void *)((uint8 *)(_kernel_shared_sstack_15) + (ROUND_STK_T(512U)))),
			512U,
 			((void *)((uint8 *)(_kernel_shared_ustack_15) + (512U))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000002U,
		0,
		0,
		(1U) - 1U,
		0x00000000U,
		{
			/*dummy at pass2*/
			0,
			0,
		},
	},
	{
		&TASKNAME(NonPriTask),
		{
			ROUND_STK_T((512U) + (512U)),
			((void *)((uint8 *)(_kernel_shared_sstack_1) + (ROUND_STK_T((512U) + (512U))))),
			0,
			0,
		},
		&(osapcb_table[KT_osap1]),
		0x00000000U,
		14,
		0,
		(8U) - 1U,
		0x00000000U,
		{
			0,
			0,
		},
	},
	{
		/* this TINIB is genereted by OS */
		NULL,
		{
			ROUND_STK_T(1024U),
			((void *)((uint8 *)(_kernel_sstack_restart_NT_osap2) + (ROUND_STK_T(1024U)))),
			0U,
 			((void *)((uint8 *)(NULL) + (0U))),
		},
		&(osapcb_table[NT_osap2]),
		0x00000001U,
		0,
		0,
		(1U) - 1U,
		0x00000000U,
		{
			0,
			0,
		},
	},
	{
		/* this TINIB is genereted by OS */
		NULL,
		{
			ROUND_STK_T(1024U),
			((void *)((uint8 *)(_kernel_sstack_restart_NT_osap1) + (ROUND_STK_T(1024U)))),
			0U,
 			((void *)((uint8 *)(NULL) + (0U))),
		},
		&(osapcb_table[NT_osap1]),
		0x00000002U,
		0,
		0,
		(1U) - 1U,
		0x00000000U,
		{
			0,
			0,
		},
	}
};

const OSAPINIB osapinib_table[TNUM_OSAP] = {
	{
		&tcb_table[21],
		TA_NONTRUSTED,
		0x00000001U,
		{
			0, 0, /* MPU1 */
			0, 0, /* MPU2 */
			0, 0, /* MPU3 */
			0, 0, /* MPU4 */
			0, 0, /* MPU5 */
			0, 0, /* MPU6 */
			NULL,
			0U,
			( (uint32)0 ),/* MPRC */
		}

	},
	{
		&tcb_table[22],
		TA_NONTRUSTED,
		0x00000002U,
		{
			0, 0, /* MPU1 */
			0, 0, /* MPU2 */
			0, 0, /* MPU3 */
			0, 0, /* MPU4 */
			0, 0, /* MPU5 */
			0, 0, /* MPU6 */
			NULL,
			0U,
			( (uint32)0 ),/* MPRC */
		}

	},
	{
		NULL,
		TA_TRUSTED,
		0x00000000U,
		{
			0, 0, /* MPU1 */
			0, 0, /* MPU2 */
			0, 0, /* MPU3 */
			0, 0, /* MPU4 */
			0, 0, /* MPU5 */
			0, 0, /* MPU6 */
			NULL,
			0U,
			( (uint32)0 ),/* MPRC */
		}

	},
	{
		NULL,
		TA_TRUSTED,
		0x00000000U,
		{
			0, 0, /* MPU1 */
			0, 0, /* MPU2 */
			0, 0, /* MPU3 */
			0, 0, /* MPU4 */
			0, 0, /* MPU5 */
			0, 0, /* MPU6 */
			NULL,
			0U,
			( (uint32)0 ),/* MPRC */
		}

	},
	{
		NULL,
		TA_TRUSTED,
		0x00000000U,
		{
			0, 0, /* MPU1 */
			0, 0, /* MPU2 */
			0, 0, /* MPU3 */
			0, 0, /* MPU4 */
			0, 0, /* MPU5 */
			0, 0, /* MPU6 */
			NULL,
			0U,
			( (uint32)0 ),/* MPRC */
		}

	},
	{
		NULL,
		TA_TRUSTED,
		0x00000000U,
		{
			0, 0, /* MPU1 */
			0, 0, /* MPU2 */
			0, 0, /* MPU3 */
			0, 0, /* MPU4 */
			0, 0, /* MPU5 */
			0, 0, /* MPU6 */
			NULL,
			0U,
			( (uint32)0 ),/* MPRC */
		}

	},
	{
		NULL,
		TA_TRUSTED,
		0x00000000U,
		{
			0, 0, /* MPU1 */
			0, 0, /* MPU2 */
			0, 0, /* MPU3 */
			0, 0, /* MPU4 */
			0, 0, /* MPU5 */
			0, 0, /* MPU6 */
			NULL,
			0U,
			( (uint32)0 ),/* MPRC */
		}

	}
};


uint8 * const shared_meminib_table[9] = {
	((uint8 *)NULL),	/* MPUL 16 */
	((uint8 *)NULL),	/* MPUA 16 */
	((uint8 *)NULL),	/* MPAT 16 */
	((uint8 *)NULL),	/* MPUL 15 */
	((uint8 *)NULL),	/* MPUA 15 */
	((uint8 *)NULL),	/* MPAT 15 */
	((uint8 *)NULL),	/* MPUL 14 */
	((uint8 *)NULL),	/* MPUA 14 */
	((uint8 *)NULL),	/* MPAT 14 */
};


