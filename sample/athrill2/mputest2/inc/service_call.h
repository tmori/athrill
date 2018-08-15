#ifndef _SERVICE_CALL_H_
#define _SERVICE_CALL_H_

#define SERVICE_CALL_USER
#include "kernel_service.h"


extern void user1_task(void);
extern void user1_internal_func(void);
extern volatile unsigned int user1_task_data __attribute__ ((section(".bss_noclr_user1")));

extern void user2_task(void);
extern void user2_internal_func(void);
extern volatile unsigned int user2_task_data __attribute__ ((section(".bss_noclr_user2")));

extern void kernel_internal_func(void);
extern volatile unsigned int kernel_task_data __attribute__ ((section(".bss_noclr_kernel")));

extern volatile unsigned int user_shared_data __attribute__ ((section(".bss_noclr_user")));
extern unsigned char user1_test_result[10] __attribute__ ((section(".bss_noclr_user")));
extern unsigned char user2_test_result[10] __attribute__ ((section(".bss_noclr_user")));

typedef enum {
	KernelMpuTestMode_User1 = 0,
	KernelMpuTestMode_User2,
	KernelMpuTestMode_Kernel,
} KernelMpuTestModeType;

typedef enum {
	KernelMpuTestNo_0 = 1,
	KernelMpuTestNo_1,
	KernelMpuTestNo_2,
	KernelMpuTestNo_3,
	KernelMpuTestNo_4,
	KernelMpuTestNo_5,
	KernelMpuTestNo_6,
} KernelMpuTestNoType;
#define KernelMpuTestNo_Num KernelMpuTestNo_6

extern KernelMpuTestNoType kernel_mputest_no  __attribute__ ((section(".bss_noclr_user")));

#endif /* _SERVICE_CALL_H_ */