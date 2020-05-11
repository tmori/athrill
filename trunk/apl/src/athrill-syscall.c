#include "athrill_syscall.h"

void athrill_syscall(AthrillSyscallArgType *param)
{
	athrill_device_func_call = (sys_addr)(param);
}
