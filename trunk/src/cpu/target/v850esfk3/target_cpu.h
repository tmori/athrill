#ifndef _CPU_REGISTER_H_
#define _CPU_REGISTER_H_

#include "cpu_dec/op_dec.h"
#include "std_types.h"

#define CPU_GREG_NUM	(32U)
#define CPU_SYSREG_NUM	(32U)

#define CPU_REG_UINT_MAX	0xFFFFFFFFULL
#define CPU_REG_PLUS_MAX	2147483647LL
#define CPU_REG_MINUS_MAX	-2147483648LL

#define CPU_REG_SP		(3)
#define CPU_REG_EP		(30)
#define CPU_REG_LP		(31)
typedef struct {
	uint32 pc;
	sint32 r[CPU_GREG_NUM];
	uint32 eipc;
	uint32 eipsw;
	uint32 ecr;
	uint32 psw;
	uint32 fepc;
	uint32 fepsw;
	uint32 ctbp;
} CpuRegisterType;

typedef struct {
	CoreIdType			core_id;
	CpuRegisterType 	reg;
	bool				is_halt;
	uint16 				current_code[OP_DECODE_MAX];
	OpDecodedCodeType	decoded_code;
} TargetCoreType;

#endif /* _CPU_REGISTER_H_ */
