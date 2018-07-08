#ifndef _TEST_REG_H_
#define _TEST_REG_H_

#include "cpu_config.h"

/*
 * 汎用レジスタの位置づけ
 * 以下のスクラッチレジスタを使用する．
 * R20
 * R21
 * R22
 * R23
 * R24
 * R25
 * R26
 * R27
 * R28
 * R29
 */
#define	REG_OUT					r20
#define	REG_IN					r21
#define	REG_ADDR				r22


#define REG_OUT_ADDR			CPU_CONFIG_DEBUG_REGADDR_R20
#define REG_IN_ADDR				CPU_CONFIG_DEBUG_REGADDR_R21

#define STACK_SIZE              1024
#define USER_STACK_SIZE         1024


#define POS_R1      72
#define POS_R6      68
#define POS_R7      64
#define POS_R8      60
#define POS_R9      56
#define POS_R10     52
#define POS_R11     48
#define POS_R12     44
#define POS_R13     40
#define POS_R14     36
#define POS_R15     32
#define POS_R16     28
#define POS_R17     24
#define POS_R18     20
#define POS_R19     16
#define POS_EP      12
#define POS_R31     8
#define POS_EIPC    4
#define POS_EIPSW   0
#define INTR_PUSH_SPSZ  -80
#define INTR_POP_SPSZ    80


#define SYSCALL_SIZE    2

#endif /* _TEST_REG_H_ */
