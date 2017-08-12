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

#endif /* _TEST_REG_H_ */
