/*
 *		cfg1_out.cをリンクするために必要なスタブの定義
 *
 *  $Id:  $
 */

#include <kernel.h>

void sta_ker(void){}
STK_T *const	_kernel_istkpt;
/*
 *  offset.hを生成するための定義
 */
const uint8_t	MAGIC_1 = 0x12;
const uint16_t	MAGIC_2 = 0x1234;
const uint32_t	MAGIC_4 = 0x12345678;
