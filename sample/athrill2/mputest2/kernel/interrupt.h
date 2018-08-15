#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

typedef unsigned char uint8;
typedef unsigned int uint32;
typedef unsigned char bool;

/*
 *	割込み制御レジスタの番地を算出するためのマクロ
 *
 *	割込み制御レジスタは割込み番号順に並んでいるため，
 *	ベースアドレスからのオフセットでアドレスを求めることができる．
 */

#define INTREG_BASE				(0xFFFFF110)
#define INTREG_ADDRESS(intno)	(INTREG_BASE + ((intno) * 2U))

extern void x_enable_int(int intno);
extern void x_clear_int(int intno);

#endif /* _INTERRUPT_H_ */