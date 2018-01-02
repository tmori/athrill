/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2015 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2014 by FUJISOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2014 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2014 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2014 by Witz Corporation, JAPAN
 *
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 *
 *  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
 *  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
 *  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
 *  用する者に対して，AUTOSARパートナーになることを求めている．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: prc_insn.h 549 2015-12-30 10:06:17Z ertl-honda $
 */

/*
 *		プロセッサの特殊命令のインライン関数定義（V850用）
 */
#ifndef TOPPERS_PRC_INSN_H
#define TOPPERS_PRC_INSN_H

#ifndef TOPPERS_MACRO_ONLY

#define V850_MEMORY_CHANGED	Asm("" ::: "memory");

LOCAL_INLINE uint32
current_psw(void)
{
	volatile uint32 psw;
	Asm("   ldsr	r0,31 \n" /* Select CPU function grp */
		"\t stsr 5 , %0 \n"
		: "=r" (psw) :);
	return(psw);
}

LOCAL_INLINE void
set_psw(uint32 psw)
{
	Asm("   ldsr	r0,31 \n"  /* Select CPU function grp */
		"\t ldsr %0 , 5 \n"
		:  : "r" (psw));
	return;
}

LOCAL_INLINE void
set_psw_wo_fgs(uint32 psw)
{
	Asm("\t ldsr %0 , 5 \n"
		:  : "r" (psw));
	return;
}

LOCAL_INLINE void
disable_int(void)
{
	Asm("	di");
}

LOCAL_INLINE void
enable_int(void)
{
	Asm("	ei");
}

LOCAL_INLINE void
set_bit(uint8 bit_offset, uint32 addr)
{
	uint32 any;

	Asm("mov %1 , %0;"
		"set1 %2 , 0[%0]" : "=r" (any) : "i" (addr), "i" (bit_offset));
}

LOCAL_INLINE void
clr_bit(uint8 bit_offset, uint32 addr)
{
	uint32 any;

	Asm("mov %1 , %0;"
		"clr1 %2 , 0[%0]" : "=r" (any) : "i" (addr), "i" (bit_offset));
}

#ifdef __v850e2v3__


//TODO
#define PEID_ADDR	0x06FF6490

LOCAL_INLINE uint32
current_peid(void)
{
	uint32 *peidaddr;
	uint32 peid;

	Asm("mov %1, %0" : "=r" (peidaddr) : "0" (PEID_ADDR));
	Asm("ld.w 0[%1], %0" : "=r" (peid) : "r" (peidaddr));
	return peid;
}

LOCAL_INLINE boolean
acquire_lock_ldlstc(uint16 *p_lock)
{
	uint16 locked = 1;
	Asm("caxi [%1], r0, %0"
		: "+r" (locked), "+r" (p_lock));

	return (locked == 0);
}

LOCAL_INLINE void
release_lock_ldlstc(uint16 *p_lock)
{
	*p_lock = 0;
}
#elif defined(__v850e3v5__)

/*
 *  V850E3V5用の割込みコントローラ操作ルーチン
 */
LOCAL_INLINE void
set_pmr(uint16 pmr)
{
	uint32 psw;

	/* PMR must be set in di sate(PSW.ID = 1) */
	psw = current_psw();
	disable_int();

	Asm("ldsr %0, sr11, 2" ::"r" (pmr));

	set_psw_wo_fgs(psw);
}

LOCAL_INLINE uint16
get_ispr(void)
{
	uint16 ispr;
	Asm("stsr sr10, %0, 2" : "=r" (ispr) :);
	return(ispr);
}

LOCAL_INLINE void
clear_ispr(void)
{
	uint32 psw;

	/* ISPR must be set in di sate(PSW.ID = 1) */
	psw = current_psw();
	disable_int();

	Asm("ldsr %0, sr13, 2" ::"r" (1));  /* INTCFG = 1; ISPR を書き換え可能に */
	Asm("ldsr %0, sr10, 2" ::"r" (0));  /* ISPR = 0 */
	Asm("ldsr %0, sr13, 2" ::"r" (0));  /* INTCFG = 0; ISPR を書き換え禁止に(自動更新に) */

	set_psw_wo_fgs(psw);
}

LOCAL_INLINE void
set_intbp(uint32 intbp)
{
	uint32 psw;

	/* INTBP must be set in di sate(PSW.ID = 1) */
	psw = current_psw();
	disable_int();

	Asm("   ldsr	r0,31 \n"  /* Select CPU function grp */
		"\t ldsr %0, 4, 1 \n"
		:  : "r" (intbp));

	set_psw_wo_fgs(psw);
}

LOCAL_INLINE uint32
current_peid(void)
{
	uint32 htcfg0_val;
	Asm("stsr sr0, %0, 2" : "=r" (htcfg0_val) :);
	return(((htcfg0_val >> 16) & 0x03));
}

LOCAL_INLINE uint32
get_eiic(void)
{
	uint32 eiic;
	Asm("stsr sr13, %0, 0" : "=r" (eiic) :);
	return(eiic);
}

LOCAL_INLINE boolean
acquire_lock_ldlstc(uint32 *p_lock)
{
	uint32 locked = 0;
	Asm("1: ldl.w [%1], r21 \n"
		"   cmp   r0, r21   \n"
		"   bnz   2f        \n"
		"   mov   1, r21    \n"
		"   stc.w r21, [%1] \n"
		"   cmp   r0, r21   \n"
		"   be    2f        \n"
		"   mov   1, %0     \n"
		"   br    3f        \n"
		"2:                 \n"
		"   mov   0, %0     \n"
		"3:                 \n"
		: "=r" (locked)
		: "r" (p_lock)
		: "cc", "r21");

	return(locked == 1);
}

LOCAL_INLINE void
release_lock_ldlstc(uint32 *p_lock)
{
	*p_lock = 0;
}

#else /* __v850e3v5__ */
#error please define ether __v850e2v3__ or __v850e3v5__
#endif /* __v850e2v3__ */

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_PRC_INSN_H */
