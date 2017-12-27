/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2014 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2014 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2014 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2014 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2014 by Witz Corporation, JAPAN
 *
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
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
 *  $Id: prc_insn.h 187 2015-06-25 03:39:04Z t_ishikawa $
 */

/*
 *		プロセッサの特殊命令のインライン関数定義（V850用）
 */
#ifndef TOPPERS_PRC_INSN_H
#define TOPPERS_PRC_INSN_H

#ifndef TOPPERS_MACRO_ONLY

#include <v800_ghs.h>

/*
 *  関数のアライン単位
 *  GHSでは2バイト境界にも配置する．
 */
#define CHECK_FUNC_ALIGN	2

/*
 *  メモリ再読込が必要なことをコンパイラに伝えるためのダミー関数
 */
extern void v850_memory_changed(void);
extern void v850_sync_memory(void);

/*
 *  メモリの再読み込みが必要なことをコンパイラに伝える
 */
#define V850_MEMORY_CHANGED	v850_memory_changed();
#define SYNCM	v850_sync_memory();

LOCAL_INLINE void
disable_int(void)
{
	__DI();
}

LOCAL_INLINE void
enable_int(void)
{
	__EI();
}

#ifdef __v850e2v3__

LOCAL_INLINE uint32
current_psw(void)
{
	__LDSR(31, 0);     /* Select CPU function grp */
	return(__GETSR());
}

LOCAL_INLINE void
set_psw(uint32 psw)
{
	__LDSR(31, 0);     /* Select CPU function grp */
	__SETSR(psw);
}

#define LDSR_REG(reg, val) {\
    __LDSR(reg, val);\
}

#define STSR_REG(reg, val) {\
    val = __STSR(reg); \
}

LOCAL_INLINE void
ipa0_write(uint32 ipal_val, uint32 ipau_val)
{
    /*
     * ldsr ipal_val, 6 ; ipa0l  6
     */
    __LDSR(6, ipal_val);
    /* 
     * ldsr ipau_val, 7 ; ipa0u  7
     */
    __LDSR(7, ipau_val);
}

LOCAL_INLINE void
ipa1_write(uint32 ipal_val, uint32 ipau_val)
{
    /*
     * ldsr ipal_val, 8 ; ipa1l  8
     */
    __LDSR(8, ipal_val);
    /*
     * ldsr ipau_val, 9 ; ipa1u  9
     */
    __LDSR(9, ipau_val);
}

LOCAL_INLINE void
ipa2_write(uint32 ipal_val, uint32 ipau_val)
{
    /*
     * ldsr ipal_val, 10 ; ipa2l  10
     */
    __LDSR(10, ipal_val);
    /* 
     * ldsr ipau_val, 11 ; ipa2u  11
     */
    __LDSR(11, ipau_val);
}

LOCAL_INLINE void
ipa3_write(uint32 ipal_val, uint32 ipau_val)
{
    /*
     * ldsr ipal_val, 12 ; ipa3l  12
     */
    __LDSR(12, ipal_val);
    /*
     * ldsr ipau_val, 13 ; ipa3u  13
     */
    __LDSR(13, ipau_val);
}

LOCAL_INLINE void
ipa4_write(uint32 ipal_val, uint32 ipau_val)
{
    /*
     * ldsr ipal_val, 14 ; ipa4l  14
     */
    __LDSR(14, ipal_val);
    /* 
     * ldsr ipau_val, 15 ; ipa4u  15
     */
    __LDSR(15, ipau_val);
}

LOCAL_INLINE void
dpa0_write(uint32 dpal_val, uint32 dpau_val)
{
    /*
     * ldsr dpal_val, 16 ; dpa0l  16
     */
    __LDSR(16, dpal_val);
    /* 
     * ldsr dpau_val, 17 ; dpa0u  17
     */
    __LDSR(17, dpau_val);
}

LOCAL_INLINE void
dpa1_write(uint32 dpal_val, uint32 dpau_val)
{
    /*
     * ldsr dpal_val, 18 ; dpa1l  18
     */
    __LDSR(18, dpal_val);
    /*
     * ldsr dpau_val, 19 ; dpa1u  19
     */
    __LDSR(19, dpau_val);
}

LOCAL_INLINE void
dpa2_write(uint32 dpal_val, uint32 dpau_val)
{
    /*
     * ldsr dpal_val, 20 ; dpa2l  20
     */
    __LDSR(20, dpal_val);
    /* 
     * ldsr dpau_val, 21 ; ipa2u  21
     */
    __LDSR(21, dpau_val);
}

LOCAL_INLINE void
dpa3_write(uint32 dpal_val, uint32 dpau_val)
{
    /*
     * ldsr dpal_val, 22 ; dpa3l  22
     */
    __LDSR(22, dpal_val);
    /*
     * ldsr dpau_val, 23 ; dpa3u  23
     */
    __LDSR(23, dpau_val);
}

LOCAL_INLINE void
dpa4_write(uint32 dpal_val, uint32 dpau_val)
{
    /*
     * ldsr dpal_val, 24 ; dpa4l  24
     */
    __LDSR(24, dpal_val);
    /* 
     * ldsr dpau_val, 25 ; dpa4u  25
     */
    __LDSR(25, dpau_val);
}

LOCAL_INLINE void
dpa5_write(uint32 dpal_val, uint32 dpau_val)
{
    /*
     * ldsr dpal_val, 26 ; dpa5l  26
     */
    __LDSR(26, dpal_val);
    /* 
     * ldsr dpau_val, 27 ; dpa5u  27
     */
    __LDSR(27, dpau_val);
}

#elif defined(__v850e3v5__)

LOCAL_INLINE uint32
current_psw(void)
{
	return(__STSR(5, 0));
}

LOCAL_INLINE void
set_psw(uint32 psw)
{
	__LDSR(5, 0, psw);
}

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

	__LDSR(11, 2, pmr);

	set_psw(psw);
	Asm("syncp");
}

LOCAL_INLINE uint16
get_ispr(void)
{
	return(__STSR(10, 2));
}

LOCAL_INLINE void
clear_ispr(void)
{
	uint32 psw;

	/* ISPR must be set in di sate(PSW.ID = 1) */
	psw = current_psw();
	disable_int();

	__LDSR(13, 2, 1);   /* INTCFG = 1; ISPR を書き換え可能に */
	__LDSR(10, 2, 0);   /* ISPR = 0 */
	__LDSR(13, 2, 0);   /* INTCFG = 0; ISPR を書き換え禁止に(自動更新に) */

	set_psw(psw);
	Asm("syncp");
}

LOCAL_INLINE uint32
current_cpuid(void)
{
	uint32 htcfg0_val;
	htcfg0_val = __STSR(0,2);
	return(((htcfg0_val >> 16) & 0x03));
}


#define LDSR_REG(reg, sel, val) {\
    __LDSR(reg, sel, val);\
}

#define STSR_REG(reg, sel, val) {\
    val = __STSR(reg, sel); \
}

#define MPA_WRITE(mpuid, mpla, mpua, mpat) { \
    __LDSR(mpuid*4, 6, mpla); \
    __LDSR(mpuid*4+1, 6, mpua); \
    __LDSR(mpuid*4+2, 6, mpat); \
}

LOCAL_INLINE void
set_intbp(uint32 intbp)
{
	uint32 psw;

	/* INTBP must be set in di sate(PSW.ID = 1) */
	psw = current_psw();
	disable_int();

	__LDSR(4, 1, intbp);

	set_psw(psw);
}

#else /* __v850e3v5__ */
#error please define ether __v850e2v3__ or __v850e3v5__
#endif /* __v850e2v3__ */

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_PRC_INSN_H */
