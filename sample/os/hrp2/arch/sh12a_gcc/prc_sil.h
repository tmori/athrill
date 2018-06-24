/*
 *	TOPPERS Software
 *		Toyohashi Open Platform for Embedded Real-Time Systems
 *	
 *	Copyright (C) 2007 by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	Copyright (C) 2007-2011 by Industrial Technology Institute,
 *								Miyagi Prefectural Government, JAPAN
 *	
 *	上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *	ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *	変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *	(1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *		権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *		スコード中に含まれていること．
 *	(2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *		用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *		者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *		の無保証規定を掲載すること．
 *	(3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *		用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *		と．
 *	  (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *		  作権表示，この利用条件および下記の無保証規定を掲載すること．
 *	  (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *		  報告すること．
 *	(4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *		害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *		また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *		由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *		免責すること．
 *	
 *	本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *	よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *	に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *	アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *	の責任を負わない．
 *	
 *	$Id: prc_sil.h 2145 2011-06-30 01:10:14Z mit-kimai $
 */

/*
 *	sil.hのプロセッサ依存部（SH12A用）
 */

#ifndef TOPPERS_PRC_SIL_H
#define TOPPERS_PRC_SIL_H

#ifndef TOPPERS_MACRO_ONLY

/*
 *	NMIを除くすべての割込みの禁止
 */
Inline uint32_t
TOPPERS_disint(void)
{
	uint32_t  TOPPERS_sr, TOPPERS_local_iipm;

	Asm("stc  sr,%0" : "=r"(TOPPERS_sr));
	/*
	 *	ここでTOPPERS_local_iipmを求めているのは、
	 *	SIL_UNL_INTの直後にSIL_LOC_INTを実行した場合に必要な
	 *	挿入するnop命令の数（LDC_NOP_SIL_SET_IIPM）が少なくて
	 *	済むようにするための時間稼ぎ
	 */
	TOPPERS_local_iipm = TOPPERS_sr & 0x000000f0U;
	Asm("ldc %0, sr" : : "r"(TOPPERS_sr | 0x000000f0U) : "memory", "t");
	return(TOPPERS_local_iipm);
}

/*
 *	割込み優先度マスク（内部表現）の現在値の設定
 */
Inline void
TOPPERS_set_iipm(uint32_t TOPPERS_iipm)
{
	uint32_t  TOPPERS_sr;

	Asm("stc  sr,%0" : "=r"(TOPPERS_sr));
	Asm("ldc %0, sr;"
#ifdef LDC_NOP_SIL_SET_IIPM
		LDC_NOP_SIL_SET_IIPM		/*  CPUからINTCに伝達されるまでの遅延  */
#endif /* LDC_NOP_SIL_SET_IIPM */
		: : "r"((TOPPERS_sr & ~0x000000f0U) | TOPPERS_iipm) : "memory", "t");
}

/*
 *	全割込みロック状態の制御
 */
#define SIL_PRE_LOC 	 uint32_t TOPPERS_iipm
#define SIL_LOC_INT()	 ((void)(TOPPERS_iipm = TOPPERS_disint()))
#define SIL_UNL_INT()	 (TOPPERS_set_iipm(TOPPERS_iipm))

/*
 *  エンディアンの反転
 */
Inline uint16_t toppers_sil_rev_endian_uint16(uint16_t src)
{
	uint16_t dst;
	Asm("swap.b %1, %0" : "=r"(dst) : "r"(src) );
	return dst;
}

#define	TOPPERS_SIL_REV_ENDIAN_UINT16(data) \
		toppers_sil_rev_endian_uint16(data)

Inline uint32_t toppers_sil_rev_endian_uint32(uint32_t src)
{
	uint32_t dst;
	Asm("swap.b %1, %0 \n"		/*  0x1234→0x1243  */
		"swap.w %0, %0 \n" 		/*  0x1243→0x4312  */
		"swap.b %0, %0"			/*  0x4312→0x4321  */
		: "=r"(dst) : "r"(src) );
	return dst;
}

#define	TOPPERS_SIL_REV_ENDIAN_UINT32(data) \
		toppers_sil_rev_endian_uint32(data)


/*
 *	レジスタに対する論理演算
 *	
 *	　インライン関数にしないで、マクロ関数にしているのは、
 *	　この時点でsil_rex_mem,sil_wrx_memが定義されていないため。
 */

/*
 *	8ビット・レジスタのOR演算
 *   void sil_orb_reg(uint8_t *p_reg, uint8_t bitptn);
 */
#define sil_orb_reg(p_reg, bitptn)				\
{												\
	uint8_t reg_val = sil_reb_mem(p_reg);		\
												\
	reg_val |= (uint8_t)(bitptn);				\
	sil_wrb_mem(p_reg, reg_val);				\
}

/*
 *	8ビット・レジスタのAND演算
 *   void sil_anb_reg(uint8_t *p_reg, uint8_t bitptn);
 */
#define sil_anb_reg(p_reg, bitptn)				\
{												\
	uint8_t reg_val = sil_reb_mem(p_reg);		\
												\
	reg_val &= (uint8_t)(bitptn);				\
	sil_wrb_mem(p_reg, reg_val);				\
}

/*
 *	16ビット・レジスタのOR演算
 *   void sil_orh_reg(uint16_t *p_reg, uint16_t bitptn);
 */
#define sil_orh_reg(p_reg, bitptn)				\
{												\
	uint16_t reg_val = sil_reh_mem(p_reg);		\
												\
	reg_val |= (uint16_t)(bitptn);				\
	sil_wrh_mem(p_reg, reg_val);				\
}


/*
 *	16ビット・レジスタのAND演算
 *   void sil_anh_reg(uint16_t *p_reg, uint16_t bitptn);
 */
#define sil_anh_reg(p_reg, bitptn)				\
{												\
	uint16_t reg_val = sil_reh_mem(p_reg);		\
												\
	reg_val &= (uint16_t)(bitptn);				\
	sil_wrh_mem(p_reg, reg_val);				\
}


/*
 *	16ビット・レジスタのビット・セット／クリア
 */
/*  void sil_seth_bit(uint16_t *p_reg, uint_t bit);  */
#define sil_seth_bit(p_reg, bit)				\
{												\
	uint16_t reg_val = sil_reh_reg(p_reg);		\
												\
	/*											\
	 *	bitが7以下の定数であれば、				\
	 *	bset命令に展開されることを期待			\
	 */											\
	reg_val |= (1U << bit);						\
	sil_wrh_reg(p_reg, reg_val);				\
}

/*  void sil_clrh_bit(uint16_t *p_reg, uint_t bit);  */
#define sil_clrh_bit(p_reg, bit)				\
{												\
	uint16_t reg_val = sil_reh_reg(p_reg);		\
												\
	/*											\
	 *	bitが7以下の定数であれば、				\
	 *	bclr命令に展開されることを期待			\
	 */											\
	reg_val &= ~(1U << (bit));					\
	sil_wrh_reg(p_reg, reg_val);				\
}

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_PRC_SIL_H */
