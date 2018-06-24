/*
 *	TOPPERS Software
 *		Toyohashi Open Platform for Embedded Real-Time Systems
 *	
 *	Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *								Toyohashi Univ. of Technology, JAPAN
 *	Copyright (C) 2007 by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
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
 *	$Id: prc_insn.h 2158 2011-07-22 05:30:47Z mit-kimai $
 */

/*
 *	プロセッサの特殊命令のインライン関数定義（SH12A用）
 */

#ifndef TOPPERS_PRC_INSN_H
#define TOPPERS_PRC_INSN_H

/*
 *	ステータスレジスタ（SR）の現在値の読出し
 */
Inline uint32_t
current_sr(void)
{
	uint32_t sr;
	Asm("stc  sr,%0" : "=r"(sr));
	return(sr);
}

/*
 *	ステータスレジスタ（SR）の現在値の変更
 */
Inline void
set_sr(uint32_t sr)
{
	Asm("ldc %0, sr" : : "r"(sr) : "t");
}

#ifdef LDC_NOP_SET_SR
Inline void
set_sr_with_nop(uint32_t sr)
{
	Asm("ldc %0, sr;"
		LDC_NOP_SET_SR			/*  CPUからINTCに伝達されるまでの遅延  */
		 : : "r"(sr) : "t");
}
#else	/*  LDC_NOP_SET_SR  */
#define set_sr_with_nop(sr)		set_sr(sr)
#endif	/*  LDC_NOP_SET_SR  */

/*
 *	ベクタベースレジスタ（VBR）の現在値の読出し
 */
Inline void *
current_vbr(void)
{
	void *vbr;
	Asm("stc vbr,%0" : "=r"(vbr));
	return(vbr);
}

/*
 *	ベクタベースレジスタ（VBR）の設定
 */
Inline void
set_vbr(const FP *vbr)
{
	Asm("ldc  %0, vbr" : : "r"(vbr) );
}

#endif /* TOPPERS_PRC_INSN_H */
