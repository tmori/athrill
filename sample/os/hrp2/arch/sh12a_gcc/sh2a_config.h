/*
 *	TOPPERS/HRP Kernel
 *		Toyohashi Open Platform for Embedded Real-Time Systems/
 *		High Reliable system Profile Kernel
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
 *	$Id: sh2a_config.h 2157 2011-07-19 06:25:30Z mit-kimai $
 */

/*
 *		プロセッサ依存モジュール（SH2A用）
 *
 *	このインクルードファイルは，sh7xxx_config.h（または，そこからインク
 *	ルードされるファイル）のみからインクルードされる．他のファイルから
 *	直接インクルードしてはならない．
 */

#ifndef TOPPERS_SH2A_CONFIG_H
#define TOPPERS_SH2A_CONFIG_H

/*  sh7xxx_config.hはtarget_config.hからインクルードされる  */
#ifndef TOPPERS_TARGET_CONFIG_H
#error  sh7xxx_config.h(target_config.h) is not included!
#endif	/*  TOPPERS_TARGET_CONFIG_H  */

/*
 *	コアの種類
 */
#define SH2A

/*
 *	使用できる命令セット
 *	　SH1用命令については、すべてのコアでサポートされるので、
 *	　定義しない。
 */
#define TOPPERS_SUPPORT_SH2_INSTRUCTION
#define TOPPERS_SUPPORT_SH2E_INSTRUCTION
#define TOPPERS_SUPPORT_SH2A_INSTRUCTION

/*
 *	NMIの割込み番号
 */
#define TINTNO_NMI					NMI_VECTOR

/*
 *	IRQ割込みかどうかを判定するためのマクロ
 *	　prc_config.hで定義すべきだが、このファイル中のx_config_intatr()
 *	　で使用するため、ここに記述している。
 */
#define INTNO_IS_IRQ(intno) 								\
		((TMIN_INTNO_IRQ <= (intno)) && ((intno) <= TMAX_INTNO_IRQ))

/*
 *	割込み番号からIRQ番号への変換
 */
#define INTNO_TO_IRQNO(intno) (((intno) - TMIN_INTNO_IRQ))


#ifndef TOPPERS_MACRO_ONLY

#include <sil.h>

/*
 * IRCの初期化
 */
Inline void
irc_initialize(void)
{
	sil_wrh_mem((uint16_t *)ICR0_h, 0x0000U);
	sil_wrh_mem((uint16_t *)ICR1_h, 0x0000U);
	
#ifdef USE_BANKED_REG
	/*
	 * ・NMI,UBC以外のすべての割込みでレジスタバンクの使用を許可
	 * 　　・BEビット=01：IBCRの設定は無視
	 * ・レジスタバンク・オーバーフロー例外を許可
	 */
	sil_wrh_mem((uint16_t *)IBNR_h, (IBNR_BE0 | IBNR_BOVE));
#else
	/* レジスタバンク使用禁止 */
	sil_wrh_mem((uint16_t *)IBNR_h, 0x0000U);
#endif /* USE_BANKED_REG */

	init_ipr();	/*	割込み優先レベル設定レジスタの初期化  */
}


/*
 *	割込みの属性設定
 */
Inline void
x_config_intatr(INTNO intno, ATR intatr)
{
	/*
	 *	IRQのみサポートする
	 */
	if(INTNO_IS_IRQ(intno)){
		uint32_t icr1_val = sil_reh_mem((uint16_t *)ICR1_h);
		uint32_t offset   = INTNO_TO_IRQNO(intno) * 2U;

		/*
		 *	エッジトリガの指定がなければ、デフォルトでローレベルトリガ
		 *	とする。
		 *	該当ビットをクリアしたままであれば、ローレベルトリガとなる。
		 */
		icr1_val &=  ~(0x03U << offset);
		
		if((intatr & TA_POSEDGE) != 0U) {
			/*
			 *	ポジティブエッジ
			 */
			icr1_val |= IRQ_POSEDGE << offset;
		}

		/*
		 *	エッジトリガで、極性の指定がなければ、ネガティブエッジ
		 *	とする。
		 */
		if(((intatr & TA_NEGEDGE) != 0U) ||
		   (((intatr & TA_EDGE) != 0U) && ((intatr & TA_BOTHEDGE) == 0U))) {
			/*
			 *	ネガティブエッジ
			 */
			icr1_val |= IRQ_NEGEDGE << offset;
		}
		
		sil_wrh_mem((uint16_t *)ICR1_h, icr1_val);
	}
}

/*
 *	割込みハンドラ入口で必要なIRC操作
 *
 *	IRQ割込みでかつエッジトリガの場合は割込み要求をクリアする．
 *	エッジトリガかレベルトリガかを判別するとオーバーヘッドが大きいので，
 *	常にクリアする．
 *	（レベルトリガでクリア処理をしても害はない。）
 *
 *	IRQRRレジスタを一度、ダミーリードして、 該当ビットに0を書き込む。
 *	この間、全割込みロック状態にしているのは、以下のようなケースを防ぐ
 *	ため。
 *
 *	3種類の割込みを考える。
 *	割込み優先度は、l > m > nの順とする。
 *	（割込みlの優先度が高い。）
 *	
 *	1.	mの割込みが発生(mビットがセット)
 *	2.	mの割込みハンドラが実行(nは割込み禁止)
 *	3.	IRQRRリード(l = 0, m = 1, n = 0)
 *	4.	lの割込みが発生(lビットがセット)
 *	5.	lの割込みハンドラが実行(nは割込み禁止)
 *	6.	nの割込みが発生（保留）(nビットがセット)
 *	7.	IRQRRリード(l = 1, m = 1, n = 1)
 *	8.	IRQRRライト(l = 0, m = 1, n = 1)
 *	9.	lの割込みハンドラ終了
 *	10. mの割込みハンドラ再開
 *	11. IRQQライト(l = 0, m = 0, n = 0)
 *	
 *	11の段階で n がクリアされる可能性がある．
 */
Inline void
i_begin_int(INTNO intno)
{
	SIL_PRE_LOC;
	uint32_t irqpr, bitptn;
	if (INTNO_IS_IRQ(intno)) {
		/*
		 * 一度、ダミーリードして、 該当ビットに0を書き込む。
		 *	　変数bitptnを用いているのは、全割込みロックの区間を
		 *	　短くするため
		 */
		bitptn = ~(0x01U << INTNO_TO_IRQNO(intno));
		SIL_LOC_INT();
		irqpr = sil_reh_mem((uint16_t *)IRQRR_h);
		irqpr &= bitptn;
		sil_wrh_mem((uint16_t *)IRQRR_h, irqpr);
		SIL_UNL_INT();
	}
}

#endif /* TOPPERS_MACRO_ONLY */

/*
 *	プロセッサ依存モジュール（SH12A用）
 */
#include "sh12a_gcc/prc_config.h"

#endif /* TOPPERS_SH2A_CONFIG_H */
