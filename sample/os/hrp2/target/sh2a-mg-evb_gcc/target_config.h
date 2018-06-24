/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2011-2012 by Embedded and Real-Time Systems Laboratory
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
 */

/*
 *	ターゲット依存モジュール（SH2A-MG_EVB用）
 *
 *	カーネルのターゲット依存部のインクルードファイル．kernel_impl.hのター
 *	ゲット依存部の位置付けとなる．
 */

#ifndef TOPPERS_TARGET_CONFIG_H
#define TOPPERS_TARGET_CONFIG_H

#include <sil.h>

/*
 *	ターゲットシステムのハードウェア資源の定義
 */
#include "sh72aw.h"

#include "target_mpu.h"

/*
 *	トレースログに関する設定
 */
#ifdef TOPPERS_ENABLE_TRACE
#include "logtrace/trace_config.h"

/*
 *	出力するログ種別の定義
 *	　必要に応じてasp/arch/logtrace/trace_dump.cのtrace_print()に
 *	　対応するログ種別を追記する。
 */
#define LOG_INH_ENTER			/*	 17(0x11)  */
#define LOG_INH_LEAVE			/*	145(0x91)  */
#define LOG_EXC_ENTER			/*	 22(0x16)  */
#define LOG_EXC_LEAVE			/*	150(0x96)  */

#endif /* TOPPERS_ENABLE_TRACE */

/*
 *  cfgのパス2で必要となるマクロ
 */
#define TARGET_INTATR (TA_POSEDGE | TA_NEGEDGE | TA_LOWLEVEL)
#define CHECK_STKSZ_ALIGN 4

/*
 *  cfgのパス3で必要となるマクロ
 */
#define CHECK_FUNC_ALIGN  2
#define CHECK_FUNC_NONNULL  1
#define CHECK_STACK_ALIGN  4
#define CHECK_STACK_NONNULL 1
#define CHECK_MPF_ALIGN 4
#define CHECK_MPF_NONNULL 1

/*
 *	ターゲット依存の文字出力に使用するポートの定義
 */
#define TARGET_PUTC_PORTID 1

/*
 *	HRPカーネル動作時のメモリマップと関連する定義
 */

/*
 *  RAM：0xFFF8 0000 - 0xFFF8 7FFF(32KB) <- リセット後
 *       0xFFF8 0000 - 0xFFF9 7FFF(96KB)
 */

/*
 *	デフォルトの非タスクコンテキスト用のスタック領域の定義
 */
/*  スタックサイズ  */
#define DEFAULT_ISTKSZ 0x800U /* 2048バイト */

/*	スタック領域の底 */
#define DEFAULT_ISTK_BOTTOM 0xfff88000 /* RAMの底 */

/*	スタック領域の先頭番地（スタック領域の底ではないので、注意） */
#define DEFAULT_ISTK	  (void *)(DEFAULT_ISTK_BOTTOM - DEFAULT_ISTKSZ)

#ifndef TOPPERS_MACRO_ONLY

#define DEFAULT_SSTKSZ 1280 

/*
 *	タスク初期化コンテキストブロック
 */

#define USE_TSKINICTXB	/*  TSKINICTXBを使用する  */

typedef struct task_initialization_context_block {
	SIZE	sstksz;			/* スタック領域のサイズ（丸めた値） */
	void	*sstk_bottom;	/* スタックポインタの初期値（スタックの底の初期値） */
	SIZE	stksz;			/* スタック領域のサイズ（丸めた値） */
	void	*stk_bottom;	/* スタックポインタの初期値（スタックの底の初期値） */
    void    *sustk; /* ユーザスタックの先頭番地 */
    void    *eustk; /* ユーザスタックの終了番地 */
} TSKINICTXB;

/*
 *  保護ドメイン初期化コンテキストブロック
 */

#define USE_DOMINICTXB	/*  TSKINICTXBを使用する  */

typedef struct {
    void *stext;
    void *etext;
    void *srodata;
    void *erodata;
    void *sdata;
    void *edata;
    void *ssrpw;
    void *esrpw;
    uint32_t *valid_map;
} DOMINICTXB;

/*
 *  ユーザタスクが特権モード実行中か？
 *  priv_modeは、サービスコール、および、拡張サービスコールの
 *  入り口処理でtrueにセットされる
 */
#define t_sense_priv(p_tcb) ((p_tcb)->tskctxb.priv_mode == true)
#define i_sense_priv(p_tcb) ((p_tcb)->tskctxb.priv_mode == true)
#define t_sense_priv_runtsk() ((p_runtsk)->tskctxb.priv_mode == true)
#define i_sense_priv_runtsk() ((p_runtsk)->tskctxb.priv_mode == true)

/*
 *  ユーザタスクのタスク例外処理呼び出し時に使用するスタックサイズが足りないか？
 *  i_check_tex_runtskは、非タスクコンテキストから呼ばれるので、
 *  ユーザスタックはTCBに保存されているはずである
 */
extern bool_t i_check_tex_runtsk();

/*
 *	ターゲットシステム依存の初期化
 */
extern void target_initialize(void);
extern void target_fput_initialize(void);
extern void target_fput_log(char c);

/*
 *	ターゲットシステムの終了
 *
 *	システムを終了する時に使う．
 */
extern void target_exit(void) NoReturn;

/*
 * CPU例外番号に関する定義
 */
#define TMIN_EXCNO		UINT_C(4)		/*	GENERAL_ILLEGAL_INSTRUCTION_VECTOR	*/
#define TMAX_EXCNO		UINT_C(63)		/*	TRAPA_INST_VECTOR  */
#define TNUM_EXC		(TMAX_EXCNO - TMIN_EXCNO + 1U)

/*
 *	例外ベクタ
 */
#define POWER_ON_RESET_VECTOR				0
#define MANUAL_RESET_VECTOR					2
#define GENERAL_ILLEGAL_INSTRUCTION_VECTOR	4
#define RAM_ERROR_VECTOR					5
#define SLOT_ILLEGAL_INSTRUCTION_VECTOR		6
#define CPU_ADDRESS_ERROR_VECTOR			9
#define DMA_ADDRESS_ERROR_VECTOR			10
#define FPU_VECTOR							13
#define BANK_OVER_FLOW_VECTOR				15
#define BANK_UNDER_FLOW_VECTOR				16
#define DIVIDE_BY_ZERO_VECTOR				17


/*
 * 割込みハンドラ番号に関する定義
 */
#define TMIN_INHNO		UINT_C(64)
#define TMAX_INHNO		UINT_C(493)
#define TNUM_INH		(TMAX_INHNO - TMIN_INHNO + 1U)

/*
 * 割込み番号に関する定義
 */
#define TMIN_INTNO		UINT_C(64)
#define TMAX_INTNO		UINT_C(493)
#define TNUM_INT		(TMAX_INTNO - TMIN_INTNO + 1U)

/*	IRQ  */
#define TMIN_INTNO_IRQ 	64U
#define TMAX_INTNO_IRQ	71U

/*
 *  割込み関連のレジスタ
 */
#define INTC_BCR 0xfffd940e /* バンク制御レジスタ */
#define INTPRI2BCR(intpri) (0x0001 << (uint16_t)intpri) /* 割込み優先度からBCRに設定する値を得るマクロ */
#define INTC_BNR 0xfffd9410 /* バンク番号レジスタ */
#define INTC_BNR_BE_DISABLE_ALL  0x0000 /* すべての割込みでバンクを禁止 */
#define INTC_BNR_BE_DISABLE_PART 0x4000 /* メインクロック発進停止、ユーザブレークのみ許可 */
#define INTC_BNR_BE_BCR 0xC000 /* BCRの設定に従う */

/*
 *  周辺機能割込み番号のベース
 *  peripheral block
 */
#define INTC_PB_INTNO_BASE 102

/*
 *  周辺機能割込み要求レジスタのベースアドレス
 *  1byteずつ増える
 */
#define INTC_PB_IR_BASE 0xfffd9800

/*
 *  周辺機能割込み制御レジスタのベースアドレス
 *  2byteずつ増える
 */
#define INTC_PB_ICR_BASE 0xfffd999a
#define INTC_PB_ICR_INTEN 0x8000    /* 割込み許可（ビットが0ならば禁止） */
#define INTC_PB_ICR_IPR_MASK 0x000f /* 割込み優先度のマスク */

/*	CFG_INTで割込み優先度が設定されていないことを示すビット  */
#ifndef NOT_CFG_INT_BIT
#define NOT_CFG_INT_BIT		0x80U
#endif /* NOT_CFG_INT_BIT */

extern const uint8_t int_iipm_tbl[TNUM_INT];


/*
 *  割込み要求のチェック
 */
Inline bool_t 
x_probe_int(INTNO intno)
{
    return ((sil_reb_mem((void *)(INTC_PB_IR_BASE + (intno - INTC_PB_INTNO_BASE))) & 0x01) == 0x01);
}

#define t_probe_int(intno) x_probe_int(intno)
#define i_probe_int(intno) x_probe_int(intno)

/*
 *  割込み要求のクリア
 */
Inline void
x_clear_int(INTNO intno)
{
    sil_wrb_mem((void *)(INTC_PB_IR_BASE + (intno - INTC_PB_INTNO_BASE)), 0x00);
}

#define t_clear_int(intno) x_clear_int(intno)
#define i_clear_int(intno) x_clear_int(intno)

/*
 *  割込み禁止
 */
Inline bool_t
x_disable_int(INTNO intno)
{
    uint8_t iipm = int_iipm_tbl[intno - TMIN_INTNO];

    if((iipm & NOT_CFG_INT_BIT) != 0){
        return false;
    }

    sil_wrh_mem((void *)(INTC_PB_ICR_BASE + (intno - INTC_PB_INTNO_BASE) * 2), 
            (sil_reh_mem((void *)(INTC_PB_ICR_BASE + (intno - INTC_PB_INTNO_BASE) * 2)) & ~INTC_PB_ICR_INTEN));

    return true;
}

#define t_disable_int(intno) x_disable_int(intno)
#define i_disable_int(intno) x_disable_int(intno)

/*
 *  割込み許可
 */
Inline bool_t
x_enable_int(INTNO intno)
{
    uint32_t iipm = int_iipm_tbl[intno - TMIN_INTNO];

    if((iipm & NOT_CFG_INT_BIT) != 0){
        return false;
    }

    sil_wrh_mem((void *)(INTC_PB_ICR_BASE + (intno - INTC_PB_INTNO_BASE) * 2), 
            (sil_reh_mem((void *)(INTC_PB_ICR_BASE + (intno - INTC_PB_INTNO_BASE) * 2)) | INTC_PB_ICR_INTEN));

    return true;
}

#define t_enable_int(intno) x_enable_int(intno)
#define i_enable_int(intno) x_enable_int(intno)


#endif /* TOPPERS_MACRO_ONLY */

#define USRTEX_STKSZ (4*2)

/*
 *	微少時間待ちのための定義（本来はSILのターゲット依存部）
 */
#define SIL_DLY_TIM1    30 /* 160MHzで，約5命令 */
#define SIL_DLY_TIM2    24 /* 160MHzで，約4命令 */

/*
 *  割込み許可が有効になるまでの時間待ちをするためのnop命令
 *  
 *  　SH7211は一般的なキャッシュを内蔵していない代わりに、
 *  　内蔵フラッシュROMに対してのみ有効なROMキャッシュを
 *  　内蔵している。このROMキャッシュは常にONで、OFFにはできない。
 *  　そのため、ENABLE_CACHEマクロではなく、ROM_BOOTマクロを
 *  　用いて、遅延に必要な命令数を判別している。
 */
#ifdef ROM_BOOT		/*  ROMキャッシュの影響を受ける場合  */
	/*
	 *  ROMキャッシュの効果を考慮
	 *  　　CPUからINTCに伝達されるまでの遅延
	 *  　　（スーパスカラで3Iφcyc分＝6命令） 
	 */

	/*  ディスパッチャ用  */
#define LDC_NOP_DISPATCHER		nop; nop; nop; nop; nop; nop

	/*  set_sr用：このターゲットでは必要なし  */
// #define LDC_NOP_SET_SR			"nop; nop"

	/*  TOPPERS_set_iipm用はtarget_sil.hに記述する。  */

#endif	/*  ROM_BOOT  */

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

#ifndef TOPPERS_TARGET_SUPPORT_OVRHDR
#define TOPPERS_TARGET_SUPPORT_OVRHDR
#endif /* TOPPERS_TARGET_SUPPORT_OVRHDR */

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

#define TEXCNO_EMULATE_TEXRTN 20
#define TEXCNO_EMULATE_RET_TEX 21

#ifndef TOPPERS_MACRO_ONLY

#include <sil.h>

/*
 * IRCの初期化
 */
Inline void
irc_initialize(void)
{
    /*
     *  すべての割込みでバンクを禁止
     */
    sil_wrh_mem((void *)INTC_BNR, INTC_BNR_BE_DISABLE_ALL);
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
}

extern void target_mpu_initialize(void);
extern void target_mpu_handler(void);

#endif /* TOPPERS_MACRO_ONLY */

#include "sh12a_gcc/prc_config.h"

#endif	/* TOPPERS_TARGET_CONFIG_H */
