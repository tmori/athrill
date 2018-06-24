/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *								Toyohashi Univ. of Technology, JAPAN
 *	Copyright (C) 2005-2007 by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	Copyright (C) 2001-2009 by Industrial Technology Institute,
 *								Miyagi Prefectural Government, JAPAN
 *	Copyright (C) 2011-2012	by Embedded and Real-Time Systems Laboratory
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
 *	$Id: prc_config.c 2156 2011-07-19 06:03:20Z mit-kimai $
 */

/*
 *		プロセッサ依存モジュール（SH12A用）
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"

/*
 * コンテキスト参照のための変数
 */
uint32_t excnest_count; /* 例外（割込み/CPU例外）のネスト回数のカウント */

/*
 *	CPUロックフラグ実現のための変数
 */
bool_t		lock_flag;		/* CPUロックフラグの値を保持する変数 */
uint32_t	saved_iipm;		/* 割込み優先度マスクを保存する変数 */

/*
 *	各割込み番号毎のIPRの情報管理テーブル
 *	内容の定義は，プロセッサ型番毎に異なるのため，
 *	プロセッサ型番毎の定義ファイルで定義する
 */

/*
 *	カーネルのベクタ
 */
extern const FP vectors;

/*
 *	プロセッサ依存の初期化
 */
void
prc_initialize(void)
{
   /*
	*  カーネル起動時は非タスクコンテキストとして動作させるため、
	*  1にしておく。
	*/
	excnest_count = 1U;
	
	/*
	 *	CPUロックフラグ実現のための変数の初期化
	 */
	lock_flag = true;
	saved_iipm = IIPM_ENAALL;
	
	/*
	 *	割込みコントローラの初期化
	 *	プロセッサ毎に定義する
	 */
	irc_initialize();

	/*
	 *	ベクターベースレジスタの初期化
	 */
	set_vbr(&vectors);
}

/*
 *	プロセッサ依存の終了処理
 */
void
prc_exit(void)
{
	extern void    software_term_hook(void);
	void (*volatile fp)(void) = software_term_hook;

	/*
	 *	software_term_hookへのポインタを，一旦volatile指定のあるfpに代
	 *	入してから使うのは，0との比較が最適化で削除されないようにするた
	 *	めである．
	 */
	if (fp != 0) {
		(*fp)();
	}
}

#ifndef OMIT_DEFAULT_EXC_HANDLER
/*
 *	登録されていない例外が発生すると呼び出される
 */
void
default_exc_handler(void *p_excinf)
{
	uint32_t *sp	= (uint32_t*)p_excinf;
	uint32_t vector = *(sp + P_EXCINF_OFFSET_VECTOR);
	uint32_t pc 	= *(sp + P_EXCINF_OFFSET_PC);
	uint32_t sr 	= *(sp + P_EXCINF_OFFSET_SR);
	uint32_t pr 	= *(sp + P_EXCINF_OFFSET_PR);
	uint32_t *sp_val = sp + P_EXCINF_OFFSET_SP;
	uint32_t reg, i;
	char 	 *msg;
	
	switch(vector) {
		case GENERAL_ILLEGAL_INSTRUCTION_VECTOR:
			msg = "General illegal instruction exception occurs.";
			break;
		case RAM_ERROR_VECTOR:
			msg = "RAM error exception occurs.";
			break;
		case SLOT_ILLEGAL_INSTRUCTION_VECTOR:
			msg = "Slot illegal instruction exception occurs.";
			break;
		case CPU_ADDRESS_ERROR_VECTOR:
			msg = "CPU address error exception occurs.";
			break;
		case DMA_ADDRESS_ERROR_VECTOR:
			msg = "DMA address error exception occurs.";
			break;

#ifdef FPU_VECTOR
		case FPU_VECTOR:
			msg = "FPU exception occurs.";
			break;
#endif /*  FPU_VECTOR  */

#ifdef SH2A
		case BANK_OVER_FLOW_VECTOR:
			msg = "Bank over flow exception occurs.";
			break;
		case BANK_UNDER_FLOW_VECTOR:
			msg = "Bank under flow exception occurs.";
			break;
#endif /*  SH2A  */

		default:
   			msg = "Unregistered exception occurs.";
   			break;
   	}
	syslog_1(LOG_EMERG, "%s", msg);

	syslog_3(LOG_EMERG, "VectorNo = %d PC = 0x%08x SP = 0x%08p",
		   vector, pc, sp_val);
	syslog_2(LOG_EMERG, "SR = 0x%08x PR=0x%08x", sr, pr);
	for(i = 0; i < 8; ++i) {
		reg = *(sp + P_EXCINF_OFFSET_R0 + i);
		syslog_2(LOG_EMERG, " r%d=0x%08x", i, reg);
	}
	target_exit();
}
#endif /* OMIT_DEFAULT_EXC_HANDLER */

#ifndef OMIT_DEFAULT_INT_HANDLER
/*
 *	未登録の割込みが発生した場合に呼び出される
 */
void
default_int_handler(INTNO intno)
{
	syslog_0(LOG_EMERG, "Unregistered Interrupt occurs.");
	syslog_1(LOG_EMERG, "INTNO = %d",intno);
	target_exit();
}
#endif /* OMIT_DEFAULT_INT_HANDLER */
