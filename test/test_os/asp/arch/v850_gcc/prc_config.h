/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel  
 * 
 *  Copyright (C) 2000-2002 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 * 
 *  Copyright (C) 2005 by Freelines CO.,Ltd
 *
 *  Copyright (C) 2010-2011 by Meika Sugimoto
 * 
 *  上記著作権者は，以下の (1)〜(4) の条件か，Free Software Foundation 
 *  によって公表されている GNU General Public License の Version 2 に記
 *  述されている条件を満たす場合に限り，本ソフトウェア（本ソフトウェア
 *  を改変したものを含む．以下同じ）を使用・複製・改変・再配布（以下，
 *  利用と呼ぶ）することを無償で許諾する．
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
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，その適用可能性も
 *  含めて，いかなる保証も行わない．また，本ソフトウェアの利用により直
 *  接的または間接的に生じたいかなる損害に関しても，その責任を負わない．
 * 
 */

/*
 *	プロセッサ依存モジュール（V850用）
 *
 *  このインクルードファイルは，target_config.h（または，そこからインク
 *  ルードされるファイル）のみからインクルードされる．他のファイルから
 *  直接インクルードしてはならない．
 */

#ifndef TOPPERS_PRC_CONFIG_H
#define TOPPERS_PRC_CONFIG_H

#ifndef TOPPERS_MACRO_ONLY

/*
 *  プロセッサの特殊命令のインライン関数定義
 */
#include "prc_insn.h"
#include <sil.h>

/*
 *  非タスクコンテキスト用のスタック初期値
 */
#define TOPPERS_ISTKPT(istk, istksz) ((STK_T *)((char *)(istk) + (istksz)))

/*
 *  タスクコンテキストブロックの定義
 */
typedef struct task_context_block {
	void	*sp;		/* スタックポインタ */
	FP		pc;			/* プログラムカウンタ */
} TSKCTXB;

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  割込み優先度マスク操作ライブラリ
 *
 *	V850は表面上は割込み優先度マスクを持たないため，ソフトウェアで
 *	エミュレーションする．具体的には，CFG_INTで定義された割込み要求ライン設定
 *	から各割込み優先度マスクの値にに対応するISPRレジスタ値を生成し，
 *	設定することで行う．
 */


/*
 *  割込み優先度マスクの外部表現と内部表現の変換
 *
 *  アセンブリ言語のソースファイルからインクルードする場合のために，CASTを使用
 */
#define EXT_IPM(iipm)	 (-CAST(PRI,(iipm)))	   /* 内部表現を外部表現に */
#define INT_IPM(ipm)	 (CAST(uint32_t, -(ipm)))  /* 外部表現を内部表現に */

#ifndef TOPPERS_MACRO_ONLY

/* (モデル上の)現在の割込み優先度 */
extern uint8_t current_intpri;
/* CPUロックフラグ */
extern bool_t lock_flag;
/* CPUロック中にCPUロック前のIMRレジスタ値を格納する領域 */
extern uint16_t saved_imr[];
/* 禁止されている割込み要因を格納する領域 */
extern uint16_t disint_table[];
/* 割込みネスト回数 */
extern uint8_t intnest;

/*
 *  コンテキストの参照
 */
Inline bool_t
sense_context(void)
{
    return (intnest > 0)? true : false;
}

/*
 *	現在の割込み優先度マスク(内部表現)の取得
 */
Inline uint8_t
get_intpri(void)
{
	return current_intpri;
}

/*
 *	現在の割込み優先度マスク(内部表現)の設定
 */

extern void set_intpri(uint8_t intpri);

/*
 *	IMRレジスタ値の退避
 *
 *  restore_imrと対に呼ばなければならない．
 */
Inline void save_imr(void)
{
	uint_least8_t i;
	
	/* ISPRから読み出し */
	for(i = 0 ; i < IMR_SIZE ; i++)
	{
		saved_imr[i] = sil_reh_mem((void *)(IMR0 + (2 * i)));
	}
}

/*
 *  IMRレジスタ値の復帰
 *
 *  save_imrと対に呼ばなければならない．
 */
Inline void restore_imr(void)
{
	uint_least8_t i;
	
	/* ISPRに書き込み */
	for(i = 0 ; i < IMR_SIZE ; i++)
	{
		sil_wrh_mem((void *)(IMR0 + (2 * i)) , saved_imr[i]);
	}
}

/*
 *  TOPPERS標準割込み処理モデルの実現
 */


#endif /* TOPPERS_MACRO_ONLY */

/*
 *  CPUロック状態での割込み優先度マスク
 *
 *  TIPM_LOCKは，CPUロック状態での割込み優先度マスク，すなわち，カーネ
 *  ル管理外のものを除くすべての割込みをマスクする値に定義する．
 */
#define TIPM_LOCK		TMIN_INTPRI

/*
 *  CPUロック状態での割込み優先度マスクの内部表現
 */
#define INTPRI_LOCK		INT_IPM(TIPM_LOCK)

/*
 *  TIPM_ENAALL（割込み優先度マスク全解除）の内部表現
 */
#define INTPRI_ENAALL	INT_IPM(TIPM_ENAALL)

#ifndef TOPPERS_MACRO_ONLY


/*
 *  CPUロック状態への移行
 */
Inline void
x_lock_cpu(void)
{
	/* 途中で割込みが入ってはならないため，割込みを禁止する． */
	disable_int();
	
	save_imr();	/* 現在のIMRを退避 */
	set_intpri(INTPRI_LOCK);
	lock_flag = true;
	
	/* 割込み解除 */
	enable_int();
}

#define t_lock_cpu()    x_lock_cpu()
#define i_lock_cpu()    x_lock_cpu()

/*
 *  CPUロック状態の解除
 */
Inline void
x_unlock_cpu(void)
{
	/* 途中で割込みが入ってはならないため，割込みを禁止する． */
	disable_int();
	
	restore_imr();	/* IMRを復帰 */
	set_intpri(current_intpri);
	lock_flag = false;
	
	/* 割込み解除 */
	enable_int();
}

#define t_unlock_cpu()    x_unlock_cpu()
#define i_unlock_cpu()    x_unlock_cpu()

/*
 *  CPUロック状態の参照
 */
Inline bool_t
x_sense_lock(void)
{
    return lock_flag;
}

#define t_sense_lock()    x_sense_lock()
#define i_sense_lock()    x_sense_lock()

/* ena_int/disintで有効な割込み番号の範囲の判定 */
#define VALID_INTNO_CFGINT(intno)	(((7u <= (intno)) && ((intno) <= 116u))	\
										|| (intno == 1u))
#define VALID_INTNO_DISINT(intno)	VALID_INTNO_CFGINT((intno))

/* cre_intで有効な割込み番号の指定  */
#define VALID_INTNO_CREINT          VALID_INTNO_CFGINT((intno))

/*
 *  chg_ipmで有効な割込み優先度の範囲の判定
 */
#define VALID_INTPRI_CHGIPM(intpri) ((intpri) < 8)
/*
 * （モデル上の）割込み優先度マスクの設定
 */

Inline void
x_set_ipm(PRI intpri)
{
	current_intpri = INT_IPM(intpri);
	set_intpri(current_intpri);
}

#define t_set_ipm(intpri)    x_set_ipm(intpri)
#define i_set_ipm(intpri)    x_set_ipm(intpri)

/*
 * （モデル上の）割込み優先度マスクの参照
 */
Inline PRI
x_get_ipm(void)
{
	return EXT_IPM(current_intpri);
}

#define t_get_ipm()    x_get_ipm()
#define i_get_ipm()    x_get_ipm()

/*
 *  最高優先順位タスクへのディスパッチ（prc_support.S）
 *
 *  dispatchは，タスクコンテキストから呼び出されたサービスコール処理か
 *  ら呼び出すべきもので，タスクコンテキスト・CPUロック状態・ディスパッ
 *  チ許可状態・（モデル上の）割込み優先度マスク全解除状態で呼び出さな
 *  ければならない．
 */
extern void dispatch(void);

/*
 *  ディスパッチャの動作開始（prc_support.S）
 *
 *  start_dispatchは，カーネル起動時に呼び出すべきもので，すべての割込
 *  みを禁止した状態（全割込みロック状態と同等の状態）で呼び出さなければ
 *  ならない．
 */
extern void start_dispatch(void);

/*
 *  現在のコンテキストを捨ててディスパッチ（prc_support.S）
 *
 *  exit_and_dispatchは，ext_tskから呼び出すべきもので，タスクコンテキ
 *  スト・CPUロック状態・ディスパッチ許可状態・（モデル上の）割込み優先
 *  度マスク全解除状態で呼び出さなければならない．
 */
extern void exit_and_dispatch(void);

/*
 *  カーネルの終了処理の呼出し（prc_support.S）
 *
 *  call_exit_kernelは，カーネルの終了時に呼び出すべきもので，非タスク
 *  コンテキストに切り換えて，カーネルの終了処理（exit_kernel）を呼び出
 *  す．
 */
extern void call_exit_kernel(void) NoReturn;

/*
 *  タスクコンテキストの初期化
 *
 *  タスクが休止状態から実行できる状態に移行する時に呼ばれる．この時点
 *  でスタック領域を使ってはならない．
 *
 *  activate_contextを，インライン関数ではなくマクロ定義としているのは，
 *  この時点ではTCBが定義されていないためである．
 */
extern void    start_r(void);

#define activate_context(p_tcb)												\
{																			\
	(p_tcb)->tskctxb.sp = (uint32_t *)(((uint32_t)(p_tcb)->p_tinib->stk) + 	\
								(p_tcb)->p_tinib->stksz);					\
	(p_tcb)->tskctxb.pc = (void *) start_r;									\
}

/*
 * ターゲット非依存部に含まれる標準の例外管理機能の初期化処理を用いない
 */
#define OMIT_INITIALIZE_EXCEPTION

/*
 * CPU例外ハンドラの初期化
 * 　空マクロにしたいが、asp/kernel/exception.hでプロトタイプ宣言
 * 　されているため、関数として定義しなければならない。
 */
extern void initialize_exception(void);

/*
 *  割込み番号・割込みハンドラ番号
 *
 *  割込みハンドラ番号(inhno)と割込み番号(intno)は，ベクタ番号を用いる．
 *
 *  ベクタ番号は64から始まるため，そのままの値を優先度管理等のテーブルの
 *  インデックスに用いると，無駄な領域が発生する．そのため，カーネル内部
 *  では，-64した値を用いる．
 *
 *  内部表現の名前は，iintno,iinhnoとする．
 */

/*
 * 割込みハンドラ番号の内部・外部表現相互変換
 */
/* 内部表現を外部表現に */
#define EXT_INHNO(iintno) (iintno)
/* 外部表現を内部表現に */
#define INT_INHNO(intno)  (intno)

/*
 * 割込み番号の内部・外部表現相互変換
 */
/* 内部表現を外部表現に */
#define EXT_INTNO(iintno)   (iintno)
/* 外部表現を内部表現に */
#define INT_INTNO(intno)    (intno)

/*
 *  CPU例外ハンドラ番号
 *
 */
#define VALID_EXCNO_DEFEXC(excno) (1)

/*
 *  割込みハンドラの設定
 *
 *	TFファイルで生成するため，空にしている
 */

#define x_define_inh(inhno, int_entry)

/*
 *  割込みハンドラの出入口処理の生成マクロ
 *
 */
#define _INT_ENTRY(inhno, inthdr)    _kernel_##inthdr##_##inhno
#define INT_ENTRY(inhno, inthdr)     _INT_ENTRY(inhno, inthdr)

#define _INTHDR_ENTRY(inhno, inthdr) extern void _kernel_##inthdr##_##inhno(void);
#define INTHDR_ENTRY(inhno, inhno_num, inthdr)  _INTHDR_ENTRY(inhno, inthdr) 

/*
 *  割込み要求禁止フラグのセット
 *
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをクリアしようとした場合には，falseを返す．
 */
Inline bool_t
x_disable_int(INTNO intno)
{
	uint32_t intreg_addr = INTREG_ADDRESS(intno);
	
	if(!VALID_INTNO_DISINT(intno))
	{
		return false;
	}
	
	/* 6bit目をセット */
	sil_wrb_mem((void *)intreg_addr , 
		sil_reb_mem((void *)intreg_addr) | (0x01U << 6));
	/* 割込み禁止状態ビットをセット */
	disint_table[(intno / 16u)] |= (1u << (intno % 16u));
	
	return(true);
}

#define t_disable_int(intno) x_disable_int(intno)
#define i_disable_int(intno) x_disable_int(intno)

/*
 *  割込み要求禁止フラグの解除
 *
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをクリアしようとした場合には，falseを返す．
 */

Inline bool_t
x_enable_int(INTNO intno)
{
	uint32_t intreg_addr = INTREG_ADDRESS(intno);
	
	if(!VALID_INTNO_DISINT(intno))
	{
		return false;
	}
	
	/* 6bit目をクリア */
	sil_wrb_mem((void *)intreg_addr , 
		sil_reb_mem((void *)intreg_addr) & ~(0x01U << 6));
	/* 割込み禁止状態ビットをクリア */
	disint_table[(intno / 16u)] &= ~(1u << (intno % 16u));
	
	return(true);
}

#define t_enable_int(intno) x_enable_int(intno)
#define i_enable_int(intno) x_enable_int(intno)


/*
 *  割込み要求のクリア
 */
Inline void
x_clear_int(INTNO intno)
{
	uint32_t intreg_addr = INTREG_ADDRESS(intno);
	
	if(!VALID_INTNO_DISINT(intno))
	{
		return ;
	}
	
	/* 7bit目をクリア */
	sil_wrb_mem((void *)intreg_addr , 
		sil_reb_mem((void *)intreg_addr) & ~(0x01U << 7));
}

#define t_clear_int(intno)		x_clear_int(intno)
#define i_clear_int(intno)		x_clear_int(intno)

/*
 *  割込み要求のチェック
 */
Inline bool_t
x_probe_int(INTNO intno)
{
	uint32_t intreg_addr = INTREG_ADDRESS(intno);
	
	if(!VALID_INTNO_DISINT(intno))
	{
		return false;
	}
	
	/* 6bit目のビット値で判定する．*/
	if((sil_reb_mem((void *)intreg_addr) & (0x01U << 6u)) != 0x00)
	{
		return true;
	}
	
	return false;
}

#define t_probe_int(intno) x_probe_int(intno)
#define i_probe_int(intno) x_probe_int(intno)


/*
 *  割込み要求ラインの属性の設定
 *
 *  V850では，カーネルで扱える割込み優先度は8段階であるため，intpri
 *  として与えることができる値は-7〜0が標準である．
 */
extern void x_config_int(INTNO intno, ATR intatr, PRI intpri);

/*
 *  割込みハンドラ入口で必要なIRC操作
 *
 *  割込み要因は自動クリアされるため何もしない
 */
Inline void
i_begin_int(INTNO intno)
{
}

/*
 *  割込みハンドラの出口で必要なIRC操作
 *
 * 
 */
Inline void
i_end_int(INTNO intno)
{
}

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  CPU例外ハンドラ関係
 */

/*
 *  例外ハンドラ引数 p_excinf から各種情報を取り出すためのマクロ
 */
#define P_EXCINF_OFFSET_PC				UINT_C(0)
#define P_EXCINF_OFFSET_SP				UINT_C(4)
#define P_EXCINF_OFFSET_PSW				UINT_C(8)
#define P_EXCINF_OFFSET_ECR				UINT_C(12)
#define P_EXCINF_OFFSET_CPU_LOCKED		UINT_C(16)

#ifndef TOPPERS_MACRO_ONLY

/*
 *  CPU例外ハンドラの設定
 *
 *  テンプレートファイルで例外ハンドラを生成するため，空に定義する．
 */
#define x_define_exc(excno, exc_entry)


/*
 *  CPU例外ハンドラの入口処理の生成マクロ
 */
#define EXC_ENTRY2(excno, exchdr)    _kernel_##exchdr##_##excno
#define EXC_ENTRY(excno, exchdr)     EXC_ENTRY2(excno, exchdr)
#define EXCHDR_ENTRY2(excno, exchdr) extern void _kernel_##exchdr##_##excno(void *sp);
#define EXCHDR_ENTRY(excno, excno_num, exchdr)  EXCHDR_ENTRY2(excno, exchdr)



/*
 *  CPU例外の発生した時のコンテキストの参照
 *
 *  CPU例外の発生した時のコンテキストが，タスクコンテキストの時にfalse，
 *  そうでない時にtrueを返す．
 */
Inline bool_t
exc_sense_context(void)
{
	return (intnest > 1u)? true : false;
}

/*
 *  CPU例外の発生した時の全割込み禁止状態の参照
 *
 *  CPU例外の発生した時の全割込み禁止状態が，全割込み禁止解除状態の時にfalse，
 *  そうでない時にtrueを返す．
 */
Inline bool_t
exc_sense_id(void *p_excinf)
{
	return (((*((uint32_t *)((uint32_t)p_excinf + P_EXCINF_OFFSET_PSW)))
				& (1u << 5)) != 0u)?
			true : false;
}

/*
 *  CPU例外の発生した時のCPUロック状態の参照
 *
 *  CPU例外の発生した時のCPUロック状態が，CPUロック解除状態の時にfalse，
 *  そうでない時にtrueを返す．
 */
Inline bool_t
exc_sense_lock(void *p_excinf)
{
	return *((bool_t *)((uint32_t)p_excinf + P_EXCINF_OFFSET_CPU_LOCKED));
}

/*
 *  CPU例外の発生した時のコンテキストと割込みのマスク状態の参照
 *
 *  CPU例外の発生した時のシステム状態が，カーネル実行中でなく，タスクコ
 *  ンテキストであり，全割込みロック状態でなく，CPUロック状態でなく，（モ
 *  デル上の）割込み優先度マスク全解除状態である時にtrue，そうでない時
 *  にfalseを返す（CPU例外がカーネル管理外の割込み処理中で発生した場合
 *  にもfalseを返す）．
 */
Inline bool_t
exc_sense_intmask(void *p_excinf)
{
	return ((exc_sense_context() == false)
			&& (exc_sense_lock(p_excinf) == false)
			&& (exc_sense_id(p_excinf) == false)
			&& (get_intpri() == INTPRI_ENAALL))?
			true : false;
}

/*
 * Trapa以外の例外で登録されていない例外が発生すると呼び出される
 */
extern void default_exc_handler(void *p_excinf);

/*
 * 未登録の割込みが発生した場合に呼び出される
 */
extern void default_int_handler(void *p_excinf);

/*
 *  プロセッサ依存の初期化
 */
extern void prc_initialize(void);

/*
 *  プロセッサ依存の終了時処理
 */
extern void prc_terminate(void);

/*
 *  atexitの処理とデストラクタの実行
 */
Inline void
call_atexit(void)
{
    extern void    software_term_hook(void);
    void (*volatile fp)(void) = software_term_hook;

    /*
     *  software_term_hookへのポインタを，一旦volatile指定のあるfpに代
     *  入してから使うのは，0との比較が最適化で削除されないようにするた
     *  めである．
     */
    if (fp != 0) {
        (*fp)();
    }
}

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_PRC_CONFIG_H */
