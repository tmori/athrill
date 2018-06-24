/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2007 by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	Copyright (C) 2007-2011 by Industrial Technology Institute,
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
 *	$Id: prc_config.h 2158 2011-07-22 05:30:47Z mit-kimai $
 */

/*
 *		プロセッサ依存モジュール（SH12A用）
 *
 *	このインクルードファイルは，target_config.h（または，そこからインク
 *	ルードされるファイル）のみからインクルードされる．他のファイルから
 *	直接インクルードしてはならない．
 */

#ifndef TOPPERS_PRC_CONFIG_H
#define TOPPERS_PRC_CONFIG_H

#ifndef TOPPERS_TARGET_CONFIG_H
#error  target_config.h is not included!
#endif	/*  TOPPERS_TARGET_CONFIG_H  */

#ifndef TOPPERS_MACRO_ONLY

/*
 *	プロセッサの特殊命令のインライン関数定義
 */
#include "prc_insn.h"

/*
 *	エントリ(start.S)
 */
extern void start(void);

/*
 *	未登録の割込み用のエントリ
 */
extern void default_int_handler_entry(void);

/*
 *	非タスクコンテキスト用のスタック初期値
 */
#define TOPPERS_ISTKPT(istk, istksz) ((STK_T *)((char *)(istk) + (istksz)))

/*
 *	タスクコンテキストブロックの定義
 */
typedef struct task_context_block {
	void	*ssp;		/* システムスタックポインタ */
	void	*usp;		/* ユーザスタックポインタ */
	FP		pc;			/* プログラムカウンタ */
    uint8_t priv_mode;  /* 特権モード実行中かどうかを表すフラグ */
} TSKCTXB;

#endif /* TOPPERS_MACRO_ONLY */

/*
 *	割込み優先度マスク操作ライブラリ
 *	
 *	SH12Aでは，ステータスレジスタ（SR）の4〜7ビット目の4ビットに割込み
 *	優先度マスク（ハードウェアの割込み優先度マスク，IPM）が置かれてい
 *	る．IPMを保存しておくために，割込み優先度の外部表現（-1から連続し
 *	た負の値）を使うことも可能であるが，余計な左右ビットシフトと符号反
 *	転が必要になる．これを避けるために，IPMを保存する場合には，SRの4〜7
 *	ビット目を取り出した値を使うことにする．この値を割込み優先度マスク
 *	の内部表現と呼び，IIPMと書くことにする．
 */

/*
 *	割込み優先度マスクの外部表現と内部表現の変換
 *
 *	アセンブリ言語のソースファイルからインクルードする場合のために，CASTを使用
 */
#define EXT_IPM(iipm)	 (-CAST(PRI,(iipm) >> 4))		/* 内部表現を外部表現に */
#define INT_IPM(ipm)	 (CAST(uint32_t, -(ipm)) << 4)	/* 外部表現を内部表現に */

#ifndef TOPPERS_MACRO_ONLY

/*
 *	IPM（ハードウェアの割込み優先度マスク，内部表現）の現在値の読出し
 */
Inline uint32_t
current_iipm(void)
{
	return((uint32_t)(current_sr() & 0xF0U));
}

/*
 *	IPM（ハードウェアの割込み優先度マスク，内部表現）の現在値の設定
 */
Inline void
set_iipm(uint32_t iipm)
{
	set_sr((current_sr() & ~0xF0U) | iipm);
}

Inline void
set_iipm_with_nop(uint32_t iipm)
{
	set_sr_with_nop((current_sr() & ~0xF0U) | iipm);
}

/*
 *	TOPPERS標準割込み処理モデルの実現
 *
 *	SH12Aはステータスレジスタ(SR)内に割込み優先度マスク(IPM)を持っている．
 *	CPUロックフラグに相当機能を持たない．そのため，擬似的にCPUロックフ
 *	ラグを実現する．
 *
 *	まず，CPUロック状態を管理すための変数(lock_flag)を用意する．
 *
 *	CPUロックフラグがクリアされている間は，IPMをモデル上の割込み優先度
 *	マスクの値に設定する．この間は，モデル上の割込み優先度マスクは，
 *	IPMを用いる．
 *	
 *	それに対してCPUロックフラグがセットされている間は，IPMを，カーネル管
 *	理外のものを除くすべての割込み要求をマスクする値(TIPM_LOCK)と，モデ
 *	ル上の割込み優先度マスクとの高い方に設定する．この間のモデル上の割
 *	込み優先度マスクは，そのための変数(saved_iipm, 内部表現で保持)を用
 *	意して保持する．
 *
 *	それに対してCPUロックフラグがセットされている間は，
 *	 　・IPMを，カーネル管理外のものを除くすべての割込み要求をマスク
 *	 　　する値(TIPM_LOCK)
 *	 　・モデル上の割込み優先度マスク
 *	の２つの内、高い方に設定する．この間のモデル上の割込み優先度マスク
 *	は，そのための変数(saved_iipm, 内部表現で保持)を用意して保持する．
 *	タスクコンテキストでは，CPUロック状態以外では，saved_iipmは常にIPM
 *	と同じ値になるようにする．
 */

/*
 *	コンテキスト参照のための変数
 *
 *	例外（割込み/CPU例外）のネスト回数のカウント
 *	
 */
extern uint32_t excnest_count;

/*
 *	コンテキストの参照
 *
 *	SH12Aでは，コンテキストの判定に割込み/CPU例外のネスト回数を使用する
 *	
 */
Inline bool_t
sense_context(void)
{
	return(excnest_count > 0U);
}

#endif /* TOPPERS_MACRO_ONLY */

/*
 *	CPUロック状態での割込み優先度マスク
 *
 *	TIPM_LOCKは，CPUロック状態での割込み優先度マスク，すなわち，カーネ
 *	ル管理外のものを除くすべての割込みをマスクする値に定義する．
 */
#define TIPM_LOCK	 TMIN_INTPRI

/*
 *	CPUロック状態での割込み優先度マスクの内部表現
 */
#define IIPM_LOCK	 INT_IPM(TIPM_LOCK)

/*
 *	割込み優先度マスク全解除（TIPM_ENAALL）の内部表現
 */
#define IIPM_ENAALL  INT_IPM(TIPM_ENAALL)

/*
 *	全割込みロックの内部表現
 */
#define IIPM_DISALL  UINT_C(0xf0)

#ifndef TOPPERS_MACRO_ONLY

/*
 *	CPUロックフラグ実現のための変数
 *	
 *	これらの変数は，CPUロック状態の時のみ書き換えてもよいとする．
 */
extern bool_t	lock_flag;	 /* CPUロックフラグの値を保持する変数 */
extern uint32_t saved_iipm;  /* 割込み優先度をマスクする変数 */

/*
 *	CPUロック状態への移行
 *
 *	IPM（ハードウェアの割込み優先度マスク）を，saved_iipmに保存し，カー
 *	ネル管理外のものを除くすべての割込みをマスクする値（TIPM_LOCK）に設
 *	定する．また，lock_flagをtrueにする．
 *
 *	IPMが，最初からTIPM_LOCKと同じかそれより高い場合には，それを
 *	saved_iipmに保存するのみで，TIPM_LOCKには設定しない．これは，モデル
 *	上の割込み優先度マスクが，TIPM_LOCKと同じかそれより高いレベルに設定
 *	されている状態にあたる．
 *
 *	この関数は，CPUロック状態（lock_flagがtrueの状態）で呼ばれることは
 *	ないものと想定している．
 */
Inline void
x_lock_cpu(void)
{
	uint32_t iipm;

	/*
	 *	current_iipm()の返り値を直接saved_iipmに保存せず，一時変数iipm
	 *	を用いているのは，current_iipm()を呼んだ直後に割込みが発生し，
	 *	起動された割込み処理でsaved_iipmが変更される可能性があるためで
	 *	ある．
	 */
	iipm = current_iipm();
	if (IIPM_LOCK > iipm) {
		set_iipm(IIPM_LOCK);
	}
	saved_iipm = iipm;
	lock_flag = true;
	/* クリティカルセクションの前後でメモリが書き換わる可能性がある */
	Asm("":::"memory");
}

#define t_lock_cpu()	x_lock_cpu()
#define i_lock_cpu()	x_lock_cpu()

/*
 *	CPUロック状態の解除
 *
 *	lock_flagをfalseにし，IPM（ハードウェアの割込み優先度マスク）を，
 *	saved_iipmに保存した値に戻す．
 *
 *	この関数は，CPUロック状態（lock_flagがtrueの状態）でのみ呼ばれるも
 *	のと想定している．
 */
Inline void
x_unlock_cpu(void)
{
	/* クリティカルセクションの前後でメモリが書き換わる可能性がある */
	Asm("":::"memory");
	lock_flag = false;
	set_iipm_with_nop(saved_iipm);
}

#define t_unlock_cpu()	  x_unlock_cpu()
#define i_unlock_cpu()	  x_unlock_cpu()

/*
 *	CPUロック状態の参照
 */
Inline bool_t
x_sense_lock(void)
{
	return(lock_flag);
}

#define t_sense_lock()	  x_sense_lock()
#define i_sense_lock()	  x_sense_lock()

/*
 *	chg_ipmで有効な割込み優先度の範囲の判定
 *
 *	TMIN_INTPRIの値によらず，chg_ipmでは，-15〜TIPM_ENAALL（＝0）の範囲
 *	に設定できることとする（ターゲット定義の拡張）．
 */
#define VALID_INTPRI_CHGIPM(intpri) \
				((-15 <= (intpri)) && ((intpri) <= TIPM_ENAALL))

/*
 * （モデル上の）割込み優先度マスクの設定
 *
 *	CPUロックフラグがクリアされている時は，ハードウェアの割込み優先度マ
 *	スクを設定する．
 *	
 *	CPUロックフラグがセットされている時は，ハードウェアの割込み優先度マ
 *	スクを，設定しようとした（モデル上の）割込み優先度マスクとTIPM_LOCK
 *	の高い方に設定する．
 *
 */
Inline void
x_set_ipm(PRI intpri)
{
	uint32_t   iipm = INT_IPM(intpri);

	
	if (!lock_flag) {
		set_iipm(iipm);
	}
	else {
		/*
		 *	CPUロック状態なので、カーネル管理の割込みは入らない。
		 *	（saved_iipmの整合性は問題ない。）
		 */
		saved_iipm = iipm;
#if TIPM_LOCK == -15
		/*
		 *	TIPM_LOCKが-15の場合には，この時点でハードウェアの割込み優先
		 *	度マスクが必ず15に設定されているため，設定し直す必要がない．
		 */
#else /* TIPM_LOCK == -15 */
		set_iipm((iipm > IIPM_LOCK) ? iipm : IIPM_LOCK);
#endif /* TIPM_LOCK == -15 */
	}
}

#define t_set_ipm(intpri)	 x_set_ipm(intpri)
#define i_set_ipm(intpri)	 x_set_ipm(intpri)

/*
 * （モデル上の）割込み優先度マスクの参照
 *
 *	CPUロックフラグがクリアされている時はハードウェアの割込み優先度マ
 *	スクを，セットされている時はsaved_iipmを参照する．
 */
Inline PRI
x_get_ipm(void)
{
	uint32_t iipm;

	if (!lock_flag) {
		iipm = current_iipm();
	}
	else {
		iipm = saved_iipm;
	}
	return(EXT_IPM(iipm));
}

#define t_get_ipm()    x_get_ipm()
#define i_get_ipm()    x_get_ipm()

/*
 *	最高優先順位タスクへのディスパッチ（prc_support.S）
 *
 *	dispatchは，タスクコンテキストから呼び出されたサービスコール処理か
 *	ら呼び出すべきもので，
 *	 ・タスクコンテキスト
 *	 ・CPUロック状態
 *	 ・ディスパッチ許可状態
 *	 ・（モデル上の）割込み優先度マスク全解除状態
 *	で呼び出さなければならない．
 */
extern void dispatch(void);

/*
 *	ディスパッチャの動作開始（prc_support.S）
 *
 *	start_dispatchは，カーネル起動時に呼び出すべきもので，すべての割込
 *	みを禁止した状態（全割込みロック状態と同等の状態）で呼び出さなければ
 *	ならない．
 */
extern void start_dispatch(void) NoReturn;

/*
 *	現在のコンテキストを捨ててディスパッチ（prc_support.S）
 *
 *	exit_and_dispatchは，ext_tskから呼び出すべきもので，
 *	 ・タスクコンテキスト
 *	 ・CPUロック状態
 *	 ・ディスパッチ許可状態
 *	 ・（モデル上の）割込み優先度マスク全解除状態
 *	で呼び出さなければならない．
 */
extern void exit_and_dispatch(void) NoReturn;

/*
 *	カーネルの終了処理の呼出し（prc_support.S）
 *
 *	call_exit_kernelは，カーネルの終了時に呼び出すべきもので，非タスク
 *	コンテキストに切り換えて，カーネルの終了処理（exit_kernel）を呼び出
 *	す．
 */
extern void call_exit_kernel(void) NoReturn;

/*
 *	タスクコンテキストの初期化
 *
 *	タスクが休止状態から実行できる状態に移行する時に呼ばれる．この時点
 *	でスタック領域を使ってはならない．
 *
 *	activate_contextを，インライン関数ではなくマクロ定義としているのは，
 *	この時点ではTCBが定義されていないためである．
 */
extern void    start_stask_r(void);
extern void    start_utask_r(void);

#define activate_context(p_tcb) 										\
{\
	(p_tcb)->tskctxb.ssp = (p_tcb)->p_tinib->tskinictxb.sstk_bottom;	\
	(p_tcb)->tskctxb.usp = (p_tcb)->p_tinib->tskinictxb.stk_bottom;     \
	(p_tcb)->tskctxb.pc = ((p_tcb)->p_tinib->p_dominib->domptn == TACP_KERNEL) ? (FP)start_stask_r: (FP)start_utask_r;       \
	(p_tcb)->tskctxb.priv_mode = 1U; \
}

/*
 *	calltexは使用しない
 */
#define OMIT_CALLTEX

/*
 * ターゲット非依存部に含まれる標準の例外管理機能の初期化処理を用いない
 */
#define OMIT_INITIALIZE_EXCEPTION

/*
 * CPU例外ハンドラの初期化
 */
Inline void
initialize_exception(void)
{
}

/*
 *	SH12Aの割込みアーキテクチャと割込み処理モデルの実現
 */

/*
 *	割込み優先度
 *
 *	各割込みの割込み優先度は割込み優先レベル設定レジスタ(IPRXX)によっ
 *	て設定する．そのため，各割込み要求ライン毎にどの割込み優先レベル設
 *	定レジスタのどのフィールドに対応しているかの情報（IPR_INFO型）の
 *	テーブルipr_info_tbl[]を用意する．
 *	割込み番号は連続していないため，サポートしていない番号には，
 *	addressエントリに0を設定し、これにより割込み番号の妥当性を判別でき
 *	る。
 *	（サポートする割込み番号はアプリケーションによらず、プロセッサの
 *	ハードウェア仕様によって決まるので、定数データとして保持する。）
 *	このテーブルのインデックスには、割込み番号の内部表現を用いる。
 */

/*
 *	各割込み番号毎の設定するIPRの情報管理のための構造体
 *	
 *	　サポートしない割込み番号の場合は、addressエントリに0を設定する。
 *	
 *	　メンバoffsetは8ビットあれば十分だが、下記の理由で32ビットにして
 *	　いる。
 *	　　・この構造体を配列にすると隙間ができるだけで、メモリの節約に
 *	　　　ならない。
 *	　　・SHでは32ビットデータでないと無駄な命令が生成される。
 */
typedef struct {
	/* 割込み優先レベル設定レジスタのアドレス */
	uint32_t address;
	/* オフセット */
	uint32_t  offset;
} IPR_INFO;

/*
 *	各割込み番号毎のIPRの情報管理テーブル
 */
extern const IPR_INFO ipr_info_tbl[TNUM_INH];

/*
 *	割込み番号・割込みハンドラ番号
 *
 *	割込みハンドラ番号(inhno)と割込み番号(intno)は，ベクタ番号を用いる．
 *
 *	ベクタ番号はTMIN_INTNO(=64)から始まるため，そのままの値を優先度管
 *	理等のテーブルのインデックスに用いると，無駄な領域が発生する．
 *	そのため，カーネル内部では，TMIN_INTNOを差し引いた値（内部表現）
 *	を用いる．
 *
 *	内部表現の名前は，iintno,iinhnoとする．
 */

/*
 * 割込みハンドラ番号の内部・外部表現相互変換
 */
											/* 内部表現を外部表現に */
#define EXT_INHNO(iintno) (CAST(uint32_t, (iinhno) + TMIN_INHNO))
											/* 外部表現を内部表現に */
#define INT_INHNO(intno)  (CAST(uint32_t, (inhno) - TMIN_INHNO))

/*
 * 割込み番号の内部・外部表現相互変換
 */
											/* 内部表現を外部表現に */
#define EXT_INTNO(iintno)	(CAST(uint32_t, (iintno) + TMIN_INTNO))
											/* 外部表現を内部表現に */
#define INT_INTNO(intno)	(CAST(uint32_t, (intno) - TMIN_INTNO))


/*
 *	割込み番号の範囲の判定
 *	
 *	TMIN_INTNO〜TMAX_INTNOの範囲であっても有効な番号でない場合があるた
 *	め，ipr_info_tbl[]で有効な番号かをチェックする
 */
#define VALID_INTNO_IPR(intno) \
		((TMIN_INTNO <= (intno)) && ((intno) <= TMAX_INTNO))
#define VALID_INTNO_DISINT(intno)	 VALID_INTNO_IPR(intno)
#ifdef TINTNO_NMI
#define VALID_INTNO_CFGINT(intno)	 (VALID_INTNO_IPR(intno)		\
										|| ((intno) == TINTNO_NMI))
#else /* TINTNO_NMI */
#define VALID_INTNO_CFGINT(intno)	 VALID_INTNO_IPR(intno)
#endif /* TINTNO_NMI */

/*
 *	CPU例外ハンドラ番号
 *
 *	CPU例外ハンドラ番号としては，ベクタ番号を用いる．
 *	サポートするCPU例外ハンドラ数はプロセッサの型番毎に異なるので，プロ
 *	セッサの型番毎の定義ファイルに定義する．
 *	
 *	TMIN_EXCNO〜TMAX_EXCNOの範囲であっても有効な番号でない場合があるが、
 *	チェックを省略している。
 *
 */
#define VALID_EXCNO_DEFEXC(excno) \
		((TMIN_EXCNO <= (excno)) && ((excno) <= TMAX_EXCNO))

/*
 *	割込みハンドラの設定
 *
 *	割込みハンドラ番号inhnoの割込みハンドラの起動番地int_entryに
 *	設定する．
 */
Inline void
x_define_inh(INHNO inhno, FP int_entry)
{
}

/*
 *	割込みハンドラの出入口処理の生成マクロ
 *
 */
#define INT_ENTRY(inhno, inthdr)	 _kernel_##inthdr##_##inhno
#define INTHDR_ENTRY(inhno, inhno_num, inthdr)					\
		extern void _kernel_##inthdr##_##inhno(void);

/*
 *	割込み要求禁止フラグ
 *
 *	SH12AのIRCは割込み要求禁止フラグを持たない．割込みを個別に禁止する
 *	場合は，割込み優先レベル設定レジスタ(IPRXX)を0に設定する必要がある．
 *	割込み優先レベル設定レジスタを0にすることで割込み要求禁止フラグを
 *	実現すると、その間、モデル上の割込み優先度を記憶しておく領域が必要
 *	となる．コンフィギュレータでこのためのuint8_t型のテーブル
 *	int_iipm_tbl[]を用意する．
 *	割込み優先度が設定されているかどうかはビット7で保持する。
 *	（アプリケーションに依存した情報）
 *	
 *	ビット割り当てが割込み優先度の外部表現とも内部表現とも異なるので、
 *	注意すること。（符号を反転しているが、ビットシフトしていない。）
 *	
 *	テーブルint_iipm_tbl[]に格納する値のビット割り当て：
 *	　　　ビット0〜3：IPRXXに設定するビットパターン
 *	　　　　　　　　　（割込み優先レベル）
 *	　　　ビット7：NOT_CFG_INT_BIT
 *	　　　　　　　　0:CFG_INTで割込み優先度が設定されている。
 *	　　　　　　　　1:CFG_INTで割込み優先度が設定されていない。
 */
extern const uint8_t int_iipm_tbl[TNUM_INT];

/*	CFG_INTで割込み優先度が設定されていないことを示すビット  */
#ifndef NOT_CFG_INT_BIT
#define NOT_CFG_INT_BIT		0x80U
#endif /* NOT_CFG_INT_BIT */

/*
 * 割込み優先レベル設定レジスタの設定
 */
Inline void
irc_set_ipr(uint32_t address, uint32_t offset, uint32_t val)
{
	uint32_t ipr_val;

	ipr_val = sil_reh_mem((uint16_t *)address);
	ipr_val &= ~(0x0FU << offset);
	ipr_val |= val << offset;
	sil_wrh_mem((uint16_t *)address, ipr_val);
}

/*
 *	割込み要求ラインの属性の設定
 *
 *	SH12Aでは，カーネルで扱える割込み優先度は16段階であるため，intpri
 *	として与えることができる値は-15〜-1が標準である．
 *	
 *	SH12Aの割込みは，内蔵周辺モジュール割込み，IRQ割込みに分類でき，そ
 *	れぞれ扱いが異なる．そのため，割込み番号から，どの割込みか分類する
 *	ためのマクロINTNO_IS_IRQ(intno)をコア依存部で用意する．
 *	本来はこのファイルで定義すべきだが、コア依存部ののshx_config.h中の
 *	x_config_intatr()で使用するため、shx_config.hに記述している。
 */
extern void    x_config_int(INTNO intno, ATR intatr, PRI intpri);

/*
 *	割込みハンドラの出口で必要なIRC操作
 *	　入口で必要なIRC操作i_begin_int()は、コア依存部で定義
 */
Inline void
i_end_int(INTNO intno)
{
}
#endif /* TOPPERS_MACRO_ONLY */

/*
 *	CPU例外ハンドラ関係
 */

/*
 *	例外ハンドラ引数 p_excinf から各種情報を取り出すためのマクロ
 */
#define P_EXCINF_OFFSET_VECTOR UINT_C( 0)
#define P_EXCINF_OFFSET_PR	   UINT_C( 1)
#define P_EXCINF_OFFSET_PC	   UINT_C(12)
#define P_EXCINF_OFFSET_SR	   UINT_C(13)

#define P_EXCINF_OFFSET_R0	   UINT_C( 4)

/*	CPU例外発生時のスタックポインタの値とp_excinfの差分  */
#define P_EXCINF_OFFSET_SP	   (P_EXCINF_OFFSET_SR + 1)


#ifndef TOPPERS_MACRO_ONLY
/*
 *	CPU例外ハンドラの設定
 */
Inline void
x_define_exc(EXCNO excno, FP exc_entry)
{
	assert(VALID_EXCNO_DEFEXC(excno));
}

/*
 *	CPU例外ハンドラの入口処理の生成マクロ
 */
#define EXC_ENTRY(excno, exchdr)	 _kernel_##exchdr##_##excno
#define EXCHDR_ENTRY(excno, excno_num, exchdr)					\
		extern void _kernel_##exchdr##_##excno(void *sp);

/*
 *	CPU例外が発生した時のコンテキストの参照
 *
 *	CPU例外が発生した時のコンテキストが，タスクコンテキストの時にfalse，
 *	そうでない時にtrueを返す．
 */
Inline bool_t
exc_sense_context(void *p_excinf)
{
	return(excnest_count > 1U);
}

/*
 *	CPU例外が発生した時のIPM（ハードウェアの割込み優先度マスク，内部表
 *	現）の参照
 */
Inline uint32_t
exc_get_iipm(void *p_excinf)
{
	uint32_t *sp = (uint32_t *)p_excinf;
	uint32_t sr = *(sp + P_EXCINF_OFFSET_SR);
	uint32_t iimp = sr & 0x00F0U;
	return(iimp);
}

/*
 *	CPU例外が発生した時のコンテキストと割込みのマスク状態の参照
 *
 *	CPU例外が発生した時のシステム状態が，
 *	　・カーネル実行中でなく，
 *	　・タスクコンテキストであり，
 *	　・全割込みロック状態でなく，
 *	　・CPUロック状態でなく，
 *	　・（モデル上の）割込み優先度マスク全解除状態
 *	である時にtrue，そうでない時にfalseを返す（CPU例外がカーネル管理外
 *	の割込み処理中で発生した場合にもfalseを返す）．
 *
 *	SH12Aでは，CPU例外が発生した時のIPM（ハードウェアの割込み優先度マ
 *	スク）がすべての割込みを許可する状態であることをチェックすることで，
 *	　・カーネル実行中でないこと，
 *	　・全割込みロック状態でないこと，
 *	　・CPUロック状態でないこと，
 *	　・（モデル上の）割込み優先度マスク全解除状態であること
 *	の4つの条件をチェックすることができる（CPU例外が発生した時の
 *	lock_flagを参照する必要はない）．
 */
Inline bool_t
exc_sense_intmask(void *p_excinf)
{
	return(!exc_sense_context(p_excinf)
					&& (exc_get_iipm(p_excinf) == IIPM_ENAALL));
}

/*
 * Trapa以外の例外で登録されていない例外が発生すると呼び出される
 */
extern void default_exc_handler(void *p_excinf);

/*
 * 未登録の割込みが発生した場合に呼び出される
 * 　APIが標準的なものと異なるため、ユーザーが用意したもので
 * 　置き換える場合は、プロトタイプ宣言も削除する。
 */
#ifndef OMIT_DEFAULT_INT_HANDLER
extern void default_int_handler(INTNO intno);
#endif /* OMIT_DEFAULT_INT_HANDLER */

/*
 *	プロセッサ依存の初期化
 */
extern void prc_initialize(void);

/*
 *	プロセッサ依存の終了時処理
 */
extern void prc_exit(void);

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_PRC_CONFIG_H */
