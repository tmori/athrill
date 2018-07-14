/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2017 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
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
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  $Id: kernel.h 801 2017-07-20 16:07:56Z ertl-hiro $
 */

/*
 *		TOPPERS/ASPカーネル 標準ヘッダファイル
 *
 *  TOPPERS/ASPカーネルがサポートするサービスコールの宣言と，必要なデー
 *  タ型，定数，マクロの定義を含むヘッダファイル．
 *
 *  アセンブリ言語のソースファイルからこのファイルをインクルードする時
 *  は，TOPPERS_MACRO_ONLYを定義しておく．これにより，マクロ定義以外を
 *  除くようになっている．
 */

#ifndef TOPPERS_KERNEL_H
#define TOPPERS_KERNEL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 *	TOPPERS共通のデータ型・定数・マクロ
 */
#include <t_stddef.h>

/*
 *  ターゲット依存部
 */
#include "target_kernel.h"

/*
 *  サポートする機能
 */
#ifdef TOPPERS_TARGET_SUPPORT_DIS_INT
#define TOPPERS_SUPPORT_DIS_INT			/* dis_intがサポートされている */
#endif /* TOPPERS_TARGET_SUPPORT_DIS_INT */

#ifdef TOPPERS_TARGET_SUPPORT_ENA_INT
#define TOPPERS_SUPPORT_ENA_INT			/* ena_intがサポートされている */
#endif /* TOPPERS_TARGET_SUPPORT_ENA_INT */

#ifdef TOPPERS_TARGET_SUPPORT_CLR_INT
#define TOPPERS_SUPPORT_CLR_INT			/* clr_intがサポートされている */
#endif /* TOPPERS_TARGET_SUPPORT_CLR_INT */

#ifdef TOPPERS_TARGET_SUPPORT_RAS_INT
#define TOPPERS_SUPPORT_RAS_INT			/* ras_intがサポートされている */
#endif /* TOPPERS_TARGET_SUPPORT_RAS_INT */

#ifdef TOPPERS_TARGET_SUPPORT_PRB_INT
#define TOPPERS_SUPPORT_PRB_INT			/* prb_intがサポートされている */
#endif /* TOPPERS_TARGET_SUPPORT_PRB_INT */

#define TOPPERS_SUPPORT_DYNAMIC_CRE		/* 動的生成機能拡張 */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  データ型の定義
 */

/*
 *  ビットパターンやオブジェクト番号の型定義
 */
typedef	uint_t		FLGPTN;		/* イベントフラグのビットパターン */
typedef	uint_t		INTNO;		/* 割込み番号 */
typedef	uint_t		INHNO;		/* 割込みハンドラ番号 */
typedef	uint_t		EXCNO;		/* CPU例外ハンドラ番号 */

/*
 *  処理単位の型定義
 */
typedef void	(*TASK)(intptr_t exinf);
typedef void	(*TMEHDR)(intptr_t exinf);
typedef void	(*ISR)(intptr_t exinf);
typedef void	(*INTHDR)(void);
typedef void	(*EXCHDR)(void *p_excinf);
typedef void	(*INIRTN)(intptr_t exinf);
typedef void	(*TERRTN)(intptr_t exinf);

/*
 *  メモリ領域確保のための型定義
 */
#ifndef TOPPERS_STK_T
#define TOPPERS_STK_T	intptr_t
#endif /* TOPPERS_STK_T */
typedef	TOPPERS_STK_T	STK_T;	/* スタック領域を確保するための型 */

#ifndef TOPPERS_MPF_T
#define TOPPERS_MPF_T	intptr_t
#endif /* TOPPERS_MPF_T */
typedef	TOPPERS_MPF_T	MPF_T;	/* 固定長メモリプール領域を確保するための型 */

/*
 *  タイムイベントの通知方法のパケット形式の定義
 */
typedef struct {
	intptr_t	exinf;		/* タイムイベントハンドラの拡張情報 */
	TMEHDR		tmehdr;		/* タイムイベントハンドラの先頭番地 */
} T_NFY_HDR;

typedef struct {
	intptr_t	*p_var;		/* 変数の番地 */
	intptr_t	value;		/* 設定する値 */
} T_NFY_VAR;

typedef struct {
	intptr_t	*p_var;		/* 変数の番地 */
} T_NFY_IVAR;

typedef struct {
	ID			tskid;		/* タスクID */
} T_NFY_TSK;

typedef struct {
	ID			semid;		/* セマフォID */
} T_NFY_SEM;

typedef struct {
	ID			flgid;		/* イベントフラグID */
	FLGPTN		flgptn;		/* セットするビットパターン */
} T_NFY_FLG;

typedef struct {
	ID			dtqid;		/* データキューID */
	intptr_t	data;		/* 送信する値 */
} T_NFY_DTQ;

typedef struct {
	intptr_t	*p_var;		/* 変数の番地 */
} T_ENFY_VAR;

typedef struct {
	ID			dtqid;		/* データキューID */
} T_ENFY_DTQ;

typedef struct {
	MODE	nfymode;			/* 通知処理モード */
	union {						/* タイムイベントの通知に関する付随情報 */
		T_NFY_HDR	handler;
		T_NFY_VAR	setvar;		
		T_NFY_IVAR	incvar;		
		T_NFY_TSK	acttsk;
		T_NFY_TSK	wuptsk;
		T_NFY_SEM	sigsem;
		T_NFY_FLG	setflg;
		T_NFY_DTQ	snddtq;
	} nfy;
	union {						/* エラーの通知に関する付随情報 */
		T_ENFY_VAR	setvar;
		T_NFY_IVAR	incvar;		
		T_NFY_TSK	acttsk;
		T_NFY_TSK	wuptsk;
		T_NFY_SEM	sigsem;
		T_NFY_FLG	setflg;
		T_ENFY_DTQ	snddtq;
	} enfy;
} T_NFYINFO;

/*
 *  パケット形式の定義
 */
typedef struct t_ctsk {
	ATR			tskatr;		/* タスク属性 */
	intptr_t	exinf;		/* タスクの拡張情報 */
	TASK		task;		/* タスクのメインルーチンの先頭番地 */
	PRI			itskpri;	/* タスクの起動時優先度 */
	size_t		stksz;		/* タスクのスタック領域のサイズ */
	STK_T 		*stk;		/* タスクのスタック領域の先頭番地 */
} T_CTSK;

typedef struct t_rtsk {
	STAT	tskstat;	/* タスク状態 */
	PRI		tskpri;		/* タスクの現在優先度 */
	PRI		tskbpri;	/* タスクのベース優先度 */
	STAT	tskwait;	/* 待ち要因 */
	ID		wobjid;		/* 待ち対象のオブジェクトのID */
	TMO		lefttmo;	/* タイムアウトするまでの時間 */
	uint_t	actcnt;		/* 起動要求キューイング数 */
	uint_t	wupcnt;		/* 起床要求キューイング数 */
	bool_t	raster;		/* タスク終了要求状態 */
	bool_t	dister;		/* タスク終了禁止状態 */
} T_RTSK;

typedef struct t_csem {
	ATR		sematr;		/* セマフォ属性 */
	uint_t	isemcnt;	/* セマフォの初期資源数 */
	uint_t	maxsem;		/* セマフォの最大資源数 */
} T_CSEM;

typedef struct t_rsem {
	ID		wtskid;		/* セマフォの待ち行列の先頭のタスクのID番号 */
	uint_t	semcnt;		/* セマフォの現在の資源数 */
} T_RSEM;

typedef struct t_cflg {
	ATR		flgatr;		/* イベントフラグ属性 */
	FLGPTN	iflgptn;	/* イベントフラグの初期ビットパターン */
} T_CFLG;

typedef struct t_rflg {
	ID		wtskid;		/* イベントフラグの待ち行列の先頭のタスクのID番号 */
	FLGPTN	flgptn;		/* イベントフラグの現在のビットパターン */
} T_RFLG;

typedef struct t_cdtq {
	ATR		dtqatr;		/* データキュー属性 */
	uint_t	dtqcnt;		/* データキュー管理領域に格納できるデータ数 */
	void 	*dtqmb;		/* データキュー管理領域の先頭番地 */
} T_CDTQ;

typedef struct t_rdtq {
	ID		stskid;		/* データキューの送信待ち行列の先頭のタスクのID番号 */
	ID		rtskid;		/* データキューの受信待ち行列の先頭のタスクのID番号 */
	uint_t	sdtqcnt;	/* データキュー管理領域に格納されているデータの数 */
} T_RDTQ;

typedef struct t_cpdq {
	ATR		pdqatr;		/* 優先度データキュー属性 */
	uint_t	pdqcnt;		/* 優先度データキュー管理領域に格納できるデータ数 */
	PRI		maxdpri;	/* 優先度データキューに送信できるデータ優先度の最
						   大値 */
	void 	*pdqmb;		/* 優先度データキュー管理領域の先頭番地 */
} T_CPDQ;

typedef struct t_rpdq {
	ID		stskid;		/* 優先度データキューの送信待ち行列の先頭のタスク
						   のID番号 */
	ID		rtskid;		/* 優先度データキューの受信待ち行列の先頭のタスク
						   のID番号 */
	uint_t	spdqcnt;	/* 優先度データキュー管理領域に格納されているデー
						   タの数 */
} T_RPDQ;

typedef struct t_cmtx {
	ATR		mtxatr;		/* ミューテックス属性 */
	PRI		ceilpri;	/* ミューテックスの上限優先度 */
} T_CMTX;

typedef struct t_rmtx {
	ID		htskid;		/* ミューテックスをロックしているタスクのID番号 */
	ID		wtskid;		/* ミューテックスの待ち行列の先頭のタスクのID番号 */
} T_RMTX;

typedef struct t_cmpf {
	ATR		mpfatr;		/* 固定長メモリプール属性 */
	uint_t	blkcnt;		/* 獲得できる固定長メモリブロックの数 */
	uint_t	blksz;		/* 固定長メモリブロックのサイズ */
	MPF_T 	*mpf;		/* 固定長メモリプール領域の先頭番地 */
	void 	*mpfmb;		/* 固定長メモリプール管理領域の先頭番地 */
} T_CMPF;

typedef struct t_rmpf {
	ID		wtskid;		/* 固定長メモリプールの待ち行列の先頭のタスクの
						   ID番号 */
	uint_t	fblkcnt;	/* 固定長メモリプール領域の空きメモリ領域に割り
						   付けることができる固定長メモリブロックの数 */
} T_RMPF;

typedef struct t_ccyc {
	ATR			cycatr;		/* 周期通知属性 */
	T_NFYINFO	nfyinfo;	/* 周期通知の通知方法 */
	RELTIM		cyctim;		/* 周期通知の通知周期 */
	RELTIM		cycphs;		/* 周期通知の通知位相 */
} T_CCYC;

typedef struct t_rcyc {
	STAT	cycstat;	/* 周期通知の動作状態 */
	RELTIM	lefttim;	/* 次回通知時刻までの相対時間 */
} T_RCYC;

typedef struct t_calm {
	ATR			almatr;		/* アラーム通知属性 */
	T_NFYINFO	nfyinfo;	/* アラーム通知の通知方法 */
} T_CALM;

typedef struct t_ralm {
	STAT	almstat;	/* アラーム通知の動作状態 */
	RELTIM	lefttim;	/* 通知時刻までの相対時間 */
} T_RALM;

typedef struct t_cisr {
	ATR			isratr;		/* 割込みサービスルーチン属性 */
	intptr_t	exinf;		/* 割込みサービスルーチンの拡張情報 */
	INTNO		intno;		/* 割込みサービスルーチンを登録する割込み番号 */
	ISR			isr;		/* 割込みサービスルーチンの先頭番地 */
	PRI			isrpri;		/* 割込みサービスルーチン優先度 */
} T_CISR;

/*
 *  サービスコールの宣言
 */

/*
 *  タスク管理機能
 */
extern ER_UINT	acre_tsk(const T_CTSK *pk_ctsk) throw();
extern ER		del_tsk(ID tskid) throw();
extern ER		act_tsk(ID tskid) throw();
extern ER_UINT	can_act(ID tskid) throw();
extern ER		get_tst(ID tskid, STAT *p_tskstat) throw();
extern ER		chg_pri(ID tskid, PRI tskpri) throw();
extern ER		get_pri(ID tskid, PRI *p_tskpri) throw();
extern ER		get_inf(intptr_t *p_exinf) throw();
extern ER		ref_tsk(ID tskid, T_RTSK *pk_rtsk) throw();

/*
 *  タスク付属同期機能
 */
extern ER		slp_tsk(void) throw();
extern ER		tslp_tsk(TMO tmout) throw();
extern ER		wup_tsk(ID tskid) throw();
extern ER_UINT	can_wup(ID tskid) throw();
extern ER		rel_wai(ID tskid) throw();
extern ER		sus_tsk(ID tskid) throw();
extern ER		rsm_tsk(ID tskid) throw();
extern ER		dly_tsk(RELTIM dlytim) throw();

/*
 *  タスク終了機能
 */
extern ER		ext_tsk(void) throw();
extern ER		ras_ter(ID tskid) throw();
extern ER		dis_ter(void) throw();
extern ER		ena_ter(void) throw();
extern bool_t	sns_ter(void) throw();
extern ER		ter_tsk(ID tskid) throw();

/*
 *  同期・通信機能
 */
extern ER_ID	acre_sem(const T_CSEM *pk_csem) throw();
extern ER		del_sem(ID semid) throw();
extern ER		sig_sem(ID semid) throw();
extern ER		wai_sem(ID semid) throw();
extern ER		pol_sem(ID semid) throw();
extern ER		twai_sem(ID semid, TMO tmout) throw();
extern ER		ini_sem(ID semid) throw();
extern ER		ref_sem(ID semid, T_RSEM *pk_rsem) throw();

extern ER_ID	acre_flg(const T_CFLG *pk_cflg) throw();
extern ER		del_flg(ID flgid) throw();
extern ER		set_flg(ID flgid, FLGPTN setptn) throw();
extern ER		clr_flg(ID flgid, FLGPTN clrptn) throw();
extern ER		wai_flg(ID flgid, FLGPTN waiptn,
						MODE wfmode, FLGPTN *p_flgptn) throw();
extern ER		pol_flg(ID flgid, FLGPTN waiptn,
						MODE wfmode, FLGPTN *p_flgptn) throw();
extern ER		twai_flg(ID flgid, FLGPTN waiptn,
						MODE wfmode, FLGPTN *p_flgptn, TMO tmout) throw();
extern ER		ini_flg(ID flgid) throw();
extern ER		ref_flg(ID flgid, T_RFLG *pk_rflg) throw();

extern ER_ID	acre_dtq(const T_CDTQ *pk_cdtq) throw();
extern ER		del_dtq(ID dtqid) throw();
extern ER		snd_dtq(ID dtqid, intptr_t data) throw();
extern ER		psnd_dtq(ID dtqid, intptr_t data) throw();
extern ER		tsnd_dtq(ID dtqid, intptr_t data, TMO tmout) throw();
extern ER		fsnd_dtq(ID dtqid, intptr_t data) throw();
extern ER		rcv_dtq(ID dtqid, intptr_t *p_data) throw();
extern ER		prcv_dtq(ID dtqid, intptr_t *p_data) throw();
extern ER		trcv_dtq(ID dtqid, intptr_t *p_data, TMO tmout) throw();
extern ER		ini_dtq(ID dtqid) throw();
extern ER		ref_dtq(ID dtqid, T_RDTQ *pk_rdtq) throw();

extern ER_ID	acre_pdq(const T_CPDQ *pk_cpdq) throw();
extern ER		del_pdq(ID pdqid) throw();
extern ER		snd_pdq(ID pdqid, intptr_t data, PRI datapri) throw();
extern ER		psnd_pdq(ID pdqid, intptr_t data, PRI datapri) throw();
extern ER		tsnd_pdq(ID pdqid, intptr_t data,
										PRI datapri, TMO tmout) throw();
extern ER		rcv_pdq(ID pdqid, intptr_t *p_data, PRI *p_datapri) throw();
extern ER		prcv_pdq(ID pdqid, intptr_t *p_data, PRI *p_datapri) throw();
extern ER		trcv_pdq(ID pdqid, intptr_t *p_data,
										PRI *p_datapri, TMO tmout) throw();
extern ER		ini_pdq(ID pdqid) throw();
extern ER		ref_pdq(ID pdqid, T_RPDQ *pk_rpdq) throw();

extern ER_ID	acre_mtx(const T_CMTX *pk_cmtx) throw();
extern ER		del_mtx(ID mtxid) throw();
extern ER		loc_mtx(ID mtxid) throw();
extern ER		ploc_mtx(ID mtxid) throw();
extern ER		tloc_mtx(ID mtxid, TMO tmout) throw();
extern ER		unl_mtx(ID mtxid) throw();
extern ER		ini_mtx(ID mtxid) throw();
extern ER		ref_mtx(ID mtxid, T_RMTX *pk_rmtx) throw();

/*
 *  メモリプール管理機能
 */
extern ER_ID	acre_mpf(const T_CMPF *pk_cmpf) throw();
extern ER		del_mpf(ID mpfid) throw();
extern ER		get_mpf(ID mpfid, void **p_blk) throw();
extern ER		pget_mpf(ID mpfid, void **p_blk) throw();
extern ER		tget_mpf(ID mpfid, void **p_blk, TMO tmout) throw();
extern ER		rel_mpf(ID mpfid, void *blk) throw();
extern ER		ini_mpf(ID mpfid) throw();
extern ER		ref_mpf(ID mpfid, T_RMPF *pk_rmpf) throw();

/*
 *  時間管理機能
 */
extern ER		set_tim(SYSTIM systim) throw();
extern ER		get_tim(SYSTIM *p_systim) throw();
extern ER		adj_tim(int32_t adjtim) throw();
extern HRTCNT	fch_hrt(void) throw();

extern ER_ID	acre_cyc(const T_CCYC *pk_ccyc) throw();
extern ER		del_cyc(ID cycid) throw();
extern ER		sta_cyc(ID cycid) throw();
extern ER		stp_cyc(ID cycid) throw();
extern ER		ref_cyc(ID cycid, T_RCYC *pk_rcyc) throw();

extern ER_ID	acre_alm(const T_CALM *pk_calm) throw();
extern ER		del_alm(ID almid) throw();
extern ER		sta_alm(ID almid, RELTIM almtim) throw();
extern ER		stp_alm(ID almid) throw();
extern ER		ref_alm(ID almid, T_RALM *pk_ralm) throw();

/*
 *  システム状態管理機能
 */
extern ER		rot_rdq(PRI tskpri) throw();
extern ER		get_tid(ID *p_tskid) throw();
extern ER		get_lod(PRI tskpri, uint_t *p_load) throw();
extern ER		get_nth(PRI tskpri, uint_t nth, ID *p_tskid) throw();
extern ER		loc_cpu(void) throw();
extern ER		unl_cpu(void) throw();
extern ER		dis_dsp(void) throw();
extern ER		ena_dsp(void) throw();
extern bool_t	sns_ctx(void) throw();
extern bool_t	sns_loc(void) throw();
extern bool_t	sns_dsp(void) throw();
extern bool_t	sns_dpn(void) throw();
extern bool_t	sns_ker(void) throw();
extern ER		ext_ker(void) throw();

/*
 *  割込み管理機能
 */
extern ER_ID	acre_isr(const T_CISR *pk_cisr) throw();
extern ER		del_isr(ID isrid) throw();
extern ER		dis_int(INTNO intno) throw();
extern ER		ena_int(INTNO intno) throw();
extern ER		clr_int(INTNO intno) throw();
extern ER		ras_int(INTNO intno) throw();
extern ER_BOOL	prb_int(INTNO intno) throw();
extern ER		chg_ipm(PRI intpri) throw();
extern ER		get_ipm(PRI *p_intpri) throw();

/*
 *  CPU例外管理機能
 */
extern bool_t	xsns_dpn(void *p_excinf) throw();

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  非タスクコンテキストから呼び出せるサービスコール
 */
#define iact_tsk(tskid)						act_tsk(tskid)
#define iwup_tsk(tskid)						wup_tsk(tskid)
#define irel_wai(tskid)						rel_wai(tskid)
#define isns_ter()							sns_ter()
#define isig_sem(semid)						sig_sem(semid)
#define iset_flg(flgid, setptn)				set_flg(flgid, setptn)
#define ipsnd_dtq(dtqid, data)				psnd_dtq(dtqid, data)
#define ifsnd_dtq(dtqid, data)				fsnd_dtq(dtqid, data)
#define ipsnd_pdq(pdqid, data, datapri)		psnd_pdq(pdqid, data, datapri)
#define ifch_hrt()							fch_hrt()
#define ista_alm(almid, almtim)				sta_alm(almid, almtim)
#define istp_alm(almid)						stp_alm(almid)
#define irot_rdq(tskpri)					rot_rdq(tskpri)
#define iget_tid(p_tskid)					get_tid(p_tskid)
#define iloc_cpu()							loc_cpu()
#define iunl_cpu()							unl_cpu()
#define isns_ctx()							sns_ctx()
#define isns_loc()							sns_loc()
#define isns_dsp()							sns_dsp()
#define isns_dpn()							sns_dpn()
#define isns_ker()							sns_ker()
#define iext_ker()							ext_ker()
#define idis_int(intno)						dis_int(intno)
#define iena_int(intno)						ena_int(intno)
#define iclr_int(intno)						clr_int(intno)
#define iras_int(intno)						ras_int(intno)
#define iprb_int(intno)						prb_int(intno)
#define ixsns_dpn(p_excinf)					xsns_dpn(p_excinf)

/*
 *  オブジェクト属性の定義
 */
#define TA_ACT			UINT_C(0x01)	/* タスクを起動された状態で生成 */
#define TA_NOACTQUE		UINT_C(0x02)	/* 起動要求をキューイングしない */

#define TA_TPRI			UINT_C(0x01)	/* タスクの待ち行列を優先度順に */

#define TA_WMUL			UINT_C(0x02)	/* 複数の待ちタスク */
#define TA_CLR			UINT_C(0x04)	/* イベントフラグのクリア指定 */

#define TA_CEILING		UINT_C(0x03)	/* 優先度上限プロトコル */

#define TA_STA			UINT_C(0x02)	/* 周期通知を動作状態で生成 */

#define TA_NONKERNEL	UINT_C(0x02)	/* カーネル管理外の割込み */

#define TA_ENAINT		UINT_C(0x01)	/* 割込み要求禁止フラグをクリア */
#define TA_EDGE			UINT_C(0x02)	/* エッジトリガ */

/*
 *  サービスコールの動作モードの定義
 */
#define TWF_ORW			UINT_C(0x01)	/* イベントフラグのOR待ち */
#define TWF_ANDW		UINT_C(0x02)	/* イベントフラグのAND待ち */

/*
 *  通知処理モードの定義
 */
#define TNFY_HANDLER	UINT_C(0x00)		/* タイムイベントハンドラの呼出し */
#define TNFY_SETVAR		UINT_C(0x01)		/* 変数の設定 */
#define TNFY_INCVAR		UINT_C(0x02)		/* 変数のインクリメント */
#define TNFY_ACTTSK		UINT_C(0x03)		/* タスクの起動 */
#define TNFY_WUPTSK		UINT_C(0x04)		/* タスクの起床 */
#define TNFY_SIGSEM		UINT_C(0x05)		/* セマフォの資源の返却 */
#define TNFY_SETFLG		UINT_C(0x06)		/* イベントフラグのセット */
#define TNFY_SNDDTQ		UINT_C(0x07)		/* データキューへの送信 */

#define TENFY_SETVAR	UINT_C(0x10)		/* 変数の設定 */
#define TENFY_INCVAR	UINT_C(0x20)		/* 変数のインクリメント */
#define TENFY_ACTTSK	UINT_C(0x30)		/* タスクの起動 */
#define TENFY_WUPTSK	UINT_C(0x40)		/* タスクの起床 */
#define TENFY_SIGSEM	UINT_C(0x50)		/* セマフォの返却 */
#define TENFY_SETFLG	UINT_C(0x60)		/* イベントフラグのセット */
#define TENFY_SNDDTQ	UINT_C(0x70)		/* データキューへの送信 */

/*
 *  オブジェクトの状態の定義
 */
#define TTS_RUN			UINT_C(0x01)	/* 実行状態 */
#define TTS_RDY			UINT_C(0x02)	/* 実行可能状態 */
#define TTS_WAI			UINT_C(0x04)	/* 待ち状態 */
#define TTS_SUS			UINT_C(0x08)	/* 強制待ち状態 */
#define TTS_WAS			UINT_C(0x0c)	/* 二重待ち状態 */
#define TTS_DMT			UINT_C(0x10)	/* 休止状態 */

#define TTW_SLP			UINT_C(0x0001)	/* 起床待ち */
#define TTW_DLY			UINT_C(0x0002)	/* 時間経過待ち */
#define TTW_SEM			UINT_C(0x0004)	/* セマフォの資源獲得待ち */
#define TTW_FLG			UINT_C(0x0008)	/* イベントフラグ待ち */
#define TTW_SDTQ		UINT_C(0x0010)	/* データキューへの送信待ち */
#define TTW_RDTQ		UINT_C(0x0020)	/* データキューからの受信待ち */
#define TTW_SPDQ		UINT_C(0x0100)	/* 優先度データキューへの送信待ち */
#define TTW_RPDQ		UINT_C(0x0200)	/* 優先度データキューからの受信待ち */
#define TTW_MTX			UINT_C(0x0080)	/* ミューテックスのロック待ち状態 */
#define TTW_MPF			UINT_C(0x2000)	/* 固定長メモリブロックの獲得待ち */

#define TCYC_STP		UINT_C(0x01)	/* 周期通知が動作していない */
#define TCYC_STA		UINT_C(0x02)	/* 周期通知が動作している */

#define TALM_STP		UINT_C(0x01)	/* アラーム通知が動作していない */
#define TALM_STA		UINT_C(0x02)	/* アラーム通知が動作している */

/*
 *  その他の定数の定義
 */
#define TSK_SELF		0			/* 自タスク指定 */
#define TSK_NONE		0			/* 該当するタスクがない */

#define TPRI_SELF		0			/* 自タスクのベース優先度 */
#define TPRI_INI		0			/* タスクの起動時優先度 */

#define TIPM_ENAALL		0			/* 割込み優先度マスク全解除 */

/*
 *  構成定数とマクロ
 */

/*
 *  優先度の範囲
 */
#define TMIN_TPRI		1			/* タスク優先度の最小値（最高値）*/
#define TMAX_TPRI		16			/* タスク優先度の最大値（最低値）*/
#define TMIN_DPRI		1			/* データ優先度の最小値（最高値）*/
#define TMAX_DPRI		16			/* データ優先度の最大値（最低値）*/
#define TMIN_ISRPRI		1			/* 割込みサービスルーチン優先度の最小値 */
#define TMAX_ISRPRI		16			/* 割込みサービスルーチン優先度の最大値 */

/*
 *  バージョン情報
 */
#define TKERNEL_MAKER	UINT_C(0x0118)	/* カーネルのメーカーコード */
#define TKERNEL_PRID	UINT_C(0x0007)	/* カーネルの識別番号 */
#define TKERNEL_SPVER	UINT_C(0xf631)	/* カーネル仕様のバージョン番号 */
#define TKERNEL_PRVER	UINT_C(0x3020)	/* カーネルのバージョン番号 */

/*
 *  キューイング回数の最大値
 */
#define TMAX_ACTCNT		UINT_C(1)		/* 起動要求キューイング数の最大値 */
#define TMAX_WUPCNT		UINT_C(1)		/* 起床要求キューイング数の最大値 */

/*
 *  ビットパターンのビット数
 */
#ifndef TBIT_FLGPTN					/* イベントフラグのビット数 */
#define TBIT_FLGPTN		(sizeof(FLGPTN) * CHAR_BIT)
#endif /* TBIT_FLGPTN */

/*
 *  システム時刻の調整できる範囲（単位：μ秒）
 */
#define TMIN_ADJTIM		-1000000		/* システム時刻の最小調整時間 */
#define TMAX_ADJTIM		1000000			/* システム時刻の最大調整時間 */

/*
 *  メモリ領域確保のためのマクロ
 *
 *  以下のTOPPERS_COUNT_SZとTOPPERS_ROUND_SZの定義は，unitが2の巾乗であ
 *  ることを仮定している．
 */
#ifndef TOPPERS_COUNT_SZ
#define TOPPERS_COUNT_SZ(sz, unit)	(((sz) + (unit) - 1) / (unit))
#endif /* TOPPERS_COUNT_SZ */
#ifndef TOPPERS_ROUND_SZ
#define TOPPERS_ROUND_SZ(sz, unit)	(((sz) + (unit) - 1) & ~((unit) - 1))
#endif /* TOPPERS_ROUND_SZ */

#define COUNT_STK_T(sz)		TOPPERS_COUNT_SZ(sz, sizeof(STK_T))
#define ROUND_STK_T(sz)		TOPPERS_ROUND_SZ(sz, sizeof(STK_T))

#define COUNT_MPF_T(blksz)	TOPPERS_COUNT_SZ(blksz, sizeof(MPF_T))
#define ROUND_MPF_T(blksz)	TOPPERS_ROUND_SZ(blksz, sizeof(MPF_T))

#define TSZ_DTQMB(dtqcnt)	(sizeof(intptr_t) * (dtqcnt))
#define TCNT_DTQMB(dtqcnt)	TOPPERS_COUNT_SZ(TSZ_DTQMB(dtqcnt), sizeof(MB_T))

#ifndef TSZ_PDQMB
#define TSZ_PDQMB(pdqcnt)	(sizeof(intptr_t) * 3 * (pdqcnt))
#endif /* TSZ_PDQMB */
#define TCNT_PDQMB(pdqcnt)	TOPPERS_COUNT_SZ(TSZ_PDQMB(pdqcnt), sizeof(MB_T))

#define TSZ_MPFMB(blkcnt)	(sizeof(uint_t) * (blkcnt))
#define TCNT_MPFMB(blkcnt)	TOPPERS_COUNT_SZ(TSZ_MPFMB(blkcnt), sizeof(MB_T))

/*
 *  その他の構成定数
 */
#define TMAX_MAXSEM		UINT_MAX	/* セマフォの最大資源数の最大値 */

#ifdef __cplusplus
}
#endif

#endif /* TOPPERS_KERNEL_H */
