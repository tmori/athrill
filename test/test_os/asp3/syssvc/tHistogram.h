/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2006-2016 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: tHistogram.h 509 2016-01-12 06:06:14Z ertl-hiro $
 */

/*
 *		実行時間分布集計サービスのヘッダファイル
 */

#ifndef TOPPERS_THISTOGRAM_H
#define TOPPERS_THISTOGRAM_H

#include "target_syssvc.h"

/*
 *  ターゲット依存部で設定変更するためのマクロ
 */
#ifndef HISTTIM						/* 実行時間計測用の時刻のデータ型 */
#define HISTTIM					HRTCNT

#ifdef TCYC_HRTCNT					/* 実行時間計測用の時刻の周期 */
#define HISTTIM_CYCLE			TCYC_HRTCNT
#endif /* TCYC_HRTCNT */
#endif /* HISTTIM */

#ifndef HIST_GET_TIM				/* 実行時間計測用の現在時刻の取得 */
#define HIST_GET_TIM(p_time)	(*(p_time) = fch_hrt())
#endif /* HIST_GET_TIM */

#ifndef HIST_CONV_TIM				/* 時刻の差から実行時間への変換 */
#define HIST_CONV_TIM(time)		((uint_t)(time))
#endif /* HIST_CONV_TIM */

#ifndef HIST_BM_HOOK				/* 実行時間計測直前に行うべき処理 */
#define HIST_BM_HOOK()			((void) 0)
#endif

/*
 *  実行時間計測用の時刻のデータ型の定義
 */
typedef HISTTIM	histtim_t;

#endif /* TOPPERS_THISTOGRAM_H */
