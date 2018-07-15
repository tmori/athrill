/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2010-2012 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2010-2011 by Digital Craft Inc.
 *  Copyright (C) 2010-2011 by NEC Communication Systems, Ltd.
 *  Copyright (C) 2010-2012 by FUJISOFT INCORPORATED
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
 *  $Id: ttsp_chip_timer.h 12 2012-10-31 04:51:45Z ertl-sangorrin $
 */

/*
 *  タイマドライバ（ARM PrimeCell Timer Module用）
 */

#ifndef TTSP_CHIP_TIMER_H
#define TTSP_CHIP_TIMER_H

/*
 *  テスト用の関数
 */

/*
 *  ティック更新の停止（全プロセッサ）
 */ 
extern void ttsp_target_stop_tick(void);

/*
 *  ティック更新の停止（特定プロセッサ）
 */ 
extern void ttsp_target_stop_tick_pe(ID prcid);

/*
 *  ティック更新の再開（全プロセッサ）
 */ 
extern void ttsp_target_start_tick(void);

/*
 *  ティック更新の再開（特定プロセッサ）
 */ 
extern void ttsp_target_start_tick_pe(ID prcid);

/*
 *  ティックの更新（全プロセッサ）
 */
extern void ttsp_target_gain_tick(void);

/*
 *  ティックの更新（特定プロセッサ）
 */
extern void ttsp_target_gain_tick_pe(ID prcid, bool_t wait_flg);

/*
 *  タイマハンドラ呼び出し完了確認関数
 */
extern void ttsp_check_timer_handler(void);

#endif /* TTSP_CHIP_TIMER_H */
