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
 *  $Id: out.h 2 2012-05-09 02:23:52Z nces-shigihara $
 */

#include "ttsp_target_test.h"

extern void main_task(intptr_t exinf);
extern void texhdr_tex(TEXPTN texptn, intptr_t exinf);

extern void inthdr_ttsp_intno_a(void);

#if defined(TTSP_INTNO_B) || defined(TTSP_INTNO_PE2_B) || defined(TTSP_INTNO_PE3_B) || defined(TTSP_INTNO_PE4_B)
extern void inthdr_ttsp_intno_b(void);
#endif /* defined(TTSP_INTNO_B) || defined(TTSP_INTNO_PE2_B) || defined(TTSP_INTNO_PE3_B) || defined(TTSP_INTNO_PE4_B) */

#if defined(TTSP_INTNO_C) || defined(TTSP_INTNO_PE2_C) || defined(TTSP_INTNO_PE3_C) || defined(TTSP_INTNO_PE4_C)
extern void inthdr_ttsp_intno_c(void);
#endif /* defined(TTSP_INTNO_C) || defined(TTSP_INTNO_PE2_C) || defined(TTSP_INTNO_PE3_C) || defined(TTSP_INTNO_PE4_C) */

#if defined(TTSP_INTNO_D) || defined(TTSP_INTNO_PE2_D) || defined(TTSP_INTNO_PE3_D) || defined(TTSP_INTNO_PE4_D)
extern void isr_ttsp_intno_d(intptr_t exinf);
#endif /* defined(TTSP_INTNO_D) || defined(TTSP_INTNO_PE2_D) || defined(TTSP_INTNO_PE3_D) || defined(TTSP_INTNO_PE4_D) */

#if defined(TTSP_INTNO_E) || defined(TTSP_INTNO_PE2_E) || defined(TTSP_INTNO_PE3_E) || defined(TTSP_INTNO_PE4_E)
extern void isr_ttsp_intno_e(intptr_t exinf);
#endif /* defined(TTSP_INTNO_E) || defined(TTSP_INTNO_PE2_E) || defined(TTSP_INTNO_PE3_E) || defined(TTSP_INTNO_PE4_E) */

#if defined(TTSP_INTNO_F) || defined(TTSP_INTNO_PE2_F) || defined(TTSP_INTNO_PE3_F) || defined(TTSP_INTNO_PE4_F)
extern void isr_ttsp_intno_f(intptr_t exinf);
#endif /* defined(TTSP_INTNO_F) || defined(TTSP_INTNO_PE2_F) || defined(TTSP_INTNO_PE3_F) || defined(TTSP_INTNO_PE4_F) */

extern void ttsp_test_lib_init(intptr_t exinf);
