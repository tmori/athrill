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
 *  上記著作権者は，以下の(1)~(4)の条件を満たす場合に限り，本ソフトウェ
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

/*
 *  割込みが発生しなかったと判断する時間
 *  (sil_dly_nse()の引数で使用)
 */
#define TTSP_WAIT_NOT_RAISE_INT  1000000

/* sil_dly_nse()でテストする遅延時間  */
#define SIL_DLY_TIME      10000000
#define SIL_DLY_TIME_SUB  SIL_DLY_TIME / 1000000

/* メモリ読み書きで使用するデータ */
#define R_DATA8       0x12
#define R_DATA16      0x1234
#define R_DATA32      0x12345678
#define W_DATA8       0x21
#define W_DATA16      0x4321
#define W_DATA32      0x87654321
#define CLEAR32       0x00000000

/* テスト判別用列挙体 */
typedef enum e_test_type {
	CHG_IPM,
	DIS_DSP
} E_TEST_TYPE;

extern void main_task(intptr_t exinf);
//extern void texhdr(TEXPTN texptn, intptr_t exinf);
extern void almhdr(intptr_t exinf);
extern void cychdr(intptr_t exinf);
extern void exchdr(void* p_excinf);
extern void inirtn(intptr_t exinf);
extern void terrtn(intptr_t exinf);

extern void inthdr_for_int_test(void);

extern void all_test(void);
extern void part_test(E_TEST_TYPE test_type);

extern void test_of_sil_mem(void);
extern void test_of_sil_dly_nse(void);
extern void test_of_sns_ker(bool_t flag);
extern void test_of_SIL_LOC_INT(void);

extern void wait_raise_int(void);
extern void check_of_sil_mem(void);

#ifdef TTSP_INTNO_B
extern void inthdr(void);
#endif /* TTSP_INTNO_B */

#ifdef TTSP_INTNO_C
extern void isr(intptr_t exinf);
#endif /* TTSP_INTNO_C */
