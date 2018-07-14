/*
 *		データキュー機能のテスト(1)のヘッダファイル
 *
 *  $Id: test_dtq1.h 737 2016-04-05 13:11:23Z ertl-hiro $
 */

#include <kernel.h>

/*
 *  ターゲット依存の定義
 */
#include "target_test.h"

/*
 *  優先度の定義
 */
#define HIGH_PRIORITY	9		/* 高優先度 */
#define MID_PRIORITY	10		/* 中優先度 */
#define LOW_PRIORITY	11		/* 低優先度 */

/*
 *  ターゲットに依存する可能性のある定数の定義
 */
#ifndef STACK_SIZE
#define	STACK_SIZE		4096		/* タスクのスタックサイズ */
#endif /* STACK_SIZE */

#ifndef TEST_TIME_CP
#define TEST_TIME_CP	50000U		/* チェックポイント到達情報の出力時間 */
#endif /* TEST_TIME_CP */

#ifndef TEST_TIME_PROC
#define TEST_TIME_PROC	1000U		/* チェックポイントを通らない場合の時間 */
#endif /* TEST_TIME_PROC */

/*
 *  送受信するデータの定義
 */
#define	DATA0		((intptr_t) 0)
#define	DATA1		((intptr_t) 1)
#define	DATA2		((intptr_t) 2)
#define	DATA3		((intptr_t) 3)
#define	DATA4		((intptr_t) 4)
#define	DATA5		((intptr_t) 5)

/*
 *  関数のプロトタイプ宣言
 */
#ifndef TOPPERS_MACRO_ONLY

extern void	task1(intptr_t exinf);
extern void	task2(intptr_t exinf);
extern void	task3(intptr_t exinf);
extern void	alarm1_handler(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */
