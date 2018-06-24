/*
 *	$Id: prc_cfg1_out.h 2084 2011-05-12 08:03:41Z mit-kimai $
 */

/*
 *		cfg1_out.cをリンクするために必要なスタブの定義
 */

void sta_ker(void){}
STK_T *const	_kernel_istkpt = 0x00;


/*
 *  オフセットファイルを生成するための定義
 */
const uint8_t	MAGIC_1 = 0x12;
const uint16_t	MAGIC_2 = 0x1234;
const uint32_t	MAGIC_4 = 0x12345678;

const TCB	TCB_enatex = {
    { NULL, NULL },     /* タスクキュー */
	NULL,               /* 初期化ブロックへのポインタ */
	0,                  /* タスク状態（内部表現）*/
	0,                  /* 拡張サービスコールのネストレベル */
	0,                  /* ベース優先度（内部表現）*/
	0,                  /* 現在の優先度（内部表現）*/
	false,              /* 起動要求キューイング */
	false,              /* 起床要求キューイング */
	true,               /* タスク例外処理許可状態 */
	false,              /* 待ち禁止状態 */
	0,                  /* 保留例外要因 */
	NULL,               /* 待ち情報ブロックへのポインタ */
    { NULL, NULL },     /* ロックしているミューテックスのキュー */
	0,                  /* 残りプロセッサ時間 */
    { NULL, NULL },     /* タスクコンテキストブロック */
};

