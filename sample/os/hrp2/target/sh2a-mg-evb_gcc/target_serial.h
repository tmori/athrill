/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2011 by Embedded and Real-Time Systems Laboratory
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
 */

/*
 *	シリアルI/Oデバイス（SIO）ドライバ（SH2A-MG-EVB用）
 */

#ifndef TOPPERS_TARGET_SERIAL_H
#define TOPPERS_TARGET_SERIAL_H

/*
 *  シリアルI/Oポート初期化ブロックの定義
 *  レジスタの番地と割込み番号
 */
typedef struct sio_port_initialization_block {
    void *sccr;     /* 制御レジスタ */
    void *scmd;     /* モードレジスタ */
    void *scemd;    /* 拡張モードレジスタ */
    void *scbr;     /* ビットレートレジスタ */
    void *sctb;     /* 送信バッファレジスタ */
    void *scrb;     /* 受信バッファレジスタ */
    void *scsr;     /* ステータスレジスタ */
    void *rxdiocr;  /* RXD端子のポート制御レジスタ */
    void *txdiocr;  /* TXD端子のポート制御レジスタ */
    void *sckiocr;  /* SCK端子のポート制御レジスタ */
    void *rxdioin;  /* RXD端子のポート入力レジスタ */
    uint8_t rxpio;  /* RXDに対応するPIOの番号：A/Bの後の数字 */
    INTNO intno_rx; /* 受信バッファフルの割込み番号 */
    INTNO intno_tx; /* 送信バッファエンプティの割込み番号 */
} SIOPINIB;

/*
 *  シリアルI/Oポート管理ブロックの定義
 */
typedef struct sio_port_control_block{
    const SIOPINIB *p_siopinib; /* シリアルI/Oポート初期化ブロック */
    intptr_t exinf;             /* 拡張情報，すなわち，ターゲット非依存部のシリアルポート管理ブロック（SPCB）へのポインタ */
    bool_t initialized_flag;    /* ハードウェアの初期化をしたか？ */
}
SIOPCB;

#define TNUM_SCI_PORT 4 /* SCIのチャネル数 */

/*
 *  レジスタ関連
 */

/*
 *  各チャネルのベースアドレス
 */
#define SCI0_BASE 0xff610000
#define SCI1_BASE 0xff610010
#define SCI2_BASE 0xff610020
#define SCI3_BASE 0xff610030

/*
 *  各レジスタのオフセットと設定値
 */
#define OFFSET_SCI_CR 0x00000002 /* 制御レジスタ */
#define SCI_CR_TIE  0x80 /* 送信バッファエンプティ割込み許可ビット */
#define SCI_CR_RIE  0x40 /* 受信バッファフル割込み許可ビット */
#define SCI_CR_TE   0x20 /* 送信許可ビット */
#define SCI_CR_RE   0x10 /* 受信許可ビット */
#define SCI_CR_TEIE 0x04 /* 送信完了割込み許可ビット */
#define SCI_CR_CKS  0x00 /* クロック選択ビット：内蔵ボーレートジェネレータ使用で，SCK端子はPIO */
#define SCI_CR_CKS_MASK  0x03 /* クロック選択ビットのマスク */

#define OFFSET_SCI_MD 0x00000000 /* モードレジスタ */
#define SCI_MD_SMS   0x80 /* クロック同期式：このビットが0である場合は，調歩同期式 */
#define SCI_MD_SDLS  0x40 /* データ長7ビット：このビットが0である場合は，8ビット */
#define SCI_MD_PE    0x20 /* パリティあり：このビットが0である場合は，パリティなし */
#define SCI_MD_OES   0x10 /* 奇パリティ：このビットが0である場合は，偶パリティ */
#define SCI_MD_TSTLS 0x08 /* 2ストップビット：このビットが0である場合は，1ストップビット */
#define SCI_MD_SCSS_DIV0  0x00 /* カウントソース選択ビット：周辺機能クロックAの分周なし */
#define SCI_MD_SCSS_DIV4  0x01 /* カウントソース選択ビット：周辺機能クロックAの4分周 */
#define SCI_MD_SCSS_DIV16 0x02 /* カウントソース選択ビット：周辺機能クロックAの16分周 */
#define SCI_MD_SCSS_DIV64 0x03 /* カウントソース選択ビット：周辺機能クロックAの64分周 */
#define SCI_MD_SCSS_MASK 0x03 /* カウントソース選択ビットのマスク */

#define OFFSET_SCI_EMD 0x00000008 /* 拡張モードレジスタ */
#define SCI_EMD_SDIR  0x08 /* MSBファースト：このビットが0である場合は，LSBファースト */
#define SCI_EMD_CKPOS 0x02 /* 極性反転する：このビットが0である場合は，しない */
#define SCI_EMD_CKPHS 0x01 /* 位相を半相遅らせる：このビットが0である場合は，遅らせない */

#define OFFSET_SCI_BR 0x00000001 /* ビットレートレジスタ */
#define OFFSET_SCI_TB 0x00000003 /* 送信バッファレジスタ */
#define OFFSET_SCI_RB 0x00000005 /* 受信バッファレジスタ */

#define OFFSET_SCI_SR 0x00000004 /* ステータスレジスタ */
#define SCI_SR_TBEF  0x80 /* 送信バッファエンプティフラグ */
#define SCI_SR_RBFF  0x40 /* 受信バッファフルフラグ */
#define SCI_SR_OREF  0x20 /* オーバランエラーフラグ */
#define SCI_SR_FREF  0x10 /* フレーミングエラーフラグ */
#define SCI_SR_PERF  0x08 /* パリティエラーフラグ */
#define SCI_SR_TSEF  0x04 /* 送信シフトレジスタエンプティフラグ */

/*
 *  コールバックルーチンの識別番号
 */
#define SIO_RDY_SND    1U        /* 送信可能コールバック */
#define SIO_RDY_RCV    2U        /* 受信通知コールバック */

/*
 *	SIOの割込みハンドラのベクタ番号
 */
#define INTNO_SCI0_RXE 312 /* SCI0受信エラー割込み */
#define INTNO_SCI0_RXF 313 /* SCI0受信バッファフル割込み */
#define INTNO_SCI0_TXE 314 /* SCI0送信バッファエンプティ割込み */
#define INTNO_SCI0_TXC 315 /* SCI0送信完了割込み */
#define INTNO_SCI1_RXE 316 /* SCI1受信エラー割込み */
#define INTNO_SCI1_RXF 317 /* SCI1受信バッファフル割込み */
#define INTNO_SCI1_TXE 318 /* SCI1送信バッファエンプティ割込み */
#define INTNO_SCI1_TXC 319 /* SCI1送信完了割込み */
#define INTNO_SCI2_RXE 320 /* SCI2受信エラー割込み */
#define INTNO_SCI2_RXF 321 /* SCI2受信バッファフル割込み */
#define INTNO_SCI2_TXE 322 /* SCI2送信バッファエンプティ割込み */
#define INTNO_SCI2_TXC 323 /* SCI2送信完了割込み */
#define INTNO_SCI3_RXE 324 /* SCI3受信エラー割込み */
#define INTNO_SCI3_RXF 325 /* SCI3受信バッファフル割込み */
#define INTNO_SCI3_TXE 326 /* SCI3送信バッファエンプティ割込み */
#define INTNO_SCI3_TXC 327 /* SCI3送信完了割込み */

/*
 *  デフォルトで使用するシリアルポートはチャネル0
 *  送信コールバックを呼ぶための割込み要因は，送信完了割込みであるが，
 *  これは，送信完了割込みがレベルトリガであり，送信バッファエンプティ
 *  割込みがエッジトリガであることに起因する．
 *  エッジトリガの場合，送信に失敗してから割込みを許可するまでの間に
 *  送信バッファが空になってしまうと，割込みを許可しても，それ以降に
 *  割込みが発生せず，以降の送信が行われなくなることが起こりうる．
 */
#define INHNO_SIO_TX	 INTNO_SCI0_TXC /* 送信割込みハンドラ番号 */
#define INTNO_SIO_TX	 INTNO_SCI0_TXC /* 送信割込み番号 */
#define INHNO_SIO_RX	 INTNO_SCI0_RXF /* 受信割込みハンドラ番号 */
#define INTNO_SIO_RX	 INTNO_SCI0_RXF /* 受信割込み番号 */
#define INTPRI_SIO		 (-4)			  /* 割込み優先度 */
#define INTATR_SIO		 0U 			  /* 割込み属性 */

/*
 *  PIO関連のレジスタ
 */
#define PIO_PPR 0xff464030 /* ポートプロテクトレジスタ */
#define PIO_PPR_UNPROTECT 0xf1 /* プロテクト解除：この値以外を書き込むと，プロテクト */

#define PIO_PA00CR 0xff464040 /* ポートA00制御レジスタ */
#define PIO_PA01CR 0xff464042 /* ポートA01制御レジスタ */
#define PIO_PA02CR 0xff464044 /* ポートA02制御レジスタ */
#define PIO_PA03CR 0xff464046 /* ポートA03制御レジスタ */
#define PIO_PA04CR 0xff464048 /* ポートA04制御レジスタ */
#define PIO_PA05CR 0xff46404A /* ポートA05制御レジスタ */
#define PIO_PA06CR 0xff46404C /* ポートA06制御レジスタ */
#define PIO_PA07CR 0xff46404E /* ポートA07制御レジスタ */
#define PIO_PA08CR 0xff464050 /* ポートA08制御レジスタ */
#define PIO_PA09CR 0xff464052 /* ポートA09制御レジスタ */
#define PIO_PA10CR 0xff464054 /* ポートA10制御レジスタ */
#define PIO_PA11CR 0xff464056 /* ポートA11制御レジスタ */
#define PIO_PA12CR 0xff464058 /* ポートA12制御レジスタ */
#define PIO_PA13CR 0xff46405A /* ポートA13制御レジスタ */
#define PIO_PA14CR 0xff46405C /* ポートA14制御レジスタ */
#define PIO_PA15CR 0xff46405E /* ポートA15制御レジスタ */

#define PIO_PB00CR 0xff464080 /* ポートB00制御レジスタ */
#define PIO_PB01CR 0xff464082 /* ポートB01制御レジスタ */
#define PIO_PB02CR 0xff464084 /* ポートB02制御レジスタ */
#define PIO_PB03CR 0xff464086 /* ポートB03制御レジスタ */
#define PIO_PB04CR 0xff464088 /* ポートB04制御レジスタ */
#define PIO_PB05CR 0xff46408A /* ポートB05制御レジスタ */
#define PIO_PB06CR 0xff46408C /* ポートB06制御レジスタ */
#define PIO_PB07CR 0xff46408E /* ポートB07制御レジスタ */
#define PIO_PB08CR 0xff464090 /* ポートB08制御レジスタ */
#define PIO_PB09CR 0xff464092 /* ポートB09制御レジスタ */

#define PIO_CR_INE  0x8000 /* 入力許可フラグ */
#define PIO_CR_OUTE 0x0010 /* 出力許可フラグ */
#define PIO_CR_PSEL_IO  0x0000 /* ポート機能選択：PIO or 周辺機能入力 */
#define PIO_CR_PSEL_SCI 0x0003 /* ポート機能選択：SCI出力 */
#define PIO_CR_PSEL_MASK 0x000f /* ポート機能選択のマスク */

#define PIO_PAIN 0xff464062 /* ポートA入力レジスタ */
#define PIO_PBIN 0xff4640A2 /* ポートB入力レジスタ */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  ビットレートから，BRに設定する値を計算するマクロ
 *  f(PBA) / (64 * 2^(2m-1) * br) - 1：m = MD_SCSS
 */
#define CONVERT_BITRATE(br) (uint8_t)(((40000000 * 2) / (64 * 1 * br)) - 1)

extern SIOPCB siopcb_table[TNUM_SCI_PORT];
extern const SIOPINIB siopinib_table[TNUM_SCI_PORT];

/*
 *	SIOドライバの初期化
 */
extern void sio_initialize(intptr_t exinf);
extern void sio_hardware_initialize(const SIOPINIB *p_siopinib);

/*
 *	シリアルI/Oポートのオープン
 */
extern SIOPCB *sio_opn_por(ID siopid, intptr_t exinf);

/*
 *	シリアルI/Oポートのクローズ
 */
extern void sio_cls_por(SIOPCB *p_siopcb);

/*
 *	SIOの割込みハンドラ
 */
extern void sio_tx_isr(intptr_t exinf);
extern void sio_rx_isr(intptr_t exinf);

/*
 *	シリアルI/Oポートへの文字送信
 */
extern bool_t sio_snd_chr(SIOPCB *siopcb, char c);

/*
 *	シリアルI/Oポートからの文字受信
 */
extern int_t sio_rcv_chr(SIOPCB *siopcb);

/*
 *	シリアルI/Oポートからのコールバックの許可
 */
extern void sio_ena_cbr(SIOPCB *siopcb, uint_t cbrtn);

/*
 *	シリアルI/Oポートからのコールバックの禁止
 */
extern void sio_dis_cbr(SIOPCB *siopcb, uint_t cbrtn);

/*
 *	シリアルI/Oポートからの送信可能コールバック
 */
extern void sio_irdy_snd(intptr_t exinf);

/*
 *	シリアルI/Oポートからの受信通知コールバック
 */
extern void sio_irdy_rcv(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_TARGET_SERIAL_H */
