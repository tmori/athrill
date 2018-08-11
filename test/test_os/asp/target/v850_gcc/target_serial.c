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
 *  Copyright (C) 2010,2013 by Meika Sugimoto
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

#include "kernel_impl.h"
#include "target_serial.h"

#define TNUM_SIOP	(8)

/*
 *  シリアルポートの初期化ブロック
 */
typedef struct sio_port_initialization_block {
	uint8_t		tx_intno;	/* 送信割込みの割込み番号 */
	uint8_t		rx_intno;	/* 受信割込みの割込み番号 */
} SIOPINIB;

/*
 *  シリアルポートの制御ブロック
 */
struct sio_port_control_block {
	const SIOPINIB	*p_siopinib;	/* 初期化ブロック */
	intptr_t	exinf;				/* 拡張情報 */
	bool_t		openflag;			/* オープン済みフラグ */
	int_t		port_id;			/* ポート番号(0〜) */
	bool_t		received;			/* 受信有無フラグ */
};

SIOPCB siopcb_table[TNUM_SIOP];

/*
 *  SIO ID から管理ブロックへの変換マクロ
 */

#define INDEX_SIO(sioid)	((uint_t)((sioid) - 1))
#define get_siopcb(sioid)	(&(siopcb_table[INDEX_SIO(sioid)]))

/*
 *  シリアルポートのハードウェア依存情報の定義
 */

#define SERIAL_CLKDIV			(0x01)
#define SERIAL_COMPAREVALUE		(0x82)

/* シリアル関連レジスタ */
#define UAnCTL0(x)  (UA0CTL0 + ((x) * 0x10))
#define UAnCTL1(x)  (UA0CTL1 + ((x) * 0x10))
#define UAnCTL2(x)  (UA0CTL2 + ((x) * 0x10))
#define UAnOPT0(x)  (UA0OPT0 + ((x) * 0x10))
#define UAnSTR(x)   (UA0STR  + ((x) * 0x10))
#define UAnRX(x)    (UA0RX   + ((x) * 0x10))
#define UAnTX(x)    (UA0TX   + ((x) * 0x10))
#define UAnRIC(x)	(UA0RIC  + ((x) * 0x10))
#define UAnTIC(x)	(UA0TIC  + ((x) * 0x10))

/*
 *	シリアル初期化ブロック
 */
const SIOPINIB siopinib_table[TNUM_SIOP] =
{
	{ 45u , 44u },		/* UARTD0 */
	{ 47u , 48u },		/* UARTD1 */
	{ 74u , 73u },		/* UARTD2 */
	{ 85u , 84u },		/* UARTD3 */
	{ 87u , 86u },		/* UARTD4 */
	{ 95u , 94u },		/* UARTD5 */
	{ 116u , 115u },	/* UARTD6 */
	{ 119u , 118u },	/* UARTD7 */
};


/*
 *  sio_initialize -- シリアルポートドライバの初期化
 */
void
sio_initialize(intptr_t exinf)
{
	uint_t	i;

	/*
	 *  シリアルI/Oポート管理ブロックの初期化
	 */
	for (i = 0; i < TNUM_SIOP; i++)
	{
		siopcb_table[i].p_siopinib = &(siopinib_table[i]);
		siopcb_table[i].openflag = false;
		siopcb_table[i].received = false;
	}
}

/*
 *  sio_opn_por -- ポートのオープン
 */
SIOPCB *
sio_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB	*siopcb = get_siopcb(siopid);

	if(siopcb->openflag == false)
	{
		siopcb->exinf = exinf;
		siopcb->openflag = true;
		siopcb->port_id = siopid - 1;

		/* UARTを有効にする */
		sil_wrb_mem((void *)UAnCTL0(siopcb->port_id), 0x80);	/* UART enable */

		/* ボーレート発生器の初期化 */
		sil_wrb_mem((void *)UAnCTL1(siopcb->port_id), SERIAL_CLKDIV);
		sil_wrb_mem((void *)UAnCTL2(siopcb->port_id), SERIAL_COMPAREVALUE);

		/* モード設定 */
		sil_wrb_mem((void *)UAnCTL0(siopcb->port_id),
			sil_reb_mem((void *)UAnCTL0(siopcb->port_id)) | 0x12);

		/* 送受信許可 */
		sil_wrb_mem((void *)UAnCTL0(siopcb->port_id),
			sil_reb_mem((void *)UAnCTL0(siopcb->port_id)) | 0x60);

		/* 送受信割込み有効 */
		(void)ena_int(siopcb->p_siopinib->tx_intno);
		(void)ena_int(siopcb->p_siopinib->rx_intno);
	}

	return siopcb;
}

/*
 *  sio_snd_chr -- 文字送信
 */
bool_t
sio_snd_chr(SIOPCB *siopcb, char chr)
{
	bool_t	result = false;

	/* 送信可能かチェック */
	if((sil_reb_mem((void *)UAnSTR(siopcb->port_id)) & 0x80) == 0)
	{
		sil_wrb_mem((void *)UAnTX(siopcb->port_id), chr);
		result = true;
	}

	return result;
}

/*
 *  sio_rcv_chr -- 文字受信
 */
int_t sio_rcv_chr(SIOPCB *siopcb)
{
	int_t chr = -1;

	if(siopcb->received == true)
	{
		chr = (int_t)sil_reb_mem((void *)UAnRX(siopcb->port_id));
		siopcb->port_id = false;
	}

	return chr;
}

/*
 *  sio_ena_cbr -- シリアル I/O からのコールバックの許可
 */
void
sio_ena_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
		case SIO_RDY_SND:
			ena_int(siopcb->p_siopinib->tx_intno);
			break;
		case SIO_RDY_RCV:
			ena_int(siopcb->p_siopinib->rx_intno);
			break;
		default:
			assert(1);
			break;
	}
}

/*
 *  sio_dis_cbr -- シリアル I/O からのコールバックの禁止
 */
void
sio_dis_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
		case SIO_RDY_SND:
			dis_int(siopcb->p_siopinib->tx_intno);
			break;
		case SIO_RDY_RCV:
			dis_int(siopcb->p_siopinib->rx_intno);
			break;
		default:
			assert(1);
			break;
	}
}

/*
 *  sio_cls_por -- ポートのクローズ
 */
void
sio_cls_por(SIOPCB *siopcb)
{
	/* UARTの停止 */
	sil_wrb_mem((void *)UAnCTL0(siopcb->port_id) ,
		(sil_reb_mem((void *)UAnCTL0(siopcb->port_id)) & ~0x80));

	/* 割込みの禁止 */
	dis_int(siopcb->p_siopinib->tx_intno);
	dis_int(siopcb->p_siopinib->rx_intno);

	/* フラグの設定 */
	siopcb->openflag = false;
	siopcb->received = false;
}

void
sio_tx_isr(intptr_t exinf)
{
	SIOPCB	*siopcb = get_siopcb(exinf);

	/* コールバックを呼び出し */
	sio_irdy_snd(siopcb->exinf);
}

void
sio_rx_isr(intptr_t exinf)
{
	SIOPCB	*siopcb = get_siopcb(exinf);

	/* 受信済みフラグをセット */
	siopcb->received = true;
	/* コールバックを呼び出し */
	sio_irdy_rcv(siopcb->exinf);
}

