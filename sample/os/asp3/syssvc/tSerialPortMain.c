/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2006-2015 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: tSerialPortMain.c 527 2016-01-14 11:43:45Z ertl-hiro $
 */

/*
 *		シリアルインタフェースドライバ
 */

#include <kernel.h>
#include "tSerialPortMain_tecsgen.h"
#include <t_syslog.h>

/*
 *  フロー制御に関連する定数とマクロ
 */
#define	FC_STOP			'\023'		/* コントロール-S */
#define	FC_START		'\021'		/* コントロール-Q */

#define BUFCNT_STOP(bufsz)		((bufsz) * 3 / 4)	/* STOPを送る基準文字数 */
#define BUFCNT_START(bufsz)		((bufsz) / 2)		/* STARTを送る基準文字数 */

/*
 *  ポインタのインクリメント
 */
#define INC_PTR(ptr, bufsz) do {	\
	if (++(ptr) == (bufsz)) {		\
		(ptr) = 0;					\
	 }								\
} while (false)

/*
 *  サービスコール呼出しマクロ
 *
 *  サービスコール呼出しを含む式expを評価し，返値がエラー（負の値）の場
 *  合には，ercにercd_expを評価した値を代入し，error_exitにgotoする．
 */
#define SVC(exp, ercd_exp) do {		\
	if ((exp) < 0) {				\
		ercd = (ercd_exp);			\
		goto error_exit;			\
	}								\
} while (false)

/*
 *  E_SYSエラーの生成
 */
static ER
gen_ercd_sys(CELLCB *p_cellcb)
{
	VAR_errorFlag = true;
	return(E_SYS);
}

/*
 *  待ちに入るサービスコールからのエラーの変換
 */
static ER
gen_ercd_wait(ER rercd, CELLCB *p_cellcb)
{
	switch (MERCD(rercd)) {
	case E_RLWAI:
	case E_DLT:
	case E_RASTER:
		return(rercd);
	default:
		VAR_errorFlag = true;
		return(E_SYS);
	}
}

/*
 *  シリアルポートのオープン（受け口関数）
 */
ER
eSerialPort_open(CELLIDX idx)
{
	CELLCB	*p_cellcb;
	ER		ercd;

	if (sns_dpn()) {				/* コンテキストのチェック */
		return(E_CTX);
	}
	if (!VALID_IDX(idx)) {
		return(E_ID);				/* ポート番号のチェック */
	}
	p_cellcb = GET_CELLCB(idx);

	SVC(dis_dsp(), gen_ercd_sys(p_cellcb));
	if (VAR_openFlag) {				/* オープン済みかのチェック */
		ercd = E_OBJ;
	}
	else {
		/*
		 *  変数の初期化
		 */
		VAR_ioControl = (IOCTL_ECHO | IOCTL_CRLF | IOCTL_FCSND | IOCTL_FCRCV);

		VAR_receiveReadPointer = 0U;
		VAR_receiveWritePointer = 0U;
		VAR_receiveCount = 0U;
		VAR_receiveFlowControl = '\0';
		VAR_receiveStopped = false;

		VAR_sendReadPointer = 0U;
		VAR_sendWritePointer = 0U;
		VAR_sendCount = 0U;
		VAR_sendStopped = false;

		/*
		 *  これ以降，割込みを禁止する．
		 */
		if (loc_cpu() < 0) {
			ercd = E_SYS;
			goto error_exit_enadsp;
		}

		/*
		 *  ハードウェア依存のオープン処理
		 */
		cSIOPort_open();

		/*
		 *  受信通知コールバックを許可する．
		 */
		cSIOPort_enableCBR(SIOReceiveReady);
		VAR_openFlag = true;
		VAR_errorFlag = false;

		if (unl_cpu() < 0) {
			VAR_errorFlag = true;
			ercd = E_SYS;
			goto error_exit_enadsp;
		}
		ercd = E_OK;
	}

  error_exit_enadsp:
	SVC(ena_dsp(), gen_ercd_sys(p_cellcb));

  error_exit:
	return(ercd);
}

/*
 *  シリアルポートのクローズ（受け口関数）
 */
ER
eSerialPort_close(CELLIDX idx)
{
	CELLCB	*p_cellcb;
	ER		ercd;
	bool_t	eflag = false;

	if (sns_dpn()) {				/* コンテキストのチェック */
		return(E_CTX);
	}
	if (!VALID_IDX(idx)) {
		return(E_ID);				/* ポート番号のチェック */
	}
	p_cellcb = GET_CELLCB(idx);

	SVC(dis_dsp(), gen_ercd_sys(p_cellcb));
	if (!VAR_openFlag) {			/* オープン済みかのチェック */
		ercd = E_OBJ;
	}
	else {
		/*
		 *  ハードウェア依存のクローズ処理
		 */
		if (loc_cpu() < 0) {
			eflag = true;
		}
		cSIOPort_close();
		VAR_openFlag = false;
		if (unl_cpu() < 0) {
			eflag = true;
		}

		/*
		 *  セマフォの初期化
		 */
		if (cSendSemaphore_initialize() < 0) {
			eflag = true;
		}
		if (cReceiveSemaphore_initialize() < 0) {
			eflag = true;
		}

		/*
		 *  エラーコードの設定
		 */
		if (eflag) {
			ercd = gen_ercd_sys(p_cellcb);
		}
		else {
			ercd = E_OK;
		}
	}
	SVC(ena_dsp(), gen_ercd_sys(p_cellcb));

  error_exit:
	return(ercd);
}

/*
 *  シリアルポートへの文字送信
 *
 *  p_cellcbで指定されるSIOポートに対して，文字cを送信する．文字を送信
 *  レジスタにいれた場合にはtrueを返す．そうでない場合には，送信レジス
 *  タが空いたことを通知するコールバック関数を許可し，falseを返す．この
 *  関数は，CPUロック状態で呼び出される．
 */
Inline bool_t
serialPort_sendChar(CELLCB *p_cellcb, char c)
{
	if (cSIOPort_putChar(c)) {
		return(true);
	}
	else {
		cSIOPort_enableCBR(SIOSendReady);
		return(false);
	}
}

/*
 *  シリアルポートへの1文字送信
 */
static ER_BOOL
serialPort_writeChar(CELLCB *p_cellcb, char c)
{
	bool_t	buffer_full;
	ER		ercd, rercd;

	/*
	 *  LFの前にCRを送信する．
	 */
	if (c == '\n' && (VAR_ioControl & IOCTL_CRLF) != 0U) {
		/*
		 *  以下のコードは再帰呼出しになっているが，引数cが'\n'の場合に
		 *  引数cを'\r'として呼び出すことから，この再帰呼出しは2回目の
		 *  呼び出しで必ず止まる．
		 */
		SVC(rercd = serialPort_writeChar(p_cellcb, '\r'), rercd);
		if ((bool_t) rercd) {
			SVC(rercd = cSendSemaphore_wait(),
										gen_ercd_wait(rercd, p_cellcb));
		}
	}

	SVC(loc_cpu(), gen_ercd_sys(p_cellcb));
	if (VAR_sendCount == 0U && !VAR_sendStopped
								&& serialPort_sendChar(p_cellcb, c)) {
		/*
		 *  SIOの送信レジスタに文字を入れることに成功した場合．
		 */
		buffer_full = false;
	}
	else {
		/*
		 *  送信バッファに文字を入れる．
		 */
		VAR_sendBuffer[VAR_sendWritePointer] = c;
		INC_PTR(VAR_sendWritePointer, ATTR_sendBufferSize);
		VAR_sendCount++;
		buffer_full = (VAR_sendCount == ATTR_sendBufferSize);
	}

	SVC(unl_cpu(), gen_ercd_sys(p_cellcb));
	ercd = (ER_BOOL) buffer_full;

  error_exit:
	return(ercd);
}

/*
 *  シリアルポートへの文字列送信（受け口関数）
 */
ER_UINT
eSerialPort_write(CELLIDX idx, const char *buffer, uint_t length)
{
	CELLCB	*p_cellcb;
	bool_t	buffer_full;
	uint_t	wricnt = 0U;
	ER		ercd, rercd;

	if (sns_dpn()) {				/* コンテキストのチェック */
		return(E_CTX);
	}
	if (!VALID_IDX(idx)) {			/* ポート番号のチェック */
		return(E_ID);
	}

	p_cellcb = GET_CELLCB(idx);
	if (!VAR_openFlag) {			/* オープン済みかのチェック */
		return(E_OBJ);
	}
	if (VAR_errorFlag) {			/* エラー状態かのチェック */
		return(E_SYS);
	}

	buffer_full = true;				/* ループの1回めはwai_semする */
	while (wricnt < length) {
		if (buffer_full) {
			SVC(rercd = cSendSemaphore_wait(),
										gen_ercd_wait(rercd, p_cellcb));
		}
		SVC(rercd = serialPort_writeChar(p_cellcb, *buffer++), rercd);
		wricnt++;
		buffer_full = (bool_t) rercd;
	}
	if (!buffer_full) {
		SVC(cSendSemaphore_signal(), gen_ercd_sys(p_cellcb));
	}
	ercd = E_OK;

  error_exit:
	return(wricnt > 0U ? (ER_UINT) wricnt : ercd);
}

/*
 *  シリアルポートからの1文字受信
 */
static bool_t
serialPort_readChar(CELLCB *p_cellcb, char *p_c)
{
	bool_t	buffer_empty;
	ER		ercd;

	SVC(loc_cpu(), gen_ercd_sys(p_cellcb));

	/*
	 *  受信バッファから文字を取り出す．
	 */
	*p_c = VAR_receiveBuffer[VAR_receiveReadPointer];
	INC_PTR(VAR_receiveReadPointer, ATTR_receiveBufferSize);
	VAR_receiveCount--;
	buffer_empty = (VAR_receiveCount == 0U);

	/*
	 *  STARTを送信する．
	 */
	if (VAR_receiveStopped && VAR_receiveCount
								<= BUFCNT_START(ATTR_receiveBufferSize)) {
		if (!serialPort_sendChar(p_cellcb, FC_START)) {
			VAR_receiveFlowControl = FC_START;
		}
		VAR_receiveStopped = false;
	}

	SVC(unl_cpu(), gen_ercd_sys(p_cellcb));
	ercd = (ER_BOOL) buffer_empty;

  error_exit:
	return(ercd);
}

/*
 *  シリアルポートからの文字列受信（受け口関数）
 */
ER_UINT
eSerialPort_read(CELLIDX idx, char *buffer, uint_t length)
{
	CELLCB	*p_cellcb;
	bool_t	buffer_empty;
	uint_t	reacnt = 0U;
	char	c = '\0';		/* コンパイラの警告を抑止するために初期化する */
	ER		ercd, rercd;

	if (sns_dpn()) {				/* コンテキストのチェック */
		return(E_CTX);
	}
	if (!VALID_IDX(idx)) {			/* ポート番号のチェック */
		return(E_ID);
	}

	p_cellcb = GET_CELLCB(idx);
	if (!VAR_openFlag) {			/* オープン済みかのチェック */
		return(E_OBJ);
	}
	if (VAR_errorFlag) {			/* エラー状態かのチェック */
		return(E_SYS);
	}

	buffer_empty = true;			/* ループの1回めはwai_semする */
	while (reacnt < length) {
		if (buffer_empty) {
			SVC(rercd = cReceiveSemaphore_wait(),
										gen_ercd_wait(rercd, p_cellcb));
		}
		SVC(rercd = serialPort_readChar(p_cellcb, &c), rercd);
		*buffer++ = c;
		reacnt++;
		buffer_empty = (bool_t) rercd;

		/*
		 *  エコーバック処理．
		 */
		if ((VAR_ioControl & IOCTL_ECHO) != 0U) {
			SVC(rercd = cSendSemaphore_wait(),
										gen_ercd_wait(rercd, p_cellcb));
			SVC(rercd = serialPort_writeChar(p_cellcb, c), rercd);
			if (!((bool_t) rercd)) {
				SVC(cSendSemaphore_signal(), gen_ercd_sys(p_cellcb));
			}
		}
	}
	if (!buffer_empty) {
		SVC(cReceiveSemaphore_signal(), gen_ercd_sys(p_cellcb));
	}
	ercd = E_OK;

  error_exit:
	return(reacnt > 0U ? (ER_UINT) reacnt : ercd);
}

/*
 *  シリアルポートの制御（受け口関数）
 */
ER
eSerialPort_control(CELLIDX idx, uint_t ioctl)
{
	CELLCB	*p_cellcb;

	if (sns_dpn()) {				/* コンテキストのチェック */
		return(E_CTX);
	}
	if (!VALID_IDX(idx)) {
		return(E_ID);				/* ポート番号のチェック */
	}

	p_cellcb = GET_CELLCB(idx);
	if (!VAR_openFlag) {			/* オープン済みかのチェック */
		return(E_OBJ);
	}
	if (VAR_errorFlag) {			/* エラー状態かのチェック */
		return(E_SYS);
	}

	VAR_ioControl = ioctl;
	return(E_OK);
}

/*
 *  シリアルポート状態の参照（受け口関数）
 */
ER
eSerialPort_refer(CELLIDX idx, T_SERIAL_RPOR* pk_rpor)
{
	CELLCB	*p_cellcb;

	if (sns_dpn()) {				/* コンテキストのチェック */
		return(E_CTX);
	}
	if (!VALID_IDX(idx)) {
		return(E_ID);				/* ポート番号のチェック */
	}

	p_cellcb = GET_CELLCB(idx);
	if (!VAR_openFlag) {			/* オープン済みかのチェック */
		return(E_OBJ);
	}
	if (VAR_errorFlag) {			/* エラー状態かのチェック */
		return(E_SYS);
	}

	pk_rpor->reacnt = VAR_receiveCount;
	pk_rpor->wricnt = VAR_sendCount;
	return(E_OK);
}

/*
 *  シリアルポートからの送信可能コールバック（受け口関数）
 */
void
eiSIOCBR_readySend(CELLIDX idx)
{
	CELLCB	*p_cellcb;

	assert(VALID_IDX(idx));
	p_cellcb = GET_CELLCB(idx);
	if (VAR_receiveFlowControl != '\0') {
		/*
		 *  START/STOP を送信する．
		 */
		(void) cSIOPort_putChar(VAR_receiveFlowControl);
		VAR_receiveFlowControl = '\0';
	}
	else if (!VAR_sendStopped && VAR_sendCount > 0U) {
		/*
		 *  送信バッファ中から文字を取り出して送信する．
		 */
		(void) cSIOPort_putChar(VAR_sendBuffer[VAR_sendReadPointer]);
		INC_PTR(VAR_sendReadPointer, ATTR_sendBufferSize);
		if (VAR_sendCount == ATTR_sendBufferSize) {
			if (ciSendSemaphore_signal() < 0) {
				VAR_errorFlag = true;
			}
		}
		VAR_sendCount--;
	}
	else {
		/*
		 *  送信すべき文字がない場合は，送信可能コールバックを禁止する．
		 */
		cSIOPort_disableCBR(SIOSendReady);
	}
}

/*
 *  シリアルポートからの受信通知コールバック（受け口関数）
 */
void
eiSIOCBR_readyReceive(CELLIDX idx)
{
	CELLCB	*p_cellcb;
	char	c;

	assert(VALID_IDX(idx));
	p_cellcb = GET_CELLCB(idx);
	c = (char) cSIOPort_getChar();
	if ((VAR_ioControl & IOCTL_FCSND) != 0U && c == FC_STOP) {
		/*
		 *  送信を一時停止する．送信中の文字はそのまま送信する．
		 */
		VAR_sendStopped = true;
	}
	else if (VAR_sendStopped && (c == FC_START
				|| (VAR_ioControl & IOCTL_FCANY) != 0U)) {
		/*
		 *  送信を再開する．
		 */
		VAR_sendStopped = false;
		if (VAR_sendCount > 0U) {
			c = VAR_sendBuffer[VAR_sendReadPointer];
			if (serialPort_sendChar(p_cellcb, c)) {
				INC_PTR(VAR_sendReadPointer, ATTR_sendBufferSize);
				if (VAR_sendCount == ATTR_sendBufferSize) {
					if (ciSendSemaphore_signal() < 0) {
						VAR_errorFlag = true;
					}
				}
				VAR_sendCount--;
			}
		}
	}
	else if ((VAR_ioControl & IOCTL_FCSND) != 0U && c == FC_START) {
		/*
		 *  送信に対してフロー制御している場合，START は捨てる．
		 */
	}
	else if (VAR_receiveCount == ATTR_receiveBufferSize) {
		/*
		 *  バッファフルの場合，受信した文字を捨てる．
		 */
	}
	else {
		/*
		 *  受信した文字を受信バッファに入れる．
		 */
		VAR_receiveBuffer[VAR_receiveWritePointer] = c;
		INC_PTR(VAR_receiveWritePointer, ATTR_receiveBufferSize);
		if (VAR_receiveCount == 0U) {
			if (ciReceiveSemaphore_signal() < 0) {
				VAR_errorFlag = true;
			}
		}
		VAR_receiveCount++;

		/*
		 *  STOPを送信する．
		 */
		if ((VAR_ioControl & IOCTL_FCRCV) != 0U && !VAR_receiveStopped
				&& VAR_receiveCount >= BUFCNT_STOP(ATTR_receiveBufferSize)) {
			if (!serialPort_sendChar(p_cellcb, FC_STOP)) {
				VAR_receiveFlowControl = FC_STOP;
			}
			VAR_receiveStopped = true;
		}
	}
}

/*
 *  シリアルインタフェースドライバからの未送信文字の取出し
 */
bool_t
enSerialPortManage_getChar(CELLIDX idx, char *p_c)
{
	CELLCB	*p_cellcb;

	if (VALID_IDX(idx)) {						/* ポート番号のチェック */
		p_cellcb = GET_CELLCB(idx);
		if (VAR_openFlag) {						/* オープン済みかのチェック */
			if (VAR_sendCount > 0U) {
				*p_c = VAR_sendBuffer[VAR_sendReadPointer];
				INC_PTR(VAR_sendReadPointer, ATTR_sendBufferSize);
				VAR_sendCount--;
				return(true);
			}
		}
	}
	return(false);
}
