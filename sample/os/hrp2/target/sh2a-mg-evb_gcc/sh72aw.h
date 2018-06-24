/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2011-2012 by Embedded and Real-Time Systems Laboratory
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
 *	SH72AW CPUボードのハードウェア資源の定義
 */

#ifndef TOPPERS_SH72AW_H
#define TOPPERS_SH72AW_H

/*
 *  クロック関連のレジスタ
 *  クロックのソースは、メインクロック（20MHz） or PLL（120MHz、160MHz）
 *  これを分周して、CPUのクロックとする（SCDR）
 *  さらに分周して、バスクロック（BCDR）、周辺機能クロックA・Bとする（PBCDR）
 */
#define SPR0 0xff46e063 /* システムプロテクトレジスタ */
#define SPR0_CANCEL_PROTECT 0xf1 /* プロテクト解除 */
#define SPR0_PROTECT 0x00 /* プロテクト(0xF1以外を書く) */

#define MCMCR 0xff46e838 /* メインクロック監視機能制御レジスタ */
#define MCMCR_LXDF_MASK 0x04 /* メインクロックの発信停止を検出したか？ */
#define MCMCR_LXIE 0x01 /* メインクロック発信停止検出割込みを許可 */

#define PLCR 0xff46e828 /* PLL制御レジスタ */
#define PLCR_VCOD 0x0001 /* PLLの出力：120MHz（このビットを0にすると、160MHz）*/

#define SCSR 0xff46e802 /* システムクロック選択レジスタ */
#define SCSR_SCS_PLL 0x01 /* PLLを選択 */
#define SCSR_SCS_XIN 0x00 /* メインクロックを選択 */

#define SCDR 0xff46e804 /* システムクロック分周レジスタ */
#define SCDR_SYSD_DIV0 0x00 /* 分周なし */
#define SCDR_SYSD_DIV2 0x01 /* 2分周 */
#define SCDR_SYSD_DIV4 0x02 /* 4分周 */
#define SCDR_SYSD_DIV8 0x03 /* 8分周 */
#define SCDR_SYSD_DIV16 0x04 /* 16分周 */

#define BCDR 0xff46e805 /* バスクロック */
#define BCDR_BUSD_DIV2 0x10 /* 2分周 */
#define BCDR_BUSD_DIV4 0x20 /* 4分周 */

#define PBCDR 0xff46e808 /* 周辺機能クロック分周レジスタ */
#define PBCDR_PBAD_DIV2 0x10000000 /* クロックA：2分周 */
#define PBCDR_PBAD_DIV4 0x20000000 /* クロックA：4分周 */
#define PBCDR_PBBD_DIV0 0x00000000 /* クロックB：分周なし */
#define PBCDR_PBBD_DIV2 0x00000100 /* クロックB：2分周 */

#define RDCCR 0xff46e80c /* RDCクロック制御レジスタ：f(PBA)の分周 */
#define RDCCR_RDCD_DIV0 0x00 /* 分周なし */
#define RDCCR_RDCD_DIV2 0x01 /* 2分周 */

#define COCR 0xff46e800 /* クロック出力機能制御レジスタ：f(BUS)の分周 */
#define COCR_COD_DIV0 0x00 /* 分周なし */
#define COCR_COD_DIV2 0x10 /* 2分周 */
#define COCR_COD_DIV4 0x20 /* 4分周 */
#define COCR_COD_DIV8 0x30 /* 8分周 */
#define COCR_COD_DIV16 0x40 /* 16分周 */
#define COCR_COE 0x01 /* f(CLKOUT)を出力する（このビットを0にすると出力しない）*/

#define MSTPCR 0xff46e850 /* モジュールストップ制御レジスタ */
#define MSTPCR_FPUSTP 0x0001 /* FPUを停止（再び動作させるためにはリセットする）*/

#endif /* TOPPERS_SH72AW_H */
