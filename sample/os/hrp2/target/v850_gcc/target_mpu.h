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
 *	MPUドライバ（SH72AW用）
 */

#ifndef TOPPERS_TARGET_MPU_H
#define TOPPERS_TARGET_MPU_H

#define MPCMPEN    0xfff78000 /* MPU有効 */
#define MPCRACR    0xfff78010 /* Rアクセス制御 */
#define MPCWACR    0xfff78014 /* Wアクセス制御 */
#define MPCIACR    0xfff78018 /* I(X)アクセス制御 */
#define MPCVLD     0xfff7801c /* 領域設定有効 */
#define MPCACBCR   0xfff78020 /* バッググランド領域のアクセス権 */
#define MPCECLR    0xfff78024 /* エラークリア */
#define MPCESR     0xfff78028 /* エラーステータス */
#define MPCHITI    0xfff78034 /* 命令ヒット */
#define MPCHITO    0xfff78038 /* データヒット，領域サーチヒット */
#define MPCHITO_R_MASK 0x00000008 /* R許可フラグ */
#define MPCHITO_W_MASK 0x00000004 /* W許可フラグ */
#define MPCHITO_I_MASK 0x00000002 /* I許可フラグ */
#define MPCRSADR   0xfff7803c /* 領域サーチアドレス */
#define MPCRSOP    0xfff78040 /* 領域サーチ開始 */
#define MPCRSOP_S_BIT    0x01 /* 領域サーチ開始ビット */   
#define MPCERADRI  0xfff7802c /* 命令アクセスエラーアドレス */
#define MPCERADRO  0xfff78030 /* データアクセスエラーアドレス */

#define MPCSADR0   0xfff78100 /* 領域0 開始アドレス */
#define MPCSADR1   0xfff78110 /* 領域1 開始アドレス */
#define MPCSADR2   0xfff78120 /* 領域2 開始アドレス */
#define MPCSADR3   0xfff78130 /* 領域3 開始アドレス */
#define MPCSADR4   0xfff78140 /* 領域4 開始アドレス */
#define MPCSADR5   0xfff78150 /* 領域5 開始アドレス */
#define MPCSADR6   0xfff78160 /* 領域6 開始アドレス */
#define MPCSADR7   0xfff78170 /* 領域7 開始アドレス */
#define MPCSADR8   0xfff78180 /* 領域8 開始アドレス */
#define MPCSADR9   0xfff78190 /* 領域9 開始アドレス */
#define MPCSADR10  0xfff781a0 /* 領域10開始アドレス */
#define MPCSADR11  0xfff781b0 /* 領域11開始アドレス */
#define MPCSADR12  0xfff781c0 /* 領域12開始アドレス */
#define MPCSADR13  0xfff781d0 /* 領域13開始アドレス */
#define MPCSADR14  0xfff781e0 /* 領域14開始アドレス */
#define MPCSADR15  0xfff781f0 /* 領域15開始アドレス */

#define MPCEADR0   0xfff78104 /* 領域0 終了アドレス */
#define MPCEADR1   0xfff78114 /* 領域1 終了アドレス */
#define MPCEADR2   0xfff78124 /* 領域2 終了アドレス */
#define MPCEADR3   0xfff78134 /* 領域3 終了アドレス */
#define MPCEADR4   0xfff78144 /* 領域4 終了アドレス */
#define MPCEADR5   0xfff78154 /* 領域5 終了アドレス */
#define MPCEADR6   0xfff78164 /* 領域6 終了アドレス */
#define MPCEADR7   0xfff78174 /* 領域7 終了アドレス */
#define MPCEADR8   0xfff78184 /* 領域8 終了アドレス */
#define MPCEADR9   0xfff78194 /* 領域9 終了アドレス */
#define MPCEADR10  0xfff781a4 /* 領域10終了アドレス */
#define MPCEADR11  0xfff781b4 /* 領域11終了アドレス */
#define MPCEADR12  0xfff781c4 /* 領域12終了アドレス */
#define MPCEADR13  0xfff781d4 /* 領域13終了アドレス */
#define MPCEADR14  0xfff781e4 /* 領域14終了アドレス */
#define MPCEADR15  0xfff781f4 /* 領域15終了アドレス */

#define MPCACR0    0xfff78108 /* 領域0 アクセス制御 */
#define MPCACR1    0xfff78118 /* 領域1 アクセス制御 */
#define MPCACR2    0xfff78128 /* 領域2 アクセス制御 */
#define MPCACR3    0xfff78138 /* 領域3 アクセス制御 */
#define MPCACR4    0xfff78148 /* 領域4 アクセス制御 */
#define MPCACR5    0xfff78158 /* 領域5 アクセス制御 */
#define MPCACR6    0xfff78168 /* 領域6 アクセス制御 */
#define MPCACR7    0xfff78178 /* 領域7 アクセス制御 */
#define MPCACR8    0xfff78188 /* 領域8 アクセス制御 */
#define MPCACR9    0xfff78198 /* 領域9 アクセス制御 */
#define MPCACR10   0xfff781a8 /* 領域10アクセス制御 */
#define MPCACR11   0xfff781b8 /* 領域11アクセス制御 */
#define MPCACR12   0xfff781c8 /* 領域12アクセス制御 */
#define MPCACR13   0xfff781d8 /* 領域13アクセス制御 */
#define MPCACR14   0xfff781e8 /* 領域14アクセス制御 */
#define MPCACR15   0xfff781f8 /* 領域15アクセス制御 */

#define PAGE_SIZE 4
#define MPCADR_mpcsadr0     0x00
#define MPCADR_mpceadr0     0x04
#define MPCADR_mpcsadr1     0x10
#define MPCADR_mpceadr1     0x14
#define MPCADR_mpcsadr2     0x20
#define MPCADR_mpceadr2     0x24
#define MPCADR_mpcsadr3     0x30
#define MPCADR_mpceadr3     0x34
#define MPCADR_mpcsadr4     0x40
#define MPCADR_mpceadr4     0x44
#define MPCADR_mpcsadr5     0x50
#define MPCADR_mpceadr5     0x54
#define MPCADR_mpcsadr6     0x60
#define MPCADR_mpceadr6     0x64
#define MPCADR_mpcsadr7     0x70
#define MPCADR_mpceadr7     0x74
#define MPCADR_mpcsadr8     0x80
#define MPCADR_mpceadr8     0x84
#define MPCADR_mpcsadr9     0x90
#define MPCADR_mpceadr9     0x94
#define MPCADR_mpcsadr10    0xa0
#define MPCADR_mpceadr10    0xa4
#define MPCADR_mpcsadr11    0xb0
#define MPCADR_mpceadr11    0xb4
#define MPCADR_mpcsadr12    0xc0
#define MPCADR_mpceadr12    0xc4
#define MPCADR_mpcsadr13    0xd0
#define MPCADR_mpceadr13    0xd4
#define MPCADR_mpcsadr14    0xe0
#define MPCADR_mpceadr14    0xe4
#define MPCADR_mpcsadr15    0xf0
#define MPCADR_mpceadr15    0xf4

#define OMIT_PROBE_MEM_WRITE
#define OMIT_PROBE_MEM_READ
#define OMIT_PROBE_STACK

#ifndef TOPPERS_MACRO_ONLY

extern void target_mpu_initialize(void);
extern void target_mpu_exc_handler(void *p_excinf);
// タスク例外実行開始時スタック不正ハンドラ
extern void target_emulate_texrtn_handler(void *p_excinf);
// タスク例外リターン時スタック不正ハンドラ
extern void target_emulate_ret_tex_handler(void *p_excinf);

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_TARGET_MPU_H */
