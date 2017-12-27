/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2014 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2014 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2014 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2014 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2014 by Witz Corporation, JAPAN
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
 *  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
 *  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
 *  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
 *  用する者に対して，AUTOSARパートナーになることを求めている．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: prc_config.c 187 2015-06-25 03:39:04Z t_ishikawa $
 */

/*
 *		プロセッサ依存モジュール（V850用）
 */
#include "kernel_impl.h"

/*
 *  例外（割込み/CPU例外）のネスト回数のカウント
 *  コンテキスト参照のために使用
 */
uint32			except_nest_cnt;

#ifdef CFG_USE_PROTECTIONHOOK
uint32			v850_cpu_exp_no;
uint32			v850_cpu_exp_pc;
uint32			v850_cpu_exp_sp;
#endif /* CFG_USE_PROTECTIONHOOK */

/*
 * 現在の割込み優先度マスクの値（内部表現）
 */
uint8			current_iintpri;

/* 割り込み要求マスクテーブル */
const uint16	pmr_setting_tbl[] =
{
#if TNUM_INTPRI == 16
	0xffff, /* ISR2 LEVEL -16 MASK */
	0xfffe, /* ISR2 LEVEL -15 MASK */
	0xfffc, /* ISR2 LEVEL -14 MASK */
	0xfff8, /* ISR2 LEVEL -13 MASK */
	0xfff0, /* ISR2 LEVEL -12 MASK */
	0xffe0, /* ISR2 LEVEL -11 MASK */
	0xffc0, /* ISR2 LEVEL -10 MASK */
	0xff80, /* ISR2 LEVEL -9 MASK */
	0xff00, /* ISR2 LEVEL -8 MASK */
	0xfe00, /* ISR2 LEVEL -7 MASK */
	0xfc00, /* ISR2 LEVEL -6 MASK */
	0xf800, /* ISR2 LEVEL -5 MASK */
	0xf000, /* ISR2 LEVEL -4 MASK */
	0xe000, /* ISR2 LEVEL -3 MASK */
	0xc000, /* ISR2 LEVEL -2 MASK */
	0x8000, /* ISR2 LEVEL -1 MASK */
	0x0000  /* Dummy			  */
#elif TNUM_INTPRI == 8
	0x00ff, /* ISR2 LEVEL -8 MASK */
	0x00fe, /* ISR2 LEVEL -7 MASK */
	0x00fc, /* ISR2 LEVEL -6 MASK */
	0x00f8, /* ISR2 LEVEL -5 MASK */
	0x00f0, /* ISR2 LEVEL -4 MASK */
	0x00e0, /* ISR2 LEVEL -3 MASK */
	0x00c0, /* ISR2 LEVEL -2 MASK */
	0x0080, /* ISR2 LEVEL -1 MASK */
	0x0000  /* Dummy			  */
#else
#error TNUM_INTPRI is illegal.
#endif /* TNUM_INTPRI == 16 */
};

/*
 *  x_nested_lock_os_int()のネスト回数
 */
volatile uint8	nested_lock_os_int_cnt;

/*
 *  OS割込み禁止状態の時に割込み優先度マスクを保存する変数
 */
volatile uint16	current_intpri;

/*
 *  起動時のハードウェア初期化
 */
void
prc_hardware_initialize(void)
{
}

#ifdef __v850e3v5__
extern const uint32 intbp_tbl[];
#endif /* __v850e3v5__ */

/*
 *  プロセッサ依存の初期化
 */
void
prc_initialize(void)
{
	/*
	 *  カーネル起動時は非タスクコンテキストとして動作させるため1に
	 */
	except_nest_cnt = 1U;

	/*
	 * 割込み優先度マスクの初期値は最低優先度
	 */
	current_iintpri = INT_IPM(0);

    /* =begin added for SC3 */
    target_mpu_initialize();
    /* =end added for SC3 */
#ifdef __v850e3v5__
	/* テーブル参照方式のINTBPアドレス設定 */
	set_intbp((uint32)intbp_tbl);
#endif /* __v850e3v5__ */
}

/*
 *  プロセッサ依存の終了処理
 */
void
prc_terminate(void)
{
	/* 割り込み処理中で終了した場合を想定してISPRをクリアする */
	clear_ispr();

	/* 例外処理から終了した場合を想定してNP/EPビットをクリアする */
	set_psw(current_psw() & ~(0x0080 | 0x0040));
}

/*
 *  割込み要求ラインの属性の設定
 */
void
x_config_int(InterruptNumberType intno, AttributeType intatr, PriorityType intpri)
{
	uint32 eic_address;

	ASSERT(VALID_INTNO(intno));

	eic_address = EIC_ADDRESS(intno);

	/*
	 *  割込みのマスク
	 *
	 *  割込みを受け付けたまま，レベルトリガ／エッジトリガの設定や，割
	 *  込み優先度の設定を行うのは危険なため，割込み属性にかかわらず，
	 *  一旦マスクする．
	 */
	(void) x_disable_int(intno);

	/*
	 *  割込み優先度の設定
	 */
	sil_wrh_mem((void *) eic_address,
				((sil_reh_mem((void *) eic_address) & ~0x0f) | INT_IPM(intpri)));

#ifdef __v850e3v5__
	/* テーブル参照方式 */
	sil_wrh_mem((uint16 *) eic_address,
				(sil_reh_mem((uint16 *) eic_address) | (1 << 6)));
#endif /* __v850e3v5__ */

    if ((intatr & ENABLE) != 0U) {
        /*
         *  初期状態が割込み許可であれば，割込みを許可
         */
        (void) x_enable_int(intno);
    }
}

#ifndef OMIT_DEFAULT_INT_HANDLER

/*
 *  未定義の割込みが入った場合の処理
 */
void
default_int_handler(void)
{
	target_fput_str("Unregistered Interrupt occurs.");
	ASSERT_NO_REACHED;
}

#endif /* OMIT_DEFAULT_INT_HANDLER */

/*
 *  無限ループ処理
 *  (デバッグ時の無限ループ到達確認用 / 最適化防止のためここで定義する)
 */
void
infinite_loop(void)
{
	while (1) {
	}
}

/* =begin added for SC3 */
/*
 *  不正な機能コードを指定して，システムサービス発行時の処理
 */

StatusType
no_support_service(void)
{
	StatusType ercd = E_OS_SERVICEID;

#ifdef CFG_USE_ERRORHOOK
	x_nested_lock_os_int();
	call_errorhook(ercd, OSServiceId_INVALID);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

	return(ercd);
}

/* =end added for SC3 */

