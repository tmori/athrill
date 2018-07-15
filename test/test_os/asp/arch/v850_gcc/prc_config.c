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
 *  Copyright (C) 2010 by Meika Sugimoto
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

/*
 *	プロセッサ依存モジュール（V850用）
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"

/*
 *  プロセッサ依存部で用いる変数
 */
uint8_t intnest;
uint8_t current_intpri;
bool_t lock_flag;
uint16_t saved_imr[IMR_SIZE];
uint16_t disint_table[IMR_SIZE];

/*
 *  プロセッサ依存の初期化
 */
void
prc_initialize(void)
{
	int_t i;
	
	/* 変数の初期化 */
	intnest = 0u;
	current_intpri = 0u;
	lock_flag = true;
	
	for(i = 0 ; i < IMR_SIZE ; i++)
	{
		disint_table[i] = 0x0000;
	}
}

/*
 *  プロセッサ依存の終了処理
 */
void
prc_terminate(void)
{
	/* 特に行う処理はない */
}

/* ISPRテーブルの参照(prc.tfにて生成) */
extern const uint16_t imr_table[][IMR_SIZE];

/*
 *	現在の割込み優先度マスク(内部表現)の設定
 *
 *	インライン関数でないのは，アセンブラからも使用するためである．
 */
void
set_intpri(uint8_t intpri)
{
	uint_least8_t i;
	
	/* ISPRに書き込み */
	for(i = 0 ; i < IMR_SIZE ; i++)
	{
		sil_wrh_mem((void *)(IMR0 + (i * 2)) , 
			imr_table[intpri][i] | disint_table[i]);
	}
}


/*
 *  割込み要求ライン属性の設定
 */

struct _int_pol_table
{
	intptr_t pol_setting0;
	intptr_t pol_setting1;
	uint8_t bitpos;
};

struct _int_pol_table const int_pol_table[] = 
{
	/* 型番によって違う可能性があるため，マクロ定義する */
	INT_POLREG_TABLE
};

void
x_config_int(INTNO intno, ATR intatr, PRI intpri)
{
	assert(VALID_INTNO_CFGINT(intno));
	assert(TMIN_INTPRI <= intpri && intpri <= TMAX_INTPRI);
	uint32_t intreg_addr = INTREG_ADDRESS(intno);
	
	/*
	 *  割込みのマスク
	 *
	 *  割込みを受け付けたまま，レベルトリガ／エッジトリガの設定や，割
	 *  込み優先度の設定を行うのは危険なため，割込み属性にかかわらず，
	 *  一旦マスクする．
	 */
	(void)x_disable_int(intno);
	
	if(((7u <= (intno)) && ((intno) <= 15u)) || (intno == 1u))
	{	
		/* INT端子の場合は割込み検知方法を設定する */
		if((intatr & TA_POSEDGE) != 0U)
		{
			/* 立上がりエッジ , pol_setting0を0に，pol_setting1を1に */
			sil_wrb_mem((void *)int_pol_table[intno].pol_setting0 , 
					((sil_reb_mem((void *)int_pol_table[intno].pol_setting0))
						& ~(1U << int_pol_table[intno].bitpos)));
			sil_wrb_mem((void *)int_pol_table[intno].pol_setting1 , 
					((sil_reb_mem((void *)int_pol_table[intno].pol_setting1))
						| (1U << int_pol_table[intno].bitpos)));
		}
		else if((intatr & TA_NEGEDGE) != 0U)
		{
			/* 立下がりエッジ , pol_setting0を1に，pol_setting1を0に */
			sil_wrb_mem((void *)int_pol_table[intno].pol_setting0 , 
					((sil_reb_mem((void *)int_pol_table[intno].pol_setting0))
						| (1U << int_pol_table[intno].bitpos)));
			sil_wrb_mem((void *)int_pol_table[intno].pol_setting1 , 
					((sil_reb_mem((void *)int_pol_table[intno].pol_setting1))
						& ~(1U << int_pol_table[intno].bitpos)));
		}
		else if((intatr & TA_BOTHEDGE) != 0U)
		{
			/* 両エッジ , pol_setting0を1に，pol_setting1を1に */
			sil_wrb_mem((void *)int_pol_table[intno].pol_setting0 , 
					((sil_reb_mem((void *)int_pol_table[intno].pol_setting0))
						| (1U << int_pol_table[intno].bitpos)));
			sil_wrb_mem((void *)int_pol_table[intno].pol_setting1 , 
					((sil_reb_mem((void *)int_pol_table[intno].pol_setting1))
						| (1U << int_pol_table[intno].bitpos)));
		}
	}
	
	/*
	 *  割込み優先度の設定
	 */
	sil_wrb_mem((void *)intreg_addr ,
		((sil_reb_mem((void *)intreg_addr) & ~0x7)
			| (7u - INT_IPM((volatile PRI)intpri))));
	
	/*
	 *  割込みのマスク解除
 	 */
	if ((intatr & TA_ENAINT) != 0U) {
		(void)x_enable_int(intno);
	}
}

/*
 * CPU例外ハンドラの初期化
 * 　空マクロにしたいが、asp/kernel/exception.hでプロトタイプ宣言
 * 　されているため、関数として定義しなければならない。
 */
void
initialize_exception(void)
{
	/* 何もしない */
}


#ifndef OMIT_DEFAULT_EXC_HANDLER
/*
 *  登録されていない例外が発生すると呼び出される
 */
void
default_exc_handler(void *p_excinf)
{
    target_exit();
}
#endif /* OMIT_DEFAULT_EXC_HANDLER */

#ifndef OMIT_DEFAULT_INT_HANDLER
/*
 *  未登録の割込みが発生した場合に呼び出される
 */
void
default_int_handler(void *p_excinf)
{
    target_exit();
}
#endif /* OMIT_DEFAULT_INT_HANDLER */

