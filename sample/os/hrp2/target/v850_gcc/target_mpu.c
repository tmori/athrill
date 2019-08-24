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

#include "kernel_impl.h"
#include <sil.h>
#include "kernel_cfg.h"
#include "target_mpu.h"

extern char _vector_start;
extern char _vector_end;
extern char _vector_entry_start;
extern char _vector_entry_end;

void
target_mpu_initialize(void)
{
    char *shared_mem_start;
    char *shared_mem_end;

    /*
     *  領域5: 共有text
     */
    shared_mem_start = shared_meminib_table[0];
    shared_mem_end   = shared_meminib_table[1];
    sil_wrw_mem((void *)(MPCSADR5), (uint32_t)shared_mem_start);
    sil_wrw_mem((void *)(MPCEADR5), (uint32_t)shared_mem_end);

    /*
     *  領域6: 共有rodata
     */
    shared_mem_start = shared_meminib_table[2];
    shared_mem_end   = shared_meminib_table[3];
    sil_wrw_mem((void *)(MPCSADR6), (uint32_t)shared_mem_start);
    sil_wrw_mem((void *)(MPCEADR6), (uint32_t)shared_mem_end);

    /*
     *  領域7: 共有data
     */
    shared_mem_start = shared_meminib_table[4];
    shared_mem_end   = shared_meminib_table[5];
    sil_wrw_mem((void *)(MPCSADR7), (uint32_t)shared_mem_start);
    sil_wrw_mem((void *)(MPCEADR7), (uint32_t)shared_mem_end);

    /*
     *  領域8: 共有リード専用ライト領域全体
     */
    shared_mem_start = shared_meminib_table[6];
    shared_mem_end   = shared_meminib_table[7];
    sil_wrw_mem((void *)(MPCSADR8), (uint32_t)shared_mem_start);
    sil_wrw_mem((void *)(MPCEADR8), (uint32_t)shared_mem_end);

    /*
     *  領域9：ベクタテーブル
     */
    sil_wrw_mem((void *)(MPCSADR9), (uint32_t)&_vector_start);
    sil_wrw_mem((void *)(MPCEADR9), (((uint32_t)(&_vector_end) + 3) & 0xfffffffc) - 4);

    /*
     *  領域10：ベクタエントリ
     */
    sil_wrw_mem((void *)(MPCSADR10), (uint32_t)&_vector_entry_start);
    sil_wrw_mem((void *)(MPCEADR10), (((uint32_t)(&_vector_entry_end) + 3) & 0xfffffffc) - 4);

    /*
     *  領域11：MPU有効レジスタ
     */
    sil_wrw_mem((void *)(MPCSADR11), (uint32_t)MPCMPEN);
    sil_wrw_mem((void *)(MPCEADR11), (uint32_t)MPCMPEN);

    /*
     *  Rアクセス制御
     */
    sil_wrw_mem((void *)(MPCRACR), (uint32_t)(0xaaaa07f7));

    /*
     *  Wアクセス制御
     */
    sil_wrw_mem((void *)(MPCWACR), (uint32_t)(0xaaaa089c));

    /*
     *  I(X)アクセス制御
     */
    sil_wrw_mem((void *)(MPCIACR), (uint32_t)(0xaaaa0421));

}

void
target_mpu_exc_handler(void *p_excinf)
{
    syslog(LOG_EMERG, "MPU EXCEPTION OCCURED!!");
    while(1);
}

// タスク例外実行開始時スタック不正ハンドラ
void
target_emulate_texrtn_handler(void *p_excinf)
{
    syslog(LOG_EMERG, "User stack is no space at prepare texrtn.");
    while(1);
}

// タスク例外リターン時スタック不正ハンドラ
void
target_emulate_ret_tex_handler(void *p_excinf)
{
    syslog(LOG_EMERG, "User stack is no space at return texrtn.");
    while(1);
}


/*
 *  指定されたアドレスがユーザモード時に書込み可能かをチェックする
 */
#ifdef OMIT_PROBE_MEM_WRITE
bool_t
probe_mem_write(void *base, SIZE size)
{
	uint32_t top;

    /*
     *  先頭アドレスをサーチ
     */
    sil_wrw_mem((void *)MPCRSADR, (uint32_t)base); /* サーチするアドレスをセット*/
    sil_wrb_mem((void *)MPCRSOP,  (uint8_t)MPCRSOP_S_BIT);  /* 領域サーチを開始 */

	/* 
     *  先頭アドレスのサーチ結果を保存
     */
	top = sil_rew_mem((void *)MPCHITO);

	/* 
     *  書込み可能かチェック
     */
	if((top & MPCHITO_W_MASK) == 0){
		return false;
	}

    /*
     *  終了アドレスをサーチ
     */
    sil_wrw_mem((void *)MPCRSADR, (uint32_t)(base + size)); /* サーチするアドレスをセット*/
    sil_wrb_mem((void *)MPCRSOP,  (uint8_t)MPCRSOP_S_BIT);  /* 領域サーチを開始 */

	/* 
     *  先頭アドレスのサーチ結果と一致するかどうかをチェック
     */
	if(sil_rew_mem((void *)MPCHITO) != top){
        return false;
    }

	return true;
}
#endif /* OMIT_PROBE_MEM_WRITE */

/*
 *  指定されたアドレスがユーザモード時に読込み可能かをチェックする
 */
#ifdef OMIT_PROBE_MEM_READ
bool_t
probe_mem_read(void *base, SIZE size)
{
	uint32_t top;

    /*
     *  先頭アドレスをサーチ
     */
    sil_wrw_mem((void *)MPCRSADR, (uint32_t)base); /* サーチするアドレスをセット*/
    sil_wrb_mem((void *)MPCRSOP,  (uint8_t)MPCRSOP_S_BIT);  /* 領域サーチを開始 */

	/* 
     *  先頭アドレスのサーチ結果を保存
     */
	top = sil_rew_mem((void *)MPCHITO);

	/* 
     *  読み出し可能かチェック
     */
	if((top & MPCHITO_R_MASK) == 0){
		return false;
	}

    /*
     *  終了アドレスをサーチ
     */
    sil_wrw_mem((void *)MPCRSADR, (uint32_t)(base + size)); /* サーチするアドレスをセット*/
    sil_wrb_mem((void *)MPCRSOP,  (uint8_t)MPCRSOP_S_BIT);  /* 領域サーチを開始 */

	/* 
     *  先頭アドレスのサーチ結果と一致するかどうかをチェック
     */
	if(sil_rew_mem((void *)MPCHITO) != top){
        return false;
    }

	return true;
}
#endif /* OMIT_PROBE_MEM_READ */

/*
 *  指定された領域がユーザスタック領域に収まっているかをチェックする
 */
#ifdef OMIT_PROBE_STACK

bool_t
probe_stack(void *base, SIZE size)
{
	if((sil_rew_mem((void *)MPCSADR4) <= (uint32_t)base) &&
	   (sil_rew_mem((void *)MPCEADR4) >= (uint32_t)((SIZE)base + size))){
		return true;
	}

	return false;
}
#endif /* OMIT_PROBE_STACK */


