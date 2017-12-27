/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *	
 *	Copyright (C) 2013-2015 by Embedded and Real-Time Systems Laboratory
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
 *	$Id$
 */

/*
 *	MPUドライバ
 */

#include "kernel_impl.h"
#include "task.h"
#include "memory.h"
#include "prc_sil.h"
#include "Os_Lcfg.h"
#include <t_syslog.h>

void
target_mpu_initialize(void)
{
#ifdef __v850e2v3__
    uint8 *ipal;
    uint8 *ipau;

    /*
     *  bselを保護設定用に
     */
    ipal = (uint8 *)0x00001001;
    LDSR_REG(BSEL, (uint32)ipal);

    /*
     *  IPA2: 共有リード領域
     */
    ipal = shared_meminib_table[0];
    ipau = shared_meminib_table[1];
    IPA_WRITE(2, ipal, ipau);

    /*
     *  IPA3: 共有rosdata
     */
    ipal = shared_meminib_table[2];
    ipau = shared_meminib_table[3];
    IPA_WRITE(3, ipal, ipau);

    /*
     *  DPA5: 共有リードライト領域
     */
    ipal = shared_meminib_table[4];
    ipau = shared_meminib_table[5];
    DPA_WRITE(5, ipal, ipau);

    /*
     *  MPU有効
     *  特権モードではデフォルトのメモリマップ
     *  ハンドラ内ではMPU無効
     */
    ipal = (uint8 *)0x07;
    LDSR_REG(MPM, (uint32)ipal);

    /*
     *  bselを基本設定用に戻す
     */
    LDSR_REG(BSEL, (0U));

#elif defined(__v850e3v5__)
    uint32 mpm = 0x0U;

    /*
     * MPU0: ユーザスタックMPU領域の設定
     */
    LDSR_REG(2, 6, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UW | MPAT_UR))

    mpu_shared_area_initialize();

    /*
     *  MPU有効
     *   DW = 0  バックグラウンドの書込みは禁止
     *   SVP = 0 特権でのアクセス制限はしない
     *   MPE = 1 MPU有効
     */
    /*
     *  RH850/F1L用の設定
     *   DR = 1  バックグラウンドの書込みは禁止
     *   DW = 0  バックグラウンドの書込みは禁止
     *   DX = 1  バックグラウンドの書込みは禁止
     *  RH850/F1H用の設定
     *   DR = 0  バックグラウンドのアクセスは禁止
     *   DW = 0  バックグラウンドのアクセスは禁止
     *   DX = 0  バックグラウンドのアクセスは禁止
     */
    LDSR_REG(0, 5, (uint32)TARGET_MPM_CONFIG);


#else /* __v850e3v5__ */
#error please define ether __v850e2v3__ or __v850e3v5__
#endif /* __v850e2v3__ */

}

#if 0
void
target_impu_exc_handler(void *p_excinf)
{
    uint32 sreg;
    uint32 vmtid;

#if defined(__v850e2v3__)
    sreg = 0x00001000;      /* プロセッサ保護違反バンク */
    LDSR_REG(BSEL, sreg);
    STSR_REG(VMTID, vmtid);
    STSR_REG(VMADR, sreg);
    LDSR_REG(BSEL, (0U));

#elif defined(__v850e3v5__)
    vmtid = (uint32)p_runosap;
    STSR_REG(2, 0, (sreg));

#else /* __v850e3v5__ */
#error please define ether __v850e2v3__ or __v850e3v5__
#endif /* __v850e2v3__ */

    syslog(LOG_EMERG, "MPU EXCEPTION (RX) at 0x%x OCCURED!!", sreg);
    syslog(LOG_EMERG, "p_dominib = 0x%x", vmtid);
    while(1);
}

void
target_dmpu_exc_handler(void *p_excinf)
{
    uint32 sreg;
    uint32 vmtid;

#if defined(__v850e2v3__)
    sreg = 0x00001000;      /* プロセッサ保護違反バンク */
    LDSR_REG(BSEL, (sreg));
    STSR_REG(VMTID, (vmtid));
    STSR_REG(VMADR, (sreg));
    LDSR_REG(BSEL, (0U));
#elif defined(__v850e3v5__)
    vmtid = (uint32)p_runosap;
    STSR_REG(2, 0, (sreg));

#else /* __v850e3v5__ */
#error please define ether __v850e2v3__ or __v850e3v5__
#endif /* __v850e2v3__ */

    syslog(LOG_EMERG, "MPU EXCEPTION (RWX) at 0x%x OCCURED!!", sreg);
    syslog(LOG_EMERG, "p_dominib = 0x%x", vmtid);
    while(1);
}

/*
 *  保護違反FEレベル例外ハンドラ
 */
void
target_protection_handler(void *p_excinf)
{
    uint32 feic = 0;

    /*
     *  FEレベル例外要因の読出し
     */
#if defined(__v850e2v3__)
    LDSR_REG(BSEL, 0U);
	STSR_REG(FEIC, feic);

#elif defined(__v850e3v5__)
	STSR_REG(FEIC, 0, feic);

#else /* __v850e3v5__ */
#error please define ether __v850e2v3__ or __v850e3v5__
#endif /* __v850e2v3__ */

    switch(feic){
        case FE_MIP: /* 実行保護違反例外 */
            target_impu_exc_handler(NULL);
            break;
        case FE_MDP: /* データ保護違反例外 */
            target_dmpu_exc_handler(NULL);
            break;
        default:
            break;
    }
}

#if 1
/*
 *  指定されたアドレスがユーザモード時に書込み可能かをチェックする
 */
#ifdef OMIT_PROBE_MEM_WRITE
boolean
probe_mem_write(const MemoryStartAddressType base, MemorySizeType size)
{
    uint32 sreg;
    boolean ret;

    /*
     *  信頼OSAPならば任意のアドレスにアクセス可能
     */
    if(run_trusted != FALSE){
        return TRUE;
    }

#ifdef __v850e2v3__
    sreg = 0x00001000;      /* プロセッサ保護違反バンク */
    /*
     *  バンクをセット
     */
    LDSR_REG(BSEL, (sreg));
    /*
     *  先頭アドレスをMCAにセット
     */
    LDSR_REG(MCA, (uint32)base);

    /*
     *  サイズをMCSにセット
     */
    LDSR_REG(MCS, (uint32)size);

    /*
     *  メモリ保護設定チェック開始
     */
    LDSR_REG(MCC, 0U);

    /*
     *  メモリ保護設定チェック結果を読み出す
     */
    STSR_REG(MCR, sreg);
#elif __v850e3v5__
    /*
     *  先頭アドレスをMCAにセット
     */
    LDSR_REG(8, 5, (uint32)base);
    /*
     *  サイズをMCSにセット
     */
    LDSR_REG(9, 5, (uint32)size);
    /*
     *  メモリ保護設定チェック開始
     */
    LDSR_REG(10, 5, 0U);
    /*
     *  メモリ保護設定チェック結果を読み出す
     */
    STSR_REG(11, 5, sreg);
#else
#error hoge!
#endif

    if(sreg & 0x00000100){
        /* オーバフローの場合 */
        ret = FALSE;
    } else if(sreg & 0x00000004){
        /* ライト許可の場合 */
        ret = TRUE;
    } else {
        ret = FALSE;
    }

#ifdef __v850e2v3__
    /*
     *  バンクを戻す
     */
    LDSR_REG(BSEL, 0U);
#endif /* __v850e2v3__ */

	return ret;
}
#endif /* OMIT_PROBE_MEM_WRITE */

#ifdef __v850e2v3__
/*
 *  指定されたアドレスがユーザモード時に読込み可能かをチェックする
 */
#ifdef OMIT_PROBE_MEM_READ
boolean
probe_mem_read(const MemoryStartAddressType base, MemorySizeType size)
{
    uint32 sreg;
    boolean ret;

    /*
     *  信頼OSAPならば任意のアドレスにアクセス可能
     */
    if(run_trusted != FALSE){
        return TRUE;
    }

    sreg = 0x00001000;      /* プロセッサ保護違反バンク */
    /*
     *  バンクをセット
     */
    LDSR_REG(BSEL, sreg);

    /*
     *  先頭アドレスをMCAにセット
     */
    LDSR_REG(MCA, (uint32)base);

    /*
     *  サイズをMCSにセット
     */
    LDSR_REG(MCS, (uint32)size);

    /*
     *  メモリ保護設定チェック開始
     */
    LDSR_REG(MCC, (uint32)size);

    /*
     *  メモリ保護設定チェック結果を読み出す
     */
    STSR_REG(MCR, sreg);

    if(sreg & 0x00000100){
        /* オーバフローの場合 */
        ret = FALSE;
    } else if(sreg & 0x00000002){
        /* リード許可の場合 */
        ret = TRUE;
    } else {
        ret = FALSE;
    }

    /*
     *  バンクを戻す
     */
    LDSR_REG(BSEL, 0U);

	return ret;
}
#endif /* OMIT_PROBE_MEM_READ */

/*
 *  指定された領域がユーザスタック領域に収まっているかをチェックする
 */
#ifdef OMIT_PROBE_STACK

boolean
probe_stack(const void *base, MemorySizeType size)
{
    uint32 sreg;
    boolean ret;

    sreg = 0x00001000;      /* プロセッサ保護違反バンク */
    /*
     *  バンクをセット
     */
    LDSR_REG(BSEL, sreg);

    /*
     *  先頭アドレスをMCAにセット
     */
    LDSR_REG(MCA, (uint32)base);

    /*
     *  サイズをMCSにセット
     */
    LDSR_REG(MCS, (uint32)size);

    /*
     *  メモリ保護設定チェック開始
     */
    LDSR_REG(MCC, 0U);

    /*
     *  メモリ保護設定チェック結果を読み出す
     */
    STSR_REG(MCR, sreg);

    if(sreg & 0x00000100){
        /* オーバフローの場合 */
        ret = FALSE;
    } else if(sreg & 0x00000010){
        /* スタックの場合 */
        ret = TRUE;
    } else {
        ret = FALSE;
    }

    /*
     *  バンクを戻す
     */
    LDSR_REG(BSEL, 0U);

	
	return ret;
}
#endif /* OMIT_PROBE_STACK */
#endif /* __v850e2v3__ */
#endif
#endif /* 0 */

AccessType
probe_trusted_osap_mem(const MemoryStartAddressType sadr, const MemoryStartAddressType eadr)
{
    /*
     *  信頼OSAPならば任意のアドレスにアクセス可能
     */
    return(AP_Executable|AP_Readable|AP_Writable);
}

