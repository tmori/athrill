/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *	
 *	Copyright (C) 2013 by Embedded and Real-Time Systems Laboratory
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
 *	MPUドライバ（V850E2用）
 */

#ifndef TOPPERS_PRC_MPU_H
#define TOPPERS_PRC_MPU_H

#ifndef TOPPERS_MACRO_ONLY
/*
 * システムレジスタへの書き込み
 */
#define IPA_WRITE(n, ipal, ipau) {\
    Asm("\tldsr %0, ipa" #n "l \n"\
        "\tldsr %1, ipa" #n "u \n"\
        : : "r"(ipal), "r"(ipau): );\
}
#define DPA_WRITE(n, dpal, dpau) {\
    Asm("\tldsr %0, dpa" #n "l \n"\
        "\tldsr %1, dpa" #n "u \n"\
        : : "r"(dpal), "r"(dpau): );\
}

extern void *p_ctxosap;
/* 初期化ルーチン */
extern void target_mpu_initialize(void);
/* メモリ保護違反例外ハンドラ */
extern void target_impu_exc_handler(void *p_excinf);
extern void target_dmpu_exc_handler(void *p_excinf);
/* タスク例外実行開始時スタック不正ハンドラ */
extern void target_emulate_texrtn_handler(void *p_excinf);
/* タスク例外リターン時スタック不正ハンドラ */
extern void target_emulate_ret_tex_handler(void *p_excinf);

/*
 *  保護違反FEレベル例外ハンドラ
 */
#define EXCNO_PROTECTION 3
extern void target_protection_handler(void *p_excinf);

/*
 *  TODO: probe_memをターゲット依存部で定義できる？
 */
#define OMIT_PROBE_MEM_WRITE
#if 0
#define PROBE_MEM_WRITE(p_var, type) \
	(probe_mem_write((MemoryStartAddressType) (p_var), sizeof(type)))
#endif
#define OMIT_PROBE_MEM_READ
#define OMIT_PROBE_STACK

extern AccessType probe_trusted_osap_mem(const MemoryStartAddressType sadr, const MemoryStartAddressType eadr);

/*
 *  Os_Lcfg.c
 */
extern void mpu_shared_area_initialize(void);

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_PRC_MPU_H */
