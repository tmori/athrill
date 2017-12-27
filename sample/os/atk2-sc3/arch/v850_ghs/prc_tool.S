/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2015 by Center for Embedded Computing Systems
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
 *  $Id: prc_tool.S 183 2015-06-24 08:19:10Z t_ishikawa $  
 */

/*
 *  V850用cal_svc実装
 *  GHSにおけるインラインアセンブラでは，破壊レジスタの指定が
 *  保証されないため，直接アセンブリで記述する
 */
#include <v850asm.inc>
#include "offset.h"

#ifdef USE_ASMCONFIG_INC
#include "asm_config.inc"
#endif /* USE_ASMCONFIG_INC */

    .section .text_kernel
    .align 4
/*
 *  メモリ再読込が必要なことをコンパイラに伝えるためのダミー関数
 */
    .globl _v850_memory_changed
    /* 
     *  void v850_memory_changed(void)
     */
FLABEL(_v850_memory_changed)
    jmp lp
    nop

    .globl _v850_sync_memory
    /* 
     *  void v850_sync_memory(void)
     */
FLABEL(_v850_sync_memory)
    syncm
    jmp lp
    nop

/*
 *  サービスコールの出入口
 *    r10 : 返り値
 *    r11：関数コード
 *    r6 - r9 : 引数（4つまで）
 *    スクラッチレジスタ保存（関数呼出しによりここでは保存済み）
 *    lp保存
 */
    .section .text_shared
    .align 4
    .globl _cal_svc_0m
    /* 
     * extern int cal_svc_0m(int fncd)
     *   __attribute__((section(".text_shared")));
     */
FLABEL(_cal_svc_0m)
    mov r6, r11
    pushsp ep-lp
    syscall 1
    nop
    popsp ep-lp
    jmp [lp]
    nop

    .globl _cal_svc_1m
    /* 
     * extern int cal_svc_1m(int par1, int fncd)
     *   __attribute__((section(".text_shared")));
     */
FLABEL(_cal_svc_1m)
    mov r7, r11
    pushsp ep-lp
    syscall 1
    nop
    popsp ep-lp
    jmp [lp]
    nop

    .globl _cal_svc_2m
    /* 
     * extern int cal_svc_2m(int par1, int par2, int fncd)
     *   __attribute__((section(".text_shared")));
     */
FLABEL(_cal_svc_2m)
    mov r8, r11
    pushsp ep-lp
    syscall 1
    nop
    popsp ep-lp
    jmp [lp]
    nop

    .globl _cal_svc_3m
    /* 
     * extern int cal_svc_3m(int par1, int par2, int par3, int fncd)
     *   __attribute__((section(".text_shared")));
     */
FLABEL(_cal_svc_3m)
    mov r9, r11
    pushsp ep-lp
    syscall 1
    nop
    popsp ep-lp
    jmp [lp]
    nop

    .globl _cal_svc_0n
    /* 
     * extern void cal_svc_0n(int fncd)
     *   __attribute__((section(".text_shared")));
     */
FLABEL(_cal_svc_0n)
    mov r6, r11
    pushsp ep-lp
    syscall 1
    nop
    popsp ep-lp
    jmp [lp]
    nop

    .globl _cal_svc_1n
    /* 
     * extern void cal_svc_1n(int par1, int fncd)
     *   __attribute__((section(".text_shared")));
     */
FLABEL(_cal_svc_1n)
    mov r7, r11
    pushsp ep-lp
    syscall 1
    nop
    popsp ep-lp
    jmp [lp]
    nop

    .globl _cal_svc_2n
    /* 
     * extern void cal_svc_2n(int par1, int par2, int fncd)
     *   __attribute__((section(".text_shared")));
     */
FLABEL(_cal_svc_2n)
    mov r8, r11
    pushsp ep-lp
    syscall 1
    nop
    popsp ep-lp
    jmp [lp]
    nop

    .globl _cal_svc_3n
    /* 
     * extern void cal_svc_3n(int par1, int par2, int par3, int fncd)
     *   __attribute__((section(".text_shared")));
     */
FLABEL(_cal_svc_3n)
    mov r9, r11
    pushsp ep-lp
    syscall 1
    nop
    popsp ep-lp
    jmp [lp]
    nop


