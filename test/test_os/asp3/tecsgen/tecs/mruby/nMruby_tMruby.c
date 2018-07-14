/*
 *   Copyright (C) 2008-2017 by TOPPERS Project
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
 *  @(#) $Id: nMruby_tMruby.c 2640 2017-06-03 11:27:12Z okuma-top $
 */

/* #[<PREAMBLE>]#
 * #[<...>]# から #[</...>]# で囲まれたコメントは編集しないでください
 * tecsmerge によるマージに使用されます
 * 呼び口関数 #_TCPF_#
 * call port : cSerialPort  signature: sSerialPort context: task
 *   ER             cSerialPort_open( );
 *   ER             cSerialPort_close( );
 *   ER_UINT        cSerialPort_read( char_t* buffer, uint_t length );
 *   ER_UINT        cSerialPort_write( const char_t* buffer, uint_t length );
 *   ER             cSerialPort_control( uint_t ioControl );
 *   ER             cSerialPort_refer( T_SERIAL_RPOR* pk_rpor );
 *
 * #[</PREAMBLE>]# */

/* プロトタイプ宣言や変数の定義をここに書きます #_PAC_# */
#include "nMruby_tMruby_tecsgen.h"
#include <string.h>

#include <mruby.h>
#include <mruby/proc.h>
#include <mruby/data.h>
#include <mruby/compile.h>

#ifndef E_OK
#define	E_OK	0		/* success */
#define	E_ID	(-18)	/* illegal ID */
#endif

#define MEM_SIZE 1024*1024 // 1MB
//#define VM_EVAL
//#define MALLOC_EVAL

#include <t_syslog.h>

void mrb_init_mrb(CELLCB *p_cellcb, mrb_state *mrb);
/* 受け口関数 #_TEPF_# */
/* #[<ENTRY_PORT>]# eMrubyBody
 * entry port: eMrubyBody
 * signature:  sMrubyBody
 * context:    task
 * #[</ENTRY_PORT>]# */

/* #[<ENTRY_FUNC>]# eMrubyBody_main
 * name:         eMrubyBody_main
 * global_name:  nMruby_tMruby_eMrubyBody_main
 * oneway:       false
 * #[</ENTRY_FUNC>]# */
void
eMrubyBody_main(CELLIDX idx)
{
#ifdef VM_EVAL
  SYSUTM tstart, tend;
#endif
  
  CELLCB	*p_cellcb;
  mrb_state *mrb;
  
  if (VALID_IDX(idx)) {
    p_cellcb = GET_CELLCB(idx);
  }
#ifdef VM_EVAL
  get_utm(&tstart);
#endif

  /* new interpreter instance */
  mrb = mrb_open();
  if (mrb == NULL) {
    syslog(LOG_EMERG, "Invalid mrb_state, exiting test driver");
//    cSerialPort_write("Invalid mrb_state, exiting test driver\n", 39);
    return;
  }

#ifdef VM_EVAL
  get_utm(&tend);
  syslog(LOG_EMERG, "mrb_open time = %d micro sec", tend - tstart);
#endif
  if(is_cInit_joined()){
    // TECS Bridge
    cInit_initializeBridge( mrb );
  }
  mrb_init_mrb(p_cellcb, mrb);
	syslog(LOG_EMERG, "end of mruby program");
#ifdef VM_EVAL
  get_utm(&tstart);
#endif
  mrb_close(mrb);
#ifdef VM_EVAL
  get_utm(&tend);
  syslog(LOG_EMERG, "mrb_close time = %d micro sec", tend - tstart);
#endif
}


/* #[<POSTAMBLE>]#
 *   これより下に非受け口関数を書きます
 * #[</POSTAMBLE>]#*/

void
mrb_init_mrb(CELLCB	*p_cellcb, mrb_state *mrb)
{
  mrb_irep *irep = mrb_read_irep(mrb, ATTR_irep);
  
  mrb_run(mrb, mrb_proc_new(mrb, irep), mrb_top_self(mrb));
  if (mrb->exc) {
    mrb_p(mrb, mrb_obj_value(mrb->exc));
    exit(0);
  }
}

/*
extern void debug_print(char * str);
extern void debug_print_address(char * str, int address);
extern void debug_print_int(char * str, int data);
#include <errno.h>
void *
_sbrk(size_t incr)
{
  // TODO: 正しいヒープサイズを指定すること (EV3はRAM 64MB)
  extern char __heap_start;
  static uint32_t cs3_heap_end = &__heap_start + MEM_SIZE; // ヒープの最後アドレス
  static char *heap_end = &__heap_start;
  char *prev_heap_end;
  uint16_t incr2;
  
  char str[20];
  
#ifdef MALLOC_EVAL
  syslog(LOG_EMERG, "heap_start = %x", &__heap_start);
#endif

  prev_heap_end = heap_end;
  incr2 = (uint16_t)incr;
  
  if (heap_end + incr2 > cs3_heap_end) {
    errno = ENOMEM;
    return (void *)-1;
  }
  heap_end += incr2;

#ifdef MALLOC_EVAL
  syslog(LOG_EMERG, "heap_end = %x", heap_end);
#endif

#ifdef DEBUG_MALLOC	
  debug_print_int("incr",incr);
  debug_print_address("prev_heap_end",(int) prev_heap_end);
  dly_tsk(50);
#endif
  return (void *) prev_heap_end;
}
*/
extern intptr_t bt_snd_chr(intptr_t c);
extern intptr_t uart_snd_chr(intptr_t c) ;
extern void initialize_lcd_dri();
#define UART1	(*(volatile struct st_uart *)0x01D0C000)
size_t fwrite(const void *buf, size_t size, size_t n, FILE *fp){
  intptr_t intp;
  char *cp;
  cp = (char*)buf;
  /* シリアルポートが結合されている場合のみ呼び出す
  if(is_cSerialPort_joined()){
    //cSerialPort_write(buf, size);
   
  }*/
    
  for(int i = 0;i < size; i++){
    intp = *cp++;
    if (intp == '\n') {
	    bt_snd_chr('\r');
	    while(!uart_send(&UART1, '\r'));
    } 
	bt_snd_chr(intp);
    while(!uart_send(&UART1, intp));

  }
    return size;
};

#ifndef EV3_SOUND
void _write(){}
void _close(){}
void _lseek(){}
void _read(){}
//void _fstat(){}
//void _isatty(){}
#endif

//void _exit(){}
//void _getpid(){}
//void _kill(){}
void _gettimeofday(){}
void _fini(){}

