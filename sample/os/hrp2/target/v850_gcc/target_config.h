#ifndef TOPPERS_TARGET_CONFIG_H
#define TOPPERS_TARGET_CONFIG_H

#include "v850es_fk3_emu_env.h"


#ifdef TOPPERS_ENABLE_TRACE
#include "logtrace/trace_config.h"
#endif /* TOPPERS_ENABLE_TRACE */


#define TARGET_PUTC_PORTID	(1)
#define TARGET_PUTC_BAUD	(38400)

#if TARGET_PUTC_PORTID == 1
	#define TARGET_FPUTC_UAnSTR	UA0STR
	#define TARGET_FPUTC_UAnTX	UA0TX
#elif TARGET_PUTC_PORTID == 2
	#define TARGET_FPUTC_UAnSTR	UA1STR
	#define TARGET_FPUTC_UAnTX	UA1TX
#elif TARGET_PUTC_PORTID == 3
	#define TARGET_FPUTC_UAnSTR	UA2STR
	#define TARGET_FPUTC_UAnTX	UA2TX
#else
	#error "TARGET_PUTC_BAUD must set [1 - 3]."
#endif


#if TARGET_PUTC_BAUD == 38400
	#define TARGET_FPUTC_UAnCTL1_SETTING		BAUD_38400_UA0CTL1
	#define TARGET_FPUTC_UAnCTL2_SETTING		BAUD_38400_UA0CTL2
#elif TARGET_TARGET_FPUTC_PUTC_BAUD == 19200
	#define TARGET_FPUTC_UAnCTL1_SETTING		BAUD_19200_UA0CTL1
	#define TARGET_FPUTC_UAnCTL2_SETTING		BAUD_19200_UA0CTL2
#elif TARGET_PUTC_BAUD == 9600
	#define TARGET_FPUTC_UAnCTL1_SETTING		BAUD_9600_UA0CTL1
	#define TARGET_FPUTC_UAnCTL2_SETTING		BAUD_9600_UA0CTL2
#else
	#error "TARGET_PUTC_BAUD must set 38400 , 19200 , or 9600."
#endif

#define DEFAULT_ISTKSZ    0x2000U

#ifndef TOPPERS_MACRO_ONLY


#define DEFAULT_SSTKSZ 1280 

/*
 *	タスク初期化コンテキストブロック
 */

#define USE_TSKINICTXB	/*  TSKINICTXBを使用する  */

typedef struct task_initialization_context_block {
	SIZE	sstksz;			/* スタック領域のサイズ（丸めた値） */
	void	*sstk_bottom;	/* スタックポインタの初期値（スタックの底の初期値） */
	SIZE	stksz;			/* スタック領域のサイズ（丸めた値） */
	void	*stk_bottom;	/* スタックポインタの初期値（スタックの底の初期値） */
    void    *sustk; /* ユーザスタックの先頭番地 */
    void    *eustk; /* ユーザスタックの終了番地 */
} TSKINICTXB;

/*
 *  保護ドメイン初期化コンテキストブロック
 */

#define USE_DOMINICTXB	/*  TSKINICTXBを使用する  */

typedef struct {
    void *stext;
    void *etext;
    void *srodata;
    void *erodata;
    void *sdata;
    void *edata;
    void *ssrpw;
    void *esrpw;
    uint32_t *valid_map;
} DOMINICTXB;

/*
 *  ユーザタスクが特権モード実行中か？
 *  priv_modeは、サービスコール、および、拡張サービスコールの
 *  入り口処理でtrueにセットされる
 */
#define t_sense_priv(p_tcb) ((p_tcb)->tskctxb.priv_mode == true)
#define i_sense_priv(p_tcb) ((p_tcb)->tskctxb.priv_mode == true)
#define t_sense_priv_runtsk() ((p_runtsk)->tskctxb.priv_mode == true)
#define i_sense_priv_runtsk() ((p_runtsk)->tskctxb.priv_mode == true)
/*
 *  ユーザタスクのタスク例外処理呼び出し時に使用するスタックサイズが足りないか？
 *  i_check_tex_runtskは、非タスクコンテキストから呼ばれるので、
 *  ユーザスタックはTCBに保存されているはずである
 */
extern bool_t i_check_tex_runtsk();
extern void target_initialize(void);
/*
 * CPU例外番号に関する定義
 */
#define TMIN_EXCNO		UINT_C(4)		/*	GENERAL_ILLEGAL_INSTRUCTION_VECTOR	*/
#define TMAX_EXCNO		UINT_C(63)		/*	TRAPA_INST_VECTOR  */
#define TNUM_EXC		(TMAX_EXCNO - TMIN_EXCNO + 1U)
/*
 * 割込みハンドラ番号に関する定義
 */
#define TMIN_INHNO		UINT_C(64)
#define TMAX_INHNO		UINT_C(493)
#define TNUM_INH		(TMAX_INHNO - TMIN_INHNO + 1U)
/*	IRQ  */
#define TMIN_INTNO_IRQ 	64U
#define TMAX_INTNO_IRQ	71U
/*	CFG_INTで割込み優先度が設定されていないことを示すビット  */
#ifndef NOT_CFG_INT_BIT
#define NOT_CFG_INT_BIT		0x80U
#endif /* NOT_CFG_INT_BIT */


extern void target_exit(void);

#endif /* TOPPERS_MACRO_ONLY */


#include "v850_gcc/prc_config.h"
#define TEXCNO_EMULATE_TEXRTN 20
#define TEXCNO_EMULATE_RET_TEX 21

#define TOPPERS_get_utm
#define TOPPERS_SUPPORT_GET_UTM
#undef OMIT_GET_UTM

#endif /* TOPPERS_TARGET_CONFIG_H */
