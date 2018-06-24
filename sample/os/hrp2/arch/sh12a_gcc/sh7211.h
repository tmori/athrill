/*
 *	TOPPERS/HRP Kernel
 *		Toyohashi Open Platform for Embedded Real-Time Systems/
 *		High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2007 by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	Copyright (C) 2007-2011 by Industrial Technology Institute,
 *								Miyagi Prefectural Government, JAPAN
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
 *	$Id: sh7211.h 2156 2011-07-19 06:03:20Z mit-kimai $
 */

/*
 *	SH7211のハードウェア資源の定義
 */

#ifndef TOPPERS_SH7211_H
#define TOPPERS_SH7211_H

/*
 *	例外ベクタ
 */
#define POWER_ON_RESET_VECTOR				0
#define MANUAL_RESET_VECTOR					2
#define GENERAL_ILLEGAL_INSTRUCTION_VECTOR	4
#define RAM_ERROR_VECTOR					5
#define SLOT_ILLEGAL_INSTRUCTION_VECTOR		6
#define CPU_ADDRESS_ERROR_VECTOR			9
#define DMA_ADDRESS_ERROR_VECTOR			10
#define FPU_VECTOR							13
#define BANK_OVER_FLOW_VECTOR				15
#define BANK_UNDER_FLOW_VECTOR				16
#define DIVIDE_BY_ZERO_VECTOR				17

/*
 *	割込みベクタ
 */
#define NMI_VECTOR 	   				11
#define IRQ0_VECTOR 	   			64
#define IRQ1_VECTOR 	   			65
#define IRQ2_VECTOR 	   			66
#define IRQ3_VECTOR 	   			67
#define IRQ4_VECTOR 	   			68
#define IRQ5_VECTOR 	   			69
#define IRQ6_VECTOR 	   			70
#define IRQ7_VECTOR    				71

#define ADI_VECTOR					92

#define DMAC0_TEI0_VECTOR		 	108
#define DMAC0_HEI0_VECTOR		 	109
#define DMAC1_TEI1_VECTOR		 	112
#define DMAC1_HEI1_VECTOR		 	113
#define DMAC2_TEI2_VECTOR		 	116
#define DMAC2_HEI2_VECTOR		 	117
#define DMAC3_TEI3_VECTOR		 	120
#define DMAC3_HEI3_VECTOR		 	121
#define DMAC4_TEI4_VECTOR		 	124
#define DMAC4_HEI4_VECTOR		 	125
#define DMAC5_TEI5_VECTOR		 	128
#define DMAC5_HEI5_VECTOR		 	129
#define DMAC6_TEI6_VECTOR		 	132
#define DMAC6_HEI6_VECTOR		 	133
#define DMAC7_TEI7_VECTOR		 	136
#define DMAC7_HEI7_VECTOR		 	137

#define CMI0_VECTOR 	   			140
#define CMI1_VECTOR 	   			144
#define CMI_VECTOR		  			148

#define ITI_VECTOR		  			152

#define MTU0_TGI0A_VECTOR		 	156
#define MTU0_TGI0B_VECTOR		 	157
#define MTU0_TGI0C_VECTOR		 	158
#define MTU0_TGI0D_VECTOR		 	159
#define MTU0_TGI0V_VECTOR		 	160
#define MTU0_TGI0E_VECTOR		 	161
#define MTU0_TGI0F_VECTOR		 	162

#define MTU1_TGI1A_VECTOR		 	164
#define MTU1_TGI1B_VECTOR		 	165
#define MTU1_TGI1V_VECTOR		 	168
#define MTU1_TGI1U_VECTOR		 	169

#define MTU2_TGI2A_VECTOR		 	172
#define MTU2_TGI2B_VECTOR		 	173
#define MTU2_TGI2V_VECTOR		 	176
#define MTU2_TGI2U_VECTOR		 	177

#define MTU3_TGI3A_VECTOR		 	180
#define MTU3_TGI3B_VECTOR		 	181
#define MTU3_TGI3C_VECTOR		 	182
#define MTU3_TGI3D_VECTOR		 	183
#define MTU3_TGI3V_VECTOR		 	184

#define MTU4_TGI4A_VECTOR		 	188
#define MTU4_TGI4B_VECTOR		 	189
#define MTU4_TGI4C_VECTOR		 	190
#define MTU4_TGI4D_VECTOR		 	191
#define MTU4_TGI4V_VECTOR		 	192

#define MTU5_TGI5U_VECTOR		 	196
#define MTU5_TGI5V_VECTOR		 	197
#define MTU5_TGI5W_VECTOR		 	198

#define OEI1_VECTOR 	   			200
#define OEI2_VECTOR 	   			201

#define MTU3S_TGI3A_VECTOR		  	204
#define MTU3S_TGI3B_VECTOR		  	205
#define MTU3S_TGI3C_VECTOR		  	206
#define MTU3S_TGI3D_VECTOR		  	207
#define MTU3S_TGI3V_VECTOR		  	208

#define MTU4S_TGI3A_VECTOR		  	212
#define MTU4S_TGI3B_VECTOR		  	213
#define MTU4S_TGI3C_VECTOR		  	214
#define MTU4S_TGI3D_VECTOR		  	215
#define MTU4S_TGI3V_VECTOR		  	216

#define MTU5S_TGI5U_VECTOR		  	220
#define MTU5S_TGI5V_VECTOR		  	221
#define MTU5S_TGI5W_VECTOR		  	222

#define OEI3_VECTOR 	   			224

#define IIC3_STPI_VECTOR			228
#define IIC3_NAKI_VECTOR			229
#define IIC3_RXI_VECTOR    			230
#define IIC3_TXI_VECTOR    			231
#define IIC3_TEI_VECTOR    			232

#define SCIF0_BRI_VECTOR   			240
#define SCIF0_ERI_VECTOR   			241
#define SCIF0_RXI_VECTOR   			242
#define SCIF0_TXI_VECTOR   			243

#define SCIF1_BRI_VECTOR   			244
#define SCIF1_ERI_VECTOR   			245
#define SCIF1_RXI_VECTOR   			246
#define SCIF1_TXI_VECTOR   			247

#define SCIF2_BRI_VECTOR   			248
#define SCIF2_ERI_VECTOR   			249
#define SCIF2_RXI_VECTOR   			250
#define SCIF2_TXI_VECTOR   			251

#define SCIF3_BRI_VECTOR   			252
#define SCIF3_ERI_VECTOR   			253
#define SCIF3_RXI_VECTOR   			254
#define SCIF3_TXI_VECTOR   			255

#define WAVEIF_ERR_VECTOR			256
#define WAVEIF_WRXI_VECTOR			257
#define WAVEIF_WTXI_VECTOR			258

/*
 * 割込みコントローラレジスタ（INTC）
 */
#define ICR0_h	UINT_C(0xfffe0800)
#define ICR1_h	UINT_C(0xfffe0802)
#define IRQRR_h	UINT_C(0xfffe0806)
#define IBCR_h	UINT_C(0xfffe080c)
#define IBNR_h	UINT_C(0xfffe080e)
 #define IBNR_BE0	 UINT_C(0x4000)
 #define IBNR_BOVE	 UINT_C(0x2000)

#define IPR01_h	UINT_C(0xfffe0818)
#define IPR02_h	UINT_C(0xfffe081a)
#define IPR05_h	UINT_C(0xfffe0820)
#define IPR06_h	UINT_C(0xfffe0c00)
#define IPR07_h	UINT_C(0xfffe0c02)
#define IPR08_h	UINT_C(0xfffe0c04)
#define IPR09_h	UINT_C(0xfffe0c06)
#define IPR10_h	UINT_C(0xfffe0c08)
#define IPR11_h	UINT_C(0xfffe0c0a)
#define IPR12_h	UINT_C(0xfffe0c0c)
#define IPR13_h	UINT_C(0xfffe0c0e)
#define IPR14_h	UINT_C(0xfffe0c10)
#define IPR15_h	UINT_C(0xfffe0c12)

#define IRQ_POSEDGE   UINT_C(0x02)
#define IRQ_NEGEDGE   UINT_C(0x01)

#define INTC_BASE	ICR0_w	/*	ベースアドレス	*/

/*
 *	コンペアマッチ・タイマ（CMT）
 */
#define CMSTR_h     UINT_C(0XFFFEC000)
#define CMCSR_0_h   UINT_C(0XFFFEC002)
#define CMCNT_0_h   UINT_C(0XFFFEC004)
#define CMCOR_0_h   UINT_C(0XFFFEC006)
#define CMCSR_1_h   UINT_C(0XFFFEC008)
#define CMCNT_1_h   UINT_C(0XFFFEC00A)
#define CMCOR_1_h   UINT_C(0XFFFEC00C)

#define CMSTR_STR0	 UINT_C(0x0001)
#define CMSTR_STR1	 UINT_C(0x0002)
#define CMCSR_CMF	 UINT_C(0x0080)
#define CMCSR_CMIE	 UINT_C(0x0040)

#define CMT_BASE	CMSTR_h	/*	ベースアドレス	*/

/*
 *	FIFO付きシリアルコミュニケーションインターフェース(SCIF)レジスタ
 */
#define SH_SCIF0_BASE	UINT_C(0xFFFE8000)	/*	ベースアドレス	*/
#define SH_SCIF1_BASE	UINT_C(0xFFFE8800)
#define SH_SCIF2_BASE	UINT_C(0xFFFE9000)

/*
 *	ピンファンクションコントローラ（PFC）
 */
#define PAIORH_h	UINT_C(0xFFFE3804)
#define PAIORL_h	UINT_C(0xFFFE3806)
#define PACRH3_h	UINT_C(0xFFFE380A)

#define PACRL1_h	UINT_C(0xfffe3816)
#define PACRL2_h	UINT_C(0xfffe3814)
#define PACRL3_h	UINT_C(0xfffe3812)
#define PACRL4_h	UINT_C(0xfffe3810)

#define PBIORH_h	UINT_C(0xfffe3884)
#define PBIORL_h	UINT_C(0xfffe3886)
#define PBCRH1_h	UINT_C(0xfffe388e)
#define PBCRL1_h	UINT_C(0xfffe3896)
#define PBCRL2_h	UINT_C(0xfffe3894)
#define PBCRL3_h	UINT_C(0xfffe3892)

#define PDIOR_h		UINT_C(0xfffe3986)
#define PDCRL1_h	UINT_C(0xfffe3996)
#define PDCRL2_h	UINT_C(0xfffe3994)
#define PDCRL3_h	UINT_C(0xfffe3992)
#define PDCRL4_h	UINT_C(0xfffe3990)

#define PFC_BASE	PAIORH_h	/*	ベースアドレス	*/

/*
 *	クロックパルス発振器（CPG）
 */
#define FRQCR_h		UINT_C(0xfffe0010)

/*
 *	低消費電力モード関連（PDM：Power-Down Modes）
 */
#define STBCR_b			UINT_C(0xfffe0014)
#define STBCR2_b		UINT_C(0xfffe0018)
 #define STBCR2_HUDI	UINT_C(0x80)
 #define STBCR2_UCB		UINT_C(0x40)
 #define STBCR2_DMAC	UINT_C(0x20)
#define STBCR3_b		UINT_C(0xfffe0408)
 #define STBCR3_HZ		UINT_C(0x80)
 #define STBCR3_MTU2S	UINT_C(0x40)
 #define STBCR3_MTU2	UINT_C(0x20)
 #define STBCR3_POE2	UINT_C(0x10)
 #define STBCR3_IIC3	UINT_C(0x08)
 #define STBCR3_ADC		UINT_C(0x04)
 #define STBCR3_DAC		UINT_C(0x02)
 #define STBCR3_FLASH	UINT_C(0x01)
#define STBCR4_b		UINT_C(0xfffe040C)
 #define STBCR4_SCIF0	UINT_C(0x80)
 #define STBCR4_SCIF1	UINT_C(0x40)
 #define STBCR4_SCIF2	UINT_C(0x20)
 #define STBCR4_SCIF3	UINT_C(0x10)
 #define STBCR4_CMT		UINT_C(0x04)
 #define STBCR4_WAVEIF	UINT_C(0x02)

#define SYSCR1_b		UINT_C(0xfffe0402)
#define SYSCR2_b		UINT_C(0xfffe0404)

#define PDM_BASE	STBCR_b		/*	ベースアドレス	*/
#define PDM_BASE2	SYSCR1_b	/*	ベースアドレス	*/

/*
 *	バスステートコントローラ（BSC）
 */
#define CMNCR_w		UINT_C(0xfffc0000)
#define CS3BCR_w	UINT_C(0xfffc0010)
#define CS3WCR_w	UINT_C(0xfffc0034)
#define SDCR_w		UINT_C(0xfffc004c)
#define RTCSR_w		UINT_C(0xfffc0050)
#define RTCOR_w		UINT_C(0xfffc0058)

#define BSC_BASE	CMNCR_w	/*	ベースアドレス	*/

/*
 *	内蔵RAM
 */
#define INNER_RAM_PAGE1_START		UINT_C(0xfff80000)
#define INNER_RAM_PAGE2_START		UINT_C(0xfff82000)
#define INNER_RAM_PAGE3_START		UINT_C(0xfff84000)
#define INNER_RAM_PAGE4_START		UINT_C(0xfff86000)

#define INNER_RAM_PAGE_SIZE			UINT_C(0x00002000)

#define INNER_RAM_PAGE4_END		(INNER_RAM_PAGE4_START + INNER_RAM_PAGE_SIZE)
/*  ページ4以外は次のページの先頭アドレスを使えばよい。  */


/*
 *  マルチファンクションタイマパルスユニットMTU2
 */

/*
 *  共通
 */
#define TSTR_b		0xfffe4280U
#define TSTR_CST0	0x01U
#define TSTR_CST1	0x02U
#define TSTR_CST2	0x04U
#define TSTR_CST3	0x40U
#define TSTR_CST4	0x80U


/*
 *  チャネル0
 */
#define TCR_0_b		0xfffe4300U
#define TIER_0_b	0xfffe4304U
#define TSR_0_b		0xfffe4305U
#define TCNT_0_h	0xfffe4306U

/*
 *  チャネル1
 */
#define TCR_1_b		0xfffe4380U
#define TIER_1_b	0xfffe4384U
#define TSR_1_b		0xfffe4385U
#define TCNT_1_h	0xfffe4386U

/*
 *  チャネル2
 */
#define TCR_2_b		0xfffe4000U
#define TIER_2_b	0xfffe4004U
#define TSR_2_b		0xfffe4005U
#define TCNT_2_h	0xfffe4006U

/*
 *  チャネル3
 */
#define TCR_3_b		0xfffe4200U
#define TIER_3_b	0xfffe4208U
#define TSR_3_b		0xfffe422cU
#define TCNT_3_h	0xfffe4210U

/*
 *  チャネル4
 */
#define TCR_4_b		0xfffe4201U
#define TIER_4_b	0xfffe4209U
#define TSR_4_b		0xfffe422dU
#define TCNT_4_h	0xfffe4212U


/*  タイマステータスレジスタ（TSR）  */
#define TCR_TCFV	0x10U	/*  オーバフローフラグ  */

/*  タイマインタラプトイネーブルレジスタ（TIER）  */
#define TIER_TCIEV	0x10U	/*  オーバフローインタラプトイネーブル  */


#endif /* TOPPERS_SH7711_H */
