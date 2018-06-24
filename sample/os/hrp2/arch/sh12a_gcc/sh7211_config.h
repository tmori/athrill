/*
 *	TOPPERS/HRP Kernel
 *		Toyohashi Open Platform for Embedded Real-Time Systems/
 *		High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2007 by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	Copyright (C) 2007-2010 by Industrial Technology Institute,
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
 *	$Id: sh7211_config.h 2142 2011-06-27 07:24:40Z mit-kimai $
 */

/*
 *		プロセッサ依存モジュール（SH7211用）
 *
 *	このインクルードファイルは，target_config.h（または，そこからインク
 *	ルードされるファイル）のみからインクルードされる．他のファイルから
 *	直接インクルードしてはならない．
 */

#ifndef TOPPERS_SH7211_CONFIG_H
#define TOPPERS_SH7211_CONFIG_H

#ifndef TOPPERS_TARGET_CONFIG_H
#error  target_config.h is not included!
#endif	/*  TOPPERS_TARGET_CONFIG_H  */

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
#define TMAX_INHNO		UINT_C(258)
#define TNUM_INH		(TMAX_INHNO - TMIN_INHNO + 1U)

/*
 * 割込み番号に関する定義
 */
#define TMIN_INTNO		UINT_C(64)
#define TMAX_INTNO		UINT_C(258)
#define TNUM_INT		(TMAX_INTNO - TMIN_INTNO + 1U)

/*	IRQ  */
#define TMIN_INTNO_IRQ 	64U
#define TMAX_INTNO_IRQ	71U

/*
 *	割込み要求ライン毎の優先度設定レジスタの設定情報
 */
#define IPR_INFO_TBL_DATA	\
	{IPR01_h, 12U}, /*  64 IRQ0 */ \
	{IPR01_h,  8U}, /*  65 IRQ1 */ \
	{IPR01_h,  4U}, /*  66 IRQ2 */ \
	{IPR01_h,  0U}, /*  67 IRQ3 */ \
	{IPR02_h, 12U}, /*  68 IRQ4 */ \
	{IPR02_h,  8U}, /*  69 IRQ5 */ \
	{IPR02_h,  4U}, /*  70 IRQ6 */ \
	{IPR02_h,  0U}, /*  71 IRQ7 */ \
\
	{0U, 0U},	  /*  72 */ \
	{0U, 0U},	  /*  73 */ \
	{0U, 0U},	  /*  74 */ \
	{0U, 0U},	  /*  75 */ \
	{0U, 0U},	  /*  76 */ \
	{0U, 0U},	  /*  77 */ \
	{0U, 0U},	  /*  78 */ \
	{0U, 0U},	  /*  79 */ \
	{0U, 0U},	  /*  80 */ \
	{0U, 0U},	  /*  81 */ \
	{0U, 0U},	  /*  82 */ \
	{0U, 0U},	  /*  83 */ \
	{0U, 0U},	  /*  84 */ \
	{0U, 0U},	  /*  85 */ \
	{0U, 0U},	  /*  86 */ \
	{0U, 0U},	  /*  87 */ \
	{0U, 0U},	  /*  88 */ \
	{0U, 0U},	  /*  89 */ \
	{0U, 0U},	  /*  90 */ \
	{0U, 0U},	  /*  91 */ \
\
	{IPR05_h, 4U}, /*  92 ADI */ \
\
	{0U, 0U},	  /*  93 */ \
	{0U, 0U},	  /*  94 */ \
	{0U, 0U},	  /*  95 */ \
	{0U, 0U},	  /*  96 */ \
	{0U, 0U},	  /*  97 */ \
	{0U, 0U},	  /*  98 */ \
	{0U, 0U},	  /*  99 */ \
	{0U, 0U},	  /* 100 */ \
	{0U, 0U},	  /* 101 */ \
	{0U, 0U},	  /* 102 */ \
	{0U, 0U},	  /* 103 */ \
	{0U, 0U},	  /* 104 */ \
	{0U, 0U},	  /* 105 */ \
	{0U, 0U},	  /* 106 */ \
	{0U, 0U},	  /* 107 */ \
\
	{IPR06_h, 12U}, /* 108 DMAC0 TEI0 */ \
	{IPR06_h, 12U}, /* 109 DMAC0 HEI0 */ \
\
	{0U, 0U},	  /* 110 */ \
	{0U, 0U},	  /* 111 */ \
\
	{IPR06_h,  8U}, /* 112 DMAC1 TEI1 */ \
	{IPR06_h,  8U}, /* 113 DMAC1 HEI1 */ \
\
	{0U, 0U},	  /* 114 */ \
	{0U, 0U},	  /* 115 */ \
\
	{IPR06_h,  4U}, /* 116 DMAC2 TEI2 */ \
	{IPR06_h,  4U}, /* 117 DMAC2 HEI2 */ \
\
	{0U, 0U},	  /* 118 */ \
	{0U, 0U},	  /* 119 */ \
\
	{IPR06_h,  0U}, /* 120 DMAC3 TEI3 */ \
	{IPR06_h,  0U}, /* 121 DMAC3 HEI3 */ \
\
	{0U, 0U},	  /* 122 */ \
	{0U, 0U},	  /* 123 */ \
\
	{IPR07_h, 12U}, /* 124 DMAC4 TEI4 */ \
	{IPR07_h, 12U}, /* 125 DMAC4 HEI4 */ \
\
	{0U, 0U},	  /* 126 */ \
	{0U, 0U},	  /* 127 */ \
\
	{IPR07_h,  8U}, /* 128 DMAC5 TEI5 */ \
	{IPR07_h,  8U}, /* 129 DMAC5 HEI5 */ \
\
	{0U, 0U},	  /* 130 */ \
	{0U, 0U},	  /* 131 */ \
\
	{IPR07_h,  4U}, /* 132 DMAC6 TEI6 */ \
	{IPR07_h,  4U}, /* 133 DMAC6 HEI6 */ \
\
	{0U, 0U},	  /* 134 */ \
	{0U, 0U},	  /* 135 */ \
\
	{IPR07_h,  0U}, /* 136 DMAC7 TEI7 */ \
	{IPR07_h,  0U}, /* 137 DMAC7 HEI7 */ \
\
	{0U, 0U},	  /* 138 */ \
	{0U, 0U},	  /* 139 */ \
\
	{IPR08_h, 12U}, /* 140 CMI0 */ \
\
	{0U, 0U},	  /* 141 */ \
	{0U, 0U},	  /* 142 */ \
	{0U, 0U},	  /* 143 */ \
\
	{IPR08_h,  8U}, /* 144 CMI1 */ \
\
	{0U, 0U},	  /* 145 */ \
	{0U, 0U},	  /* 146 */ \
	{0U, 0U},	  /* 147 */ \
\
	{IPR08_h,  4U}, /* 148 CMI */ \
\
	{0U, 0U},	  /* 149 */ \
	{0U, 0U},	  /* 150 */ \
	{0U, 0U},	  /* 151 */ \
\
	{IPR08_h,  0U}, /* 152 ITI */ \
\
	{0U, 0U},	  /* 153 */ \
	{0U, 0U},	  /* 154 */ \
	{0U, 0U},	  /* 155 */ \
\
	{IPR09_h, 12U}, /* 156 MTU0 TGI0A */ \
	{IPR09_h, 12U}, /* 157 MTU0 TGI0B */ \
	{IPR09_h, 12U}, /* 158 MTU0 TGI0C */ \
	{IPR09_h, 12U}, /* 159 MTU0 TGI0D */ \
	{IPR09_h,  8U}, /* 160 MTU0 TGI0V */ \
	{IPR09_h,  8U}, /* 161 MTU0 TGI0E */ \
	{IPR09_h,  8U}, /* 162 MTU0 TGI0F */ \
\
	{0U, 0U},	  /* 163 */ \
\
	{IPR09_h,  4U}, /* 164 MTU1 TGI1A */ \
	{IPR09_h,  4U}, /* 165 MTU1 TGI1B */ \
\
	{0U, 0U},	  /* 166 */ \
	{0U, 0U},	  /* 167 */ \
\
	{IPR09_h,  0U}, /* 168 MTU1 TGI1V */ \
	{IPR09_h,  0U}, /* 169 MTU1 TGI1U */ \
\
	{0U, 0U},	  /* 170 */ \
	{0U, 0U},	  /* 171 */ \
\
	{IPR10_h, 12U}, /* 172 MTU2 TGI2A */ \
	{IPR10_h, 12U}, /* 173 MTU2 TGI2B */ \
\
	{0U, 0U},	  /* 174 */ \
	{0U, 0U},	  /* 175 */ \
\
	{IPR10_h,  8U}, /* 176 MTU2 TGI2V */ \
	{IPR10_h,  8U}, /* 177 MTU2 TGI2U */ \
\
	{0U, 0U},	  /* 178 */ \
	{0U, 0U},	  /* 179 */ \
\
	{IPR10_h,  4U}, /* 180 MTU3 TGI3A */ \
	{IPR10_h,  4U}, /* 181 MTU3 TGI3B */ \
	{IPR10_h,  4U}, /* 182 MTU3 TGI3C */ \
	{IPR10_h,  4U}, /* 183 MTU3 TGI3D */ \
	{IPR10_h,  0U}, /* 184 MTU3 TGI3V */ \
\
	{0U, 0U},	  /* 185 */ \
	{0U, 0U},	  /* 186 */ \
	{0U, 0U},	  /* 187 */ \
\
	{IPR11_h, 12U}, /* 188 MTU4 TGI4A */ \
	{IPR11_h, 12U}, /* 189 MTU4 TGI4B */ \
	{IPR11_h, 12U}, /* 190 MTU4 TGI4C */ \
	{IPR11_h, 12U}, /* 191 MTU4 TGI4D */ \
	{IPR11_h,  8U}, /* 192 MTU4 TGI4V */ \
\
	{0U, 0U},	  /* 193 */ \
	{0U, 0U},	  /* 194 */ \
	{0U, 0U},	  /* 195 */ \
\
	{IPR11_h,  4U}, /* 196 MTU5 TGI5U */ \
	{IPR11_h,  4U}, /* 197 MTU5 TGI5V */ \
	{IPR11_h,  4U}, /* 198 MTU5 TGI5W */ \
\
	{0U, 0U},	  /* 199 */ \
\
	{IPR11_h,  0U}, /* 200 OEI1 */ \
	{IPR11_h,  0U}, /* 201 OEI2 */ \
\
	{0U, 0U},	  /* 202 */ \
	{0U, 0U},	  /* 203 */ \
\
	{IPR12_h, 12U}, /* 204 MTU3S TGI3A */ \
	{IPR12_h, 12U}, /* 205 MTU3S TGI3B */ \
	{IPR12_h, 12U}, /* 206 MTU3S TGI3C */ \
	{IPR12_h, 12U}, /* 207 MTU3S TGI3D */ \
	{IPR12_h,  8U}, /* 208 MTU3S TGI3V */ \
\
	{0U, 0U},	  /* 209 */ \
	{0U, 0U},	  /* 210 */ \
	{0U, 0U},	  /* 211 */ \
\
	{IPR12_h,  4U}, /* 212 MTU3S TGI3A */ \
	{IPR12_h,  4U}, /* 213 MTU3S TGI3B */ \
	{IPR12_h,  4U}, /* 214 MTU3S TGI3C */ \
	{IPR12_h,  4U}, /* 215 MTU3S TGI3D */ \
	{IPR12_h,  0U}, /* 216 MTU3S TGI3V */ \
\
	{0U, 0U},	  /* 217 */ \
	{0U, 0U},	  /* 218 */ \
	{0U, 0U},	  /* 219 */ \
\
	{IPR13_h, 12U}, /* 220 MTU5 TGI5U */ \
	{IPR13_h, 12U}, /* 221 MTU5 TGI5V */ \
	{IPR13_h, 12U}, /* 222 MTU5 TGI5W */ \
\
	{0U, 0U},	  /* 223 */ \
\
	{IPR13_h,  8U}, /* 224 OEI3 */ \
\
	{0U, 0U},	  /* 225 */ \
	{0U, 0U},	  /* 226 */ \
	{0U, 0U},	  /* 227 */ \
\
	{IPR13_h,  4U}, /* 228 IIC3 STPI */ \
	{IPR13_h,  4U}, /* 229 IIC3 NAKI */ \
	{IPR13_h,  4U}, /* 230 IIC3 RXI  */ \
	{IPR13_h,  4U}, /* 231 IIC3 TXI  */ \
	{IPR13_h,  4U}, /* 232 IIC3 TEI  */ \
\
	{0U, 0U},	  /* 233 */ \
	{0U, 0U},	  /* 234 */ \
	{0U, 0U},	  /* 235 */ \
	{0U, 0U},	  /* 236 */ \
	{0U, 0U},	  /* 237 */ \
	{0U, 0U},	  /* 238 */ \
	{0U, 0U},	  /* 239 */ \
\
	{IPR14_h, 12U}, /* 240 SCIF0 BRI0 */ \
	{IPR14_h, 12U}, /* 241 SCIF0 ERI0 */ \
	{IPR14_h, 12U}, /* 242 SCIF0 RXI0 */ \
	{IPR14_h, 12U}, /* 243 SCIF0 TXI0 */ \
\
	{IPR14_h,  8U}, /* 244 SCIF1 BRI1 */ \
	{IPR14_h,  8U}, /* 245 SCIF1 ERI1 */ \
	{IPR14_h,  8U}, /* 246 SCIF1 RXI1 */ \
	{IPR14_h,  8U}, /* 247 SCIF1 TXI1 */ \
\
	{IPR14_h,  4U}, /* 248 SCIF2 BRI2 */ \
	{IPR14_h,  4U}, /* 249 SCIF2 ERI2 */ \
	{IPR14_h,  4U}, /* 250 SCIF2 RXI2 */ \
	{IPR14_h,  4U}, /* 251 SCIF2 TXI2 */ \
\
	{IPR14_h,  0U}, /* 252 SCIF3 BRI3 */ \
	{IPR14_h,  0U}, /* 253 SCIF3 ERI3 */ \
	{IPR14_h,  0U}, /* 254 SCIF3 RXI3 */ \
	{IPR14_h,  0U}, /* 255 SCIF3 TXI3 */ \
\
	{IPR15_h, 12U}, /* 256 WAVEIF ERR  */ \
	{IPR15_h, 12U}, /* 257 WAVEIF WRXI */ \
	{IPR15_h, 12U}  /* 258 WAVEIF WTXI */


#ifndef TOPPERS_MACRO_ONLY
/*
 * 割込み優先レベル設定レジスタの初期化
 * 　ペリファラルの数によって、割込み優先レベル設定レジスタの構成が
 * 　異なるため、チップ依存部で定義する。
 */
Inline void
init_ipr(void)
{
	sil_wrh_mem((uint16_t *)IPR01_h, 0x0000U);
	sil_wrh_mem((uint16_t *)IPR02_h, 0x0000U);
	sil_wrh_mem((uint16_t *)IPR05_h, 0x0000U);
	sil_wrh_mem((uint16_t *)IPR06_h, 0x0000U);
	sil_wrh_mem((uint16_t *)IPR07_h, 0x0000U);
	sil_wrh_mem((uint16_t *)IPR08_h, 0x0000U);
	sil_wrh_mem((uint16_t *)IPR09_h, 0x0000U);
	sil_wrh_mem((uint16_t *)IPR10_h, 0x0000U);
	sil_wrh_mem((uint16_t *)IPR11_h, 0x0000U);
	sil_wrh_mem((uint16_t *)IPR12_h, 0x0000U);
	sil_wrh_mem((uint16_t *)IPR13_h, 0x0000U);
	sil_wrh_mem((uint16_t *)IPR14_h, 0x0000U);
	sil_wrh_mem((uint16_t *)IPR15_h, 0x0000U);
}

#endif /* TOPPERS_MACRO_ONLY */

/*
 *	プロセッサ依存モジュール（SH2A用）
 */
#include "sh12a_gcc/sh2a_config.h"

#endif /* TOPPERS_SH7211_CONFIG_H */
