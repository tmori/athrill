$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2012-2013 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
$  Copyright (C) 2012-2013 by FUJISOFT INCORPORATED, JAPAN
$  Copyright (C) 2012-2013 by FUJITSU VLSI LIMITED, JAPAN
$  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
$  Copyright (C) 2012-2013 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
$  Copyright (C) 2012-2013 by Renesas Electronics Corporation, JAPAN
$  Copyright (C) 2012-2013 by Sunny Giken Inc., JAPAN
$  Copyright (C) 2012-2013 by TOSHIBA CORPORATION, JAPAN
$  Copyright (C) 2012-2013 by Witz Corporation, JAPAN
$  Copyright (C) 2013 by Embedded and Real-Time Systems Laboratory
$               Graduate School of Information Science, Nagoya Univ., JAPAN
$
$  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
$  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
$  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
$  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
$      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
$      スコード中に含まれていること．
$  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
$      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
$      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
$      の無保証規定を掲載すること．
$  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
$      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
$      と．
$    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
$        作権表示，この利用条件および下記の無保証規定を掲載すること．
$    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
$        報告すること．
$  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
$      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
$      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
$      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
$      免責すること．
$
$  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
$  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
$  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
$  用する者に対して，AUTOSARパートナーになることを求めている．
$
$  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
$  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
$  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
$  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
$  の責任を負わない．
$
$  $Id: prc_offset.tf 189 2015-06-26 01:54:57Z t_ishikawa $
$

$
$       オフセットファイル生成用テンプレートファイル（V850用）
$

$
$  オフセット値のマクロ定義の生成
$
$DEFINE("TCB_p_tinib", offsetof_TCB_p_tinib)$
$DEFINE("TCB_curpri", offsetof_TCB_curpri)$
$  =begin modified for SC3
$DEFINE("TCB_ssp", offsetof_TCB_ssp)$
$DEFINE("TCB_usp", offsetof_TCB_usp)$
$DEFINE("TCB_priv_mode", offsetof_TCB_priv_mode)$
$  =end modified for SC3
$DEFINE("TCB_pc", offsetof_TCB_pc)$

$DEFINE("TINIB_task", offsetof_TINIB_task)$
$  =begin modified for SC3
$DEFINE("TINIB_sstksz", offsetof_TINIB_TSKINICTXB_sstksz)$
$DEFINE("TINIB_sstk_bottom", offsetof_TINIB_TSKINICTXB_sstk_bottom)$
$DEFINE("TINIB_ustksz", offsetof_TINIB_TSKINICTXB_stksz)$
$DEFINE("TINIB_ustk_bottom", offsetof_TINIB_TSKINICTXB_stk_bottom)$
$DEFINE("TINIB_start_ustk", offsetof_TINIB_STKMPUINFOB_start_ustk)$
$DEFINE("TINIB_limit_ustk", offsetof_TINIB_STKMPUINFOB_limit_ustk)$
$DEFINE("TINIB_p_osapcb", offsetof_TINIB_p_osapcb)$
$  =end modified for SC3
$DEFINE("TINIB_exepri", offsetof_TINIB_exepri)$

$DEFINE("INTINIB_remain_stksz", offsetof_INTINIB_remain_stksz)$
$DEFINE("ISRCB_p_isrinib", offsetof_ISRCB_p_isrinib)$

$  =begin modified for SC3
$DEFINE("ISRINIB_p_intinib", offsetof_ISRINIB_p_intinib)$
$DEFINE("ISRINIB_p_osapcb",         offsetof_ISRINIB_p_osapcb)$

$DEFINE("OSAPCB_p_osapinib", offsetof_OSAPCB_p_osapinib)$
 
$DEFINE("OSAPINIB_osap_trusted",    offsetof_OSAPINIB_osap_trusted)$
$DEFINE("OSAPINIB_osapmpu",         offsetof_OSAPINIB_osapmpu)$
$DEFINE("OSAPINIB_start_text",      offsetof_OSAPINIB_start_text)$
$DEFINE("OSAPINIB_limit_text",      offsetof_OSAPINIB_limit_text)$
$DEFINE("OSAPINIB_start_rosdata",   offsetof_OSAPINIB_start_rosdata)$
$DEFINE("OSAPINIB_limit_rosdata",   offsetof_OSAPINIB_limit_rosdata)$
$DEFINE("OSAPINIB_start_ram",       offsetof_OSAPINIB_start_ram)$
$DEFINE("OSAPINIB_limit_ram",       offsetof_OSAPINIB_limit_ram)$
$DEFINE("OSAPINIB_start_sram",      offsetof_OSAPINIB_start_sram)$
$DEFINE("OSAPINIB_limit_sram",      offsetof_OSAPINIB_limit_sram)$
$DEFINE("OSAPINIB_start_srpw",      offsetof_OSAPINIB_start_srpw)$
$DEFINE("OSAPINIB_limit_srpw",      offsetof_OSAPINIB_limit_srpw)$
$DEFINE("OSAPINIB_start_ssrpw",     offsetof_OSAPINIB_start_ssrpw)$
$DEFINE("OSAPINIB_limit_ssrpw",     offsetof_OSAPINIB_limit_ssrpw)$
$DEFINE("OSAPINIB_mpu_area_info",   offsetof_OSAPINIB_mpu_area_info)$
$DEFINE("OSAPINIB_tnum_mpu_area",   offsetof_OSAPINIB_tnum_mpu_area)$
$DEFINE("OSAPINIB_mprc",            offsetof_OSAPINIB_mprc)$
$  =end modified for SC3

