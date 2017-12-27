$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2012-2014 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
$  Copyright (C) 2012-2014 by FUJISOFT INCORPORATED, JAPAN
$  Copyright (C) 2012-2013 by Spansion LLC, USA
$  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
$  Copyright (C) 2012-2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
$  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
$  Copyright (C) 2012-2014 by Sunny Giken Inc., JAPAN
$  Copyright (C) 2012-2014 by TOSHIBA CORPORATION, JAPAN
$  Copyright (C) 2012-2014 by Witz Corporation, JAPAN
$
$  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
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
$  $Id: prc_offset.tf 845 2017-09-25 04:33:56Z ertl-honda $
$

$
$       オフセットファイル生成用テンプレートファイル（V850用）
$

$
$  オフセット値のマクロ定義の生成
$
$DEFINE("TCB_p_tinib", offsetof_TCB_p_tinib)$
$DEFINE("TCB_curpri", offsetof_TCB_curpri)$
$DEFINE("TCB_sp", offsetof_TCB_sp)$
$DEFINE("TCB_pc", offsetof_TCB_pc)$

$DEFINE("TINIB_task", offsetof_TINIB_task)$
$DEFINE("TINIB_stk", offsetof_TINIB_stk)$
$DEFINE("TINIB_exepri", offsetof_TINIB_exepri)$
$DEFINE("TINIB_p_osapinib", offsetof_TINIB_p_osapinib)$

$DEFINE("ISRINIB_p_osapinib", offsetof_ISRINIB_p_osapinib)$
$DEFINE("ISRCB_p_isrinib", offsetof_ISRCB_p_isrinib)$

$DEFINE("CCB_p_runtsk", offsetof_CCB_p_runtsk)$
$DEFINE("CCB_p_schedtsk", offsetof_CCB_p_schedtsk)$
$DEFINE("CCB_p_runisr", offsetof_CCB_p_runisr)$
$DEFINE("CCB_p_currentosap", offsetof_CCB_p_currentosap)$
$DEFINE("CCB_kerflg", offsetof_CCB_kerflg)$
$DEFINE("CCB_ici_request_map", offsetof_CCB_ici_request_map)$
$DEFINE("CCB_ici_disreqflg", offsetof_CCB_ici_disreqflg)$
$DEFINE("CCB_callevel_stat", offsetof_CCB_callevel_stat)$

$DEFINE("ISRINIB_p_intinib", offsetof_ISRINIB_p_intinib)$
$DEFINE("INTINIB_remain_stksz", offsetof_INTINIB_remain_stksz)$
$DEFINE("CCB_except_nest_cnt", offsetof_CCB_except_nest_cnt)$
$DEFINE("CCB_current_iintpri", offsetof_CCB_current_iintpri)$
$DEFINE("CCB_nested_lock_os_int_cnt", offsetof_CCB_nested_lock_os_int_cnt)$
$DEFINE("CCB_current_intpri", offsetof_CCB_current_intpri)$
$DEFINE("CCB_trusted_hook_savedsp", offsetof_CCB_trusted_hook_savedsp)$
