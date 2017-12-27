$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2007 by TAKAGI Nobuhisa
$  Copyright (C) 2007-2017 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
$  Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPANf
$  Copyright (C) 2011-2013 by Spansion LLC, USA
$  Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
$  Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
$  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
$  Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
$  Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
$  Copyright (C) 2011-2017 by Witz Corporation
$  Copyright (C) 2014-2016 by AISIN COMCRUISE Co., Ltd., JAPAN
$  Copyright (C) 2014-2016 by eSOL Co.,Ltd., JAPAN
$  Copyright (C) 2014-2017 by SCSK Corporation, JAPAN
$  Copyright (C) 2015-2017 by SUZUKI MOTOR CORPORATION
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
$  $Id: kernel.tf 842 2017-09-25 04:29:37Z ertl-honda $
$

$TRACE("EXEC kernel.tf")$

$ XMLのデータ構造に問題があるか
$FATAL = 0$

$ =====================================================================
$ boolean判定関数
$ =====================================================================

$FUNCTION IS_TRUE$
	$true_list = {1, "1", "true", "TRUE", "ON", "ENABLE"}$
	$false_list = {0, "0", "false", "FALSE", "OFF", "DISABLE"}$
	$error_flag = 1$
	
	$FOREACH false false_list$
		$IF EQ(false, ARGV[1])$
			$check_result = VALUE("FALSE", 0)$
			$error_flag = 0$
		$END$
	$END$
	$FOREACH true true_list$
		$IF EQ(true, ARGV[1])$
			$check_result = VALUE("TRUE", 1)$
			$error_flag = 0$
		$END$
	$END$

	$IF error_flag > 0$
		$ERROR ARGV[2]$$FORMAT(_("%1% must be boolean type"), ARGV[1])$$END$
	$END$

	$RESULT = check_result$
$END$

$ =====================================================================
$ HOOK.PROTECTIONHOOKがない場合にFALSEを入れる
$ =====================================================================

$IF !LENGTH(HOOK.PROTECTIONHOOK[1])$
	$HOOK.PROTECTIONHOOK[1] = VALUE("false", 0)$
$END$

$ =====================================================================
$ boolean変換
$ =====================================================================

$OS.STACKMONITORING[1] = IS_TRUE(OS.STACKMONITORING[1], OS.TEXT_LINE[1])$
$OS.GETSERVICEID[1] = IS_TRUE(OS.GETSERVICEID[1], OS.TEXT_LINE[1])$
$OS.PARAMETERACCESS[1] = IS_TRUE(OS.PARAMETERACCESS[1], OS.TEXT_LINE[1])$
$HOOK.STARTUPHOOK[1] = IS_TRUE(HOOK.STARTUPHOOK[1], HOOK.TEXT_LINE[1])$
$HOOK.SHUTDOWNHOOK[1] = IS_TRUE(HOOK.SHUTDOWNHOOK[1], HOOK.TEXT_LINE[1])$
$HOOK.PRETASKHOOK[1] = IS_TRUE(HOOK.PRETASKHOOK[1], HOOK.TEXT_LINE[1])$
$HOOK.POSTTASKHOOK[1] = IS_TRUE(HOOK.POSTTASKHOOK[1], HOOK.TEXT_LINE[1])$
$HOOK.ERRORHOOK[1] = IS_TRUE(HOOK.ERRORHOOK[1], HOOK.TEXT_LINE[1])$
$HOOK.PROTECTIONHOOK[1] = IS_TRUE(HOOK.PROTECTIONHOOK[1], HOOK.TEXT_LINE[1])$
$FOREACH schtblid SCHTBL.ID_LIST$
	$SCHTBL.REPEAT[schtblid] = IS_TRUE(SCHTBL.REPEAT[schtblid],SCHTBL.TEXT_LINE[schtblid])$
$END$
$FOREACH osapid OSAP.ID_LIST$
	$OSAP.TRUSTED[osapid] = IS_TRUE(OSAP.TRUSTED[osapid], OSAP.TEXT_LINE[osapid])$
$END$

$ =====================================================================
$ マスタコアの設定値とマスタコアIDが一致か
$ =====================================================================
$IF LENGTH(OS.MASTERCOREID[1]) && OS.MASTERCOREID[1] != OS_CORE_ID_MASTER$
	$FATAL = 1$
	$ERROR OS.TEXT_LINE[1]$$FORMAT(_("OsMasterCoreId `%1%'  is not equal to target-defined master core id(`%2%')"), OS.MASTERCOREID[1], +OS_CORE_ID_MASTER)$$END$
$END$


$ =====================================================================
$ OSAPのエラーチェックと関連づけ
$ =====================================================================

$FOREACH osapid OSAP.ID_LIST$
	$IF EQ(OSAP.TRUSTED[osapid], "FALSE")$
		$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsTrusted of OSApplication `%1%' must be TRUE in SC1-MC"), osapid)$$END$
	$END$

	$FOREACH tskid OSAP.TSK_LIST[osapid]$
		$IF LENGTH(TSK.OSAPID[tskid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("Task `%1%' belongs to another OSApplication `%2%'"), tskid, TSK.OSAPID[tskid])$$END$
		$ELSE$
			$TSK.OSAPID[tskid] = osapid$
		$END$
	$END$

	$FOREACH cntid OSAP.CNT_LIST[osapid]$
		$IF LENGTH(CNT.OSAPID[cntid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("Counter `%1%' belongs to another OSApplication `%2%'"), cntid, CNT.OSAPID[cntid])$$END$
		$ELSE$
			$CNT.OSAPID[cntid] = osapid$
		$END$
	$END$

	$FOREACH almid OSAP.ALM_LIST[osapid]$
		$IF LENGTH(ALM.OSAPID[almid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("Alarm `%1%' belongs to another OSApplication `%2%'"), almid, ALM.OSAPID[almid])$$END$
		$ELSE$
			$ALM.OSAPID[almid] = osapid$
		$END$
	$END$

	$FOREACH isrid OSAP.ISR_LIST[osapid]$
		$IF LENGTH(ISR.OSAPID[isrid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("ISR `%1%' belongs to another OSApplication `%2%'"), isrid, ISR.OSAPID[isrid])$$END$
		$ELSE$
			$ISR.OSAPID[isrid] = osapid$
		$END$
	$END$

	$FOREACH iciid OSAP.ICI_LIST[osapid]$
		$IF LENGTH(ICI.OSAPID[iciid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsInterCoreInterrupt `%1%' belongs to another OSApplication `%2%'"), iciid, ICI.OSAPID[iciid])$$END$
		$ELSE$
			$ICI.OSAPID[iciid] = osapid$
		$END$
	$END$

	$FOREACH schtblid OSAP.SCHTBL_LIST[osapid]$
		$IF LENGTH(SCHTBL.OSAPID[schtblid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("ScheduleTable `%1%' belongs to another OSApplication `%2%'"), schtblid, SCHTBL.OSAPID[schtblid])$$END$
		$ELSE$
			$SCHTBL.OSAPID[schtblid] = osapid$
		$END$
	$END$
$END$

$ =====================================================================
$ OSTK.STKがない場合にNULLを入れる
$ =====================================================================

$FOREACH ostkid OSTK.ID_LIST$
	$IF !LENGTH(OSTK.STK[ostkid])$
		$OSTK.STK[ostkid] = "NULL"$
	$END$
$END$


$ =====================================================================
$ TSKのエラーチェックと関連づけ
$ =====================================================================

$TRACE("ASSOCIATE TASK")$

$ TSK.AUTOSTARTをTSKにコピー
$FOREACH aid TSK.AUTOSTART.ID_LIST$
	$tskid                = TSK.AUTOSTART.PARENT[aid]$
	$TSK.APP_LIST[tskid]  = TSK.AUTOSTART.APP_LIST[aid]$
$END$


$FOREACH tskid TSK.ID_LIST$
$   // 省略された場合はNULLにする
	$IF !LENGTH(TSK.STK[tskid])$
		$TSK.STK[tskid] = "NULL"$
	$END$
	
	$IF LENGTH(TSK.STKSZ[tskid]) && TSK.STKSZ[tskid] != 0$
		$TSK.STKSZ[tskid] = VALUE(CONCAT(+TSK.STKSZ[tskid],"U"),+TSK.STKSZ[tskid])$
	$ELSE$
		$TSK.STKSZ[tskid] = VALUE(CONCAT(+DEFAULT_TASKSTKSZ,"U"),+DEFAULT_TASKSTKSZ)$
	$END$
$END$


$ =====================================================================
$ ISRのエラーチェックと関連づけ
$ =====================================================================

$TRACE("ASSOCIATE ISR")$

$FOREACH isrid ISR.ID_LIST$
$   // C1ISRはENABLEにする必要がある 
	$IF EQ(ISR.SOURCE[isrid], "DISABLE") && EQ(ISR.CATEGORY[isrid], "CATEGORY_1")$ 
		$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("OsIsrInterruptSource of C1ISR OsIsr(%1%) must include ENABLE"), isrid)$$END$ 
	$END$

	$IF EQ(ISR.SOURCE[isrid], "ENABLE")$ 
		$IF LENGTH(ISR.INTATR[isrid])$ 
			$ISR.INTATR[isrid] = VALUE(CONCAT(ISR.INTATR[isrid], " | ENABLE"), ISR.INTATR[isrid] | ENABLE)$ 
		$ELSE$ 
			$ISR.INTATR[isrid] = ENABLE$ 
		$END$
	$ELSE$
$		// 割込み要求ラインの有効/無効を制御出来ないにも関わらず，OsIsrInterruptSourceがDISABLEになっている場合はエラーとする 
		$IF !LENGTH(FIND(INTNO_CONTROLLABLE, ISR.INTNO[isrid]))$ 
			$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("Although the irq-line of ISR OsIsr(%1%) is not controllable, OsIsrInterruptSource of OsIsr(%1%) is DISABLE,  "), isrid)$$END$  
		$END$
	
		$IF LENGTH(ISR.INTATR[isrid])$ 
			$ISR.INTATR[isrid] = VALUE(CONCAT(ISR.INTATR[isrid], " | DISABLE"), ISR.INTATR[isrid] | DISABLE)$ 
		$ELSE$ 
			$ISR.INTATR[isrid] = DISABLE$
		$END$
	$END$

	$IF EQ(ISR.CATEGORY[isrid], "CATEGORY_1")$
		$IF LENGTH(ISR.STKSZ[isrid])$
			$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("OsIsrStackSize of C1ISR OsIsr(%1%) must not define"), isrid)$$END$
		$ELSE$
			$ISR.STKSZ[isrid] = 0$
		$END$
	$ELIF EQ(ISR.CATEGORY[isrid], "CATEGORY_2")$
		$IF LENGTH(ISR.STKSZ[isrid]) && (ISR.STKSZ[isrid] == 0)$
			$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("OsIsrStackSize of C2ISR OsIsr(%1%) should not defined zero"), isrid)$$END$
		$END$
$		// C2ISRスタックサイズのデフォルト設定
		$IF !LENGTH(ISR.STKSZ[isrid])$
			$ISR.STKSZ[isrid] = DEFAULT_ISRSTKSZ$
		$END$
	$END$
$END$


$ =====================================================================
$ DEF_ICIのエラーチェックと関連づけ
$ =====================================================================

$TRACE("ASSOCIATE ICISR")$

$FOREACH iciid ICI.ID_LIST$
	$IF LENGTH(ICI.STKSZ[iciid]) && (ICI.STKSZ[iciid] == 0)$
		$ERROR ICI.TEXT_LINE[iciid]$$FORMAT(_("OsInterCoreInterruptStackSize of ICISR OsInterCoreInterrupt(%1%) should not defined zero"), iciid)$$END$
	$END$
$	// ICISRスタックサイズのデフォルト設定
	$IF !LENGTH(ICI.STKSZ[iciid])$
		$ICI.STKSZ[iciid] = DEFAULT_ISRSTKSZ$
	$END$
$END$


$ =====================================================================
$ EVTに関する処理
$ =====================================================================

$FOREACH evtid EVT.ID_LIST$
$	// マスクが省略された場合はAUTOにする
	$IF !LENGTH(EVT.MASK[evtid])$
		$EVT.MASK[evtid] = AUTO$
	$END$
$END$


$ =====================================================================
$ CNTに関する処理
$ =====================================================================

$
$ 文字列が数値かどうかチェック
$
$FUNCTION VALIDATE_NUMBER$

$   // 指数部のチェック
    $e = REGEX_REPLACE(ARGV[1], "^[0-9]+\\.[0-9]+[eE][-]?[0-9]+$|^[0-9]+[eE][-]?[0-9]+$", "")$
    $IF EQ(e, ARGV[1])$
$       // 指数部がない場合
$       // 仮数部のチェック
        $e = REGEX_REPLACE(ARGV[1], "^[0-9]+\\.[0-9]+$", "")$
        $IF EQ(e, ARGV[1])$
$           // 少数部がない場合
$           // 整数部のチェック
            $e = REGEX_REPLACE(ARGV[1], "^[0-9]+$", "")$
            $IF EQ(e, ARGV[1])$
                $ERROR$
                    $FORMAT("%1% is invalid number\n", ARGV[1])$
                $END$
            $END$
        $END$
    $END$

$END$

$
$ 浮動小数点型の文字列を整数に変換
$
$FUNCTION FLOAT_TO_FIXINT$
    $num_org = ARGV[1]$
    $VALIDATE_NUMBER(num_org)$

$   // 指数部の変換
    $exp = REGEX_REPLACE(num_org, "^[0-9]+\\.[0-9]+[eE]([-]?[0-9]+)$", "$1")$
    $IF EQ(exp, num_org)$
        $exp = REGEX_REPLACE(num_org, "^[0-9]+[eE]([-]?[0-9]+)$", "$1")$
        $IF EQ(exp, num_org)$
$           // 指数表現でない
            $exp = 0$
        $ELSE$
$           // 指数表現
            $exp = ATOI(exp)$
            $num_org = REGEX_REPLACE(num_org, "^([0-9]+)[eE][-]?[0-9]+$", "$1")$
        $END$
    $ELSE$
$       // 指数表現
        $exp = ATOI(exp)$
        $num_org = REGEX_REPLACE(num_org, "^([0-9]+\\.[0-9]+)[eE][-]?[0-9]+$", "$1")$
    $END$

    $multiple = 1$
    $e = REGEX_REPLACE(num_org, "^[0-9]+\\.[0-9]+$", "")$
    $IF !EQ(e, num_org)$
$       // 小数点
$       // 仮数部の変換
$       // 小数点以上
        $num_l = REGEX_REPLACE(num_org, "^([0-9]+)\\.[0-9]+$", "$1")$
$       // 小数点以下
        $num_r_tmp = REGEX_REPLACE(num_org, "^[0-9]+\\.([0-9]+)$", "$1")$
        $num_r = "0"$
$       // 小数点以下の桁数を計算
        $WHILE !EQ(num_r_tmp, "")$
            $num_r = CONCAT(num_r, REGEX_REPLACE(num_r_tmp, "^(.).*$", "$1"))$
            $num_r_tmp = REGEX_REPLACE(num_r_tmp, "^.(.*)$", "$1")$
            $multiple = multiple * 10$
        $END$

$       // 仮数部を整数に変換
        $num_result = ATOI(CONCAT("0", num_l), 10) * multiple + ATOI(num_r, 10)$
$       // 指数部が正数の場合は，その分の基数を仮数部に乗ずる
    $ELSE$
$       // 整数
        $num_result = ATOI(num_org)$
    $END$

$   // 指数部が整数の場合
    $WHILE exp > 0$
        $IF multiple >= 10$
            $multiple = multiple / 10$
        $ELSE$
            $num_result = num_result * 10$
        $END$
        $exp = exp - 1$
    $END$
    
$   // 指数部が負数の場合は，返り値とする指数分に加える
    $WHILE exp < 0$
        $multiple = multiple * 10$
        $exp = exp + 1$
    $END$

$   // VALUE("整数化した数値が元の数値の何倍か", 元の数値を整数化した数値)を返す
    $RESULT = VALUE(FORMAT("%d", multiple), num_result)$
$END$

$FOREACH cntid CNT.ID_LIST$
	$IF EQ(CNT.CNTATR[cntid], "HARDWARE")$
		$IF !LENGTH(CNT.ISRID[cntid])$
			$FATAL = 1$
			$ERROR CNT.TEXT_LINE[cntid]$$FORMAT(_("OsCounterIsrRef of HARDWARE OsCounter(%1%) is not defined"), cntid)$$END$
		$END$

$		// NSPERTICKをSECONDSPERTICKから計算する
		$IF LENGTH(CNT.SECONDSPERTICK[cntid])$
                        $secondspertick = FLOAT_TO_FIXINT(CNT.SECONDSPERTICK[cntid])$
			$nspertick = (+secondspertick * 1000000000) / ATOI(secondspertick)$
			$CNT.NSPERTICK[cntid] = CONCAT(nspertick,"U")$
		$END$
	$END$
$END$


$ =====================================================================
$ ALMに関する処理
$ =====================================================================

$TRACE("ASSOCIATE ALARM")$

$ ALM.AUTOSTART.*からALM.*にコピー
$FOREACH aid ALM.AUTOSTART.ID_LIST$
	$almid                              = ALM.AUTOSTART.PARENT[aid]$
	$ALM.APP_LIST[almid]                = ALM.AUTOSTART.APP_LIST[aid]$
	$ALM.ALARMTIME[almid]               = ALM.AUTOSTART.ALARMTIME[aid]$
	$ALM.CYCLETIME[almid]               = ALM.AUTOSTART.CYCLETIME[aid]$
	$ALM.AUTOSTARTTYPE[almid]           = ALM.AUTOSTART.TYPE[aid]$
$END$

$FOREACH almid ALM.ID_LIST$
	$ALM.ALMATR_COUNT[almid] = 0$
$END$

$ ALM.ACTION.ACTIVATETASK.*からALM.*にコピー
$FOREACH almactid ALM.ACTION.ACTIVATETASK.ID_LIST$
	$almid                      = ALM.ACTION.PARENT[ALM.ACTION.ACTIVATETASK.PARENT[almactid]]$
	$ALM.ALMATR[almid]          = "ACTIVATETASK"$
	$ALM.TSKID[almid]           = ALM.ACTION.ACTIVATETASK.TSKID[almactid]$
	$ALM.ALMATR_COUNT[almid]    = ALM.ALMATR_COUNT[almid] + 1$
$END$

$ ALM.ACTION.SETEVENT.*からALM.*にコピー
$FOREACH almactid ALM.ACTION.SETEVENT.ID_LIST$
	$almid                      = ALM.ACTION.PARENT[ALM.ACTION.SETEVENT.PARENT[almactid]]$
	$ALM.ALMATR[almid]          = "SETEVENT"$
	$ALM.TSKID[almid]           = ALM.ACTION.SETEVENT.TSKID[almactid]$
	$ALM.EVTID[almid]           = ALM.ACTION.SETEVENT.EVTID[almactid]$
	$ALM.ALMATR_COUNT[almid]    = ALM.ALMATR_COUNT[almid] + 1$
$END$

$ ALM.ACTION.CALLBACK.*からALM.*にコピー
$FOREACH almactid ALM.ACTION.CALLBACK.ID_LIST$
	$almid                         = ALM.ACTION.PARENT[ALM.ACTION.CALLBACK.PARENT[almactid]]$
	$ALM.ALMATR[almid]             = "CALLBACK"$
	$ALM.CALLBACK[almid]           = ALM.ACTION.CALLBACK.CALLBACK[almactid]$
	$ALM.ALMATR_COUNT[almid]       = ALM.ALMATR_COUNT[almid] + 1$
$END$

$ ALM.ACTION.INCREMENTCOUNTER.*からALM.*にコピー
$FOREACH almactid ALM.ACTION.INCREMENTCOUNTER.ID_LIST$
	$almid                      = ALM.ACTION.PARENT[ALM.ACTION.INCREMENTCOUNTER.PARENT[almactid]]$
	$ALM.ALMATR[almid]          = "INCREMENTCOUNTER"$
	$ALM.INCID[almid]           = ALM.ACTION.INCREMENTCOUNTER.INCID[almactid]$
	$ALM.ALMATR_COUNT[almid]    = ALM.ALMATR_COUNT[almid] + 1$
$END$

$FOREACH almid ALM.ID_LIST$
$ 	//ALM.ACTION.*を2つ以上定義していないか
	$IF ALM.ALMATR_COUNT[almid] != 1$
		$FATAL = 1$
		$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmAction of OsAlarm(%1%) have too many sub container"), almid)$$END$
	$END$
$END$


$ =====================================================================
$ SCHTBLに関する処理
$ =====================================================================

$TRACE("ASSOCIATE SCHTBL")$

$ 省略された場合はNONEにする
$FOREACH schtblid SCHTBL.ID_LIST$
	$SCHTBL.SYNCSTRATEGY[schtblid] = "NONE"$
$END$

$ SCHTBL.AUTOSTART.*からSCHTBL.*にコピー
$FOREACH aid SCHTBL.AUTOSTART.ID_LIST$
	$schtblid                                 = SCHTBL.AUTOSTART.PARENT[aid]$
	$SCHTBL.APP_LIST[schtblid]                = SCHTBL.AUTOSTART.APP_LIST[aid]$
	$IF LENGTH(SCHTBL.AUTOSTART.STARTTICK[aid])$
		$SCHTBL.STARTTICK[schtblid]           = SCHTBL.AUTOSTART.STARTTICK[aid]$
	$ELSE$
$		// 省略されたことを示す
		$SCHTBL.STARTTICK[schtblid]           = "NULL"$
	$END$
	$SCHTBL.AUTOSTARTTYPE[schtblid]           = SCHTBL.AUTOSTART.TYPE[aid]$
$END$

$FOREACH syncid SCHTBL.SYNC.ID_LIST$
	$schtblid = SCHTBL.SYNC.PARENT[syncid]$
	$SCHTBL.SYNCSTRATEGY[schtblid] = SCHTBL.SYNC.STRATEGY[syncid]$
$END$


$ =====================================================================
$ EXPPTACTに関する処理
$ =====================================================================

$ SCHTBL.EXPPT.ACTIVATETASK.*からEXPPTACTにコピー
$i = 0$
$FOREACH acttskid SCHTBL.EXPPT.ACTIVATETASK.ID_LIST$
	$expid                          = SCHTBL.EXPPT.ACTIVATETASK.PARENT[acttskid]$
	$schtblid                       = SCHTBL.EXPPT.PARENT[expid]$

	$EXPPTACT.SCHTBLID[i]           = schtblid$
	$EXPPTACT.EXPPT[i]              = expid$
	$EXPPTACT.OFFSET[i]             = SCHTBL.EXPPT.OFFSET[expid]$
	$EXPPTACT.TSKID[i]              = SCHTBL.EXPPT.ACTIVATETASK.TSKID[acttskid]$
	$EXPPTACT.EXPIREATR[i]          = "ACTIVATETASK"$

	$EXPPTACT.ID_LIST               = APPEND(EXPPTACT.ID_LIST, VALUE(acttskid,i))$
	$i                              = i + 1$
$END$

$ SCHTBL.EXPPT.SETEVENT.*からEXPPTACTにコピー
$FOREACH setevid SCHTBL.EXPPT.SETEVENT.ID_LIST$
	$expid                          = SCHTBL.EXPPT.SETEVENT.PARENT[setevid]$
	$schtblid                       = SCHTBL.EXPPT.PARENT[expid]$

	$EXPPTACT.SCHTBLID[i]           = schtblid$
	$EXPPTACT.EXPPT[i]              = expid$
	$EXPPTACT.OFFSET[i]             = SCHTBL.EXPPT.OFFSET[expid]$
	$EXPPTACT.TSKID[i]              = SCHTBL.EXPPT.SETEVENT.TSKID[setevid]$
	$EXPPTACT.EVTID[i]              = SCHTBL.EXPPT.SETEVENT.EVTID[setevid]$
	$EXPPTACT.EXPIREATR[i]          = "SETEVENT"$

	$EXPPTACT.ID_LIST               = APPEND(EXPPTACT.ID_LIST, VALUE(setevid,i))$
	$i                              = i + 1$
$END$


$ =====================================================================
$ CRE_SPNのエラーチェックと関連づけ
$ =====================================================================
$FOREACH spnid SPN.ID_LIST$
	$SPN.SPN[spnid] = spnid$
$END$
$FOREACH spnid SPN.ID_LIST$
$	// LOCKMETHODの内容チェック（"LOCK_ALL_INTERRUPTS","LOCK_CAT2_INTERRUPTS","LOCK_NOTHING"以外ならエラー）
	$IF !(EQ(SPN.LOCKMETHOD[spnid], "LOCK_ALL_INTERRUPTS") || EQ(SPN.LOCKMETHOD[spnid], "LOCK_CAT2_INTERRUPTS") || EQ(SPN.LOCKMETHOD[spnid], "LOCK_NOTHING"))$
		$ERROR SPN.TEXT_LINE[spnid]$$FORMAT(_("OsSpinlockLockMethod of OsSpinlock must be LOCK_ALL_INTERRUPTS or LOCK_CAT2_INTERRUPTS or LOCK_NOTHING"))$$END$
	$END$

$	// SUCCESSORのチェック
	$IF LENGTH(SPN.SUCCESSOR[spnid]) && !EQ(SPN.SUCCESSOR[spnid],"NULL")$
		$next_spn = SPN.SUCCESSOR[spnid]$
		$find = 0$

$		// SUCCESSORとSPNIDの関連づけ
		$FOREACH temp SPN.ID_LIST$
			$IF EQ(SPN.SPN[temp] , next_spn)$
				$SPN.SUCCESSOR_ID[spnid] = +temp$
				$find = 1$
			$END$
		$END$
		
$		// SUCCESSORが存在しているか？
		$IF !find$
			$ERROR SPN.TEXT_LINE[spnid]$$FORMAT(_("illeagl %1% `%2%\' in %3%"),"successor", SPN.SUCCESSOR[spnid], "OsSpinlockSuccessor")$$END$
		$END$
	$END$
	
	$new_list = {}$
	$FOREACH osap SPN.ACS_OSAP_LIST[spnid]$
		$find = 0$
		$FOREACH osapid OSAP.ID_LIST$
			$IF EQ(osap, osapid)$
				$new_list = APPEND(new_list, osapid)$
				$find = 1$
			$END$
		$END$
$		// osapが存在するか．
		$IF !find$
			$ERROR SPN.TEXT_LINE[spnid]$$FORMAT(_("illegal %1% `%2%\'"), "OSApplication", osap)$$END$
		$END$
	$END$
	$SPN.ACS_OSAP_LIST[spnid] = new_list$

$	// osapが指定されているか？
	$IF !LENGTH(SPN.ACS_OSAP_LIST[spnid])$
		$ERROR SPN.TEXT_LINE[spnid]$$FORMAT(_("OsSpinlockAccessingApplication of OsSpinlock(%1%) is not defined"), spnid)$$END$
	$END$

$   //ネイティブロック方式の場合のチェック
	$IF EQ(+TTYPE_SPN, +NATIVE_SPN)$
		$IF (LENGTH(SPN.ID_LIST) > TMAX_NATIVE_SPN)$
			$ERROR SPN.TEXT_LINE[TMAX_NATIVE_SPN + 1]$E_NORES: $FORMAT(_("maximum number of spin lock is %1%."), +TMAX_NATIVE_SPN)$$END$
		$END$
	$END$
$END$


$ =====================================================================
$ IOCに関する処理
$ =====================================================================

$ OsIocCommunication
$FOREACH iocid IOC.ID_LIST$
$ 	// 省略されたパラメータにNULLを入れる
	$IF !LENGTH(IOC.BUFLEN[iocid])$
		$IOC.BUFLEN[iocid] = VALUE("NULL", 0)$
	$END$
$END$

$ OsIocDataProperties
$FOREACH iocdataid IOCDATA.ID_LIST$
$ 	// IOCDATA.PARENTをIOCDATA.IOCへコピー
	$IOCDATA.IOC[iocdataid]  = IOCDATA.PARENT[iocdataid]$

$ 	// 省略されたパラメータにNULLを入れる
	$IF !LENGTH(IOCDATA.INDEX[iocdataid])$
		$IOCDATA.INDEX[iocdataid] = "NULL"$
	$END$
	$IF !LENGTH(IOCDATA.INITVAL[iocdataid])$
		$IOCDATA.INITVAL[iocdataid] = "NULL"$
	$END$
$END$

$ OsIocSenderProperties
$FOREACH iocsndid IOCSND.ID_LIST$
$ 	// IOCSND.PARENTをIOCSND.IOCへコピー
	$IOCSND.IOC[iocsndid]   = IOCSND.PARENT[iocsndid]$

$ 	// 省略されたパラメータにNULLを入れる
	$IF !LENGTH(IOCSND.SENDERID[iocsndid])$
		$IOCSND.SENDERID[iocsndid] = "NULL"$
	$END$
	$IF !LENGTH(IOCSND.IMPLKIND[iocsndid])$
		$IOCSND.IMPLKIND[iocsndid] = "NULL"$
	$END$
$END$

$ OsIocReceiverProperties
$FOREACH iocrcvid IOCRCV.ID_LIST$
$ 	// IOCRCV.PARENTをIOCRCV.IOCへコピー
	$IOCRCV.IOC[iocrcvid] = IOCRCV.PARENT[iocrcvid]$

$ 	// 省略されたパラメータにNULLを入れる
	$IF !LENGTH(IOCRCV.IMPLKIND[iocrcvid])$
		$IOCRCV.IMPLKIND[iocrcvid] = "NULL"$
	$END$
$END$

$IF FATAL$
	$DIE()$
$END$


$ =====================================================================
$ OSのエラーチェックと内部データの生成
$ =====================================================================

$IF !(EQ(OS.STATUS[1], "EXTENDED") || EQ(OS.STATUS[1], "STANDARD"))$
	$ERROR OS.TEXT_LINE[1]$$FORMAT(_("OsStatus must be STANDARD or EXTENDED"))$$END$
$END$

$IF LENGTH(OS.SC[1]) && !EQ(OS.SC[1], "SC1")$
	$ERROR OS.TEXT_LINE[1]$$FORMAT(_("OsScalabilityClass must be SC1"))$$END$
$END$

$ コア数
$IF LENGTH(OS.NUMCORE[1])$
	$IF (OS.NUMCORE[1] < 1) || (OS.NUMCORE[1] > 65535)$
		$ERROR OS.TEXT_LINE[1]$$FORMAT(_("illegal %1% `%2%\'"), "OsNumberOfCores", OS.NUMCORE[1])$$END$
	$END$

$	// コア数のハードウェア定義のエラーチェック
	$IF (OS.NUMCORE[1] > TNUM_HWCORE)$
		$ERROR OS.TEXT_LINE[1]$$FORMAT(_("OsNumberOfCores(%1%) is more than the core number(%2%) of target hardware"), OS.NUMCORE[1], +TNUM_HWCORE)$$END$
	$END$

	$OS_NUMBER_OF_CORE_CORES = OS.NUMCORE[1]$
$ELSE$
	$OS_NUMBER_OF_CORE_CORES = 1$
$END$

$TMAX_COREID = OS_NUMBER_OF_CORE_CORES -1$

$ =====================================================================
$ OSStackのエラーチェックと内部データの生成
$ =====================================================================

$FOREACH ostkid OSTK.ID_LIST$
	$IF OSTK.CORE[ostkid] > TMAX_COREID$
		$ERROR OSTK.TEXT_LINE[ostkid]$$FORMAT(_("illegal %1% `%2%\' more than OsNumberOfCores(%3%\)"), "core", OSTK.CORE[ostkid], OS_NUMBER_OF_CORE_CORES)$$END$
	$END$
$END$

$FOREACH coreid RANGE(0, TMAX_COREID)$
	$cnt = 0$
	$FOREACH ostkid OSTK.ID_LIST$
		$IF coreid == OSTK.CORE[ostkid]$
			$cnt = cnt + 1$
			$OSTK.COREID[coreid] = ostkid$
		$END$
	$END$

$ 	//1つのコアに複数指定がある
	$IF cnt > 1$
		$ERROR$$FORMAT(_("too many %1% CoreID(%2%)"), "OsOsStack",coreid)$$END$
	$END$
$END$


$ =====================================================================
$ HookStackのエラーチェックと内部データの生成
$ =====================================================================

$FOREACH hstkid HSTK.ID_LIST$
	$IF HSTK.CORE[hstkid] > TMAX_COREID$
		$ERROR HSTK.TEXT_LINE[hstkid]$$FORMAT(_("illegal %1% `%2%\' more than OsNumberOfCores(%3%\)"), "core", HSTK.CORE[hstkid], OS_NUMBER_OF_CORE_CORES)$$END$
	$END$
$END$

$FOREACH coreid RANGE(0, TMAX_COREID)$
	$cnt = 0$
	$FOREACH hstkid HSTK.ID_LIST$
		$IF coreid == HSTK.CORE[hstkid]$
			$cnt = cnt + 1$
			$HSTK.COREID[coreid] = hstkid$
		$END$
	$END$

$ 	//1つのコアに複数指定がある
	$IF cnt > 1$
		$ERROR$$FORMAT(_("too many %1% CoreID(%2%)"), "OsHookStack",coreid)$$END$
	$END$
$END$


$ =====================================================================
$ OSAPのエラーチェックと内部データの生成
$ =====================================================================

$TRACE("OSAP CHECK")$

$i = 0$
$FOREACH osapid OSAP.ID_LIST$
	$OSAP.ID[osapid] = i$
	$i = i + 1$

	$IF OSAP.CORE[osapid] > TMAX_COREID$
		$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("illegal %1% `%2%\' more than OsNumberOfCores(%3%\)"), "core", OSAP.CORE[osapid], OS_NUMBER_OF_CORE_CORES)$$END$
	$END$
$END$


$ =====================================================================
$ タスクの関連づけ
$ =====================================================================

$FOREACH tskid TSK.ID_LIST$
$	// TSKとRESの関連づけ
	$FOREACH resid TSK.RES_LIST[tskid]$
		$RES.TSK_LIST[resid] = APPEND(RES.TSK_LIST[resid], tskid)$
	$END$
$	// TSKとEVTの関連づけ
	$FOREACH evtid TSK.EVT_LIST[tskid]$
		$EVT.TSK_LIST[evtid] = APPEND(EVT.TSK_LIST[evtid], tskid)$
	$END$
$	// タスクがOSAPに所属するかチェックする．
	$IF !LENGTH(TSK.OSAPID[tskid])$
		$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTask(%1%) does not belong OsApplication)"), tskid)$$END$
		$DIE()$
	$END$
$END$


$ =====================================================================
$ ISR関連のエラーチェックと内部データの生成
$ =====================================================================

$TRACE("ISR CHECK")$

$FOREACH isrid ISR.ID_LIST$
$	// INTNOが他のISRと重複している場合
	$FOREACH isrid2 ISR.ID_LIST$
		$IF ISR.INTNO[isrid] == ISR.INTNO[isrid2] && isrid2 < isrid$
			$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("OsIsrInterruptNumber(%1%) of OsIsr(%2%) is duplicated"), ISR.INTNO[isrid], isrid)$$END$
		$END$
	$END$

$	// intpriが割込み優先度として正しくない場合
	$IF (ISR.INTPRI[isrid] > TNUM_INTPRI) || (ISR.INTPRI[isrid] < 1)$
		$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("illegal OsIsrInterruptPriority(%1%) of OsIsr(%2%)"), ISR.INTPRI[isrid], isrid)$$END$
	$END$

	$IF !(EQ(ISR.CATEGORY[isrid], "CATEGORY_1") || EQ(ISR.CATEGORY[isrid], "CATEGORY_2"))$
		$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("OsIsrCategory of OsIsr(%1%) must be CATEGORY_1 or CATEGORY_2"), isrid)$$END$
	$END$

$	// ISRとRESの関連づけ
	$FOREACH resid ISR.RES_LIST[isrid]$
		$RES.ISR_LIST[resid] = APPEND(RES.ISR_LIST[resid], isrid)$
	$END$
$END$


$ C2ISRの最低優先度・最高優先度
$FOREACH coreid RANGE(0, TMAX_COREID)$
	$MAX_PRI_ISR2[coreid] = -1$
	$MIN_PRI_ISR2[coreid] = 0$
$END$

$i = 0$
$FOREACH isrid ISR.ID_LIST$
$	// ISRがOSAPに所属するかチェックする．
	$IF !LENGTH(ISR.OSAPID[isrid])$
		$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("OsIsr(%1%) does not belong OsApplication)"), isrid)$$END$
		$DIE()$
	$END$

	$IF EQ(ISR.CATEGORY[isrid], "CATEGORY_2")$
		$ISR.ID[isrid] = i$
		$i = i + 1$
		$ISR2.ID_LIST = APPEND(ISR2.ID_LIST, isrid)$
		$coreid = OSAP.CORE[ISR.OSAPID[isrid]]$
		$IF LENGTH(MIN_PRI_ISR2[coreid])$
			$IF MIN_PRI_ISR2[coreid] > -ISR.INTPRI[isrid]$
				$MIN_PRI_ISR2[coreid] = -ISR.INTPRI[isrid]$
			$END$
		$END$
		$ISR.INT_ENTRY[isrid] = CONCAT("kernel_inthdr_", FORMAT("0x%x",+ISR.INTNO[isrid]))$

$		// INTNOが割込み番号として正しくない場合
		$IF !LENGTH(FIND(INTNO_CREISR2_VALID, ISR.INTNO[isrid]))$
			$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("illegal OsIsrInterruptNumber(%1%) of OsIsr(%2%)"), ISR.INTNO[isrid], isrid)$$END$
		$END$
	$END$
$END$

$ C2ISRが定義されていない場合
$FOREACH coreid RANGE(0, TMAX_COREID)$
	$IF MIN_PRI_ISR2[coreid] == 0$
		$MAX_PRI_ISR2[coreid] = 0$
	$END$
$END$

$ =====================================================================
$ ISRの一部としてICISR関連のエラーチェックと内部データの生成
$ =====================================================================

$FOREACH iciid ICI.ID_LIST$
$	// ICISRとRESの関連づけ
	$FOREACH resid ICI.RES_LIST[iciid]$
		$RES.ICI_LIST[resid] = APPEND(RES.ICI_LIST[resid], iciid)$
	$END$
$END$

$FOREACH  iciid ICI.ID_LIST$
$	// ICISRがOSAPに所属するかチェックする．
	$IF !LENGTH(ICI.OSAPID[iciid])$
		$ERROR ICI.TEXT_LINE[iciid]$$FORMAT(_("OsInterCoreInterrupt(%1%) does not belong OsApplication)"), iciid)$$END$
		$DIE()$
	$END$
$END$

$ // ICISRをコア順にソートし，ビットパターンを決める
$new_list = {}$
$id_count = 0$
$FOREACH coreid RANGE(0, TMAX_COREID)$
$	// ICISRのIDはC2ISRのIDの続きから振る
	$CORE.ICISR_NUM[coreid] = 0$
	$bit_count = 0$
	$FOREACH iciid ICI.ID_LIST$
		$IF OSAP.CORE[ICI.OSAPID[iciid]] == coreid$
			$ICI.ID[iciid] = i$
			$i = i + 1$
			$new_list = APPEND(new_list, iciid)$
			$ICI.BITPTN[iciid] = 1 << bit_count$
$			// コア毎の先頭ICIのID及び初期化管理ブロックの添字を保持
			$IF bit_count == 0$
				$CORE.ICIID[coreid] = iciid$
				$CORE.ICIID_CNT[coreid] = id_count$
			$END$
			$id_count = id_count + 1$
			$bit_count = bit_count + 1$
		$END$
	$END$
$	// コア毎のICISRの数を保持する
	$CORE.ICISR_NUM[coreid] = bit_count$
	$IF bit_count > 32$
		$ERROR$$FORMAT(_("too many OsInterCoreInterrupt(%1% > %2%) are defined in core %3%"), bit_count, 32, coreid)$$END$
	$END$
$END$
$ICI.ID_LIST = new_list$

$ C1ISRの最低優先度
$MAX_PRI_ISR1 = -1$
$FOREACH coreid RANGE(0, TMAX_COREID)$
	$IF MAX_PRI_ISR1 >= MIN_PRI_ISR2[coreid]$
		$MAX_PRI_ISR1 = MIN_PRI_ISR2[coreid] - 1$
	$END$
$END$

$
$ コア間割り込みを考慮したC1ISRの最低優先度
$
$FOREACH intpri INTPRI_ICI_LIST$
	$IF MAX_PRI_ISR1 >= (-intpri)$
		$MAX_PRI_ISR1 = (-intpri) - 1$
	$END$
$END$

$ C1ISRの最高優先度
$MIN_PRI_ISR1 = -TNUM_INTPRI$


$FOREACH isrid ISR.ID_LIST$
	$IF EQ(ISR.CATEGORY[isrid], "CATEGORY_1")$
		$ISR1.ID_LIST = APPEND(ISR1.ID_LIST, isrid)$
		$ISR.ID[isrid] = i$
		$i = i + 1$

$		// ISR1の割込み優先度がMIN_PRI_ISR2以上である場合
		$IF -ISR.INTPRI[isrid] > MAX_PRI_ISR1$
			$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("OsIsrInterruptPriority(%1%) of OsIsr(%2%) is lower than or equal to C2ISR(or inter-core interrupt) min priority(%3%)"), ISR.INTPRI[isrid], isrid, -(MAX_PRI_ISR1+1))$$END$
		$END$
		$ISR.INT_ENTRY[isrid] = CONCAT("C1ISRMain", isrid)$

$		// INTNOが割込み番号として正しくない場合
		$IF !LENGTH(FIND(INTNO_VALID, ISR.INTNO[isrid]))$
			$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("illegal OsIsrInterruptNumber(%1%) of OsIsr(%2%)"), ISR.INTNO[isrid], isrid)$$END$
		$END$
	$END$
$END$


$ ISR.ID_LISTをC2ISR, C1ISRの順序にする
$ISR.ID_LIST = APPEND(ISR2.ID_LIST, ISR1.ID_LIST)$


$FOREACH isrid ISR1.ID_LIST$
$	// C1ISRはリソースを取得できない
	$IF LENGTH(ISR.RES_LIST[isrid])$
		$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("C1ISR(%1%) can't use RESOURCE"), isrid)$$END$
	$END$
$END$


$FOREACH isrid ISR2.ID_LIST$
$	// C2ISRは内部リソースを取得できない
$	// リンクリソースは標準リソースにリンクするものであるためリンクリソースであればエラーとする．

	$FOREACH resid ISR.RES_LIST[isrid]$
		$IF EQ(RES.RESATR[resid], "INTERNAL") || EQ(RES.RESATR[resid], "LINKED")$
			$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("OsIsrResourceRef(%1%) of OsIsr(%2%) must be standard resource"), resid, isrid)$$END$
		$END$
	$END$
$END$


$ =====================================================================
$ リソース関連のエラーチェックと内部データの生成
$ =====================================================================

$TRACE("RESOURCE CHECK")$

$FOREACH resid RES.ID_LIST$
$	// 関連付けフラグの初期化(リソースがどのタスク/C2ISRにも関連付けされていない場合，warning出力し，オブジェクトを生成しない)
	$RES.RELATION_FLAG[resid] = (LENGTH(RES.TSK_LIST[resid]) || LENGTH(RES.ISR_LIST[resid]) || LENGTH(RES.ICI_LIST[resid]))$
$END$


$FOREACH resid RES.ID_LIST$
	$resatr = RES.RESATR[resid]$
$	// resatrの内容チェック（"STANDARD","INTERNAL","LINKED"以外ならエラー）
	$IF !(EQ(resatr, "STANDARD") || EQ(resatr, "INTERNAL") || EQ(resatr, "LINKED"))$
		$ERROR RES.TEXT_LINE[resid]$$FORMAT(_("OsResourceProperty of OsResource(%1%) must be STANDARD or INTERNAL or LINKED"), resid)$$END$
	$END$

	$IF EQ(RES.RESATR[resid], "LINKED")$
		$lnkresid = RES.LINKEDRESID[resid]$

		$IF !LENGTH(lnkresid)$
			$ERROR RES.TEXT_LINE[resid]$$FORMAT(_("OsResourceLinkedResourceRef of OsResource(%1%) is not defined"), resid)$$END$
			$DIE()$
		$END$

		$RES.RELATION_FLAG[resid] = RES.RELATION_FLAG[lnkresid]$

$		// 被リンク側の属性が標準リソースか．
		$IF !EQ(RES.RESATR[lnkresid], "STANDARD")$
			$ERROR RES.TEXT_LINE[resid]$$FORMAT(_("OsResourceLinkedResourceRef(%1%) of OsResource(%2%) must be standard resource"), lnkresid, resid)$$END$
		$END$
	$END$

	$IF !RES.RELATION_FLAG[resid]$
		$WARNING RES.TEXT_LINE[resid]$$FORMAT(_("OsResource(%1%) is not related to any TASK, ISR or ICISR"), resid)$$END$
	$END$
$END$


$
$ リソースのOS内部で用いるIDを割り当てる
$ 内部リソース以外(標準，リンク)からIDを割り当てる
$
$i = 0$
$tnum_intres = 0$
$new_id_list = {}$
$FOREACH resid RES.ID_LIST$
	$IF !EQ(RES.RESATR[resid], "INTERNAL") && RES.RELATION_FLAG[resid]$
		$RES.ID[resid] = i$
		$i = i + 1$
		$new_id_list = APPEND(new_id_list, resid)$
	$END$
$END$
$STDRES.ID_LIST = new_id_list$
$FOREACH resid RES.ID_LIST$
	$IF EQ(RES.RESATR[resid], "INTERNAL") && RES.RELATION_FLAG[resid]$
		$RES.ID[resid] = i$
		$i = i + 1$
		$tnum_intres = tnum_intres + 1$
		$new_id_list = APPEND(new_id_list, resid)$
	$END$
$END$
$RES.ID_LIST = new_id_list$


$FOREACH resid RES.ID_LIST$
	$IF !EQ(RES.RESATR[resid], "LINKED")$

$		// リソースの上限優先度の計算(タスク)
		$ceilpri = 0$
		$FOREACH tskid RES.TSK_LIST[resid]$
			$tskpri = TSK.PRIORITY[tskid]$
			$IF ceilpri < tskpri$
				$ceilpri = tskpri$
			$END$
		$END$
		$RES.CEILPRI[resid] = ceilpri$

$		// リソースの上限優先度の計算(ISR)
$		// ISRの優先度は-1〜としている
		$ceilpri = 0$
		$FOREACH isrid RES.ISR_LIST[resid]$
			$intpri = -ISR.INTPRI[isrid]$
			$IF ceilpri > intpri$
				$ceilpri = intpri$
			$END$
		$END$
$		// ISRを用いるリソースの場合のみ更新
		$IF ceilpri < 0$
			$RES.CEILPRI[resid] = ceilpri$
		$END$

$		// リソースの上限優先度の計算(ICISR)
$		// ICISRの優先度は-1〜としている
		$ceilpri = 0$
		$FOREACH iciid RES.ICI_LIST[resid]$
			$intpri = -AT(INTPRI_ICI_LIST, OSAP.CORE[ICI.OSAPID[iciid]])$
			$IF ceilpri > intpri$
				$ceilpri = intpri$
			$END$
		$END$
$		// ICISRを用いるリソースの場合のみ更新
		$IF (ceilpri < 0) && (ceilpri < RES.CEILPRI[resid])$
			$RES.CEILPRI[resid] = ceilpri$
		$END$
	$END$
$END$


$ リソースの割り付けコアの設定
$FOREACH resid RES.ID_LIST$
	$RES.CORE[resid] = 0xff$
	$IF !EQ(RES.RESATR[resid], "LINKED")$
		$IF LENGTH(RES.TSK_LIST[resid])$
			$tskid = AT(RES.TSK_LIST[resid],0)$
			$RES.CORE[resid] = OSAP.CORE[TSK.OSAPID[tskid]]$
		$ELIF LENGTH(RES.ISR_LIST[resid])$
			$isrid = AT(RES.ISR_LIST[resid],0)$
			$RES.CORE[resid] = OSAP.CORE[ISR.OSAPID[isrid]]$
		$ELIF LENGTH(RES.ICI_LIST[resid])$
			$iciid = AT(RES.ICI_LIST[resid],0)$
			$RES.CORE[resid] = OSAP.CORE[ICI.OSAPID[iciid]]$
		$END$

$		//同一コアで関連付けられているか確認
		$IF LENGTH(RES.TSK_LIST[resid])$
			$FOREACH tskid RES.TSK_LIST[resid]$
				$IF (RES.CORE[resid] != OSAP.CORE[TSK.OSAPID[tskid]])$
					$RES.CORE[resid] = 0xff$
					$ERROR RES.TEXT_LINE[resid]$$FORMAT(_("OsResource(%1%) referred to by any OsTask(%2%) assigned to different cores"), resid, tskid)$$END$
				$END$
			$END$
		$END$

		$IF LENGTH(RES.ISR_LIST[resid])$
			$FOREACH isrid RES.ISR_LIST[resid]$
				$IF (RES.CORE[resid] != OSAP.CORE[ISR.OSAPID[isrid]])$
					$RES.CORE[resid] = 0xff$
					$ERROR RES.TEXT_LINE[resid]$$FORMAT(_("OsResource(%1%) referred to by any OsIsr(%2%) assigned to different cores"), resid, isrid)$$END$
				$END$
			$END$
		$END$

		$IF LENGTH(RES.ICI_LIST[resid])$
			$FOREACH iciid RES.ICI_LIST[resid]$
				$IF (RES.CORE[resid] != OSAP.CORE[ICI.OSAPID[iciid]])$
					$RES.CORE[resid] = 0xff$
					$ERROR RES.TEXT_LINE[resid]$$FORMAT(_("OsResource(%1%) referred to by any OsInterCoreInterrupt(%2%) assigned to different cores"), resid, iciid)$$END$
				$END$
			$END$
		$END$
	$END$
$END$


$FOREACH resid RES.ID_LIST$
$	// リンクリソースの上限優先度と割り付けコアの設定
	$IF EQ(RES.RESATR[resid], "LINKED")$
		$RES.CEILPRI[resid] = RES.CEILPRI[RES.LINKEDRESID[resid]]$
		$RES.CORE[resid] = RES.CORE[RES.LINKEDRESID[resid]]$
	$END$
$END$


$ =====================================================================
$ アプリケーションモードのエラーチェック
$ =====================================================================

$ APP
$ アプリケーションモードは32個以下か．
$IF LENGTH(APP.ID_LIST) > 32$
	$ERROR$$FORMAT(_("too many OsAppMode %1% > 32"), LENGTH(APP.ID_LIST))$$END$
$END$

$i = 0$
$FOREACH appid APP.ID_LIST$
	$APP.ID[appid] = i$
	$i = i + 1$
$END$


$ =====================================================================
$ タスク関連のエラーチェックと内部データの生成
$ =====================================================================

$TRACE("TASK CHECK")$

$FOREACH tskid TSK.ID_LIST$
	$TSK.EXTENDED[tskid] = LENGTH(TSK.EVT_LIST[tskid]) > 0$

$	// 内部リソースを取得する場合，取得時の優先度を設定
	$FOREACH resid TSK.RES_LIST[tskid]$
		$IF EQ(RES.RESATR[resid], "INTERNAL")$
			$TSK.INRESPRI[tskid] = RES.CEILPRI[resid]$
		$END$
	$END$

$	// タスクのAUTOSTARTの起動パターン
	$astptn = 0$
	$FOREACH appid TSK.APP_LIST[tskid]$
		$astptn = astptn | (1 << APP.ID[appid])$
	$END$
	$TSK.ASTPTN[tskid] = astptn$

$	// タスクに関連付けされたイベントマスクのORをとる(AUTOは0で計算)
	$TSK.EVTMSK[tskid] = 0$

	$FOREACH evtid TSK.EVT_LIST[tskid]$
		$TSK.EVTMSK[tskid] = TSK.EVTMSK[tskid] | EVT.MASK[evtid]$
	$END$


	$tskpri = TSK.PRIORITY[tskid]$
	$tskext = TSK.EXTENDED[tskid]$
	$tskact = TSK.ACTIVATION[tskid]$
	$tsksch = TSK.SCHEDULE[tskid]$

$	// priorityの範囲チェック
	$IF (tskpri < TMIN_TPRI) || (TMAX_TPRI < tskpri)$
		$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskPriority(%1%) of OsTask(%2%) is a range error"), tskpri, tskid)$$END$
	$END$

$	// activationの範囲チェック
	$IF !tskext$
		$IF (tskact == 0) || (256 < tskact)$
			$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskActivation(%1%) of OsTask(%2%) is a range error"), tskact, tskid)$$END$
		$END$
	$ELSE$
		$IF (tskact != 1)$
			$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskActivation(%1%) of OsTask(%2%) must be 1"), tskact, tskid)$$END$
		$END$
	$END$

$	// scheduleの内容チェック（"NON","FULL"以外ならエラー）
	$IF !(EQ(tsksch, "NON") || EQ(tsksch, "FULL"))$
		$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskSchedule of OsTask(%1%) must be NON or FULL"), tskid)$$END$
	$END$

	$intres_flag = 0$

	$FOREACH resid TSK.RES_LIST[tskid]$
		$IF EQ(RES.RESATR[resid], "INTERNAL")$

$			// タスクには最大1つの内部リソースしか割り当てられないため，
$			// 内部リソースを2つ以上割り当てた場合はエラーとする．

$			// 割り当てたリソースが内部リソースなら確認用フラグをセットする．
$			// 内部リソース確認用フラグがすでにセットされていたらエラーとする．
			$IF intres_flag == 0$
				$intres_flag = 1$
			$ELSE$
				$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("too many assigned internal resource OsTaskResourceRef(%1%) of OsTask(%2%)"), resid, tskid)$$END$
			$END$

$			// 内部リソースがノンプリエンプティブのタスクと関連付けられていたらエラーとする．
			$IF EQ(TSK.SCHEDULE[tskid], "NON")$
				$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("internal resource OsTaskResourceRef(%1%) of OsTask(%2%) can't be assign to a nonpreemptable task"), resid, tskid)$$END$
			$END$
		$END$

$		// リンクリソースは標準リソースにリンクするものであるため
$		// リンクリソースであればエラーとする．
		$IF EQ(RES.RESATR[resid], "LINKED")$
			$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskResourceRef(%1%) of OsTask(%2%) is linked resource"), resid, tskid)$$END$
		$END$
	$END$
$END$


$
$ タスクのOS内部で用いるIDを割り当てる
$ 拡張タスクからIDを割り当てる
$
$i = 0$
$tnum_exttask = 0$
$new_id_list = {}$
$FOREACH tskid TSK.ID_LIST$
	$IF TSK.EXTENDED[tskid]$
		$TSK.ID[tskid] = i$
		$i = i + 1$
		$tnum_exttask = tnum_exttask + 1$
		$new_id_list = APPEND(new_id_list, tskid)$
	$END$
$END$
$FOREACH tskid TSK.ID_LIST$
	$IF !TSK.EXTENDED[tskid]$
		$TSK.ID[tskid] = i$
		$i = i + 1$
		$new_id_list = APPEND(new_id_list, tskid)$
	$END$
$END$
$TSK.ID_LIST = new_id_list$



$ =====================================================================
$ イベントマスク値決定
$ =====================================================================

$TRACE("EVENT CHECK")$

$auto_evt_msk = 1$
$auto_evt_num = 0$
$FOREACH evtid EVT.ID_LIST$
$	//マスク値指定有
	$IF !EQ(EVT.MASK[evtid], AUTO)$
		$IF (EVT.MASK[evtid] == 0)$
			$ERROR EVT.TEXT_LINE[evtid]$$FORMAT(_("OsEventMask of OsEvent(%1%) must not be zero"), evtid)$$END$
			$DIE()$
		$END$
		$EVT.CALC_MASK[evtid] = EVT.MASK[evtid]$
$	//マスク値指定無
	$ELSE$
		$EVT.CALC_MASK[evtid] = 0$
$		//タスクに関連付け有
		$IF LENGTH(EVT.TSK_LIST[evtid])$
			$hit_flg = 0$
			$auto_tskevt_msk = 1$
			$FOREACH i RANGE(1, 32)$
				$IF !hit_flg$
					$ok_flg = 1$
					$FOREACH tskid EVT.TSK_LIST[evtid]$
						$IF (TSK.EVTMSK[tskid] & auto_tskevt_msk) != 0$
							$ok_flg = 0$
						$END$
					$END$

$					//全タスクで空きビット発見
					$IF ok_flg$
						$hit_flg = 1$
						$FOREACH tskid EVT.TSK_LIST[evtid]$
							$TSK.EVTMSK[tskid] = TSK.EVTMSK[tskid] | auto_tskevt_msk$
						$END$
						$EVT.CALC_MASK[evtid] = auto_tskevt_msk$
					$ELSE$
						$auto_tskevt_msk = auto_tskevt_msk << 1$
					$END$
				$END$
			$END$
$			//空きビットなし
			$IF !hit_flg$
				$ERROR EVT.TEXT_LINE[evtid]$$FORMAT(_("Auto OsEventMask of OsEvent(%1%) is no free"), evtid)$$END$
				$DIE()$
			$END$
$		//タスクに関連付け無
		$ELSE$
			$IF auto_evt_num > 31$
				$ERROR EVT.TEXT_LINE[evtid]$$FORMAT(_("Auto OsEventMask of OsEvent(%1%) is over(32)"))$$END$
				$DIE()$
			$END$
			$EVT.CALC_MASK[evtid] = auto_evt_msk$
			$auto_evt_msk = auto_evt_msk << 1$
			$auto_evt_num = auto_evt_num + 1$
		$END$
	$END$
$END$
$NL$



$ =====================================================================
$ カウンタ関連のエラーチェックと内部データの生成
$ =====================================================================

$TRACE("COUNTER CHECK")$


$FOREACH cntid CNT.ID_LIST$
$	// カウンタの属性はSOFTWARE / HARDWAREかチェック
	$IF !(EQ(CNT.CNTATR[cntid], "SOFTWARE") || EQ(CNT.CNTATR[cntid], "HARDWARE"))$
		$ERROR CNT.TEXT_LINE[cntid]$$FORMAT(_("OsCounterType of OsCounter(%1%) must be SOFTWARE or HARDWARE"), cntid)$$END$
	$END$
$END$


$
$ カウンタのOS内部で用いるIDを割り当てる
$ ハードウェアカウンタからIDを割り当てる
$
$i = 0$
$new_id_list = {}$
$FOREACH cntid CNT.ID_LIST$
	$IF EQ(CNT.CNTATR[cntid], "HARDWARE")$
		$CNT.ID[cntid] = i$
		$i = i + 1$
		$new_id_list = APPEND(new_id_list, cntid)$
	$END$
$END$
$FOREACH cntid CNT.ID_LIST$
	$IF EQ(CNT.CNTATR[cntid], "SOFTWARE")$
		$CNT.ID[cntid] = i$
		$i = i + 1$
		$new_id_list = APPEND(new_id_list, cntid)$
	$END$
$END$
$CNT.ID_LIST = new_id_list$


$HWCNT.ID_LIST = {}$
$FOREACH cntid CNT.ID_LIST$
	$cnttpb = CNT.TICKSPERBASE[cntid]$
	$cntmin = CNT.MINCYCLE[cntid]$
	$cntmax = CNT.MAXALLOWED[cntid]$

$	// ticksperbaseの範囲確認
	$IF (cnttpb < 1) || (cnttpb > 0x7fffffff)$
		$ERROR CNT.TEXT_LINE[cntid]$$FORMAT(_("OsCounterTicksPerBase(%1%) of OsCounter(%2%) is a range error"), cnttpb, cntid)$$END$
	$END$

$	// mincycleの範囲確認
	$IF (cntmin < 1) || (cntmin > 0x7fffffff)$
		$ERROR CNT.TEXT_LINE[cntid]$$FORMAT(_("OsCounterMinCycle(%1%) of OsCounter(%2%) is a range error"), cntmin, cntid)$$END$
	$END$

$	// maxallowedvalueの範囲確認
	$IF (cntmax < 1) || (cntmax > 0x7fffffff)$
		$ERROR CNT.TEXT_LINE[cntid]$$FORMAT(_("OsCounterMaxAllowedValue(%1%) of OsCounter(%2%) is a range error"), cntmax, cntid)$$END$
	$END$

$	// mincycleがmaxallowedvalueより大きいか．
	$IF cntmax < cntmin$
		$ERROR CNT.TEXT_LINE[cntid]$$FORMAT(_("OsCounterMinCycle(%1%) of OsCounter(%2%) is more than OsCounterMaxAllowedValue(%3%)"), cntmin, cntid, cntmax)$$END$
	$END$

$	// カウンタがOSAPに所属するかチェックする．
	$IF !LENGTH(CNT.OSAPID[cntid])$
		$ERROR CNT.TEXT_LINE[tskid]$$FORMAT(_("OsCounter(%1%) does not belong OsApplication)"), cntid)$$END$
		$DIE()$
	$END$

$	// ハードウェアカウンタのISRがCATEGORY_2かチェック
	$IF EQ(CNT.CNTATR[cntid], "HARDWARE")$
		$IF EQ(ISR.CATEGORY[CNT.ISRID[cntid]], "CATEGORY_2")$
			$HWCNT.ID_LIST = APPEND(HWCNT.ID_LIST, cntid)$
		$ELSE$
			$ERROR CNT.TEXT_LINE[cntid]$$FORMAT(_("OsCounterIsrRef(%1%) of OsCounter(%2%) is C1ISR"), CNT.ISRID[cntid], cntid)$$END$
		$END$

		$IF ISR.OSAPID[CNT.ISRID[cntid]] != CNT.OSAPID[cntid]$
			$ERROR CNT.TEXT_LINE[cntid]$$FORMAT(_("HARDWARE OsCounter(%1%) and OsCounterIsrRef(%2%) must belong to the same OsApplication"), cntid, isrid)$$END$
		$END$
	$END$
$END$



$ =====================================================================
$ アラーム関連のエラーチェックと内部データの生成
$ =====================================================================

$TRACE("ALARM CHECK")$

$i = 0$
$FOREACH almid ALM.ID_LIST$

$	// アラームのAUTOSTARTの起動パターン
	$astptn = 0$
	$FOREACH appid ALM.APP_LIST[almid]$
		$astptn = astptn | (1 << APP.ID[appid])$
	$END$
	$ALM.ASTPTN[almid] = astptn$

$	// IDの設定
	$ALM.ID[almid] = i$
	$i = i + 1$

$	// アラムがOSAPに所属するかチェックする．
	$IF !LENGTH(ALM.OSAPID[almid])$
		$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarm(%1%) does not belong OsApplication)"), almid)$$END$
		$DIE()$
	$END$


$	// 割付けコアの確認
	$IF OSAP.CORE[ALM.OSAPID[almid]] != OSAP.CORE[CNT.OSAPID[ALM.CNTID[almid]]]$
		$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarm(%1%) is assigned to a OsCounter(%2%) on a different core"), almid, ALM.CNTID[almid])$$END$
	$END$

$	// アプリケーションモード設定した場合のみ，アラーム初回起動ティック値と周期の適切性をチェック
	$IF ALM.ASTPTN[almid]$
$		// アラーム初回起動ティック値は，カウンタ最大値を超えるか或いは0より小さいか
		$IF CNT.MAXALLOWED[ALM.CNTID[almid]] < ALM.ALARMTIME[almid] || ALM.ALARMTIME[almid] < 0$
			$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmAlarmTime(%1%) of OsAlarm(%2%) must be in the range 0 .. OsCounterMaxAllowedValue(%3%)"), 
													ALM.ALARMTIME[almid], almid, CNT.MAXALLOWED[ALM.CNTID[almid]])$$END$
		$END$

$		// アラーム周期は，カウンタ最大値を超えるか，0より小さいか，カウンタの最小周期より小さい(0を除く)か
		$IF CNT.MAXALLOWED[ALM.CNTID[almid]] < ALM.CYCLETIME[almid] || ALM.CYCLETIME[almid] < 0 ||
			(ALM.CYCLETIME[almid] < CNT.MINCYCLE[ALM.CNTID[almid]] && ALM.CYCLETIME[almid] != 0)$
			$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmCycleTime(%1%) of OsAlarm(%2%) must be zero OR in the range OsCounterMinCycle(%3%) .. OsCounterMaxAllowedValue(%4%)"), 
											ALM.CYCLETIME[almid], almid, CNT.MINCYCLE[ALM.CNTID[almid]], CNT.MAXALLOWED[ALM.CNTID[almid]])$$END$
		$END$

$		// autostarttypeにABSOLUTE/RELATIVE以外のものが登録されていないか
		$IF !(EQ(ALM.AUTOSTARTTYPE[almid], "ABSOLUTE") || EQ(ALM.AUTOSTARTTYPE[almid], "RELATIVE"))$
			$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmAutostartType of OsAlarm(%1%) must be ABSOLUTE or RELATIVE"), almid)$$END$
		$END$

$		// autostarttypeがRELATIVEの場合はSTARTTICK==0は禁止
		$IF EQ(ALM.AUTOSTARTTYPE[almid], "RELATIVE") && ALM.ALARMTIME[almid] == 0$
			$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmAlarmTime of RELATIVE OsAlarm(%1%) must not be zero"), almid)$$END$
		$END$
	$END$

$	// taskが拡張タスクか(SETEVENTの場合)
	$IF EQ(ALM.ALMATR[almid], "SETEVENT")$
		$IF !TSK.EXTENDED[ALM.TSKID[almid]]$
			$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmSetEventTaskRef(%1%) of OsAlarm(%2%) is not extended task"), ALM.TSKID[almid], almid)$$END$
		$END$
	$END$

	$IF EQ(ALM.ALMATR[almid], "INCREMENTCOUNTER")$
$		// アラームからハードウェアカウンタをインクリメントしているかチェック
		$IF EQ(CNT.CNTATR[ALM.INCID[almid]], "HARDWARE")$
			$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmIncrementCounterRef(%1%) of OsAlarm(%2%) is hardware counter"), ALM.INCID[almid], almid)$$END$
		$END$

$		// アラームから別コアのカウンタをインクリメントしているかチェック
		$IF OSAP.CORE[ALM.OSAPID[almid]] != OSAP.CORE[CNT.OSAPID[ALM.INCID[almid]]]$
			$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmIncrementCounterRef(%1%) of OsAlarm(%2%) is on a different core"), ALM.INCID[almid], almid)$$END$
		$END$
	$END$
$END$


$
$ IncrementCounterの循環チェックのための関数
$

$ 行列の掛け算
$FUNCTION MATRIX_MUL$
	$FOREACH i RANGE(0, matrix_size - 1)$
		$FOREACH j RANGE(0, matrix_size - 1)$
			$result = {}$
			$FOREACH k RANGE(0, matrix_size - 1)$
				$IF LENGTH(result) == 0$
					$IF LENGTH(result_matrix[i * matrix_size + k]) && LENGTH(base_matrix[k * matrix_size + j])$
						$result = APPEND(result_matrix[i * matrix_size + k], base_matrix[k * matrix_size + j])$
					$END$
				$END$
			$END$
$			// 掛け算結果を一時行列に保持
			$tmp_matrix[i * matrix_size + j] = result$
		$END$
	$END$
$	// 掛け算結果を結果行列にセット
	$FOREACH i RANGE(0, matrix_size - 1)$
		$FOREACH j RANGE(0, matrix_size - 1)$
			$result_matrix[i * matrix_size + j] = tmp_matrix[i * matrix_size + j]$
		$END$
	$END$
$END$


$ 行列の0初期化
$ base_matrix   初期行列
$ result_matrix 掛け算の結果の行列
$FUNCTION INIT_MATRIX$
	$FOREACH almid ALM.ID_LIST$
		$IF EQ(ALM.ALMATR[almid], "INCREMENTCOUNTER")$
			$i = ALM.CNTID[almid] - 1$
			$j = ALM.INCID[almid] - 1$
			$IF !LENGTH(base_matrix[i * matrix_size + j])$
				$base_matrix[i * matrix_size + j] = {almid}$
				$result_matrix[i * matrix_size + j] = {almid}$
			$END$
		$END$
	$END$
$END$


$ アラームの循環リストを最小IDのアラームが先頭になるようにローテートさせる
$ また，循環リストに重複したアラームが含まれていたら重複したアラーム以降を
$ リストから取り除く
$FUNCTION ROTATE_LOOPED_ALARM$
$	// 関数内でiを使うため退避
	$tmp_i = i$
	$alm_list = ARGV[1]$
	$size = LENGTH(alm_list)$
$	// リスト内でのアラームIDの最小値
	$min_alm_id = AT(alm_list, 0)$
	$min_alm_id_index = 0$
	$FOREACH i RANGE(1, size - 1)$
		$IF min_alm_id > AT(alm_list, i)$
			$min_alm_id = AT(alm_list, i)$
			$min_alm_id_index = i$
		$END$
	$END$
	$RESULT = {}$
$	// 重複したアラームIDが存在するか
	$duplicate = 0$
	$FOREACH i RANGE(0, size - 1)$
		$append_alm = AT(alm_list, (min_alm_id_index + i) % size)$
		$IF LENGTH(FIND(RESULT, append_alm)) > 0$
$			// 重複したアラームがあればそれ以降は追加しない
			$duplicate = 1$
		$END$
		$IF duplicate == 0$
			$RESULT = APPEND(RESULT, append_alm)$
		$END$
	$END$
	$i = tmp_i$
$END$


$FUNCTION CHECK_COUNTER_LOOP$
	$matrix_size = LENGTH(CNT.ID_LIST)$

	$INIT_MATRIX()$

$	// 循環アラームリストのリスト
	$alm_loop_list_size = 0$

$	// ループの検出とエラー出力
	$FOREACH n RANGE(0,  matrix_size - 1)$
		$FOREACH i RANGE(0,  matrix_size - 1)$
			$IF LENGTH(result_matrix[i * matrix_size + i]) > 0$
				$result_matrix[i * matrix_size + i] = ROTATE_LOOPED_ALARM(result_matrix[i * matrix_size + i])$
				$find = 0$
				$FOREACH ii RANGE(0, alm_loop_list_size - 1)$
$	 				// 過去に検出したリストに含まれるか調べる
					$IF EQ(alm_loop_list[ii], result_matrix[i * matrix_size + i])$
						$find = 1$
					$END$
				$END$
				$IF find == 0$
					$alm_loop_list[alm_loop_list_size] = result_matrix[i * matrix_size + i]$
					$alm_loop_list_size = alm_loop_list_size + 1$
					$ERROR ALM.TEXT_LINE[AT(result_matrix[i * matrix_size + i], 0)]$
								$FORMAT(_("OsAlarmIncrementCounterRef of OsAlarm(%1%) is a cyclic chain of alarm action"), result_matrix[i * matrix_size + i])$$END$
				$END$
			$END$
		$END$
		$MATRIX_MUL()$
	$END$
$END$


$ 直接/間接的にカウンタをインクリメントするか(INCREMENTCOUNTERの場合)
$ ユーザヘッダにOMIT_CHECK_CYCLIC_CHAINの定義があれば，アラーム循環チェックを行わない
$IF !OMIT_CHECK_CYCLIC_CHAIN$
	$CHECK_COUNTER_LOOP()$
$END$



$ =====================================================================
$ スケジュールテーブル関連のエラーチェックと内部データの生成
$ =====================================================================

$TRACE("SCHEDULETABLE CHECK")$

$FOREACH schtblid SCHTBL.ID_LIST$
$	// 同期方法設定値チェック
$	// syncstrategyにNONE/IMPLICITいずれか一つが指定されているか
	$IF !(EQ(SCHTBL.SYNCSTRATEGY[schtblid], "NONE") || EQ(SCHTBL.SYNCSTRATEGY[schtblid], "IMPLICIT"))$
		$ERROR SCHTBL.TEXT_LINE[schtblid]$
				$FORMAT(_("OsScheduleTblSyncStrategy of OsScheduleTable(%1%) must be NONE or IMPLICIT"), schtblid)$$END$
	$END$

$	// スケジュールテーブルがOSAPに所属するかチェックする．
	$IF !LENGTH(SCHTBL.OSAPID[schtblid])$
		$ERROR SCHTBL.TEXT_LINE[schtblid]$$FORMAT(_("OsScheduleTable(%1%) does not belong OsApplication)"), schtblid)$$END$
		$DIE()$
	$END$

$	// 割付けコアの確認
	$IF OSAP.CORE[SCHTBL.OSAPID[schtblid]] != OSAP.CORE[CNT.OSAPID[SCHTBL.CNTID[schtblid]]]$
		$ERROR SCHTBL.TEXT_LINE[schtblid]$$FORMAT(_("OsScheduleTable(%1%) is assigned to a OsCounter(%2%) on a different core"), schtblid, SCHTBL.CNTID[schtblid])$$END$
	$END$
$END$


$ 各スケジュールテーブルIDの定義(暗黙同期定義は先，非同期定義は後)
$i = 0$
$ スケジュール初期化ブロック生成時に使用する順序付きリスト変数
$new_id_list = {}$
$FOREACH schtblid SCHTBL.ID_LIST$
	$IF EQ(SCHTBL.SYNCSTRATEGY[schtblid], "IMPLICIT")$
		$SCHTBL.ID[schtblid] = i$
		$i = i + 1$
		$new_id_list = APPEND(new_id_list, schtblid)$
	$END$
$END$
$FOREACH schtblid SCHTBL.ID_LIST$
	$IF EQ(SCHTBL.SYNCSTRATEGY[schtblid], "NONE")$
		$SCHTBL.ID[schtblid] = i$
		$i = i + 1$
		$new_id_list = APPEND(new_id_list, schtblid)$
	$END$
$END$
$NL$
$SCHTBL.ID_LIST = new_id_list$


$exppt_index = 0$
$EXPPT.ID_LIST = {}$
$FOREACH expptactid EXPPTACT.ID_LIST$
$	// 満了アクションの属性と満了アクション処理内容設定の整合性チェック
$	// taskが拡張タスクか(SETEVENTの場合)
	$IF EQ(EXPPTACT.EXPIREATR[expptactid], "SETEVENT")$
		$IF !TSK.EXTENDED[EXPPTACT.TSKID[expptactid]]$
			$ERROR EXPPTACT.TEXT_LINE[expptactid]$
						$FORMAT(_("OsScheduleTableSetEventTaskRef(%1%) of OsScheduleTableExpiryPoint(%2%) is not extended task"), EXPPTACT.TSKID[expptactid], expptactid)$$END$
		$END$
	$END$

	$schtblid = EXPPTACT.SCHTBLID[expptactid]$
	$FOREACH expptid EXPPT.ID_LIST$
		$IF EXPPT.OFFSET[expptid] == EXPPTACT.OFFSET[expptactid] &&
			EXPPTACT.SCHTBLID[expptactid] == EXPPT.SCHTBLID[expptid]$
			$EXPPTACT.EXPPTID[expptactid] = expptid$
		$END$
	$END$

$	// 満了点情報の生成とスケジュールテーブルとの関連付け
	$IF !LENGTH(EXPPTACT.EXPPTID[expptactid])$
		$EXPPT.ID_LIST = APPEND(EXPPT.ID_LIST, exppt_index)$
		$EXPPT.SCHTBLID[exppt_index] = schtblid$
		$EXPPT.OFFSET[exppt_index] = EXPPTACT.OFFSET[expptactid]$
		$SCHTBL.EXPPTINDEX_LIST[schtblid] = APPEND(SCHTBL.EXPPTINDEX_LIST[schtblid], exppt_index)$
		$EXPPTACT.EXPPTID[expptactid] = exppt_index$

$		満了点のオフセットがスケジュールテーブルの周期を超えたかチェック
		$IF EXPPT.OFFSET[exppt_index] > SCHTBL.DURATIONTICK[schtblid]$
			$ERROR EXPPTACT.TEXT_LINE[expptactid]$
						$FORMAT(_("OsScheduleTblExpPointOffset(%1%) of OsScheduleTableExpiryPoint(%2%) of OsScheduleTable(%3%) more than OsScheduleTableDuration(%4%)"), 
							EXPPTACT.OFFSET[expptactid], expptactid, schtblid, SCHTBL.DURATIONTICK[schtblid])$$END$
		$END$
		$exppt_index = exppt_index + 1$
	$END$

$	// 同一スケジュールテーブルの他の満了点で設定したオフセット値が重複していないかチェック
	$overlap_flg = 0$
	$FOREACH expptid EXPPTACT.ID_LIST$
		$IF EXPPTACT.SCHTBLID[expptactid] == EXPPTACT.SCHTBLID[expptid] &&
			EXPPTACT.EXPPT[expptactid] != EXPPTACT.EXPPT[expptid] &&
			EXPPTACT.OFFSET[expptactid] == EXPPTACT.OFFSET[expptid] &&
			overlap_flg == 0$
			$ERROR EXPPTACT.TEXT_LINE[expptactid]$
				$FORMAT(_("OsScheduleTblExpPointOffset(%1%) of %2% of %3% of %4% and OsScheduleTblExpPointOffset(%5%) of %6% of %7% of %8% are duplicated"), 
					EXPPTACT.OFFSET[expptactid], expptactid, EXPPTACT.EXPPT[expptactid], schtblid, EXPPTACT.OFFSET[expptid], expptid, EXPPTACT.EXPPT[expptid], schtblid)$$END$
			$overlap_flg = 1$
		$END$
	$END$
$END$


$ 暗黙スケジュールカウント用変数
$tnum_implscheduletable = 0$

$FOREACH schtblid SCHTBL.ID_LIST$
$	// EXPPTINDEX_LISTをOFFSETの順にソートする
	$SCHTBL.EXPPTINDEX_LIST[schtblid] = SORT(SCHTBL.EXPPTINDEX_LIST[schtblid], "EXPPT.OFFSET")$

	$astptn = 0$
	$FOREACH appid SCHTBL.APP_LIST[schtblid]$
		$astptn = astptn | (1 << APP.ID[appid])$
	$END$
	$SCHTBL.ASTPTN[schtblid] = astptn$

$	// アプリケーションモード設定した場合のみ，初回起動ティック値をチェック
	$IF SCHTBL.ASTPTN[schtblid]$

		$IF !(EQ(SCHTBL.AUTOSTARTTYPE[schtblid], "ABSOLUTE") || EQ(SCHTBL.AUTOSTARTTYPE[schtblid], "RELATIVE"))$
$			// autostarttypeにABSOLUTE/RELATIVE以外のものが登録されていないか
			$ERROR SCHTBL.TEXT_LINE[schtblid]$$FORMAT(_("OsScheduleTableAutostartType of OsScheduleTable(%1%) must be ABSOLUTE or RELATIVE"), schtblid)$$END$
		$ELIF EQ(SCHTBL.STARTTICK[schtblid], "NULL")$
$			// 初回起動ティック値が省略されたたか
			$ERROR SCHTBL.TEXT_LINE[schtblid]$$FORMAT(_("OsScheduleTableStartValue(%1%) of %2% AutoStart OsScheduleTable(%3%) must be designated"),
													SCHTBL.STARTTICK[schtblid], SCHTBL.AUTOSTARTTYPE[schtblid], schtblid)$$END$
			$DIE()$
		$END$

$		// 初回起動ティック値は，カウンタ最大値を超えるか或いは0より小さいか
		$IF CNT.MAXALLOWED[SCHTBL.CNTID[schtblid]] < SCHTBL.STARTTICK[schtblid] || SCHTBL.STARTTICK[schtblid] < 0$
			$ERROR SCHTBL.TEXT_LINE[schtblid]$$FORMAT(_("OsScheduleTableStartValue(%1%) of OsScheduleTable(%2%) must be in the range 0 .. OsCounterMaxAllowedValue(%3%)"), 
													SCHTBL.STARTTICK[schtblid], schtblid, CNT.MAXALLOWED[SCHTBL.CNTID[schtblid]])$$END$
		$END$

		$IF EQ(SCHTBL.AUTOSTARTTYPE[schtblid], "RELATIVE")$
$			// autostarttypeがRELATIVEの場合はSTARTTICK==0は禁止
			$IF SCHTBL.STARTTICK[schtblid] == 0$
				$ERROR SCHTBL.TEXT_LINE[schtblid]$$FORMAT(_("OsScheduleTableStartValue of RELATIVE OsScheduleTable(%1%) must not be zero"), schtblid)$$END$
			$END$

$			//暗黙スケジュールテーブルのautostarttypeにRELATIVEが指定されているか
			$IF EQ(SCHTBL.SYNCSTRATEGY[schtblid], "IMPLICIT")$
				$ERROR SCHTBL.TEXT_LINE[schtblid]$$FORMAT(_("OsScheduleTableAutostartType of IMPLICIT OsScheduleTable(%1%) must not be RELATIVE"), schtblid)$$END$
			$END$
		$END$
	$END$

$	//満了点数254を超えるかチェック
	$IF LENGTH(SCHTBL.EXPPTINDEX_LIST[schtblid]) > 254$
		$ERROR SCHTBL.TEXT_LINE[schtblid]$
					$FORMAT(_("too many OsScheduleTableExpiryPoint of OsScheduleTable(%1%) %2% > 254"), schtblid, LENGTH(SCHTBL.EXPPTINDEX_LIST[schtblid]))$$END$
	$ELSE$
		$previous_exp_expptid = AT(SCHTBL.EXPPTINDEX_LIST[schtblid], 0)$

$		// 初期オフセットは，0か接続カウンタの最小周期〜最大値かになる必要がある
		$IF EXPPT.OFFSET[previous_exp_expptid] != 0 && (EXPPT.OFFSET[previous_exp_expptid] < CNT.MINCYCLE[SCHTBL.CNTID[schtblid]] ||
												 EXPPT.OFFSET[previous_exp_expptid] > CNT.MAXALLOWED[SCHTBL.CNTID[schtblid]])$
			$ERROR EXPPT.TEXT_LINE[previous_exp_expptid]$
							$FORMAT(_("The Initial Offset(%1%) of OsScheduleTable(%2%) must be zero OR in the range OsCounterMinCycle(%3%) .. OsCounterMaxAllowedValue(%4%)"), 
								EXPPT.OFFSET[previous_exp_expptid], schtblid, CNT.MINCYCLE[SCHTBL.CNTID[schtblid]], CNT.MAXALLOWED[SCHTBL.CNTID[schtblid]])$$END$
		$END$

$		// 各満了点間の遅延値は，接続カウンタの最小周期〜最大値かになる必要がある
		$FOREACH exp_expptid SCHTBL.EXPPTINDEX_LIST[schtblid]$
			$IF previous_exp_expptid != exp_expptid$
				$IF (EXPPT.OFFSET[exp_expptid] - EXPPT.OFFSET[previous_exp_expptid] < CNT.MINCYCLE[SCHTBL.CNTID[schtblid]]) ||
					(EXPPT.OFFSET[exp_expptid] - EXPPT.OFFSET[previous_exp_expptid] > CNT.MAXALLOWED[SCHTBL.CNTID[schtblid]])$
					$ERROR EXPPT.TEXT_LINE[exp_expptid]$
						$FORMAT(_("The delay of OsScheduleTable(%1%) between OsScheduleTblExpPointOffset(%2% and %3%) must be in the range OsCounterMinCycle(%4%) .. OsCounterMaxAllowedValue(%5%)"),
							schtblid, EXPPT.OFFSET[previous_exp_expptid], EXPPT.OFFSET[exp_expptid], CNT.MINCYCLE[SCHTBL.CNTID[schtblid]], CNT.MAXALLOWED[SCHTBL.CNTID[schtblid]])$$END$
				$END$
				$previous_exp_expptid = exp_expptid$
			$END$
		$END$

$		// 最終遅延値は，接続カウンタの最小周期〜最大値かになる必要がある（単発の場合0〜最大値が許される）
		$IF (SCHTBL.REPEAT[schtblid] != 1) &&
			((SCHTBL.DURATIONTICK[schtblid] - EXPPT.OFFSET[previous_exp_expptid] < 0) ||
			 (SCHTBL.DURATIONTICK[schtblid] - EXPPT.OFFSET[previous_exp_expptid] > CNT.MAXALLOWED[SCHTBL.CNTID[schtblid]]))$
$			// 単発駆動の場合，0未満またはカウンタの最大値を超える値を設定するとエラー
			$ERROR EXPPT.TEXT_LINE[previous_exp_expptid]$
						$FORMAT(_("Final Delay(%1%) of OsScheduleTable(%2%) must be in the range OsCounterMinCycle(%3%) .. OsCounterMaxAllowedValue(%4%), and allow zero when single-shot"), 
							(SCHTBL.DURATIONTICK[schtblid] - EXPPT.OFFSET[previous_exp_expptid]), schtblid, CNT.MINCYCLE[SCHTBL.CNTID[schtblid]], CNT.MAXALLOWED[SCHTBL.CNTID[schtblid]])$$END$
		$ELIF (SCHTBL.REPEAT[schtblid] == 1) &&
			((SCHTBL.DURATIONTICK[schtblid] - EXPPT.OFFSET[previous_exp_expptid] < CNT.MINCYCLE[SCHTBL.CNTID[schtblid]]) ||
			 (SCHTBL.DURATIONTICK[schtblid] - EXPPT.OFFSET[previous_exp_expptid] > CNT.MAXALLOWED[SCHTBL.CNTID[schtblid]]))$
$			// 周期駆動の場合，カウンタの最小周期未満またはカウンタの最大値を超える値を設定するとエラー
			$ERROR EXPPT.TEXT_LINE[previous_exp_expptid]$
						$FORMAT(_("Final Delay(%1%) of OsScheduleTable(%2%) must be in the range OsCounterMinCycle(%3%) .. OsCounterMaxAllowedValue(%4%), and allow zero when single-shot"), 
							(SCHTBL.DURATIONTICK[schtblid] - EXPPT.OFFSET[previous_exp_expptid]), schtblid, CNT.MINCYCLE[SCHTBL.CNTID[schtblid]], CNT.MAXALLOWED[SCHTBL.CNTID[schtblid]])$$END$
		$END$
	$END$

$	// 暗黙スケジュールテーブルのカウント
	$IF EQ(SCHTBL.SYNCSTRATEGY[schtblid], "IMPLICIT")$
		$tnum_implscheduletable = tnum_implscheduletable + 1$

$		暗黙スケジュールテーブルの周期は，駆動カウント最大値+1かチェック
		$IF SCHTBL.DURATIONTICK[schtblid] != CNT.MAXALLOWED[SCHTBL.CNTID[schtblid]] + 1$
			$ERROR SCHTBL.TEXT_LINE[schtblid]$$FORMAT(_("OsScheduleTableDuration(%1%) of IMPLICIT OsScheduleTable(%2%) must be OsCounterMaxAllowedValue(%3%) + 1"), 
													SCHTBL.DURATIONTICK[schtblid], schtblid, CNT.MAXALLOWED[SCHTBL.CNTID[schtblid]])$$END$
		$END$
	$END$

$END$



$ =====================================================================
$ スピンロック関連のエラーチェックと内部データの生成
$ =====================================================================

$TRACE("SPINLOCK CHECK")$

$invalid_spinlock = 0$
$i = 1$
$FOREACH spnid SPN.ID_LIST$
	$SPN.ID[spnid] = i$
	$i = i + 1$
$END$

$ ネスト順序リストの生成
$FOREACH spnid SPN.ID_LIST$
	$nest_order = 0$

	$IF (LENGTH(SPN.SUCCESSOR[spnid]) && !EQ(SPN.SUCCESSOR[spnid],"NULL"))$
		$next_spn_id = SPN.SUCCESSOR_ID[spnid]$
		$nest_order = next_spn_id$

$		// 初期化ブロックのために次に取得するスピンロックIDを保存
		$SPN.NEXT_ID[spnid] = SPN.ID[next_spn_id]$

$		// デッドロックのチェック
		$IF !OMIT_CHECK_DEADLOCK_SPINLOCK$
$			// ネストの最大深さはスピンロックの数と同じなので，スピンロックの数だけループ
			$FOREACH id2 SPN.ID_LIST$
				$IF (LENGTH(SPN.SUCCESSOR[next_spn_id]) && !EQ(SPN.SUCCESSOR[next_spn_id],"NULL"))$
					$next_spn_id = SPN.SUCCESSOR_ID[next_spn_id]$

$					// ネスト順序リスト(nest_order)の中にsuccessorがあったらデッドロックするためエラー
					$IF LENGTH(FIND(nest_order, next_spn_id))$
						$ERROR SPN.TEXT_LINE[spnid]$$FORMAT(_(" %1% `%2%\' illegal nest order `%3%\'"), "spinlock", SPN.SPN[spnid], SPN.SPN[next_spn_id])$$END$
						$DIE()$
					$END$

					$nest_order = APPEND(nest_order , next_spn_id)$
				$END$
			$END$
		$END$
	$ELSE$
$		// 次に取得するスピンロックはないので，invalid_spinlockとしておく
		$SPN.NEXT_ID[spnid] = invalid_spinlock$
	$END$
$END$

$ OSAPがすべて同じコアに所属していないか？
$FOREACH spnid SPN.ID_LIST$
	$IF LENGTH(SPN.ACS_OSAP_LIST[spnid]) == 1$
		$ERROR SPN.TEXT_LINE[spnid]$$FORMAT(_("Spinlock must be used on different cores"))$$END$
		$DIE()$
	$END$
	$IF LENGTH(SPN.ACS_OSAP_LIST[spnid])$
		$diff = 0$
		$coreid = TMAX_COREID + 1$
		$FOREACH app SPN.ACS_OSAP_LIST[spnid]$
			$IF EQ(coreid, TMAX_COREID + 1)$
				$coreid = OSAP.CORE[app]$
			$ELIF !EQ(coreid, OSAP.CORE[app])$
				$diff = 1$
			$END$
		$END$
		$IF EQ(diff,0)$
			$ERROR SPN.TEXT_LINE[spnid]$$FORMAT(_("Spinlock must be used on different cores"))$$END$
		$END$
	$END$
$END$


$ =====================================================================
$ Os_Cfg_tmp.hの生成 Os_Cfg.hと差分がある場合Makefile内Os_Cfg.hへコピー
$ =====================================================================

$TRACE("OUTPUT FILES")$

$FILE "Os_Cfg_tmp.h"$
/* Os_Cfg.h */$NL$
#ifndef TOPPERS_OS_CFG_H$NL$
#define TOPPERS_OS_CFG_H$NL$
$NL$

/****** Scalability Class ******/$NL$$NL$

#define CFG_USE_SCALABILITYCLASS1$NL$
$NL$

/****** Object OS ******/$NL$$NL$
$IF OS.STACKMONITORING[1]$
	#define CFG_USE_STACKMONITORING$NL$
$END$

$IF OS.GETSERVICEID[1]$
	#define CFG_USE_GETSERVICEID$NL$
$END$

$IF OS.PARAMETERACCESS[1]$
	#define CFG_USE_PARAMETERACCESS$NL$
$END$

$IF EQ(OS.STATUS[1], "EXTENDED")$
	#define CFG_USE_EXTENDEDSTATUS$NL$
$END$

$USE_HOOK = 0$

$NL$
/****** Object HOOK ******/$NL$$NL$
$IF HOOK.STARTUPHOOK[1]$
	#define CFG_USE_STARTUPHOOK$NL$
	$USE_HOOK = 1$
$END$

$IF HOOK.SHUTDOWNHOOK[1]$
	#define CFG_USE_SHUTDOWNHOOK$NL$
	$USE_HOOK = 1$
$END$

$IF HOOK.PRETASKHOOK[1]$
	#define CFG_USE_PRETASKHOOK$NL$
	$USE_HOOK = 1$
$END$

$IF HOOK.POSTTASKHOOK[1]$
	#define CFG_USE_POSTTASKHOOK$NL$
	$USE_HOOK = 1$
$END$

$IF HOOK.ERRORHOOK[1]$
	#define CFG_USE_ERRORHOOK$NL$
	$USE_HOOK = 1$
$END$

$IF HOOK.PROTECTIONHOOK[1]$
	#define CFG_USE_PROTECTIONHOOK$NL$
	$USE_HOOK = 1$
$END$

$NL$
/****** Object MC ******/$NL$$NL$
#define TotalNumberOfCores $OS_NUMBER_OF_CORE_CORES$$NL$

$NL$
#endif /* TOPPERS_OS_CFG_H */$NL$


$ =====================================================================
$ Os_Lcfg.hの生成
$ =====================================================================

$FILE "Os_Lcfg.h"$
/* Os_Lcfg.h */$NL$
#ifndef TOPPERS_OS_LCFG_H$NL$
#define TOPPERS_OS_LCFG_H$NL$
$NL$

$
$ オブジェクト数マクロの出力
$
#define TNUM_ALARM				UINT_C($LENGTH(ALM.ID_LIST)$)$NL$
#define TNUM_COUNTER			UINT_C($LENGTH(CNT.ID_LIST)$)$NL$
#define TNUM_HARDCOUNTER		UINT_C($LENGTH(HWCNT.ID_LIST)$)$NL$
#define TNUM_ISR2				UINT_C($LENGTH(ISR2.ID_LIST) + LENGTH(ICI.ID_LIST)$)$NL$
#define TNUM_ICI				UINT_C($LENGTH(ICI.ID_LIST)$)$NL$
#define TNUM_STD_RESOURCE		UINT_C($LENGTH(STDRES.ID_LIST)$)$NL$
#define TNUM_TASK				UINT_C($LENGTH(TSK.ID_LIST)$)$NL$
#define TNUM_EXTTASK			UINT_C($tnum_exttask$)$NL$
#define TNUM_APP_MODE			UINT_C($LENGTH(APP.ID_LIST)$)$NL$
#define TNUM_SCHEDULETABLE		UINT_C($LENGTH(SCHTBL.ID_LIST)$)$NL$
#define TNUM_IMPLSCHEDULETABLE	UINT_C($tnum_implscheduletable$)$NL$
#define TNUM_OSAP				UINT_C($LENGTH(OSAP.ID_LIST)$)$NL$
#define TNUM_SPINLOCK			UINT_C($LENGTH(SPN.ID_LIST)$)$NL$
$NL$

/*$NL$
$SPC$*  Default Definitions of Trace Log Macros$NL$
$SPC$*/$NL$
$NL$
#ifndef TOPPERS_ENABLE_TRACE$NL$
#ifndef LOG_USER_MARK$NL$
#define LOG_USER_MARK(str)$NL$
#endif /* LOG_USER_MARK */$NL$
#endif /* TOPPERS_ENABLE_TRACE */$NL$
$NL$
 /****** Object TASK ******/$NL$$NL$

$ タスクのIDの出力
$FOREACH tskid TSK.ID_LIST$
	#define $tskid$$TAB$UINT_C($TSK.ID[tskid]$)$NL$
$END$
$NL$

 /****** Object COUNTER ******/$NL$$NL$

$ カウンタIDの出力

$FOREACH cntid CNT.ID_LIST$
	#define $cntid$$TAB$UINT_C($CNT.ID[cntid]$)$NL$
$END$
$NL$

$ カウンタオブジェクトのOS定数の出力
$FOREACH cntid CNT.ID_LIST$
	#define OSMAXALLOWEDVALUE_$cntid$$TAB$((TickType) $+CNT.MAXALLOWED[cntid]$)$NL$
	#define OSTICKSPERBASE_$cntid$$TAB$((TickType) $+CNT.TICKSPERBASE[cntid]$)$NL$
	#define OSMINCYCLE_$cntid$$TAB$((TickType) $+CNT.MINCYCLE[cntid]$)$NL$
$END$
$NL$

$ ティック値から時間への変換マクロ(PhysicalTimeTypeの型はfloat,doubleのいずれかと想定)
$FOREACH cntid HWCNT.ID_LIST$
	$IF LENGTH(CNT.NSPERTICK[cntid])$
		#define OS_TICKS2SEC_$cntid$(tick)$TAB$(((PhysicalTimeType)$CNT.NSPERTICK[cntid]$) * (tick) / 1000000000U)	/* ($CNT.SECONDSPERTICK[cntid]$ * 1000000000) * (tick) / 1000000000 */$NL$
		#define OS_TICKS2MS_$cntid$(tick)$TAB$(((PhysicalTimeType)$CNT.NSPERTICK[cntid]$) * (tick) / 1000000U)		/* ($CNT.SECONDSPERTICK[cntid]$ * 1000000000) * (tick) / 1000000 */$NL$
		#define OS_TICKS2US_$cntid$(tick)$TAB$(((PhysicalTimeType)$CNT.NSPERTICK[cntid]$) * (tick) / 1000U)			/* ($CNT.SECONDSPERTICK[cntid]$ * 1000000000) * (tick) / 1000 */$NL$
		#define OS_TICKS2NS_$cntid$(tick)$TAB$(((PhysicalTimeType)$CNT.NSPERTICK[cntid]$) * (tick))					/* ($CNT.SECONDSPERTICK[cntid]$ * 1000000000) * (tick) */$NL$
	$END$
	$NL$
$END$

 /****** Object ALARM ******/$NL$$NL$

$ アラームIDの出力
$FOREACH almid ALM.ID_LIST$
	#define $almid$$TAB$UINT_C($ALM.ID[almid]$)$NL$
$END$
$NL$

 /****** Object SCHEDULETABLE ******/$NL$$NL$

$ スケジュールテーブルIDの出力
$FOREACH schtblid SCHTBL.ID_LIST$
	#define $schtblid$$TAB$UINT_C($SCHTBL.ID[schtblid]$)$NL$
$END$
$NL$

 /****** Object RESOURCE ******/$NL$$NL$

$ リソースIDの出力
$FOREACH resid RES.ID_LIST$
	#define $resid$$TAB$UINT_C($RES.ID[resid]$)$NL$
$END$
$NL$

 /****** Object ISR ******/$NL$$NL$

$ ISRのIDの出力
$FOREACH isrid ISR.ID_LIST$
	#define $isrid$$TAB$UINT_C($ISR.ID[isrid]$)$NL$
$END$
$NL$

$FOREACH iciid ICI.ID_LIST$
	#define $iciid$$TAB$UINT_C($ICI.ID[iciid]$)$NL$
$END$
$NL$

 /****** Object APPMODE ******/$NL$$NL$

$FOREACH appid APP.ID_LIST$
	#define $appid$$TAB$UINT_C($APP.ID[appid]$)$NL$
$END$
$NL$

$ イベントの出力
 /****** Object EVENT ******/$NL$
$FOREACH evtid EVT.ID_LIST$
	#define $evtid$$TAB$UINT_C($FORMAT("0x%08x", +EVT.CALC_MASK[evtid])$)$NL$
$END$
$NL$

$ OSAPのIDの出力
 /****** Object OSApplication ******/$NL$
$FOREACH osapid OSAP.ID_LIST$
	#define $osapid$$TAB$UINT_C($OSAP.ID[osapid]$)$NL$
$END$
$NL$

 /****** Object SPINLOCK ******/$NL$
$ スピンロックIDの出力 
$FOREACH spnid SPN.ID_LIST$
	#define $spnid$$TAB$UINT_C($SPN.ID[spnid]$)$NL$
$END$

$NL$


$ =====================================================================
$ Os_Lcfg.cの生成
$ =====================================================================
$FILE "Os_Lcfg.c"$

/* Os_Lcfg.c */$NL$
#include "kernel/kernel_int.h"$NL$
#include "Os_Lcfg.h"$NL$
$IF LENGTH(IOC.ID_LIST)$
	#include "Ioc.h"$NL$
	$NL$
$END$
$NL$
#ifndef TOPPERS_EMPTY_LABEL$NL$
#define TOPPERS_EMPTY_LABEL(x, y) x y[0]$NL$
#endif$NL$
$NL$

$
$  トレースログマクロのデフォルト定義
$
/*$NL$
$SPC$*  Default Definitions of Trace Log Macros$NL$
$SPC$*/$NL$
$NL$
#ifndef LOG_ISR_ENTER$NL$
#define LOG_ISR_ENTER(isrid)$NL$
#endif /* LOG_ISR_ENTER */$NL$
$NL$
#ifndef LOG_ISR_LEAVE$NL$
#define LOG_ISR_LEAVE(isrid)$NL$
#endif /* LOG_ISR_LEAVE */$NL$
$NL$

$
$  インクルードディレクティブ（#include）
$
/*$NL$
$SPC$*  Include Directives (#include)$NL$
$SPC$*/$NL$
$NL$
$INCLUDES$
$NL$

$
$ オブジェクト数変数の出力
$
const AlarmType					kernel_tnum_alarm				= TNUM_ALARM;$NL$
const CounterType				kernel_tnum_counter			= TNUM_COUNTER;$NL$
const CounterType				kernel_tnum_hardcounter		= TNUM_HARDCOUNTER;$NL$
const ISRType					kernel_tnum_isr2				= TNUM_ISR2;$NL$
const ISRType					kernel_tnum_ici				= TNUM_ICI;$NL$
const ResourceType				kernel_tnum_stdresource		= TNUM_STD_RESOURCE;$NL$
const TaskType					kernel_tnum_task				= TNUM_TASK;$NL$
const TaskType					kernel_tnum_exttask			= TNUM_EXTTASK;$NL$
const AppModeType				kernel_tnum_appmode			= TNUM_APP_MODE;$NL$
const ScheduleTableType			kernel_tnum_scheduletable		= TNUM_SCHEDULETABLE;$NL$
const ScheduleTableType			kernel_tnum_implscheduletable	= TNUM_IMPLSCHEDULETABLE;$NL$
const ApplicationType			kernel_tnum_osap				= TNUM_OSAP;$NL$
const SpinlockIdType			kernel_tnum_spinlock			= TNUM_SPINLOCK;$NL$
$NL$

$
$ オブジェクト変数の出力
$

$IF USE_EXTERNAL_ID$

	$NL$
	/****** Object ID ******/$NL$
	$NL$

$ 	// タスクのIDの出力
	$FOREACH tskid TSK.ID_LIST$
		const TaskType $tskid$_id = $tskid$;$NL$
	$END$
	$NL$

$	// カウンタIDの出力
	$FOREACH cntid CNT.ID_LIST$
		const CounterType $cntid$_id = $cntid$;$NL$
	$END$
	$NL$

$	// アラームIDの出力
	$FOREACH almid ALM.ID_LIST$
		const AlarmType $almid$_id = $almid$;$NL$
	$END$
	$NL$

$	// リソースIDの出力
	$FOREACH resid RES.ID_LIST$
		const ResourceType $resid$_id = $resid$;$NL$
	$END$
	$NL$

$	// イベントIDの出力
	$FOREACH evtid EVT.ID_LIST$
		const EventMaskType $evtid$_id = $evtid$;$NL$
	$END$
	$NL$

$	// ISRのIDの出力
	$FOREACH isrid ISR.ID_LIST$
		const ISRType $isrid$_id = $isrid$;$NL$
	$END$
	$NL$

$END$

$ 
$ CCB
$ 
$NL$
 /****** CCB ******/$NL$
$NL$

$ CCBの出力
$FOREACH id RANGE(0, TMAX_COREID)$
	$IF ISFUNCTION("GENERATE_CCB")$
		$GENERATE_CCB(id)$
	$ELSE$
		CCB kernel_core$id$_ccb = {0U, FALSE, 0U, 0U, 0U, 0U, 0U};$NL$
	$END$
$END$
$NL$
$NL$

$ CCBテーブルの出力
CCB * const kernel_p_ccb_table[TotalNumberOfCores] = {$NL$

$JOINEACH id RANGE(0, TMAX_COREID) ",\n"$
	$TAB$&kernel_core$id$_ccb
$END$
$NL$};$NL$
$NL$

$
$  タスク
$
$NL$
 /****** Object TASK ******/$NL$
$NL$

$ スタック領域の生成とそれに関するエラーチェック
$FOREACH tskid TSK.ID_LIST$

$	// ターゲット定義の最小値（TARGET_MIN_STKSZ）よりも小さい場合
	$IF TARGET_MIN_STKSZ && (TSK.STKSZ[tskid] < TARGET_MIN_STKSZ)$
		$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskStackSize(%1%) of OsTask(%2%) is too small"), TSK.STKSZ[tskid], tskid)$$END$
	$END$
$ 	// stkszがスタック領域のサイズとして正しくない場合
	$IF !EQ(TSK.STK[tskid], "NULL") && CHECK_STKSZ_ALIGN
							&& (TSK.STKSZ[tskid] & (CHECK_STKSZ_ALIGN - 1))$
		$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskStackSize(%1%) of OsTask(%2%) is not aligned"), TSK.STKSZ[tskid], tskid)$$END$
	$END$

	$IF EQ(TSK.STK[tskid], "NULL")$
 		$IF TSK.EXTENDED[tskid]$
			$IF ISFUNCTION("GENERATE_TSKSTK")$
				$GENERATE_TSKSTK(tskid)$
			$ELSE$
				static StackType kernel_stack_$tskid$[COUNT_STK_T($TSK.STKSZ[tskid]$)];$NL$
			$END$
			$TSK.TINIB_STKSZ[tskid] = FORMAT("ROUND_STK_T(%1%)", TSK.STKSZ[tskid])$
			$TSK.TINIB_STK[tskid] = CONCAT("kernel_stack_", tskid)$
		$ELSE$
			$core_idx = OSAP.CORE[TSK.OSAPID[tskid]] * TNUM_TPRI$
 			$TSK.SHARED_STK_ID[tskid] = core_idx + TSK.PRIORITY[tskid]$
 			$TSK.TINIB_STK[tskid] = CONCAT("kernel_shared_stack_", TSK.SHARED_STK_ID[tskid])$
$ 			// 基本タスク用の共有スタックのサイズを求める
 			$IF !LENGTH(shared_stack_size[TSK.SHARED_STK_ID[tskid]])
 					|| shared_stack_size[TSK.SHARED_STK_ID[tskid]] < TSK.STKSZ[tskid]$
 				$shared_stack_size[TSK.SHARED_STK_ID[tskid]] = TSK.STKSZ[tskid]$
 			$END$
 		$END$
	$ELSE$
		$TSK.TINIB_STKSZ[tskid] = TSK.STKSZ[tskid]$
		$TSK.TINIB_STK[tskid] = TSK.STK[tskid]$
	$END$
$END$

$FOREACH tskid TSK.ID_LIST$
	$IF LENGTH(TSK.SHARED_STK_ID[tskid])$
		$TSK.TINIB_STKSZ[tskid] = FORMAT("ROUND_STK_T(%1%)", shared_stack_size[TSK.SHARED_STK_ID[tskid]])$
	$END$
$END$

$FOREACH coreid RANGE(0, TMAX_COREID)$
	$FOREACH tskpri RANGE(TMIN_TPRI, TMAX_TPRI)$
		$shared_stkid = coreid * TNUM_TPRI + tskpri$
		$IF LENGTH(shared_stack_size[shared_stkid])$
			$IF ISFUNCTION("GENERATE_SHARED_TSKSTK")$
				$GENERATE_SHARED_TSKSTK(coreid, shared_stkid, shared_stack_size[shared_stkid])$
			$ELSE$
				static StackType kernel_shared_stack_$shared_stkid$[COUNT_STK_T($shared_stack_size[shared_stkid]$)];$NL$
			$END$
		$END$
	$END$
$END$
$NL$


$IF LENGTH(TSK.ID_LIST)$
$	// タスク初期化ブロックの出力
	const TINIB kernel_tinib_table[TNUM_TASK] = {$NL$
	$JOINEACH tskid TSK.ID_LIST ",\n"$
		$TAB${$NL$
		$TAB$$TAB$&TASKNAME($tskid$),$NL$
		$IF USE_TSKINICTXB$
			$GENERATE_TSKINICTXB(tskid)$
		$ELSE$
			$TAB$$TAB$$TSK.TINIB_STKSZ[tskid]$,$NL$
			$TAB$$TAB$$TSK.TINIB_STK[tskid]$,$NL$
		$END$
		$TAB$$TAB$&(kernel_osapinib_table[$TSK.OSAPID[tskid]$]),$NL$
		$TAB$$TAB$$TMAX_TPRI - TSK.PRIORITY[tskid]$,$NL$
		$IF EQ(TSK.SCHEDULE[tskid], "NON")$
			$TAB$$TAB$$+TPRI_MAXTASK$,$NL$
		$ELSE$
			$IF LENGTH(TSK.INRESPRI[tskid])$
				$TAB$$TAB$$TMAX_TPRI - TSK.INRESPRI[tskid]$,$NL$
			$ELSE$
				$TAB$$TAB$$TMAX_TPRI - TSK.PRIORITY[tskid]$,$NL$
			$END$
		$END$
		$TAB$$TAB$($+TSK.ACTIVATION[tskid]$U) - 1U,$NL$
		$TAB$$TAB$$FORMAT("0x%08xU", +TSK.ASTPTN[tskid])$,$NL$
		$TAB$$TAB$&kernel_core$+OSAP.CORE[TSK.OSAPID[tskid]]$_ccb$NL$
		$TAB$}
	$END$
	$NL$
	};$NL$
	$NL$

$	// TCBの出力
	$FOREACH tskid TSK.ID_LIST$
	$IF ISFUNCTION("GENERATE_TCB")$
		$GENERATE_TCB(tskid)$
	$ELSE$
		TCB kernel_tcb_$tskid$;$NL$
	$END$
	$END$$NL$
$	// TCBのテーブル
	TCB * const kernel_p_tcb_table[TNUM_TASK] ={$NL$
	$JOINEACH tskid TSK.ID_LIST ",\n"$
		$TAB$&kernel_tcb_$tskid$
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const TINIB, kernel_tinib_table);$NL$
	TOPPERS_EMPTY_LABEL(TCB * const, kernel_p_tcb_table);$NL$
$END$

$
$ カウンタの出力
$
$NL$
 /****** Object COUNTER ******/$NL$
$NL$

$IF LENGTH(CNT.ID_LIST)$
	const CNTINIB kernel_cntinib_table[TNUM_COUNTER] = {$NL$
	$JOINEACH cntid CNT.ID_LIST ",\n"$
		$TAB${ $+CNT.MAXALLOWED[cntid]$U,$SPC$
		($+CNT.MAXALLOWED[cntid]$U * 2U) + 1U,$SPC$
		$+CNT.TICKSPERBASE[cntid]$U,$SPC$
		$+CNT.MINCYCLE[cntid]$U,$SPC$
		$OSAP.CORE[CNT.OSAPID[cntid]]$,$SPC$
		&(kernel_osapinib_table[$CNT.OSAPID[cntid]$]) }
	$END$
	$NL$
	};$NL$
	$NL$

	$FOREACH cntid CNT.ID_LIST$
		$IF ISFUNCTION("GENERATE_CNTCB")$
			$GENERATE_CNTCB(cntid)$
		$ELSE$
			CNTCB kernel_cntcb_$cntid$;$NL$
		$END$
	$END$$NL$

	CNTCB * const kernel_p_cntcb_table[TNUM_COUNTER] = {$NL$
	$JOINEACH cntid CNT.ID_LIST ",\n"$
		$TAB$&kernel_cntcb_$cntid$
	$END$
	$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const CNTINIB, kernel_cntinib_table);$NL$
	TOPPERS_EMPTY_LABEL(CNTCB * const, kernel_p_cntcb_table);$NL$
$END$

$ ハードウェアカウンタポインタテーブル定義の出力
$IF LENGTH(HWCNT.ID_LIST)$
	const HWCNTINIB kernel_hwcntinib_table[TNUM_HARDCOUNTER] = $NL$
	{$NL$
	$JOINEACH cntid HWCNT.ID_LIST ",\n"$
		$TAB${$NL$
			$TAB$$TAB$&init_hwcounter_$cntid$,$NL$
			$TAB$$TAB$&start_hwcounter_$cntid$,$NL$
			$TAB$$TAB$&stop_hwcounter_$cntid$,$NL$
			$TAB$$TAB$&set_hwcounter_$cntid$,$NL$
			$TAB$$TAB$&get_hwcounter_$cntid$,$NL$
			$TAB$$TAB$&cancel_hwcounter_$cntid$,$NL$
			$TAB$$TAB$&trigger_hwcounter_$cntid$,$NL$
			$TAB$$TAB$&int_clear_hwcounter_$cntid$,$NL$
			$TAB$$TAB$&int_cancel_hwcounter_$cntid$,$NL$
			$TAB$$TAB$&increment_hwcounter_$cntid$,$NL$
			$TAB$$TAB$$CNT.NSPERTICK[cntid]$,		/* $CNT.SECONDSPERTICK[cntid]$ * 1000000000 */ $NL$
			$TAB$$TAB$$OSAP.CORE[CNT.OSAPID[cntid]]$$NL$
		$TAB$}
	$END$
	$NL$};$NL$

$ELSE$
	TOPPERS_EMPTY_LABEL(const HWCNTINIB, kernel_hwcntinib_table);$NL$
$END$
$NL$


$
$ アラームの出力
$
$NL$
 /****** Object ALARM ******/$NL$
$NL$

$FOREACH almid ALM.ID_LIST$

$	// タスク起動用のアクション関数
	$IF EQ(ALM.ALMATR[almid], "ACTIVATETASK")$
		static void$NL$
		activate_alarm_$+almid$(void);$NL$
		static void$NL$
		activate_alarm_$+almid$(void)$NL$
		{$NL$
		$TAB$(void) kernel_activate_task_action($ALM.TSKID[almid]$);$NL$
		}$NL$

$	// イベントセット用のアクション関数
	$ELIF EQ(ALM.ALMATR[almid], "SETEVENT")$
		static void$NL$
		setevent_alarm_$+almid$(void);$NL$
		static void$NL$
		setevent_alarm_$+almid$(void)$NL$
		{$NL$
		$TAB$(void) kernel_set_event_action($ALM.TSKID[almid]$, $ALM.EVTID[almid]$);$NL$
		}$NL$

$	// IncrementCounter用のアクション関数
	$ELIF EQ(ALM.ALMATR[almid], "INCREMENTCOUNTER")$
		static void$NL$
		incrementcounter_alarm_$+almid$(void);$NL$
		static void$NL$
		incrementcounter_alarm_$+almid$(void)$NL$
		{$NL$
		$TAB$(void) kernel_incr_counter_action($ALM.INCID[almid]$);$NL$
		}$NL$
	$END$

	$NL$
$END$

$ アラームコントロールブロック

$IF LENGTH(ALM.ID_LIST)$
	const ALMINIB kernel_alminib_table[TNUM_ALARM] = {$NL$
	$JOINEACH almid ALM.ID_LIST ",\n"$
		$TAB${ &kernel_cntcb_$ALM.CNTID[almid]$,$SPC$
		$IF EQ(ALM.ALMATR[almid], "ACTIVATETASK")$
$			// タスク起動用
			&activate_alarm_$+almid$,$SPC$
		$ELIF EQ(ALM.ALMATR[almid], "SETEVENT")$
$			// イベントセット用
			&setevent_alarm_$+almid$,$SPC$
		$ELIF EQ(ALM.ALMATR[almid], "INCREMENTCOUNTER")$
$			// IncrementCounter用
			&incrementcounter_alarm_$+almid$,$SPC$
		$ELIF EQ(ALM.ALMATR[almid], "CALLBACK")$
$			// コールバック用
			ALARMCALLBACKNAME($ALM.CALLBACK[almid]$),$SPC$
		$END$

		$FORMAT("0x%08xU", +ALM.ASTPTN[almid])$,$SPC$
		$IF ALM.ASTPTN[almid]$
			$+ALM.ALARMTIME[almid]$U,$SPC$
			$+ALM.CYCLETIME[almid]$U,$SPC$
$			// アラームコールバックと自動起動属性以外の属性を管理しない
			($ALM.ALMATR[almid]$ & CALLBACK) | $ALM.AUTOSTARTTYPE[almid]$,$SPC$
		$ELSE$
			0U, 0U, $ALM.ALMATR[almid]$ & CALLBACK,$SPC$
		$END$
		&(kernel_osapinib_table[$ALM.OSAPID[almid]$])$SPC$}
	$END$
	$NL$
	};$NL$
	$NL$

	$FOREACH almid ALM.ID_LIST$
		$IF ISFUNCTION("GENERATE_ALMCB")$
			$GENERATE_ALMCB(almid)$
		$ELSE$
			ALMCB kernel_almcb_$almid$;$NL$
		$END$
	$END$$NL$

	ALMCB * const kernel_p_almcb_table[TNUM_ALARM] = {$NL$
	$JOINEACH almid ALM.ID_LIST ",\n"$
		$TAB$&kernel_almcb_$almid$
	$END$
	$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const ALMINIB, kernel_alminib_table);$NL$
	TOPPERS_EMPTY_LABEL(ALMCB * const, kernel_p_almcb_table);$NL$
$END$


$
$ スケジュールテーブルの出力
$
$NL$
 /****** Object SCHEDULETABLE ******/$NL$
$NL$

$ 各スケジュールテーブルの各満了点のアクション関数と満了点管理ブロックの生成
$FOREACH schtblid SCHTBL.ID_LIST$
	/* Object SCHEDULETABLE($schtblid$) */$NL$
	$NL$
	$FOREACH exppt_index RANGE(0, LENGTH(SCHTBL.EXPPTINDEX_LIST[schtblid]) - 1)$
		$expptid = AT(SCHTBL.EXPPTINDEX_LIST[schtblid], exppt_index)$
		static void$NL$
		expire_scheduletable_$+schtblid$_$exppt_index$(void);$NL$
		static void$NL$
		expire_scheduletable_$+schtblid$_$exppt_index$(void)$NL$
		{$NL$
$		// タスク起動
		$FOREACH expptactid EXPPTACT.ID_LIST$
			$IF EXPPTACT.EXPPTID[expptactid] == expptid && EQ(EXPPTACT.EXPIREATR[expptactid], "ACTIVATETASK")$
				$TAB$(void) kernel_activate_task_action($EXPPTACT.TSKID[expptactid]$);$NL$
			$END$
		$END$
$		// イベントセット
		$FOREACH expptactid EXPPTACT.ID_LIST$
			$IF EXPPTACT.EXPPTID[expptactid] == expptid && EQ(EXPPTACT.EXPIREATR[expptactid], "SETEVENT")$
				$TAB$(void) kernel_set_event_action($EXPPTACT.TSKID[expptactid]$, $EXPPTACT.EVTID[expptactid]$);$NL$
			$END$
		$END$
		}$NL$
	$END$
	$NL$
	static const SCHTBLEXPPTCB schtblexppt_table_$+schtblid$[$LENGTH(SCHTBL.EXPPTINDEX_LIST[schtblid])$] = {$NL$
	$JOINEACH exppt_index RANGE(0, LENGTH(SCHTBL.EXPPTINDEX_LIST[schtblid]) - 1) ",\n"$
		$expptid = AT(SCHTBL.EXPPTINDEX_LIST[schtblid], exppt_index)$
		$TAB${ $EXPPT.OFFSET[expptid]$U, &expire_scheduletable_$+schtblid$_$exppt_index$ }
	$END$
	$NL$
	};$NL$
	$NL$
$END$
$NL$

$ スケジュールテーブル関連のデータブロックの宣言
$IF LENGTH(SCHTBL.ID_LIST)$
	const SCHTBLINIB kernel_schtblinib_table[TNUM_SCHEDULETABLE] = {$NL$
	$JOINEACH schtblid SCHTBL.ID_LIST ",\n"$
		$TAB${ &kernel_cntcb_$SCHTBL.CNTID[schtblid]$,$SPC$
		$+SCHTBL.DURATIONTICK[schtblid]$U,$SPC$
		$FORMAT("0x%08xU", +SCHTBL.ASTPTN[schtblid])$,$SPC$
		$IF SCHTBL.ASTPTN[schtblid]$
			$SCHTBL.AUTOSTARTTYPE[schtblid]$,$SPC$
			$+SCHTBL.STARTTICK[schtblid]$U, $SPC$
		$ELSE$
			0U, 0U,$SPC$
		$END$
		schtblexppt_table_$+schtblid$,$SPC$
		$IF SCHTBL.REPEAT[schtblid]$
			TRUE,$SPC$
		$ELSE$
			FALSE,$SPC$
		$END$
		$LENGTH(SCHTBL.EXPPTINDEX_LIST[schtblid])$U,$SPC$
		&(kernel_osapinib_table[$SCHTBL.OSAPID[schtblid]$]) }
	$END$
	$NL$
	};$NL$
	$NL$

	$FOREACH schtblid SCHTBL.ID_LIST$
		$IF ISFUNCTION("GENERATE_SCHTBLCB")$
			$GENERATE_SCHTBLCB(schtblid)$
		$ELSE$
			SCHTBLCB kernel_schtblcb_$schtblid$;$NL$
		$END$
	$END$$NL$

	SCHTBLCB * const kernel_p_schtblcb_table[TNUM_SCHEDULETABLE] = {$NL$
	$JOINEACH schtblid SCHTBL.ID_LIST ",\n"$
		$TAB$&kernel_schtblcb_$schtblid$
	$END$
	$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const SCHTBLINIB, kernel_schtblinib_table);$NL$
	TOPPERS_EMPTY_LABEL(SCHTBLCB * const, kernel_p_schtblcb_table);$NL$
$END$

$NL$


$
$ リソースの出力
$
$NL$
 /****** Object RESOURCE ******/$NL$
$NL$

$IF LENGTH(STDRES.ID_LIST)$
	const RESINIB kernel_resinib_table[TNUM_STD_RESOURCE] = {$NL$
	$JOINEACH resid STDRES.ID_LIST ",\n"$
		$TAB${ 
		$IF RES.CEILPRI[resid] >= 0$
$           タスクのみが扱うリソースの場合
			$TMAX_TPRI - RES.CEILPRI[resid]$, 
		$ELSE$
$           ISRも扱うリソースの場合
			$RES.CEILPRI[resid]$, 
		$END$
		$+RES.CORE[resid]$U }
	$END$
	$NL$
	};$NL$
	$NL$

	$FOREACH resid STDRES.ID_LIST$
		$IF ISFUNCTION("GENERATE_RESCB")$
			$GENERATE_RESCB(resid)$
		$ELSE$
			RESCB kernel_rescb_$resid$;$NL$
		$END$
	$END$$NL$

	RESCB * const kernel_p_rescb_table[TNUM_STD_RESOURCE] = {$NL$
	$JOINEACH resid STDRES.ID_LIST ",\n"$
		$TAB$&kernel_rescb_$resid$
	$END$
	$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const RESINIB, kernel_resinib_table);$NL$
	TOPPERS_EMPTY_LABEL(RESCB * const, kernel_p_rescb_table);$NL$
$END$

$NL$


$
$ ISRの出力
$
$NL$
 /****** Object ISR(include InterCoreInterrupts) ******/$NL$
$NL$
$NL$

$IF LENGTH(ISR2.ID_LIST) || LENGTH(ICI.ID_LIST)$
$	// C2ISR用の出力
	const ISRINIB kernel_isrinib_table[TNUM_ISR2] = {$NL$
	$JOINEACH isrid ISR2.ID_LIST ",\n"$
		$TAB${$NL$
		$TAB$$TAB$&(kernel_intinib_table[$isrid$]),$SPC$
		&(kernel_osapinib_table[$ISR.OSAPID[isrid]$])$NL$
		$TAB$}
	$END$

	$IF LENGTH(ISR2.ID_LIST) && LENGTH(ICI.ID_LIST)$
		,$NL$
	$END$

$	// ICISR用の出力
	$JOINEACH iciid ICI.ID_LIST ",\n"$
		$TAB${$NL$
		$TAB$$TAB$NULL,$SPC$
		&(kernel_osapinib_table[$ICI.OSAPID[iciid]$])$NL$
		$TAB$}
	$END$
	$NL$
	};$NL$
	$NL$

	$FOREACH isrid ISR2.ID_LIST$
		$IF ISFUNCTION("GENERATE_ISRCB")$
			$GENERATE_ISRCB(isrid)$
		$ELSE$
			ISRCB kernel_isrcb_$isrid$;$NL$
		$END$
	$END$$NL$

	$FOREACH iciid ICI.ID_LIST$
		$IF ISFUNCTION("GENERATE_ICICB")$
			$GENERATE_ICICB(iciid)$
		$ELSE$
			ISRCB kernel_icicb_$iciid$;$NL$
		$END$
	$END$$NL$

	ISRCB * const kernel_p_isrcb_table[TNUM_ISR2] = {$NL$
	$JOINEACH isrid ISR2.ID_LIST ",\n"$
		$TAB$&kernel_isrcb_$isrid$
	$END$

	$IF LENGTH(ISR2.ID_LIST) && LENGTH(ICI.ID_LIST)$
		,$NL$
	$END$

	$JOINEACH iciid ICI.ID_LIST ",\n"$
		$TAB$&kernel_icicb_$iciid$
	$END$
	$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const ISRINIB, kernel_isrinib_table);$NL$
	TOPPERS_EMPTY_LABEL(ISRCB * const, kernel_p_isrcb_table);$NL$
$END$
$NL$

$NL$
/****** Object InterCoreInterrupts ******/$NL$
$NL$

$ 
$  コア間割込み初期化ブロック
$ 
$IF LENGTH(ICI.ID_LIST)$
	const ICIINIB kernel_iciinib_table[TNUM_ICI] = {$NL$
	$JOINEACH iciid ICI.ID_LIST ",\n"$
		$TAB${&ICISRNAME($iciid$), $ICI.BITPTN[iciid]$U, $+ICI.ID[iciid]$U}
	$END$
	$NL$
	};$NL$

	const ICIINIB * kernel_p_iciinb_table[TotalNumberOfCores] = {$NL$
	$JOINEACH coreid RANGE(0, TMAX_COREID) ",\n"$
		$IF LENGTH(CORE.ICIID[coreid])$
			$TAB$&kernel_iciinib_table[$CORE.ICIID_CNT[coreid]$]
		$ELSE$
			$TAB$NULL
		$END$
	$END$
	$NL$
	};$NL$
	const uint8 kernel_tnum_ici_of_core[TotalNumberOfCores] = {$NL$
	$JOINEACH coreid RANGE(0, TMAX_COREID) ",\n"$
		$TAB$$CORE.ICISR_NUM[coreid]$U
	$END$
	$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const ICIINIB, kernel_iciinib_table);$NL$
	const ICIINIB * kernel_p_iciinb_table[TotalNumberOfCores] = {0};$NL$
	const uint8 kernel_tnum_ici_of_core[TotalNumberOfCores] = {0};$NL$
$END$
$NL$

$
$  OSアプリケーション初期化ブロックの出力
$
$NL$
$IF LENGTH(OSAP.ID_LIST)$
	const OSAPINIB kernel_osapinib_table[TNUM_OSAP] = {$NL$
	$JOINEACH osapid OSAP.ID_LIST ",\n"$
		$TAB${$NL$
		$TAB$$TAB$$OSAP.CORE[osapid]$$NL$
		$TAB$}
	$END$
	$NL$
	};$NL$
	$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const OSAPINIB, kernel_osapinib_table);$NL$
$END$


$ 
$ スピンロックの出力
$ 
 /****** Object SPINLOCK ******/$NL$
$ 初期化ブロックの出力
$IF LENGTH(SPN.ID_LIST)$
	const SPNINIB kernel_spninib_table[TNUM_SPINLOCK] = {$NL$
	$JOINEACH spnid SPN.ID_LIST ",\n"$
		$TAB${$NL$
		$IF EQ(SPN.LOCKMETHOD[spnid], "LOCK_ALL_INTERRUPTS")$
			$TAB$$TAB$&kernel_wrap_sus_all_int,$NL$
			$TAB$$TAB$&kernel_wrap_res_all_int,$NL$
		$ELIF EQ(SPN.LOCKMETHOD[spnid], "LOCK_CAT2_INTERRUPTS")$
			$TAB$$TAB$&kernel_wrap_sus_os_int,$NL$
			$TAB$$TAB$&kernel_wrap_res_os_int,$NL$
		$ELSE$
			$TAB$$TAB$NULL,$NL$
			$TAB$$TAB$NULL,$NL$
		$END$
		$IF SPN.NEXT_ID[spnid] == invalid_spinlock$
			$TAB$$TAB$NULL$NL$
		$ELSE$
			$TAB$$TAB$&kernel_spninib_table[$SPN.NEXT_ID[spnid] - 1$]$NL$
		$END$
		$TAB$}
	$END$
	$NL$
	};$NL$
	$NL$
	
	SPNCB kernel_spncb_table[TNUM_SPINLOCK];$NL$
	$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const SPNINIB, kernel_spninib_table);$NL$
	TOPPERS_EMPTY_LABEL(SPNCB, kernel_spncb_table);$NL$
$END$

$
$ オブジェクト初期化ルーチン
$
$NL$
void$NL$
kernel_object_initialize(void)$NL$
{$NL$
$TAB$kernel_interrupt_initialize();$NL$
$IF LENGTH(STDRES.ID_LIST)$
$TAB$kernel_resource_initialize();$NL$
$END$
$TAB$kernel_task_initialize();$NL$
$IF LENGTH(CNT.ID_LIST)$
$TAB$kernel_counter_initialize();$NL$
$END$
$IF LENGTH(ALM.ID_LIST)$
$TAB$kernel_alarm_initialize();$NL$
$END$
$IF LENGTH(SCHTBL.ID_LIST)$
$TAB$kernel_schtbl_initialize();$NL$
$END$
$IF LENGTH(SPN.ID_LIST)$
$TAB$kernel_spinlock_initialize();$NL$
$END$
$IF LENGTH(IOC.ID_LIST)$
$TAB$kernel_ioc_initialize();$NL$
$END$
}$NL$
$NL$

$
$ オブジェクト終了処理ルーチン
$
$NL$
void$NL$
kernel_object_terminate(void)$NL$
{$NL$
$IF LENGTH(CNT.ID_LIST)$
$TAB$kernel_counter_terminate();$NL$
$END$
}$NL$
$NL$

$
$  割込み管理機能
$
$NL$
/*$NL$
$SPC$*  Interrupt Management Functions$NL$
$SPC$*/$NL$
$NL$

$ ISR用の割込みハンドラ
$FOREACH isrid ISR2.ID_LIST$
	void$NL$
	$ISR.INT_ENTRY[isrid]$(void)$NL$
	{$NL$
	$TAB$i_begin_int($+ISR.INTNO[isrid]$U);$NL$
	$TAB$LOG_ISR_ENTER($isrid$);$NL$
	$TAB$ISRNAME($isrid$)();$NL$
	$TAB$LOG_ISR_LEAVE($isrid$);$NL$
	$TAB$i_end_int($+ISR.INTNO[isrid]$U);$NL$
	}$NL$
$END$
$NL$

$ ハードウェアカウンタ割込み処理
/* HardWare Counter Interrupt Handler(C2ISR) */$NL$
$FOREACH cntid HWCNT.ID_LIST$
	ISR($CNT.ISRID[cntid]$)$NL$
	{$NL$
		$TAB$kernel_notify_hardware_counter($cntid$);$NL$
	}$NL$
$END$
$NL$

$
$  非タスクコンテキスト用のスタック領域
$
/*$NL$
$SPC$*  Stack Area for Non-task Context$NL$
$SPC$*/$NL$
$NL$

$ 非タスクコンテキスト用のスタック領域計算の為の変数初期化
$FOREACH coreid RANGE(0, TMAX_COREID)$
	$ICI.STKSZ_2[coreid] = 0$
$END$

$ 非タスクコンテキスト用のスタック領域サイズ計算
$ // ICISR使用するスタックサイズの計算
$FOREACH iciid ICI.ID_LIST$
$ 	// NULL(0)か0を指定した場合，デフォルト値設定
	$ICI.INTPRI[iciid] = +AT(INTPRI_ICI_LIST, OSAP.CORE[ICI.OSAPID[iciid]])$
	$IF ICI.STKSZ[iciid] == 0$
		$ICI.STKSZ[iciid] = DEFAULT_ISRSTKSZ$
	$END$

$	// 非タスクコンテキスト用のスタック領域計算の為，優先度毎最大値の合計準備（ICISR）
	$core_idx = OSAP.CORE[ICI.OSAPID[iciid]] * TNUM_INTPRI$
	$IF LENGTH(ICI.STKSZ[iciid]) && (!LENGTH(isr_shared_stack_size[core_idx + ICI.INTPRI[iciid]])
			|| isr_shared_stack_size[core_idx + ICI.INTPRI[iciid]] < ICI.STKSZ[iciid])$
		$isr_shared_stack_size[core_idx + ICI.INTPRI[iciid]] = ICI.STKSZ[iciid]$
		$ICI.STKSZ_2[OSAP.CORE[ICI.OSAPID[iciid]]] = ICI.STKSZ[iciid]$
	$END$
$END$

$ // C2ISR使用するスタックサイズの計算
$FOREACH isrid ISR2.ID_LIST$
$ 	// NULL(0)か0を指定した場合，デフォルト値設定
	$IF ISR.STKSZ[isrid] == 0$
		$ISR.STKSZ[isrid] = DEFAULT_ISRSTKSZ$
	$END$

$	// 非タスクコンテキスト用のスタック領域計算の為，優先度毎最大値の合計準備（C2ISR）
	$core_idx = OSAP.CORE[ISR.OSAPID[isrid]] * TNUM_INTPRI$
	$IF LENGTH(ISR.STKSZ[isrid]) && (!LENGTH(isr_shared_stack_size[core_idx + ISR.INTPRI[isrid]])
			|| isr_shared_stack_size[core_idx + ISR.INTPRI[isrid]] < ISR.STKSZ[isrid])$
		$isr_shared_stack_size[core_idx + ISR.INTPRI[isrid]] = ISR.STKSZ[isrid]$
	$END$
$END$


$ ISRスタック領域のサイズを合計
/* calculate stack size for MC */$NL$
$FOREACH coreid RANGE(0, TMAX_COREID)$
$	// MINIMUM_OSTKSZは，非タスクコンテキストスタックサイズの最小値として，依存部に設置
	$total_stksz[coreid] = MINIMUM_OSTKSZ$
	$core_idx = coreid * TNUM_INTPRI$
	$FOREACH isrpri RANGE(-MAX_PRI_ISR2[coreid], -MIN_PRI_ISR2[coreid])$
		$IF LENGTH(isr_shared_stack_size[core_idx + isrpri])$
			$total_stksz[coreid] = total_stksz[coreid] + isr_shared_stack_size[core_idx + isrpri]$
		$END$
	$END$
$END$

$IF USE_HOOK$
	$FOREACH coreid RANGE(0, TMAX_COREID)$
$ 		// スタックサイズのデフォルト設定
$ 		// NULL(0)か0を指定した場合，デフォルト値設定
		$IF LENGTH(HSTK.COREID[coreid]) && (HSTK.STKSZ[HSTK.COREID[coreid]] != 0)$
			$total_stksz[coreid] = total_stksz[coreid] + HSTK.STKSZ[HSTK.COREID[coreid]]$
		$ELSE$
			$total_stksz[coreid] = total_stksz[coreid] + DEFAULT_HOOKSTKSZ$
		$END$
	$END$
$END$

$ スタック残量チェック方式用のスタック残量サイズ計算
$FOREACH isrid ISR.ID_LIST$
	$ISR.STKSZ_2[isrid] = ISR.STKSZ[isrid]$
	$IF (EQ(ISR.CATEGORY[isrid], "CATEGORY_2"))$
		$core_idx = OSAP.CORE[ISR.OSAPID[isrid]] * TNUM_INTPRI$
		$FOREACH isrpri	RANGE(-ISR.INTPRI[isrid], -MIN_PRI_ISR2[OSAP.CORE[ISR.OSAPID[isrid]]])$
			$IF ISR.INTPRI[isrid] < isrpri$
				$IF LENGTH(isr_shared_stack_size[core_idx + isrpri])$
					$ISR.STKSZ_2[isrid] = ISR.STKSZ_2[isrid] + isr_shared_stack_size[core_idx + isrpri]$
				$END$
			$END$
		$END$
	$END$
$END$

$ ICISRスタック残量チェック方式用のスタック残量サイズ計算
$FOREACH coreid RANGE(0, TMAX_COREID)$
	$core_idx = coreid * TNUM_INTPRI$
	$FOREACH isrpri	RANGE(+AT(INTPRI_ICI_LIST, coreid), -MIN_PRI_ISR2[coreid])$
		$IF +AT(INTPRI_ICI_LIST, coreid) < isrpri$
			$IF LENGTH(isr_shared_stack_size[core_idx + isrpri])$
				$ICI.STKSZ_2[coreid] = ICI.STKSZ_2[coreid] + isr_shared_stack_size[core_idx + isrpri]$
			$END$
		$END$
	$END$
$END$

$ フック有効時，スタック残量チェック方式用スタックサイズを加算
$IF USE_HOOK$
	$FOREACH coreid RANGE(0, TMAX_COREID)$
		$FOREACH isrid2 ISR2.ID_LIST$
			$IF OSAP.CORE[ISR.OSAPID[isrid2]] == coreid$
			    $IF LENGTH(HSTK.COREID[coreid]) && (HSTK.STKSZ[HSTK.COREID[coreid]] != 0)$
    	    		    $ISR.STKSZ_2[isrid2] = ISR.STKSZ_2[isrid2] + HSTK.STKSZ[HSTK.COREID[coreid]]$
			  	$ELSE$
					$ISR.STKSZ_2[isrid2] = ISR.STKSZ_2[isrid2] + DEFAULT_HOOKSTKSZ$
    	    		$END$
   	   		$END$
		$END$
	$END$
$END$

$ フック有効時，ICISRスタック残量チェック方式用のスタックサイズを加算
$IF USE_HOOK$
	$FOREACH coreid RANGE(0, TMAX_COREID)$
		$IF LENGTH(HSTK.COREID[coreid]) && (HSTK.STKSZ[HSTK.COREID[coreid]] != 0)$
			$ICI.STKSZ_2[coreid] = ICI.STKSZ_2[coreid] + HSTK.STKSZ[HSTK.COREID[coreid]]$
		$ELSE$
			$ICI.STKSZ_2[coreid] = ICI.STKSZ_2[coreid] + DEFAULT_HOOKSTKSZ$
		$END$
	$END$
$END$

$ 割込み管理機能のための標準的な初期化情報の生成
$IF !OMIT_INITIALIZE_INTERRUPT$

$	// 割込み要求ライン数
	#define TNUM_INTNO	UINT_C($LENGTH(ISR.ID_LIST)$)$NL$
	const InterruptNumberType kernel_tnum_intno = TNUM_INTNO;$NL$
	$NL$

$	// 割込み要求ライン初期化テーブル
	$IF LENGTH(ISR.ID_LIST)$
		const INTINIB kernel_intinib_table[TNUM_INTNO] = {$NL$
		$JOINEACH isrid ISR.ID_LIST ",\n"$
			$TAB${ ($+ISR.INTNO[isrid]$U), $ISR.INTATR[isrid]$, ($-ISR.INTPRI[isrid]$), $OSAP.CORE[ISR.OSAPID[isrid]]$, 
			$IF OS.STACKMONITORING[1]$
				$FORMAT("0x%xU",+ISR.STKSZ_2[isrid])$
			$END$
			 }
		$END$$NL$
		};$NL$
	$ELSE$
		TOPPERS_EMPTY_LABEL(const INTINIB, kernel_intinib_table);$NL$
	$END$
	$NL$
$END$

$IF OS.STACKMONITORING[1]$
$ // ICISRスタック残量チェック用スタックサイズ情報生成
const MemorySizeType kernel_ici_remain_stksz[TotalNumberOfCores] = {$NL$
$JOINEACH coreid RANGE(0,TMAX_COREID) ",\n"$
	$TAB$ $FORMAT("0x%xU",+ICI.STKSZ_2[coreid])$
$END$$NL$
};$NL$
$END$

$ $	// ICISRスタック残量チェック用スタックサイズ情報生成
$ 	$IF LENGTH(ICI.ID_LIST)$
$ 		const MemorySizeType kernel_ici_remain_stksz[TotalNumberOfCores] = {$NL$
$ 		$JOINEACH coreid RANGE(0,TMAX_COREID) ",\n"$
$ 			$TAB$ $FORMAT("0x%x",+ICI.STKSZ_2[coreid])$
$ 		$END$$NL$
$ 		};$NL$
$ 	$ELSE$
$ 		TOPPERS_EMPTY_LABEL(const MemorySizeType, kernel_ici_remain_stksz);$NL$
$ 	$END$
$ 	$NL$
$ $END$
$ 
$FOREACH coreid RANGE(0, TMAX_COREID)$
$IF !LENGTH(OSTK.COREID[coreid])$
$	// OSTKがない場合の計算したスタック合計値の設定
$	// 非スタックコンテキストスタックサイズ = ISR共有後サイズ + MINIMUM_OSTKSZ
	$OSTK.COREID[coreid] = coreid$
	$IF ISFUNCTION("GENERATE_OSSTACK")$
		$GENERATE_OSSTACK(coreid, +total_stksz[coreid])$
	$ELSE$
		static StackType			kernel_core$coreid$_ostack[$FORMAT("COUNT_STK_T(0x%x)", +total_stksz[coreid])$];$NL$
	$END$
	#define TOPPERS_CORE$coreid$_OSTKSZ		$FORMAT("ROUND_STK_T(0x%xU)", +total_stksz[coreid])$$NL$
	#define TOPPERS_CORE$coreid$_OSTK		kernel_core$coreid$_ostack$NL$
$ELSE$
$ 	// スタックサイズのデフォルト設定
$ 	// NULL(0)か0を指定した場合，デフォルト値設定
	$IF OSTK.STKSZ[OSTK.COREID[coreid]] == 0$
		$OSTK.STKSZ[OSTK.COREID[coreid]] = DEFAULT_OSSTKSZ$
	$END$

$ 	// stkszがスタック領域のサイズとして正しくない場合
	$IF !EQ(OSTK.STK[OSTK.COREID[coreid]], "NULL") && CHECK_STKSZ_ALIGN
							&& OSTK.STKSZ[OSTK.COREID[coreid]] & (CHECK_STKSZ_ALIGN - 1)$
		$ERROR OSTK.TEXT_LINE[OSTK.COREID[coreid]]$$FORMAT(_("OsOsStackSize(%1%) is not aligned"), OSTK.STKSZ[OSTK.COREID[coreid]])$$END$
	$END$

$ 	// stkszが必要なスタックサイズより小さい場合
	$IF OSTK.STKSZ[OSTK.COREID[coreid]] < total_stksz[coreid]$
		$ERROR OSTK.TEXT_LINE[OSTK.COREID[coreid]]$$FORMAT(_("OsOsStackSize(%1%) is necessary 0x%2$x and more)"), OSTK.STKSZ[OSTK.COREID[coreid]], +total_stksz[coreid])$$END$
	$END$

	$IF EQ(OSTK.STK[OSTK.COREID[coreid]], "NULL")$
$		// スタック領域の自動割付け
		$IF ISFUNCTION("GENERATE_OSSTACK_DEF")$
			$GENERATE_OSSTACK_DEF(coreid)$
		$ELSE$
			static StackType				kernel_core$coreid$_ostack[COUNT_STK_T($OSTK.STKSZ[OSTK.COREID[coreid]]$)];$NL$
		$END$
		#define TOPPERS_CORE$coreid$_OSTKSZ		ROUND_STK_T($OSTK.STKSZ[OSTK.COREID[coreid]]$)$NL$
		#define TOPPERS_CORE$coreid$_OSTK		kernel_core$coreid$_ostack$NL$
	$ELSE$
		#define TOPPERS_CORE$coreid$_OSTKSZ		($OSTK.STKSZ[OSTK.COREID[coreid]]$)$NL$
		#define TOPPERS_CORE$coreid$_OSTK		($OSTK.STK[OSTK.COREID[coreid]]$)$NL$
	$END$
$END$
$NL$
$END$
$NL$

$ 非タスクコンテキスト用のスタック領域
const MemorySizeType	kernel_ostksz_table[TotalNumberOfCores] = {$NL$
$JOINEACH coreid RANGE(0, TMAX_COREID) ",\n"$
	$TAB$TOPPERS_CORE$coreid$_OSTKSZ
$END$
$NL$};$NL$
$NL$
StackType * const		kernel_ostk_table[TotalNumberOfCores] = {$NL$
$JOINEACH coreid RANGE(0, TMAX_COREID) ",\n"$
	$TAB$(StackType *) TOPPERS_CORE$coreid$_OSTK
$END$
$NL$};$NL$
$NL$

#ifdef TOPPERS_OSTKPT$NL$
StackType * const	kernel_ostkpt_table[TotalNumberOfCores] = {$NL$
$JOINEACH coreid RANGE(0, TMAX_COREID) ",\n"$
	$TAB$TOPPERS_OSTKPT(TOPPERS_CORE$coreid$_OSTK, TOPPERS_CORE$coreid$_OSTKSZ)
$END$
$NL$};$NL$
#endif /* TOPPERS_OSTKPT */$NL$
$NL$


$ =====================================================================
$ IOC情報の生成
$ =====================================================================
$INCLUDE "kernel/ioc.tf"$


$FILE "Os_Lcfg.h"$
#ifndef TOPPERS_MACRO_ONLY$NL$

$ タスクのextern宣言
$FOREACH tskid TSK.ID_LIST$
	extern TASK($tskid$);$NL$
$END$
$NL$

$ ハードウェアカウンタの外部参照宣言の出力
$FOREACH cntid HWCNT.ID_LIST$
	extern void init_hwcounter_$cntid$(TickType maxval, TimeType nspertick);$NL$
	extern void start_hwcounter_$cntid$(void);$NL$
	extern void stop_hwcounter_$cntid$(void);$NL$
	extern void set_hwcounter_$cntid$(TickType exprtick);$NL$
	extern TickType get_hwcounter_$cntid$(void);$NL$
	extern void cancel_hwcounter_$cntid$(void);$NL$
	extern void trigger_hwcounter_$cntid$(void);$NL$
	extern void int_clear_hwcounter_$cntid$(void);$NL$
	extern void int_cancel_hwcounter_$cntid$(void);$NL$
	extern void increment_hwcounter_$cntid$(void);$NL$
$END$
$NL$

$FOREACH almid ALM.ID_LIST$
$	// アラームコールバック関数のextern宣言
	$IF EQ(ALM.ALMATR[almid], "CALLBACK")$
		extern ALARMCALLBACK($ALM.CALLBACK[almid]$);$NL$
	$END$
$END$
$NL$

$ コア間割込みハンドラのextern宣言
$FOREACH iciid ICI.ID_LIST$
	extern ICISR($iciid$);$NL$
$END$
$NL$

$FOREACH isrid ISR.ID_LIST$
$	//割込み関数であることを示すコンパイルオプションを付ける関数が定義されている場合か
	$IF EQ(ISR.CATEGORY[isrid], "CATEGORY_1") && ISFUNCTION("EXTERN_C1ISR_HANDLER")$
		$EXTERN_C1ISR_HANDLER(ISR.INT_ENTRY[isrid])$$NL$
	$ELSE$
		extern void $ISR.INT_ENTRY[isrid]$(void);$NL$
	$END$
$END$
$NL$

$ ISR用の割込みハンドラ
$FOREACH isrid ISR2.ID_LIST$
	extern ISR($isrid$);$NL$
$END$
$NL$

$IF ISFUNCTION("EXTERN_INT_HANDLER")$
	$EXTERN_INT_HANDLER()$
$END$

#ifdef TOPPERS_ENABLE_TRACE$NL$
extern const char8 *kernel_appid_str(AppModeType id);$NL$
extern const char8 *kernel_tskid_str(TaskType id);$NL$
extern const char8 *kernel_isrid_str(ISRType id);$NL$
extern const char8 *kernel_cntid_str(CounterType id);$NL$
extern const char8 *kernel_almid_str(AlarmType id);$NL$
extern const char8 *kernel_resid_str(ResourceType id);$NL$
extern const char8 *kernel_schtblid_str(ScheduleTableType id);$NL$
extern const char8 *kernel_evtid_str(TaskType task, EventMaskType event);$NL$
extern const char8 *kernel_osapid_str(ApplicationType id);$NL$
extern const char8 *kernel_iocid_str(IocType id);$NL$
#endif /* TOPPERS_ENABLE_TRACE */$NL$
#endif /* TOPPERS_MACRO_ONLY */$NL$

#endif /* TOPPERS_OS_LCFG_H */$NL$


$FILE "Os_Lcfg.c"$
#ifdef TOPPERS_ENABLE_TRACE$NL$
const char8 *$NL$
kernel_appid_str(AppModeType id)$NL$
{$NL$
$IF LENGTH(APP.ID_LIST)$
	$TAB$const char8	*appid_str;$NL$
	$TAB$switch (id) {$NL$
	$FOREACH appid APP.ID_LIST$
		$TAB$case $appid$:$NL$
		$TAB$$TAB$appid_str = "$appid$";$NL$
		$TAB$$TAB$break;$NL$
	$END$
	$TAB$default:$NL$
	$TAB$$TAB$appid_str = "";$NL$
	$TAB$$TAB$break;$NL$
	$TAB$}$NL$
	$TAB$return(appid_str);$NL$
$ELSE$
	$TAB$return("");$NL$
$END$
}$NL$

const char8 *$NL$
kernel_tskid_str(TaskType id)$NL$
{$NL$
$TAB$const char8	*tskid_str;$NL$
$TAB$switch (id) {$NL$
$FOREACH tskid TSK.ID_LIST$
	$TAB$case $tskid$:$NL$
	$TAB$$TAB$tskid_str = "$tskid$";$NL$
	$TAB$$TAB$break;$NL$
$END$
$TAB$case INVALID_TASK:$NL$
$TAB$$TAB$tskid_str = "INVALID_TASK";$NL$
$TAB$$TAB$break;$NL$
$TAB$default:$NL$
$TAB$$TAB$tskid_str = "";$NL$
$TAB$$TAB$break;$NL$
$TAB$}$NL$
$TAB$return(tskid_str);$NL$
}$NL$

$NL$
const char8 *$NL$
kernel_isrid_str(ISRType id)$NL$
{$NL$
$TAB$const char8	*isrid_str;$NL$
$TAB$switch (id) {$NL$
$FOREACH isrid ISR.ID_LIST$
	$TAB$case $isrid$:$NL$
	$TAB$$TAB$isrid_str = "$isrid$";$NL$
	$TAB$$TAB$break;$NL$
$END$
$TAB$case INVALID_ISR:$NL$
$TAB$$TAB$isrid_str = "INVALID_ISR";$NL$
$TAB$$TAB$break;$NL$
$TAB$default:$NL$
$TAB$$TAB$isrid_str = "";$NL$
$TAB$$TAB$break;$NL$
$TAB$}$NL$
$TAB$return(isrid_str);$NL$
}$NL$

$NL$
const char8 *$NL$
kernel_cntid_str(CounterType id)$NL$
{$NL$
$IF LENGTH(CNT.ID_LIST)$
	$TAB$const char8	*cntid_str;$NL$
	$TAB$switch (id) {$NL$
	$FOREACH cntid CNT.ID_LIST$
		$TAB$case $cntid$:$NL$
		$TAB$$TAB$cntid_str = "$cntid$";$NL$
		$TAB$$TAB$break;$NL$
	$END$
	$TAB$default:$NL$
	$TAB$$TAB$cntid_str = "";$NL$
	$TAB$$TAB$break;$NL$
	$TAB$}$NL$
	$TAB$return(cntid_str);$NL$
$ELSE$
	$TAB$return("");$NL$
$END$
}$NL$

$NL$
const char8 *$NL$
kernel_almid_str(AlarmType id)$NL$
{$NL$
$IF LENGTH(ALM.ID_LIST)$
	$TAB$const char8	*almid_str;$NL$
	$TAB$switch (id) {$NL$
	$FOREACH almid ALM.ID_LIST$
		$TAB$case $almid$:$NL$
		$TAB$$TAB$almid_str = "$almid$";$NL$
		$TAB$$TAB$break;$NL$
	$END$
	$TAB$default:$NL$
	$TAB$$TAB$almid_str = "";$NL$
	$TAB$$TAB$break;$NL$
	$TAB$}$NL$
	$TAB$return(almid_str);$NL$
$ELSE$
	$TAB$return("");$NL$
$END$
}$NL$

$NL$
const char8 *$NL$
kernel_resid_str(ResourceType id)$NL$
{$NL$
$IF LENGTH(RES.ID_LIST)$
	$TAB$const char8	*resid_str;$NL$
	$TAB$switch (id) {$NL$
	$FOREACH resid RES.ID_LIST$
		$TAB$case $resid$:$NL$
		$TAB$$TAB$resid_str = "$resid$";$NL$
		$TAB$$TAB$break;$NL$
	$END$
	$TAB$default:$NL$
	$TAB$$TAB$resid_str = "";$NL$
	$TAB$$TAB$break;$NL$
	$TAB$}$NL$
	$TAB$return(resid_str);$NL$
$ELSE$
	$TAB$return("");$NL$
$END$
}$NL$

$NL$
const char8 *$NL$
kernel_schtblid_str(ScheduleTableType id)$NL$
{$NL$
$IF LENGTH(SCHTBL.ID_LIST)$
	$TAB$const char8	*schtblid_str;$NL$
	$TAB$switch (id) {$NL$
	$FOREACH schtblid SCHTBL.ID_LIST$
		$TAB$case $schtblid$:$NL$
		$TAB$$TAB$schtblid_str = "$schtblid$";$NL$
		$TAB$$TAB$break;$NL$
	$END$
	$TAB$default:$NL$
	$TAB$$TAB$schtblid_str = "";$NL$
	$TAB$$TAB$break;$NL$
	$TAB$}$NL$
	$TAB$return(schtblid_str);$NL$
$ELSE$
	$TAB$return("");$NL$
$END$
}$NL$

$NL$
const char8 *$NL$
kernel_evtid_str(TaskType task, EventMaskType event)$NL$
{$NL$
$TAB$const char8	*evtid_str;$NL$
$IF LENGTH(TSK.ID_LIST)$
	$TAB$switch (task) {$NL$
	$FOREACH tskid TSK.ID_LIST$
		$TAB$case $tskid$:$NL$
		$IF LENGTH(TSK.EVT_LIST[tskid])$
			$TAB$$TAB$switch (event) {$NL$
			$FOREACH evtid TSK.EVT_LIST[tskid]$
				$TAB$$TAB$case $evtid$:$NL$
				$TAB$$TAB$$TAB$evtid_str = "$evtid$";$NL$
				$TAB$$TAB$$TAB$break;$NL$
			$END$
			$TAB$$TAB$default:$NL$
			$TAB$$TAB$$TAB$evtid_str = NULL;$NL$
			$TAB$$TAB$$TAB$break;$NL$
			$TAB$$TAB$}$NL$
		$ELSE$
			$TAB$$TAB$evtid_str = NULL;$NL$
		$END$
		$TAB$$TAB$break;$NL$
	$END$
	$TAB$default:$NL$
	$TAB$$TAB$evtid_str = NULL;$NL$
	$TAB$$TAB$break;$NL$
	$TAB$}$NL$
$END$
$TAB$if (evtid_str == NULL) {$NL$
$FOREACH evtid EVT.ID_LIST$
	$TAB$$TAB$if (event == $evtid$) {$NL$
	$TAB$$TAB$$TAB$evtid_str = "$evtid$";$NL$
	$TAB$$TAB$}$NL$
$END$
$TAB$}$NL$
$TAB$return(evtid_str);$NL$
}$NL$

$NL$
const char8 *$NL$
kernel_osapid_str(ApplicationType id)$NL$
{$NL$
$TAB$const char8	*osapid_str;$NL$
$TAB$switch (id) {$NL$
$FOREACH osapid OSAP.ID_LIST$
	$TAB$case $osapid$:$NL$
	$TAB$$TAB$osapid_str = "$osapid$";$NL$
	$TAB$$TAB$break;$NL$
$END$
$TAB$case INVALID_OSAPPLICATION:$NL$
$TAB$$TAB$osapid_str = "INVALID_OSAPPLICATION";$NL$
$TAB$$TAB$break;$NL$
$TAB$default:$NL$
$TAB$$TAB$osapid_str = "";$NL$
$TAB$$TAB$break;$NL$
$TAB$}$NL$
$TAB$return(osapid_str);$NL$
}$NL$

$NL$
const char8 *$NL$
kernel_iocid_str(IocType id)$NL$
{$NL$
$TAB$const char8	*iocid_str;$NL$
$TAB$switch (id) {$NL$
$FOREACH iocid IOC.ID_LIST$
	$TAB$case $iocid$:$NL$
	$TAB$$TAB$iocid_str = "$iocid$";$NL$
	$TAB$$TAB$break;$NL$
$END$
$TAB$default:$NL$
$TAB$$TAB$iocid_str = "";$NL$
$TAB$$TAB$break;$NL$
$TAB$}$NL$
$TAB$return(iocid_str);$NL$
}$NL$

#endif /* TOPPERS_ENABLE_TRACE */$NL$

$FILE "cfg2_out.tf"$
$$ cfg2_out.tf$NL$
$NL$
$$TSK.ID_LIST = {
$JOINEACH tskid TSK.ID_LIST ","$
	VALUE("$tskid$", $+tskid$)
$END$
}$$$NL$

$NL$
$$ALM.ID_LIST = {
$JOINEACH almid ALM.ID_LIST ","$
	VALUE("$almid$", $+almid$)
$END$
}$$$NL$

$NL$
$$HWCNT.ID_LIST = {
$JOINEACH hwcntid HWCNT.ID_LIST ","$
	VALUE("$hwcntid$", $+hwcntid$)
$END$
}$$$NL$

$NL$
$$OSTK.ID_LIST = {
$JOINEACH coreid RANGE(0, TMAX_COREID) ","$
	VALUE("$OSTK.COREID[coreid]$", $+OSTK.COREID[coreid]$)
$END$
}$$$NL$
