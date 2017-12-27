$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2007 by TAKAGI Nobuhisa
$  Copyright (C) 2007-2015 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
$  Copyright (C) 2011-2015 by FUJI SOFT INCORPORATED, JAPAN
$  Copyright (C) 2011-2013 by Spansion LLC, USA
$  Copyright (C) 2011-2015 by NEC Communication Systems, Ltd., JAPAN
$  Copyright (C) 2011-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
$  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
$  Copyright (C) 2011-2015 by Sunny Giken Inc., JAPAN
$  Copyright (C) 2011-2015 by TOSHIBA CORPORATION, JAPAN
$  Copyright (C) 2011-2015 by Witz Corporation
$  Copyright (C) 2014-2015 by AISIN COMCRUISE Co., Ltd., JAPAN
$  Copyright (C) 2014-2015 by eSOL Co.,Ltd., JAPAN
$  Copyright (C) 2014-2015 by SCSK Corporation, JAPAN
$  Copyright (C) 2015 by SUZUKI MOTOR CORPORATION
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
$  $Id: kernel.tf 491 2015-12-17 10:50:31Z nces-okajima $
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
$FOREACH regid REG.ID_LIST$
	$REG.WRITE[regid] = IS_TRUE(REG.WRITE[regid])$
$END$
$FOREACH memid MEM.ID_LIST$
	$MEM.WRITE[memid] = IS_TRUE(MEM.WRITE[memid], MEM.TEXT_LINE[memid])$
	$MEM.READ[memid] = IS_TRUE(MEM.READ[memid], MEM.TEXT_LINE[memid])$
	$MEM.EXEC[memid] = IS_TRUE(MEM.EXEC[memid], MEM.TEXT_LINE[memid])$
	$MEM.CACHE[memid] = IS_TRUE(MEM.CACHE[memid], MEM.TEXT_LINE[memid])$
	$MEM.DEVICE[memid] = IS_TRUE(MEM.DEVICE[memid], MEM.TEXT_LINE[memid])$
$END$
$FOREACH secid SEC.ID_LIST$
	$SEC.WRITE[secid] = IS_TRUE(SEC.WRITE[secid], SEC.TEXT_LINE[secid])$
	$SEC.READ[secid] = IS_TRUE(SEC.READ[secid], SEC.TEXT_LINE[secid])$
	$SEC.EXEC[secid] = IS_TRUE(SEC.EXEC[secid], SEC.TEXT_LINE[secid])$
	$SEC.SDATA[secid] = IS_TRUE(SEC.SDATA[secid], SEC.TEXT_LINE[secid])$
	$SEC.CACHE[secid] = IS_TRUE(SEC.CACHE[secid], SEC.TEXT_LINE[secid])$
	$SEC.DEVICE[secid] = IS_TRUE(SEC.DEVICE[secid], SEC.TEXT_LINE[secid])$
$END$
$FOREACH modid MOD.ID_LIST$
	$MOD.EXPORT[modid] = IS_TRUE(MOD.EXPORT[modid], MOD.TEXT_LINE[modid])$
$END$


$ =====================================================================
$ OSAPのエラーチェックと関連づけ
$ =====================================================================

$tnum_os_restarttask = 0$
$FOREACH osapid OSAP.ID_LIST$
	$FOREACH tskid OSAP.TSK_LIST[osapid]$
		$IF LENGTH(TSK.OSAPID[tskid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsTask(%1%) belongs to another OsApplication(%2%)"), tskid, TSK.OSAPID[tskid])$$END$
		$ELSE$
			$TSK.OSAPID[tskid] = osapid$
		$END$
	$END$

	$FOREACH cntid OSAP.CNT_LIST[osapid]$
		$IF LENGTH(CNT.OSAPID[cntid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsCounter(%1%) belongs to another OsApplication(%2%)"), cntid, CNT.OSAPID[cntid])$$END$
		$ELSE$
			$CNT.OSAPID[cntid] = osapid$
		$END$
	$END$

	$FOREACH almid OSAP.ALM_LIST[osapid]$
		$IF LENGTH(ALM.OSAPID[almid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsAlarm(%1%) belongs to another OsApplication(%2%)"), almid, ALM.OSAPID[almid])$$END$
		$ELSE$
			$ALM.OSAPID[almid] = osapid$
		$END$
	$END$

	$FOREACH isrid OSAP.ISR_LIST[osapid]$
		$IF LENGTH(ISR.OSAPID[isrid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsIsr(%1%) belongs to another OsApplication(%2%)"), isrid, ISR.OSAPID[isrid])$$END$
		$ELSE$
			$ISR.OSAPID[isrid] = osapid$
		$END$
	$END$

	$FOREACH schtblid OSAP.SCHTBL_LIST[osapid]$
		$IF LENGTH(SCHTBL.OSAPID[schtblid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsScheduleTable(%1%) belongs to another OsApplication(%2%)"), schtblid, SCHTBL.OSAPID[schtblid])$$END$
		$ELSE$
			$SCHTBL.OSAPID[schtblid] = osapid$
		$END$
	$END$

	$FOREACH secid OSAP.SEC_LIST[osapid]$
		$IF LENGTH(SEC.OSAPID[secid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsMemorySection(%1%) belongs to another OsApplication(%2%)"),secid, SEC.OSAPID[secid])$$END$
		$ELSE$
			$SEC.OSAPID[secid] = osapid$
		$END$
	$END$

	$FOREACH modid OSAP.MOD_LIST[osapid]$
		$IF LENGTH(MOD.OSAPID[modid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsMemoryModule(%1%) belongs to another OsApplication(%2%)"), modid, MOD.OSAPID[modid])$$END$
		$ELSE$
			$MOD.OSAPID[modid] = osapid$
		$END$
	$END$

	$FOREACH memid OSAP.MEM_LIST[osapid]$
		$IF LENGTH(MEM.OSAPID[memid])$
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsMemoryArea(%1%) belongs to another OsApplication(%2%)"), memid, MEM.OSAPID[memid])$$END$
		$ELSE$
			$MEM.OSAPID[memid] = osapid$
		$END$
	$END$

	$IF LENGTH(OSAP.RESTARTTASK[osapid])$
		$FOREACH tskid TSK.ID_LIST$
			$IF EQ(OSAP.RESTARTTASK[osapid], tskid)$
				$IF LENGTH(TSK.OSAPID[tskid]) && !EQ(TSK.OSAPID[tskid], osapid)$
					$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsRestartTask(%1%) belongs to another OsApplication(%2%)"), tskid, TSK.OSAPID[tskid])$$END$
				$ELIF LENGTH(TSK.AUTOSTART.APP_LIST[tskid])$
					$WARNING OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsRestartTask(%1%) is set to an auto start task"), tskid)$$END$
					$OSAP.RESTARTTASK[osapid] = tskid$
					$TSK.OSAPID[tskid] = osapid$
					$TSK.RESTARTTASK[tskid] = osapid$
				$ELSE$
					$OSAP.RESTARTTASK[osapid] = tskid$
					$TSK.OSAPID[tskid] = osapid$
					$TSK.RESTARTTASK[tskid] = osapid$
				$END$
			$END$
		$END$
		$IF !LENGTH(OSAP.RESTARTTASK[osapid])$
$			// OsRestartTaskが見つからなかった
			$ERROR OSAP.TEXT_LINE[osapid]$$FORMAT(_("OsRestartTask(%1%) of OSAP(%2%) no exist"), OSAP.RESTARTTASK[osapid], osapid)$$END$
		$END$
	$ELSE$
$		//信頼OSAPの場合は，強制終了することがないため，リスタートタスクは不要
		$IF OSAP.TRUSTED[osapid]$
			$OSAP.RESTARTTASK[osapid] = VALUE("NO_RESTART_TASK", 0)$
		$ELSE$
			$OSAP.RESTARTTASK[osapid] = VALUE("OS_GEN_RESTARTTASK", LENGTH(TSK.ID_LIST)+1)$
			$TSK.ID_LIST = APPEND(TSK.ID_LIST, VALUE("OS_GEN_RESTARTTASK", +OSAP.RESTARTTASK[osapid]))$
$			//OS指定リスタートタスクのパラメータを設定する．
			$TSK.OSAPID[OSAP.RESTARTTASK[osapid]] = osapid$
			$TSK.PRIORITY[OSAP.RESTARTTASK[osapid]] = TMAX_TPRI$
			$TSK.EXTENDED[OSAP.RESTARTTASK[osapid]] = 0$
			$TSK.ACTIVATION[OSAP.RESTARTTASK[osapid]] = VALUE("1",1)$
			$TSK.SCHEDULE[OSAP.RESTARTTASK[osapid]] = "FULL"$
			$TSK.RESTARTTASK[OSAP.RESTARTTASK[osapid]] = osapid$
			$tnum_os_restarttask = tnum_os_restarttask + 1$
		$END$
	$END$
$END$

$ OSで生成したリスタートタスクの値属性のうち最小のもの
$tmin_os_restarttask = LENGTH(TSK.ID_LIST) - tnum_os_restarttask + 1$
$ OSAP.TFN.*をTFN.*にコピー
$TFN.ID_LIST = {}$
$FOREACH tfnid OSAP.TFN.ID_LIST$
	$osapid = OSAP.TFN.PARENT[tfnid]$
	$FOREACH tfnid2 TFN.ID_LIST$
		$IF EQ(tfnid, tfnid2)$
			$ERROR OSAP.TEXT_LINE[tfnid]$$FORMAT(_("duplicate OsApplicationTrustedFunction name(%1%) in OSAP (%2%) and (%3%)"), tfnid, osapid, TFN.OSAPID[tfnid2])$$END$
			$DIE()$
		$END$
	$END$
	$TFN.ID_LIST = APPEND(TFN.ID_LIST, tfnid)$
	$TFN.FUNC[tfnid] = OSAP.TFN.FUNC[tfnid]$
	$TFN.STKSZ[tfnid] = OSAP.TFN.STKSZ[tfnid]$
	$TFN.OSAPID[tfnid] = osapid$
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
$ NTHSTK.STKがない場合にNULLを入れる
$ =====================================================================

$FOREACH nthstkid NTHSTK.ID_LIST$
	$IF !LENGTH(NTHSTK.STK[nthstkid])$
		$NTHSTK.STK[nthstkid] = "NULL"$
	$END$
$END$


$ =====================================================================
$ REG.REGATRとREG.REGNAMEの作成
$ REG.REGIONとREG.ID_LISTの整形
$ =====================================================================

$new_id_list = {}$
$FOREACH regid REG.ID_LIST$
	$IF !LENGTH(REG.REGATR[regid])$
		$REG.REGATR[regid] = TA_NULL$
	$END$
	$IF !REG.WRITE[regid]$
		$REG.REGATR[regid] = REG.REGATR[regid] | TA_NOWRITE$
	$END$
	$REG.REGION[regid] = ESCSTR(REG.REGION[regid])$
	$new_id_list = APPEND(new_id_list, +regid)$
$END$
$REG.ID_LIST = new_id_list$
$REG.ORDER_LIST = new_id_list$


$ =====================================================================
$ SRG.STDROMとSRG.STDRAMを整形
$ =====================================================================

$new_id_list = {}$
$FOREACH srgid SRG.ID_LIST$
	$SRG.STDROM[srgid] = VALUE(ESCSTR(SRG.STDROM[srgid]), +srgid)$
	$SRG.STDRAM[srgid] = VALUE(ESCSTR(SRG.STDRAM[srgid]), +srgid)$
	$new_id_list = APPEND(new_id_list, +srgid)$
$END$
$SRG.ORDER_LIST = new_id_list$

$ =====================================================================
$ SECのエラーチェックとSEC.MEMATRの作成
$ =====================================================================

$FOREACH secid SEC.ID_LIST$
	$SEC.MEMATR[secid] = 0$

	$IF LENGTH(SEC.INIT[secid])$
		$IF EQ(SEC.INIT[secid], "DATA")$
			$SEC.MEMATR[secid] = TA_MEMINI$
		$ELIF EQ(SEC.INIT[secid], "BSS")$
			$SEC.MEMATR[secid] = TA_NULL$
		$ELIF EQ(SEC.INIT[secid], "NO_INITIALIZE")$
			$SEC.MEMATR[secid] = TA_MEMPRSV$
		$ELSE$
			$ERROR SEC.TEXT_LINE[secid]$$FORMAT(_("OsMemorySectionInitialize must be DATA or BSS or NO_INITIALIZE"))$$END$
		$END$
		$IF !REG.WRITE[SEC.MEMREG[secid]]$
			$ERROR SEC.TEXT_LINE[secid]$$FORMAT(_("OsMemorySectionInitialize must not be defined on nonwritable memory region"))$$END$
		$END$
	$ELSE$
		$IF REG.WRITE[SEC.MEMREG[secid]]$
			$ERROR SEC.TEXT_LINE[secid]$$FORMAT(_("OsMemorySectionInitialize must be defined on writable memory region"))$$END$
		$END$
	$END$

	$IF !SEC.WRITE[secid]$
		$SEC.MEMATR[secid] = SEC.MEMATR[secid] | TA_NOWRITE$
	$END$
	$IF !SEC.READ[secid]$
		$SEC.MEMATR[secid] = SEC.MEMATR[secid] | TA_NOREAD$
	$END$
	$IF SEC.EXEC[secid]$
		$SEC.MEMATR[secid] = SEC.MEMATR[secid] | TA_EXEC$
	$END$
	$IF SEC.SDATA[secid]$
		$SEC.MEMATR[secid] = SEC.MEMATR[secid] | TA_SDATA$
	$END$
	$IF !SEC.CACHE[secid]$
		$SEC.MEMATR[secid] = SEC.MEMATR[secid] | TA_UNCACHE$
	$END$
	$IF SEC.DEVICE[secid]$
		$SEC.MEMATR[secid] = SEC.MEMATR[secid] | TA_IODEV$
	$END$
$END$


$ =====================================================================
$ MEM.MEMATRの作成
$ =====================================================================

$FOREACH memid MEM.ID_LIST$
	$MEM.MEMATR[memid] = TA_MEMPRSV$
	$IF !MEM.WRITE[memid]$
		$MEM.MEMATR[memid] = MEM.MEMATR[memid] | TA_NOWRITE$
	$END$
	$IF !MEM.READ[memid]$
		$MEM.MEMATR[memid] = MEM.MEMATR[memid] | TA_NOREAD$
	$END$
	$IF MEM.EXEC[memid]$
		$MEM.MEMATR[memid] = MEM.MEMATR[memid] | TA_EXEC$
	$END$
	$IF !MEM.CACHE[memid]$
		$MEM.MEMATR[memid] = MEM.MEMATR[memid] | TA_UNCACHE$
	$END$
	$IF MEM.DEVICE[memid]$
		$MEM.MEMATR[memid] = MEM.MEMATR[memid] | TA_IODEV$
	$END$
$END$


$ =====================================================================
$ LNKをSECに統合する
$ =====================================================================

$IF LENGTH(SEC.ID_LIST)$
	$i = AT(SEC.ID_LIST, LENGTH(SEC.ID_LIST) - 1) + 1$
$ELSE$
	$i = 1$
$END$
$FOREACH lnkid LNK.ID_LIST$
	$SEC.SECTION[i] = LNK.SECTION[lnkid]$
	$SEC.MEMREG[i] = LNK.MEMREG[lnkid]$
	$SEC.ID_LIST = APPEND(SEC.ID_LIST, VALUE(lnkid, i))$
	$SEC.ORDER_LIST = APPEND(SEC.ORDER_LIST, VALUE(lnkid, i))$
	$i = i + 1$
$END$

$ MEMREGはREFになっているがkernel.tfでは文字列として扱う
$FOREACH secid SEC.ID_LIST$
	$SEC.MEMREG[secid] = REG.REGION[SEC.MEMREG[secid]]$
	$SEC.SECTION[secid] = ESCSTR(SEC.SECTION[secid])$
$END$


$ =====================================================================
$ MOD.MODULEの作成
$ =====================================================================

$nummod = LENGTH(MOD.ORDER_LIST)$
$FOREACH modid MOD.ID_LIST$
	$FOREACH name MOD.MODULE[modid]$
		$IF EQ(name, AT(MOD.MODULE[modid], 0))$
			$MOD.MODULE[modid] = ESCSTR(name)$
		$ELSE$
			$nummod = nummod + 1$
			$MOD.ORDER_LIST = APPEND(nummod, MOD.ORDER_LIST)$
			$MOD.MODULE[nummod] = ESCSTR(name)$
			$MOD.OSAPID[nummod] = MOD.OSAPID[modid]$
			$MOD.TEXT_LINE[nummod] = MOD.TEXT_LINE[modid]$
			$MOD.EXPORT[nummod] = MOD.EXPORT[modid]$
		$END$
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
	$IF LENGTH(TSK.OSAPID[tskid])$
		$IF OSAP.TRUSTED[TSK.OSAPID[tskid]]$
$			// 信頼タスクはOsTaskSystemStackStartAddressの使用不可
			$IF LENGTH(TSK.SSTK[tskid])$
				$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("Trusted OsTask(%1%) can't use OsTaskSystemStackStartAddress"), tskid)$$END$
			$END$
		$ELSE$
$			// 非信頼タスクでシステムスタックの先頭番地が指定されてサイズが指定されない
			$IF LENGTH(TSK.SSTK[tskid]) && !LENGTH(TSK.SSTKSZ[tskid])$
				$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskSystemStackSize of Non-Trusted OsTask(%1%) is necessary when OsTaskSystemStackStartAddress is used"), tskid)$$END$
			$END$
		$END$
	$ELSE$
		$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTask(%1%) does not belong OsApplication"), tskid)$$END$
		$DIE()$
	$END$
$END$


$ =====================================================================
$ ISRのエラーチェックと関連づけ
$ =====================================================================

$TRACE("ASSOCIATE ISR")$

$FOREACH isrid ISR.ID_LIST$
	$IF !(EQ(ISR.SOURCE[isrid], "ENABLE") || EQ(ISR.SOURCE[isrid], "DISABLE"))$
		$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("OsIsrInterruptSource must be ENABLE or DISABLE"))$$END$
	$END$

$	// C1ISRはENABLEにする必要がある
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

$ ALM.ACTION.INCREMENTCOUNTER.*からALM.*にコピー
$FOREACH almactid ALM.ACTION.INCREMENTCOUNTER.ID_LIST$
	$almid                      = ALM.ACTION.PARENT[ALM.ACTION.INCREMENTCOUNTER.PARENT[almactid]]$
	$ALM.ALMATR[almid]          = "INCREMENTCOUNTER"$
	$ALM.INCID[almid]           = ALM.ACTION.INCREMENTCOUNTER.INCID[almactid]$
	$ALM.ALMATR_COUNT[almid]    = ALM.ALMATR_COUNT[almid] + 1$
$END$

$IF LENGTH(ALM.ACTION.CALLBACK.ID_LIST)$
	$ERROR ALM.ACTION.CALLBACK.TEXT_LINE[1]$$FORMAT(_("OsAlarmCallback can't be used in SC3"))$$END$
$END$

$ ALM.ACTION.*を2つ以上定義していないか
$FOREACH almid ALM.ID_LIST$
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
	$EXPPTACT.OFFSET[i]             = SCHTBL.EXPPT.OFFSET[expid]$
	$EXPPTACT.TSKID[i]              = SCHTBL.EXPPT.ACTIVATETASK.TSKID[acttskid]$
	$EXPPTACT.EXPIREATR[i]          = "ACTIVATETASK"$

	$EXPPTACT.ID_LIST               = APPEND(EXPPTACT.ID_LIST, i)$
	$i                              = i + 1$
$END$

$ SCHTBL.EXPPT.SETEVENT.*からEXPPTACTにコピー
$FOREACH setevid SCHTBL.EXPPT.SETEVENT.ID_LIST$
	$expid                          = SCHTBL.EXPPT.SETEVENT.PARENT[setevid]$
	$schtblid                       = SCHTBL.EXPPT.PARENT[expid]$

	$EXPPTACT.SCHTBLID[i]           = schtblid$
	$EXPPTACT.OFFSET[i]             = SCHTBL.EXPPT.OFFSET[expid]$
	$EXPPTACT.TSKID[i]              = SCHTBL.EXPPT.SETEVENT.TSKID[setevid]$
	$EXPPTACT.EVTID[i]              = SCHTBL.EXPPT.SETEVENT.EVTID[setevid]$
	$EXPPTACT.EXPIREATR[i]          = "SETEVENT"$

	$EXPPTACT.ID_LIST               = APPEND(EXPPTACT.ID_LIST, i)$
	$i                              = i + 1$
$END$

$IF FATAL$
	$DIE()$
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

$INCLUDE "kernel/kernel_common.tf"$

$ =====================================================================
$ OSのエラーチェック
$ =====================================================================

$IF !EQ(OS.STATUS[1], "EXTENDED")$
	$ERROR OS.TEXT_LINE[1]$$FORMAT(_("OsStatus must be EXTENDED"))$$END$
$END$

$IF LENGTH(OS.SC[1]) && !EQ(OS.SC[1], "SC3")$
	$ERROR OS.TEXT_LINE[1]$$FORMAT(_("OsScalabilityClass must be SC3"))$$END$
$END$

$ =====================================================================
$ OSAPのエラーチェックと内部データの生成
$ =====================================================================

$i = 0$
$ntosap = 0$
$FOREACH osapid OSAP.ID_LIST$
	$OSAP.ID[osapid] = i$
	$i = i + 1$
	$IF OSAP.TRUSTED[osapid]$
		$OSAP.BTMP[osapid] = 0$
		$OSAP.ACPTN[osapid] = VALUE("TACP_KERNEL", TACP_KERNEL)$
	$ELSE$
		$OSAP.BTMP[osapid] = 1 << ntosap$
		$OSAP.ACPTN[osapid] = 1 << ntosap$
		$ntosap = ntosap + 1$
	$END$
$END$

$ 非信頼OSAP数はTMAX_NTOSAPP以下か？
$IF ntosap > TMAX_NTOSAPP$
	$ERROR$$FORMAT(_("Too many Non-Trusted OsApplication %1% > %2%)"), ntosap, TMAX_NTOSAPP)$$END$
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

	$IF LENGTH(ISR.OSAPID[isrid])$
$		// ISRを 定義した OSアプリケーションは信頼OSアプリケーションか?
$		// 機能レベル2ではISRは非信頼に所属できない
		$IF !OSAP.TRUSTED[ISR.OSAPID[isrid]]$
			$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("OsAppIsrRef(%1%) of OsApplication(%2%) can't be assign to Non-Trusted OS-Application"), 
											isrid, ISR.OSAPID[isrid])$$END$
		$END$

$		// ISRへのアクセス権の設定
		$ISR.ACSBTMP[isrid] = OSAP.BTMP[ISR.OSAPID[isrid]]$
		$FOREACH osapid ISR.ACS_OSAP_LIST[isrid]$
			$ISR.ACSBTMP[isrid] = ISR.ACSBTMP[isrid] | OSAP.BTMP[osapid]$
		$END$
	$ELSE$
		$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("OsIsr(%1%) does not belong OsApplication"), isrid)$$END$
		$DIE()$
	$END$
$END$


$ C2ISRの最低優先度
$MAX_PRI_ISR2 = -1$
$ C2ISRの最高優先度
$MIN_PRI_ISR2 = MAX_PRI_ISR2$

$i = 0$
$FOREACH isrid ISR.ID_LIST$
	$IF EQ(ISR.CATEGORY[isrid], "CATEGORY_2")$
		$ISR.ID[isrid] = i$
		$i = i + 1$
		$ISR2.ID_LIST = APPEND(ISR2.ID_LIST, isrid)$
		$IF MIN_PRI_ISR2 > -ISR.INTPRI[isrid]$
			$MIN_PRI_ISR2 = -ISR.INTPRI[isrid]$
		$END$
		$ISR.INT_ENTRY[isrid] = CONCAT("_kernel_inthdr_", +ISR.INTNO[isrid])$

$		// INTNOが割込み番号として正しくない場合
		$IF !LENGTH(FIND(INTNO_CREISR2_VALID, ISR.INTNO[isrid]))$
			$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("illegal OsIsrInterruptNumber(%1%) of OsIsr(%2%)"), ISR.INTNO[isrid], isrid)$$END$
		$END$
	$END$
$END$

$ C2ISRが定義されていない場合
$IF !LENGTH(ISR2.ID_LIST)$
	$MAX_PRI_ISR2 = 0$
	$MIN_PRI_ISR2 = 0$
$END$


$ C1ISRの最低優先度
$MAX_PRI_ISR1 = MIN_PRI_ISR2 - 1$
$ C1ISRの最高優先度
$MIN_PRI_ISR1 = -TNUM_INTPRI$

$FOREACH isrid ISR.ID_LIST$
	$IF EQ(ISR.CATEGORY[isrid], "CATEGORY_1")$
		$ISR1.ID_LIST = APPEND(ISR1.ID_LIST, isrid)$
		$ISR.ID[isrid] = i$
		$i = i + 1$

$		// C1ISRの割込み優先度がMIN_PRI_ISR2以上である場合
		$IF -ISR.INTPRI[isrid] >= MIN_PRI_ISR2$
			$ERROR ISR.TEXT_LINE[isrid]$$FORMAT(_("OsIsrInterruptPriority(%1%) of OsIsr(%2%) is lower than or equal to C2ISR min priority(%3%)"), ISR.INTPRI[isrid], isrid, -MIN_PRI_ISR2)$$END$
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
$	// ISRは内部リソースを取得できない
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
	$RES.RELATION_FLAG[resid] = (LENGTH(RES.TSK_LIST[resid]) || LENGTH(RES.ISR_LIST[resid]))$
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

$		// LINKEDRESOURCE以外のパラメータ（ACS_OSAP_LIST）指定しているか.
		$IF LENGTH(RES.ACS_OSAP_LIST[resid])$
			$ERROR RES.TEXT_LINE[resid]$$FORMAT(_("OsResourceAccessingApplication of LINKED OsResource(%1%) must not define"), resid)$$END$
		$END$
	$END$

	$IF !RES.RELATION_FLAG[resid]$
		$WARNING RES.TEXT_LINE[resid]$$FORMAT(_("OsResource(%1%) is not related to any TASK or ISR"), resid)$$END$
	$END$

$	// リソースへのアクセス権の設定
	$RES.ACSBTMP[resid] = 0$
	$FOREACH osapid RES.ACS_OSAP_LIST[resid]$
		$RES.ACSBTMP[resid] = RES.ACSBTMP[resid] | OSAP.BTMP[osapid]$
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
	$END$
$END$


$FOREACH resid RES.ID_LIST$
$	// リンクリソースの上限優先度とアクセス権の設定
	$IF EQ(RES.RESATR[resid], "LINKED")$
		$RES.CEILPRI[resid] = RES.CEILPRI[RES.LINKEDRESID[resid]]$
		$RES.ACSBTMP[resid] = RES.ACSBTMP[RES.LINKEDRESID[resid]]$
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

			$osapid = TSK.OSAPID[tskid]$
			$IF !OSAP.TRUSTED[osapid]$
				$IF !(RES.ACSBTMP[resid] & OSAP.BTMP[osapid])$
					$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskResourceRef of OsTask(%1%) can't access INTERNAL OsResource(%2%)"), tskid, resid)$$END$
				$END$
			$END$
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

$	// タスクへのアクセス権の設定
	$TSK.ACSBTMP[tskid] = OSAP.BTMP[TSK.OSAPID[tskid]]$
	$FOREACH osapid TSK.ACS_OSAP_LIST[tskid]$
		$TSK.ACSBTMP[tskid] = TSK.ACSBTMP[tskid] | OSAP.BTMP[osapid]$
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

$	// カウンタへのアクセス権の設定
	$IF LENGTH(CNT.OSAPID[cntid])$
		$CNT.ACSBTMP[cntid] = OSAP.BTMP[CNT.OSAPID[cntid]]$
		$FOREACH osapid CNT.ACS_OSAP_LIST[cntid]$
			$CNT.ACSBTMP[cntid] = CNT.ACSBTMP[cntid] | OSAP.BTMP[osapid]$
		$END$
	$ELSE$
		$ERROR CNT.TEXT_LINE[cntid]$$FORMAT(_("OsCounter(%1%) does not belong OsApplication"), cntid)$$END$
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

$	// アラームへのアクセス権の設定
	$IF LENGTH(ALM.OSAPID[almid])$
		$ALM.ACSBTMP[almid] = OSAP.BTMP[ALM.OSAPID[almid]]$
		$FOREACH osapid ALM.ACS_OSAP_LIST[almid]$
			$ALM.ACSBTMP[almid] = ALM.ACSBTMP[almid] | OSAP.BTMP[osapid]$
		$END$
	$ELSE$
		$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarm(%1%) does not belong OsApplication"), almid)$$END$
		$DIE()$
	$END$

	$osapid = ALM.OSAPID[almid]$

	$IF EQ(ALM.ALMATR[almid], "ACTIVATETASK") || EQ(ALM.ALMATR[almid], "SETEVENT")$
		$IF !OSAP.TRUSTED[osapid]$
			$IF !(TSK.ACSBTMP[ALM.TSKID[almid]] & OSAP.BTMP[osapid])$
				$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmActivateTaskRef or OsAlarmSetEventTaskRef of OsAlarm(%1%) can't access OsTask(%2%)"), 
												almid, ALM.TSKID[almid])$$END$
			$END$
		$END$
	$END$

$	// taskが拡張タスクか(SETEVENTの場合)
	$IF EQ(ALM.ALMATR[almid], "SETEVENT")$
		$IF !TSK.EXTENDED[ALM.TSKID[almid]]$
			$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmSetEventTaskRef(%1%) of OsAlarm(%2%) is not extended task"), ALM.TSKID[almid], almid)$$END$
		$END$
	$END$

	$IF EQ(ALM.ALMATR[almid], "INCREMENTCOUNTER")$
		$IF !OSAP.TRUSTED[osapid]$
			$IF !(CNT.ACSBTMP[ALM.INCID[almid]] & OSAP.BTMP[osapid])$
				$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmIncrementCounterRef of OsAlarm(%1%) can't access OsCounter(%2%)"), 
												almid, ALM.INCID[almid])$$END$
			$END$
		$END$

$		// アラームからハードウェアカウンタをインクリメントしているかチェック
		$IF EQ(CNT.CNTATR[ALM.INCID[almid]], "HARDWARE")$
			$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmIncrementCounterRef(%1%) of OsAlarm(%2%) is hardware counter"), ALM.INCID[almid], almid)$$END$
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

$ 他の非信頼OSAPに所属するカウンタを指定した場合
$FOREACH almid ALM.ID_LIST$
	$IF !EQ(ALM.OSAPID[almid], CNT.OSAPID[ALM.CNTID[almid]])$
		$IF !OSAP.TRUSTED[CNT.OSAPID[ALM.CNTID[almid]]]$
			$ERROR ALM.TEXT_LINE[almid]$$FORMAT(_("OsAlarmCounterRef(%1%) is on a other Non-Trusted OS-Application"), ALM.CNTID[almid])$$END$
		$END$
	$END$
$END$


$ =====================================================================
$ スケジュールテーブル関連のエラーチェックと内部データの生成
$ =====================================================================

$TRACE("SCHEDULETABLE CHECK")$

$FOREACH schtblid SCHTBL.ID_LIST$
$	// スケジュールテーブルへのアクセス権の設定
	$IF LENGTH(SCHTBL.OSAPID[schtblid])$
		$SCHTBL.ACSBTMP[schtblid] = OSAP.BTMP[SCHTBL.OSAPID[schtblid]]$
		$FOREACH osapid SCHTBL.ACS_OSAP_LIST[schtblid]$
			$SCHTBL.ACSBTMP[schtblid] = SCHTBL.ACSBTMP[schtblid] | OSAP.BTMP[osapid]$
		$END$
	$ELSE$
		$ERROR SCHTBL.TEXT_LINE[schtblid]$$FORMAT(_("OsScheduleTable(%1%) does not belong OsApplication"), schtblid)$$END$
		$DIE()$
	$END$
$	// 同期方法設定値チェック
$	// syncstrategyにNONE/IMPLICITいずれか一つが指定されているか
	$IF !(EQ(SCHTBL.SYNCSTRATEGY[schtblid], "NONE") || EQ(SCHTBL.SYNCSTRATEGY[schtblid], "IMPLICIT"))$
		$ERROR SCHTBL.TEXT_LINE[schtblid]$
				$FORMAT(_("OsScheduleTblSyncStrategy of OsScheduleTable(%1%) must be NONE or IMPLICIT"), schtblid)$$END$
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

	$osapid = SCHTBL.OSAPID[schtblid]$
	$IF !OSAP.TRUSTED[osapid]$
		$IF !(TSK.ACSBTMP[EXPPTACT.TSKID[expptactid]] & OSAP.BTMP[osapid])$
			$ERROR SCHTBL.TEXT_LINE[schtblid]$$FORMAT(_("OsScheduleTableActivateTaskRef or OsScheduleTableSetEventTaskRef of OsScheduleTable(%1%) can't access OsTask(%2%)"), 
												schtblid, EXPPTACT.TSKID[expptactid])$$END$
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

$ 他の非信頼OSAPに所属するカウンタを指定した場合
$FOREACH schtblid SCHTBL.ID_LIST$
	$IF !EQ(SCHTBL.OSAPID[schtblid], CNT.OSAPID[SCHTBL.CNTID[schtblid]])$
		$IF !OSAP.TRUSTED[CNT.OSAPID[SCHTBL.CNTID[schtblid]]]$
			$ERROR SCHTBL.TEXT_LINE[schtblid]$$FORMAT(_("OsScheduleTableCounterRef(%1%) is on a other Non-Trusted OS-Application"), SCHTBL.CNTID[schtblid])$$END$
		$END$
	$END$
$END$


$ =====================================================================
$ 信頼関数のエラーチェックとID設定
$ =====================================================================

$i = 0$
$FOREACH tfnid TFN.ID_LIST$
	$TFN.ID[tfnid] = i$
	$i = i + 1$

$	// OSアプリケーションは信頼OSアプリケーションか?
	$IF !OSAP.TRUSTED[TFN.OSAPID[tfnid]]$
		$ERROR TFN.TEXT_LINE[tfnid]$$FORMAT(_("OsApplicationTrustedFunction(%1%) of OsApplication(%2%) can't be assign to Non-Trusted OS-Application"), 
										tfnid, TFN.OSAPID[tfnid])$$END$
	$END$

	$IF EQ(TFN.FUNC[tfnid], REGEX_REPLACE(TFN.FUNC[tfnid], "^TRUSTED_", ""))$
		$ERROR TFN.TEXT_LINE[tfnid]$$FORMAT(_("OsTrustedFunctionName(%1%) of OsApplication(%2%) function name must begin with TRUSTED_"), 
										TFN.FUNC[tfnid], TFN.OSAPID[tfnid])$$END$
	$END$
$END$


$ =====================================================================
$ 以下はHRP2からの移植
$ =====================================================================

$ =====================================================================
$ 保護ドメインに関する前処理
$ =====================================================================

$
$  保護ドメインリストの作成
$	DOMLIST：ユーザドメインのリスト
$	DOMLIST_ALL：カーネルドメイン，ユーザドメイン，無所属のリスト
$
$FOREACH domid OSAP.ID_LIST$
	$IF !OSAP.TRUSTED[domid]$
		$DOMLIST = APPEND(DOMLIST, domid)$
	$END$
$END$
$DOMLIST_ALL = APPEND(TDOM_KERNEL, DOMLIST, TDOM_NONE)$

$
$  保護ドメイン毎のデフォルトのアクセス許可パターンの作成
$
$DEFAULT_ACPTN[TDOM_KERNEL] = VALUE("TACP_KERNEL", TACP_KERNEL)$
$FOREACH domid OSAP.ID_LIST$
	$DEFAULT_ACPTN[domid] = OSAP.ACPTN[domid]$
$END$
$DEFAULT_ACPTN[TDOM_NONE] = VALUE("TACP_SHARED", TACP_SHARED)$

$
$  保護ドメインのラベルの作成
$	OSAP.LABEL[domid]：保護ドメインのラベル
$
$OSAP.LABEL[TDOM_KERNEL] = "kernel"$
$FOREACH domid OSAP.ID_LIST$
	$IF !OSAP.TRUSTED[domid]$
		$OSAP.LABEL[domid] = domid$
	$END$
$END$
$OSAP.LABEL[TDOM_NONE] = "shared"$

$ =====================================================================
$ メモリオブジェクトに関する前処理
$
$ 統合前のメモリオブジェクトの情報をMO.XXXX[moid]に生成する．
$
$ nummo：統合前のメモリオブジェクトの数
$ MO.TYPE[moid]：メモリオブジェクトのタイプ
$	TOPPERS_ATTMOD：OsMemoryModuleで登録されたセクション
$					モジュール名をMO.MODULE[moid]に設定
$	TOPPERS_ATTSEC：OsMemorySectionで登録されたセクション
$	TOPPERS_ATTMEM：OsMemoryAreaで登録されたセクション
$					先頭番地をMO.BASE[moid]に設定
$					サイズをMO.SIZE[moid]に設定
$	TOPPERS_USTACK：タスクのユーザスタック領域
$					タスクIDをMO.STKORDER[moid]に設定
$					先頭番地をMO.BASE[moid]に設定（ユーザスタック領域をア
$					プリケーションが指定した場合のみ）
$					サイズをMO.SIZE[moid]に設定
$ MO.LINKER[moid]：リンカが配置するメモリオブジェクトか？
$ MO.OSAPID[moid]：属するOSAP（無所属の場合はTDOM_NONE）
$ MO.MEMREG[moid]：メモリリージョン番号（リンカが配置する場合のみ）
$ MO.SECTION[moid]：セクション名（リンカが配置する場合のみ）
$ MO.MEMATR[moid]：メモリオブジェクト属性
$ MO.ACPTN1[moid]：通常操作1（書込み）のアクセス許可パターン
$ MO.ACPTN2[moid]：通常操作2（読出し，実行）のアクセス許可パターン
$ MO.TEXT_LINE[moid]：メモリオブジェクトを登録したコンフィギュレーションフィアルの行番号
$ MO_USTACK_LIST：ジェネレータが割り付けるユーザスタック領域のリスト
$ MO_USTACK_LIST2：先頭番地を指定されたユーザスタック領域のリスト
$ =====================================================================

$TOPPERS_ATTMOD = VALUE("TOPPERS_ATTMOD", TOPPERS_ATTSEC + 1)$
$nummo = 0$
$MO_USTACK_LIST = {}$
$MO_USTACK_LIST2 = {}$ 

$
$  配置するセクションに関する前処理
$
$ OsLinkSectionで配置するセクション（メモリオブジェクトとして登録しない）の情
$ 報を，LNKSEC[lsid]に生成する．
$
$ numls：配置するセクションの数
$ LNKSEC.MEMREG[lsid]：メモリリージョン番号
$ LNKSEC.SECTION[lsid]：セクション名

$numls = 0$

$
$  OsMemoryRegionで登録されたリージョンに関するエラーチェックと前処理
$
$ REG_LIST：処理済みのメモリリージョンのリスト
$ REG.REGNAME[reg]：メモリリージョン名（＝UNESCSTR(MO.REGION[reg])）
$
$FOREACH reg REG.ORDER_LIST$
$	// REG.REGNAME[reg]の作成
	$REG.REGNAME[reg] = UNESCSTR(REG.REGION[reg])$

$	// メモリリージョン名が登録済みの場合
	$FOREACH reg2 REG_LIST$
		$IF EQ(REG.REGNAME[reg], REG.REGNAME[reg2])$
			$ERROR REG.TEXT_LINE[reg]$$FORMAT(_("OsMemoryRegionName(%1%) of OsMemoryRegion(%2%) is duplicated"), REG.REGNAME[reg], reg)$$END$
		$END$
	$END$

$	// regatrが（［TA_NOWRITE］）でない場合
	$IF (REG.REGATR[reg] & ~(TA_NOWRITE|TARGET_REGATR)) != 0$
		$ERROR REG.TEXT_LINE[reg]$$FORMAT(_("illegal attribute(%1%) of OsMemoryRegion(%2%)"), REG.REGATR[reg], reg)$$END$
	$END$

$	// ドメインに所属している場合
	$IF LENGTH(REG.DOMAIN[reg])$
		$ERROR REG.TEXT_LINE[reg]$$FORMAT(_("OsMemoryRegion(%1%) must be outside of OsApplication"), REG.REGNAME[reg])$$END$
	$END$

$	// sizeが0の場合
	$IF REG.SIZE[reg] == 0$
		$ERROR REG.TEXT_LINE[reg]$$FORMAT(_("OsMemoryRegionSize of OsMemoryRegion(%1%) must not be zero"), reg)$$END$
	$END$

$	// base+sizeが最大アドレスを越える場合
	$limit = (REG.BASE[reg] + REG.SIZE[reg]) & ((1 << sizeof_void_ptr * 8) - 1)$
	$IF limit < REG.BASE[reg] && limit != 0$
		$ERROR REG.TEXT_LINE[reg]$$FORMAT(_("OsMemoryRegionSize(%1%) of OsMemoryRegion(%2%) is too large"), REG.SIZE[reg], reg)$$END$
	$END$

$	// 登録済みのメモリリージョンと領域が重なる場合
	$FOREACH reg2 REG_LIST$
		$IF ((REG.BASE[reg] <= REG.BASE[reg2]
						&& REG.BASE[reg] + REG.SIZE[reg] > REG.BASE[reg2])
		  || (REG.BASE[reg2] < REG.BASE[reg]
						&& REG.BASE[reg2] + REG.SIZE[reg2] > REG.BASE[reg]))$
			$ERROR REG.TEXT_LINE[reg]$
					$FORMAT(_("OsMemoryRegionName(%1%) of OsMemoryRegion(%2%) overlaps with another OsMemoryRegionName(%3%) of OsMemoryRegion(%4%)"), 
						REG.REGNAME[reg], reg, REG.REGNAME[reg2], reg2)$$END$
		$END$
	$END$

$	// ターゲット依存のエラーチェック NOS0804
	$IF ISFUNCTION("HOOK_ERRORCHECK_REG")$
		$HOOK_ERRORCHECK_REG(reg)$
	$END$
	$REG_LIST = APPEND(REG_LIST, reg)$
$END$

$ REG_ORDERの生成
$REG_ORDER = SORT(REG.ORDER_LIST, "REG.BASE")$

$
$  OsStandardMemoryRegionで定義された標準メモリリージョンに関するエラーチェックと前処理
$
$ STANDARD_ROM：標準ROMリージョンのメモリリージョン番号
$ STANDARD_RAM：標準RAMリージョンのメモリリージョン番号
$
$IF !LENGTH(SRG.ORDER_LIST)$
$	//OsStandardMemoryRegionがない場合は，ここで処理を止める（以降のエラーの抑止）
	$ERROR$$FORMAT(_("no standard memory region is registered"))$$END$
	$DIE()$
$ELSE$
$	// ドメインの囲みの中に記述されている場合 
	$IF LENGTH(SRG.DOMAIN[1])$
		$ERROR SEG.TEXT_LINE[1]$$FORMAT(_("OsStandardMemoryRegion must be outside of OsApplication"))$$END$
	$END$

$	// OsStandardMemoryRegionが複数ある
	$IF LENGTH(SRG.ORDER_LIST) > 1$
		$ERROR$$FORMAT(_("too many OsStandardMemoryRegion"))$$END$
	$END$

$	// stdromが登録されているかのチェック
	$STANDARD_ROM = 0$
	$FOREACH reg REG.ORDER_LIST$
		$IF EQ(UNESCSTR(SRG.STDROM[1]), REG.REGNAME[reg])$
			$STANDARD_ROM = reg$
		$END$
	$END$
	$IF STANDARD_ROM == 0$
		$ERROR SRG.TEXT_LINE[1]$$FORMAT(_("illegal region name(%1%) in OsStandardMemoryRegion"), UNESCSTR(SRG.STDROM[1]))$
		$END$
	$END$

$	// stdromがTA_NOWRITE属性かのチェック
	$IF (REG.REGATR[STANDARD_ROM] & TA_NOWRITE) == 0$
		$ERROR SRG.TEXT_LINE[1]$$FORMAT(_("standard ROM region must have TA_NOWRITE attribute"))$$END$
	$END$

$	// stdramが登録されているかのチェック
	$STANDARD_RAM = 0$
	$FOREACH reg REG.ORDER_LIST$
		$IF EQ(UNESCSTR(SRG.STDRAM[1]), REG.REGNAME[reg])$
			$STANDARD_RAM = reg$
		$END$
	$END$
	$IF STANDARD_RAM == 0$
		$ERROR SRG.TEXT_LINE[1]$$FORMAT(_("illegal region name(%1%) in OsStandardMemoryRegion"), UNESCSTR(SRG.STDRAM[1]))$
		$END$
	$END$

$	// stdramがTA_NOWRITE属性でないかのチェック
	$IF (REG.REGATR[STANDARD_RAM] & TA_NOWRITE) != 0$
		$ERROR SRG.TEXT_LINE[1]$$FORMAT(_("standard RAM region must not have TA_NOWRITE attribute"))$$END$
	$END$

$	// どちらかがエラーの場合は，ここで処理を止める（以降のエラーの抑止）
	$IF STANDARD_ROM==0 || STANDARD_RAM==0$
		$DIE()$
	$END$
$END$

$
$  標準のセクションの定義と標準セクションのリストの作成
$
$ DSEC_SECTION_LIST：標準のセクションと保護ドメイン毎標準セクションのリスト
$
$FOREACH dsec DSEC.ORDER_LIST$
	$DSEC_SECTION_LIST = APPEND(DSEC_SECTION_LIST, DSEC.SECTION[dsec])$
$END$
$FOREACH domid DOMLIST_ALL$
	$FOREACH dsec DSEC.ORDER_LIST$
		$DSEC_SECTION_LIST = APPEND(DSEC_SECTION_LIST,
					FORMAT("%s_%s", DSEC.SECTION[dsec], OSAP.LABEL[domid]))$
	$END$
$END$

$
$  OsMemoryModuleで登録したのと同等に扱うモジュールの処理
$
$ 先頭でOsMemoryModuleで登録したのと同等：START_OBJS, libkernel.o, kernel_mem.o, libkernel.a
$ 末尾でOsMemoryModuleで登録したのと同等：*, END_OBJS
$
$IF TOPPERS_SUPPORT_ATT_MOD$
	$nummod = LENGTH(MOD.ORDER_LIST)$

	$FUNCTION ATT_MOD_FIRST$
		$nummod = nummod + 1$
		$MOD.ORDER_LIST = APPEND(nummod, MOD.ORDER_LIST)$
		$MOD.MODULE[nummod] = ESCSTR(ARGV[1])$
		$MOD.OSAPID[nummod] = ARGV[2]$
		$MOD.TEXT_LINE[nummod] = 0$
		$MOD.EXPORT[nummod] = 0$
	$END$

	$FUNCTION ATT_MOD_LAST$
		$nummod = nummod + 1$
		$MOD.ORDER_LIST = APPEND(MOD.ORDER_LIST, nummod)$
		$MOD.MODULE[nummod] = ESCSTR(ARGV[1])$
		$MOD.OSAPID[nummod] = ARGV[2]$
		$MOD.TEXT_LINE[nummod] = 0$
		$MOD.EXPORT[nummod] = 0$
	$END$

	$FOREACH module APPEND("libkernel.a", "kernel_mem.o", "Os_Lcfg.o",
															START_OBJS)$
		$ATT_MOD_FIRST(module, TDOM_KERNEL)$
	$END$

	$ATT_MOD_LAST("*", TDOM_NONE)$
	$FOREACH module END_OBJS$
		$ATT_MOD_LAST(module, TDOM_KERNEL)$
	$END$
$END$

$
$  OsMemoryModuleで登録されたモジュールに関する情報の生成
$
$ MOD_LIST：処理済みのモジュールのリスト
$
$FOREACH mod MOD.ORDER_LIST$
$	// OsMemoryModuleがサポートされていない場合
	$IF !TOPPERS_SUPPORT_ATT_MOD$
		$ERROR MOD.TEXT_LINE[mod]$$FORMAT(_("OsMemoryModule is not supported on this target"))$$END$
	$END$

$	// moduleが登録済みの場合
	$FOREACH mod2 MOD_LIST$
		$IF EQ(MOD.MODULE[mod], MOD.MODULE[mod2])$
			$ERROR MOD.TEXT_LINE[mod]$$FORMAT(_("OsMemoryModuleName(%1%) of OsMemoryModule(%2%) is duplicated"), UNESCSTR(MOD.MODULE[mod]), mod)$$END$
		$END$
	$END$

$	// ターゲット依存のエラーチェック
	$IF ISFUNCTION("HOOK_ERRORCHECK_MOD")$
		$HOOK_ERRORCHECK_MOD(mod)$
	$END$

$	// メモリオブジェクト情報の生成
	$FOREACH dsec DSEC.ORDER_LIST$
		$nummo = nummo + 1$
		$MO.TYPE[nummo] = TOPPERS_ATTMOD$
		$MO.MODULE[nummo] = UNESCSTR(MOD.MODULE[mod])$
		$MO.LINKER[nummo] = 1$
		$IF EQ(MOD.OSAPID[mod], "")$
			$MO.OSAPID[nummo] = TDOM_NONE$
		$ELSE$
			$MO.OSAPID[nummo] = MOD.OSAPID[mod]$
		$END$
		$IF DSEC.MEMREG[dsec] == 1$
			$MO.MEMREG[nummo] = STANDARD_ROM$
		$ELSE$
			$MO.MEMREG[nummo] = STANDARD_RAM$
		$END$
		$MO.SECTION[nummo] = DSEC.SECTION[dsec]$
		$MO.MEMATR[nummo] = DSEC.MEMATR[dsec]$

		$domptn = DEFAULT_ACPTN[MO.OSAPID[nummo]]$
		$MO.ACPTN1[nummo] = domptn$
		$MO.ACPTN2[nummo] = domptn$
		$IF MOD.EXPORT[mod] == 1$
			$MO.ACPTN2[nummo] = TACP_SHARED$
		$END$
		$MO.TEXT_LINE[nummo] = MOD.TEXT_LINE[mod]$
	$END$
	$MOD_LIST = APPEND(MOD_LIST, mod)$
$END$

$
$  OsMemorySection定義と同等の機能を提供
$  第1引数 セクション名
$  第2引数 メモリ属性
$  第3引数 メモリリージョン
$  第4引数 共有リード専有ライトにするか
$
$FUNCTION ATT_SEC$
	$section = ARGV[1]$
	$mematr = ARGV[2]$
	$memreg = ARGV[3]$
	$export = ARGV[4]$

	$numsec = LENGTH(SEC.ORDER_LIST)$
	$numsec = numsec + 1$
	$SEC.ORDER_LIST = APPEND(numsec, SEC.ORDER_LIST)$
	$SEC.SECTION[numsec] = ESCSTR(section)$
	$SEC.MEMATR[numsec] = mematr$
	$SEC.MEMREG[numsec] = REG.REGION[memreg]$
	$SEC.OSAPID[numsec] = TDOM_KERNEL$
	$SEC.TEXT_LINE[numsec] = 0$
	$SEC.EXPORT[numsec] = export$
$END$

$ATT_SEC(".srpw_bss_kernel", TA_SDATA, STANDARD_RAM, 1)$

$
$  OsMemorySection／OsLinkSectionで登録されたセクションに関する情報の生成
$
$ SEC_LIST：処理済みのセクションのリスト
$
$FOREACH sec SEC.ORDER_LIST$
$	// sectionが標準のセクションの場合
	$IF TOPPERS_SUPPORT_ATT_MOD || !LENGTH(SEC.MEMATR[sec])$
		$IF LENGTH(FIND(DSEC_SECTION_LIST, UNESCSTR(SEC.SECTION[sec])))$
			$ERROR SEC.TEXT_LINE[sec]$
				$FORMAT(_("OsMemorySectionName of OsMemorySection(%1%) can't be attached default section(%2%)"), sec, UNESCSTR(SEC.SECTION[sec]))$
			$END$
		$END$
	$END$

$	// sectionが登録済みの場合
	$FOREACH sec2 SEC_LIST$
		$IF EQ(SEC.SECTION[sec], SEC.SECTION[sec2])$
			$ERROR SEC.TEXT_LINE[sec]$
				$FORMAT(_("OsMemorySectionName(%1%) of OsMemorySection(%2%) is duplicated"), UNESCSTR(SEC.SECTION[sec]), sec)$
			$END$
		$END$
	$END$

$	// memregのチェック
	$memreg = 0$
	$FOREACH reg REG.ORDER_LIST$
		$IF EQ(UNESCSTR(SEC.MEMREG[sec]), REG.REGNAME[reg])$
			$memreg = reg$
		$END$
	$END$
	$IF memreg == 0$
		$ERROR SEC.TEXT_LINE[sec]$
			$FORMAT(_("illegal OSMemorySectionMemoryRegionRef(%1%) of OsMemorySection(%2%)"), SEC.MEMREG[sec], sec)$
		$END$
$		// 以降のエラーの抑止
		$memreg = STANDARD_RAM$
	$END$

	$IF LENGTH(SEC.MEMATR[sec])$
$		// OsMemorySectionの場合

$		// mematrが（［TA_NOWRITE|TA_NOREAD|TA_EXEC|TA_MEMINI|TA_MEMPRSV|TA_SDATA|TA_UNCACHE|TA_IODEV］）でない場合
$		// mematrにTA_MEMINIとTA_MEMPRSVの両方を指定した場合（TA_RSATR）
		$IF (SEC.MEMATR[sec] & ~(TA_NOWRITE|TA_NOREAD|TA_EXEC|TA_MEMINI|TA_MEMPRSV|TA_SDATA|TA_UNCACHE|TA_IODEV|TARGET_MEMATR)) != 0
			|| (SEC.MEMATR[sec] & (TA_MEMINI|TA_MEMPRSV)) == (TA_MEMINI|TA_MEMPRSV)$
			$ERROR SEC.TEXT_LINE[sec]$$FORMAT(_("illegal attribute(%1%) of OsMemorySection(%2%)"), SEC.MEMATR[sec], sec)$$END$
		$END$

		$IF (SEC.EXPORT[sec] == 1 && (SEC.MEMATR[sec] & (TA_NOWRITE | TA_NOREAD | TA_EXEC)))$
			$ERROR SEC.TEXT_LINE[sec]$$FORMAT(_("illegal Exported section memory attribute(%1%) of OsMemorySection(%2%)"), SEC.MEMATR[sec], sec)$$END$
		$END$

$		// ターゲット依存のエラーチェック
		$IF ISFUNCTION("HOOK_ERRORCHECK_SEC")$
			$HOOK_ERRORCHECK_SEC(sec)$
		$END$

$		// メモリオブジェクト情報の生成
		$nummo = nummo + 1$
		$MO.TYPE[nummo] = TOPPERS_ATTSEC$
		$MO.LINKER[nummo] = 1$
		$IF EQ(SEC.OSAPID[sec], "")$
			$MO.OSAPID[nummo] = TDOM_NONE$
		$ELSE$
			$MO.OSAPID[nummo] = SEC.OSAPID[sec]$
		$END$
		$MO.MEMREG[nummo] = memreg$
		$MO.SECTION[nummo] = UNESCSTR(SEC.SECTION[sec])$
		$IF (REG.REGATR[memreg] & TA_NOWRITE) != 0$
$			// メモリリージョン属性にTA_NOWRITEが設定されている時は，
$			// メモリオブジェクト属性にTA_NOWRITEを設定する．
			$MO.MEMATR[nummo] = SEC.MEMATR[sec] | TA_NOWRITE$
		$ELSE$
			$MO.MEMATR[nummo] = SEC.MEMATR[sec]$
		$END$

		$domptn = DEFAULT_ACPTN[MO.OSAPID[nummo]]$
		$MO.ACPTN1[nummo] = domptn$
		$MO.ACPTN2[nummo] = domptn$
		$IF SEC.EXPORT[sec] == 1$
			$MO.ACPTN2[nummo] = TACP_SHARED$
		$END$
		$MO.TEXT_LINE[nummo] = SEC.TEXT_LINE[sec]$
	$ELSE$
$		// OsLinkSectionの場合

$		// 配置するセクション情報の生成
		$numls = numls + 1$
		$LNKSEC.MEMREG[numls] = memreg$
		$LNKSEC.SECTION[numls] = UNESCSTR(SEC.SECTION[sec])$
	$END$
	$SEC_LIST = APPEND(SEC_LIST, sec)$
$END$

$
$  OsLinkSection定義と同等の機能を提供
$  第1引数 セクション名
$  第3引数 メモリリージョン
$
$FUNCTION LNK_SEC$
	$section = ARGV[1]$
	$memreg = ARGV[2]$

	$numls = numls + 1$
	$LNKSEC.MEMREG[numls] = memreg$
	$LNKSEC.SECTION[numls] = section$
$END$

$
$  保護ドメイン毎の標準セクションに関する情報の生成
$
$FOREACH domid DOMLIST_ALL$
	$FOREACH dsec DSEC.ORDER_LIST$
$		// メモリオブジェクト情報の生成
		$nummo = nummo + 1$
		$MO.TYPE[nummo] = TOPPERS_ATTSEC$
		$MO.LINKER[nummo] = 1$
		$MO.OSAPID[nummo] = domid$
		$IF DSEC.MEMREG[dsec] == 1$
			$MO.MEMREG[nummo] = STANDARD_ROM$
		$ELSE$
			$MO.MEMREG[nummo] = STANDARD_RAM$
		$END$
		$MO.SECTION[nummo] = FORMAT("%s_%s", DSEC.SECTION[dsec], OSAP.LABEL[domid])$
		$MO.MEMATR[nummo] = DSEC.MEMATR[dsec]$

		$domptn = DEFAULT_ACPTN[domid]$
		$MO.ACPTN1[nummo] = domptn$
		$MO.ACPTN2[nummo] = domptn$
	$END$
$END$

$
$  OsMemoryAreaで登録されたセクションに関する情報の生成
$
$FOREACH mem MEM.ORDER_LIST$
$	// mematrが（［TA_NOWRITE|TA_NOREAD|TA_EXEC|TA_MEMINI|TA_MEMPRSV|TA_UNCACHE|TA_IODEV］）でない場合
$	// mematrにTA_MEMPRSVを指定しないかTA_MEMINIを指定した場合（TA_RSATR）
	$IF (MEM.MEMATR[mem] & ~(TA_NOWRITE|TA_NOREAD|TA_EXEC|TA_MEMINI|TA_MEMPRSV|TA_UNCACHE|TA_IODEV|TARGET_MEMATR)) != 0
		|| (MEM.MEMATR[mem] & (TA_MEMINI|TA_MEMPRSV)) != TA_MEMPRSV$
		$ERROR MEM.TEXT_LINE[mem]$$FORMAT(_("illegal attribute(%1%) of OsMemoryArea(%2%)"), MEM.MEMATR[mem], mem)$$END$
	$END$

$	// sizeが0の場合
	$IF MEM.SIZE[mem] == 0$
		$ERROR MEM.TEXT_LINE[mem]$$FORMAT(_("OsMemoryAreaSize of OsMemoryArea(%2%) must not be zero"), MEM.SIZE[mem], mem)$$END$
	$END$

$	// ターゲット依存のエラーチェック
	$IF ISFUNCTION("HOOK_ERRORCHECK_MEM")$
		$HOOK_ERRORCHECK_MEM(mem)$
	$END$

$	// メモリオブジェクト情報の生成
	$nummo = nummo + 1$
	$MO.TYPE[nummo] = TOPPERS_ATTMEM$
	$MO.BASE[nummo] = MEM.BASE[mem]$
	$MO.SIZE[nummo] = MEM.SIZE[mem]$
	$MO.LINKER[nummo] = 0$
	$IF EQ(MEM.OSAPID[mem], "")$
		$MO.OSAPID[nummo] = TDOM_NONE$
	$ELSE$
		$MO.OSAPID[nummo] = MEM.OSAPID[mem]$
	$END$
	$MO.MEMATR[nummo] = MEM.MEMATR[mem]$

	$domptn = DEFAULT_ACPTN[MO.OSAPID[nummo]]$
	$MO.ACPTN1[nummo] = domptn$
	$MO.ACPTN2[nummo] = domptn$
	$MO.TEXT_LINE[nummo] = MEM.TEXT_LINE[mem]$
$END$

$FOREACH moid RANGE(1, nummo)$
	$IF MO.OSAPID[moid] != TDOM_KERNEL && MO.OSAPID[moid] != TDOM_NONE &&
		OSAP.TRUSTED[MO.OSAPID[moid]]$
		$MO.OSAPID[moid] = TDOM_KERNEL$
	$END$
$END$

$ =====================================================================
$ HRP2からの移植ここまで
$ =====================================================================


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

#define CFG_USE_SCALABILITYCLASS3$NL$
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
#define TNUM_ISR2				UINT_C($LENGTH(ISR2.ID_LIST)$)$NL$
#define TNUM_STD_RESOURCE		UINT_C($LENGTH(STDRES.ID_LIST)$)$NL$
#define TNUM_TASK				UINT_C($LENGTH(TSK.ID_LIST) - tnum_os_restarttask$)$NL$
#define TNUM_TASK_INC_RT		UINT_C($LENGTH(TSK.ID_LIST)$)$NL$
#define TNUM_EXTTASK			UINT_C($tnum_exttask$)$NL$
#define TNUM_APP_MODE			UINT_C($LENGTH(APP.ID_LIST)$)$NL$
#define TNUM_SCHEDULETABLE		UINT_C($LENGTH(SCHTBL.ID_LIST)$)$NL$
#define TNUM_IMPLSCHEDULETABLE	UINT_C($tnum_implscheduletable$)$NL$
#define	TNUM_TFN				UINT_C($LENGTH(TFN.ID_LIST)$)$NL$
#define TNUM_OSAP				UINT_C($LENGTH(OSAP.ID_LIST)$)$NL$
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
$	//OSが生成したリスタートタスク以外のタスクIDを出力する
	$IF tskid < tmin_os_restarttask$
		#define $tskid$$TAB$UINT_C($TSK.ID[tskid]$)$NL$
	$END$
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

$ 信頼関数のIDの出力
 /****** Object Trusted Function ******/$NL$$NL$
$FOREACH tfnid TFN.ID_LIST$
	#define $tfnid$$TAB$UINT_C($TFN.ID[tfnid]$)$NL$
$END$
$NL$

$ OSAPのIDの出力
 /****** Object OSApplication ******/$NL$
$FOREACH osapid OSAP.ID_LIST$
	#define $osapid$ UINT_C($OSAP.ID[osapid]$)$NL$
$END$
$NL$

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
const AlarmType					tnum_alarm				= TNUM_ALARM;$NL$
const CounterType				tnum_counter			= TNUM_COUNTER;$NL$
const CounterType				tnum_hardcounter		= TNUM_HARDCOUNTER;$NL$
const ISRType					tnum_isr2				= TNUM_ISR2;$NL$
const ResourceType				tnum_stdresource		= TNUM_STD_RESOURCE;$NL$
const TaskType					tnum_task				= TNUM_TASK;$NL$
const TaskType					tnum_exttask			= TNUM_EXTTASK;$NL$
const AppModeType				tnum_appmode			= TNUM_APP_MODE;$NL$
const ScheduleTableType			tnum_scheduletable		= TNUM_SCHEDULETABLE;$NL$
const ScheduleTableType			tnum_implscheduletable	= TNUM_IMPLSCHEDULETABLE;$NL$
const TrustedFunctionIndexType	tnum_tfn				= TNUM_TFN;$NL$
const ApplicationType			tnum_osap				= TNUM_OSAP;$NL$
$NL$

$
$  タスク
$
$NL$
 /****** Object TASK ******/$NL$
$NL$

$ =====================================================================
$ 以下はHRP2からの移植
$ =====================================================================
$
$  システムスタック領域の確保関数
$
$IF !ISFUNCTION("ALLOC_SSTACK")$
$FUNCTION ALLOC_SSTACK$
	$IF !EQ(ARGV[3], "")$
		$ARGV[3]$$SPC$
	$END$
	StackType $ARGV[1]$[COUNT_STK_T($ARGV[2]$)];$NL$
$END$
$END$

$  システムスタックサイズが，0であるか，ターゲット定義の最小値
$  （TARGET_MIN_SSTKSZ）よりも小さい場合のエラーチェック関数
$FUNCTION CHECK_MIN_SSTKSZ$
	$IF ARGV[1] == 0 || (TARGET_MIN_SSTKSZ && ARGV[1] < TARGET_MIN_SSTKSZ)$
		$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("%1%(%2%) of OsTask(%3%) is too small"), ARGV[2], ARGV[1], tskid)$$END$
	$END$
$END$

$ システムスタック領域の生成とそれに関するエラーチェック
$FOREACH tskid TSK.ID_LIST$
	$IF OSAP.TRUSTED[TSK.OSAPID[tskid]]$
$		// 信頼タスクの場合の処理

$		// sstkが省略されておらず，NULLでない場合
		$IF LENGTH(TSK.SSTK[tskid]) && !EQ(TSK.SSTK[tskid],"NULL")$
			$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskSystemStackStartAddress of trusted OsTask(%1%) must not define"), tskid)$$END$
		$END$

		$IF LENGTH(TSK.STKSZ[tskid]) && TSK.STKSZ[tskid] != 0$
			$TSK.STKSZ[tskid] = VALUE(CONCAT(+TSK.STKSZ[tskid],"U"),+TSK.STKSZ[tskid])$
		$ELSE$
			$TSK.STKSZ[tskid] = VALUE(CONCAT(+DEFAULT_TASKSTKSZ,"U"),+DEFAULT_TASKSTKSZ)$
		$END$
$		//信頼タスクに対するSystemStackSizeは，存在する場合にStackSizeに合算する仕様であるため，存在しない場合にはDEFAULT_TASKSTKSZにしない
		$IF LENGTH(TSK.SSTKSZ[tskid]) && TSK.SSTKSZ[tskid] != 0$
			$TSK.SSTKSZ[tskid] = VALUE(CONCAT(+TSK.SSTKSZ[tskid],"U"),+TSK.SSTKSZ[tskid])$
		$END$

		$IF EQ(TSK.STK[tskid], "NULL")$
$			// stkがNULLの場合の処理

			$IF LENGTH(TSK.SSTKSZ[tskid]) && TSK.SSTKSZ[tskid] > 0$
$				// sstkszが省略されていない場合の処理

$				// システムスタック領域のサイズを求める（エラーチェックに
$				// 使うため，エラーチェックの前に求めておく）
				$sstksz = VALUE(FORMAT("(%1%) + (%2%)",
									TSK.STKSZ[tskid], TSK.SSTKSZ[tskid]),
								TSK.STKSZ[tskid] + TSK.SSTKSZ[tskid])$

$				// stksz+sstkszがターゲット定義の最小値よりも小さい場合
				$CHECK_MIN_SSTKSZ(sstksz, "OsTaskStackSize+OsTaskSystemStackSize")$
			$ELSE$
$				// sstkszが省略されている場合の処理

$				// stkszが，ターゲット定義の最小値よりも小さい場合
				$CHECK_MIN_SSTKSZ(TSK.STKSZ[tskid], "OsTaskStackSize")$

$				// システムスタック領域のサイズを求める
				$sstksz = TSK.STKSZ[tskid]$
			$END$

			$IF TSK.EXTENDED[tskid]$
$				// システムスタック領域の確保
				$ALLOC_SSTACK(CONCAT("_kernel_sstack_", tskid), sstksz, "")$
				$TSK.TINIB_SSTKSZ[tskid] = FORMAT("ROUND_STK_T(%1%)", sstksz)$
				$TSK.TINIB_SSTK[tskid] = CONCAT("_kernel_sstack_", tskid)$
			$ELSE$
				$TSK.TINIB_SSTK[tskid] = CONCAT("_kernel_shared_sstack_", +TSK.PRIORITY[tskid])$
				$TSK.SHARED_SSTK_ID[tskid] = TSK.PRIORITY[tskid]$
$ 				// 基本タスク用の共有スタックのサイズを求める
				$IF !LENGTH(shared_sstack_size[TSK.PRIORITY[tskid]])
						|| shared_sstack_size[TSK.PRIORITY[tskid]] < sstksz$
					$shared_sstack_size[TSK.PRIORITY[tskid]] = sstksz$
				$END$
			$END$
		$ELSE$
$			// stkがNULLでない場合の処理

$			// stkszが，ターゲット定義の最小値よりも小さい場合
			$CHECK_MIN_SSTKSZ(TSK.STKSZ[tskid], "OsTaskStackSize")$

$			// stkszがスタック領域のサイズの制約を満たしていない場合
			$IF CHECK_STKSZ_ALIGN
							&& (TSK.STKSZ[tskid] & (CHECK_STKSZ_ALIGN - 1))$
				$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskStackSize(%1%) of OsTask(%2%) is not aligned"), TSK.STKSZ[tskid], tskid)$$END$
			$END$

$			// sstkszが省略されておらず，0でない場合
			$IF LENGTH(TSK.SSTKSZ[tskid]) && (TSK.SSTKSZ[tskid] != 0)$
				$ERROR TSK.TEXT_LINE[tskid]$
						$FORMAT(_("OsTaskSystemStackSize(%1%) of OsTask(%2%) must be zero because OsTaskStackStartAddress defined"), TSK.SSTKSZ[tskid], tskid)$$END$
			$END$

			$TSK.TINIB_SSTKSZ[tskid] = TSK.STKSZ[tskid]$
			$TSK.TINIB_SSTK[tskid] = TSK.STK[tskid]$
		$END$
	$ELSE$
$		// 非信頼タスクの場合

		$IF !LENGTH(TSK.SSTK[tskid]) || EQ(TSK.SSTK[tskid], "NULL")$
$			// sstkが省略されているか，NULLの場合の処理

			$IF LENGTH(TSK.SSTKSZ[tskid]) && TSK.SSTKSZ[tskid] != 0$
				$TSK.SSTKSZ[tskid] = VALUE(CONCAT(+TSK.SSTKSZ[tskid],"U"),+TSK.SSTKSZ[tskid])$
$				// sstkszが0か，ターゲット定義の最小値よりも小さい場合
				$CHECK_MIN_SSTKSZ(TSK.SSTKSZ[tskid], "OsTaskSystemStackSize")$
			$ELSE$
				$TSK.SSTKSZ[tskid] = VALUE(CONCAT(+DEFAULT_TASKSYSTEMSTKSZ,"U"),+DEFAULT_TASKSYSTEMSTKSZ)$
			$END$

$			// システムスタック領域のサイズを求める
			$sstksz = TSK.SSTKSZ[tskid]$


			$IF TSK.EXTENDED[tskid]$
$				// システムスタック領域の確保
				$TSK.TINIB_SSTKSZ[tskid] = FORMAT("ROUND_STK_T(%1%)", sstksz)$
				$ALLOC_SSTACK(CONCAT("_kernel_sstack_", tskid), TSK.TINIB_SSTKSZ[tskid], "")$
				$TSK.TINIB_SSTK[tskid] = CONCAT("_kernel_sstack_", tskid)$
			$ELIF !EQ(TSK.RESTARTTASK[tskid],"")$
				$TSK.TINIB_SSTKSZ[tskid] = FORMAT("ROUND_STK_T(%1%)", sstksz)$
				$ALLOC_SSTACK(CONCAT("_kernel_sstack_restart_", TSK.RESTARTTASK[tskid]), TSK.TINIB_SSTKSZ[tskid], "")$
				$TSK.TINIB_SSTK[tskid] = CONCAT("_kernel_sstack_restart_", TSK.RESTARTTASK[tskid])$
			$ELSE$
				$TSK.TINIB_SSTK[tskid] = CONCAT("_kernel_shared_sstack_", +TSK.PRIORITY[tskid])$
				$TSK.SHARED_SSTK_ID[tskid] = TSK.PRIORITY[tskid]$
$ 				// 基本タスク用の共有スタックのサイズを求める
				$IF !LENGTH(shared_sstack_size[TSK.PRIORITY[tskid]])
						|| shared_sstack_size[TSK.PRIORITY[tskid]] < sstksz$
					$shared_sstack_size[TSK.PRIORITY[tskid]] = sstksz$
				$END$
			$END$
		$ELSE$
$			// sstkが省略されておらず，NULLでない場合の処理

			$IF LENGTH(TSK.SSTKSZ[tskid]) && TSK.SSTKSZ[tskid] != 0$
				$TSK.SSTKSZ[tskid] = VALUE(CONCAT(+TSK.SSTKSZ[tskid],"U"),+TSK.SSTKSZ[tskid])$
			$ELSE$
				$TSK.SSTKSZ[tskid] = VALUE(CONCAT(+DEFAULT_TASKSYSTEMSTKSZ,"U"),+DEFAULT_TASKSYSTEMSTKSZ)$
			$END$

$			// sstkszが0か，ターゲット定義の最小値よりも小さい場合
			$CHECK_MIN_SSTKSZ(TSK.SSTKSZ[tskid], "OsTaskSystemStackSize")$

$			// sstkszがスタック領域のサイズの制約を満たしていない場合
			$IF CHECK_STKSZ_ALIGN
							&& (TSK.SSTKSZ[tskid] & (CHECK_STKSZ_ALIGN - 1))$
				$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskSystemStackSize(%1%) of OsTask(%2%) is not aligned"), TSK.SSTKSZ[tskid], tskid)$$END$
			$END$

			$TSK.TINIB_SSTKSZ[tskid] = TSK.SSTKSZ[tskid]$
			$TSK.TINIB_SSTK[tskid] = TSK.SSTK[tskid]$
		$END$
	$END$
$END$

$FOREACH tskid TSK.ID_LIST$
	$IF LENGTH(TSK.SHARED_SSTK_ID[tskid])$
		$TSK.TINIB_SSTKSZ[tskid] = FORMAT("ROUND_STK_T(%1%)", shared_sstack_size[TSK.PRIORITY[tskid]])$
	$END$
$END$

$FOREACH tskpri RANGE(TMIN_TPRI, TMAX_TPRI)$
	$IF LENGTH(shared_sstack_size[tskpri])$
		$ALLOC_SSTACK(CONCAT("_kernel_shared_sstack_", +tskpri), shared_sstack_size[tskpri], "")$
	$END$
$END$
$NL$

$ ユーザスタック領域の生成とそれに関するエラーチェック
$FOREACH tskid TSK.ID_LIST$
	$IF OSAP.TRUSTED[TSK.OSAPID[tskid]] || tmin_os_restarttask <= tskid $
$		// 信頼タスクもしくはOSが生成したリスタートタスクの場合の処理
		$TSK.TINIB_USTKSZ[tskid] = VALUE("0U", 0)$
		$TSK.TINIB_USTK[tskid] = "NULL"$
	$ELSE$
$		// 非信頼タスクの場合の処理

$		// MO生成するかのフラグ
		$create_mo = 1$

		$IF LENGTH(TSK.STKSZ[tskid]) && TSK.STKSZ[tskid] != 0$
			$TSK.STKSZ[tskid] = VALUE(CONCAT(+TSK.STKSZ[tskid],"U"),+TSK.STKSZ[tskid])$
		$ELSE$
			$TSK.STKSZ[tskid] = VALUE(CONCAT(+DEFAULT_TASKSTKSZ,"U"),+DEFAULT_TASKSTKSZ)$
		$END$

$		// stkszが0か，ターゲット定義の最小値（TARGET_MIN_USTKSZ）よりも
$		// 小さい場合
		$IF TSK.STKSZ[tskid] == 0 || (TARGET_MIN_USTKSZ &&
							TSK.STKSZ[tskid] < TARGET_MIN_USTKSZ)$
			$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskStackSize(%1%) of OsTask(%2%) is too small"), TSK.STKSZ[tskid], tskid)$$END$
		$END$

		$IF EQ(TSK.STK[tskid],"NULL")$
$			// stkがNULLの場合の処理

$			// ユーザスタック領域の確保
			$IF TSK.EXTENDED[tskid]$
				$ALLOC_USTACK(tskid, TSK.STKSZ[tskid])$
			$ELSE$
				$TSK.TINIB_USTK[tskid] = CONCAT("_kernel_shared_ustack_", +TSK.PRIORITY[tskid])$
				$TSK.SHARED_USTK_ID[tskid] = TSK.PRIORITY[tskid]$
$ 				// 基本タスク用の共有スタックのサイズを求める
				$IF !LENGTH(shared_ustack_size[TSK.PRIORITY[tskid]])
						|| shared_ustack_size[TSK.PRIORITY[tskid]] < TSK.STKSZ[tskid]$
					$shared_ustack_size[TSK.PRIORITY[tskid]] = TSK.STKSZ[tskid]$
				$END$
			$END$
		$ELSE$
$			// stkがNULLでないの場合の処理

$			// stkszがスタック領域のサイズの制約を満たしていない場合
			$IF CHECK_USTKSZ_ALIGN
							&& (TSK.STKSZ[tskid] & (CHECK_USTKSZ_ALIGN - 1))$
				$ERROR TSK.TEXT_LINE[tskid]$$FORMAT(_("OsTaskStackSize(%1%) of OsTask(%2%) is not aligned"), TSK.STKSZ[tskid], tskid)$$END$
			$END$

			$TSK.TINIB_USTKSZ[tskid] = TSK.STKSZ[tskid]$
			$TSK.TINIB_USTK[tskid] = TSK.STK[tskid]$

$			// 共有するユーザスタック領域の検出
			$FOREACH moid MO_USTACK_LIST2$
				$IF MO.OSAPID[moid] == TSK.OSAPID[tskid] && 
						EQ(MO.BASE[moid], TSK.STK[tskid]) && 
							EQ(MO.SIZE[moid], TSK.STKSZ[tskid])$
					$create_mo = 0$
					$TSK.USTACK_MO[tskid] = moid$
				$END$
			$END$
		$END$

		$IF LENGTH(TSK.SHARED_USTK_ID[tskid])$
			$IF LENGTH(shared_ustack_mo[TSK.SHARED_USTK_ID[tskid]])$
				$create_mo = 0$
				$TSK.USTACK_MO[tskid] = shared_ustack_mo[TSK.SHARED_USTK_ID[tskid]]$
			$END$
		$END$

		$IF create_mo$
$			// メモリオブジェクト情報の生成
			$nummo = nummo + 1$
			$MO.TYPE[nummo] = TOPPERS_USTACK$
			$MO.TSKID[nummo] = tskid$
			$MO.STKORDER[nummo] = tskid$
			$IF !EQ(TSK.STK[tskid],"NULL")$
				$MO.BASE[nummo] = TSK.STK[tskid]$
				$MO.LINKER[nummo] = 0$
				$MO.SIZE[nummo] = TSK.TINIB_USTKSZ[tskid]$
				$MO_USTACK_LIST2 = APPEND(MO_USTACK_LIST2, nummo)$
			$ELSE$
				$MO.LINKER[nummo] = 1$
				$MO.MEMREG[nummo] = STANDARD_RAM$
				$IF LENGTH(TSK.SHARED_USTK_ID[tskid])$
					$MO.SECTION[nummo] = SECTION_SHARED_USTACK(TSK.SHARED_USTK_ID[tskid])$
					$shared_ustack_mo[TSK.SHARED_USTK_ID[tskid]] = nummo$
				$ELSE$
					$MO.SECTION[nummo] = SECTION_USTACK(tskid)$
					$MO.SIZE[nummo] = TSK.STKSZ[tskid]$
				$END$
				$MO_USTACK_LIST = APPEND(MO_USTACK_LIST, nummo)$
			$END$
			$MO.OSAPID[nummo] = TSK.OSAPID[tskid]$
			$MO.MEMATR[nummo] = TARGET_MEMATR_USTACK$

			$domptn = DEFAULT_ACPTN[MO.OSAPID[nummo]]$
			$MO.ACPTN1[nummo] = domptn$
			$MO.ACPTN2[nummo] = domptn$
			$MO.TEXT_LINE[nummo] = TSK.TEXT_LINE[tskid]$
			$TSK.USTACK_MO[tskid] = nummo$
		$END$
	$END$
$END$

$ =====================================================================
$ HRP2からの移植ここまで
$ =====================================================================

$ OSAPCBの出力
$IF LENGTH(OSAP.ID_LIST)$
	OSAPCB osapcb_table[TNUM_OSAP];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(OSAPCB, osapcb_table);$NL$
$END$

$FOREACH tskid TSK.ID_LIST$
	$IF LENGTH(TSK.SHARED_USTK_ID[tskid])$
		$TSK.TINIB_USTKSZ[tskid] = shared_ustack_size[TSK.PRIORITY[tskid]]$
		$MO.SIZE[TSK.USTACK_MO[tskid]] = TSK.TINIB_USTKSZ[tskid]$
	$END$
$END$

$FOREACH tskpri RANGE(TMIN_TPRI, TMAX_TPRI)$
	$IF LENGTH(shared_ustack_size[tskpri])$
		$shared_ustack_size[tskpri] = CONCAT(USTACK_ALIGN_SIZE(shared_ustack_size[tskpri]), "U")$
		$ALLOC_SHARED_USTACK(CONCAT("_kernel_shared_ustack_", +tskpri), tskpri, shared_ustack_size[tskpri])$
	$END$
$END$

$NL$

$ TCBの出力
$IF LENGTH(TSK.ID_LIST)$
	TCB tcb_table[TNUM_TASK_INC_RT];$NL$
	$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(TCB, tcb_table);$NL$
$END$

$
$ カウンタの出力
$
$NL$
 /****** Object COUNTER ******/$NL$
$NL$

$IF LENGTH(CNT.ID_LIST)$
	const CNTINIB cntinib_table[TNUM_COUNTER] = {$NL$
	$JOINEACH cntid CNT.ID_LIST ",\n"$
		$TAB${ $+CNT.MAXALLOWED[cntid]$U,$SPC$
		($+CNT.MAXALLOWED[cntid]$U * 2U) + 1U,$SPC$
		$+CNT.TICKSPERBASE[cntid]$U,$SPC$
		$+CNT.MINCYCLE[cntid]$U,$SPC$
		&(osapcb_table[$CNT.OSAPID[cntid]$]),$SPC$
		$FORMAT("0x%08xU", +CNT.ACSBTMP[cntid])$ }
	$END$
	$NL$
	};$NL$
	$NL$

	CNTCB cntcb_table[TNUM_COUNTER];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const CNTINIB, cntinib_table);$NL$
	TOPPERS_EMPTY_LABEL(CNTCB, cntcb_table);$NL$
$END$

$ ハードウェアカウンタポインタテーブル定義の出力
$IF LENGTH(HWCNT.ID_LIST)$
	const HWCNTINIB hwcntinib_table[TNUM_HARDCOUNTER] = $NL$
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
			$TAB$$TAB$$CNT.NSPERTICK[cntid]$		/* $CNT.SECONDSPERTICK[cntid]$ * 1000000000 */ $NL$
		$TAB$}
	$END$
	$NL$};$NL$

$ELSE$
	TOPPERS_EMPTY_LABEL(const HWCNTINIB, hwcntinib_table);$NL$
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
		_activate_alarm_$+almid$(void);$NL$
		static void$NL$
		_activate_alarm_$+almid$(void)$NL$
		{$NL$
		$TAB$(void) activate_task_action(&(osapcb_table[$ALM.OSAPID[almid]$]), $ALM.TSKID[almid]$);$NL$
		}$NL$

$	// イベントセット用のアクション関数
	$ELIF EQ(ALM.ALMATR[almid], "SETEVENT")$
		static void$NL$
		_setevent_alarm_$+almid$(void);$NL$
		static void$NL$
		_setevent_alarm_$+almid$(void)$NL$
		{$NL$
		$TAB$(void) set_event_action(&(osapcb_table[$ALM.OSAPID[almid]$]), $ALM.TSKID[almid]$, $ALM.EVTID[almid]$);$NL$
		}$NL$

$	// IncrementCounter用のアクション関数
	$ELIF EQ(ALM.ALMATR[almid], "INCREMENTCOUNTER")$
		static void$NL$
		_incrementcounter_alarm_$+almid$(void);$NL$
		static void$NL$
		_incrementcounter_alarm_$+almid$(void)$NL$
		{$NL$
		$TAB$(void) incr_counter_action(&(osapcb_table[$ALM.OSAPID[almid]$]), $ALM.INCID[almid]$);$NL$
		}$NL$
	$END$

	$NL$
$END$

$ アラームコントロールブロック

$IF LENGTH(ALM.ID_LIST)$
	const ALMINIB alminib_table[TNUM_ALARM] = {$NL$
	$JOINEACH almid ALM.ID_LIST ",\n"$
		$TAB${ &(cntcb_table[$ALM.CNTID[almid]$]),$SPC$
		$IF EQ(ALM.ALMATR[almid], "ACTIVATETASK")$
$			// タスク起動用
			&_activate_alarm_$+almid$,$SPC$
		$ELIF EQ(ALM.ALMATR[almid], "SETEVENT")$
$			// イベントセット用
			&_setevent_alarm_$+almid$,$SPC$
		$ELIF EQ(ALM.ALMATR[almid], "INCREMENTCOUNTER")$
$			// IncrementCounter用
			&_incrementcounter_alarm_$+almid$,$SPC$
		$END$

		$FORMAT("0x%08xU", +ALM.ASTPTN[almid])$,$SPC$
		$IF ALM.ASTPTN[almid]$
			$+ALM.ALARMTIME[almid]$U,$SPC$
			$+ALM.CYCLETIME[almid]$U,$SPC$
$			// 自動起動属性以外の属性を管理しない
			$+ALM.AUTOSTARTTYPE[almid]$U,$SPC$
		$ELSE$
			0U, 0U, 0U,$SPC$
		$END$
		&(osapcb_table[$ALM.OSAPID[almid]$]),$SPC$
		$FORMAT("0x%08xU", +ALM.ACSBTMP[almid])$$SPC$}
	$END$
	$NL$
	};$NL$
	$NL$

	ALMCB	almcb_table[TNUM_ALARM];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const ALMINIB, alminib_table);$NL$
	TOPPERS_EMPTY_LABEL(ALMCB, almcb_table);$NL$
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
		_expire_scheduletable_$+schtblid$_$exppt_index$(void);$NL$
		static void$NL$
		_expire_scheduletable_$+schtblid$_$exppt_index$(void)$NL$
		{$NL$
$		// タスク起動
		$FOREACH expptactid EXPPTACT.ID_LIST$
			$IF EXPPTACT.EXPPTID[expptactid] == expptid && EQ(EXPPTACT.EXPIREATR[expptactid], "ACTIVATETASK")$
				$TAB$(void) activate_task_action(&(osapcb_table[$SCHTBL.OSAPID[EXPPTACT.SCHTBLID[expptactid]]$]), $EXPPTACT.TSKID[expptactid]$);$NL$
			$END$
		$END$
$		// イベントセット
		$FOREACH expptactid EXPPTACT.ID_LIST$
			$IF EXPPTACT.EXPPTID[expptactid] == expptid && EQ(EXPPTACT.EXPIREATR[expptactid], "SETEVENT")$
				$TAB$(void) set_event_action(&(osapcb_table[$SCHTBL.OSAPID[EXPPTACT.SCHTBLID[expptactid]]$]), $EXPPTACT.TSKID[expptactid]$, $EXPPTACT.EVTID[expptactid]$);$NL$
			$END$
		$END$
		}$NL$
	$END$
	$NL$
	static const SCHTBLEXPPTCB schtblexppt_table_$+schtblid$[$LENGTH(SCHTBL.EXPPTINDEX_LIST[schtblid])$] = {$NL$
	$JOINEACH exppt_index RANGE(0, LENGTH(SCHTBL.EXPPTINDEX_LIST[schtblid]) - 1) ",\n"$
		$expptid = AT(SCHTBL.EXPPTINDEX_LIST[schtblid], exppt_index)$
		$TAB${ $+EXPPT.OFFSET[expptid]$U, &_expire_scheduletable_$+schtblid$_$exppt_index$ }
	$END$
	$NL$
	};$NL$
	$NL$
$END$
$NL$

$ スケジュールテーブル関連のデータブロックの宣言
$IF LENGTH(SCHTBL.ID_LIST)$
	const SCHTBLINIB schtblinib_table[TNUM_SCHEDULETABLE] = {$NL$
	$JOINEACH schtblid SCHTBL.ID_LIST ",\n"$
		$TAB${ &(cntcb_table[$SCHTBL.CNTID[schtblid]$]),$SPC$
		$+SCHTBL.DURATIONTICK[schtblid]$U,$SPC$
		$FORMAT("0x%08xU", +SCHTBL.ASTPTN[schtblid])$,$SPC$
		$IF SCHTBL.ASTPTN[schtblid]$
			$+SCHTBL.AUTOSTARTTYPE[schtblid]$U,$SPC$
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
		&(osapcb_table[$SCHTBL.OSAPID[schtblid]$]),$SPC$
		$FORMAT("0x%08xU", +SCHTBL.ACSBTMP[schtblid])$ }
	$END$
	$NL$
	};$NL$
	$NL$
	SCHTBLCB schtblcb_table[TNUM_SCHEDULETABLE];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const SCHTBLINIB, schtblinib_table);$NL$
	TOPPERS_EMPTY_LABEL(SCHTBLCB, schtblcb_table);$NL$
$END$

$NL$


$
$ リソースの出力
$
$NL$
 /****** Object RESOURCE ******/$NL$
$NL$

$IF LENGTH(STDRES.ID_LIST)$
	const RESINIB resinib_table[TNUM_STD_RESOURCE] = {$NL$
	$JOINEACH resid STDRES.ID_LIST ",\n"$
		$IF RES.CEILPRI[resid] > 0$
$           タスクのみが扱うリソースの場合
			$TAB${ $TMAX_TPRI - RES.CEILPRI[resid]$,$SPC$
		$ELSE$
$           ISRも扱うリソースの場合
			$TAB${ $RES.CEILPRI[resid]$,$SPC$
		$END$
		$FORMAT("0x%08xU", +RES.ACSBTMP[resid])$ }
	$END$
	$NL$
	};$NL$
	$NL$

	RESCB rescb_table[TNUM_STD_RESOURCE];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const RESINIB, resinib_table);$NL$
	TOPPERS_EMPTY_LABEL(RESCB, rescb_table);$NL$
$END$

$NL$


$
$ オブジェクト初期化ルーチン
$
$NL$
void$NL$
object_initialize(void)$NL$
{$NL$
$TAB$osap_initialize();$NL$
$TAB$interrupt_initialize();$NL$
$IF LENGTH(STDRES.ID_LIST)$
$TAB$resource_initialize();$NL$
$END$
$TAB$task_initialize();$NL$
$ //OSが生成したリスタートタスクの初期化コードを追加する
$FOREACH killer RANGE(tmin_os_restarttask, LENGTH(TSK.ID_LIST))$
	$TAB$/* initialize the Os-genereted Restart Task */$NL$
	$TAB$tcb_table[$killer - 1$].p_tinib = &(tinib_table[$killer - 1$]);$NL$ 
$END$ 
$IF LENGTH(CNT.ID_LIST)$
$TAB$counter_initialize();$NL$
$END$
$IF LENGTH(ALM.ID_LIST)$
$TAB$alarm_initialize();$NL$
$END$
$IF LENGTH(SCHTBL.ID_LIST)$
$TAB$schtbl_initialize();$NL$
$END$
$IF LENGTH(IOC.ID_LIST)$
$TAB$ioc_initialize();$NL$
$END$
}$NL$
$NL$

$
$ オブジェクト終了処理ルーチン
$
$NL$
void$NL$
object_terminate(void)$NL$
{$NL$
$IF LENGTH(CNT.ID_LIST)$
$TAB$counter_terminate();$NL$
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
		$TAB$notify_hardware_counter($cntid$);$NL$
	}$NL$
$END$
$NL$

/*$NL$
$SPC$*  Stack Area for Non-task Context$NL$
$SPC$*/$NL$
$NL$

$ 非タスクコンテキスト用のスタック領域サイズ計算
$FOREACH isrid ISR2.ID_LIST$
$ 	// NULL(0)か0を指定した場合，デフォルト値設定
	$IF ISR.STKSZ[isrid] == 0$
		$ISR.STKSZ[isrid] = DEFAULT_ISRSTKSZ$
	$END$

$	// 非タスクコンテキスト用のスタック領域計算の為，優先度毎最大値の合計準備
	$IF LENGTH(ISR.STKSZ[isrid]) && (!LENGTH(isr_shared_stack_size[ISR.INTPRI[isrid]])
			|| isr_shared_stack_size[ISR.INTPRI[isrid]] < ISR.STKSZ[isrid])$
		$isr_shared_stack_size[ISR.INTPRI[isrid]] = ISR.STKSZ[isrid]$
	$END$
$END$

$	// MINIMUM_OSTKSZは，非タスクコンテキストスタックサイズの最小値として，依存部に設置
$total_stksz = MINIMUM_OSTKSZ$

$ ISRスタック領域のサイズを合計
$FOREACH isrpri RANGE(-MAX_PRI_ISR2, -MIN_PRI_ISR2)$
	$IF LENGTH(isr_shared_stack_size[isrpri])$
		$total_stksz = total_stksz + isr_shared_stack_size[isrpri]$
	$END$
$END$

$IF USE_HOOK$
$ 	// スタックサイズのデフォルト設定
$ 	// NULL(0)か0を指定した場合，デフォルト値設定
	$IF LENGTH(HSTK.ID_LIST) && HSTK.STKSZ[1] != 0$
		$total_stksz = total_stksz + HSTK.STKSZ[1]$
	$ELSE$
		$total_stksz = total_stksz + DEFAULT_HOOKSTKSZ$
	$END$
$END$

$IF OS.STACKMONITORING[1]$
$ スタック残量チェック方式用のスタック残量サイズ計算
$FOREACH isrid ISR.ID_LIST$
	$ISR.STKSZ_2[isrid] = ISR.STKSZ[isrid]$
	$IF (EQ(ISR.CATEGORY[isrid], "CATEGORY_2"))$
		$FOREACH isrpri RANGE(-ISR.INTPRI[isrid], -MIN_PRI_ISR2)$
			$IF ISR.INTPRI[isrid] < isrpri$
				$IF	LENGTH(isr_shared_stack_size[isrpri])$
					$ISR.STKSZ_2[isrid] = ISR.STKSZ_2[isrid] + isr_shared_stack_size[isrpri]$
				$END$
			$END$
		$END$
	$END$
$END$

$ フック有効時，スタック残量チェック方式用スタックサイズを加算
$IF USE_HOOK$
	$FOREACH isrid2 ISR2.ID_LIST$
		$IF LENGTH(HSTK.ID_LIST) && HSTK.STKSZ[1] != 0$
			$ISR.STKSZ_2[isrid2] = ISR.STKSZ_2[isrid2] + HSTK.STKSZ[1]$
		$ELSE$
			$ISR.STKSZ_2[isrid2] = ISR.STKSZ_2[isrid2] +DEFAULT_HOOKSTKSZ$
		$END$
	$END$
$END$
$END$

$ 割込み管理機能のための標準的な初期化情報の生成
$IF !OMIT_INITIALIZE_INTERRUPT$

$	// 割込み要求ライン数
	#define TNUM_INTNO	UINT_C($LENGTH(ISR.ID_LIST)$)$NL$
	const InterruptNumberType tnum_intno = TNUM_INTNO;$NL$
	$NL$

$	// 割込み要求ライン初期化テーブル
	$IF LENGTH(ISR.ID_LIST)$
		const INTINIB intinib_table[TNUM_INTNO] = {$NL$
		$JOINEACH isrid ISR.ID_LIST ",\n"$
			$TAB${ ($+ISR.INTNO[isrid]$U),$ISR.INTATR[isrid]$, ($-ISR.INTPRI[isrid]$), 
			$IF OS.STACKMONITORING[1]$
				$FORMAT("0x%xU",+ISR.STKSZ_2[isrid])$ 
			$END$
			}
		$END$$NL$
		};$NL$
	$ELSE$
		TOPPERS_EMPTY_LABEL(const INTINIB, intinib_table);$NL$
	$END$
	$NL$
$END$

$
$ ISRの出力
$
$NL$
 /****** Object ISR ******/$NL$
$NL$
$NL$

$IF LENGTH(ISR2.ID_LIST)$
	const ISRINIB isrinib_table[TNUM_ISR2] = {$NL$
	$JOINEACH isrid ISR2.ID_LIST ",\n"$
		$TAB${$NL$
		$TAB$$TAB$&(intinib_table[$isrid$]),$SPC$
		&(osapcb_table[$ISR.OSAPID[isrid]$]), $FORMAT("0x%08xU", +ISR.ACSBTMP[isrid])$$NL$
		$TAB$}
	$END$
	$NL$
	};$NL$
	$NL$

	ISRCB isrcb_table[TNUM_ISR2];$NL$
	$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const ISRINIB, isrinib_table);$NL$
	TOPPERS_EMPTY_LABEL(ISRCB, isrcb_table);$NL$
$END$

$
$  Trusted Function
$

$NL$
 /******** Trusted Function ********/
$NL$

$  Trusted Function 初期化ブロックの生成
$IF LENGTH(TFN.ID_LIST)$
	const TFINIB tfinib_table[TNUM_TFN] = {$NL$
	$JOINEACH tfnid TFN.ID_LIST ",\n"$
$		// 信頼関数スタックサイズのデフォルト設定
$ 		// NULL(0)か0を指定した場合，デフォルト設定
		$IF !LENGTH(TFN.STKSZ[tfnid]) || TFN.STKSZ[tfnid] == 0$
			$TFN.STKSZ[tfnid] = DEFAULT_TRUSTEDFUNCTIONSTKSZ$
		$END$
		$TAB${ &$TFN.FUNC[tfnid]$, $+TFN.STKSZ[tfnid]$U }
	$END$$NL$
	};$NL$
	$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const TFINIB, tfinib_table);$NL$
$END$

$
$  非タスクコンテキスト用のスタック領域
$
$IF !LENGTH(OSTK.ID_LIST)$
$	// OSTKがない場合の計算したスタック合計値の設定
$	// 非スタックコンテキストスタックサイズ = ISR共有後サイズ + MINIMUM_OSTKSZ
	$ALLOC_SSTACK("_kernel_ostack", FORMAT("0x%xU", +total_stksz), "static")$
	#define TOPPERS_OSTKSZ		$FORMAT("ROUND_STK_T(0x%xU)", +total_stksz)$$NL$
	#define TOPPERS_OSTK		(_kernel_ostack)$NL$
$ELSE$
$ 	// スタックサイズのデフォルト設定
$ 	// NULL(0)か0を指定した場合，デフォルト値設定
	$IF !LENGTH(OSTK.STKSZ[1]) || OSTK.STKSZ[1] == 0$
		$OSTK.STKSZ[1] = DEFAULT_OSSTKSZ$
	$END$

$ 	// stkszがスタック領域のサイズとして正しくない場合
	$IF !EQ(OSTK.STK[1], "NULL") && CHECK_STKSZ_ALIGN
							&& OSTK.STKSZ[1] & (CHECK_STKSZ_ALIGN - 1)$
		$ERROR OSTK.TEXT_LINE[1]$$FORMAT(_("OsOsStackSize(%1%) is not aligned"), OSTK.STKSZ[1])$$END$
	$END$

$ 	// stkszが必要なスタックサイズより小さい場合
	$IF OSTK.STKSZ[1] < total_stksz$
		$ERROR OSTK.TEXT_LINE[1]$$FORMAT(_("OsOsStackSize(%1%) is necessary 0x%2$x and more)"), OSTK.STKSZ[1], +total_stksz)$$END$
	$END$

	$IF EQ(OSTK.STK[1], "NULL")$
$		// スタック領域の自動割付け
		$ALLOC_SSTACK("_kernel_ostack", OSTK.STKSZ[1], "")$
		#define TOPPERS_OSTKSZ		ROUND_STK_T($OSTK.STKSZ[1]$)$NL$
		#define TOPPERS_OSTK		(_kernel_ostack)$NL$
	$ELSE$
		#define TOPPERS_OSTKSZ		($OSTK.STKSZ[1]$)$NL$
		#define TOPPERS_OSTK		($OSTK.STK[1]$)$NL$
	$END$
$END$
$NL$

$ 非タスクコンテキスト用のスタック領域
const MemorySizeType	_ostksz = TOPPERS_OSTKSZ;$NL$
StackType * const		_ostk = (StackType *) TOPPERS_OSTK;$NL$
$NL$
#ifdef TOPPERS_OSTKPT$NL$
$IF ISFUNCTION("DEFINE_CONST_VAR")$
	$DEFINE_CONST_VAR("StackType * const", "_ostkpt")$ = TOPPERS_OSTKPT(TOPPERS_OSTK, TOPPERS_OSTKSZ);$NL$
$ELSE$
	StackType * const	_ostkpt = TOPPERS_OSTKPT(TOPPERS_OSTK, TOPPERS_OSTKSZ);$NL$
$END$
#endif /* TOPPERS_OSTKPT */$NL$
$NL$

$ =====================================================================
$ IOC情報の生成
$ =====================================================================
$INCLUDE "kernel/ioc.tf"$


$FILE "Os_Lcfg.h"$
#ifndef TOPPERS_MACRO_ONLY$NL$

$
$  割込み管理機能
$
$NL$
/*$NL$
$SPC$*  Interrupt Management Functions$NL$
$SPC$*/$NL$
$NL$

$FOREACH isrid ISR.ID_LIST$
$	//割込み関数であることを示すコンパイルオプションを付ける関数が定義されている場合か
	$IF EQ(ISR.CATEGORY[isrid], "CATEGORY_1") && ISFUNCTION("EXTERN_C1ISR_HANDLER")$
		$EXTERN_C1ISR_HANDLER(ISR.INT_ENTRY[isrid])$$NL$
	$ELSE$
		extern void $ISR.INT_ENTRY[isrid]$(void);$NL$
	$END$
$END$

$ ISR用の割込みハンドラ
$FOREACH isrid ISR2.ID_LIST$
	extern ISR($isrid$);$NL$
$END$
$NL$

#ifdef TOPPERS_ENABLE_TRACE$NL$
extern const char8 *atk2_appid_str(AppModeType id);$NL$
extern const char8 *atk2_tskid_str(TaskType id);$NL$
extern const char8 *atk2_isrid_str(ISRType id);$NL$
extern const char8 *atk2_cntid_str(CounterType id);$NL$
extern const char8 *atk2_almid_str(AlarmType id);$NL$
extern const char8 *atk2_resid_str(ResourceType id);$NL$
extern const char8 *atk2_schtblid_str(ScheduleTableType id);$NL$
extern const char8 *atk2_evtid_str(TaskType task, EventMaskType event);$NL$
extern const char8 *atk2_osapid_str(ApplicationType id);$NL$
extern const char8 *atk2_iocid_str(IocType id);$NL$
extern const char8 *atk2_tfnid_str(TrustedFunctionIndexType id);$NL$
#endif /* TOPPERS_ENABLE_TRACE */$NL$
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

 /******** Trusted Function ********/
$NL$

$ TFNのextern宣言
$FOREACH tfnid TFN.ID_LIST$
extern TRUSTEDFUNCTION($TFN.FUNC[tfnid]$, FunctionIndex, FunctionParams);$NL$
$END$
$NL$

$IF ISFUNCTION("EXTERN_INT_HANDLER")$
	$EXTERN_INT_HANDLER()$
$END$

#endif /* TOPPERS_MACRO_ONLY */$NL$

#endif /* TOPPERS_OS_LCFG_H */$NL$


$FILE "Os_Lcfg.c"$
#ifdef TOPPERS_ENABLE_TRACE$NL$
const char8 *$NL$
atk2_appid_str(AppModeType id)$NL$
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
atk2_tskid_str(TaskType id)$NL$
{$NL$
$TAB$const char8	*tskid_str;$NL$
$TAB$switch (id) {$NL$
$FOREACH tskid TSK.ID_LIST$
$	//OSが生成したリスタートタスク以外のタスクIDを出力する
	$IF tskid < tmin_os_restarttask$
		$TAB$case $tskid$:$NL$
		$TAB$$TAB$tskid_str = "$tskid$";$NL$
		$TAB$$TAB$break;$NL$
	$END$
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
atk2_isrid_str(ISRType id)$NL$
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
atk2_cntid_str(CounterType id)$NL$
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
atk2_almid_str(AlarmType id)$NL$
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
atk2_resid_str(ResourceType id)$NL$
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
atk2_schtblid_str(ScheduleTableType id)$NL$
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
atk2_evtid_str(TaskType task, EventMaskType event)$NL$
{$NL$
$TAB$const char8	*evtid_str;$NL$
$IF LENGTH(TSK.ID_LIST)$
	$TAB$switch (task) {$NL$
	$FOREACH tskid TSK.ID_LIST$
$		//OSが生成したリスタートタスク以外のタスクIDを出力する
		$IF tskid < tmin_os_restarttask$
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
atk2_osapid_str(ApplicationType id)$NL$
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
atk2_iocid_str(IocType id)$NL$
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

$NL$
const char8 *$NL$
atk2_tfnid_str(TrustedFunctionIndexType id)$NL$
{$NL$
$IF LENGTH(TFN.ID_LIST)$
	$TAB$const char8	*tfnid_str;$NL$
	$TAB$switch (id) {$NL$
	$FOREACH tfnid TFN.ID_LIST$
	$TAB$case $tfnid$:$NL$
	$TAB$$TAB$tfnid_str = "$tfnid$";$NL$
	$TAB$$TAB$break;$NL$
	$END$
	$TAB$default:$NL$
	$TAB$$TAB$tfnid_str = "";$NL$
	$TAB$$TAB$break;$NL$
	$TAB$}$NL$
	$TAB$return(tfnid_str);$NL$
$ELSE$
	$TAB$return("");$NL$
$END$
}$NL$
#endif /* TOPPERS_ENABLE_TRACE */$NL$


$ =====================================================================
$ 以下はHRP2からの移植
$ =====================================================================

$
$  メモリオブジェクト管理機能
$

$
$  統合前のメモリオブジェクトの情報の加工
$
$ 統合前のメモリオブジェクトの情報に，以下の情報を追加ないしは修正を加
$ える．
$
$ MO.MEMATR[moid]：メモリオブジェクト属性
$	TA_NOWRITE属性のメモリリージョンに配置される場合は，TA_MEMINIと
$	TA_MEMPRSVをクリアする
$ MO.ACPTN1[moid]：通常操作1（書込み）のアクセス許可パターン
$	MO.MEMATR[moid]にTA_NOWRITEが設定されている場合は0にする．
$ MO.CLASS[moid]：メモリオブジェクトを配置するための分類指標（リンカが
$				  配置する場合のみ）
$	0：標準のメモリオブジェクト属性，ACPTN1とACPTN2が標準
$	  （タスクのユーザスタック領域もここに含める）
$	1：標準のメモリオブジェクト属性，ACPTN1またはACPTN2が標準でない
$	2：標準でないメモリオブジェクト属性
$ MO.SRPW[moid]：共有リード専有ライト領域か？（リンカが配置する場合のみ）
$ MEMATR_MASK：メモリオブジェクト属性が標準であるかを判定する時のマスク
$
$IF !LENGTH(MEMATR_MASK)$
	$MEMATR_MASK = ~TA_SDATA$
$END$
$FOREACH moid RANGE(1, nummo)$
	$domptn = DEFAULT_ACPTN[MO.OSAPID[moid]]$

	$IF (MO.MEMATR[moid] & TA_NOWRITE) != 0$
		$MO.ACPTN1[moid] = TACP_KERNEL$
	$END$

	$IF (MO.MEMATR[moid] & (TA_NOREAD | TA_EXEC)) == TA_NOREAD$
		$MO.ACPTN2[moid] = TACP_KERNEL$
	$END$

	$IF MO.MEMATR[moid] & TA_NOREAD$
		$MO.ACPTN_R[moid] = TACP_KERNEL$
	$ELSE$
		$MO.ACPTN_R[moid] = MO.ACPTN2[moid]$
	$END$

	$MO.ACPTN_W[moid] = MO.ACPTN1[moid]$

	$IF MO.MEMATR[moid] & TA_EXEC$
		$MO.ACPTN_X[moid] = MO.ACPTN2[moid]$
	$ELSE$
		$MO.ACPTN_X[moid] = TACP_KERNEL$
	$END$

	$IF MO.LINKER[moid]$
		$MO.SRPW[moid] = 0$
		$IF (REG.REGATR[MO.MEMREG[moid]] & TA_NOWRITE) != 0$
			$MO.MEMATR[moid] = MO.MEMATR[moid] & ~(TA_MEMINI|TA_MEMPRSV)$
			$IF (MO.MEMATR[moid] & TA_EXEC) != 0
						&& (MO.MEMATR[moid] & MEMATR_MASK) != MEMATR_TEXT
				|| (MO.MEMATR[moid] & TA_EXEC) == 0
						&& (MO.MEMATR[moid] & MEMATR_MASK) != MEMATR_RODATA$
				$MO.CLASS[moid] = 2$
			$ELIF MO.ACPTN2[moid] != domptn$
				$MO.CLASS[moid] = 1$
$				// ROMで共有リード（実行）の場合はドメインなしにする
$				// 次のIF文は必ず真なのでROMの場合はCLASS==1は存在しない
				$IF MO.ACPTN2[moid] == TACP_SHARED$
					$MO.OSAPID[moid] = TDOM_NONE$
					$MO.CLASS[moid] = 0$
				$END$
			$ELSE$
				$MO.CLASS[moid] = 0$
			$END$
		$ELSE$
			$IF MO.TYPE[moid] == TOPPERS_USTACK$
				$MO.CLASS[moid] = 0$
			$ELIF (MO.MEMATR[moid] & TA_MEMINI) != 0
						&& (MO.MEMATR[moid] & MEMATR_MASK) != MEMATR_DATA
				|| (MO.MEMATR[moid] & TA_MEMPRSV) != 0
						&& (MO.MEMATR[moid] & MEMATR_MASK) != MEMATR_PRSV
				|| (MO.MEMATR[moid] & (TA_MEMINI|TA_MEMPRSV)) == 0
						&& (MO.MEMATR[moid] & MEMATR_MASK) != MEMATR_BSS$
				$MO.CLASS[moid] = 2$

			$ELIF MO.ACPTN1[moid] != domptn$
$				// ここに来ることはない
				$MO.CLASS[moid] = 1$
			$ELIF MO.ACPTN2[moid] != domptn$
				$MO.CLASS[moid] = 1$
				$IF MO.ACPTN2[moid] == TACP_SHARED$
					$MO.SRPW[moid] = 1$
				$END$
			$ELSE$
				$MO.CLASS[moid] = 0$
			$END$
		$END$

$		// 書き込みできない領域はSRPWにしない
		$IF (MO.MEMATR[moid] & TA_NOWRITE) == 0$
			$IF MO.ACPTN1[moid] == domptn && MO.ACPTN2[moid] != domptn && MO.ACPTN2[moid] == TACP_SHARED$
				$MO.SRPW[moid] = 1$
			$END$
		$END$
	$END$
$END$

$
$  統合前のメモリオブジェクトをソートするための配列を作る
$
$ MO.ORDER[moid]：メモリオブジェクトをソートするための1次指標
$ MO.MEMATR1[moid]：ソートする際に，ACPTN1，ACPTN2より先に用いる属性
$ MO.MEMATR2[moid]：ソートする際に，ACPTN1，ACPTN2より後に用いる属性
$
$FOREACH moid RANGE(1, nummo)$
$	// まず，STANDARD_ROMを最初にし，その後はメモリリージョン番号の順に
$	// ソートする．
	$IF MO.LINKER[moid]$
		$IF MO.MEMREG[moid] == STANDARD_ROM$
			$memreg = 0$
		$ELSE$
			$memreg = MO.MEMREG[moid]$
		$END$
	$ELSE$
$		// リンカが配置しないものは，最後にする．
		$memreg = LENGTH(REG.ORDER_LIST) + 1$
	$END$
	$order = memreg$

$	// 次に，配置タイプによってソートする．
	$IF MO.TYPE[moid] == TOPPERS_USTACK$
$		// ユーザスタック領域の配置タイプの決定．
		$type = 1$
	$ELIF MO.ACPTN1[moid] == 0 && MO.ACPTN2[moid] == TACP_SHARED
									&& (MO.MEMATR[moid] & TA_SDATA) != 0$
$		// 共有リード・ライト不可のショートデータ領域の配置タイプの決定．
$		// カーネルドメインの共有リード専有ライト領域もここに含まれる．
		$type = 4$
	$ELIF MO.LINKER[moid] && MO.SRPW[moid]$
$		// 共有リード専有ライト領域の配置タイプの決定．
		$IF (MO.MEMATR[moid] & TA_SDATA) != 0$
			$type = 5$
		$ELSE$
			$type = 3$
		$END$
	$ELIF MO.OSAPID[moid] == TDOM_KERNEL$
$		// カーネルドメインの配置タイプの決定．
		$IF (MO.MEMATR[moid] & TA_SDATA) != 0$
			$type = 6$
		$ELSE$
			$type = 0$
		$END$
	$ELIF MO.OSAPID[moid] == TDOM_NONE$
$		// 無所属の配置タイプの決定．
		$IF MO.LINKER[moid] && MO.CLASS[moid] == 0$
			$IF (MO.MEMATR[moid] & TA_SDATA) != 0$
				$type = 7$
			$ELSE$
				$type = 8$
			$END$
		$ELSE$
			$IF (MO.MEMATR[moid] & TA_SDATA) != 0$
				$type = 6$
			$ELSE$
				$type = 2$
			$END$
		$END$
	$ELSE$
$		// ユーザドメインの配置タイプの決定．
		$IF (MO.MEMATR[moid] & TA_SDATA) != 0$
			$type = 6$
		$ELSE$
			$type = 2$
		$END$
	$END$
	$order = (order << 4) + type$

$	// 次に，保護ドメインによってソートする．
$	// ユーザドメインの数が32個以下であることを仮定している．
	$IF MO.OSAPID[moid] == TDOM_KERNEL$
		$domain = 0$
	$ELIF MO.OSAPID[moid] == TDOM_NONE$
		$domain = 63$
	$ELSE$	
		$domain = MO.OSAPID[moid]$
	$END$
	$order = (order << 6) + domain$

$	// 次に，メモリオブジェクトを配置するための分類指標によって配置する．
	$class = 0$
	$IF MO.LINKER[moid] && MO.TYPE[moid] != TOPPERS_USTACK$
		$IF (REG.REGATR[MO.MEMREG[moid]] & TA_NOWRITE) != 0$
			$IF MO.OSAPID[moid] == TDOM_NONE$
$				// TA_NOWRITE属性のメモリリージョンの無所属は，CLASSの逆順
$				// に配置する．
				$class = 2 - MO.CLASS[moid]$
			$ELSE$
$				// カーネルドメインとユーザドメインは，CLASSの順に配置する．
				$class = MO.CLASS[moid]$
			$END$
		$ELSE$
			$IF MO.OSAPID[moid] == TDOM_KERNEL || MO.OSAPID[moid] == TDOM_NONE$
$				// TA_NOWRITE属性でないメモリリージョンのカーネルドメイン
$				// と無所属は，CLASSの逆順に配置する．
				$class = 2 - MO.CLASS[moid]$
			$ELSE$
$				// ユーザドメインは，CLASSの順に配置する．
				$class = MO.CLASS[moid]$
			$END$
		$END$
	$END$
	$order = (order << 2) + class$

$	// 結果をMO.ORDER[moid]に格納する
	$MO.ORDER[moid] = order$

$	// MO.MEMATR1[moid]とMO.MEMATR2[moid]を設定する
	$IF MO.LINKER[moid]$
		$IF (REG.REGATR[MO.MEMREG[moid]] & TA_NOWRITE) != 0$
			$MO.MEMATR1[moid] = MO.MEMATR[moid] & ~TA_EXEC$
			$MO.MEMATR2[moid] = MO.MEMATR[moid] & TA_EXEC$
$			// TA_EXECが設定されている方を前に配置する
			$MO.MEMATR2[moid] = MO.MEMATR2[moid] ^ TA_EXEC$
		$ELSE$
			$MO.MEMATR1[moid] = MO.MEMATR[moid] & ~(TA_MEMINI|TA_MEMPRSV)$
			$MO.MEMATR2[moid] = MO.MEMATR[moid] & (TA_MEMINI|TA_MEMPRSV)$
$			// TA_MEMINIが設定されている方を前に配置する
			$MO.MEMATR2[moid] = MO.MEMATR2[moid] ^ TA_MEMINI$
		$END$
	$END$
$END$

$
$  メモリオブジェクトをソートするための比較関数
$
$IF !ISFUNCTION("MO_COMPARE")$
$FUNCTION MO_COMPARE$
$	まずは，MO.ORDERで比較する．
	$RESULT = MO.ORDER[ARGV[1]] - MO.ORDER[ARGV[2]]$
	$IF (RESULT == 0) && MO.LINKER[ARGV[1]]$
$		// MO.ORDERが同じで，リンカの配置対象の場合は，さらにソートする．
		$_moid1 = ARGV[1]$
		$_moid2 = ARGV[2]$
		$IF MO.TYPE[_moid1] == TOPPERS_USTACK$
$			// ユーザスタック領域の場合
$			// MO.STKORDERでソートする．
			$RESULT = MO.STKORDER[_moid1] - MO.STKORDER[_moid2]$
		$ELIF MO.MEMATR1[_moid1] != MO.MEMATR1[_moid2]$
$			// MO.MEMATR1が異なれば，それでソートする．
			$RESULT = MO.MEMATR1[_moid1] - MO.MEMATR1[_moid2]$
		$ELIF MO.ACPTN1[_moid1] != MO.ACPTN1[_moid2]$
$			// MO.ACPTN1が異なれば，それでソートする．
			$RESULT = MO.ACPTN1[_moid1] - MO.ACPTN1[_moid2]$
		$ELIF MO.ACPTN2[_moid1] != MO.ACPTN2[_moid2]$
$			// MO.ACPTN2が異なれば，それでソートする．
			$RESULT = MO.ACPTN2[_moid1] - MO.ACPTN2[_moid2]$
		$ELIF MO.MEMATR2[_moid1] != MO.MEMATR2[_moid2]$
$			// MO.MEMATR2が異なれば，それでソートする．
			$RESULT = MO.MEMATR2[_moid1] - MO.MEMATR2[_moid2]$
		$ELSE$
			$RESULT = 0$
		$END$
	$END$
$END$
$END$

$
$  メモリオブジェクトのソート
$
$ 最初にMO.ORDERでソートしておくことで，MO_COMPAREを使ったソートが効率
$ 化されることを期待している．
$
$MO_ORDER = SORT(RANGE(1, nummo), "MO.ORDER")$
$MO_ORDER = LSORT(MO_ORDER, "MO_COMPARE")$

$
$  メモリオブジェクトの統合処理
$
$ MO.SEFLAG[moid]：以下のビットのビット毎論理和に設定する．
$	0x01：セクションの先頭
$	0x02：セクションの最後
$	0x04：メモリオブジェクトの先頭
$	0x08：メモリオブジェクトの最後
$	0x10：メモリ保護単位の先頭
$	0x20：メモリ保護単位の最後
$	0x40：メモリリージョンの先頭
$	0x80：メモリリージョンの最後
$	0x100：共有リード専用ライト領域全体の先頭
$	0x200：共有リード専用ライト領域全体の最後
$	0x400：ショートデータセクションの先頭
$ MO.MOEND[moid]：統合後のメモリオブジェクトの最後のmoid（統合後の
$							メモリオブジェクトの先頭のmoidに対して設定）
$ MO_SECTION_LIST：セクションの先頭のリスト
$ MO_START_LIST：メモリオブジェクトの先頭のリスト
$ MO_START_LIST_NOLINKER：リンカが配置しないメモリオブジェクトの先頭のリスト
$ MO_MPROTECT_LIST：メモリ保護単位の先頭のリスト
$
$MO_SECTION_LIST = {}$
$MO_START_LIST = {}$
$MO_START_LIST_NOLINKER = {}$
$MO_MPROTECT_LIST = {}$
$i = 0$
$FOREACH moid MO_ORDER$
	$IF i == 0$
		$MO.SEFLAG[moid] = 0x15$
		$mostart = moid$
		$IF MO.LINKER[moid]$
			$MO.SEFLAG[moid] = MO.SEFLAG[moid] | 0x40$
		$END$
	$ELSE$
		$IF MO.ORDER[moid] != MO.ORDER[prev_moid]
				|| (MO.TYPE[moid] & TOPPERS_ATTSEC) == 0
				|| (MO.TYPE[prev_moid] & TOPPERS_ATTSEC) == 0
				|| (MO.MEMATR[moid] & ~(TA_MEMINI|TA_MEMPRSV))
						!= (MO.MEMATR[prev_moid] & ~(TA_MEMINI|TA_MEMPRSV))
				|| MO.ACPTN1[moid] != MO.ACPTN1[prev_moid]
				|| MO.ACPTN2[moid] != MO.ACPTN2[prev_moid]$
			$MO.SEFLAG[moid] = 0x15$
			$MO.SEFLAG[prev_moid] = MO.SEFLAG[prev_moid] | 0x2a$
			$MO.MOEND[mostart] = prev_moid$
			$mostart = moid$
		$ELIF MO.MEMATR[moid] != MO.MEMATR[prev_moid]$
			$MO.SEFLAG[moid] = 0x05$
			$MO.SEFLAG[prev_moid] = MO.SEFLAG[prev_moid] | 0x0a$
			$MO.MOEND[mostart] = prev_moid$
			$mostart = moid$
		$ELSE$
			$MO.SEFLAG[moid] = 0$
		$END$

		$IF MO.LINKER[prev_moid] && MO.LINKER[moid]
					&& MO.MEMREG[prev_moid] != MO.MEMREG[moid]$
			$MO.SEFLAG[moid] = MO.SEFLAG[moid] | 0x40$
			$MO.SEFLAG[prev_moid] = MO.SEFLAG[prev_moid] | 0x80$
		$ELIF MO.LINKER[prev_moid] && !MO.LINKER[moid]$
			$MO.SEFLAG[prev_moid] = MO.SEFLAG[prev_moid] | 0x80$
		$END$

		$IF !(MO.LINKER[prev_moid] && MO.SRPW[prev_moid])
					&& (MO.LINKER[moid] && MO.SRPW[moid])$
			$MO.SEFLAG[moid] = MO.SEFLAG[moid] | 0x100$
		$END$
		$IF (MO.LINKER[prev_moid] && MO.SRPW[prev_moid])
					&& !(MO.LINKER[moid] && MO.SRPW[moid])$
			$MO.SEFLAG[prev_moid] = MO.SEFLAG[prev_moid] | 0x200$
		$END$
		$IF !(MO.LINKER[prev_moid] && (MO.MEMATR[prev_moid] & TA_SDATA) != 0)
					&& (MO.LINKER[moid] && (MO.MEMATR[moid] & TA_SDATA) != 0)$
			$MO.SEFLAG[moid] = MO.SEFLAG[moid] | 0x400$
		$END$
	$END$

$	// MO_SECTION_LIST，MO_START_LIST，MO_MPROTECT_LISTへの追加
	$IF (MO.SEFLAG[moid] & 0x01) != 0$
		$MO_SECTION_LIST = APPEND(MO_SECTION_LIST, moid)$
	$END$
	$IF (MO.SEFLAG[moid] & 0x04) != 0$
		$MO_START_LIST = APPEND(MO_START_LIST, moid)$
		$IF (!MO.LINKER[moid])$
			$MO_START_LIST_NOLINKER = APPEND(MO_START_LIST_NOLINKER, moid)$
		$END$
	$END$
	$IF (MO.SEFLAG[moid] & 0x10) != 0$
		$MO_MPROTECT_LIST = APPEND(MO_MPROTECT_LIST, moid)$
	$END$

	$prev_moid = moid$
	$i = i + 1$
$END$
$MO.SEFLAG[prev_moid] = MO.SEFLAG[prev_moid] | 0x2a$
$MO.MOEND[mostart] = prev_moid$
$IF MO.LINKER[prev_moid]$
	$MO.SEFLAG[prev_moid] = MO.SEFLAG[prev_moid] | 0x80$
$END$
$IF MO.LINKER[prev_moid] && MO.SRPW[prev_moid]$
	$MO.SEFLAG[prev_moid] = MO.SEFLAG[prev_moid] | 0x200$
$END$

$
$  メモリオブジェクトのラベルの生成
$
$ MO.SLABEL[moid]：セクションのラベル
$ MO.ILABEL[moid]：初期化データ領域のラベル
$ MO.MLABEL[moid]：メモリオブジェクトのラベル
$ MO.PLABEL[moid]：メモリ保護単位のラベル
$ DATASEC_LIST：dataセクションのリスト
$ BSSSEC_LIST：bssセクションのリスト
$
$DATASEC_LIST = {}$
$BSSSEC_LIST = {}$
$FOREACH moid MO_ORDER$
	$IF MO.LINKER[moid]$
		$IF MO.MEMREG[moid] == STANDARD_ROM || MO.MEMREG[moid] == STANDARD_RAM$
			$reglabel = ""$
		$ELSE$
			$reglabel = FORMAT("%s_", REG.REGNAME[MO.MEMREG[moid]])$
		$END$
		$domlabel = OSAP.LABEL[MO.OSAPID[moid]]$

		$ilabel = ""$
		$plabel = ""$
		$IF (REG.REGATR[MO.MEMREG[moid]] & TA_NOWRITE) != 0$
			$IF (MO.MEMATR[moid] & TA_EXEC) != 0$
				$typelabel = "text"$
			$ELSE$
				$typelabel = "rodata"$
			$END$
			$acptn12 = FORMAT("%x", +MO.ACPTN2[moid])$
		$ELSE$
			$IF (MO.MEMATR[moid] & TA_MEMINI) != 0$
				$IF (MO.MEMATR[moid] & TA_SDATA) != 0$
					$typelabel = "sdata"$
				$ELSE$
					$typelabel = "data"$
				$END$
				$IF (MO.SEFLAG[moid] & 0x01) != 0$
					$DATASEC_LIST = APPEND(DATASEC_LIST, moid)$
				$END$
				$ilabel = FORMAT("%si%s_%s", reglabel, typelabel, domlabel)$
			$ELIF (MO.MEMATR[moid] & TA_MEMPRSV) != 0$
				$IF (MO.MEMATR[moid] & TA_SDATA) != 0$
					$typelabel = "sprsv"$
				$ELSE$
					$typelabel = "prsv"$
				$END$
			$ELSE$
				$IF (MO.MEMATR[moid] & TA_SDATA) != 0$
					$typelabel = "sbss"$
				$ELSE$
					$typelabel = "bss"$
				$END$
				$IF (MO.SEFLAG[moid] & 0x01) != 0$
					$BSSSEC_LIST = APPEND(BSSSEC_LIST, moid)$
				$END$
			$END$
			$acptn12 = FORMAT("%x_%x", +MO.ACPTN1[moid], +MO.ACPTN2[moid])$
			$IF (MO.MEMATR[moid] & TA_SDATA) != 0$
				$plabel = FORMAT("%ssram_%s", reglabel, domlabel)$
			$ELSE$
				$plabel = FORMAT("%sram_%s", reglabel, domlabel)$
			$END$
		$END$
		$label = FORMAT("%s%s_%s", reglabel, typelabel, domlabel)$
		$IF MO.TYPE[moid] == TOPPERS_USTACK$
			$MO.SLABEL[moid] = REGEX_REPLACE(MO.SECTION[moid], "\\.", "")$
			$MO.MLABEL[moid] = REGEX_REPLACE(MO.SECTION[moid], "\\.", "")$
			$MO.PLABEL[moid] = ""$
		$ELIF MO.CLASS[moid] == 0$
			$MO.SLABEL[moid] = label$
			$IF !EQ(ilabel, "")$
				$MO.ILABEL[moid] = ilabel$
			$END$
			$MO.MLABEL[moid] = FORMAT("%s__std", label)$
			$IF !EQ(plabel, "")$
				$MO.PLABEL[moid] = plabel$
			$END$
		$ELIF MO.CLASS[moid] == 1$
			$MO.SLABEL[moid] = FORMAT("%s_%s", label, acptn12)$
			$IF !EQ(ilabel, "")$
				$MO.ILABEL[moid] = FORMAT("%s_%s", ilabel, acptn12)$
			$END$
			$MO.MLABEL[moid] = FORMAT("%s__%s", label,	acptn12)$
			$IF !EQ(plabel, "")$
				$MO.PLABEL[moid] = FORMAT("%s_%s", plabel, acptn12)$
			$END$
		$ELSE$
			$MO.SLABEL[moid] = FORMAT("%s_%s_%x", label,
												acptn12, +MO.MEMATR[moid])$
			$IF !EQ(ilabel, "")$
				$MO.ILABEL[moid] = FORMAT("%s_%s_%x", ilabel,
												acptn12, +MO.MEMATR[moid])$
			$END$
			$MO.MLABEL[moid] = FORMAT("%s__%s_%x", label,
								acptn12, +MO.MEMATR[moid])$
			$IF !EQ(plabel, "")$
				$MO.PLABEL[moid] = FORMAT("%s_%s_%x", plabel,
						acptn12, MO.MEMATR[moid] & ~(TA_MEMINI|TA_MEMPRSV))$
			$END$
		$END$
	$END$
$END$

$		
$ =====================================================================
$ 仮のメモリ構成・初期化ファイルの生成
$ =====================================================================
$FILE "kernel_mem2.c"$
#include "kernel_int.h"$NL$
#include "Os_Lcfg.h"$NL$

#ifndef TOPPERS_EMPTY_LABEL$NL$
#define TOPPERS_EMPTY_LABEL(x, y) x y[0]$NL$
#endif$NL$

/*$NL$
$SPC$*  Include Directives (#include)$NL$
$SPC$*/$NL$
$NL$
$INCLUDES$
$NL$

$
$  仮メモリオブジェクト初期化ブロックの生成
$
$IF !OMIT_STANDARD_MEMINIB$

$	// アドレス0を置く領域
	$tsize_meminib = 1$
	$FOREACH moid MO_START_LIST$
$		// メモリオブジェクトの先頭番地を置く領域
		$tsize_meminib = tsize_meminib + 1$
		$IF !MO.LINKER[moid]$
$			// リンカが配置しないメモリオブジェクトは最終番地も必要
			$tsize_meminib = tsize_meminib + 1$
		$ELIF (MO.SEFLAG[MO.MOEND[moid]] & 0x80) != 0$
$			// メモリリージョンの最後のメモリオブジェクトは最終番地も必要
			$tsize_meminib = tsize_meminib + 1$
		$END$
	$END$

	const uint32 tnum_meminib = $tsize_meminib$U;$NL$
	$NL$

	void *const memtop_table[$tsize_meminib$] = {
	$IF LENGTH(MO_START_LIST_NOLINKER)$
		$NL$
		$JOINEACH moid MO_START_LIST_NOLINKER ",\n"$
			$TAB$(void *)($MO.BASE[moid]$)  /* $MO.ACPTN_R[moid]$, $MO.ACPTN_W[moid]$, $MO.ACPTN_X[moid]$ */
		$END$$NL$
	$ELSE$
		$SPC$0$SPC$
	$END$
	};$NL$
	$NL$

	const MEMINIB meminib_table[$tsize_meminib$] =
	$SPC${{ TA_NULL, 0U, 0U, 0U }};$NL$
	$NL$
$END$

$
$  dataセクション初期化ブロックの生成
$
$IF !OMIT_STANDARD_DATASECINIB$
	/*$NL$
	$SPC$*	Data Section Management Functions$NL$
	$SPC$*/$NL$
	$NL$

$	// dataセクションの数
	#define TNUM_DATASEC	$IF !OMIT_IDATA$$LENGTH(DATASEC_LIST)$$ELSE$0$END$$NL$
	$NL$

	$IF ISFUNCTION("DEFINE_CONST_VAR")$
		$DEFINE_CONST_VAR("const uint32", "tnum_datasec")$ = TNUM_DATASEC;$NL$
	$ELSE$
		const uint32 tnum_datasec = TNUM_DATASEC;$NL$
	$END$
	$NL$

$	// dataセクション初期化ブロック
	$IF !OMIT_IDATA && LENGTH(DATASEC_LIST)$
		$IF ISFUNCTION("DEFINE_CONST_VAR")$
			$DEFINE_CONST_VAR("const DATASECINIB", "datasecinib_table[TNUM_DATASEC]")$ =
		$ELSE$
			const DATASECINIB datasecinib_table[TNUM_DATASEC] =
		$END$
		$SPC${{ 0U, 0U, 0U }};$NL$
	$ELSE$
		TOPPERS_EMPTY_LABEL(const DATASECINIB, datasecinib_table);$NL$
		$NL$
	$END$$NL$
$END$

$
$  bssセクション初期化ブロックの生成
$
$IF !OMIT_STANDARD_BSSSECINIB$
	/*$NL$
	$SPC$*	BSS Section Management Functions$NL$
	$SPC$*/$NL$
	$NL$

$	// bssセクションの数
	#define TNUM_BSSSEC	$LENGTH(BSSSEC_LIST)$$NL$
	$NL$

	const uint32 tnum_bsssec = TNUM_BSSSEC;$NL$
	$NL$

$	// bssセクション初期化ブロック
	$IF LENGTH(BSSSEC_LIST)$
		const BSSSECINIB bsssecinib_table[TNUM_BSSSEC] = {{0U, 0U}};$NL$
	$ELSE$
		TOPPERS_EMPTY_LABEL(const BSSSECINIB, bsssecinib_table);$NL$
	$END$$NL$
$END$

$ =====================================================================
$ HRP2からの移植ここまで
$ =====================================================================

$ ターゲット依存部で必要なMPUINFOBを出力する
$GENERATE_TARGET_MPUINFOB()$

$ タスクスタック領域をextern宣言する(kernel_common.tf)
$GENERATE_EXPORT_TSK_STK()$

$ タスクをextern宣言する(kernel_common.tf)
$EXTERN_TSK()$

$ タスク管理ブロックをextern宣言する(kernel_common.tf)
$EXTERN_TCB()$

$ OSAP管理ブロックをextern宣言する(kernel_common.tf)
$EXTERN_OSAPCB()$

$ タスク初期化ブロックを出力する(kernel_common.tf)
$GENERATE_TINIB_TABLE()$

$ OSアプリケーション初期化ブロックを出力する(kernel_common.tf)
$GENERATE_OSAPINIB_TABLE()$

$ =====================================================================
$  パス3, パス4に渡す情報の生成
$ =====================================================================

$FILE "cfg2_out.tf"$
$$ cfg2_out.tf$NL$
$NL$

$ STANDARD_ROM，STANDARD_RAMの出力
$$STANDARD_ROM = $STANDARD_ROM$$$$NL$
$$STANDARD_RAM = $STANDARD_RAM$$$$NL$
$NL$

$ REG_ORDERの出力
$$REG_ORDER = { $REG_ORDER$ }$$$NL$
$NL$

$ REG.*の出力
$FOREACH reg REG.ORDER_LIST$
	$$REG.REGNAME[$reg$] = $REG.REGION[reg]$$$$NL$
	$$REG.REGATR[$reg$] = VALUE($ESCSTR(REG.REGATR[reg])$,
									$SPC$$+REG.REGATR[reg]$)$$$NL$
	$$REG.BASE[$reg$] = VALUE($ESCSTR(REG.BASE[reg])$,
									$SPC$$+REG.BASE[reg]$)$$$NL$
	$$REG.SIZE[$reg$] = VALUE($ESCSTR(REG.SIZE[reg])$,
									$SPC$$+REG.SIZE[reg]$)$$$NL$
	$NL$
$END$

$ MO_ORDER，MO_SECTION_LIST，MO_START_LIST，MO_START_LIST_NOLINKER，
$ MO_MPROTECT_LISTの出力
$$MO_ORDER = { $MO_ORDER$ }$$$NL$
$NL$
$$MO_SECTION_LIST = { $MO_SECTION_LIST$ }$$$NL$
$NL$
$$MO_START_LIST = { $MO_START_LIST$ }$$$NL$
$NL$
$$MO_START_LIST_NOLINKER = { $MO_START_LIST_NOLINKER$ }$$$NL$
$NL$
$$MO_MPROTECT_LIST = { $MO_MPROTECT_LIST$ }$$$NL$
$NL$

$ tsize_meminibの出力
$IF !OMIT_STANDARD_MEMINIB$
	$$tsize_meminib = $tsize_meminib$$$$NL$
	$NL$
$END$

$ MO_SECTION_LIST，DATASEC_LIST，BSSSEC_LISTの出力
$$MO_SECTION_LIST = { $MO_SECTION_LIST$ }$$$NL$
$NL$
$$DATASEC_LIST = { $DATASEC_LIST$ }$$$NL$
$NL$
$$BSSSEC_LIST = { $BSSSEC_LIST$ }$$$NL$
$NL$

$ MO.*の出力
$FOREACH moid MO_ORDER$
	$$MO.TYPE[$moid$] = VALUE($ESCSTR(MO.TYPE[moid])$,
									$SPC$$+MO.TYPE[moid]$)$$$NL$
	$IF MO.TYPE[moid] == TOPPERS_ATTMOD$
		$$MO.MODULE[$moid$] = $ESCSTR(MO.MODULE[moid])$$$$NL$
	$ELIF MO.TYPE[moid] == TOPPERS_ATTMEM$
		$$MO.BASE[$moid$] = $ESCSTR(MO.BASE[moid])$$$$NL$
		$$MO.SIZE[$moid$] = VALUE($ESCSTR(MO.SIZE[moid])$,
									$SPC$$+MO.SIZE[moid]$)$$$NL$
		$IF LENGTH(MO.PADDR[moid])$
			$$MO.PADDR[$moid$] = VALUE($ESCSTR(MO.PADDR[moid])$,
									$SPC$$+MO.PADDR[moid]$)$$$NL$
		$END$
	$ELIF MO.TYPE[moid] == TOPPERS_USTACK$
		$$MO.TSKID[$moid$] = VALUE($ESCSTR(MO.TSKID[moid])$,
									$SPC$$+MO.TSKID[moid]$)$$$NL$
		$IF !MO.LINKER[moid]$
			$$MO.BASE[$moid$] = $ESCSTR(MO.BASE[moid])$$$$NL$
			$$MO.SIZE[$moid$] = VALUE($ESCSTR(MO.SIZE[moid])$,
									$SPC$$+MO.SIZE[moid]$)$$$NL$
		$ELSE$
			$$MO.SIZE[$moid$] = VALUE($ESCSTR(MO.SIZE[moid])$,
									$SPC$$+MO.SIZE[moid]$)$$$NL$
		$END$
		$IF LENGTH(MO.STKORDER[moid])$
			$$MO.STKORDER[$moid$] = $+MO.STKORDER[moid]$$$$NL$
		$END$
	$END$
	$$MO.LINKER[$moid$] = $MO.LINKER[moid]$$$$NL$
	$$MO.OSAPID[$moid$] = VALUE($ESCSTR(MO.OSAPID[moid])$,
									$SPC$$+MO.OSAPID[moid]$)$$$NL$
	$IF MO.LINKER[moid]$
		$$MO.MEMREG[$moid$] = $MO.MEMREG[moid]$$$$NL$
		$$MO.SECTION[$moid$] = $ESCSTR(MO.SECTION[moid])$$$$NL$
		$$MO.CLASS[$moid$] = $MO.CLASS[moid]$$$$NL$
		$$MO.SRPW[$moid$] = $MO.SRPW[moid]$$$$NL$
		$$MO.MEMATR1[$moid$] = $MO.MEMATR1[moid]$$$$NL$
		$$MO.MEMATR2[$moid$] = $MO.MEMATR2[moid]$$$$NL$
	$END$
	$$MO.MEMATR[$moid$] = VALUE($ESCSTR(MO.MEMATR[moid])$,
									$SPC$$+MO.MEMATR[moid]$)$$$NL$
	$$MO.ACPTN1[$moid$] = VALUE($ESCSTR(MO.ACPTN1[moid])$,
									$SPC$$+MO.ACPTN1[moid]$)$$$NL$
	$$MO.ACPTN2[$moid$] = VALUE($ESCSTR(MO.ACPTN2[moid])$,
									$SPC$$+MO.ACPTN2[moid]$)$$$NL$
	$$MO.ACPTN_R[$moid$] = VALUE($ESCSTR(MO.ACPTN_R[moid])$,
									$SPC$$+MO.ACPTN_R[moid]$)$$$NL$
	$$MO.ACPTN_W[$moid$] = VALUE($ESCSTR(MO.ACPTN_W[moid])$,
									$SPC$$+MO.ACPTN_W[moid]$)$$$NL$
	$$MO.ACPTN_X[$moid$] = VALUE($ESCSTR(MO.ACPTN_X[moid])$,
									$SPC$$+MO.ACPTN_X[moid]$)$$$NL$
	$IF LENGTH(MO.TEXT_LINE[moid])$
		$$MO.TEXT_LINE[$moid$] = VALUE($ESCSTR(MO.TEXT_LINE[moid])$,
									$SPC$$+MO.TEXT_LINE[moid]$)$$$NL$
	$END$
	$IF LENGTH(MO.ILABEL[moid])$
		$$MO.ILABEL[$moid$] = $ESCSTR(MO.ILABEL[moid])$$$$NL$
	$END$
	$$MO.ORDER[$moid$] = $MO.ORDER[moid]$$$$NL$
	$$MO.SEFLAG[$moid$] = 0x$FORMAT("%x", +MO.SEFLAG[moid])$$$$NL$
	$IF LENGTH(MO.MOEND[moid])$
		$$MO.MOEND[$moid$] = $MO.MOEND[moid]$$$$NL$
	$END$
	$$MO.SLABEL[$moid$] = $ESCSTR(MO.SLABEL[moid])$$$$NL$
	$$MO.ILABEL[$moid$] = $ESCSTR(MO.ILABEL[moid])$$$$NL$
	$$MO.MLABEL[$moid$] = $ESCSTR(MO.MLABEL[moid])$$$$NL$
	$$MO.PLABEL[$moid$] = $ESCSTR(MO.PLABEL[moid])$$$$NL$
	$NL$
$END$

$$OSAP.ID_LIST = {
$JOINEACH osapid OSAP.ID_LIST ","$
	VALUE("$osapid$", $+osapid$)
$END$
}$$$NL$
$NL$
$FOREACH osapid OSAP.ID_LIST$
	$$OSAP.BTMP[$+osapid$] = $OSAP.BTMP[osapid]$$$$NL$
	$$OSAP.RESTARTTASK[$+osapid$] = VALUE($ESCSTR(OSAP.RESTARTTASK[osapid])$, $+OSAP.RESTARTTASK[osapid]$)$$$NL$
	$$DEFAULT_ACPTN[$+osapid$] = VALUE($ESCSTR(DEFAULT_ACPTN[osapid])$,
									$SPC$$+DEFAULT_ACPTN[osapid]$)$$$NL$
$END$
$NL$
$$DEFAULT_ACPTN[TDOM_KERNEL] = $DEFAULT_ACPTN[TDOM_KERNEL]$$$$NL$
$$DEFAULT_ACPTN[TDOM_NONE] = $DEFAULT_ACPTN[TDOM_NONE]$$$$NL$
$NL$

$$TSK.ID_LIST = {
$JOINEACH tskid TSK.ID_LIST ","$
	VALUE("$tskid$", $+tskid$)
$END$
}$$$NL$
$NL$
$FOREACH tskid TSK.ID_LIST$
	$$TSK.OSAPID[$+tskid$] = VALUE("$TSK.OSAPID[tskid]$", $+TSK.OSAPID[tskid]$)$$$NL$
	$$TSK.ACSBTMP[$+tskid$] = $TSK.ACSBTMP[tskid]$$$$NL$
	$$TSK.ASTPTN[$+tskid$] = $TSK.ASTPTN[tskid]$$$$NL$
	$IF LENGTH(TSK.INRESPRI[tskid])$
		$$TSK.INRESPRI[$+tskid$] = $TSK.INRESPRI[tskid]$$$$NL$
	$END$
	$$TSK.TINIB_USTKSZ[$+tskid$] = $ESCSTR(TSK.TINIB_USTKSZ[tskid])$$$$NL$
	$$TSK.TINIB_USTK  [$+tskid$] = $ESCSTR(TSK.TINIB_USTK  [tskid])$$$$NL$
	$$TSK.TINIB_SSTKSZ[$+tskid$] = $ESCSTR(TSK.TINIB_SSTKSZ[tskid])$$$$NL$
	$$TSK.TINIB_SSTK  [$+tskid$] = $ESCSTR(TSK.TINIB_SSTK  [tskid])$$$$NL$
	$IF LENGTH(TSK.SHARED_USTK_ID[tskid])$
		$$TSK.SHARED_USTK_ID[$+tskid$] = $TSK.SHARED_USTK_ID[tskid]$$$$NL$
	$END$
	$IF LENGTH(TSK.SHARED_SSTK_ID[tskid])$
		$$TSK.SHARED_SSTK_ID[$+tskid$] = $TSK.SHARED_SSTK_ID[tskid]$$$$NL$
	$END$
	$IF LENGTH(TSK.USTACK_MO[tskid])$
		$$TSK.USTACK_MO[$+tskid$]    = VALUE($ESCSTR(TSK.USTACK_MO[tskid])$,
											$SPC$$+TSK.USTACK_MO[tskid]$)$$$NL$
	$END$
$	//OS指定リスタートタスクの情報のうち，パス3,パス4でで使用するものを出力する
	$$TSK.PRIORITY[$+tskid$] = $TSK.PRIORITY[tskid]$$$$NL$
	$$TSK.ACTIVATION[$+tskid$] = VALUE($ESCSTR(TSK.ACTIVATION[tskid])$,$TSK.ACTIVATION[tskid]$)$$$NL$
	$$TSK.RESTARTTASK[$+tskid$] = $ESCSTR(TSK.RESTARTTASK[tskid])$$$$NL$
	$NL$
$END$

$$tnum_os_restarttask = VALUE($ESCSTR(tnum_os_restarttask)$, $+tnum_os_restarttask$)$$$NL$
$$tmin_os_restarttask = VALUE($ESCSTR(tmin_os_restarttask)$, $+tmin_os_restarttask$)$$$NL$
$NL$

$FOREACH tskpri RANGE(TMIN_TPRI, TMAX_TPRI)$
	$IF LENGTH(shared_ustack_size[tskpri])$
		$$shared_ustack_size[$tskpri$] = $ESCSTR(shared_ustack_size[tskpri])$$$$NL$
	$END$
	$IF LENGTH(shared_sstack_size[tskpri])$
		$$shared_sstack_size[$tskpri$] = $ESCSTR(shared_sstack_size[tskpri])$$$$NL$
	$END$
$END$

$NL$
$$HWCNT.ID_LIST = {
$JOINEACH hwcntid HWCNT.ID_LIST ","$
	VALUE("$hwcntid$", $+hwcntid$)
$END$
}$$$NL$

$NL$

$ LNKSEC.*の出力
$$numls = $numls$$$$NL$
$NL$
$FOREACH lsid RANGE(1, numls)$
	$$LNKSEC.MEMREG[$lsid$] = $LNKSEC.MEMREG[lsid]$$$$NL$
	$$LNKSEC.SECTION[$lsid$] = $ESCSTR(LNKSEC.SECTION[lsid])$$$$NL$
	$NL$
$END$
