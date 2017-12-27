$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2012-2015 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
$  Copyright (C) 2012-2015 by FUJI SOFT INCORPORATED, JAPAN
$  Copyright (C) 2012-2013 by Spansion LLC, USA
$  Copyright (C) 2012-2015 by NEC Communication Systems, Ltd., JAPAN
$  Copyright (C) 2012-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
$  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
$  Copyright (C) 2012-2015 by Sunny Giken Inc., JAPAN
$  Copyright (C) 2012-2015 by TOSHIBA CORPORATION, JAPAN
$  Copyright (C) 2012-2015 by Witz Corporation
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
$  $Id: ioc.tf 630 2016-03-17 07:42:55Z ertl-ishikawa $
$

$ =====================================================================
$ IMPLEMENTATION_DATA_TYPE.CATEGORYの値を返す関数
$ ARGV[1] = IMPLEMENTATION_DATA_TYPE.SHORT_NAME_PATH
$ =====================================================================

$FUNCTION GET_CATEGORY$
	$datatypeid = 0$
	$datatypecategory = "NULL"$

	$FOREACH shortnamepath  IMPLEMENTATION_DATA_TYPE.SHORT_NAME_PATH.ID_LIST$
		$IF EQ(shortnamepath , ARGV[1]) && LENGTH(IMPLEMENTATION_DATA_TYPE.CATEGORY[datatypeid])$
			$datatypecategory = IMPLEMENTATION_DATA_TYPE.CATEGORY[datatypeid]$
		$END$
		$datatypeid = datatypeid + 1$
	$END$
	
$	// 一致するCATEGORYが存在しないとエラー
	$IF EQ(datatypecategory, "NULL")$
		$ERROR ARGV[2]$$FORMAT(_("CATEGORY(%1%) is not defined"), ARGV[1])$$END$
	$END$
	
	$RESULT = datatypecategory$
$END$

$ =====================================================================
$ IMPLEMENTATION_DATA_TYPE.SHORT_NAMEの値を返す関数
$ ARGV[1] = IMPLEMENTATION_DATA_TYPE.SHORT_NAME_PATH
$ =====================================================================

$FUNCTION GET_SHORT_NAME$
	$datatypeid = 0$
	$datatypeshortname = "NULL"$

	$FOREACH shortnamepath  IMPLEMENTATION_DATA_TYPE.SHORT_NAME_PATH.ID_LIST$
		$IF EQ(shortnamepath , ARGV[1])$
			$datatypeshortname = IMPLEMENTATION_DATA_TYPE.SHORT_NAME[datatypeid]$
		$END$
		$datatypeid = datatypeid + 1$
	$END$
	
$	// 一致するIMPLEMENTATION_DATA_TYPEが存在しないとエラー
	$IF EQ(datatypeshortname, "NULL")$
		$ERROR ARGV[2]$$FORMAT(_("%1% is not defined"), ARGV[1])$$END$
	$END$

	$RESULT = datatypeshortname$
$END$

$ =====================================================================
$ IOCのエラーチェックと内部データの生成
$ =====================================================================
$TRACE("IOC CHECK")$

$
$ IOCのOS内部で用いるIDを割り当てる
$ キューあり通信からIDを割り当てる
$
$i = 0$
$tnum_queioc = 0$
$new_id_list = {}$
$FOREACH iocid IOC.ID_LIST$
	$IF !EQ(IOC.BUFLEN[iocid] ,"NULL")$
		$IOC.ID[iocid] = i$
		$i = i + 1$
		$tnum_queioc = tnum_queioc + 1$
		$new_id_list = APPEND(new_id_list, iocid)$
	$END$
$END$
$FOREACH iocid IOC.ID_LIST$
	$IF EQ(IOC.BUFLEN[iocid] ,"NULL")$
		$IOC.ID[iocid] = i$
		$i = i + 1$
		$new_id_list = APPEND(new_id_list, iocid)$
	$END$
$END$
$IOC.ID_LIST = new_id_list$

$ 
$ IOCとIOCDATAの関連付け
$ 
$ IOCDATA.IOCからIOC.IOCDATA_LISTの作成
$FOREACH iocdataid IOCDATA.ID_LIST$
	$iocid                   = IOCDATA.IOC[iocdataid]$
	$IOC.IOCDATA_LIST[iocid] = APPEND(IOC.IOCDATA_LIST[iocid], VALUE(IOCDATA.ID_LIST[iocdataid], iocdataid))$
$END$

$ 
$ IOCとIOCSNDの関連付け
$ 
$ IOCSND.IOCからIOC.IOCSND_LISTの作成
$FOREACH iocsndid IOCSND.ID_LIST$
	$iocid                  = IOCSND.IOC[iocsndid]$
	$IOC.IOCSND_LIST[iocid] = APPEND(IOC.IOCSND_LIST[iocid], VALUE(IOCSND.ID_LIST[iocsndid], iocsndid))$
$END$


$ 
$ IOCとIOCRCVの関連付け
$ 
$ IOCRCV.IOCからIOC.IOCRCV_LISTの作成
$FOREACH iocrcvid IOCRCV.ID_LIST$
	$iocid             = IOCRCV.IOC[iocrcvid]$
	$IOC.IOCRCV[iocid] = APPEND(IOC.IOCRCV[iocid], VALUE(IOCRCV.ID_LIST[iocrcvid], iocrcvid))$
$END$


$ 
$ エラーチェック
$ 
$FOREACH iocid IOC.ID_LIST$
$ 	// buflenに0を指定していないか
	$IF IOC.BUFLEN[iocid] == 0 && !EQ(IOC.BUFLEN[iocid], "NULL")$
		$ERROR IOC.TEXT_LINE[iocid]$$FORMAT(_("illegal %1% `%2%\'. Do not specify `0\'" ), "buflen", IOC.BUFLEN[iocid])$$END$
	$END$

$ 	// グループ通信の場合，indexで0が定義されていか．また，indexが省略されていないか
	$IF LENGTH(IOC.IOCDATA_LIST[iocid]) > 1$
		$FOREACH iocdataid IOC.IOCDATA_LIST[iocid]$
			$IF EQ(IOCDATA.INDEX[iocdataid], "NULL")$
				$ERROR IOCDATA.TEXT_LINE[iocdataid]$$FORMAT(_("illegal %1% `%2%\' if Group communication"), "index", IOCDATA.INDEX[iocdataid])$$END$
			$END$
		$END$
	$END$

$ 	// 1:1通信の場合，OsIocSenderPropertiesのsenderidがNULL以外ではないか
	$IF LENGTH(IOC.IOCSND_LIST[iocid]) == 1$
		$FOREACH iocsndid IOC.IOCSND_LIST[iocid]$
			$IF !EQ(IOCSND.SENDERID[iocsndid], "NULL")$
				$ERROR IOCSND.TEXT_LINE[iocsndid]$$FORMAT(_("illegal %1% `%2%\' if 1:1 communication"), "senderid", IOCSND.SENDERID[iocsndid])$$END$
			$END$
		$END$
	$END$

$ 	// N:1通信の場合，OsIocSenderPropertiesのsenderidが省略されていないか
	$IF LENGTH(IOC.IOCSND_LIST[iocid]) > 1$
		$FOREACH iocsndid IOC.IOCSND_LIST[iocid]$
			$IF EQ(IOCSND.SENDERID[iocsndid], "NULL")$
				$ERROR IOCSND.TEXT_LINE[iocsndid]$$FORMAT(_("illegal %1% `%2%\' if N:1 communication"), "senderid", IOCSND.SENDERID[iocsndid])$$END$
			$END$
		$END$
	$END$

$ 	// キュー無し通信の場合，初期化値を指定しているか
	$IF IOC.ID[iocid] >= tnum_queioc$
		$IF EQ(IOCDATA.INITVAL[IOC.IOCDATA_LIST[iocid]], "NULL")$
			$ERROR IOC.TEXT_LINE[iocid]$$FORMAT(_("illegal %1% `%2%\' in case of using unqueued" ), "initval", IOCDATA.INITVAL[IOC.IOCDATA_LIST[iocid]])$$END$
		$END$
	$END$

$	//1:1通信の場合，センダとレシーバが同一OSAPに所属していないか
	$iocsnd = IOC.IOCSND_LIST[iocid]$
	$IF LENGTH(IOC.IOCSND_LIST[iocid]) == 1$
		$IF EQ(IOCSND.OSAPID[iocsnd], IOCRCV.OSAPID[IOC.IOCRCV[iocid]])$
			$ERROR IOC.TEXT_LINE[iocsndid]$$FORMAT(_("Sender and receiver are refferenced the same osap `%1%`"), IOCSND.OSAPID[iocsnd])$$END$
		$END$
	$END$
$END$

$FOREACH iocsndid IOC.IOCSND_LIST$
$ 	// FUNCTION, MACRO, DO_NOT_CARE, 省略(NULL)以外はエラー
	$IF !EQ(IOCSND.IMPLKIND[iocsndid], "FUNCTION")
	 && !EQ(IOCSND.IMPLKIND[iocsndid], "DO_NOT_CARE")
	 && !EQ(IOCSND.IMPLKIND[iocsndid], "NULL")
	 && !EQ(IOCSND.IMPLKIND[iocsndid], "MACRO")$
		$ERROR IOCSND.TEXT_LINE[iocsndid]$$FORMAT(_("illegal %1% `%2%\'. Do not specify anything other than `FUNCTION\', `MACRO\', `DO_NOT_CARE\' and omission"), "implkind", IOCSND.IMPLKIND[iocsnd])$$END$
	$END$
$END$


$ 
$ グループ通信となるIOC.IOCDATA_LISTをIOCDATA.INDEXの順にソート
$ 
$FOREACH iocid IOC.ID_LIST$
	$IF LENGTH(IOC.IOCDATA_LIST[iocid]) > 1$
		$find = 0$
		$new_list = {}$
		$FOREACH index RANGE(0, LENGTH(IOC.IOCDATA_LIST[iocid])-1)$
			$FOREACH iocdataid IOC.IOCDATA_LIST[iocid]$
				$IF IOCDATA.INDEX[iocdataid] == index$
					$new_list = APPEND(new_list, iocdataid)$
					$find = find + 1$
				$END$
			$END$
			$IF find == 0$
				$ERROR IOCDATA.TEXT_LINE[iocdataid]$$FORMAT(_("OsIocDataPropertyIndex `%1%\' is missing for `%2%\'. These should be sequential."), index, IOCDATA.IOC[iocdataid])$$END$
			$END$
			$IF find > 1$
				$ERROR IOCDATA.TEXT_LINE[iocdataid]$$FORMAT(_("OsIocDataPropertyIndex `%1%\' is duplicated for `%2%\'. These should be unique."), index, IOCDATA.IOC[iocdataid])$$END$
			$END$
			$find = 0$
		$END$
		$IOC.IOCDATA_LIST[iocid] = new_list$
	$END$
$END$


$ =====================================================================
$ Os_Lcfg.hの生成
$ =====================================================================

$FILE "Os_Lcfg.h"$

/****** Object IOC ******/$NL$
$IF LENGTH(IOC.ID_LIST)$
$ 	// IOC IDの作成
	/* IOC ID */$NL$
	$FOREACH iocid IOC.ID_LIST$
		#define $iocid$$TAB$UINT_C($IOC.ID[iocid]$)$NL$
	$END$
	$NL$

$ 	// Wrapper IDの作成．IOCSNDのオブジェクト識別IDを使用する
	/* Wrapper ID */$NL$
	$FOREACH wrapperid RANGE(0, LENGTH(IOCSND.ID_LIST) - 1)$
		#define IOC_WRAPPER_$wrapperid$$TAB$UINT_C($wrapperid$)$NL$
	$END$
	$NL$
$END$

	#define TNUM_IOC			UINT_C($LENGTH(IOC.ID_LIST)$)$NL$
	#define TNUM_QUEUEIOC		UINT_C($tnum_queioc$)$NL$
	#define TNUM_IOC_WRAPPER	UINT_C($LENGTH(IOCSND.ID_LIST)$)$NL$


$ =====================================================================
$ Os_Lcfg.cの生成
$ =====================================================================
$FILE "Os_Lcfg.c"$

	/****** Object IOC ******/$NL$
	const IocType	tnum_ioc = TNUM_IOC;$NL$
	const IocType	tnum_queueioc = TNUM_QUEUEIOC;$NL$
	$NL$

$IF LENGTH(IOC.ID_LIST)$
$ 	// IOC初期化値
	/*$NL$
	$SPC$*  IOC Initialize Value$NL$
	$SPC$*/$NL$
	$FOREACH iocid IOC.ID_LIST$
		/* $iocid$ */$NL$
		$IF IOC.ID[iocid] < tnum_queioc$
			/* No initialize value because queue communication */$NL$
		$ELSE$
			$IF EQ(IOCDATA.INITVAL[IOC.IOCDATA_LIST[iocid]], "NULL")$
$ 				// ここには来ない（エラーチェック処理で検出済み）
				/* No initialize value */$NL$
			$ELSE$
				IOCMB_$iocid$ ioc_inival_$iocid$ = {$NL$
				$JOINEACH iocdata IOC.IOCDATA_LIST[iocid] ",\n"$
$					//OsIocInitValueは文字列型で読み込むため，arxmlでUを記載する方針とする
					$TAB$$IOCDATA.INITVAL[iocdata]$
				$END$
				$NL$};$NL$
			$END$
		$END$
		$NL$
	$END$
	$NL$

$ 	// IOC初期化値テーブル
	void *ioc_inival_table[TNUM_IOC] = {$NL$
	$JOINEACH iocid IOC.ID_LIST ",\n"$
		$IF IOC.ID[iocid] < tnum_queioc$
			$TAB$NULL
		$ELSE$
			$IF EQ(IOCDATA.INITVAL[IOC.IOCDATA_LIST[iocid]], "NULL")$
				$TAB$NULL
			$ELSE$
				$TAB$(void *)&ioc_inival_$iocid$
			$END$
		$END$
	$END$
	$NL$};$NL$
	$NL$

$ 	// IOC管理ブロックテーブル
	$IF tnum_queioc > 0$
		IOCCB ioccb_table[TNUM_QUEUEIOC];$NL$
	$ELSE$
		TOPPERS_EMPTY_LABEL(IOCCB, ioccb_table);$NL$
	$END$
	$NL$

$ 	// IOCデータ管理ブロックテーブルの宣言
	$FOREACH iocid IOC.ID_LIST$
		$IF EQ(IOC.BUFLEN[iocid], "NULL")$
			IOCMB_$iocid$ iocmb_table_$iocid$[1];$NL$
		$ELSE$
			IOCMB_$iocid$ iocmb_table_$iocid$[$IOC.BUFLEN[iocid]$];$NL$
		$END$
	$END$
	$NL$

$ 	// IOC初期化テーブル
	const IOCINIB iocinib_table[TNUM_IOC] = {$NL$
	$JOINEACH iocid IOC.ID_LIST ",\n"$
		$TAB${$NL$
			$TAB$$TAB$$+IOC.BUFLEN[iocid]$U,$NL$
			$TAB$$TAB$sizeof(IOCMB_$iocid$),$NL$
			$TAB$$TAB$alignof(IOCMB_$iocid$),$NL$
			#ifdef CFG_USE_ERRORHOOK$NL$
			$IF LENGTH(IOC.IOCDATA_LIST[iocid]) != 1$
				$TAB$$TAB$TRUE,$NL$
			$ELSE$
				$TAB$$TAB$FALSE,$NL$
			$END$
			#endif /* CFG_USE_ERRORHOOK */$NL$
			$TAB$$TAB$&osapcb_table[$IOCRCV.OSAPID[IOC.IOCRCV[iocid]]$],$NL$
			$TAB$$TAB$&iocmb_table_$iocid$$NL$
		$TAB$}
	$END$
	$NL$};$NL$
	$NL$

$ 	// IOCラッパー初期化テーブル
	const IOCWRPINIB iocwrpinib_table[TNUM_IOC_WRAPPER] = {$NL$
	$JOINEACH iocsndid IOCSND.ID_LIST ",\n"$
		$TAB${$NL$
			$TAB$$TAB$&iocinib_table[$IOCSND.IOC[iocsndid]$],$NL$
			$TAB$$TAB$&osapcb_table[$IOCSND.OSAPID[iocsndid]$],$NL$
			$IF EQ(IOCSND.SENDERID[iocsndid], "NULL")$
				$TAB$0U$NL$
			$ELSE$
				$TAB$$TAB$$IOCSND.SENDERID[iocsndid]$U$NL$
			$END$
		$TAB$}
	$END$
	$NL$};$NL$
	$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(void *, ioc_inival_table);$NL$
	TOPPERS_EMPTY_LABEL(IOCCB, ioccb_table);$NL$
	TOPPERS_EMPTY_LABEL(const IOCINIB, iocinib_table);$NL$
	TOPPERS_EMPTY_LABEL(const IOCWRPINIB, iocwrpinib_table);$NL$

$END$


$ =====================================================================
$ Ioc.hの生成
$ =====================================================================
$FILE "Ioc.h"$
#ifndef TOPPERS_IOC_H$NL$
#define TOPPERS_IOC_H$NL$
$NL$

#include "Os.h"$NL$
#include "Os_Lcfg.h"$NL$
$NL$

$IF LENGTH(IOC.ID_LIST)$
	/* IOC data management block */$NL$
	$FOREACH iocid IOC.ID_LIST$
		typedef struct ioc_data_management_block_$iocid$ {$NL$
		$arg = 1$
		$FOREACH iocdataid IOC.IOCDATA_LIST[iocid]$
			$TAB$$GET_SHORT_NAME(IOCDATA.DATATYPE[iocdataid])$$TAB$data$arg$;$NL$
			$arg = arg + 1$
		$END$
		}IOCMB_$iocid$;$NL$
		$NL$
	$END$
$ELSE$
	/* IOC is not used */
$END$


$ =====================================================================
$ Ioc.cの生成
$ =====================================================================
$FILE "Ioc.c"$

$IF LENGTH(IOC.ID_LIST)$
	#include "Ioc.h"$NL$
$ELSE$
	/* IOC is not used */
$END$

$ =====================================================================
$ APIの生成
$ =====================================================================
$IF LENGTH(IOC.ID_LIST)$
	$FOREACH iocid IOC.ID_LIST$
		$FILE "Ioc.h"$
		$NL$
		/* $iocid$ API */
		$FILE "Ioc.c"$
		$NL$
		
		$shortname = GET_SHORT_NAME(IOCDATA.DATATYPE[IOC.IOCDATA_LIST[iocid]], iocid)$
		$category = GET_CATEGORY(IOCDATA.DATATYPE[IOC.IOCDATA_LIST[iocid]], iocid)$
		/* $iocid$ API */
$ 		// 送信API
$		// 1:1通信
		$IF LENGTH(IOC.IOCSND_LIST[iocid]) == 1$
$ 			// 1:1，単一データ通信
			$IF LENGTH(IOC.IOCDATA_LIST[iocid]) == 1$
$				// 1:1，単一データ通信，キュー無し
				$IF EQ(IOC.BUFLEN[iocid], "NULL")$
					$iocsender = "IocWrite"$
					$iocsender_generic = "ioc_write_generic"$
$				// 1:1，単一データ通信，キューあり
				$ELSE$
					$iocsender = "IocSend"$
					$iocsender_generic = "ioc_send_generic"$
				$END$
$				// FUNCTION, MACRO, DO_NOT_CARE, 省略(NULL)を判断してコードを生成
				$FOREACH iocsnd IOC.IOCSND_LIST[iocid]$
$					// FUNCTION, MACRO, DO_NOT_CARE, 省略(NULL)以外のエラーチェックは確認済み
					$IF !EQ(IOCSND.IMPLKIND[iocsnd], "MACRO")$
						$FILE "Ioc.h"$
						extern Std_ReturnType $iocsender$_$iocid$(
							$IF !EQ(category, "VALUE")$
								const $shortname$ *
							$ELSE$
								$shortname$$SPC$
							$END$
						in1);
						
						$FILE "Ioc.c"$
						Std_ReturnType$NL$
						$iocsender$_$iocid$(
							$IF !EQ(category, "VALUE")$
								const $shortname$ *
							$ELSE$
								$shortname$$SPC$
							$END$
						in1)$NL$
						{$NL$
							$TAB$Std_ReturnType$TAB$ercd;$NL$
							$TAB$IOCMB_$iocid$$TAB$in;$NL$
							$TAB$in.data1 =$SPC$
								$IF !EQ(category, "VALUE")$
									*
								$END$
							in1;$NL$
							$TAB$ercd = $iocsender_generic$(IOC_WRAPPER_$+iocsnd - 1$, (void *)(&in));$NL$
							$TAB$return(ercd);$NL$
						}$NL$
					$ELSE$
						$FILE "Ioc.h"$
						LOCAL_INLINE Std_ReturnType$NL$
						$iocsender$_$iocid$(
							$IF !EQ(category, "VALUE")$
								const $shortname$ *
							$ELSE$
								$shortname$$SPC$
							$END$
						 in1)$NL$
						{$NL$
							$TAB$Std_ReturnType$TAB$ercd;$NL$
							$TAB$IOCMB_$iocid$$TAB$in;$NL$
							$TAB$in.data1 =$SPC$
								$IF !EQ(category, "VALUE")$
								 *
								$END$
							in1;$NL$
							$TAB$ercd = $iocsender_generic$(IOC_WRAPPER_$+iocsnd - 1$, (void *)(&in));$NL$
							$TAB$return(ercd);$NL$
						}$NL$
					$END$
				$END$
$ 			// 1:1，グループ通信
			$ELSE$
$				// 1:1，グループ通信，キュー無し
				$IF EQ(IOC.BUFLEN[iocid], "NULL")$
					$iocsender = "IocWriteGroup"$
					$iocsender_generic = "ioc_write_generic"$
$				// 1:1，グループ通信，キューあり
				$ELSE$
					$iocsender = "IocSendGroup"$
					$iocsender_generic = "ioc_send_generic"$
				$END$
$				// FUNCTION, MACRO, DO_NOT_CARE, 省略(NULL)を判断してコードを生成
				$FOREACH iocsnd IOC.IOCSND_LIST[iocid]$
$					// FUNCTION, MACRO, DO_NOT_CARE, 省略(NULL)以外のエラーチェックは確認済み
					$IF !EQ(IOCSND.IMPLKIND[iocsnd], "MACRO")$
						$FILE "Ioc.h"$
						extern Std_ReturnType $iocsender$_$iocid$(
							$arg = 1$
							$JOINEACH iocdata IOC.IOCDATA_LIST[iocid] ", "$
								$shortname = GET_SHORT_NAME(IOCDATA.DATATYPE[iocdata], iocid)$
								$IF !EQ(GET_CATEGORY(IOCDATA.DATATYPE[iocdata], iocid), "VALUE")$
									const $shortname$ *in$arg$
								$ELSE$
									$shortname$ in$arg$
								$END$
								$arg = arg + 1$
							$END$
						);
						$FILE "Ioc.c"$
						Std_ReturnType$NL$
						$iocsender$_$iocid$(
							$arg = 1$
							$JOINEACH iocdata IOC.IOCDATA_LIST[iocid] ", "$
								$shortname = GET_SHORT_NAME(IOCDATA.DATATYPE[iocdata], iocid)$
								$IF !EQ(GET_CATEGORY(IOCDATA.DATATYPE[iocdata], iocid), "VALUE")$
									const $shortname$ *in$arg$
								$ELSE$
									$shortname$ in$arg$
								$END$
								$arg = arg + 1$
							$END$
						)$NL$
						{$NL$
							$TAB$Std_ReturnType$TAB$ercd;$NL$
							$TAB$IOCMB_$iocid$$TAB$in;$NL$
							$arg = 1$
							$FOREACH iocdata IOC.IOCDATA_LIST[iocid]$
								$TAB$in.data$arg$ =$SPC$
									$IF !EQ(GET_CATEGORY(IOCDATA.DATATYPE[iocdata], iocid), "VALUE")$
										*
									$END$
								in$arg$;$NL$
								$arg = arg + 1$
							$END$
							$TAB$ercd = $iocsender_generic$(IOC_WRAPPER_$+iocsnd - 1$, (void *)(&in));$NL$
							$TAB$return(ercd);$NL$
						}$NL$
					$ELSE$
						$FILE "Ioc.h"$
						LOCAL_INLINE Std_ReturnType$NL$
						$iocsender$_$iocid$(
							$arg = 1$
							$JOINEACH iocdata IOC.IOCDATA_LIST[iocid] ", "$
								$shortname = GET_SHORT_NAME(IOCDATA.DATATYPE[iocdata], iocid)$
								$IF !EQ(GET_CATEGORY(IOCDATA.DATATYPE[iocdata], iocid), "VALUE")$
									const $shortname$ *in$arg$
								$ELSE$
									$shortname$ in$arg$
								$END$
								$arg = arg + 1$
							$END$
						)$NL$
						{$NL$
							$TAB$Std_ReturnType$TAB$ercd;$NL$
							$TAB$IOCMB_$iocid$$TAB$in;$NL$
							$arg = 1$
							$FOREACH iocdata IOC.IOCDATA_LIST[iocid]$
								$TAB$in.data$arg$ =$SPC$
									$IF !EQ(GET_CATEGORY(IOCDATA.DATATYPE[iocdata], iocid), "VALUE")$
										*
									$END$
								in$arg$;$NL$
								$arg = arg + 1$
							$END$
							$TAB$ercd = $iocsender_generic$(IOC_WRAPPER_$+iocsnd - 1$, (void *)(&in));$NL$
							$TAB$return(ercd);$NL$
						}
					$END$
				$END$
			$END$
$ 		// N:1通信
		$ELSE$
$ 			// N:1，通常通信
			$IF LENGTH(IOC.IOCDATA_LIST[iocid]) == 1$
$ 				// N:1，単一データ通信，キュー無し
				$IF EQ(IOC.BUFLEN[iocid], "NULL")$
					$iocsender = "IocWrite"$
					$iocsender_generic = "ioc_write_generic"$
$ 				// N:1，単一データ通信，キューあり
				$ELSE$
					$iocsender = "IocSend"$
					$iocsender_generic = "ioc_send_generic"$
				$END$
$				// FUNCTION, MACRO, DO_NOT_CARE, 省略(NULL)を判断してコードを生成
				$FOREACH iocsnd IOC.IOCSND_LIST[iocid]$
$					// FUNCTION, MACRO, DO_NOT_CARE, 省略(NULL)以外のエラーチェックは確認済み
					$IF !EQ(IOCSND.IMPLKIND[iocsnd], "MACRO")$
						$FILE "Ioc.h"$
						extern Std_ReturnType $iocsender$_$iocid$_$IOCSND.SENDERID[iocsnd]$(
							$IF !EQ(category, "VALUE")$
								const $shortname$ *
							$ELSE$
								$shortname$$SPC$
							$END$
						in1);
						
						$FILE "Ioc.c"$
						Std_ReturnType$NL$
						$iocsender$_$iocid$_$IOCSND.SENDERID[iocsnd]$(
							$IF !EQ(category, "VALUE")$
								const $shortname$ *
							$ELSE$
								$shortname$$SPC$
							$END$
						in1)$NL$
						{$NL$
							$TAB$Std_ReturnType$TAB$ercd;$NL$
							$TAB$IOCMB_$iocid$$TAB$in;$NL$
							$TAB$in.data1 =$SPC$
								$IF !EQ(category, "VALUE")$
									*
								$END$
							in1;$NL$
							$TAB$ercd = $iocsender_generic$(IOC_WRAPPER_$+iocsnd - 1$, (void *)(&in));$NL$
							$TAB$return(ercd);$NL$
						}$NL$
					$ELSE$
						$FILE "Ioc.h"$
						LOCAL_INLINE Std_ReturnType$NL$
						$iocsender$_$iocid$_$IOCSND.SENDERID[iocsnd]$(
							$IF !EQ(category, "VALUE")$
								const $shortname$ *
							$ELSE$
								$shortname$$SPC$
							$END$
						in1)$NL$
						{$NL$
							$TAB$Std_ReturnType$TAB$ercd;$NL$
							$TAB$IOCMB_$iocid$$TAB$in;$NL$
							$TAB$in.data1 =$SPC$
								$IF !EQ(category, "VALUE")$
								 *
								$END$
							in1;$NL$
							$TAB$ercd = $iocsender_generic$(IOC_WRAPPER_$+iocsnd - 1$, (void *)(&in));$NL$
							$TAB$return(ercd);$NL$
						}
					$END$
				$END$
$ 			// N:1，グループ通信のコンフィギュレーションはエラー
			$ELSE$
				$ERROR IOC.TEXT_LINE[iocid]$$FORMAT(_("N:1 and Group IOC can not be used."))$$END$
			$END$
		$END$

$ 		// 受信API - 受信APIは1:1 or N:1によってAPI生成は変化しない
$ 		// 単一データ通信
		$IF LENGTH(IOC.IOCDATA_LIST[iocid]) == 1$
$	 		// 単一データ通信，キュー無し
			$IF EQ(IOC.BUFLEN[iocid], "NULL")$
				$iocreceiver = "IocRead"$
				$iocreceiver_generic = "ioc_read_generic"$
				$iocerrorcheck = "ercd == IOC_E_OK"$
$	 		// 単一データ通信，キューあり
			$ELSE$
				$iocreceiver = "IocReceive"$
				$iocreceiver_generic = "ioc_receive_generic"$
				$iocerrorcheck = "(ercd == IOC_E_OK) || (ercd == IOC_E_LOST_DATA)"$
			$END$
$			// FUNCTION, MACRO, DO_NOT_CARE, 省略(NULL)を判断してコードを生成
$			// FUNCTION, MACRO, DO_NOT_CARE, 省略(NULL)以外のエラーチェックは確認済み
			$IF !EQ(IOCRCV.IMPLKIND[IOC.IOCRCV[iocid]], "MACRO")$
				$FILE "Ioc.h"$
				extern Std_ReturnType $iocreceiver$_$iocid$($shortname$ *out1);
				
				$FILE "Ioc.c"$
				Std_ReturnType$NL$
				$iocreceiver$_$iocid$($shortname$ *out1)$NL$
				{$NL$
					$TAB$Std_ReturnType$TAB$ercd;$NL$
					$TAB$IOCMB_$iocid$$TAB$out;$NL$
					$TAB$ercd = $iocreceiver_generic$($iocid$, (void *)(&out));$NL$
					$TAB$if ($iocerrorcheck$) {$NL$
					$TAB$$TAB$*out1 = out.data1;$NL$
					$TAB$}$NL$
					$TAB$return(ercd);$NL$
				}$NL$
			$ELSE$
				$FILE "Ioc.h"$
				LOCAL_INLINE Std_ReturnType$NL$
				$iocreceiver$_$iocid$($shortname$ *out1)$NL$
				{$NL$
					$TAB$Std_ReturnType$TAB$ercd;$NL$
					$TAB$IOCMB_$iocid$$TAB$out;$NL$
					$TAB$ercd = $iocreceiver_generic$($iocid$, (void *)(&out));$NL$
					$TAB$if ($iocerrorcheck$) {$NL$
					$TAB$$TAB$*out1 = out.data1;$NL$
					$TAB$}$NL$
					$TAB$return(ercd);$NL$
				}
			$END$
$ 		// グループ通信
		$ELSE$
$ 			// グループ通信，キュー無し
			$IF EQ(IOC.BUFLEN[iocid], "NULL")$
				$iocreceiver = "IocReadGroup"$
				$iocreceiver_generic = "ioc_read_generic"$
				$iocerrorcheck = "ercd == IOC_E_OK"$
$ 			// グループ通信，キューあり
			$ELSE$
				$iocreceiver = "IocReceiveGroup"$
				$iocreceiver_generic = "ioc_receive_generic"$
				$iocerrorcheck = "(ercd == IOC_E_OK) || (ercd == IOC_E_LOST_DATA)"$
			$END$
$			// FUNCTION, MACRO, DO_NOT_CARE, 省略(NULL)を判断してコードを生成
$			// FUNCTION, MACRO, DO_NOT_CARE, 省略(NULL)以外のエラーチェックは確認済み
			$IF !EQ(IOCRCV.IMPLKIND[IOC.IOCRCV[iocid]], "MACRO")$
				$FILE "Ioc.h"$
				extern Std_ReturnType $iocreceiver$_$iocid$(
				$arg = 1$
				$JOINEACH iocdata IOC.IOCDATA_LIST[iocid] ", "$
					$shortname = GET_SHORT_NAME(IOCDATA.DATATYPE[iocdata], iocid)$
					$shortname$$SPC$*out$arg$
					$arg = arg + 1$
				$END$
				);
				
				$FILE "Ioc.c"$
				Std_ReturnType$NL$
				$iocreceiver$_$iocid$(
				$arg = 1$
				$JOINEACH iocdata IOC.IOCDATA_LIST[iocid] ", "$
					$shortname = GET_SHORT_NAME(IOCDATA.DATATYPE[iocdata], iocid)$
					$shortname$$SPC$*out$arg$
					$arg = arg + 1$
				$END$
				)$NL$
				{$NL$
					$TAB$Std_ReturnType$TAB$ercd;$NL$
					$TAB$IOCMB_$iocid$$TAB$out;$NL$
					$TAB$ercd = $iocreceiver_generic$($iocid$, (void *)(&out));$NL$
					$TAB$if ($iocerrorcheck$) {$NL$
					$arg = 1$
					$FOREACH iocdata IOC.IOCDATA_LIST[iocid]$
						$TAB$$TAB$*out$arg$ = out.data$arg$;$NL$
						$arg = arg + 1$
					$END$
					$TAB$}$NL$
					$TAB$return(ercd);$NL$
				}$NL$
			$ELSE$
				$FILE "Ioc.h"$
				LOCAL_INLINE Std_ReturnType$NL$
				$iocreceiver$_$iocid$(
				$arg = 1$
				$JOINEACH iocdata IOC.IOCDATA_LIST[iocid] ", "$
					$shortname = GET_SHORT_NAME(IOCDATA.DATATYPE[iocdata], iocid)$
					$shortname$$SPC$*out$arg$
					$arg = arg + 1$
				$END$
				)$NL$
				{$NL$
					$TAB$Std_ReturnType$TAB$ercd;$NL$
					$TAB$IOCMB_$iocid$$TAB$out;$NL$
					$TAB$ercd = $iocreceiver_generic$($iocid$, (void *)(&out));$NL$
					$TAB$if ($iocerrorcheck$) {$NL$
					$arg = 1$
					$FOREACH iocdata IOC.IOCDATA_LIST[iocid]$
						$TAB$$TAB$*out$arg$ = out.data$arg$;$NL$
						$arg = arg + 1$
					$END$
					$TAB$}$NL$
					$TAB$return(ercd);$NL$
				}
			$END$
		$END$

$ 		// queue empty API
		$IF !EQ(IOC.BUFLEN[iocid], "NULL")$
			$FILE "Ioc.h"$
			extern Std_ReturnType IocEmptyQueue_$iocid$(void);
			
			$FILE "Ioc.c"$
			Std_ReturnType$NL$
			IocEmptyQueue_$iocid$(void)$NL$
			{$NL$
				$TAB$Std_ReturnType	ercd;$NL$
				$TAB$ercd = ioc_empty_queue_generic($iocid$);$NL$
				$TAB$return(ercd);$NL$
			}$NL$
		$END$
	$END$
$END$

$FILE "Ioc.h"$
$NL$
#endif /* TOPPERS_IOC_H_ */$NL$
