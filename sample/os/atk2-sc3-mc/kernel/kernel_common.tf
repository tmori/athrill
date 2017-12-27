$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2011-2015 by Center for Embedded Computing Systems
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
$  $Id: kernel_common.tf 427 2015-12-07 12:26:47Z witz-itoyo $
$

$ =====================================================================
$ TSK.STKとTSK.SSTKを整形
$ =====================================================================

$FOREACH tskid TSK.ID_LIST$
$   // 省略された場合はNULLにする
	$IF !LENGTH(TSK.STK[tskid])$
		$TSK.STK[tskid] = "NULL"$
	$END$

$   // 省略された場合はNULLにする
	$IF !LENGTH(TSK.SSTK[tskid])$
		$TSK.SSTK[tskid] = "NULL"$
	$END$
$END$

$ =====================================================================
$  メモリオブジェクトの先頭と末尾のアドレスの取り出し
$ =====================================================================

$IF !ISFUNCTION("START_SYMBOL")$
$FUNCTION START_SYMBOL$
	$RESULT = SYMBOL(CONCAT("__start_", ARGV[1]))$
$END$
$END$

$IF !ISFUNCTION("LIMIT_SYMBOL")$
$FUNCTION LIMIT_SYMBOL$
	$RESULT = SYMBOL(CONCAT("__limit_", ARGV[1]))$
$END$
$END$

$ =====================================================================
$  データセクションのLMAからVMAへのコピー
$ =====================================================================

$FUNCTION COPY_LMA$
	$FOREACH lma LMA.ORDER_LIST$
		$start_data = SYMBOL(LMA.START_DATA[lma])$
		$end_data = SYMBOL(LMA.END_DATA[lma])$
		$start_idata = SYMBOL(LMA.START_IDATA[lma])$
		$IF !LENGTH(start_data)$
			$ERROR$
				$FORMAT(_("symbol '%1%' not found"), LMA.START_DATA[lma])$
			$END$
		$ELIF !LENGTH(end_data)$
			$ERROR$
				$FORMAT(_("symbol '%1%' not found"), LMA.END_DATA[lma])$
			$END$
		$ELIF !LENGTH(start_idata)$
			$ERROR$
				$FORMAT(_("symbol '%1%' not found"), LMA.START_IDATA[lma])$
			$END$
		$ELSE$
			$BCOPY(start_idata, start_data, end_data - start_data)$
		$END$
	$END$
$END$

$
$  タスクのスタックをextern宣言する
$  スタックサイズにマクロがあるため，スタックサイズは空にする必要がある
$
$FUNCTION GENERATE_EXPORT_TSK_STK$
	$FOREACH tskid TSK.ID_LIST$
		$IF OSAP.TRUSTED[TSK.OSAPID[tskid]] || tmin_os_restarttask <= tskid$
			$extern_sstk = EQ(TSK.STK[tskid], "NULL") && !LENGTH(TSK.SHARED_SSTK_ID[tskid])$
			$extern_ustk = 0$
		$ELSE$
			$extern_sstk = EQ(TSK.SSTK[tskid], "NULL") && !LENGTH(TSK.SHARED_SSTK_ID[tskid])$
			$extern_ustk = EQ(TSK.STK[tskid], "NULL") && !LENGTH(TSK.SHARED_USTK_ID[tskid])$
		$END$

		$IF extern_sstk$
			$IF !EQ(TSK.RESTARTTASK[tskid],"")$
				extern StackType _kernel_sstack_restart_$TSK.RESTARTTASK[tskid]$[];$NL$
			$ELSE$
				extern StackType _kernel_sstack_$tskid$[];$NL$
			$END$
		$END$
		$IF extern_ustk$
			extern StackType _kernel_ustack_$tskid$[];$NL$
		$END$
	$END$
	$FOREACH coreid RANGE(0, TMAX_COREID)$
		$FOREACH tskpri RANGE(TMIN_TPRI, TMAX_TPRI)$
			$shared_stkid = coreid * TNUM_TPRI + tskpri$
			$IF LENGTH(shared_ustack_size[shared_stkid])$
				extern StackType _kernel_shared_ustack_$shared_stkid$[];$NL$
			$END$
			$IF LENGTH(shared_sstack_size[shared_stkid])$
				extern StackType _kernel_shared_sstack_$shared_stkid$[];$NL$
			$END$
		$END$
	$END$
	$NL$
$END$

$
$ タスクのextern宣言
$
$FUNCTION EXTERN_TSK$
$ 	//タスクのextern宣言
	$FOREACH tskid TSK.ID_LIST$
		$IF tskid < tmin_os_restarttask$
			extern TASK($tskid$);$NL$
		$END$
	$END$
	$NL$
$END$

$
$ タスク初期化ブロックの出力
$
$FUNCTION GENERATE_TINIB_TABLE$

	/* Task Initialization Block */$NL$
	$IF LENGTH(TSK.ID_LIST)$
		const TINIB tinib_table[TNUM_TASK_INC_RT] = {$NL$
		$JOINEACH tskid TSK.ID_LIST ",\n"$
			$TAB${$NL$
$			// OSが生成したリスタートタスクの場合は，
$           // この初期化ブロックがOSによって生成されたものであることを
$           // 示すコメントを出力する
			$IF tmin_os_restarttask <= tskid$
				$TAB$$TAB$/* this TINIB is genereted by OS */$NL$
			$END$
$			//OSが生成したリスタートタスクの場合は，タスクの先頭アドレスをNULLにする
			$IF tmin_os_restarttask <= tskid$
				$TAB$$TAB$NULL,$NL$
			$ELSE$
				$TAB$$TAB$&TASKNAME($tskid$),$NL$
			$END$

$			// タスク初期化コンテキストブロック，スタック領域
			$IF USE_TSKINICTXB$
				$GENERATE_TSKINICTXB(tskid)$
			$ELSE$
				$TAB$$TAB$$TSK.TINIB_SSTKSZ[tskid]$,$NL$
				$TAB$$TAB$(void *) $TSK.TINIB_SSTK[tskid]$,$NL$
				$TAB$$TAB$$TSK.TINIB_USTKSZ[tskid]$,$NL$
				$TAB$$TAB$(void *) $TSK.TINIB_USTK[tskid]$,$NL$
			$END$

			$TAB$$TAB$&(_kernel_osapcb_$TSK.OSAPID[tskid]$),$NL$
			$TAB$$TAB$$FORMAT("0x%08xU", +TSK.ACSBTMP[tskid])$,$NL$
			$TAB$$TAB$$TMAX_TPRI - TSK.PRIORITY[tskid]$,$NL$
			$IF EQ(TSK.SCHEDULE[tskid], "NON") $
				$TAB$$TAB$$+TPRI_MAXTASK$,$NL$
			$ELSE$
				$IF LENGTH(TSK.INRESPRI[tskid])$
					$TAB$$TAB$$TMAX_TPRI - TSK.INRESPRI[tskid]$,$NL$
				$ELSE$
					$TAB$$TAB$$TMAX_TPRI - TSK.PRIORITY[tskid]$,$NL$
				$END$
			$END$

			$TAB$$TAB$($+TSK.ACTIVATION[tskid]$U) - 1U,$NL$
			$TAB$$TAB$&_kernel_core$+OSAP.CORE[TSK.OSAPID[tskid]]$_ccb,$NL$
			$IF !OMIT_STKMPUINFOB$
				$TAB$$TAB$$FORMAT("0x%08xU", +TSK.ASTPTN[tskid])$,$NL$
				$GENERATE_STKMPUINFOB(tskid)$
			$ELSE$
				$TAB$$TAB$$FORMAT("0x%08xU", +TSK.ASTPTN[tskid])$$NL$
			$END$
			$TAB$}
		$END$
		$NL$
		};$NL$
	$ELSE$
		TOPPERS_EMPTY_LABEL(const TINIB, tinib_table);$NL$
	$END$
	$NL$
$END$

$
$  OSアプリケーション初期化ブロックの出力
$
$FUNCTION GENERATE_OSAPINIB_TABLE$
	$IF LENGTH(OSAP.ID_LIST)$
		const OSAPINIB osapinib_table[TNUM_OSAP] = {$NL$
		$JOINEACH osapid OSAP.ID_LIST ",\n"$
			$TAB${$NL$
			$IF EQ(OSAP.RESTARTTASK[osapid], "NO_RESTART_TASK")$
				$TAB$$TAB$NULL,$NL$
			$ELSE$
				$IF EQ(OSAP.RESTARTTASK[osapid], "OS_GEN_RESTARTTASK")$
$					//OSが生成したリスタートタスクの場合
					$TAB$$TAB$&_kernel_tcb_restart_$+OSAP.RESTARTTASK[osapid]$,$NL$
				$ELSE$
$					//ユーザが生成したリスタートタスクの場合
					$TAB$$TAB$&_kernel_tcb_$OSAP.RESTARTTASK[osapid]$,$NL$
				$END$
			$END$
			$IF OSAP.TRUSTED[osapid]$
				$TAB$$TAB$TA_TRUSTED,$NL$
			$ELSE$
				$TAB$$TAB$TA_NONTRUSTED,$NL$
			$END$
			$TAB$$TAB$$OSAP.CORE[osapid]$,$NL$

			$IF !OMIT_OSAPMPUINFOB$
				$TAB$$TAB$$FORMAT("0x%08xU", +OSAP.BTMP[osapid])$,$NL$
				$GENERATE_OSAPINIB_MPUINFOB(osapid)$
			$ELSE$
				$TAB$$TAB$$FORMAT("0x%08xU", +OSAP.BTMP[osapid])$$NL$
			$END$

			$TAB$}
		$END$
		$NL$
		};$NL$
		$NL$
	$ELSE$
		TOPPERS_EMPTY_LABEL(const OSAPINIB, osapinib_table);$NL$
	$END$
$END$

$
$  メモリオブジェクトのベースアドレス順のリストから
$  非信頼OSAPからアクセスできない領域を除いたリスト
$  をMO_MEMTOP_ORDER_MEMINIBに入れる
$
$FUNCTION SET_MEMTOP_ORDER$
	$prev = 0$
	$FOREACH moid SORT(MO_SECTION_LIST, "MO.BASEADDR")$
	
		$IF (MO.ACPTN_R[moid] != TACP_KERNEL || MO.ACPTN_W[moid] != TACP_KERNEL || MO.ACPTN_X[moid] != TACP_KERNEL)$
			$IF !prev || (MO.LIMITADDR[prev] <= MO.BASEADDR[moid] && MO.LIMITADDR[prev] != 0)$
				$MO_MEMTOP_ORDER_MEMINIB = APPEND(MO_MEMTOP_ORDER_MEMINIB, moid)$
				$prev = moid$
			$ELSE$
$				// 同じOSAPに所属するユーザスタックの領域合併
				$IF MO.TYPE[moid] == TOPPERS_USTACK &&
					MO.TYPE[prev] == TOPPERS_USTACK &&
					MO.OSAPID[prev] == MO.OSAPID[moid]$
$					// ユーザスタック領域の併合処理
					$MO.TSKID[prev] = 0$
					$MO.TSKID[moid] = 0$
					$MO.BASE[moid] = MO.BASE[prev]$
					$MO.BASEADDR[moid] = MO.BASEADDR[prev]$
					$IF MO.LIMITADDR[prev] < MO.LIMITADDR[moid]
													&& MO.LIMITADDR[prev] != 0$
						$MO.SIZE[prev] = MO.LIMITADDR[moid] - MO.BASEADDR[prev]$
						$MO.LIMITADDR[prev] = MO.LIMITADDR[moid]$
					$ELSE$
						$MO.SIZE[moid] = MO.SIZE[prev]$
						$MO.LIMITADDR[moid] = MO.LIMITADDR[prev]$
					$END$
				$END$
			$END$
		$END$
	$END$
$END$

$
$  隣接するメモリオブジェクトのメモリ保護属性が等しい場合は統合する
$
$FUNCTION MERGE_MEMINIB$
	$merged_moid = 0$
	$FOREACH moid MO_MEMTOP_ORDER_MEMINIB$
		$IF merged_moid &&
			(CFG_PASS4 ||
			 MO.LINKER[merged_moid] && MO.LINKER[moid] &&
			 MO.MEMREG[merged_moid] == MO.MEMREG[moid]) &&
			((MO.MERGED_LIMITADDR[merged_moid] == MO.BASEADDR[moid] &&
			MO.ACPTN_R[merged_moid] == MO.ACPTN_R[moid] &&
			MO.ACPTN_W[merged_moid] == MO.ACPTN_W[moid] &&
			MO.ACPTN_X[merged_moid] == MO.ACPTN_X[moid]) ||
			MO.LIMITADDR[moid] == MO.BASEADDR[moid])$

			$MO.MERGED_LIMITADDR[merged_moid] = MO.LIMITADDR[moid]$
			$MO.MERGED_LIMIT[merged_moid] = MO.LIMIT[moid]$
		$ELSE$
			$merged_moid = moid$
			$new_list = APPEND(new_list, merged_moid)$
			$MO.MERGED_LIMITADDR[moid] = MO.LIMITADDR[moid]$
			$MO.MERGED_LIMIT[moid] = MO.LIMIT[moid]$
		$END$
	$END$
	$MO_MEMTOP_ORDER_MEMINIB = new_list$
$END$

$ 
$ CCB生成
$ 
$FUNCTION GENERATE_CCB_TABLE$
	$NL$
	 /****** CCB ******/$NL$
	$NL$

$ 	CCBの出力
	$FOREACH id RANGE(0, TMAX_COREID)$
		$IF ISFUNCTION("GENERATE_CCB")$
			$GENERATE_CCB(id)$
		$ELSE$
			CCB _kernel_core$id$_ccb = {0, FALSE, 0, 0, 0, 0, 0};$NL$
		$END$
	$END$
	$NL$
	$NL$

$ 	CCBテーブルの出力
	CCB * const p_ccb_table[TotalNumberOfCores] = {$NL$

	$JOINEACH id RANGE(0, TMAX_COREID) ",\n"$
		$TAB$&_kernel_core$id$_ccb
	$END$
	$NL$};$NL$
	$NL$
$END$

$ 
$ CCB外部参照宣言生成
$ 
$FUNCTION GENERATE_CCB_EXTERN$
	$NL$
	 /****** CCB ******/$NL$
	$NL$

$ 	CCBの出力
	$FOREACH id RANGE(0, TMAX_COREID)$
		$IF ISFUNCTION("GENERATE_CCB_TARGET_EXTERN")$
			$GENERATE_CCB_TARGET_EXTERN(id)$
		$ELSE$
			extern CCB _kernel_core$id$_ccb;$NL$
		$END$
	$END$
	$NL$
$END$

$
$ OSAPCB外部参照宣言生成
$
$FUNCTION EXTERN_OSAPCB$
$ 	//OSAPCBのextern宣言
	$FOREACH osapid OSAP.ID_LIST$
		extern OSAPCB _kernel_osapcb_$osapid$;$NL$
	$END$
	$NL$
$END$

$
$ TCB外部参照宣言生成
$
$FUNCTION EXTERN_TCB$
$ 	//TCBのextern宣言
	$FOREACH taskid TSK.ID_LIST$
		$IF taskid < tmin_os_restarttask$
            extern TCB _kernel_tcb_$taskid$;$NL$
		$ELSE$
            extern TCB _kernel_tcb_restart_$+taskid$;$NL$
		$END$
	$END$
	$NL$
$END$

