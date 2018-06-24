$ ======================================================================
$
$   TOPPERS/HRP Kernel
$       Toyohashi Open Platform for Embedded Real-Time Systems/
$       High Reliable system Profile Kernel
$
$   Copyright (C) 2011 by Embedded and Real-Time Systems Laboratory
$               Graduate School of Information Science, Nagoya Univ., JAPAN
$  
$   上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
$   ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
$   変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
$   (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
$       権表示，この利用条件および下記の無保証規定が，そのままの形でソー
$       スコード中に含まれていること．
$   (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
$       用できる形で再配布する場合には，再配布に伴うドキュメント（利用
$       者マニュアルなど）に，上記の著作権表示，この利用条件および下記
$       の無保証規定を掲載すること．
$   (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
$       用できない形で再配布する場合には，次のいずれかの条件を満たすこ
$       と．
$     (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
$         作権表示，この利用条件および下記の無保証規定を掲載すること．
$     (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
$         報告すること．
$   (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
$       害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
$       また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
$       由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
$       免責すること．
$  
$   本ソフトウェアは，無保証で提供されているものである．上記著作権者お
$   よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
$   に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
$   アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
$   の責任を負わない．
$
$   $Id: redzone.tf 471 2011-04-05 05:41:43Z ertl-hiro $
$  
$ =====================================================================

$ =====================================================================
$  レッドゾーン方式におけるユーザスタック領域の配置順序の決定
$
$ ここで配置順序を決定するのは，コンフィギュレータが割り付けるユーザス
$ タック領域のみであり，アプリケーションが指定したものは対象としない．
$ =====================================================================

$
$  ドメイン毎のユーザスタック領域のリストの作成と数のカウント
$
$ total_num_ustack：ユーザスタック領域の数の合計
$ ustack_list_dom[domid]：ドメイン毎のユーザスタック領域のリスト
$ max_num_ustack：ドメイン毎のユーザスタック領域の数の最大値
$ max_ustack_dom：ユーザスタック領域の数が最大であるドメイン

$total_num_ustack = 0$
$max_num_ustack = 0$
$FOREACH moid MO_USTACK_LIST$
	$total_num_ustack = total_num_ustack + 1$
	$ustack_list_dom[MO.DOMAIN[moid]]
					= APPEND(ustack_list_dom[MO.DOMAIN[moid]], moid)$
	$IF max_num_ustack < LENGTH(ustack_list_dom[MO.DOMAIN[moid]])$
		$max_num_ustack = LENGTH(ustack_list_dom[MO.DOMAIN[moid]])$
		$max_ustack_dom = MO.DOMAIN[moid]$
	$END$
$END$

$
$  ダミースタック領域の追加
$
$num_dummy = max_num_ustack * 2 - 1 - total_num_ustack$
$IF num_dummy > 0$
	$FOREACH dummy RANGE(1,num_dummy)$
$		// ダミータスク名
		$dummy_name = FORMAT("TOPPERS_DUMMY%d", dummy)$

$		// ダミースタック領域の確保
		$ALLOC_USTACK_DUMMY(dummy_name, TARGET_DUMMY_STKSZ)$

$		// ダミースタック領域のメモリオブジェクト情報の生成
$		// ダミースタック領域の属するドメインは無所属とする
		$nummo = nummo + 1$
		$MO.TYPE[nummo] = TOPPERS_USTACK$
		$MO.TSKID[nummo] = VALUE(dummy_name, 0)$
		$MO.SIZE[nummo] = TARGET_DUMMY_STKSZ$
		$MO.LINKER[nummo] = 1$
		$MO.MEMREG[nummo] = STANDARD_RAM$
		$MO.SECTION[nummo] = SECTION_USTACK(dummy_name)$
		$MO_USTACK_LIST = APPEND(MO_USTACK_LIST, nummo)$
		$MO.DOMAIN[nummo] = TDOM_NONE$
		$MO.MEMATR[nummo] = TARGET_MEMATR_USTACK$
		$MO.ACPTN1[nummo] = TACP_KERNEL$
		$MO.ACPTN2[nummo] = TACP_KERNEL$
		$MO.ACPTN4[nummo] = TACP_KERNEL$

		$total_num_ustack = total_num_ustack + 1$
		$ustack_list_dom[MO.DOMAIN[nummo]]
					= APPEND(ustack_list_dom[MO.DOMAIN[nummo]], nummo)$
	$END$
$END$

$
$  配置順序の決定
$

$ 配置順序の割り当て関数
$FUNCTION ASSIGN_STKORDER$
	$stkorder = stkorder + 1$
	$MO.STKORDER[AT(ustack_list_dom[ARGV[1]], at_dom[ARGV[1]])] = stkorder$
	$at_dom[ARGV[1]] = at_dom[ARGV[1]] + 1$

	$total_num_ustack = total_num_ustack - 1$
	$IF ARGV[1] == max_ustack_dom$
		$max_num_ustack = max_num_ustack - 1$
	$END$
$END$

$ at_dom[domid]の初期化（各要素を0にする）
$FOREACH domid DOMLIST_ALL$
	$at_dom[domid] = 0$
$END$

$ 配置順序を順に割り当てる
$stkorder = 0$
$WHILE stkorder < LENGTH(MO_USTACK_LIST)$
	$FOREACH domid DOMLIST_ALL$
		$IF at_dom[domid] < LENGTH(ustack_list_dom[domid])$
			$IF domid != max_ustack_dom
								&& max_num_ustack * 2 > total_num_ustack$
				$ASSIGN_STKORDER(max_ustack_dom)$
			$END$
			$ASSIGN_STKORDER(domid)$
		$END$
	$END$
$END$
