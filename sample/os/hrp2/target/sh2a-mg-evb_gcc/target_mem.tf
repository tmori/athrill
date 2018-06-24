$ ======================================================================
$
$   TOPPERS/HRP Kernel
$       Toyohashi Open Platform for Embedded Real-Time Systems/
$       High Reliable system Profile Kernel
$
$   Copyright (C) 2011-2012 by Embedded and Real-Time Systems Laboratory
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
$ =====================================================================

$FUNCTION GET_SSTK_TSKINICTXB$
    $bottom = PEEK(ARGV[1] + offsetof_TINIB_TSKINICTXB_sstk_bottom, sizeof_void_ptr)$
    $size = PEEK(ARGV[1] + offsetof_TINIB_TSKINICTXB_sstksz, sizeof_SIZE)$
    $RESULT = (bottom - size)$
$END$

$FUNCTION GET_USTK_TSKINICTXB$
    $bottom = PEEK(ARGV[1] + offsetof_TINIB_TSKINICTXB_stk_bottom, sizeof_void_ptr)$
    $size = PEEK(ARGV[1] + offsetof_TINIB_TSKINICTXB_stksz, sizeof_SIZE)$
    $RESULT = (bottom - size)$
$END$

$INCLUDE "sh12a_gcc/prc_mem.tf"$

$FILE "kernel_mem.c"$

$ 
$  ユーザスタックは必ず有効である
$  ベクタテーブル，ベクターエントリ，MPU有効レジスタは必ず有効である
$  キーコードは0xaaaa0000
$ 
$VALID_MAP_INIT = (0x00000000 | (1 << 4) | (1 << 9) | (1 << 10) | (1 << 11) | 0xaaaa0000)$

$IF LENGTH(DOM.ID_LIST)$
    $shared_meminib = SYMBOL("shared_meminib_table")$
    $FOREACH id_bit RANGE(5,8)$
$   領域の開始アドレスを読み込む 
        $start_address = PEEK(shared_meminib, 4)$
        $shared_meminib = shared_meminib + 4$
$   領域の終了アドレスを読み込む 
        $end_address = PEEK(shared_meminib, 4) + 4$
        $shared_meminib = shared_meminib + 4$

        $IF start_address != end_address$
            $VALID_MAP_INIT = VALID_MAP_INIT | (1 << id_bit)$ 
        $END$
    $END$

    $dominib = SYMBOL("_kernel_dominib_table")$
	const uint32_t dom_valid_map_table[TNUM_DOMID] = {$NL$
	$JOINEACH domid DOM.ID_LIST ",\n"$
        $WARNING$
            $FORMAT("domid = %s\n",domid)$
        $END$
$   アクセス許可パターンはパスする
        $dominib = dominib + sizeof_ACPTN$
        $valid_map = VALID_MAP_INIT$
        $FOREACH id_bit RANGE(0,3)$
$   領域の開始アドレスを読み込む 
            $start_address = PEEK(dominib, 4)$
            $dominib = dominib + 4$
$   領域の終了アドレスを読み込む 
            $end_address = PEEK(dominib, 4) + 4$
            $dominib = dominib + 4$

            $IF start_address != end_address$
               $valid_map = valid_map | (1 << id_bit)$ 
            $END$
        $END$
        $TAB$$FORMAT("0x%08x", +valid_map)$
$   有効ビットのアドレスはパスする
        $dominib = dominib + 4$
	$END$,$NL$
	};$NL$
	$NL$
$ELSE$
	const DOMINIB dom_valid_map_table[0];$NL$
$END$$NL$


