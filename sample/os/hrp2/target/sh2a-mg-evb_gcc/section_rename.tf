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

$FILE "section_rename_cfg.sh"$
$PRE_OBJ = "hrp2.tmp.elf"$
$FINAL_OBJ = "hrp2.elf"$
$FINAL_LDSCRIPT = "ldscript3.ld"$
$CRE_FILE_LIST = {}$

mv $FINAL_OBJ$ $PRE_OBJ$$NL$

$IF !OMIT_IDATA$
    $IF LENGTH(DATASEC_LIST)$
        $FOREACH moid DATASEC_LIST$
$           # hrp2.elfから.dataセクションのみをコピー
            sh-elf-objcopy --only-section=.$MO.SLABEL[moid]$ $PRE_OBJ$ $MO.SLABEL[moid]$.$PRE_OBJ$$NL$
$           # hrp2.elfの.dataセクションを.idataセクションにリネーム
            sh-elf-objcopy --rename-section .$MO.SLABEL[moid]$=.$MO.ILABEL[moid]$ $PRE_OBJ$$NL$
$           # hrp2.dataのシンボルをファイルに保存
            sh-elf-nm $MO.SLABEL[moid]$.$PRE_OBJ$ | sed "s/^[^ ]* [^ ]* //g" > $MO.SLABEL[moid]$.$PRE_OBJ$.syms$NL$
$           # hrp2.elfから、hrp2.elfとhrp2.dataで衝突するシンボルを削除
            sh-elf-objcopy --strip-symbols=$MO.SLABEL[moid]$.$PRE_OBJ$.syms $PRE_OBJ$$NL$

            $CRE_FILE_LIST = APPEND(FORMAT("%s.%s", MO.SLABEL[moid], PRE_OBJ), CRE_FILE_LIST)$
            $CRE_FILE_LIST = APPEND(FORMAT("%s.%s.syms", MO.SLABEL[moid], PRE_OBJ), CRE_FILE_LIST)$
        $END$$NL$
    $END$
$END$

$  hrp2.elfとhrp2.dataをリンク 
sh-elf-gcc -nostdlib -mb -T $FINAL_LDSCRIPT$ -o $FINAL_OBJ$ $PRE_OBJ$
$IF !OMIT_IDATA$
    $IF LENGTH(DATASEC_LIST)$
        $FOREACH moid DATASEC_LIST$
            $SPC$$MO.SLABEL[moid]$.$PRE_OBJ$
        $END$
    $END$
$END$
$SPC$-lgcc$NL$

$IF !OMIT_IDATA$
    $IF LENGTH(DATASEC_LIST)$
        $FOREACH moid DATASEC_LIST$
$           # .dataセクションのLOAD属性を削除
            sh-elf-objcopy --set-section-flags .$MO.SLABEL[moid]$="alloc,noload" $FINAL_OBJ$$NL$
        $END$$NL$
    $END$
$END$

$  ローカルシンボルを削除
sh-elf-nm $FINAL_OBJ$ > $FINAL_OBJ$.syms$NL$
ruby rename.rb $FINAL_OBJ$.syms > $FINAL_OBJ$.localsyms$NL$
sh-elf-objcopy --strip-symbols=$FINAL_OBJ$.localsyms $FINAL_OBJ$$NL$
$NL$

$  中間ファイルを削除
$FOREACH file CRE_FILE_LIST$
    rm -f $file$$NL$
$END$

$INCLUDE "ldscript3.tf"$

