# -*- coding: utf-8 -*-
#
#  TECS Generator
#      Generator for TOPPERS Embedded Component System
#  
#   Copyright (C) 2008-2014 by TOPPERS Project
#--
#   上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
#   ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
#   変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
#   (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
#       権表示，この利用条件および下記の無保証規定が，そのままの形でソー
#       スコード中に含まれていること．
#   (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
#       用できる形で再配布する場合には，再配布に伴うドキュメント（利用
#       者マニュアルなど）に，上記の著作権表示，この利用条件および下記
#       の無保証規定を掲載すること．
#   (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
#       用できない形で再配布する場合には，次のいずれかの条件を満たすこ
#       と．
#     (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
#         作権表示，この利用条件および下記の無保証規定を掲載すること．
#     (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
#         報告すること．
#   (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
#       害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
#       また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
#       由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
#       免責すること．
#  
#   本ソフトウェアは，無保証で提供されているものである．上記著作権者お
#   よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
#   に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
#   アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
#   の責任を負わない．
#  
#   $Id: messages_file_ja_JP.rb 2587 2016-07-31 13:26:28Z okuma-top $
#++


# File Messages for ja_JP
class TECSMsg

## generate.rb ##

@@comment = {}

# MSg.note
@@comment[ :note ] = " * このファイルは tecsgen により自動生成されました
 * このファイルを編集して使用することは、意図されていません
"

@@comment[ :preamble_note ] = "/* #[<PREAMBLE>]#
 * #[<...>]# から #[</...>]# で囲まれたコメントは編集しないでください
 * tecsmerge によるマージに使用されます
"

@@comment[ :postamble_note ] = "/* #[<POSTAMBLE>]#
 *   これより下に非受け口関数を書きます
 * #[</POSTAMBLE>]#*/
"

@@comment[ :template_note ] = "/*
 * このファイルは tecsgen によりテンプレートとして自動生成されました
 * このファイルを編集して使用することが意図されていますが
 * tecsgen の再実行により上書きされてしまうため、通常
 *   gen/%s_template.c => src/%s.c
 * のように名前, フォルダを変更してから修正します
 */
"

@@comment[ :inline_template_note ] = "/*
 * このファイルは tecsgen によりテンプレートとして自動生成されました
 * このファイルを編集して使用することが意図されていますが
 * tecsgen の再実行により上書きされてしまうため、通常
 *   gen/%s_inline_template.h => src/%s_inline.h
 * のように名前, フォルダを変更してから修正します
 */
"

@@comment[ :Makefile_note ] = "# このファイルは tecsgen により自動生成されました
# Makefile.temp は gen の一つ上のディレクトリに移して使用します
#  % mv Makefile.temp Makefile
# 
# clean のデフォルト動作は $(GEN_DIR)/Makefile.* を削除します
#
# このファイルは GNU make で gcc を用い POSIX 環境で動作するモジュールをビルドするのに適切なように生成されています
# vpath, CFLAGS, OTHER_OBJS などを調整する必要があるかもしれません
# 他の環境やターゲットの場合、このファイルを元に変更する必要があります

"

@@comment[ :oneway_ercd_note ] = "/* oneway の場合 E_OK しか返せません */"
@@comment[ :ercd_note ] = "/* エラー処理コードをここに記述します */"

@@comment[ :IMP_comment ] = "\n/* import_C により import されるヘッダ %s */\n"
@@comment[ :MVAR_comment ] = "# 変数 %s\n"
@@comment[ :MRUL_comment ] = "# ルール %s\n"
@@comment[ :TCM_comment ] = "# テンプレートコードでメークしてみる場合 %s \n"
@@comment[ :MDEP_comment ] = "\n# depend を include %s\n"
@@comment[ :SDI_comment ] = "/* シグニチャディスクリプタ参照ヘッダ %s */\n"
@@comment[ :SD_comment ] = "/* シグニチャディスクリプタ %s */\n"
@@comment[ :SFT_comment ] = "/* シグニチャ関数テーブル %s */\n"
@@comment[ :SDES_comment ] = "/* シグニチャディスクリプタ(動的結合用) %s */\n"
# @@comment[ :IRTH_comment ] = "/* ランタイムヘッダ %s */\n"
@@comment[ :UDF_comment ] = "/* inline のための undef %s */\n"
@@comment[ :IGH_comment ] = "/* グローバルヘッダ %s */\n"
@@comment[ :ISH_comment ] = "/* シグニチャヘッダ %s */\n"
@@comment[ :ICT_comment ] = "/* 最適化のため参照するセルタイプの CB 型の定義を取込む %s */\n"
@@comment[ :NIDB_comment ] = "ID のベース "
@@comment[ :NCEL_comment ] = "セルの個数"
@@comment[ :CVI_comment ] = "/* IDXの正当性チェックマクロ %s */\n"
@@comment[ :CVIA_comment ] = "/* IDXの正当性チェックマクロ（短縮形） %s */\n"
@@comment[ :NCPA_comment ] = "/* 呼び口配列の大きさを得るマクロ %s */\n"
@@comment[ :NEPA_comment ] = "/* 受け口配列の大きさを得るマクロ %s */\n"
@@comment[ :TOCP_comment ] = "/* optional 呼び口をテストするマクロ %s */\n"
@@comment[ :TOCPA_comment ] = "/* optional 呼び口をテストするマクロ（短縮形） %s */\n"
@@comment[ :GCB_comment ] = "\n/* セルCBを得るマクロ %s */\n"
@@comment[ :GCBA_comment ] = "\n/* セルCBを得るマクロ(短縮形) %s */\n"
@@comment[ :CCT_comment ] = "/* CELLCB 型(短縮形) %s */\n"
@@comment[ :CTIXA_comment ] = "/* セルタイプのIDX型(短縮形) %s */\n"
@@comment[ :AAM_comment ] = "\n/* 属性アクセスマクロ %s */\n"
@@comment[ :VAM_comment ] = "\n/* var アクセスマクロ %s */\n"
@@comment[ :AAMA_comment ] = "\n/* 属性アクセスマクロ(短縮形) %s */\n"
@@comment[ :VAMA_comment ] = "\n/* var アクセスマクロ(短縮形) %s */\n"
@@comment[ :CPM_comment ] = " /* 呼び口関数マクロ %s */\n"
@@comment[ :CPMA_comment ] = "/* 呼び口関数マクロ（短縮形）%s */\n"
@@comment[ :EPM_comment ] = "\n/* 受け口関数マクロ（短縮形） %s */\n"
@@comment[ :CRD_comment ] = "/* ディスクリプタ参照関数 %s */\n"
@@comment[ :CRDA_comment ] = "\n/* ディスクリプタ参照マクロ（短縮形） %s */\n"
@@comment[ :SDF_comment ] = "/* ディスクリプタ設定関数 %s */\n"
@@comment[ :SDMA_comment ] = "\n/* ディスクリプタ設定マクロ（短縮形） %s */\n"
@@comment[ :CTIX_comment ] = "\n/* セルタイプのIDX型 %s */\n"
@@comment[ :EPP_comment ] = "\n/* 受け口関数プロトタイプ宣言 %s */\n"
@@comment[ :EPSP_comment ] = "\n/* 受け口スケルトン関数プロトタイプ宣言（VMT不要最適化により参照するもの） %s */\n"
@@comment[ :INL_comment ] = "/* inline ヘッダの include %s */\n"
@@comment[ :CIP_comment ] = "/* セル INIB 型宣言 %s */\n"
@@comment[ :CCTPA_comment ] = "/* セル CB 型宣言 %s */\n"
@@comment[ :CCDP_comment ] = "/* セル CB (ダミー)型宣言 %s */\n"
@@comment[ :CCTPO_comment ] = "/* セル CB 型宣言 %s */\n"
@@comment[ :SCP_comment ] = "/* シングルトンセル CB プロトタイプ宣言 %s */\n"
@@comment[ :DCI_comment ] = "\n/* CB は存在しない。INIB を CB の代わりに使用するための define %s */\n"
@@comment[ :FEC_comment ] = "/* イテレータコード (FOREACH_CELL)の生成 %s */\n"
@@comment[ :DAL_comment ] = "/* deallocate マクロ %s */\n"
@@comment[ :NFEC_comment ] = "/* イテレータコード (FOREACH_CELL)の生成(CB,INIB は存在しない) %s */\n"
@@comment[ :CIM_comment ] = "/* CB 初期化マクロ %s */\n"
@@comment[ :EDT_comment ] = "/* 受け口ディスクリプタ型 %s */\n"
@@comment[ :EPSF_comment ] = "/* 受け口スケルトン関数 %s */\n"
@@comment[ :EPSFT_comment ] = "/* 受け口スケルトン関数テーブル %s */\n"
@@comment[ :CPEPD_comment ] = "/* 呼び口の参照する受け口ディスクリプタ(実際の型と相違した定義) %s */\n"
@@comment[ :CPA_comment ] = "/* 呼び口配列 %s */\n"
@@comment[ :CIC_comment ] = "/* CB 初期化コード %s */\n"
@@comment[ :AVAI_comment ] = "/* 属性・変数の配列 %s */\n"
@@comment[ :AVI_comment ] = "/* 変数(構造体、配列)初期値 %s */\n"
@@comment[ :INIB_comment ] = "/* セル INIB %s */\n"
@@comment[ :CB_comment ] = "/* セル CB %s */\n"
@@comment[ :EPD_comment ] = "/* 受け口ディスクリプタ %s */\n"
@@comment[ :PAC_comment ] = "/* プロトタイプ宣言や変数の定義をここに書きます %s */\n"
@@comment[ :CAAM_comment ] = " *\n * 属性アクセスマクロ %s\n"
@@comment[ :CAAMI_comment ] = " *\n * 属性アクセスマクロ %s\n"
@@comment[ :TYP_comment ] = " *\n * 型 %s\n * CELLCB 型  : %s\n * CELLIDX 型 : %s\n"
@@comment[ :TCPF_comment ] = " * 呼び口関数 %s\n"
@@comment[ :TEPF_comment ] = "/* 受け口関数 %s */\n"
@@comment[ :TEFB_comment ] = "\t/* ここに処理本体を記述します %s */\n"

end
