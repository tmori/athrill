
	TTSP - TOPPERS Test Suite Package -
	ASPカーネル apsh2a_gcc依存部 設定ガイド

----------------------------------------------------------------------
 TTSP
     TOPPERS Test Suite Package

 Copyright (C) 2010-2011 by Center for Embedded Computing Systems
             Graduate School of Information Science, Nagoya Univ., JAPAN
 Copyright (C) 2010-2011 by Digital Craft Inc.
 Copyright (C) 2010-2011 by NEC Communication Systems, Ltd.
 Copyright (C) 2010-2011 by FUJISOFT INCORPORATED
 Copyright (C) 2010-2011 by Industrial Technology Institute,
                                 Miyagi Prefectural Government, JAPAN

 上記著作権者は，以下の (1)〜(3) の条件を満たす場合に限り，本ドキュメ
 ント（本ドキュメントを改変したものを含む．以下同じ）を使用・複製・改
 変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 (1) 本ドキュメントを利用する場合には，上記の著作権表示，この利用条件
     および下記の無保証規定が，そのままの形でドキュメント中に含まれて
     いること．
 (2) 本ドキュメントを改変する場合には，ドキュメントを改変した旨の記述
     を，改変後のドキュメント中に含めること．ただし，改変後のドキュメ
     ントが，TOPPERSプロジェクト指定の開発成果物である場合には，この限
     りではない．
 (3) 本ドキュメントの利用により直接的または間接的に生じるいかなる損害
     からも，上記著作権者およびTOPPERSプロジェクトを免責すること．また，
     本ドキュメントのユーザまたはエンドユーザからのいかなる理由に基づ
     く請求からも，上記著作権者およびTOPPERSプロジェクトを免責すること．

 本ドキュメントは，無保証で提供されているものである．上記著作権者およ
 びTOPPERSプロジェクトは，本ドキュメントに関して，特定の使用目的に対す
 る適合性も含めて，いかなる保証も行わない．また，本ドキュメントの利用
 により直接的または間接的に生じたいかなる損害に関しても，その責任を負
 わない．

 $Id: README.txt 2 2012-05-09 02:23:52Z nces-shigihara $
----------------------------------------------------------------------
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ターゲット名称の変更
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ttsp/configure.sh 48行目
修正前：TARGET_NAME="at91skyeye_gcc"
修正後：TARGET_NAME="apsh2ad_gcc"


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
テストライブラリをカーネルライブラリへ追加
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ttsp_target_test.oとttsp_target_mtu2.oをコンパイル対象に追加する処理が
TTSP側に記述されており、カーネル側は修正する必要がない。


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
その他
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
変数USE_KERNEL_LIBをtrueに設定すると、ライブラリ形式でカーネルがリンク
され、複数の実行ファイルを構築する際にビルドが高速化される。

ttsp/configure.sh 67行目
#
# カーネルライブラリを使用するか
# [true : 使用する，false : 使用しない]
#
USE_KERNEL_LIB="true"


以上．
