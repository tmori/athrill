#
#  TTSP
#      TOPPERS Test Suite Package
#
#  Copyright (C) 2009-2011 by Center for Embedded Computing Systems
#              Graduate School of Information Science, Nagoya Univ., JAPAN
#  Copyright (C) 2009-2011 by Digital Craft Inc.
#  Copyright (C) 2009-2011 by NEC Communication Systems, Ltd.
#  Copyright (C) 2009-2011 by FUJISOFT INCORPORATED
#
#  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
#  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
#  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
#  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
#      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
#      スコード中に含まれていること．
#  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
#      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
#      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
#      の無保証規定を掲載すること．
#  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
#      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
#      と．
#    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
#        作権表示，この利用条件および下記の無保証規定を掲載すること．
#    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
#        報告すること．
#  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
#      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
#      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
#      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
#      免責すること．
#
#  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
#  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
#  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
#  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
#  の責任を負わない．
#
#  $Id: ttsp_target.sh 2 2012-05-09 02:23:52Z nces-shigihara $
#

#
# アプリケーション名
#
APPLI_NAME="out"

#
# アーキテクチャフォルダ(ttsp/library/ASP(FMP)/arch配下の使用するフォルダ)の定義
# (必要な場合のみ)
# (複数指定可能，複数指定する場合はスペースで区切る)
#  例) "arm_gcc/mpcore arm_gcc/common"
#
ARCH_PATH="arm_gcc/mpcore"

#
# プロセッサ数(FMPカーネルのみ使用)
#
PROCESSOR_NUM=4

#
# タイマアーキテクチャ(FMPカーネルのみ使用)
# [local : ローカルタイマ方式，global : グローバルタイマ方式]
#
TIMER_ARCH="local"

#
# ターゲット依存APIの有無
# [true: 有り，false: 無し]
#
FUNC_TIME="true"		# システム時刻制御関数
FUNC_INTERRUPT="true"	# 割込み発生関数
FUNC_EXCEPTION="true"	# CPU例外発生関数

#
# IRCアーキテクチャ(FMPカーネルのみ使用)
# [local: ローカルIRCのみサポート，global: グローバルIRCのみサポート，
#   combination: ローカルIRC，グローバルIRC両方サポート]
#
IRC_ARCH="combination"

#
# TTGへの追加オプション
# (-a，-f，-c，--prc_num，--timer_arch，--out_file_name，--func_time，
#  --func_interrupt，--func_exception, --irc_archは使用不可，詳細はttsp/user.txtを参照)
#
TTG_OPT=

#
# コンフィギュレータへの追加オプション(必要な場合のみ)
# (-T, -A, -U, -a, -LはTTSPで使用するため使用不可)
# (コーテーションを含むオプションは指定不可(例:"-d \"dir1 dir2\""))
#
CONFIG_OPT="-s"

#
# ターゲット依存部でKERNEL_COBJSへ追加するオブジェクトファイル
#
KERNEL_COBJS_TARGET="target_config.o chip_config.o mpcore.o core_config.o ttsp_chip_timer.o ttsp_target_test.o"

#
# make depend / make の追加オプション(必要な場合のみ)
# (コーテーションを含むオプションは指定不可(例:"-d \"dir1 dir2\""))
#
MAKE_OPT="ENABLE_QEMU=true ENABLE_G_SYSLOG=false"

#
# 実行モジュールの実行をシェルスクリプトで実装済みか
# [true : 実装済み，false : 実装していない]
# (trueの場合，ttb.shの"e: Run executable module (Target Dependent)"を
#  選択すると，以下の関数simulation()が実行される)
#
EXC_MODULE="true"

#
# 実行モジュールの実行(ターゲット依存)
# (EXC_MODULEがtrueの場合に実行される)
#
simulation()
{
        qemu-system-arm -cpu cortex-a9 -M vexpress-a9 -smp $PROCESSOR_NUM -serial vc:80Cx40C -serial vc:80Cx40C -serial vc:80Cx40C -serial vc:80Cx40C -no-reboot -icount auto -m 1024M -kernel fmp
}
