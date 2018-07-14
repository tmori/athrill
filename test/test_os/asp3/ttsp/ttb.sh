#!/bin/sh
#
#  TTSP
#      TOPPERS Test Suite Package
# 
#  Copyright (C) 2009-2012 by Center for Embedded Computing Systems
#              Graduate School of Information Science, Nagoya Univ., JAPAN
#  Copyright (C) 2009-2011 by Digital Craft Inc.
#  Copyright (C) 2009-2011 by NEC Communication Systems, Ltd.
#  Copyright (C) 2009-2012 by FUJISOFT INCORPORATED
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
#  $Id: ttb.sh 2 2012-05-09 02:23:52Z nces-shigihara $
# 

# カレント実行チェック
chk=`dirname $0`
if [ $chk != "." ]
then
	echo "Need to execute at a directory of existing \"ttb.sh\""
	exit 1
fi

# ttspフォルダの絶対パス
TTSP_DIR=`pwd`

# TTB実行に必要なフォルダ
NECESSARY_DIR="\
api_test \
library \
scripts \
sil_test \
tools"

# TTB実行に必要なフォルダの有無をチェック
for dir in ${NECESSARY_DIR[@]}
do
	if [ ! -d $dir ]
	then
		echo "$dir is necessary directory"
		exit 1
	fi
done

# 設定ファイルの読込み
source ./scripts/common.sh
source ./configure.sh

# TTBターゲット依存設定ファイルのインクルード
source ./library/$PROFILE_NAME/target/$TARGET_NAME/ttsp_target.sh

#
# プロファイル毎の設定
#
if [ $PROFILE_NAME = "ASP" ]
then
	API_TEST_DIR="api_test/ASP"
	SIL_TEST_DIR="sil_test/ASP"
	KERNEL_COBJS_COMMON="startup.o task.o wait.o time_event.o task_manage.o task_refer.o task_sync.o task_except.o semaphore.o eventflag.o dataqueue.o pridataq.o mailbox.o mempfix.o time_manage.o cyclic.o alarm.o sys_manage.o interrupt.o exception.o"
elif [ $PROFILE_NAME = "FMP" ]
then
	API_TEST_DIR="api_test"
	SIL_TEST_DIR="sil_test/FMP"
	CONFIG_PRC="-P $PROCESSOR_NUM"
	KERNEL_COBJS_COMMON="startup.o task.o wait.o time_event.o task_manage.o task_refer.o task_sync.o task_except.o semaphore.o eventflag.o dataqueue.o pridataq.o mailbox.o mempfix.o time_manage.o cyclic.o alarm.o sys_manage.o interrupt.o exception.o spin_lock.o mp.o"
else
	echo "$ERR_PROFILE_INVALID ($PROFILE_NAME)"
	exit 1
fi

#
# INCLUDE対象のパス定義
#
TTSP_DIR_NAME=${TTSP_DIR##*/}

TEST_LIB_FILE="ttsp_test_lib.o"
INCLUDE_DIR=`echo '\$(SRCDIR)/'$TTSP_DIR_NAME'/library/'$PROFILE_NAME'/test \$(SRCDIR)/'$TTSP_DIR_NAME'/library/'$PROFILE_NAME'/target/'$TARGET_NAME`
ARCH_DIR=`echo '\$(SRCDIR)/'$TTSP_DIR_NAME'/library/'$PROFILE_NAME'/arch'`
for dir in ${ARCH_PATH[@]}
do
	INCLUDE_DIR=`echo "$INCLUDE_DIR $ARCH_DIR/$dir"`
done


#
# configureオプション
#
CONFIG_KERNEL_LIB=`echo "-T $TARGET_NAME -U $TEST_LIB_FILE $CONFIG_PRC $CONFIG_OPT -a "`
if [ $USE_KERNEL_LIB = true ]
then
	CONFIG_TEST_PROGRAM=`echo '-T '$TARGET_NAME' -A '$APPLI_NAME' -L \$(SRCDIR)/'$TTSP_DIR_NAME'/'$KERNEL_LIB' -U '$TEST_LIB_FILE $CONFIG_PRC $CONFIG_OPT' -a'`
elif [ $USE_KERNEL_LIB = false ]
then
	CONFIG_TEST_PROGRAM=`echo '-T '$TARGET_NAME' -A '$APPLI_NAME' -U '$TEST_LIB_FILE $CONFIG_PRC $CONFIG_OPT' -a'`
else
	echo "$ERR_USE_KER_LIB_INVALID ($USE_KERNEL_LIB)"
	exit 1
fi

# TTSPメインメニュー
ttsp_main_menu()
{
	go_main_flg=0
	go_api_test_flg=0

	check_makefile

	while true
	do
		cat<<EOS

$DOUBLE_LINE
 $MAIN_MENU
$DOUBLE_LINE
 1: $API_TEST_MENU
 2: $SIL_TEST_MENU
 c: $CHECK_LIBRARY_MENU
 k: $KERNEL_LIBRARY_MENU
 q: $EXIT_TOOL
$SINGLE_LINE
EOS
echo -n " $INPUT_NO "
		# キー読み込み
		read key
		echo ""
		# 読み込んだキーによって分岐
		case ${key} in
			1)
			 source ./scripts/api_test.sh
			 ;;
			2)
			 source ./scripts/sil_test.sh
			 ;;
			c)
			 source ./scripts/check_library.sh
			 ;;
			k)
			 source ./scripts/kernel_lib.sh
			 ;;
			q)
			 exit 0
			 ;;
			*)
			 echo " $key $ERR_INVALID_NO"
			 ;;
		esac
	done
	exit 0
}

# 処理開始
ttsp_main_menu
