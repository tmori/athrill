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
ARCH_PATH=

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
IRC_ARCH="local"

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
CONFIG_OPT=

#
# ターゲット依存部でKERNEL_COBJSへ追加するオブジェクトファイル
#
KERNEL_COBJS_TARGET="core_config.o target_config.o ttsp_target_timer.o ttsp_target_test.o"

#
# make depend / make の追加オプション(必要な場合のみ)
# (コーテーションを含むオプションは指定不可(例:"-d \"dir1 dir2\""))
#
MAKE_OPT=

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
	# skyeye定義
	MODULE_NAME="fmp.exe"
	SKYEYE_EXE="skyeye.exe"
	SKYEYE_CONF_PATH="../target/at91skyeye_gcc"

	# 
	# プロセッサ毎に実行するため，cygstart(cygwin)，もしくはrxvtを使用して
	# 実行モジュールを実行する
	# c : cygstartによりskyeye実行
	# r : rxvtによりskyeye実行
	#
	TERMINAL_SEL="c"
	RXVT_OPT="-geometry 80x30 -font FixedSys -bg black -fg white"

	# エラーメッセージ
	ERR_SKYEYE_PATH="Need to pass a path to skyeye"


	which $SKYEYE_EXE &> /dev/null
	if [ $? -eq 1 ]
	then
		echo "$ERR_SKYEYE_PATH"
		return
	fi
	check_file_name=$MODULE_NAME
	check_file
	if [ $? -eq 1 ]
	then
		return
	fi

	get_skyeye_conf
	which_sh=`which sh`
	which_script=`which script`
	get_log_time
	get_log_file_name
	create_log_file

	if [ $PROCESSOR_NUM -gt 1 ]
	then
		prc_num=$PROCESSOR_NUM
		while [ $prc_num -gt 0 ]
		do
			echo "$which_script -c \"$SKYEYE_EXE -c ${arr_skyeye_conf[$prc_num]} -e $MODULE_NAME\" -a ${arr_logfile[$prc_num]}" > pe${prc_num}_script.sh
			echo "exit" >> "pe"$prc_num"_script.sh"
			prc_num=`expr $prc_num - 1`
		done
		echo "exit" >> pe1_script.sh
	else
		echo "$which_script -c \"$SKYEYE_EXE -c ${arr_skyeye_conf[0]} -e $MODULE_NAME\" -a ${arr_logfile[1]}" > pe1_script.sh
		echo "exit" >> pe1_script.sh
		echo "exit" >> pe1_script.sh
	fi
	if [ $TERMINAL_SEL = "r" ]
	then
		if [ $PROCESSOR_NUM -gt 1 ]
		then
			prc_num=$PROCESSOR_NUM
			while [ $prc_num -gt 1 ]
			do
				echo "rxvt $RXVT_OPT -title PE$prc_num -e sh pe${prc_num}_script.sh &" > pe${prc_num}_rxvt.sh
				sh ./pe${prc_num}_rxvt.sh
				prc_num=`expr $prc_num - 1`
			done
			wait_skyeye `expr $PROCESSOR_NUM - 1`
			sleep 1
			rxvt $RXVT_OPT -title "PE1" -e sh pe1_script.sh
			sleep 1
		else
			rxvt $RXVT_OPT -title "PE1" -e sh pe1_script.sh
		fi
		kill_proc "rxvt"
	else
		if [ $PROCESSOR_NUM -gt 1 ]
		then
			prc_num=$PROCESSOR_NUM
			while [ $prc_num -gt 1 ]
			do
				cygstart --shownoactivate $which_sh "./pe"$prc_num"_script.sh"
				prc_num=`expr $prc_num - 1`
			done
			wait_skyeye `expr $PROCESSOR_NUM - 1`
			sleep 1
			cygstart -w --shownoactivate $which_sh ./pe1_script.sh
			sleep 1
		else
			cygstart -w --shownoactivate $which_sh ./pe1_script.sh
		fi
		kill_proc "cygstart"
	fi
	prc_num=1
	while [ $prc_num -le $PROCESSOR_NUM ]
	do
		grep "PE "$prc_num ${arr_logfile[$prc_num]} | tail -n 1
		if [ "$kind" = "$SCRATCH" ]
		then
			success=`grep "All" ${arr_logfile[$prc_num]} | wc -l`
			success_num=`expr $success_num + $success`
		fi
		sed -i "s/\r\r\n/\r\n/g" ${arr_logfile[$prc_num]}
		prc_num=`expr $prc_num + 1`
	done
	rm -f ./pe*.sh
}

# skyeye_conf取得
get_skyeye_conf()
{
	skyeye_conf_path="$TTSP_DIR/$SKYEYE_CONF_PATH"
	if [ $PROCESSOR_NUM -gt 1 ]
	then
		prc_num=1
		while [ $prc_num -le $PROCESSOR_NUM ]
		do
			arr_skyeye_conf[$prc_num]=${skyeye_conf_path}/skyeye_pe${prc_num}.conf
			prc_num=`expr $prc_num + 1`
		done
	else
		arr_skyeye_conf[0]=${skyeye_conf_path}/skyeye_single.conf
	fi
}

# 時間取得
get_log_time()
{
	LOGTIME=`date '+%Y-%m-%d_%H-%M-%S'`
}

# ログファイル名取得
get_log_file_name()
{
	arr_logfile[0]=offset
	prc_num=1
	while [ $prc_num -le $PROCESSOR_NUM ]
	do
		arr_logfile[$prc_num]=`echo "skyeye_"$LOGTIME"_"$prc_num".log"`
		prc_num=`expr $prc_num + 1`
	done
}

# ログファイル生成
create_log_file()
{
	prc_num=1
	while [ $prc_num -le $PROCESSOR_NUM ]
	do
		echo $SINGLE_LINE >> ${arr_logfile[$prc_num]}
		echo $dir_name >> ${arr_logfile[$prc_num]}
		echo $SINGLE_LINE >> ${arr_logfile[$prc_num]}
		prc_num=`expr $prc_num + 1`
	done
}

# プロセッサ1が起動する前に，他のプロセッサが起動するまで待機
wait_skyeye()
{
	chk=0
	while [ $chk -ne $1 ]
	do
		chk=`ps ax | grep skyeye | wc -l`
		sleep 1
	done
}

# プロセスを終了
kill_proc()
{
	pid=`ps ax | grep $1 | awk -F ' ' '{print $1}'`
	for id in ${pid[@]}
	do
		kill -9 $id
	done
}

# スクラッチコードテストの結果出力
output_scr_test_result()
{
	header_single "$TEST_RESULT"
	echo "$NUM_OF_TEST: "$test_num
	echo "$NUM_OF_ALL_PASS: "$success_num
}
