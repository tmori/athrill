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
#  $Id: check_library.sh 2 2012-05-09 02:23:52Z nces-shigihara $
# 

# チェックライブラリ確認メインメニュー
check_library_main()
{
	if [ $USE_KERNEL_LIB = "true" ]
	then
		check_file_name=$KERNEL_LIB/libkernel.a
		check_file
		if [ $? -eq 1 ]
		then
			return
		fi
	fi
	while true
	do
		cat<<EOS

$DOUBLE_LINE
 $CHECK_LIBRARY_MENU
$DOUBLE_LINE
 1: $CHECK_ALL_FUNC_2_4
 2: $CHECK_EXC
 3: $CHECK_INT
 4: $CHECK_TIM
 r: $BACK_TO_MAIN_MENU
 q: $EXIT_TOOL
$SINGLE_LINE
EOS
echo -n " $INPUT_NO "
		# キー読み込み
		read key
		# 読み込んだキーによって分岐
		case ${key} in
			1)
			 check_all_function
			 ;;
			2)
			 check_exception
			 ;;
			3)
			 check_interrupt
			 ;;
			4)
			 check_timer
			 ;;
			r)
			 go_main_flg=1
			 ;;
			q)
			 exit 0
			 ;;
			*)
			 echo " $key $ERR_INVALID_NO"
			 ;;
		esac
		if [ $go_main_flg -eq 1 ]
		then
			go_main_flg=0
			break
		fi
		
	done
}

# 全機能チェックのメニュー
check_all_function()
{
	while true
	do
		cat<<EOS

$DOUBLE_LINE
 $CHECK_ALL_FUNC
$DOUBLE_LINE
 1: $CHECK_ALL_FUNC_ALL_BUILD
 2: $CHECK_ALL_FUNC_MKDIR
 3: $CHECK_ALL_FUNC_DEPEND
 4: $CHECK_ALL_FUNC_BUILD
 5: $CHECK_ALL_FUNC_CLEAN
 6: $CHECK_ALL_FUNC_REALCLEAN
 e: $RUN_EXEC_MODULE
 r: $BACK_TO_MAIN_MENU
 q: Quit
$SINGLE_LINE
EOS
echo -n " $INPUT_NO "
		# キー読み込み
		read key
		echo ""
		# 読み込んだキーによって分岐
		case ${key} in
			1)
			 header_double "$CHECK_ALL_FUNC_ALL_BUILD"
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$EXCEPTION_DIR_NAME $INTERRUPT_DIR_NAME $TIMER_DIR_NAME"
			 continuous_execute_for_check_library
			 ;;
			2)
			 header_double "$CHECK_ALL_FUNC_MKDIR"
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$EXCEPTION_DIR_NAME $INTERRUPT_DIR_NAME $TIMER_DIR_NAME"
			 make_directory_for_check_library
			 ;;
			3)
			 header_double "$CHECK_ALL_FUNC_DEPEND"
			 rule=$RULE_DEPEND
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$EXCEPTION_DIR_NAME $INTERRUPT_DIR_NAME $TIMER_DIR_NAME"
			 make_for_check_library
			 ;;
			4)
			 header_double "$CHECK_ALL_FUNC_BUILD"
			 rule=$RULE_BUILD
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$EXCEPTION_DIR_NAME $INTERRUPT_DIR_NAME $TIMER_DIR_NAME"
			 make_for_check_library
			 ;;
			5)
			 header_double "$CHECK_ALL_FUNC_CLEAN"
			 rule=$RULE_CLEAN
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$EXCEPTION_DIR_NAME $INTERRUPT_DIR_NAME $TIMER_DIR_NAME"
			 make_for_check_library
			 ;;
			6)
			 header_double "$CHECK_ALL_FUNC_REALCLEAN"
			 rule=$RULE_REALCLEAM
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$EXCEPTION_DIR_NAME $INTERRUPT_DIR_NAME $TIMER_DIR_NAME"
			 make_for_check_library
			 ;;
			e)
			 header_double "RUN_EXEC_MODULE"
			 kind="$CHECK_ALL_FUNC"
			 execute_module
			 ;;
			r)
			 go_main_flg=1
			 ;;
			q)
			 exit 0
			 ;;
			*)
			 echo " $key $ERR_INVALID_NO"
			 ;;
		esac
		if [ $go_main_flg -eq 1 ]
		then
			break
		fi
	done
}

# CPU例外ハンドラリのチェック
check_exception()
{
	while true
	do
		cat<<EOS

$DOUBLE_LINE
 $CHECK_EXC
$DOUBLE_LINE
 1: $CHECK_EXC_ALL_BUILD
 2: $CHECK_EXC_MKDIR
 3: $CHECK_EXC_DEPEND
 4: $CHECK_EXC_BUILD
 5: $CHECK_EXC_CLEAN
 6: $CHECK_EXC_REALCLEAN
 e: $RUN_EXEC_MODULE
 r: $BACK_TO_MAIN_MENU
 q: Quit
$SINGLE_LINE
EOS
echo -n " $INPUT_NO "
		# キー読み込み
		read key
		echo ""
		# 読み込んだキーによって分岐
		case ${key} in
			1)
			 header_double "$CHECK_EXC_ALL_BUILD"
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$EXCEPTION_DIR_NAME"
			 continuous_execute_for_check_library
			 ;;
			2)
			 header_double "$CHECK_EXC_MKDIR"
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$EXCEPTION_DIR_NAME"
			 make_directory_for_check_library
			 ;;
			3)
			 header_double "$CHECK_EXC_DEPEND"
			 rule=$RULE_DEPEND
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$EXCEPTION_DIR_NAME"
			 make_for_check_library
			 ;;
			4)
			 header_double "$CHECK_EXC_BUILD"
			 rule=$RULE_BUILD
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$EXCEPTION_DIR_NAME"
			 make_for_check_library
			 ;;
			5)
			 header_double "$CHECK_EXC_CLEAN"
			 rule=$RULE_CLEAN
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$EXCEPTION_DIR_NAME"
			 make_for_check_library
			 ;;
			6)
			 header_double "$CHECK_EXC_REALCLEAN"
			 rule=$RULE_REALCLEAM
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$EXCEPTION_DIR_NAME"
			 make_for_check_library
			 ;;
			e)
			 header_double "RUN_EXEC_MODULE"
			 kind=$CHECK_EXC
			 execute_module
			 ;;
			r)
			 go_main_flg=1
			 ;;
			q)
			 exit 0
			 ;;
			*)
			 echo " $key $ERR_INVALID_NO"
			 ;;
		esac
		if [ $go_main_flg -eq 1 ]
		then
			break
		fi
	done
}

# 割込みハンドラのチェック
check_interrupt()
{
	while true
	do
		cat<<EOS

$DOUBLE_LINE
 $CHECK_INT
$DOUBLE_LINE
 1: $CHECK_INT_ALL_BUILD
 2: $CHECK_INT_MKDIR
 3: $CHECK_INT_DEPEND
 4: $CHECK_INT_BUILD
 5: $CHECK_INT_CLEAN
 6: $CHECK_INT_REALCLEAN
 e: $RUN_EXEC_MODULE
 r: $BACK_TO_MAIN_MENU
 q: Quit
$SINGLE_LINE
EOS
echo -n " $INPUT_NO "
		# キー読み込み
		read key
		echo ""
		# 読み込んだキーによって分岐
		case ${key} in
			1)
			 header_double "$CHECK_INT_ALL_BUILD"
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$INTERRUPT_DIR_NAME"
			 continuous_execute_for_check_library
			 ;;
			2)
			 header_double "$CHECK_INT_MKDIR"
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$INTERRUPT_DIR_NAME"
			 make_directory_for_check_library
			 ;;
			3)
			 header_double "$CHECK_INT_DEPEND"
			 rule=$RULE_DEPEND
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$INTERRUPT_DIR_NAME"
			 make_for_check_library
			 ;;
			4)
			 header_double "$CHECK_INT_BUILD"
			 rule=$RULE_BUILD
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$INTERRUPT_DIR_NAME"
			 make_for_check_library
			 ;;
			5)
			 header_double "$CHECK_INT_CLEAN"
			 rule=$RULE_CLEAN
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$INTERRUPT_DIR_NAME"
			 make_for_check_library
			 ;;
			6)
			 header_double "$CHECK_INT_REALCLEAN"
			 rule=$RULE_REALCLEAM
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$INTERRUPT_DIR_NAME"
			 make_for_check_library
			 ;;
			e)
			 header_double "RUN_EXEC_MODULE"
			 kind=$CHECK_INT
			 execute_module
			 ;;
			r)
			 go_main_flg=1
			 ;;
			q)
			 exit 0
			 ;;
			*)
			 echo " $key $ERR_INVALID_NO"
			 ;;
		esac
		if [ $go_main_flg -eq 1 ]
		then
			break
		fi
	done
}

# タイマのチェック
check_timer()
{
	while true
	do
		cat<<EOS

$DOUBLE_LINE
 $CHECK_TIM
$DOUBLE_LINE
 1: $CHECK_TIM_ALL_BUILD
 2: $CHECK_TIM_MKDIR
 3: $CHECK_TIM_DEPEND
 4: $CHECK_TIM_BUILD
 5: $CHECK_TIM_CLEAN
 6: $CHECK_TIM_REALCLEAN
 e: $RUN_EXEC_MODULE
 r: $BACK_TO_MAIN_MENU
 q: Quit
$SINGLE_LINE
EOS
echo -n " $INPUT_NO "
		# キー読み込み
		read key
		echo ""
		# 読み込んだキーによって分岐
		case ${key} in
			1)
			 header_double "$CHECK_TIM_ALL_BUILD"
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$TIMER_DIR_NAME"
			 continuous_execute_for_check_library
			 ;;
			2)
			 header_double "$CHECK_TIM_MKDIR"
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$TIMER_DIR_NAME"
			 make_directory_for_check_library
			 ;;
			3)
			 header_double "$CHECK_TIM_DEPEND"
			 rule=$RULE_DEPEND
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$TIMER_DIR_NAME"
			 make_for_check_library
			 ;;
			4)
			 header_double "$CHECK_TIM_BUILD"
			 rule=$RULE_BUILD
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$TIMER_DIR_NAME"
			 make_for_check_library
			 ;;
			5)
			 header_double "$CHECK_TIM_CLEAN"
			 rule=$RULE_CLEAN
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$TIMER_DIR_NAME"
			 make_for_check_library
			 ;;
			6)
			 header_double "$CHECK_TIM_REALCLEAN"
			 rule=$RULE_REALCLEAM
			 root_dir=$CHECK_LIBRARY_ROOT
			 loop_dir_name="$TIMER_DIR_NAME"
			 make_for_check_library
			 ;;
			e)
			 header_double "RUN_EXEC_MODULE"
			 kind=$CHECK_TIM
			 execute_module
			 ;;
			r)
			 go_main_flg=1
			 ;;
			q)
			 exit 0
			 ;;
			*)
			 echo " $key $ERR_INVALID_NO"
			 ;;
		esac
		if [ $go_main_flg -eq 1 ]
		then
			break
		fi
	done
}

# フォルダ作成
make_directory_for_check_library()
{
	rm -rf $OBJECT_DIR/$root_dir/temp_dir_chk
	mkdir -p $OBJECT_DIR/$root_dir/temp_dir_chk
	cd $OBJECT_DIR/$root_dir/temp_dir_chk
	perl ../../../../configure $CONFIG_TEST_PROGRAM "$INCLUDE_DIR" &> /dev/null
	cd ../
	for dir_name in ${loop_dir_name[@]}
	do
		rm -rf $dir_name
		echo "$PRE_MAKE_DIRECTORY $dir_name $POST_MAKE_DIRECTORY"
		mkdir -p $dir_name
		cp -p temp_dir_chk/$MAKE_FILE_NAME ./$dir_name
		cp -p $TTSP_DIR/library/$PROFILE_NAME/check_library/$dir_name/* ./$dir_name
		cd $dir_name
		rename_scratch_program
		cd ../
	done
	rm -rf temp_dir_chk
	cd $TTSP_DIR
}

# Makefileを用いる処理の実行
make_for_check_library()
{
	check_dir_name=$OBJECT_DIR/$root_dir/
	check_dir
	if [ $? -eq 0 ]
	then
		for dir_name in ${loop_dir_name[@]}
		do
			case $rule in
				$RULE_DEPEND)
				 header_single "$MAKE_DEPEND $dir_name"
				 ;;
				$RULE_BUILD)
				 header_single "$MAKE_BUILD $dir_name"
				 ;;
				$RULE_CLEAN)
				 header_single "$MAKE_CLEAN $dir_name"
				 ;;
				$RULE_REALCLEAM)
				 header_single "$MAKE_REALCLEAN $dir_name"
				 ;;
			esac
			cd $OBJECT_DIR/$root_dir/
			appli_name=$APPLI_NAME
			make_for_no_manifest
		done
	fi
	cd $TTSP_DIR
}

# フォルダ毎に実行
continuous_execute_for_check_library()
{
	rm -rf $OBJECT_DIR/$root_dir/temp_dir_chk
	mkdir -p $OBJECT_DIR/$root_dir/temp_dir_chk
	cd $OBJECT_DIR/$root_dir/temp_dir_chk
	perl ../../../../configure $CONFIG_TEST_PROGRAM "$INCLUDE_DIR" &> /dev/null
	cd ../
	for dir_name in ${loop_dir_name[@]}
	do
		rm -rf $dir_name
		# フォルダ作成
		echo "$PRE_MAKE_DIRECTORY $dir_name $POST_MAKE_DIRECTORY"
		mkdir -p $dir_name
		cp -p temp_dir_chk/$MAKE_FILE_NAME ./$dir_name
		cp -p $TTSP_DIR/library/$PROFILE_NAME/check_library/$dir_name/* ./$dir_name

		# make depend実行
		rule=$RULE_DEPEND
		cd $dir_name
		rename_scratch_program
		appli_name=$APPLI_NAME
		substitution_file_name
		check_file
		if [ $? -ne 1 ]
		then
			make_for_common

			# make実行
			if [ $? -eq 0 ]
			then
				rule=$RULE_BUILD
				make_for_common
				echo ""
			else
				echo "$ERR_DEPEND_FAIL "$dir_name
				echo ""
			fi
		fi
		cd ../
	done
	rm -rf temp_dir_chk
	cd $TTSP_DIR
}

# 処理開始
check_library_main
