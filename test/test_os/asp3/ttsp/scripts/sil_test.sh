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
#  $Id: sil_test.sh 2 2012-05-09 02:23:52Z nces-shigihara $
# 

# SILテストメニュー
sil_test_main()
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
 $SIL_TEST_MENU
$DOUBLE_LINE
 1: $SIL_ALL_BUILD
 2: $SIL_MAKE_DIR
 3: $SIL_MAKE_DEPEND
 4: $SIL_MAKE_BUILD
 5: $SIL_MAKE_CLEAN
 6: $SIL_MAKE_REALCLEAN
 e: $RUN_EXEC_MODULE
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
			 header_double "$SIL_ALL_BUILD"
			 dir_name=$SIL_TEST_ROOT
			 continuous_execute_for_sil_test
			 ;;
			2)
			 header_double "$SIL_MAKE_DIR"
			 dir_name=$SIL_TEST_ROOT
			 make_directory_for_sil_test
			 ;;
			3)
			 header_double "$SIL_MAKE_DEPEND"
			 rule=$RULE_DEPEND
			 dir_name=$SIL_TEST_ROOT
			 make_for_sil_test
			 ;;
			4)
			 header_double "$SIL_MAKE_BUILD"
			 rule=$RULE_BUILD
			 dir_name=$SIL_TEST_ROOT
			 make_for_sil_test
			 ;;
			5)
			 header_double "$SIL_MAKE_CLEAN"
			 rule=$RULE_CLEAN
			 dir_name=$SIL_TEST_ROOT
			 make_for_sil_test
			 ;;
			6)
			 header_double "$SIL_MAKE_REALCLEAN"
			 rule=$RULE_REALCLEAM
			 dir_name=$SIL_TEST_ROOT
			 make_for_sil_test
			 ;;
			e)
			 header_double "RUN_EXEC_MODULE"
			 kind="$SIL_TEST_MENU"
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
			go_main_flg=0
			break
		fi
		
	done
}

# フォルダ作成
make_directory_for_sil_test()
{
	rm -rf $OBJECT_DIR/$dir_name
	echo "$PRE_MAKE_DIRECTORY $dir_name $POST_MAKE_DIRECTORY"
	mkdir -p $OBJECT_DIR/$dir_name
	cd $OBJECT_DIR/$dir_name
	perl ../../../configure $CONFIG_TEST_PROGRAM "$INCLUDE_DIR" &> /dev/null
	cp -p $TTSP_DIR/$SIL_TEST_DIR/* .
	rename_scratch_program
	cd $TTSP_DIR
}

# Makefileを用いる処理の実行
make_for_sil_test()
{
	check_dir_name=$OBJECT_DIR/$dir_name/
	check_dir
	if [ $? -eq 0 ]
	then
		cd $OBJECT_DIR/$dir_name/
		case $rule in
			$RULE_DEPEND)
			 check_file_name="$APPLI_NAME.c \
			                  $APPLI_NAME.h \
			                  $APPLI_NAME.cfg \
			                  $MAKE_FILE_NAME"
			 ;;
			$RULE_BUILD)
			 check_file_name="$APPLI_NAME.c \
			                  $APPLI_NAME.h \
			                  $APPLI_NAME.cfg \
			                  $MAKE_FILE_NAME"
			 ;;
			$RULE_CLEAN)
			 check_file_name="$MAKE_FILE_NAME"
			 ;;
			$RULE_REALCLEAM)
			 check_file_name="$MAKE_FILE_NAME"
			 ;;
		esac
		check_file
		make_for_common
	fi
	cd $TTSP_DIR

}

# フォルダ作成、make depend、make実行
continuous_execute_for_sil_test()
{
	rm -rf $OBJECT_DIR/$dir_name
	echo "$PRE_MAKE_DIRECTORY $dir_name $POST_MAKE_DIRECTORY"
	mkdir -p $OBJECT_DIR/$dir_name
	cd $OBJECT_DIR/$dir_name
	perl ../../../configure $CONFIG_TEST_PROGRAM "$INCLUDE_DIR" &> /dev/null
	cp -p $TTSP_DIR/$SIL_TEST_DIR/* .
	rename_scratch_program

	# make depend実行
	rule=$RULE_DEPEND
	check_file_name="$APPLI_NAME.c \
                     $APPLI_NAME.h \
                     $APPLI_NAME.cfg \
                     $MAKE_FILE_NAME"
	check_file
	if [ $? -ne 1 ]
	then
		make_for_common

		# make実行
		if [ $? -eq 0 ]
		then
			rule=$RULE_BUILD
			make_for_common
		else
			echo "$ERR_DEPEND_FAIL "$dir_name
		fi
	fi
	cd $TTSP_DIR
}

# 処理開始
sil_test_main
