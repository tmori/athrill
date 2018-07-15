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
#  $Id: kernel_lib.sh 2 2012-05-09 02:23:52Z nces-shigihara $
# 

# カーネルライブラリ作成メインメニュー
kernel_lib_main()
{
	while true
	do
		cat<<EOS

$DOUBLE_LINE
 $KERNEL_LIBRARY_MENU
$DOUBLE_LINE
 1: $L_ALL
 2: $L_MKDIR
 3: $L_DEPEND
 4: $L_BUILD
 5: $L_CLEAN
 6: $L_REALCLEAN
 r: $BACK_TO_MAIN_MENU
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
			 header_double "$L_ALL"
			 build_kernel_library
			 ;;
			2)
			 header_double "$L_MKDIR"
			 rm -rf $KERNEL_LIB
			 mkdir -p $KERNEL_LIB
			 cd $KERNEL_LIB
			 perl ../../configure $CONFIG_KERNEL_LIB "$INCLUDE_DIR"
			 cd $TTSP_DIR
			 ;;
			3)
			 header_double "$L_DEPEND"
			 rule=$RULE_DEPEND
			 dir_name=$KERNEL_LIB
			 appli_name=$L_APPLI_NAME
			 make_for_no_manifest
			 ;;
			4)
			 header_double "$L_BUILD"
			 rule=$RULE_BUILD
			 dir_name=$KERNEL_LIB
			 appli_name=$L_APPLI_NAME
			 make_for_no_manifest
			 ;;
			5)
			 header_double "$L_CLEAN"
			 rule=$RULE_CLEAN
			 dir_name=$KERNEL_LIB
			 make_for_no_manifest
			 ;;
			6)
			 header_double "$L_REALCLEAN"
			 rule=$RULE_REALCLEAM
			 dir_name=$KERNEL_LIB
			 make_for_no_manifest
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

build_kernel_library()
{
	rm -rf $KERNEL_LIB
	mkdir -p $KERNEL_LIB
	cd $KERNEL_LIB
	perl ../../configure $CONFIG_KERNEL_LIB "$INCLUDE_DIR" 
	( make depend $MAKE_OPT KERNEL_COBJS="$KERNEL_COBJS_COMMON $KERNEL_COBJS_TARGET" 2>&1; echo $? >status_file ) | tee $RESULT_MAKE_DEPEND
	status=`cat status_file`
	rm -rf status_file
	if [ $status = 0 ]
	then
		make $MAKE_OPT KERNEL_COBJS="$KERNEL_COBJS_COMMON $KERNEL_COBJS_TARGET" 2>&1 | tee $RESULT_MAKE
	else
		echo "$ERR_DEPEND_FAIL "$KERNEL_LIB
	fi
	cd $TTSP_DIR
}

# 処理開始
kernel_lib_main
