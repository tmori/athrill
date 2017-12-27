#
#  TOPPERS ATK2
#      Toyohashi Open Platform for Embedded Real-Time Systems
#      Automotive Kernel Version 2
#
#  Copyright (C) 2012-2015 by Center for Embedded Computing Systems
#              Graduate School of Information Science, Nagoya Univ., JAPAN
#  Copyright (C) 2012-2014 by FUJI SOFT INCORPORATED, JAPAN
#  Copyright (C) 2012-2013 by Spansion LLC, USA
#  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
#  Copyright (C) 2012-2013 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
#  Copyright (C) 2012-2013 by Renesas Electronics Corporation, JAPAN
#  Copyright (C) 2012-2013 by Sunny Giken Inc., JAPAN
#  Copyright (C) 2012-2013 by TOSHIBA CORPORATION, JAPAN
#  Copyright (C) 2012-2013 by Witz Corporation, JAPAN
#  Copyright (C) 2013 by Embedded and Real-Time Systems Laboratory
#              Graduate School of Information Science, Nagoya Univ., JAPAN
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
#  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
#  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
#  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
#  用する者に対して，AUTOSARパートナーになることを求めている．
#
#  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
#  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
#  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
#  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
#  の責任を負わない．
#
#  $Id: Makefile.prc 182 2015-06-24 07:14:37Z t_ishikawa $
#

#
#		Makefile のプロセッサ依存部（V850用）
#

#
#  プロセッサ名，開発環境名の定義
#
PRC  = v850
TOOL = ghs

#
#  プロセッサ依存部ディレクトリ名の定義
#
PRCDIR = $(SRCDIR)/arch/$(PRC)_$(TOOL)
PRCDIR_GCC = $(SRCDIR)/arch/$(PRC)_gcc

#
#  コンパイルオプション
#
INCLUDES := $(INCLUDES) -I$(PRCDIR) -I$(PRCDIR_GCC) -I$(SRCDIR)/arch/$(TOOL) 
COPTS := $(COPTS) -ansi -preprocess_assembly_files \
         -srec -kanji=utf8 -noobj \
         -no_callt \
         -prepare_dispose \
         -registermode=32 \
         -keeptempfiles \
         --no_commons \
         -dual_debug -G\
         --no_slash_comment \
         -sda=0 \
         -reserve_r2 \
         -large_sda

LDFLAGS := $(LDFLAGS) -Wl,-append --allow_different_section_types

#
#  アーキテクチャの切り替え
#
ifeq ($(ARCH),V850E2V3)
	CDEFS := $(CDEFS) -D__v850e2v3__
	KERNEL_ASMOBJS := $(KERNEL_ASMOBJS) Os_Lcfg_asm.o
endif
ifeq ($(ARCH),V850E3V5)
	CDEFS := $(CDEFS) -D__v850e3v5__
endif

#
#  コアタイプ(実装)による切り替え
#
ifeq ($(CORETYPE),RH850G3M)
	COPTS := $(COPTS) -cpu=rh850g3m
	ifeq ($(USE_HARD_FLOAT),true)
		COPTS := $(COPTS) -mhard-float -DTOPPERS_USE_HFLOAT
	else
		COPTS := $(COPTS) -msoft-float
	endif
endif

#
#  最適化に関するオプション
#
# Makefile で -O2 を付けない
OMIT_OPTIMIZATION = true
COPTS := $(COPTS) -Ospeed -Omax -Olink -Ointerproc
#COPTS := $(COPTS) -Odebug

#
#  警告に関するオプション
#
COPTS := $(COPTS) --prototype_warnings  \
                   -Wimplicit-int \
                   -Wshadow \
                   -Wundef \
                   -Wtrigraphs \
                   --assembler_warnings \
                   -linker_warnings \
                   --diag_suppress 174,188,191,767,177,826

#
#  カーネルに関する定義
#
KERNEL_DIR := $(KERNEL_DIR) $(PRCDIR) $(PRCDIR_GCC)
KERNEL_ASMOBJS := $(KERNEL_ASMOBJS) prc_support.o prc_tool.o 
KERNEL_COBJS := $(KERNEL_COBJS) prc_config.o prc_mpu.o 

#
#  GNU開発環境のターゲットアーキテクチャの定義
#
GCC_TARGET = v850-elf

#
#  スタートアップモジュールに関する定義
#
START_OBJS := $(START_OBJS) start.o

$(START_OBJS): %.o: %.S
	$(CC) -c $(CFLAGS) $(KERNEL_CFLAGS) $<

$(START_OBJS:.o=.d): %.d: %.S
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) \
		-O "$(CFLAGS) $(KERNEL_CFLAGS)" $< >> Makefile.depend



#
#  リンクに関する設定
#
LDFLAGS := $(LDFLAGS) -e __reset -nostartfiles

CFG1_OUT_LDFLAGS := -nostartfiles
CFG1_OUT_LDFLAGS := $(CFG1_OUT_LDFLAGS) $(LDFLAGS)
CFG2_OUT_LDFLAGS := $(CFG2_OUT_LDFLAGS) $(LDFLAGS)
CFG3_OUT_LDFLAGS := $(CFG3_OUT_LDFLAGS) $(LDFLAGS)

# メモリ配置決定前の暫定的なリンカスクリプト
CFG2_OUT_LDSCRIPT = cfg2_out.ld
# メモリ配置決定時のリンカスクリプト
CFG3_OUT_LDSCRIPT = cfg3_out.ld
# 最終的なリンカスクリプト
LDSCRIPT = ldscript.ld
ifeq ($(ARCH),V850E2V3)
	USE_CFG_PASS3 = true
endif

$(LDSCRIPT): kernel_cfg.timestamp
$(CFG2_OUT_LDSCRIPT): kernel_cfg.timestamp

#
#  依存関係の定義
#
cfg1_out.c: $(PRCDIR_GCC)/prc_def.csv
Os_Lcfg.timestamp: $(PRCDIR)/prc.tf
$(OBJFILE): $(PRCDIR_GCC)/prc_mem.tf
offset.h: $(PRCDIR_GCC)/prc_offset.tf


#
#  ジェネレータ関係の変数の定義
#
CFG_TABS := $(CFG_TABS) --cfg1-def-table $(PRCDIR_GCC)/prc_def.csv

#
#  開発ツールのコマンド名の定義
#
ifeq ($(ARCH),V850E2V3)
	CC = ccv850.exe
	AR = ccv850.exe
endif

ifeq ($(ARCH),V850E3V5)
	CC = ccrh850.exe
	AR = ccrh850.exe
endif

NM = gnm -p -h -no_debug -no_dotdot -no_line

#
#  clean で削除するファイル
#
CLEAN_FILES := $(CLEAN_FILES) *.dbo *.dla *.map *.run *.dnm *.dep *.dba *.si *.inf
