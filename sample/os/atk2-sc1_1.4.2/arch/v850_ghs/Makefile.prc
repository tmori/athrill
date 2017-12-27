#
#  TOPPERS ATK2
#      Toyohashi Open Platform for Embedded Real-Time Systems
#      Automotive Kernel Version 2
#
#  Copyright (C) 2012-2016 by Center for Embedded Computing Systems
#              Graduate School of Information Science, Nagoya Univ., JAPAN
#  Copyright (C) 2012-2014 by FUJI SOFT INCORPORATED, JAPAN
#  Copyright (C) 2012-2013 by Spansion LLC, USA
#  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
#  Copyright (C) 2012-2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
#  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
#  Copyright (C) 2012-2014 by Sunny Giken Inc., JAPAN
#  Copyright (C) 2012-2014 by TOSHIBA CORPORATION, JAPAN
#  Copyright (C) 2012-2014 by Witz Corporation, JAPAN
#
#  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
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
#  $Id: Makefile.prc 831 2017-09-23 02:12:19Z ertl-honda $
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
         -srec -kanji=euc -noobj \
         -no_callt \
         -prepare_dispose \
         -registermode=32 \
         -keeptempfiles \
         --no_commons \
         -dual_debug -G\
         -sda=all \
         -reserve_r2 \
         -large_sda
CDEFS := $(CDEFS) -DTOPPERS_LABEL_ASM
LDFLAGS := $(LDFLAGS) -Wl,-append


#
#  アーキテクチャの切り替え
#
ifeq ($(ARCH),V850E2V3)
	COPTS := $(COPTS) -cpu=v850e2v3
	CDEFS := $(CDEFS) -D__v850e2v3__
	KERNEL_ASMOBJS := $(KERNEL_ASMOBJS) Os_Lcfg_asm.o
endif
ifeq ($(ARCH),V850E3V5)
	CDEFS := $(CDEFS) -D__v850e3v5__
endif

#
#  コアタイプ(実装)による切り替え
#
ifeq ($(CORETYPE),RH850G3K)
	COPTS := $(COPTS) -cpu=rh850g3k
endif
ifeq ($(CORETYPE),RH850G3M)
	COPTS := $(COPTS) -cpu=rh850g3m
	ifeq ($(USE_HARD_FLOAT),true)
		COPTS := $(COPTS) -DTOPPERS_USE_HFLOAT
	else
		COPTS := $(COPTS) -fsoft
	endif
endif
ifeq ($(CORETYPE),RH850G3KH)
	COPTS := $(COPTS) -cpu=rh850g3kh
	ifeq ($(USE_HARD_FLOAT),true)
		COPTS := $(COPTS) -DTOPPERS_USE_HFLOAT
	else
		COPTS := $(COPTS) -fsoft
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
KERNEL_ASMOBJS := $(KERNEL_ASMOBJS) prc_support.o
KERNEL_COBJS := $(KERNEL_COBJS) prc_config.o prc_tool.o

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

LDFLAGS := $(LDFLAGS) -e __reset -nostartfiles
CFG1_OUT_LDFLAGS := -e __start

#
#  依存関係の定義
#
cfg1_out.c: $(PRCDIR_GCC)/prc_def.csv
Os_Lcfg.timestamp: $(PRCDIR_GCC)/prc.tf
$(OBJFILE): $(PRCDIR_GCC)/prc_check.tf
offset.h: $(PRCDIR_GCC)/prc_offset.tf

#
#  ジェネレータ関係の変数の定義
#
CFG_TABS := $(CFG_TABS) --cfg1-def-table $(PRCDIR_GCC)/prc_def.csv
CFG2_OUT := Os_Lcfg.S

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

NM = nm

#
#  clean で削除するファイル
#
CLEAN_FILES := $(CLEAN_FILES) *.dbo *.dla *.map *.run *.dnm *.dep *.dba *.si *.inf *.s
