#
#  TOPPERS ATK2
#      Toyohashi Open Platform for Embedded Real-Time Systems
#      Automotive Kernel Version 2
#
#  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
#                              Toyohashi Univ. of Technology, JAPAN
#  Copyright (C) 2006-2015 by Center for Embedded Computing Systems
#              Graduate School of Information Science, Nagoya Univ., JAPAN
#  Copyright (C) 2011-2015 by FUJI SOFT INCORPORATED, JAPAN
#  Copyright (C) 2011-2013 by Spansion LLC, USA
#  Copyright (C) 2011-2015 by NEC Communication Systems, Ltd., JAPAN
#  Copyright (C) 2011-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
#  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
#  Copyright (C) 2011-2015 by Sunny Giken Inc., JAPAN
#  Copyright (C) 2011-2015 by TOSHIBA CORPORATION, JAPAN
#  Copyright (C) 2011-2015 by Witz Corporation
#  Copyright (C) 2014-2015 by AISIN COMCRUISE Co., Ltd., JAPAN
#  Copyright (C) 2014-2015 by eSOL Co.,Ltd., JAPAN
#  Copyright (C) 2014-2015 by SCSK Corporation, JAPAN
#  Copyright (C) 2015 by SUZUKI MOTOR CORPORATION
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
#  $Id: Makefile 495 2015-12-21 04:55:08Z ertl-ishikawa $
#

include Makefile.common

#
#  ターゲットの指定（Makefile.targetで上書きされるのを防ぐため）
#
all:

#
#  ターゲット略称の定義
#
TARGET = v850esfk3_gcc

#
#  プログラミング言語の定義
#
SRCLANG = c
ifeq ($(SRCLANG),c++)
  USE_CXX = true
  CXXLIBS = -lstdc++ -lm -lc
  CXXRTS = cxxrt.o newlibrt.o
endif

#
#  ソースファイルのディレクトリの定義
#
SRCDIR = ..

#
#  オブジェクトファイル名の拡張子の設定
#
OBJEXT = 

#
#  実行環境の定義（ターゲット依存に上書きされる場合がある）
#
DBGENV := 

#
#  カーネルライブラリ（libkernel.a）のディレクトリ名
#  （カーネルライブラリもmake対象にする時は，空に定義する）
#
KERNEL_LIB = 

#
#  カーネルを関数単位でコンパイルするかどうかの定義
#
KERNEL_FUNCOBJS = 

#
#  トレースログを取得するかどうかの定義
#
ENABLE_TRACE = 

#
#  システムタイマ(タイマドライバ)を使用するかどうかの定義
#
ENABLE_SYS_TIMER = 

#
#  ハードウェアカウンタを無効にするかどうかの定義
#
OMIT_HW_COUNTER = 

#
#  トレースログに関する設定
#
ifeq ($(ENABLE_TRACE),true)
      ENABLE_SYS_TIMER = true
      CDEFS := $(CDEFS) -DTOPPERS_ENABLE_TRACE
      KERNEL_DIR := $(KERNEL_DIR) $(SRCDIR)/arch/logtrace
      KERNEL_COBJS := $(KERNEL_COBJS) trace_config.o trace_dump.o
endif

#  コンフィギュレーションファイルに関する定義
#
CFGNAME = sample1
CFGNAME := $(CFGNAME) uart
ifdef ENABLE_SYS_TIMER
  CFGNAME := $(CFGNAME) target_timer
endif
ifndef OMIT_HW_COUNTER
  CFGNAME := $(CFGNAME) target_hw_counter
endif

#
#  ユーティリティプログラムの名称
#
PERL = /usr/bin/perl
CFG = $(SRCDIR)/cfg/cfg/cfg

#
#  オブジェクトファイル名の定義
#
OBJNAME = atk2-sc1
ifdef OBJEXT
  OBJFILE = $(OBJNAME).$(OBJEXT)
  CFG1_OUT = cfg1_out.$(OBJEXT)
else
  OBJFILE = $(OBJNAME)
  CFG1_OUT = cfg1_out
endif

#
#  ターゲット依存部のディレクトリの定義
#
TARGETDIR = $(SRCDIR)/target/$(TARGET)

#
#  ターゲット依存の定義のインクルード
#
include $(TARGETDIR)/Makefile.target

#
#  ジェネレータ関係の変数の定義
#
PASS2_TF = $(SRCDIR)/kernel/kernel.tf
DEF_TABS = $(SRCDIR)/kernel/kernel.csv
CFG_KERNEL = atk2
CFG_TABS := --ini-file $(SRCDIR)/kernel/kernel.ini $(CFG_TABS)

CFG_TABS := --api-table $(DEF_TABS) \
			--cfg1-def-table $(SRCDIR)/kernel/kernel_def.csv $(CFG_TABS)
CFG_OBJS := Os_Lcfg.o $(CFG_OBJS)
CFG2_OUT := Os_Lcfg.c Os_Lcfg.h Os_Cfg.h $(CFG2_OUT)
ALL_CFG_OBJS := $(CFG_OBJS) cfg1_out.o

#
#  共通コンパイルオプションの定義
#
COPTS := $(COPTS) -g 
ifndef OMIT_WARNING_ALL
  COPTS := $(COPTS) -Wall -Wno-unused-label -Wpointer-arith
endif
ifndef OMIT_OPTIMIZATION
  COPTS := $(COPTS) -O0  -gdwarf-2
endif

INCLUDES := -I. -I$(SRCDIR)/include -I$(SRCDIR)/arch -I$(SRCDIR) $(INCLUDES)
LDFLAGS := $(LDFLAGS) 
LIBS := $(LIBS) $(CXXLIBS)
CFLAGS = $(COPTS) $(CDEFS) $(INCLUDES)

#
#  アプリケーションプログラムに関する定義
#
APPLNAME = sample1
APPLDIR = 
APPL_CFG_INPUT := $(foreach file,$(CFGNAME),$(file).arxml)

APPL_DIR = $(APPLDIR) $(SRCDIR)/library
APPL_ASMOBJS =
ifdef USE_CXX
  APPL_CXXOBJS = $(APPLNAME).o 
  APPL_COBJS =
else
  APPL_COBJS = $(APPLNAME).o 
endif
APPL_CFLAGS =
APPL_LIBS =
ifdef APPLDIR
  INCLUDES := $(INCLUDES) $(foreach dir,$(APPLDIR),-I$(dir))
endif


#
#  システムモジュールに関する定義
#
SYSMOD_DIR := $(SYSMOD_DIR) $(SRCDIR)/sysmod $(SRCDIR)/library
SYSMOD_ASMOBJS := $(SYSMOD_ASMOBJS)
SYSMOD_COBJS := $(SYSMOD_COBJS) banner.o syslog.o serial.o \
				log_output.o vasyslog.o t_perror.o strerror.o \
				$(CXXRTS)
SYSMOD_CFLAGS := $(SYSMOD_CFLAGS)
SYSMOD_LIBS := $(SYSMOD_LIBS)
INCLUDES := $(INCLUDES)

#
#  カーネルに関する定義
#
#  KERNEL_ASMOBJS: カーネルライブラリに含める，ソースがアセンブリ言語の
#				   オブジェクトファイル．
#  KERNEL_COBJS: カーネルのライブラリに含める，ソースがC言語で，ソース
#				 ファイルと1対1に対応するオブジェクトファイル．
#  KERNEL_LCSRCS: カーネルのライブラリに含めるC言語のソースファイルで，
#				  1つのソースファイルから複数のオブジェクトファイルを生
#				  成するもの．
#  KERNEL_LCOBJS: 上のソースファイルから生成されるオブジェクトファイル．
#  KERNEL_AUX_COBJS: ロードモジュールに含めないが，カーネルのソースファ
#					 イルと同じオプションを適用してコンパイルすべき，ソー
#					 スがC言語のオブジェクトファイル．
#
KERNEL_DIR := $(KERNEL_DIR) $(SRCDIR)/kernel
KERNEL_ASMOBJS := $(KERNEL_ASMOBJS)
KERNEL_COBJS := $(KERNEL_COBJS)
KERNEL_CFLAGS := $(KERNEL_CFLAGS) -I$(SRCDIR)/kernel
ifdef OMIT_MAKEOFFSET
  OFFSET_H =
else
  OFFSET_H = offset.h
endif

#
#  ターゲットファイル（複数を同時に選択してはならない）
#
all: $(OBJFILE)
#all: $(OBJNAME).bin
#all: $(OBJNAME).srec

##### 以下は編集しないこと #####

#
#  環境に依存するコンパイルオプションの定義
#
ifdef DBGENV
  CDEFS := $(CDEFS) -D$(DBGENV)
endif

#
#  カーネルのファイル構成の定義
#
include $(SRCDIR)/kernel/Makefile.kernel
ifdef KERNEL_FUNCOBJS
  KERNEL_LCSRCS := $(KERNEL_FCSRCS)
  KERNEL_LCOBJS := $(foreach file,$(KERNEL_FCSRCS),$($(file:.c=)))
else
  KERNEL_CFLAGS := -DALLFUNC $(KERNEL_CFLAGS)
  KERNEL_COBJS := $(KERNEL_COBJS) \
					$(foreach file,$(KERNEL_FCSRCS),$(file:.c=.o))
endif

#
#  ソースファイルのあるディレクトリに関する定義
#
vpath %.c $(KERNEL_DIR) $(SYSMOD_DIR) $(APPL_DIR)
vpath %.S $(KERNEL_DIR) $(SYSMOD_DIR) $(APPL_DIR)
vpath %.arxml $(KERNEL_DIR) $(SYSMOD_DIR) $(APPL_DIR)

#
#  コンパイルのための変数の定義
#
KERNEL_LIB_OBJS = $(KERNEL_ASMOBJS) $(KERNEL_COBJS) $(KERNEL_LCOBJS)
SYSMOD_OBJS = $(SYSMOD_ASMOBJS) $(SYSMOD_COBJS)
APPL_OBJS = $(APPL_ASMOBJS) $(APPL_COBJS) $(APPL_CXXOBJS)
ALL_OBJS = $(START_OBJS) $(APPL_OBJS) $(SYSMOD_OBJS) $(CFG_OBJS) \
											$(END_OBJS) $(HIDDEN_OBJS)
ifdef KERNEL_LIB
  ALL_LIBS = $(APPL_LIBS) $(SYSMOD_LIBS) -lkernel $(LIBS)
  LIBS_DEP = $(filter %.a,$(ALL_LIBS)) $(KERNEL_LIB)/libkernel.a
  LDFLAGS := $(LDFLAGS) -L$(KERNEL_LIB)
else
  ALL_LIBS = $(APPL_LIBS) $(SYSMOD_LIBS) libkernel.a $(LIBS)
  LIBS_DEP = $(filter %.a,$(ALL_LIBS))
endif

ifdef TEXT_START_ADDRESS
  LDFLAGS := $(LDFLAGS) -Wl,-Ttext,$(TEXT_START_ADDRESS)
endif
ifdef DATA_START_ADDRESS
  LDFLAGS := $(LDFLAGS) -Wl,-Tdata,$(DATA_START_ADDRESS)
endif
ifdef LDSCRIPT
  LDFLAGS := $(LDFLAGS) -T $(LDSCRIPT)
endif

#
#  オフセットファイル（offset.h）の生成規則
#
offset.h: $(APPL_CFG_INPUT) Os_Lcfg.timestamp $(SRCDIR)/kernel/genoffset.tf
	$(CFG) --pass 3 --kernel $(CFG_KERNEL) $(INCLUDES) \
				--rom-image cfg1_out.srec --symbol-table cfg1_out.syms \
				-T $(OFFSET_TF) $(CFG_TABS) $(filter %.arxml,$^)

#
#  カーネルのコンフィギュレーションファイルの生成
#
cfg1_out.c: $(APPL_CFG_INPUT) $(SRCDIR)/kernel/kernel_def.csv
	$(CFG) --pass 1 --kernel $(CFG_KERNEL) $(INCLUDES) $(CFG_TABS) $(filter %.arxml,$^)

$(CFG2_OUT): Os_Lcfg.timestamp
Os_Lcfg.timestamp: $(APPL_CFG_INPUT) $(START_OBJS) cfg1_out.o $(END_OBJS) $(HIDDEN_OBJS) $(PASS2_TF) $(DEF_TABS)
	$(LINK) $(CFLAGS) $(LDFLAGS) $(CFG1_OUT_LDFLAGS) -o $(CFG1_OUT) \
						$(START_OBJS) cfg1_out.o $(END_OBJS)
	$(NM) -n $(CFG1_OUT) > cfg1_out.syms
	$(OBJCOPY) -O srec -S $(CFG1_OUT) cfg1_out.srec
	$(CFG) --pass 2 --kernel $(CFG_KERNEL) $(INCLUDES) \
				-T $(TARGETDIR)/target.tf $(CFG_TABS) $(filter %.arxml,$^)

#  既存Os_Cfg.hとpass2で生成したOs_Cfg_tmp.hとの差分がある場合Os_Cfg.hを上書き
	if ! cmp Os_Cfg.h Os_Cfg_tmp.h >/dev/null 2>&1 ; then \
		mv Os_Cfg_tmp.h Os_Cfg.h ;\
	else \
		rm Os_Cfg_tmp.h ;\
	fi
ifdef KERNEL_LIB
	if ! cmp Os_Cfg.h $(KERNEL_LIB)/Os_Cfg.h >/dev/null 2>&1 ; then \
		echo Config files Os_Cfg.h and $(KERNEL_LIB)/Os_Cfg.h differ! ;\
		diff -c Os_Cfg.h $(KERNEL_LIB)/Os_Cfg.h ;\
		false;\
	fi
endif

	touch -r Os_Lcfg.c Os_Lcfg.timestamp

#
#  カーネルライブラリファイルの生成
#
libkernel.a: $(OFFSET_H) $(KERNEL_LIB_OBJS)
	rm -f libkernel.a
	$(AR) -rcs libkernel.a $(KERNEL_LIB_OBJS)
	$(RANLIB) libkernel.a

#
#  特別な依存関係の定義
#
banner.o: Os_Lcfg.timestamp $(filter-out banner.o,$(ALL_OBJS)) $(LIBS_DEP)

#
#  全体のリンク
#
$(OBJFILE): $(APPL_CFG_INPUT) Os_Lcfg.timestamp $(ALL_OBJS) $(LIBS_DEP)
	$(LINK) $(CFLAGS) $(LDFLAGS) -o $(OBJFILE) $(START_OBJS) \
			-Wl,-Map,$(OBJNAME).map \
			$(APPL_OBJS) $(SYSMOD_OBJS) $(CFG_OBJS) $(ALL_LIBS) $(END_OBJS)
	$(NM) -n $(OBJFILE) > $(OBJNAME).syms
	$(OBJCOPY) -O srec -S $(OBJFILE) $(OBJNAME).srec
	$(CFG) --pass 3 --kernel $(CFG_KERNEL) $(INCLUDES) \
				--rom-image $(OBJNAME).srec --symbol-table $(OBJNAME).syms \
				-T $(TARGETDIR)/target_check.tf $(CFG_TABS) $(filter %.arxml,$^)
	$(OBJDUMP) -d $(OBJFILE) > $(OBJNAME).dump

#
#  バイナリファイルの生成
#
$(OBJNAME).bin: $(OBJFILE)
	$(OBJCOPY) -O binary -S $(OBJFILE) $(OBJNAME).bin

#
#  Sレコードファイルの生成
#
$(OBJNAME).srec: $(OBJFILE)
	$(OBJCOPY) -O srec -S $(OBJFILE) $(OBJNAME).srec

#
#  コンパイル結果の消去
#
.PHONY: clean
clean:
	rm -f \#* *~ *.o $(CLEAN_FILES)
	rm -f $(OBJFILE) $(OBJNAME).syms $(OBJNAME).srec $(OBJNAME).bin $(OBJNAME).map
	rm -f Os_Lcfg.timestamp $(CFG2_OUT)
	rm -f cfg2_out.tf
	rm -f cfg1_out.c $(CFG1_OUT) cfg1_out.syms cfg1_out.srec
	rm -f $(OBJNAME).dump
	rm -f kernel.res
ifndef KERNEL_LIB
	rm -f libkernel.a
endif
	rm -f offset.h

.PHONY: cleankernel
cleankernel:
	rm -rf $(KERNEL_LIB_OBJS)
	rm -f offset.h

.PHONY: cleandep
cleandep:
	if ! [ -f Makefile.depend ]; then \
		rm -f Os_Lcfg.timestamp $(CFG2_OUT); \
		rm -f cfg1_out.c cfg1_out.o $(CFG1_OUT) cfg1_out.syms cfg1_out.srec; \
		rm -f offset.h; \
	fi
	rm -f Makefile.depend

.PHONY: realclean
realclean: cleandep clean
	rm -f $(REALCLEAN_FILES)

#
#  ジェネレータが生成したファイルのコンパイルルールと依存関係作成
#  ルールの定義
#
#  ジェネレータが生成したファイルは，アプリケーションプログラム用，シス
#  テムサービス用，カーネル用のすべてのオプションを付けてコンパイルする．
#
OS_LCFG_CFLAGS = $(APPL_CFLAGS) $(SYSMOD_CFLAGS) $(KERNEL_CFLAGS)

$(ALL_CFG_OBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $(OS_LCFG_CFLAGS) $<

$(ALL_CFG_OBJS:.o=.s): %.s: %.c
	$(CC) -S $(CFLAGS) $(OS_LCFG_CFLAGS) $<

$(ALL_CFG_OBJS:.o=.d): %.d: %.c
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(OS_LCFG_CFLAGS)" $< >> Makefile.depend

#
#  依存関係ファイルの生成
#
.PHONY: gendepend
gendepend:
	@echo "Generating Makefile.depend."

.PHONY: depend
ifdef KERNEL_LIB
depend: cleandep Os_Lcfg.timestamp gendepend \
		cfg1_out.d \
		$(ALL_OBJS:.o=.d)
else
depend: cleandep $(OFFSET_H) Os_Lcfg.timestamp gendepend \
		cfg1_out.d \
		$(KERNEL_AUX_COBJS:.o=.d) $(KERNEL_ASMOBJS:.o=.d) \
		$(KERNEL_COBJS:.o=.d) $(KERNEL_LCSRCS:.c=.d) $(ALL_OBJS:.o=.d)
endif

#
#  依存関係ファイルをインクルード
#
-include Makefile.depend

#
#  開発ツールのコマンド名の定義
#
ifeq ($(TOOL),gcc)
  #
  #  GNU開発環境用
  #
  ifdef GCC_TARGET
    GCC_TARGET_PREFIX = $(GCC_TARGET)-
  else
    GCC_TARGET_PREFIX =
  endif
  CC = $(GCC_TARGET_PREFIX)gcc
  CXX = $(GCC_TARGET_PREFIX)g++
  AS = $(GCC_TARGET_PREFIX)as
  LD = $(GCC_TARGET_PREFIX)ld
  AR = $(GCC_TARGET_PREFIX)ar
  NM = $(GCC_TARGET_PREFIX)nm
  RANLIB = $(GCC_TARGET_PREFIX)ranlib
  OBJCOPY = $(GCC_TARGET_PREFIX)objcopy
  OBJDUMP = $(GCC_TARGET_PREFIX)objdump
endif

ifdef USE_CXX
  LINK = $(CXX)
else
  LINK = $(CC)
endif

#
#  コンパイルルールの定義
#
KERNEL_ALL_COBJS = $(KERNEL_COBJS) $(KERNEL_AUX_COBJS)

$(KERNEL_ALL_COBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $(KERNEL_CFLAGS) $<

$(KERNEL_ALL_COBJS:.o=.s): %.s: %.c
	$(CC) -S $(CFLAGS) $(KERNEL_CFLAGS) $<

$(KERNEL_LCOBJS): %.o:
	$(CC) -DTOPPERS_$(*F) -o $@ -c $(CFLAGS) $(KERNEL_CFLAGS) $<

$(KERNEL_LCOBJS:.o=.s): %.s:
	$(CC) -DTOPPERS_$(*F) -o $@ -S $(CFLAGS) $(KERNEL_CFLAGS) $<

$(KERNEL_ASMOBJS): %.o: %.S
	$(CC) -c $(CFLAGS) $(KERNEL_CFLAGS) $<

$(SYSMOD_COBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $(SYSMOD_CFLAGS) $<

$(SYSMOD_COBJS:.o=.s): %.s: %.c
	$(CC) -S $(CFLAGS) $(SYSMOD_CFLAGS) $<

$(SYSMOD_ASMOBJS): %.o: %.S
	$(CC) -c $(CFLAGS) $(SYSMOD_CFLAGS) $<

$(APPL_COBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $(APPL_CFLAGS) $<

$(APPL_COBJS:.o=.s): %.s: %.c
	$(CC) -S $(CFLAGS) $(APPL_CFLAGS) $<

$(APPL_CXXOBJS): %.o: %.cpp
	$(CXX) -c $(CFLAGS) $(APPL_CFLAGS) $<

$(APPL_CXXOBJS:.o=.s): %.s: %.cpp
	$(CXX) -S $(CFLAGS) $(APPL_CFLAGS) $<

$(APPL_ASMOBJS): %.o: %.S
	$(CC) -c $(CFLAGS) $(APPL_CFLAGS) $<

#
#  依存関係作成ルールの定義
#
$(KERNEL_COBJS:.o=.d): %.d: %.c
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(KERNEL_CFLAGS)" $< >> Makefile.depend

$(KERNEL_LCSRCS:.c=.d): %.d: %.c
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) -T "$($*)" \
		-O "-DALLFUNC $(CFLAGS) $(KERNEL_CFLAGS)" $< >> Makefile.depend

$(KERNEL_ASMOBJS:.o=.d): %.d: %.S
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(KERNEL_CFLAGS)" $< >> Makefile.depend

$(SYSMOD_COBJS:.o=.d): %.d: %.c
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(SYSMOD_CFLAGS)" $< >> Makefile.depend

$(SYSMOD_ASMOBJS:.o=.d): %.d: %.S
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(SYSMOD_CFLAGS)" $< >> Makefile.depend

$(APPL_COBJS:.o=.d): %.d: %.c
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(APPL_CFLAGS)" $< >> Makefile.depend

$(APPL_CXXOBJS:.o=.d): %.d: %.cpp
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CXX) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(APPL_CFLAGS)" $< >> Makefile.depend

$(APPL_ASMOBJS:.o=.d): %.d: %.S
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(APPL_CFLAGS)" $< >> Makefile.depend

#
#  デフォルトコンパイルルールを上書き
#
%.o: %.c
	@echo "*** Default compile rules should not be used."
	$(CC) -c $(CFLAGS) $<

%.s: %.c
	@echo "*** Default compile rules should not be used."
	$(CC) -S $(CFLAGS) $<

%.o: %.cpp
	@echo "*** Default compile rules should not be used."
	$(CXX) -c $(CFLAGS) $<

%.s: %.cpp
	@echo "*** Default compile rules should not be used."
	$(CXX) -S $(CFLAGS) $<

%.o: %.S
	@echo "*** Default compile rules should not be used."
	$(CC) -c $(CFLAGS) $<
