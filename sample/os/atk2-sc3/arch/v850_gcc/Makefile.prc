#
#  TOPPERS ATK2
#      Toyohashi Open Platform for Embedded Real-Time Systems
#      Automotive Kernel Version 2
#
#  Copyright (C) 2012-2013 by Center for Embedded Computing Systems
#              Graduate School of Information Science, Nagoya Univ., JAPAN
#  Copyright (C) 2012-2013 by FUJISOFT INCORPORATED, JAPAN
#  Copyright (C) 2012-2013 by FUJITSU VLSI LIMITED, JAPAN
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
#  $Id: Makefile.prc 187 2015-06-25 03:39:04Z t_ishikawa $
#

#
#		Makefile のプロセッサ依存部（V850用）
#

#
#  プロセッサ名，開発環境名の定義
#
PRC  = v850
TOOL = gcc

#
#  プロセッサ依存部ディレクトリ名の定義
#
PRCDIR = $(SRCDIR)/arch/$(PRC)_$(TOOL)

#
#  コンパイルオプション
#
INCLUDES := $(INCLUDES) -I$(PRCDIR) -I$(SRCDIR)/arch/$(TOOL)
COPTS := $(COPTS) -mdisable-callt -mno-app-regs -mtda=0
LIBS := $(LIBS) -lgcc -lc
LDFLAGS :=  $(LDFLAGS) -Wl,-S # for CuteSuite+ debugger

#
#  アーキテクチャの切り替え
#
ifeq ($(ARCH),V850E2V3)
	COPTS := $(COPTS) -mv850e2v3 -D__v850e2v3__ -Wa,-mno-bcond17
	KERNEL_ASMOBJS := $(KERNEL_ASMOBJS) Os_Lcfg_asm.o
endif
ifeq ($(ARCH),V850E3V5)
	COPTS := $(COPTS) -mv850e3v5
endif
 
#
#  コアタイプ(実装)による切り替え
#
ifeq ($(CORETYPE),RH850G3K)
	COPTS := $(COPTS) -msoft-float
endif
ifeq ($(CORETYPE),RH850G3M)
	ifeq ($(USE_HARD_FLOAT),true)
		COPTS := $(COPTS) -mhard-float -DTOPPERS_USE_HFLOAT
	else
		COPTS := $(COPTS) -msoft-float
	endif
endif

#
#  カーネルに関する定義
#
KERNEL_DIR := $(KERNEL_DIR) $(PRCDIR)
KERNEL_ASMOBJS := $(KERNEL_ASMOBJS) prc_support.o
# =begin modified for SC3
KERNEL_COBJS := $(KERNEL_COBJS) prc_config.o prc_mpu.o
# =end modified for SC3

#
#  スタートアップモジュールに関する定義
#
#  リンカスクリプトに「STARTUP(start.o)」を記述したため，スタートアップモジュー
#  ルの名前をHIDDEN_OBJSに定義する．また，LDFLAGSに-nostdlibを追加している．
#
HIDDEN_OBJS = start.o

$(HIDDEN_OBJS): %.o: %.S
	$(CC) -c $(CFLAGS) $(KERNEL_CFLAGS) $<

$(HIDDEN_OBJS:.o=.d): %.d: %.S
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) \
		-O "$(CFLAGS) $(KERNEL_CFLAGS)" $< >> Makefile.depend
 
# =begin modified for SC3
#
#  リンクに関する設定
#

LDFLAGS := -nostdlib $(LDFLAGS)
LDFLAGS := -nostartfiles -lgcc -lc $(LDFLAGS)
CFG1_OUT_LDFLAGS := -nostdlib
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
# =end modified for SC3

#
#  依存関係の定義
#
cfg1_out.c: $(PRCDIR)/prc_def.csv
Os_Lcfg.timestamp: $(PRCDIR)/prc.tf
# =begin modified for SC3
$(OBJFILE): $(PRCDIR)/prc_mem.tf
# =end modified for SC3
offset.h: $(PRCDIR)/prc_offset.tf

#
#  ジェネレータ関係の変数の定義
#
CFG_TABS := $(CFG_TABS) --cfg1-def-table $(PRCDIR)/prc_def.csv

# =begin added for aktsp
# vpath %.c $(SRCDIR)/aktsp/library/arch/v850e2/gcc/memory_protection/
# 
# KERNEL_COBJS := $(KERNEL_COBJS) t_osap1_share.o t_osap1.o nt_osap1_share.o nt_osap1.o nt_osap2_share.o nt_osap2.o common.o aktsp_target_lib_mem.o
# =end added for aktsp



#
#  CS+用に変換
#
cs: $(OBJFILE)
	v850-elf-objdump.exe -h $(OBJFILE) | $(SRCDIR)/arch/v850_gcc/conv2cs+.rb  $(OBJFILE)
 
