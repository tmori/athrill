#
#  $Id: Makefile.prc 2084 2011-05-12 08:03:41Z mit-kimai $
#

#
#		Makefile のアーキテクチャ依存部（プロセッサ依存部）（SH12A用）
#


#
#  コンパイルオプション
#
COPTS := $(COPTS)
LDFLAGS := -nostdlib $(LDFLAGS)
CDEFS := $(CDEFS) -DTOPPERS_LABEL_ASM
LIBS := $(LIBS)  -lgcc

#
#  システムサービスに関する定義
#  　kernel_cfg_asm.Sをシステムサービスに含めるのは、適切ではないが、
#  　　・コンパイル・オプション（特にインクルードパス）
#  　　・カーネル・ライブラリの構成
#  　の都合で、便宜上、ここに含めている。
#
SYSSVC_ASMOBJS := $(SYSSVC_ASMOBJS) kernel_cfg_asm.o
REALCLEAN_FILES := $(REALCLEAN_FILES) kernel_cfg_asm.S

#
#  コンフィギュレータ関係の変数の定義
#
CFG_TABS := $(CFG_TABS) --cfg1-def-table $(PRCDIR)/prc_def.csv

#
#  カーネルに関する定義
#
KERNEL_DIR := $(KERNEL_DIR) $(PRCDIR)
KERNEL_ASMOBJS := $(KERNEL_ASMOBJS) prc_support.o
KERNEL_COBJS := $(KERNEL_COBJS) prc_config.o
KERNEL_CFLAGS := $(KERNEL_CFLAGS) -fno-strict-aliasing

#
#  依存関係の定義
#
cfg1_out.c: $(PRCDIR)/prc_def.csv
kernel_cfg.timestamp: $(PRCDIR)/prc.tf
$(OBJFILE): $(PRCDIR)/prc_mem.tf

#
#  オフセットファイル生成のための定義
#
# OFFSET_TF = $(PRCDIR)/prc_offset.tf
