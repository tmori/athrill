#
#  TOPPERS/JSP Kernel
#      Toyohashi Open Platform for Embedded Real-Time Systems/
#      Just Standard Profile Kernel
#
#  Copyright (C) 2000,2001 by Embedded and Real-Time Systems Laboratory
#                              Toyohashi Univ. of Technology, JAPAN
#  Copyright (C) 2005 by Freelines CO.,Ltd
#
#  Copyright (C) 2010 by Meika Sugimoto
#
#  上記著作権者は，以下の (1)~(4) の条件か，Free Software Foundation
#  によって公表されている GNU General Public License の Version 2 に記
#  述されている条件を満たす場合に限り，本ソフトウェア（本ソフトウェア
#  を改変したものを含む．以下同じ）を使用・複製・改変・再配布（以下，
#  利用と呼ぶ）することを無償で許諾する．
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
#
#  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
#  よびTOPPERSプロジェクトは，本ソフトウェアに関して，その適用可能性も
#  含めて，いかなる保証も行わない．また，本ソフトウェアの利用により直
#  接的または間接的に生じたいかなる損害に関しても，その責任を負わない．
#
#

#
#  Makefile のプロセッサ依存 (V850ES/FK3用)
#

#
#  GNU開発環境のターゲットアーキテクチャの定義
#
GCC_TARGET = v850-elf

#
#  コンパイルフラグ
#
INCLUDES := $(INCLUDES) -I$(SRCDIR)/arch/$(PRC)_$(TOOL)
COPTS := $(COPTS) -mv850e -mdisable-callt
CDEFS := $(CDEFS) -DLABEL_ASM

#
#  カーネルに関する定義
#
KERNEL_DIR		:= $(KERNEL_DIR) $(SRCDIR)/arch/$(PRC)_$(TOOL)
KERNEL_ASMOBJS	:= $(KERNEL_ASMOBJS) prc_support.o prc_sil.o
KERNEL_COBJS	:= $(KERNEL_COBJS) prc_config.o

#
#  コンフィギュレータ関連の設定
#
CFG2_OUT := kernel_cfg_asm.S $(CFG2_OUT)
CFG_ASMOBJS := kernel_cfg_asm.o $(CFG_ASMOBJS)
CFG_TABS := --cfg1-def-table $(SRCDIR)/arch/v850_gcc/prc_def.csv $(CFG_TABS)

#
#  リンクに関する設定
#

LDFLAGS := -nostartfiles -lgcc -lc $(LDFLAGS)
CFG1_OUT_LDFLAGS := -nostdlib

#
#  スタートアップモジュールに関する定義
#
START_OBJS = start.o

$(START_OBJS): %.o: %.S
	$(CC) -c $(CFLAGS) $(KERNEL_CFLAGS) $<

$(START_OBJS:.o=.d): %.d: %.S
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) \
		-O "$(CFLAGS) $(KERNEL_CFLAGS)" $< >> Makefile.depend

#
#  kernel_cfg_asm.Sののコンパイルルールと依存関係作成ルールの定義
#
#  kernel_cfg_asm.Sは，アプリケーションプログラム用，システムサー
#  ビス用，カーネル用のすべてのオプションを付けてコンパイルする．
#

CFG2_OUT_SRCS := kernel_cfg_asm.S $(CFG2_OUT_SRCS)

#
#  コンフィギュレータ用の依存関係の定義
#

cfg1_out.c: $(SRCDIR)/arch/v850_gcc/prc_def.csv
