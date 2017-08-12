#!/usr/bin/env ruby
# -*- coding: utf-8 -*-
#
#  TOPPERS Software
#      Toyohashi Open Platform for Embedded Real-Time Systems
# 
#  Copyright (C) 2001-2003 by Embedded and Real-Time Systems Laboratory
#                              Toyohashi Univ. of Technology, JAPAN
#  Copyright (C) 2006-2016 by Embedded and Real-Time Systems Laboratory
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
#  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
#  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
#  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
#  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
#  の責任を負わない．
# 
#  $Id: configure.rb 626 2016-02-12 13:46:32Z ertl-hiro $
# 

require "optparse"
require "fileutils"
require "shell"

#  オプションの定義
#
#  -T <target>			ターゲット名（必須）
#  -a <appldirs>		アプリケーションのディレクトリ名（複数指定可．デ
#						フォルトはsampleディレクトリ）
#  -A <applname>		アプリケーションプログラム名（デフォルトはsample1）
#  -t					メインのオブジェクトファイルをリンク対象に含めない
#  -c <cfgfile>			システムコンフィギュレーションファイル（.cfgファイ
#						ル名）名
#  -C <cdlflle>			コンポーネント記述ファイル（.cdlファイル）名
#  -U <applobjs>		他のアプリケーションプログラムファイル
#						（.oファイル名で指定．複数指定可）
#  -S <syssvcobjs>		システムサービスのプログラムファイル
#						（.oファイル名で指定．複数指定可）
#  -L <kernel_lib>		カーネルライブラリ（libkernel.a）のディレクトリ名
#						（省略した場合，カーネルライブラリもmakeする）
#  -f					カーネルを関数単位でコンパイルするかどうかの指定
#  -D <srcdir>			カーネルソースの置かれているディレクトリ
#  -l <srclang>			プログラミング言語（現時点ではcとc++のみサポート）
#  -m <tempmakefile>	Makefileのテンプレートのファイル名の指定（デフォル
#						トはsampleディレクトリのMakefile）
#  -d <depdir>			依存関係ファイルのディレクトリ名（デフォルトはdeps）
#  -w					TECSを使用しない
#  -r					トレースログ記録のサンプルコードを使用するかどうか
#						の指定
#  -V <devtooldir>		開発ツール（コンパイラ等）の置かれているディレクトリ
#  -R <ruby>			rubyのパス名（明示的に指定する場合）
#  -g <cfg>				コンフィギュレータ（cfg）のパス名
#  -G <tecsgen>			TECSジェネレータ（tecsgen）のパス名
#  -o <options>			コンパイルオプション（COPTSに追加）
#  -O <options>			シンボル定義オプション（CDEFSに追加）
#  -k <options>			リンカオプション（LDFLAGS等に追加）

#  使用例(1)
#
#  % ../configure.rb -T ct11mpcore_gcc -O "-DTOPPERS_USE_QEMU" \
#					-A perf1 -a ../test -S "test_svc.o histogram.o"
#
#  使用例(2)
#
#  % ../configure.rb -T macosx_gcc -L .
#	アプリケーションプログラムは，sample1になる．
#
#  使用例(3)
#
#  % ../configure.rb -T ct11mpcore_gcc -O "-DTOPPERS_USE_QEMU" -A tSample2 -t
#	アプリケーションプログラムは，TECS版のサンプルプログラムになる．
#
#  使用例(4)
#
#  % ../configure.rb -T ct11mpcore_gcc PRC_NUM=4
#	PRC_NUMを4に定義する．

#
#  変数の初期化
#
$target = nil
$appldirs = []
$applobjs = []
$syssvcobjs = []
$kernel_lib = ""
$kernel_funcobjs = ""
$srclang = "c"
$depdir = "deps"
$omit_tecs = ""
$enable_trace = ""
$devtooldir = ""
$ruby = "ruby"
$copts = []
$cdefs = []
$ldflags = []

#
#  オプションの処理
#
OptionParser.new(nil, 22) do |opt|
  opt.on("-T target",		"taget name (mandatory)") do |val|
    $target = val
  end
  opt.on("-a appldirs",		"application directories") do |val|
    $appldirs += val.split("\s+")
  end
  opt.on("-A applname",		"application program name") do |val|
    $applname = val
  end
  opt.on("-t",				"TECS is used for application development") do |val|
    $option_t = true
  end
  opt.on("-c cfgfile",		"system configuration file") do |val|
    $cfgfile = val
  end
  opt.on("-C cdlfile",		"component description file") do |val|
    $cdlfile = val
  end
  opt.on("-U applobjs",		"additional application object files") do |val|
    $applobjs += val.split("\s+")
  end
  opt.on("-S syssvcobjs",	"system service object files") do |val|
    $syssvcobjs += val.split("\s+")
  end
  opt.on("-L kernel_lib",	"directory of built kernel library") do |val|
    $kernel_lib = val
  end
  opt.on("-f", "each function is complied separately in kernel") do |val|
    $kernel_funcobjs = true
  end
  opt.on("-D srcdir",		"path of source code directory") do |val|
    $srcdir = val
  end
  opt.on("-l srclang",		"programming language (C or C++)") do |val|
    $srclang = val
  end
  opt.on("-m tempmakefile", "template file of Makefile") do |val|
    $tempmakefile = val
  end
  opt.on("-d depdir",		"dependency relation file directory") do |val|
    $depdir = val
  end
  opt.on("-w",				"TECS is not used at all") do |val|
    $omit_tecs = true
  end
  opt.on("-r",				"use the sample code for trace log") do |val|
    $enable_trace = true
  end
  opt.on("-V devtooldir",	"development tools directory") do |val|
    $devtooldir = val
  end
  opt.on("-R ruby",			"path of ruby command") do |val|
    $ruby = val
  end
  opt.on("-g cfg",			"path of configurator") do |val|
    $cfg = val
  end
  opt.on("-G tecsgen",		"path of TECS generator") do |val|
    $tecsgen = val
  end
  opt.on("-o options",		"compiler options") do |val|
    $copts += val.split("\s+")
  end
  opt.on("-O options",		"symbol definition options") do |val|
    $cdefs += val.split("\s+")
  end
  opt.on("-k options",		"linker options") do |val|
    $ldflags += val.split("\s+")
  end
  opt.parse!(ARGV)
end

#
#  オブジェクトファイル名の拡張子を返す
#
def GetObjectExtension
  if /cygwin/ =~ RUBY_PLATFORM
    return("exe")
  else
    return("")
  end
end

#
#  変数のデフォルト値
#
if $appldirs.empty?
  $appldirs.push("\$(SRCDIR)/sample")
end
$applname ||= "sample1"
if $option_t.nil?
  $applobjs.unshift($applname + ".o")
end
$cfgfile ||= $applname + ".cfg"
$cdlfile ||= $applname + ".cdl"
if $srcdir.nil?
  # ソースディレクトリ名を取り出す
  if /(.*)\/configure/ =~ $0
    $srcdir = $1
  else
    $srcdir = Shell.new.cwd
  end
end
if /^\// =~ $srcdir
  $srcabsdir = $srcdir
else
  $srcabsdir = Shell.new.cwd + "/" + $srcdir
end
$tempmakefile ||= $srcdir + "/sample/Makefile"
$cfg ||= $ruby + " \$(SRCDIR)/cfg/cfg.rb"
$tecsgen ||= $ruby + " \$(SRCDIR)/tecsgen/tecsgen.rb"

#
#  -Tオプションとターゲット依存部ディレクトリの確認
#
if $target.nil?
  puts("configure.rb: -T option is mandatory")
elsif !File.directory?($srcdir + "/target/" + $target)
  puts("configure.rb: #{$srcdir}/target/#{$target} not exist.")
  $target = nil
end

if $target.nil?
  puts("Installed targets are:")
  Dir.glob($srcdir + "/target/[a-zA-Z0-9]*").each do |target|
    target.sub!($srcdir + "/target/", "")
    puts("\t" + target)
  end
  abort
end

#
#  変数テーブルの作成
#
$vartable = Hash.new("")
$vartable["TARGET"] = $target
$vartable["APPLDIRS"] = $appldirs.join(" ")
$vartable["APPLNAME"] = $applname
$vartable["CFGFILE"] = $cfgfile
$vartable["CDLFILE"] = $cdlfile
$vartable["APPLOBJS"] = $applobjs.join(" ")
$vartable["SYSSVCOBJS"] = $syssvcobjs.join(" ")
$vartable["KERNEL_LIB"] = $kernel_lib
$vartable["KERNEL_FUNCOBJS"] = $kernel_funcobjs
$vartable["SRCDIR"] = $srcdir
$vartable["SRCABSDIR"] = $srcabsdir
$vartable["SRCLANG"] = $srclang
$vartable["DEPDIR"] = $depdir
$vartable["OMIT_TECS"] = $omit_tecs
$vartable["ENABLE_TRACE"] = $enable_trace
$vartable["DEVTOOLDIR"] = $devtooldir
$vartable["RUBY"] = $ruby
$vartable["CFG"] = $cfg
$vartable["TECSGEN"] = $tecsgen
$vartable["COPTS"] = $copts.join(" ")
$vartable["CDEFS"] = $cdefs.join(" ")
$vartable["LDFLAGS"] = $ldflags.join(" ")
$vartable["OBJEXT"] = GetObjectExtension()
ARGV.each do |arg|
  if /^([A-Za-z0-9_]+)\s*\=\s*(.*)$/ =~ arg
    $vartable[$1] = $2
  else
    $vartable[arg] = true
  end
end

#
#  ファイルを変換する
#
def convert(inFileName, outFileName)
  puts("Generating #{outFileName} from #{inFileName}.\n")
  if (File.file?(outFileName))
    puts("#{outFileName} exists.  Save as #{outFileName}.bak.\n")
    FileUtils.move(outFileName, outFileName + ".bak")
  end

  begin
    inFile = File.open(inFileName)
    outFile = File.open(outFileName, "w")
  rescue Errno::ENOENT, Errno::EACCES => ex
    abort(ex.message)
  end

  inFile.each do |line|
    line.chomp!
    while line.sub!(/\@\(([A-Za-z0-9_]+)\)/) {|var| $vartable[$1]} do end
    outFile.print(line,"\n")
  end

  inFile.close
  outFile.close
end

#
#  Makefileの生成
#
convert($tempmakefile, "Makefile")

#
#  依存関係ファイルのディレクトリの作成
#
if !File.directory?($depdir)
  Dir.mkdir($depdir)
end
