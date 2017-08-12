#!/usr/bin/env ruby
# -*- coding: utf-8 -*-
#
#  TOPPERS Software
#      Toyohashi Open Platform for Embedded Real-Time Systems
# 
#  Copyright (C) 2003 by Embedded and Real-Time Systems Laboratory
#                              Toyohashi Univ. of Technology, JAPAN
#  Copyright (C) 2005-2016 by Embedded and Real-Time Systems Laboratory
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
#  $Id: genrename.rb 566 2016-01-29 16:55:57Z ertl-hiro $
# 

#
#  先頭につける文字列
#
def prefixString(sym)
  if sym =~ /[a-z]/
    return("_kernel_")
  else
    return("_KERNEL_")
  end
end

#
#  リネーム定義を生成する
#
def generateDefine(outFile, sym, prefix)
  outFile.print("#define #{prefix}#{sym}");
  outFile.print("\t" * [(6 - (sym.length + prefix.length) / 4), 0].max);
  outFile.print("\t#{prefix}#{prefixString(sym)}#{sym}\n");
end

#
#  リネーム解除を生成する
#
def generateUndef(outFile, sym, prefix)
  outFile.print("#undef #{prefix}#{sym}\n");
end

#
#  コメントを生成する
#
def generateComment(outFile, comment)
  outFile.print("/*\n")
  outFile.print(" *  #{comment}\n")
  outFile.print(" */\n")
end

#
#  エラーチェック
#
if ARGV.length < 1
  abort("Usage: ruby genrename.rb <prefix>")
end

#
#  初期化
#
syms = []
name = ARGV.shift
NAME = name.upcase

inFileName = name + "_rename.def"
headerDefineSymbol = "TOPPERS_" + NAME + "_RENAME_H"

#
#  シンボルリストを読み込む
#
begin
  inFile = File.open(inFileName)
rescue Errno::ENOENT, Errno::EACCES => ex
  abort(ex.message)
end

inFile.each do |line|
  syms.push(line.chomp)
end

#
#  ???_rename.h を生成する
#
begin
  outFile = File.open(name + "_rename.h", "w")
rescue Errno::ENOENT, Errno::EACCES => ex
  abort(ex.message)
end

outFile.print <<EOS
/* This file is generated from #{inFileName} by genrename. */

#ifndef #{headerDefineSymbol}
#define #{headerDefineSymbol}

EOS

includes = ""
syms.each do |sym|
  if /^#\s*(.*)$/ =~ sym
    generateComment(outFile, $1)
  elsif /^INCLUDE\s+(.*)$/ =~ sym
    fileName = $1.sub(/([>"])$/, "_rename.h\\1")
    includes += "#include #{fileName}\n"
  elsif sym != ""
    generateDefine(outFile, sym, "")
  else
    outFile.print("\n")
  end
end

outFile.print <<EOS

#{includes}
#endif /* #{headerDefineSymbol} */
EOS

#
#  ???_unrename.h を生成する
#
begin
  outFile = File.open(name + "_unrename.h", "w")
rescue Errno::ENOENT, Errno::EACCES => ex
  abort(ex.message)
end

outFile.print <<EOS
/* This file is generated from #{inFileName} by genrename. */

/* This file is included only when #{name}_rename.h has been included. */
#ifdef #{headerDefineSymbol}
#undef #{headerDefineSymbol}

EOS

includes = ""
syms.each do |sym|
  if /^#\s*(.*)$/ =~ sym
    generateComment(outFile, $1)
  elsif /^INCLUDE\s+(.*)$/ =~ sym
    fileName = $1.sub(/([>"])$/, "_unrename.h\\1")
    includes += "#include #{fileName}\n"
  elsif sym != ""
    generateUndef(outFile, sym, "")
  else
    outFile.print("\n")
  end
end

outFile.print <<EOS

#{includes}
#endif /* #{headerDefineSymbol} */
EOS
