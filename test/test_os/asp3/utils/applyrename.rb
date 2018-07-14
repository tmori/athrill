#!/usr/bin/env ruby
# -*- coding: utf-8 -*-
#
#  TOPPERS Software
#      Toyohashi Open Platform for Embedded Real-Time Systems
# 
#  Copyright (C) 2003 by Embedded and Real-Time Systems Laboratory
#                              Toyohashi Univ. of Technology, JAPAN
#  Copyright (C) 2004-2016 by Embedded and Real-Time Systems Laboratory
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
#  $Id: applyrename.rb 562 2016-01-21 14:24:34Z ertl-hiro $
# 

require "fileutils"

#
#  ファイルにリネームを適用する
#
def applyRename(inFileName, syms)
  symsRegexp = syms.join("|")
  outFileName = inFileName + ".new"

  begin
    inFile = File.open(inFileName)
    outFile = File.open(outFileName, "w")
  rescue Errno::ENOENT, Errno::EACCES => ex
    abort(ex.message)
  end

  inFile.each do |line|
    line.gsub!(/\b(_?)(#{symsRegexp})\b/, "\\1_kernel_\\2")
    outFile.print line
  end

  inFile.close
  outFile.close

  if FileUtils.cmp(inFileName, outFileName)
    # ファイルの内容が変化しなかった場合
    FileUtils.remove(outFileName)
  else
    # ファイルの内容が変化した場合
    FileUtils.move(inFileName, inFileName + ".bak")
    FileUtils.move(outFileName, inFileName)
    puts("Modified: #{inFileName}")
  end
end

#
#  エラーチェック
#
if ARGV.length < 1
  abort("Usage: ruby appyrename.rb <prefix> <filelists>")
end

#
#  初期化
#
syms = []
name = ARGV.shift

#
#  シンボルリストを読み込む
#
defFileName = name + "_rename.def"

begin
  defFile = File.open(defFileName)
rescue Errno::ENOENT, Errno::EACCES => ex
  abort(ex.message)
end

defFile.each do |line|
  line.chomp!
  if /^#\s*(.*)$/ =~ line
    # do nothing
  elsif /^INCLUDE\s+(.*)$/ =~ line
    # do nothing
  elsif line != ""
    syms.push(line)
  end
end

#
#  ファイルにリネームを適用する
#
ARGV.each do |inFileName|
  if inFileName != defFileName && File.readable?(inFileName)
    applyRename(inFileName, syms)
  end
end
