#!/usr/bin/env ruby
# -*- coding: utf-8 -*-
#
#  TOPPERS Configurator by Ruby
#
#  Copyright (C) 2015 by FUJI SOFT INCORPORATED, JAPAN
#  Copyright (C) 2015,2016 by Embedded and Real-Time Systems Laboratory
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
#  $Id: cfg.rb 130 2016-07-31 16:30:35Z ertl-hiro $
#

if $0 == __FILE__
  TOOL_ROOT = File.expand_path(File.dirname(__FILE__)) + "/"
  $LOAD_PATH.unshift(TOOL_ROOT)
end

require "pp"
require "csv"
require "optparse"
require "pstore"
require "GenFile.rb"
require "SRecord.rb"

#
#  定数定義
#
# 共通
VERSION = "1.2.2"

# cfg1_out関係
CFG1_PREFIX        = "TOPPERS_cfg_"
CFG1_MAGIC_NUM     = "TOPPERS_magic_number"
CFG1_SIZEOF_SIGNED = "TOPPERS_sizeof_signed_t"
CFG1_OUT_C         = "cfg1_out.c"
CFG1_OUT_DB        = "cfg1_out.db"
CFG1_OUT_SREC      = "cfg1_out.srec"
CFG1_OUT_SYMS      = "cfg1_out.syms"
CFG1_OUT_TIMESTAMP = "cfg1_out.timestamp"
CFG1_OUT_TARGET_H  = "target_cfg1_out.h"

# cfg2_out関係
CFG2_OUT_DB        = "cfg2_out.db"

# cfg3_out関係
CFG3_OUT_DB        = "cfg3_out.db"

#
#  エラー発生有無フラグ
#
$errorFlag = false

#
#  エラー／警告表示関数
#
# 一般的なエラー表示（処理を中断）
def error_exit(message, location = "")
  location += " " if location != ""
  abort("#{location}error: #{message}")
end

# 一般的なエラー表示（処理を継続）
def error(message, location = "")
  location += " " if location != ""
  STDERR.puts("#{location}error: #{message}")
  $errorFlag = true
end

# 一般的な警告表示
def warning(message, location = "")
  location += " " if location != ""
  STDERR.puts("#{location}warning: #{message}")
end

# システムコンフィギュレーションファイルの構文解析時のエラー
$noParseError = 0
def parse_error(cfgFile, message)
  error(message, "#{cfgFile.getFileName()}:#{cfgFile.getLineNo}:")
  if ($noParseError += 1) >= 10
    abort("too many errors emitted, stopping now")
  end
end

# システムコンフィギュレーションファイルの構文解析時の警告
def parse_warning(cfgFile, message)
  warning(message, "#{cfgFile.getFileName()}:#{cfgFile.getLineNo}:")
end

#
#  静的API処理時のエラー／警告表示関数
#
# 静的API処理時のエラー／警告を短く記述できるように，メッセージ中の%ま
# たは%%で始まる記述を以下のように展開する．
#	%label → #{params[:label]}
#	%%label → label `#{params[:label]}'
#
# エラー／警告メッセージの展開
def expand_message(message, params)
  result = message.dup
  while /%%(\w+)\b/ =~ result
    param = $1
    paramVal = params[param.to_sym].to_s
    result.sub!(/%%#{param}\b/, "#{param} `#{paramVal}'")
  end
  while /%(\w+)\b/ =~ result
    param = $1
    paramVal = params[param.to_sym].to_s
    result.sub!(/%#{param}\b/, paramVal)
  end
  return(result)
end

# 静的API処理時のエラー
def error_api(params, message)
  error(expand_message(message, params), \
			"#{params[:_file_]}:#{params[:_line_]}:")
end

# 静的API処理時の警告
def warning_api(params, message)
  warning(expand_message(message, params), \
			"#{params[:_file_]}:#{params[:_line_]}:")
end

# 静的API処理時のエラー（エラーコード付き）
def error_ercd(errorCode, params, message)
  error_api(params, "#{errorCode}: #{message}")
end

# 静的API処理時の警告（エラーコード付き）
def warning_ercd(errorCode, params, message)
  warning_api(params, "#{errorCode}: #{message}")
end

# パラメータのエラー
def error_wrong(errorCode, params, symbol, wrong)
  error_ercd(errorCode, params, "%%#{symbol} is #{wrong} in %apiname")
end

def error_wrong_id(errorCode, params, symbol, objid, wrong)
  error_ercd(errorCode, params, "%%#{symbol} is #{wrong} " \
	             					"in %apiname of %#{objid}")
end

def error_wrong_sym(errorCode, params, symbol, symbol2, wrong)
  error_ercd(errorCode, params, "%%#{symbol} is #{wrong} " \
									"in %apiname of %%#{symbol2}")
end

# パラメータ不正のエラー
def error_illegal(errorCode, params, symbol)
  error_ercd(errorCode, params, "illegal %%#{symbol} in %apiname")
end

def error_illegal_id(errorCode, params, symbol, objid)
  error_ercd(errorCode, params, "illegal %%#{symbol} " \
	             					"in %apiname of %#{objid}")
end

def error_illegal_sym(errorCode, params, symbol, symbol2)
  error_ercd(errorCode, params, "illegal %%#{symbol} " \
									"in %apiname of %%#{symbol2}")
end

#
#  Stringクラスの拡張（二重引用符で囲まれた文字列の作成／展開）
#
class String
  #
  #  二重引用符で囲まれた文字列の作成
  #
  def quote
    result = ""
    self.chars do |c|
      case c
      when "'"
        result += "\\\'"
      when "\""
        result += "\\\""
      when "\0"
        result += "\\0"
      when "\a"
        result += "\\a"
      when "\b"
        result += "\\b"
      when "\f"
        result += "\\f"
      when "\n"
        result += "\\n"
      when "\r"
        result += "\\r"
      when "\t"
        result += "\\t"
      when "\v"
        result += "\\v"
      when "\\"
        result += "\\\\"
      else
        result += c
      end
    end
    return("\"" + result + "\"")
  end

  #
  #  二重引用符で囲まれた文字列の展開
  #
  def unquote
    if /^\"(.*)\"$/m =~ self
      str = $1
      result = ""
      while (/^(.*)\\(.*)$/m =~ str)
        result += $1
        str = $2
        case str
        when /^[aA](.*)$/m
          result += "\a"
          str = $1
        when /^[bB](.*)$/m
          result += "\b"
          str = $1
        when /^[fF](.*)$/m
          result += "\f"
          str = $1
        when /^[nN](.*)$/m
          result += "\n"
          str = $1
        when /^[rR](.*)$/m
          result += "\r"
          str = $1
        when /^[tT](.*)$/m
          result += "\t"
          str = $1
        when /^[vV](.*)$/m
          result += "\v"
          str = $1
        when /^[xX]([0-9a-fA-F][0-9a-fA-F]?)(.*)$/m
          result += $1.hex
          str = $2
        when /^([0-7][0-7]?[0-7]?)(.*)$/m
          result += $1.oct
          str = $2
        when /^\\(.*)$/m
          result += "\\"
          str = $1
        end
      end
      return(result + str)
    else
      return(self.dup)
    end
  end
end

#
#  NumStrクラス（数値に文字列を付加したもの）の定義
#
class NumStr
  def initialize(val, str = val.to_s)
    @val = val
    @str = str
  end

  # 数値情報を返す
  def val
    return @val
  end
  alias_method :to_i, :val

  # 文字列情報を返す
  def str
    return @str
  end
  alias_method :to_s, :str

  # 比較は数値情報で行う
  def ==(other)
    @val == other
  end
  def !=(other)
    @val != other
  end
  def <=>(other)
    @val <=> other
  end

  # ハッシュのキーとして使う時の比較も数値情報で行う
  def eql?(other)
    @val == other.val
  end

  # ハッシュ値の定義も上書きする
  def hash
    return @val.hash
  end

  # 数値クラスと演算できるようにする
  def coerce(other)
    if other.kind_of?(Numeric)
      return other, @val
    else
      raise
    end
  end

  # 二重引用符で囲まれた文字列の作成
  def quote
    str.quote
  end

  # 二重引用符で囲まれた文字列の展開
  def unquote
    str.unquote
  end

  # pp時の表示
  def pretty_print(q)
    q.text("[#{@val}(=0x#{@val.to_s(16)}),")
    @str.pretty_print(q)
    q.text("]")
  end

  # 未定義のメソッドは@valに送る
  def method_missing(*method)
    @val.send(*method)
  end
end

#
#  シンボルファイルの読み込み
#
#  以下のメソッドは，GNUのnmが生成するシンボルファイルに対応している．
#  別のツールに対応する場合には，このメソッドを書き換えればよい．
#
def ReadSymbolFile(symbolFileName)
  begin
    symbolFile = File.open(symbolFileName)
  rescue Errno::ENOENT, Errno::EACCES => ex
    abort(ex.message)
  end

  symbolAddress = {}
  symbolFile.each do |line|
    # スペース区切りで分解
    fields = line.split(/\s+/)

    # 3列になっていない行は除外
    if fields.size == 3
      symbolAddress[fields[2]] = fields[0].hex
    end
  end
  symbolFile.close
  return(symbolAddress)
end

#
#  値取得シンボルをグローバル変数として定義する
#
def DefineSymbolValue
  $symbolValueTable.each do |symbolName, symbolData|
    if symbolData.has_key?(:VALUE)
      eval("$#{symbolName} = #{symbolData[:VALUE]}")
    end
  end
end

#
#  インクルードパスからファイルを探す
#
def SearchFilePath(fileName)
  if File.exist?(fileName)
    # 指定したファイルパスに存在する
    return fileName
  elsif /^\./ =~ fileName
    # 相対パスを指定していて見つからなかった場合，存在しないものとする
    #（意図しないファイルが対象となることを防止）
    return nil
  else
    # 各インクルードパスからファイル存在チェック
    $includeDirectories.each do |includeDirectory|
      path = includeDirectory + "/" + fileName
      # 見つかったら相対パスを返す
      if File.exist?(path)
        return path
      end
    end
    return nil
  end
end

#
#  指定した生成スクリプト（trbファイル）を検索してloadする
#
def IncludeTrb(fileName)
  filePath = SearchFilePath(fileName)
  if filePath.nil?
    error_exit("`#{fileName}' not found")
  else
    load(filePath)
  end
end

#
#  パス3の処理
#
def Pass3
  #
  #  パス2から引き渡される情報をファイルから読み込む
  #
  db = PStore.new(CFG2_OUT_DB)
  db.transaction(true) do
    $apiDefinition = db[:apiDefinition]
    $symbolValueTable = db[:symbolValueTable]
    $cfgFileInfo = db[:cfgFileInfo]
    $includeFiles = db[:includeFiles]
    $cfgData = db[:cfgData]
    $asmLabel = db[:asmLabel]
    $endianLittle = db[:endianLittle]
    $cfg2Data = db[:cfg2Data]
  end
  $cfg3Data = {}

  #
  #  値取得シンボルをグローバル変数として定義する
  #
  DefineSymbolValue()

  #
  #  生成スクリプト（trbファイル）を実行する
  #
  $trbFileNames.each do |trbFileName|
    IncludeTrb(trbFileName)
  end

  #
  #  パス4に引き渡す情報をファイルに生成
  #
  if $omitOutputDb.nil?
    db = PStore.new(CFG3_OUT_DB)
    db.transaction do
      db[:apiDefinition] = $apiDefinition
      db[:symbolValueTable] = $symbolValueTable
      db[:cfgFileInfo] = $cfgFileInfo
      db[:includeFiles] = $includeFiles
      db[:cfgData] = $cfgData
      db[:asmLabel] = $asmLabel
      db[:endianLittle] = $endianLittle
      db[:cfg3Data] = $cfg3Data
    end
  end
end

#
#  パス4の処理
#
def Pass4
  #
  #  パス3から引き渡される情報をファイルから読み込む
  #
  db = PStore.new(CFG3_OUT_DB)
  db.transaction(true) do
    $apiDefinition = db[:apiDefinition]
    $symbolValueTable = db[:symbolValueTable]
    $cfgFileInfo = db[:cfgFileInfo]
    $includeFiles = db[:includeFiles]
    $cfgData = db[:cfgData]
    $asmLabel = db[:asmLabel]
    $endianLittle = db[:endianLittle]
    $cfg3Data = db[:cfg3Data] || db[:cfg2Data]
  end

  #
  #  値取得シンボルをグローバル変数として定義する
  #
  DefineSymbolValue()

  #
  #  生成スクリプト（trbファイル）を実行する
  #
  $trbFileNames.each do |trbFileName|
    IncludeTrb(trbFileName)
  end
end

#
#  生成スクリプト（trbファイル）向けの関数
#
def SYMBOL(symbol)
  if !$romSymbol.nil? && $romSymbol.has_key?($asmLabel + symbol)
    return $romSymbol[$asmLabel + symbol]
  else
    return nil
  end
end

def BCOPY(fromAddress, toAddress, size)
  if !$romImage.nil?
    copyData = $romImage.get_data(fromAddress, size)
    if !copyData.nil?
      $romImage.set_data(toAddress, copyData)
    end
  end
end

def BZERO(address, size)
  if !$romImage.nil?
    $romImage.set_data(address, "00" * size)
  end
end

def PEEK(address, size, signed=false)
  if !$romImage.nil?
    return $romImage.get_value(address, size, signed)
  else
    return nil
  end
end

#
#  グローバル変数の初期化
#
$kernel = nil
$pass = nil
$includeDirectories = []
$trbFileNames = []
$apiTableFileNames = []
$symvalTableFileNames = []
$romImageFileName = nil
$romSymbolFileName = nil
$dependencyFileName = nil
$idInputFileName = nil
$idOutputFileName = nil

#
#  オプションの処理
#
OptionParser.new(banner="Usage: cfg.rb [options] CONFIG-FILE", 40) do |opt|
  opt.version = VERSION
  opt.on("-k KERNEL", "--kernel KERNEL", "kernel profile name") do |val|
    $kernel = val
  end
  opt.on("-p NUM", "--pass NUM", "processing pass number") do |val|
    $pass = val
  end
  opt.on("-I DIRECTORY", "--include-directory DIRECTORY",
										 "include directory") do |val|
    $includeDirectories.push(val)
  end
  opt.on("-T TRB-FILE", "--trb-file TRB-FILE",
         "generation script (trb file)") do |val|
    $trbFileNames.push(val)
  end
  opt.on("--api-table API-TABLE-FILE", "static API table file") do |val|
    $apiTableFileNames.push(val)
  end
  opt.on("--symval-table SYMVAL-TABLE-FILE", "symbol-value table file") do |val|
    $symvalTableFileNames.push(val)
  end
  opt.on("--rom-image SREC-FILE", "rom image file (s-record)") do |val|
    $romImageFileName = val
  end
  opt.on("--rom-symbol SYMS-FILE", "rom symbol table file (nm)") do |val|
    $romSymbolFileName = val
  end
  opt.on("--id-input-file ID-FILE", "ID input file") do |val|
    $idInputFileName = val
  end
  opt.on("--id-output-file ID-FILE", "ID output file") do |val|
    $idOutputFileName = val
  end
  opt.on("-M [DEPEND-FILE]", "--print-dependencies [DEPEND-FILE]",
         "dependency file") do |val|
    $dependencyFileName = val.nil? ? "" : val
  end
  opt.on("-O", "--omit-output-db", "omit DB file output") do
    $omitOutputDb = true
  end
  opt.on("-v", "--version", "show version number") do
    abort(opt.ver)
  end
  opt.on("-h", "--help", "show help (this)") do
    abort(opt.help)
  end
  opt.parse!(ARGV)
end
$configFileNames = ARGV

#
#  オプションのチェック
#
if $pass.nil?
  # パスの指定は必須
  abort("`--pass' option is mandatory")
elsif /^[1234]$/ !~ $pass
  abort("pass number `#{$pass}' is not valid")
end

# パス1では，静的APIテーブルは必須
if ($pass == "1" && $apiTableFileNames.empty?)
  abort("`--api-table' option must be specified in pass 1")
end

# パス1以外では，生成スクリプト（trbファイル）が必須
if ($pass != "1" && $trbFileNames.empty?)
  abort("`--trb-file' must be specified except in pass 1")
end

#
#  カーネルオプションの処理
#
case $kernel
when /^hrp/
	$supportDomain = true
when /^fmp/
	$supportClass = true
end

#
#  ID番号入力ファイルの取り込み
#
$inputObjid = {}
if !$idInputFileName.nil?
  begin
    idInputFile = File.open($idInputFileName)
  rescue Errno::ENOENT, Errno::EACCES => ex
    abort(ex.message)
  end

  idInputFile.each do |line|
    ( objName, objidNumber ) = line.split(/\s+/)
    $inputObjid[objName] = objidNumber.to_i
  end

  idInputFile.close
end

#
#  指定されたシンボルファイルの読み込み
#
if !$romSymbolFileName.nil?
  if File.exist?($romSymbolFileName)
    $romSymbol = ReadSymbolFile($romSymbolFileName)
  else
    error_exit("`#{$romSymbolFileName}' not found")
  end
end

#
#  指定されたSレコードファイルの読み込み
#
if !$romImageFileName.nil?
  if File.exist?($romImageFileName)
    $romImage = SRecord.new($romImageFileName)
  else
    error_exit("`#{$romImageFileName}' not found")
  end
end

#
#  パスに従って各処理を実行
#
case $pass
when "1"
  load("pass1.rb")
  Pass1()
when "2"
  load("pass2.rb")
  Pass2()
when "3"
  Pass3()
when "4"
  Pass4()
else
  error_exit("invalid pass: #{$pass}")
end

# エラー発生時はabortする
if $errorFlag
  if ($0 == __FILE__)
    abort()
  else
    # simplecov対応
    raise()
  end
end

#
#  作成したすべてのファイルを出力する
#
GenFile.output

# 
#  タイムスタンプファイルの生成
# 
if !$timeStampFileName.nil?
  File.open($timeStampFileName, "w").close
end
