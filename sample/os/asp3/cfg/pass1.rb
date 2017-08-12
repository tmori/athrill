# -*- coding: utf-8 -*-
#
#  TOPPERS Configurator by Ruby
#
#  Copyright (C) 2015 by FUJI SOFT INCORPORATED, JAPAN
#  Copyright (C) 2015-2017 by Embedded and Real-Time Systems Laboratory
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
#  $Id: pass1.rb 133 2017-03-26 05:37:50Z ertl-hiro $
#

#
#		パス1の処理
#

#
#  値取得シンボルテーブルへの固定登録
#
$symbolValueTable = {
  "CHAR_BIT"  => { EXPR: "CHAR_BIT" },
  "SCHAR_MAX" => { EXPR: "SCHAR_MAX", SIGNED: true },
  "SCHAR_MIN" => { EXPR: "SCHAR_MIN", SIGNED: true },
  "UCHAR_MAX" => { EXPR: "UCHAR_MAX" },
  "CHAR_MAX"  => { EXPR: "CHAR_MAX",  SIGNED: true },
  "CHAR_MIN"  => { EXPR: "CHAR_MIN",  SIGNED: true },
  "SHRT_MAX"  => { EXPR: "SHRT_MAX",  SIGNED: true },
  "SHRT_MIN"  => { EXPR: "SHRT_MIN",  SIGNED: true },
  "USHRT_MAX" => { EXPR: "USHRT_MAX" },
  "INT_MAX"   => { EXPR: "INT_MAX",   SIGNED: true },
  "INT_MIN"   => { EXPR: "INT_MIN",   SIGNED: true },
  "UINT_MAX"  => { EXPR: "UINT_MAX" },
  "LONG_MAX"  => { EXPR: "LONG_MAX",  SIGNED: true },
  "LONG_MIN"  => { EXPR: "LONG_MIN",  SIGNED: true },
  "ULONG_MAX" => { EXPR: "ULONG_MAX" }
}

#
#  静的APIテーブルへの固定登録
#
$apiDefinition = { "INCLUDE" =>
  { :PARAM => [ { :NAME => :file, :STRING => true }]}}

#
#  静的APIテーブルの読み込み
#
def ReadApiTableFile
  $apiTableFileNames.each do |apiTableFileName|
    if /^(.+):(\w+)$/ =~ apiTableFileName
      apiTableFileName = $1
      apiPhase = $2.to_sym
    end

    if !File.exist?(apiTableFileName)
      error_exit("`#{apiTableFileName}' not found")
      next
    end

    apiFile = File.open(apiTableFileName)
    apiFile.each do |line|
      next if /^#/ =~ line			# コメントをスキップ

      fields = line.split(/\s+/)	# フィールドに分解

      apiName = fields.shift		# API名の取り出し
      if /^(.+)\[(.+)\]$/ =~ apiName
        apiName = $1
        apiDef = { APINAME: apiName, API: $2 }
      else
        apiDef = { APINAME: apiName, API: apiName }
      end
      if !apiPhase.nil?
        apiDef[:PHASE] = apiPhase
      end

      apiParams = []
      fields.each do |param|
        case param
        when /^(\W*)(\w+)(\W*)$/
          prefix = $1
          name = $2
          postfix = $3
          apiParam = { :NAME => name }

          case prefix
          when "#"					# オブジェクト識別名（定義）
            apiParam[:ID_DEF] = true
          when "%"					# オブジェクト識別名（参照）
            apiParam[:ID_REF] = true
          when "."					# 符号無し整数定数式パラメータ
            apiParam[:EXPTYPE] = "unsigned_t"
          when "+"					# 符号付き整数定数式パラメータ
            apiParam[:EXPTYPE] = "signed_t"
            apiParam[:SIGNED] = true
          when "&"					# 一般整数定数式パラメータ
            # do nothing
          when "$"					# 文字列定数式パラメータ
            apiParam[:STRING] = true
          else
            error_exit("`#{param}' is invalid")
          end

          case postfix
          when "*"					# キーを決めるパラメータ
            apiDef[:KEYPAR] = name
          when "?"					# オプションパラメータ
            apiParam[:OPTIONAL] = true
          when "\.\.\."				# リストパラメータ
            apiParam[:LIST] = true
          end
        
        when "{"					# {
          apiParam = { :BRACE => "{" }
        when "{?"					# {?
          apiParam = { :BRACE => "{", :OPTBRACE => true }

        when "}"					# }
          apiParam = { :BRACE => "}" }

        else
          error_exit("`#{param}' is invalid")
        end
        apiParams.push(apiParam)
      end
      apiDef[:PARAM] = apiParams
      $apiDefinition[apiName] = apiDef
    end
    apiFile.close
  end
end

#
#  値取得シンボルテーブルの読み込み
#
def ReadSymvalTable
  $symvalTableFileNames.each do |symvalTableFileName|
    if !File.exist?(symvalTableFileName)
      error_exit("`#{symvalTableFileName}' not found")
      next
    end

    symvalCsv = CSV.open(symvalTableFileName,
						{ skip_blanks: true, skip_lines: /^#/ })
    symvalCsv.each do |record|
      # 変数名
      if record[0].nil?
        error_exit("invalid variable name in " \
						"`#{symvalTableFileName}:#{symvalCsv.to_io.lineno}'")
      end

      symbol = {}
      variable = record[0]

      # 式
      if record[1].nil? || record[1].empty?
        symbol[:EXPR] = variable
      else
        symbol[:EXPR] = record[1]
      end

      # 式の型
      if !record[2].nil? && !record[2].empty?
        case record[2]
        when /^[bB]/				# 真偽値
          symbol[:BOOL] = true
        when /^[uU]/				# 符号無し整数値
          # 何も設定しない
        else						# 符号付き整数値
          symbol[:SIGNED] = true
        end
      end

      # コンパイル条件
      if !record[3].nil? && !record[3].empty?
        symbol[:CONDITION] = record[3]
      end

      # 条件が成立しない時の式
      if !record[4].nil? && !record[4].empty?
        symbol[:ELSE_EXPR] = record[4]
      end

      $symbolValueTable[variable] = symbol
    end
    symvalCsv.close
  end
end

#
#  システムコンフィギュレーションファイルからの読み込みクラス
#
class ConfigFile
  def initialize(fileName)
    @cfgFileName = fileName
    begin
      @cfgFile = File.open(@cfgFileName)
    rescue Errno::ENOENT, Errno::EACCES => ex
      abort(ex.message)
    end
    @lineNo = 0
    @withinComment = false
  end

  def close
    @cfgFile.close
  end

  def getNextLine(withinApi)
    line = @cfgFile.gets
    return(nil) if line.nil?

	line.encode!("UTF-16BE", "UTF-8",	# 不正なバイト列を除外する
					:invalid => :replace,
					:undef => :replace,
					:replace => '?').encode!("UTF-8")
    @lineNo += 1

    line.chomp!
    if @withinComment
      case line
      when /\*\//						# C言語スタイルのコメント終了
        line.sub!(/^.*?\*\//, "")		# 最初の*/にマッチさせる */
        @withinComment = false
      else
        line = ""
      end
    end
    if !@withinComment
      line.gsub!(/\/\*.*?\*\//, "")		# C言語スタイルのコメントの除去
										# 最初の*/にマッチさせる */
      case line
      when /^\s*#/						# プリプロセッサディレクティブ
        if withinApi
          parse_error(self, \
					"preprocessor directive must not be within static API")
          line = ""
        end
      when /\/\*/						# C言語スタイルのコメント開始
        line.sub!(/\/\*.*$/, "")
        @withinComment = true
      when /\/\//						# C++言語スタイルのコメント
        line.sub!(/\/\/.*$/, "")
      end
    end
    return(line)
  end

  def getFileName
    return(@cfgFileName)
  end

  def getLineNo
    return(@lineNo)
  end
end

#
#  システムコンフィギュレーションファイルのパーサークラス
#
class CfgParser
  @@lastApiIndex = 0
  @@currentDomain = nil
  @@currentClass = nil
  @@nestDC = []

  def initialize
    @line = ""
    @skipComma = false						# 次が,であれば読み飛ばす
  end

  #
  #  文字列末まで読む
  #
  def parseString(cfgFile)
    string = ""
    begin
      case @line
      when /^([^"]*\\\\)(.*)$/				# \\まで読む
        string += $1
        @line = $2
      when /^([^"]*\\\")(.*)$/				# \"まで読む
        string += $1
        @line = $2
      when /^([^"]*\")(.*)$/				# "まで読む
        string += $1
        @line = $2
        return(string)
      else									# 行末まで読む
        string += @line + "\n"
        @line = cfgFile.getNextLine(true)
      end
    end while (@line)
    error_exit("unterminated string meets end-of-file")
    return(string)
  end

  #
  #  文字末まで読む
  #
  def parseChar(cfgFile)
    string = ""
    begin
      case @line
      when /^([^']*\\\\)(.*)$/				# \\まで読む
        string += $1
        @line = $2
      when /^([^']*\\\')(.*)$/				# \'まで読む
        string += $1
        @line = $2
      when /^([^']*\')(.*)$/				# 'まで読む
        string += $1
        @line = $2
        return(string)
      else									# 行末まで読む
        string += @line + "\n"
        @line = cfgFile.getNextLine(true)
      end
    end while (@line)
    error_exit("unterminated string meets end-of-file")
    return(string)
  end

  #
  #  改行と空白文字を読み飛ばす
  #
  def skipSpace(cfgFile, withinApi)
    loop do
      return if @line.nil?						# ファイル末であればリターン
      @line.lstrip!								# 先頭の空白を削除
      return if @line != ""						# 空行でなければリターン
      @line = cfgFile.getNextLine(withinApi)	# 次の行を読む
    end
  end

  #
  #  次の文字まで読み飛ばす
  #
  def skipToToken(cfgFile, withinApi=true)
    skipSpace(cfgFile, withinApi)
    if @line.nil?							# ファイル末であればエラー終了
      error_exit("unexpexced end-of-file")
    end
  end

  #
  #  パラメータを1つ読む
  #
  # @lineの先頭からパラメータを1つ読んで，それを文字列で返す．読んだパ
  # ラメータは，@lineからは削除する．パラメータの途中で行末に達した時は，
  # cfgFileから次の行を取り出す．ファイル末に達した時は，nilを返す．
  #
  def parseParam(cfgFile)
    param = ""								# 読んだ文字列
    parenLevel = 0							# 括弧のネストレベル
    skipComma = @skipComma
    @skipComma = false

    skipToToken(cfgFile)					# 次の文字まで読み飛ばす
    begin
      if parenLevel == 0
        case @line
        when /^(\s*,)(.*)$/					# ,
          @line = $2
          if param == "" && skipComma
            skipComma = false
            return(parseParam(cfgFile))		# 再帰呼び出し
          else
            return(param.strip)
          end
        when /^(\s*{)(.*)$/					# {
          if param != ""
            return(param.strip)
          else
            @line = $2
            return("{")
          end
        when /^(\s*\()(.*)$/				# (
          param += $1
          @line = $2
          parenLevel += 1
        when /^(\s*([)}]))(.*)$/			# }か)
          if param != ""
            return(param.strip)
          else
            @line = $3
            @skipComma = true if $2 == "}"
            return($2)
          end
        when /^(\s*\")(.*)$/				# "
          @line = $2
          param += $1 + parseString(cfgFile)
        when /^(\s*\')(.*)$/				# '
          @line = $2
          param += $1 + parseChar(cfgFile)
        when /^(\s*[^,{}()"'\s]+)(.*)$/		# その他の文字列
          param += $1
          @line = $2
        else								# 行末
          param += " "
          @line = cfgFile.getNextLine(true)
        end
      else
        # 括弧内の処理
        case @line
        when /^(\s*\()(.*)$/				# "("
          param += $1
          @line = $2
          parenLevel += 1
        when /^(\s*\))(.*)$/				# ")"
          param += $1
          @line = $2
          parenLevel -= 1
        when /^(\s*\")(.*)$/				# "
          @line = $2
          param += $1 + parseString(cfgFile)
        when /^(\s*\')(.*)$/				# '
          @line = $2
          param += $1 + parseChar(cfgFile)
        when /^(\s*[^()"'\s]+)(.*)$/		# その他の文字列
          param += $1
          @line = $2
        else								# 行末
          param += " "
          @line = cfgFile.getNextLine(true)
        end
      end
    end while (@line)
    return(param.strip)
  end

  def getParam(apiParam, param, cfgFile)
    if param == ""
      if !apiParam.has_key?(:OPTIONAL)
        parse_error(cfgFile, "unexpected `,'")
      end
      return(param)
    end

    if apiParam.has_key?(:ID_DEF) || apiParam.has_key?(:ID_REF)
      if (/^[A-Za-z_]\w*$/ !~ param)
        parse_error(cfgFile, "`#{param}' is illegal object ID")
      end
      return(param)
    end

    if apiParam.has_key?(:STRING)
      return(param.unquote)
    else
      return(param)
    end
  end

  def parseApi(cfgFile, apiName)
    # 静的APIの読み込み
    staticApi = {}
    tooFewParams = false
    skipUntilBrace = 0

    skipToToken(cfgFile)					# 次の文字まで読み飛ばす
    if (/^\((.*)$/ =~ @line)
      @line = $1

      staticApi[:APINAME] = apiName
      staticApi[:_FILE_] = cfgFile.getFileName
      staticApi[:_LINE_] = cfgFile.getLineNo
      apiDef = $apiDefinition[apiName]
      param = parseParam(cfgFile)

      apiDef[:PARAM].each do |apiParam|
        return(staticApi) if param.nil?		# ファイル末であればリターン

        if skipUntilBrace > 0
          # API定義を}までスキップ中
          if apiParam.has_key?(:BRACE)
            case apiParam[:BRACE]
            when "{"
              skipUntilBrace += 1
            when "}"
              skipUntilBrace -= 1
            end
          end
        elsif apiParam.has_key?(:OPTIONAL)
          if /^([{})])$/ !~ param
            store_param = getParam(apiParam, param, cfgFile)
            if store_param != ""
              staticApi[apiParam[:NAME]] = store_param
            end
            param = parseParam(cfgFile)
          end
        elsif apiParam.has_key?(:LIST)
          staticApi[apiParam[:NAME]] = []
          while /^([{})])$/ !~ param
            staticApi[apiParam[:NAME]].push(getParam(apiParam, param, cfgFile))
            param = parseParam(cfgFile)
            break if param.nil?				# ファイル末の場合
          end
        elsif apiParam.has_key?(:OPTBRACE)
          if param == apiParam[:BRACE]
            param = parseParam(cfgFile)
            break if param.nil?				# ファイル末の場合
          else
            if param == ""
              param = parseParam(cfgFile)
              break if param.nil?			# ファイル末の場合
            elsif /^([})])$/ !~ param
              parse_error(cfgFile, "`{...}' expected before #{param}")
            end
            skipUntilBrace += 1          	# API定義を}までスキップ
          end
        elsif !apiParam.has_key?(:BRACE)
          if /^([{})])$/ !~ param
            staticApi[apiParam[:NAME]] = getParam(apiParam, param, cfgFile)
            param = parseParam(cfgFile)
          elsif !tooFewParams
            parse_error(cfgFile, "too few parameters before `#{$1}'")
            tooFewParams = true
          end
        elsif param == apiParam[:BRACE]
          param = parseParam(cfgFile)
          tooFewParams = false
        else
          parse_error(cfgFile, "`#{apiParam[:BRACE]}' expected before #{param}")
          # )かファイル末まで読み飛ばす
          loop do
            param = parseParam(cfgFile)
            break if (param.nil? || param == ")")
          end
          break
        end
      end

      # 期待されるパラメータをすべて読んだ後の処理
      if param != ")"
        begin
          param = parseParam(cfgFile)
          return(staticApi) if param.nil?	# ファイル末であればリターン
        end while param != ")"
        parse_error(cfgFile, "too many parameters before `)'")
      end
    else
      parse_error(cfgFile, "syntax error: #{@line}")
      @line = ""
    end
    return(staticApi)
  end

  def parseOpenBrace(cfgFile)
    # {の読み込み
    skipToToken(cfgFile)					# 次の文字まで読み飛ばす
    if (/^\{(.*)$/ =~ @line)
      @line = $1
    else
      parse_error(cfgFile, "`{' expected before #{@line}")
    end
  end

  def parseFile(cfgFileName)
    cfgFiles = [ ConfigFile.new(cfgFileName) ]
    @line = ""
    loop do
      cfgFile = cfgFiles.last

      skipSpace(cfgFile, false)				# 改行と空白文字を読み飛ばす
      if @line.nil?
        # ファイル末の処理
        cfgFiles.pop.close
        if cfgFiles.empty?
          break								# パース処理終了
        else
          @line = ""						# 元のファイルに戻って続ける
        end
      elsif /^;(.*)$/ =~ @line
        # ;は読み飛ばす
        @line = $1
      elsif /^#/ =~ @line
        # プリプロセッサディレクティブを読む
        case @line
        when /^#include\b(.*)$/
          $includeFiles.push($1.strip)
        when /^#(ifdef|ifndef|if|endif|else|elif)\b/
          directive = { :DIRECTIVE => @line.strip }
          $cfgFileInfo.push(directive)
        else
          parse_error(cfgFile, "unknown preprocessor directive: #{@line}")
        end
        @line = ""
      elsif (/^([A-Z_][A-Z0-9_]*)\b(.*)$/ =~ @line)
        apiName = $1
        @line = $2

        case apiName
        when "KERNEL_DOMAIN"
          if $supportDomain.nil?
            parse_warning(cfgFile, "`KERNEL_DOMAIN' is not supported")
          end
          if !@@currentDomain.nil?
            parse_error(cfgFile, "`DOMAIN' must not be nested")
          end
          @@currentDomain = "TDOM_KERNEL"
          parseOpenBrace(cfgFile)
          @@nestDC.push("domain")
        when "DOMAIN"
          if $supportDomain.nil?
            parse_warning(cfgFile, "`DOMAIN' is not supported")
          end
          if !@@currentDomain.nil?
            parse_error(cfgFile, "`DOMAIN' must not be nested")
          end
          domid = parseParam(cfgFile).sub(/^\((.+)\)$/m, "\\1").strip
          if (/^[A-Za-z_]\w*$/ !~ domid)
            parse_error(cfgFile, "`#{domid}' is illegal domain ID")
          else
            if !$domainId.has_key?(domid)
              if $inputObjid.has_key?(domid)
                # ID番号入力ファイルに定義されていた場合
                $domainId[domid] = $inputObjid[domid]
                if $domainId[domid] > 32
                  error_exit("domain ID for `#{domid}' is too large")
                end
              else
                $domainId[domid] = nil
              end
            end
            @@currentDomain = domid
          end
          parseOpenBrace(cfgFile)
          @@nestDC.push("domain")
        when "CLASS"
          if $supportClass.nil?
            parse_warning(cfgFile, "`CLASS' is not supported")
          end
          if !@@currentClass.nil?
            parse_error(cfgFile, "`CLASS' must not be nested")
          end
          @@currentClass = parseParam(cfgFile).sub(/^\((.+)\)$/m, "\\1").strip
          @@classFile = cfgFile.getFileName
          @@classLine = cfgFile.getLineNo
          parseOpenBrace(cfgFile)
          @@nestDC.push("class")
        else
          if $apiDefinition.has_key?(apiName)
            # 静的APIを1つ読む
            staticApi = parseApi(cfgFile, apiName)
            if staticApi.empty?
              # ファイル末か文法エラー
            elsif (staticApi[:APINAME] == "INCLUDE")
              # INCLUDEの処理
              includeFilePath = SearchFilePath(staticApi[:file])
              if includeFilePath.nil?
                parse_error(cfgFile, "`#{staticApi[:file]}' not found")
              else
                $dependencyFiles.push(includeFilePath)
                cfgFiles.push(ConfigFile.new(includeFilePath))
              end
            else
              # 静的APIの処理
              if !@@currentDomain.nil?
                staticApi[:DOMAIN] = @@currentDomain
              end
              if !@@currentClass.nil?
                staticApi[:CLASS] = @@currentClass
                staticApi[:CLASS_FILE_] = @@classFile
                staticApi[:CLASS_LINE_] = @@classLine
              end
              staticApi[:INDEX] = (@@lastApiIndex += 1)
              $cfgFileInfo.push(staticApi)
            end
          else
            parse_error(cfgFile, "unknown static API: #{apiName}")
          end
        end
      elsif (/^\}(.*)$/ =~ @line)
        # }の処理
        if @@nestDC.size > 0
          case @@nestDC.pop
          when "domain"
            @@currentDomain = nil
          when "class"
            @@currentClass = nil
          end
        else
          error_exit("unexpected `}'")
        end
        @line = $1
      else
        parse_error(cfgFile, "syntax error: #{@line}")
        @line = ""
      end
    end
  end
end

#
#  cfg1_out.cの生成
#
module Cfg1OutC
  #
  #  静的APIのファイル名と行番号の出力
  #
  def self.OutLineNumber(cfgInfo)
    @cfg1Out.add("#line #{cfgInfo[:_LINE_]} \"#{cfgInfo[:_FILE_]}\"")
  end

  #
  #  クラス記述のファイル名と行番号の出力
  #
  def self.OutClassLineNumber(cfgInfo)
    @cfg1Out.add("#line #{cfgInfo[:CLASS_LINE_]} \"#{cfgInfo[:CLASS_FILE_]}\"")
  end

  #
  #  パラメータに関する定義の出力
  #
  def self.OutParamDef(param, index, apiParam, cfgInfo)
    if apiParam.has_key?(:ID_DEF)
      @cfg1Out.add("#define #{param}\t(<>)")
    elsif apiParam.has_key?(:EXPTYPE)
      OutLineNumber(cfgInfo)
      @cfg1Out.add("const #{apiParam[:EXPTYPE]} #{CFG1_PREFIX}valueof_" \
								"#{apiParam[:NAME]}_#{index} = " \
								"(#{apiParam[:EXPTYPE]})(#{param});")
    end
  end

  #
  #  cfg1_out.cの生成（メインの処理）
  #
  def self.Generate
    @cfg1Out = GenFile.new(CFG1_OUT_C)

    @cfg1Out.append(<<EOS)
/* #{CFG1_OUT_C} */
#define TOPPERS_CFG1_OUT
#include "kernel/kernel_int.h"
EOS

    # インクルードヘッダファイル
    $includeFiles.each do |file|
      @cfg1Out.add("#include #{file}")
    end

    @cfg1Out.add(<<EOS)

#ifdef INT64_MAX
  typedef int64_t signed_t;
  typedef uint64_t unsigned_t;
#else
  typedef int32_t signed_t;
  typedef uint32_t unsigned_t;
#endif

#include "#{CFG1_OUT_TARGET_H}"

#if defined(SIL_ENDIAN_BIG) && defined(SIL_ENDIAN_LITTLE)
#error Both SIL_ENDIAN_BIG and SIL_ENDIAN_LITTLE are defined.
#endif
#if !defined(SIL_ENDIAN_BIG) && !defined(SIL_ENDIAN_LITTLE)
#error Neither SIL_ENDIAN_BIG nor SIL_ENDIAN_LITTLE is defined.
#endif

const uint32_t #{CFG1_MAGIC_NUM} = 0x12345678;
const uint32_t #{CFG1_SIZEOF_SIGNED} = sizeof(signed_t);
EOS

    # 値取得シンボルの処理
    $symbolValueTable.each do |symbolName, symbolData|
      if symbolData.has_key?(:BOOL) || symbolData.has_key?(:SIGNED)
        type = "signed_t"
      else
        type = "unsigned_t"
      end
      if symbolData.has_key?(:CONDITION)
        @cfg1Out.add("#if #{symbolData[:CONDITION]}")
      end
      @cfg1Out.add("const #{type} #{CFG1_PREFIX}#{symbolName} = " \
								"(#{type})(#{symbolData[:EXPR]});")
      if symbolData.has_key?(:ELSE_EXPR)
        @cfg1Out.add("#else")
        @cfg1Out.add("const #{type} #{CFG1_PREFIX}#{symbolName} = " \
								"(#{type})(#{symbolData[:ELSE_EXPR]});")
      end
      if symbolData.has_key?(:CONDITION)
        @cfg1Out.add("#endif")
      end
    end
    @cfg1Out.add

    # ドメインIDの定義の生成
    $domainId.each do |domainName, domainVal|
      if domainVal > 0
        @cfg1Out.add("#define #{domainName} #{domainVal}")
      end
    end
    @cfg1Out.add

    # 静的API／プリプロセッサディレクティブの処理
    $cfgFileInfo.each do |cfgInfo|
      if cfgInfo.has_key?(:DIRECTIVE)
        @cfg1Out.add2(cfgInfo[:DIRECTIVE])
      else
        apiDef = $apiDefinition[cfgInfo[:APINAME]]
        apiIndex = cfgInfo[:INDEX]
        OutLineNumber(cfgInfo)
        @cfg1Out.add("const unsigned_t #{CFG1_PREFIX}static_api_" \
										"#{apiIndex} = #{apiIndex};")
        apiDef[:PARAM].each do |apiParam|
          next unless apiParam.has_key?(:NAME)
          paramName = apiParam[:NAME]
          next unless cfgInfo.has_key?(paramName)	# パラメータがない場合
          paramData = cfgInfo[paramName]

          if apiParam.has_key?(:LIST)
            paramData.each.with_index(1) do |param, index|
              OutParamDef(param, "#{apiIndex}_#{index}", apiParam, cfgInfo)
            end
          else
            OutParamDef(paramData, "#{apiIndex}", apiParam, cfgInfo)
          end
        end
        if cfgInfo.has_key?(:CLASS)
          # クラスIDの取得のための処理
          OutClassLineNumber(cfgInfo)
          @cfg1Out.add("const signed_t #{CFG1_PREFIX}valueof_CLASS_" \
                      "#{apiIndex} = (signed_t)(#{cfgInfo[:CLASS]});")
        end
        @cfg1Out.add
      end
    end
  end
end

#
#  パス1の処理
#
def Pass1
  # 
  #  タイムスタンプファイルの指定
  # 
  $timeStampFileName = CFG1_OUT_TIMESTAMP

  #
  #  静的APIテーブルの読み込み
  #
  ReadApiTableFile()

  #
  #  値取得シンボルテーブルの読み込み
  #
  ReadSymvalTable()

  #
  #  システムコンフィギュレーションファイルの読み込み
  #
  $cfgFileInfo = []
  $dependencyFiles = $configFileNames.dup
  $includeFiles = []
  $domainId = { "TDOM_KERNEL" => -1, "TDOM_NONE" => -2 }
  $configFileNames.each do |configFileName|
    CfgParser.new.parseFile(configFileName)
  end
  abort if $errorFlag					# エラー発生時はabortする

  #
  #  ドメインIDの割当て処理
  #
  nextDomainId = 1
  $domainId.each do |domainName, domainVal|
    if domainVal.nil?
      while $domainId.has_value?(nextDomainId)
        nextDomainId += 1
      end
      $domainId[domainName] = nextDomainId
      if nextDomainId > 32
        error_exit("too large number of user domains")
      end
      nextDomainId += 1
    end
  end

  #
  #  cfg1_out.cの生成
  #
  Cfg1OutC::Generate()

  #
  #  依存関係の出力
  #
  if !$dependencyFileName.nil?
    if $dependencyFileName == ""
      depFile = STDOUT
    else
      begin
        depFile = File.open($dependencyFileName, "w")
      rescue Errno::ENOENT, Errno::EACCES => ex
        abort(ex.message)
      end
    end

    depFile.print("#{CFG1_OUT_TIMESTAMP}:")
    $dependencyFiles.each do |fileName|
      depFile.print(" #{fileName}")
    end
    depFile.puts("")

    if $dependencyFileName != ""
      depFile.close
    end
  end

  #
  #  パス2に引き渡す情報をファイルに生成
  #
  if $omitOutputDb.nil?
    db = PStore.new(CFG1_OUT_DB)
    db.transaction do
      db[:apiDefinition] = $apiDefinition
      db[:symbolValueTable] = $symbolValueTable
      db[:cfgFileInfo] = $cfgFileInfo
      db[:includeFiles] = $includeFiles
      db[:domainId] = $domainId
    end
  end
end
