#!/usr/bin/env ruby
# -*- coding: utf-8 -*-
#
#  TOPPERS Software
#      Toyohashi Open Platform for Embedded Real-Time Systems
# 
#  Copyright (C) 2007-2016 by Embedded and Real-Time Systems Laboratory
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
#  $Id: gentest.rb 796 2017-07-19 14:21:08Z ertl-hiro $
# 

#
#		テストプログラム生成ツール
#
	
#
#  生成動作を決めるための設定
#
$parameterDefinition = {
  "get_tst" => { 2 => "STAT" },
  "get_pri" => { 2 => "PRI" },
  "get_inf" => { 1 => "intptr_t" },
  "ref_tsk" => { 2 => "T_RTSK" },
  "ref_sem" => { 2 => "T_RSEM" },
  "wai_flg" => { 4 => "FLGPTN" },
  "pol_flg" => { 4 => "FLGPTN" },
  "twai_flg" => { 4 => "FLGPTN" },
  "ref_flg" => { 2 => "T_RFLG" },
  "rcv_dtq" => { 2 => "intptr_t" },
  "prcv_dtq" => { 2 => "intptr_t" },
  "trcv_dtq" => { 2 => "intptr_t" },
  "ref_dtq" => { 2 => "T_RDTQ" },
  "rcv_pdq" => { 2 => "intptr_t" , 3 => "PRI" },
  "prcv_pdq" => { 2 => "intptr_t" , 3 => "PRI" },
  "trcv_pdq" => { 2 => "intptr_t" , 3 => "PRI" },
  "ref_pdq" => { 2 => "T_RPDQ" },
  "ref_mtx" => { 2 => "T_RMTX" },
  "ref_mbf" => { 2 => "T_RMBF" },
  "get_mpf" => { 2 => "void *" },
  "pget_mpf" => { 2 => "void *" },
  "tget_mpf" => { 2 => "void *" },
  "ref_mpf" => { 2 => "T_RMPF" },
  "get_tim" => { 1 => "SYSTIM" },
  "ref_cyc" => { 2 => "T_RCYC" },
  "ref_alm" => { 2 => "T_RALM" },
  "ref_ovr" => { 2 => "T_ROVR" },
  "get_tid" => { 1 => "ID" },
  "get_did" => { 1 => "ID" },
  "get_lod" => { 2 => "uint_t" },
  "mget_lod" => { 3 => "uint_t" },
  "get_nth" => { 3 => "ID" },
  "mget_nth" => { 4 => "ID" },
  "get_ipm" => { 1 => "PRI" },
  "get_som" => { 1 => "ID" }
}

$functionParameters = {
  "target_hrt_set_event" => "HRTCNT hrtcnt"
}
  
$functionValue = {
  "target_hrt_get_current" => "HRTCNT"
}
  
$functionReturn = {
  "target_hrt_get_current" => "0U"
}
  
$functionCheckParameter = {
  "target_hrt_set_event" => "hrtcnt"
}

#
#  処理単位のコードを格納するクラス
#
class PUCode
  # 初期化
  def initialize(puName)
    @puName = puName						# 処理単位の名前
    @currentCount = ""						# 処理カウント
    @countFlag = false						# 処理カウントが使われたか
    @code = Hash.new { |h,k| h[k] = [] }	# 処理単位のコード
    @variableList = {}						# 処理単位の変数
    @silFlag = false						# SILが使われたか

    # 処理カウント変数名の生成
    case puName
    when /^TASK([0-9]*)$/
      @count_var = "task#{$1}_count"
    when /^CYC([0-9]*)$/
      @count_var = "cyclic#{$1}_count"
    when /^ALM([0-9]*)$/
      @count_var = "alarm#{$1}_count"
    when /^OVR$/
      @count_var = "overrun_count"
    when /^ISR([0-9]*)$/
      @count_var = "isr#{$1}_count"
    when /^INTHDR([0-9]*)$/
      @count_var = "inthdr#{$1}_count"
    when /^CPUEXC([0-9]*)$/
      @count_var = "cpuexc#{$1}_count"
    when /^EXTSVC([0-9]*)$/
      @count_var = "extsvc#{$1}_count"
    else
      @count_var = puName + "_count"
    end
  end

  # 処理カウントの設定
  def setCount(count)
    @countFlag = true if count != ""
    @currentCount = count
  end

  # 処理カウントのインクリメント
  def incrementCount
    if @currentCount != ""
      @currentCount = (@currentCount.to_i + 1).to_s
    end
  end

  # コードの追加
  def append(*lines)
    lines.each do |line|
      @code[@currentCount].push(line)
        end
  end

  # 変数の追加
  def addVariable(varName, typeName)
    @variableList[varName] = typeName
  end

  # SILの使用
  def useSil
    @silFlag = true
  end

  # 処理単位のコードの出力
  def generateCode
    # 不要な処理単位の判定
    return if @code.length == 0

    # 処理カウント変数の生成
    if @countFlag
      print("\nstatic uint_t\t#{@count_var} = 0;\n")
    end

    # 関数ヘッダの生成
    case @puName
    when /^TASK([0-9]*)$/
      print("\nvoid\n")
      print("task#{$1}(intptr_t exinf)\n")
    when /^CYC([0-9]*)$/
      print("\nvoid\n")
      print("cyclic#{$1}_handler(intptr_t exinf)\n")
    when /^ALM([0-9]*)$/
      print("\nvoid\n")
      print("alarm#{$1}_handler(intptr_t exinf)\n")
    when /^OVR$/
      print("\nvoid\n")
      print("overrun_handler(ID tskid, intptr_t exinf)\n")
    when /^ISR([0-9]*)$/
      print("\nvoid\n")
      print("isr#{$1}(intptr_t exinf)\n")
    when /^INTHDR([0-9]*)$/
      print("\nvoid\n")
      print("inthdr#{$1}_handler(void)\n")
    when /^CPUEXC([0-9]*)$/
      print("\nvoid\n")
      print("cpuexc#{$1}_handler(void *p_excinf)\n")
    when /^EXTSVC([0-9]*)$/
      print("\nER_UINT\n")
      print("extsvc#{$1}_routine")
      print("(intptr_t par1, intptr_t par2, intptr_t par3,\n")
      print("\t\t\t\t\t\t\t\tintptr_t par4, intptr_t par5, ID cdmid)\n")
    else
      if $functionValue[@puName]
        print("\n#{$functionValue[@puName]}\n")
      else
        print("\nvoid\n")
      end
      print(@puName)
      if $functionParameters[@puName]
        print("(#{$functionParameters[@puName]})\n")
      else
        print("(void)\n")
      end
    end

    print("{\n")

    @variableList.each do |varName, varType|
      if /^(.+)\w*\*$/ =~ varType
        varBaseType = $1
        print("\t#{varBaseType}")
        print(varBaseType.length < 4 ? "\t\t*" : "\t*")
      else
        print("\t#{varType}")
        print(varType.length < 4 ? "\t\t" : "\t")
      end
      print("#{varName};\n")
    end
    if @silFlag
      print("\tSIL_PRE_LOC;\n")
    end
    print("\n")

    if @countFlag
      print("\tswitch (++#{@count_var}) {\n")
      @code.keys.sort_by { |c| c.to_i }.each do |count|
        print("\tcase #{count}:\n")
        @code[count].each do |line|
          print("\t",line) if line != ""
          print("\n")
        end
        print("\t\tcheck_point(0);\n\n")
      end
      print("\tdefault:\n")
      print("\t\tcheck_point(0);\n")
      print("\t}\n")
    else
      @code[""].each do |line|
        print(line,"\n")
      end
    end

    print("\tcheck_point(0);\n")
    if /^EXTSVC([0-9]*)$/ =~ @puName
      print("\treturn(E_SYS);\n")
    elsif $functionReturn[@pu_nama]
      print("\treturn(#{$functionReturn[@puName]});\n")
    end
    print("}\n")
  end
end

#
#  サービスコール呼び出しの読み込み
#
def genServiceCall(pu, svc_call, error_code)
  pu.addVariable("ercd", "ER_UINT")
  pu.append("\tercd = #{svc_call};")

  if /^([a-z_]+)\((.*)\)$/ =~ svc_call
    svcName = $1;
    params = $2.split(/\s*,\s*/)

    if $parameterDefinition.has_key?(svcName)
      $parameterDefinition[svcName].each do |pos, type|
        if params.size >= pos
          varName = params[pos - 1].sub(/^\&/, "")
          pu.addVariable(varName, type)
        end
      end
    end
  end

  if !error_code
    # E_OKが返る場合
    pu.append("\tcheck_ercd(ercd, E_OK);", "")
  elsif error_code == "noreturn"
    # リターンしない場合
    pu.append("")
  else
    pu.append("\tcheck_ercd(ercd, #{error_code});", "")
  end
end

#
#  テスト開始コードの生成
#
def testStartCode(pu)
  # テスト開始コードは一度のみ出力する
  if $startFlag == 0
    pu.append("\ttest_start(__FILE__);", "")
    $startFlag = 1
  end
end

#
#  ターゲット依存部関数の振る舞いの読み込み
#
def targetFunction(line, checkNum)
  if /^([a-zA-Z_]+)\s*(.*)$/ =~ line
    functionName = $1
    line = $2
    if (pu = $puList[functionName]).nil?
      # 新しい処理単位の生成
      pu = $puList[functionName] = PUCode.new(functionName)
      pu.setCount("1")
      testStartCode(pu)
    end
  end

  if /\-\>\s*([^\s]+)\s*(.*)$/ =~ line
    retval = $1
    line = $2
  end
  if /\<\-\s*([^\s]+)\s*(.*)$/ =~ line
    param = $1
    line = $2
  end

  pu.append("\tcheck_point(#{checkNum});");
  if param && $functionCheckParameter[functionName]
    pu.append(sprintf("\tcheck_assert(%s == %s);",
					$functionCheckParameter[functionName], param), "")
  end
  if retval
    pu.append("\treturn(#{retval});", "")
  else
    pu.append("\treturn;", "")
  end
  pu.incrementCount()
end

#
#  テストスクリプトの読み込み
#
def parseLine(line)
  if /^==\s*(([a-zA-Z_]+)[0-9]*)(.*)$/ =~ line
    # 処理単位の開始
    $procFlag = 1
    puName = $1
    line2 = $3
    if (pu = $puList[puName]).nil?
      # 新しい処理単位の生成
      pu = $puList[puName] = PUCode.new(puName)
    end
    $currentPu = pu

    case line2
    when /^\-([0-9]+)(.*)$/
      pu.setCount($1)
    when /^\-[nN](.*)$/
      # do nothing.
    else
      pu.setCount("")
    end
    testStartCode(pu) if /^START/ !~ puName
  elsif $procFlag != 0
    pu = $currentPu
    if /^([0-9]+\:)\s*(.*)$/ =~ line
      # チェックポイント番号の処理
      originalCheckNum = $1
      line = $2
      checkNum = ($lastCheckPoint += 1).to_s
      $outputLine.sub!(/#{originalCheckNum}/, "#{checkNum}:")

      case line
      when /^END$/
        pu.append("\tcheck_finish(#{checkNum});")
        $procFlag = 0
        return
      when /^HOOK\((.*)\)$/
        pu.append(sprintf("\t#{$1};", checkNum))
        return
      when /^\[(.*)\]$/
        targetFunction($1, checkNum)
        return
      else
        pu.append("\tcheck_point(#{checkNum});")
      end
    end

    case line
    when /^((assert|state|ipm)\(.*\))$/
      pu.append("\tcheck_#{$1};", "")
    when /^(call|DO)\((.*)\)$/
      call_string = $2
      pu.append("\t#{call_string};", "")
      pu.useSil() if /^SIL_..._INT\(\)$/ =~ call_string
    when /^RETURN((\(.*\))?)$/
      pu.append("\treturn#{$1};", "")
      pu.incrementCount()
    when /^GOTO\((.*)\)$/
      pu.append("\tgoto #{$1};", "")
    when /^LABEL\((.*)\)$/
      pu.append("#{$1}:", "")
    when /^([a-z_]+\(.*\))\s*(\-\>\s*([A-Za-z0-9_]*))?\s*$/
      genServiceCall(pu, $1, $3)
    else
      warn("Error: #{line}")
    end
  end
end

#
#  エラーチェック
#
if ARGV.length < 1
  abort("Usage: ruby gentest.rb <test_program>")
end

#
#  初期化
#
inFileName = ARGV.shift

#
#  スクリプトファイル読み込み処理
#
$lastCheckPoint = 0		# 最後のチェックポイント番号
$procFlag = 0				# スクリプト処理中フラグ
$startFlag = 0			# テスト開始コードの出力フラグ
$currentPu = nil			# 読み込み中の処理単位
$puList = {}				# 処理単位のリスト

begin
  inFile = File.open(inFileName)
rescue Errno::ENOENT, Errno::EACCES => ex
  abort(ex.message)
end

while line = inFile.gets do
  $outputLine = line.dup
  line.chomp!
  line.sub!(/^\s*\*\s*/, "")
  line.sub!(/\s*\/\/.*$/, "")
  line.sub!(/\s*\.\.\..*$/, "")

  while line.sub!(/\\$/, "") do
    line1 = inFile.gets
    $outputLine += line1.dup
    line1.chomp!
    line1.sub!(/^\s*\*\s*/, "")
    line1.sub!(/\s*\/\/.*$/, "")
    line1.sub!(/\s*\.\.\..*$/, "")
    line += line1
  end
  parseLine(line) if line != ""
  print($outputLine)
  break if /DO NOT DELETE THIS LINE/ =~ $outputLine
end

#
#  テストプログラム出力処理
#
$puList.keys.sort.each do |puName|
  $puList[puName].generateCode()
end
