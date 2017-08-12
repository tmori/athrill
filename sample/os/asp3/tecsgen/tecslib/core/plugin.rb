# -*- coding: utf-8 -*-
#
#  TECS Generator
#      Generator for TOPPERS Embedded Component System
#  
#   Copyright (C) 2008-2014 by TOPPERS Project
#--
#   上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
#   ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
#   変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
#   (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
#       権表示，この利用条件および下記の無保証規定が，そのままの形でソー
#       スコード中に含まれていること．
#   (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
#       用できる形で再配布する場合には，再配布に伴うドキュメント（利用
#       者マニュアルなど）に，上記の著作権表示，この利用条件および下記
#       の無保証規定を掲載すること．
#   (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
#       用できない形で再配布する場合には，次のいずれかの条件を満たすこ
#       と．
#     (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
#         作権表示，この利用条件および下記の無保証規定を掲載すること．
#     (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
#         報告すること．
#   (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
#       害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
#       また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
#       由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
#       免責すること．
#  
#   本ソフトウェアは，無保証で提供されているものである．上記著作権者お
#   よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
#   に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
#   アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
#   の責任を負わない．
#  
#   $Id: plugin.rb 2633 2017-04-02 06:02:05Z okuma-top $
#++

#== Plugin クラス
# ThroughPlugin, SignaturePlugin, CelltypePlugin に include する
class Plugin < Node
#@error_backlog:: [msg1, msg2, ... ]   @locale が設定される前に発生したエラー

  PluginArgProc = {
    "silent"  => Proc.new { |obj,rhs| obj.set_silent rhs },
  }

  def initialize
    super
    @b_silent = false
    @locale = nil       # set_locale が呼び出されるまで nil となる
    @error_backlog = []
  end

  #=== Plugin#cdl_error
  # set_locale が呼び出されるまで @error_backlog に保存し保留する
  def cdl_error *arg
    if @locale then
      Generator.error2( @locale, *arg )
    else
      @error_backlog << arg
    end
  end

  #=== locale を設定する
  # Node は initialize で locale を設定するが、plugin は parse とは
  # 異なるタイミング new されるため、locale を再設定する
  # このメソッドを2度呼び出すと @error_backlog のエラーが2度出力されてしまう
  def set_locale locale
    @locale = locale
    @error_backlog.each { |arg|
      Generator.error2( locale, *arg )
    }
  end

### 構文解釈 または 意味解析段階で呼び出されるメソッド ###
# generate 指定子の場合、構文解釈次第(end_of_parseで)呼び出される
# generate 文の場合、出現次第呼び出される
  ### 意味解析段階で呼び出されるメソッド ### <<< コメント誤り (V1.4.2)
  #===  CDL ファイルの生成
  #      typedef, signature, celltype, cell のコードを生成
  #      重複して生成してはならない
  #      すでに生成されている場合は出力しないこと。
  #      もしくは同名の import により、重複を避けること。
  #file::        FILE       生成するファイル
  def gen_cdl_file file
  end


### コード生段階で呼び出されるメソッド ###
  #=== プラグインは gen_ep_func を提供するか
  # gen_ep_func 定義   ⇒ テンプレートではない、セルタイプコード(tCelltype.c)を生成 
  # gen_ep_func 未定義 ⇒ テンプレート(tCelltype_templ.c)を生成
  def gen_ep_func?
    self.class.method_defined?( :gen_ep_func_body )
  end

  #===  受け口関数の本体(C言語)を生成する
  #     プラグインの場合、変更する必要のないセルタイプコードを生成する
  #     このメソッドが未定義であれば、プラグインはセルタイプコードを生成しない (通常通りテンプレートを生成する)
  #      gen_cdl_file の中で生成されたセルタイプに対して呼び出される
  #file::           FILE        出力先ファイル (tCelltype.c)
  #b_singleton::    bool        true if singleton
  #ct_name::        Symbol
  #global_ct_name:: string
  #sig_name::       string
  #ep_name::        string
  #func_name::      string
  #func_global_name:: string
  #func_type::      class derived from Type
#  def gen_ep_func_body( file, b_singleton, ct_name, global_ct_name, sig_name, ep_name, func_name, func_global_name, func_type, params )
#  end

  #===  受け口関数の preamble (C言語)を生成する
  #     必要なら preamble 部に出力する
  #      gen_cdl_file の中でで生成されたセルタイプに対して呼び出される
  #file::           FILE        出力先ファイル
  #b_singleton::    bool        true if singleton
  #ct_name::        Symbol
  #global_ct_name:: string
  def gen_preamble( file, b_singleton, ct_name, global_ct_name )
    # デフォルトでは何も出力しない
  end

  #===  受け口関数の postamble (C言語)を生成する
  #     必要なら postamble 部に出力する
  #      gen_cdl_file の中で生成されたセルタイプに対して呼び出される
  #file::           FILE        出力先ファイル
  #b_singleton::    bool        true if singleton
  #ct_name::        Symbol
  #global_ct_name:: string
  def gen_postamble( file, b_singleton, ct_name, global_ct_name )
    # デフォルトでは何も出力しない
  end

  #=== gen_cdl_file の中で生成されたセルタイプに新しいセルが生成された
  # どのセルタイプかは cell.get_celltype で分かる
  #
  #file::           FILE        出力先ファイル
  #b_singleton::    bool        true if singleton
  #ct_name::        Symbol
  #global_ct_name:: string
  def new_cell cell
    # デフォルトでは何もしない
  end

### プラグイン引数の解釈 ###
  def parse_plugin_arg
    arg = @plugin_arg_str.dup

    # 改行を消す
    arg.gsub!( /\\\n/, "" )

    while arg != ""

      # 前の空白読み飛ばす
      arg.sub!( /\A\s*(?:\\\n)*\s*(.*)/, '\1')

      #  識別子取得
      if arg =~ /\A[a-zA-Z_]\w*/ then
        ident = $~
        arg = $'
      else
        cdl_error( "P1001 plugin arg: cannot find identifier in $1" , arg )
        return
      end
      
      # 前の空白読み飛ばす
      arg.sub!( /\A\s*(?:\\\n)*\s*(.*)/, '\1')

      if arg =~ /=/ then
        arg = $'
      else
        cdl_error( "P1002 plugin arg: expecting \'=\' not \'$1\'" , arg )
        return
      end

      # 前の空白読み飛ばす
      arg.sub!( /\A\s*(?:\\\n)*\s*(.*)/, '\1')

      # 右辺文字列
      if arg =~ /\A\\"(.*?)\\"\s*,/ then      # \"  \" で囲まれている場合
        rhs = $1
        remain = $'
      elsif arg =~ /\A%(.*?)%\s*,/ then      # %   % で囲まれている場合
        rhs = $1
        remain = $'
      elsif arg =~ /\A!(.*?)!\s*,/ then    # $   $ で囲まれている場合
        rhs = $1
        remain = $'
      elsif arg =~ /\A'(.*?)'\s*,/ then    # $   $ で囲まれている場合
        rhs = $1
        remain = $'
      elsif  arg =~ /\A\\"(.*?)\\"\s*,/ then  # || にも [,$] にもできなかった
        rhs = $1
        remain = $'
      # elsif arg =~ /\A(.*?)\s*$/ then
      elsif arg =~ /\A\\"(.*?)\\"\s*\z/ then      # \"  \" で囲まれている場合
        rhs = $1
        remain = $'
      elsif arg =~ /\A%(.*?)%\s*\z/ then      # %   % で囲まれている場合
        rhs = $1
        remain = $'
      elsif arg =~ /\A!(.*?)!\s*\z/ then    # $   $ で囲まれている場合
        rhs = $1
        remain = $'
      elsif arg =~ /\A'(.*?)'\s*\z/ then    # $   $ で囲まれている場合
        rhs = $1
        remain = $'
      elsif  arg =~ /\A\\"(.*?)\\"\s*\z/ then  # || にも [,$] にもできなかった
        rhs = $1
        remain = $'
      elsif arg =~ /\A(.*?),/ then
        rhs = $1
        remain = $'
        # 前の空白読み飛ばす
        rhs.sub!( /\A\s*(.*)\s*\z/, '\1')
      elsif arg =~ /\A(.*?)\s*\z/ then
        rhs = $1
        remain = $'
      else
        cdl_error( "P1003 plugin arg: unexpected $1" , arg )
        return
      end

      # 0文字の文字列を to_sym すると例外発生するので空白文字とする
      if rhs == "" then
        rhs = " "
      end

      arg = remain         # arg の残りの部分
      arg.sub!( /\A\s*(?:\\\n)*\s*(.*)/, '\1')      # 前の空白読み飛ばす

      # \ を外す
      rhs = rhs.gsub( /\\(.)/, "\\1" )   # ここで $' が変わることに注意！
      # print "parse_plugin_arg:  #{ident} #{rhs}\n"
      @plugin_arg_list[ ident ] = rhs

      check_plugin_arg( ident, rhs )
    end
    # @plugin_arg_list.each{|i,r|  print "ident: #{i}  rhs: #{r}\n" }
  end

  #=== プラグイン引数をチェックする
  # @plugin_arg_check_proc_tab に従ってプラグイン引数をチェックすする
  # 古い用法：子クラスでオーバーライドし、引数識別子が正しいかチェックする
  #ident:: string: 引数識別子
  #rhs:: string: 右辺文字列
  def check_plugin_arg( ident, rhs )

    dbgPrint "check_plugin_arg: #{ident} #{rhs.to_str}\n"
    proc = nil
    if @plugin_arg_check_proc_tab  then
      proc = @plugin_arg_check_proc_tab[ident.to_s]
    end
    if proc == nil then
      proc = PluginArgProc[ ident.to_s ]
    end
    if proc.instance_of? Proc then
      dbgPrint "calling: #{self.class.name}.#{proc.to_s}\n"
      proc.call( self, rhs )
    else
      params = ""
      delim = ""
      @plugin_arg_check_proc_tab.each{ |j,p|
        params = "#{params}#{delim}#{j}"
        delim = ", "
      }
      cdl_error( "P1004 $1: unknown plugin argument\'s identifier\n  $2 are acceptible for RPCPlugin." , ident, params )
    end
  end

  #=== プラグインのメッセージ出力
  def print_msg( msg )
    if @b_silent == true then
      return
    end
    print msg
  end

  #=== プラグイン引数 silent
  def set_silent rhs
    if rhs == "true" || rhs == nil then
      @b_silent = true
    end
  end
end

#== 出力文字列を utf-8 から出力ファイルに convert する
# tecsgen のソースコードは utf-8 で記述されている
# これを、出力ファイルの文字コードに変換して出力する
#
# generate.rb で出力するものは message.rb で変換している
# generate.rb で出力するものは APPFile クラスを使用している
# mikan: CFile で出力したものに factory で追記できない (cdl ファイルの場合、追記できても意味がない)
class CFile
  def self.open( path, mode )
    CFile.new( path, mode )
  end

  def initialize( path, mode )
    if $b_no_kcode then 
      mode += ":" + $Ruby19_File_Encode
    end
    @file = File.open( path, mode )
  end

  def print str
    if $b_no_kcode && $KCONV_CONSOLE == Kconv::BINARY then 
      @file.print( str )
    else
      @file.print( str.kconv( $KCONV_CDL, $KCONV_TECSGEN ) )
    end
  end

  def puts str
    if $b_no_kcode && $KCONV_CONSOLE == Kconv::BINARY then 
      @file.print( str )
    else
      @file.print( str.kconv( $KCONV_CDL, $KCONV_TECSGEN ) )
    end
    @file.print( "\n" )
  end

  def printf( format, *arg )
    if $b_no_kcode && $KCONV_CONSOLE == Kconv::BINARY then 
      @file.print( sprintf( format, *arg ) )
    else
      @file.print( sprintf( format, *arg ).kconv( $KCONV_CDL, $KCONV_TECSGEN ) )
    end
  end

  def close
    @file.close
  end
end
