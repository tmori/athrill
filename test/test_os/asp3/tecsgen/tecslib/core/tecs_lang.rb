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
#   $Id: tecs_lang.rb 2061 2014-05-31 22:15:33Z okuma-top $
#++

#== 言語に関する変数を設定
# メッセージファイルの読み込みも行う (読み込みに失敗した場合、デフォルトの文字コードに変更する)
class TECS_LANG
  # ハッシュのタグは case insensitive のため、大文字の文字列とする
  CHARSET_ALIAS = {
    "UJIS" => :eucJP,
    "UTF-8" => :utf8,
    "EUCJP" => :eucJP,   # 以下 case insensitive にするため
    "SJIS" => :sjis,
    "UTF8" => :utf8,
    "ISO8859-1" => :"iso8859-1"
  }
  LANG_ALIAS = {
    "C" => :en_US,
    "EN_US" => :en_US,   # 以下 case insensitive にするため
    "JA_JP" => :ja_JP
  }
  SUITABLE_CHARSET = {
    :ja_JP => [ :eucJP, :sjis, :utf8 ],
    :en_US => [ :"iso8859-1", :utf8, nil ]
  }

  #=== LANG のパース
  #lang::String  "ja_JP.eucJP@cjknarrow", "C" など
  #RETURN:: [ :ja_JP, :eucJP, :cjknarrow ]
  def self.parse_lang( lang )
    lang =~ /([^\.@]*)(\.([^@]*))?(@(.*))?/

    lang_terri = $1.to_sym if $1 != nil && $1 != ""
    # set_kcode_binary により、C.UTF-8 のみを特別扱いする必要がなくなった
    # if lang_terri == :C then    # LANG=C.* は、すべて 1 byte 文字コードとして扱う
    #  codeset = nil
    #  modifier = nil
    # else
      codeset = $3.to_sym if $3 != nil && $3 != ""
      modifier = $5.to_sym if $5 != nil && $5 != ""
    # end
    [ lang_terri, codeset, modifier ]
  end

  #=== lang, charset の別明解決および妥当性のチェック
  #lang::Symbol    : :en_US, :ja_JP など
  #charset::Symbol : :eucJP, :utf8 など
  #RETURN:
  #  [ lang, charset, result ]::　　result = false の場合 lang, charset は不適切
  def self.resolve_alias_and_check( lang, charset )
    if LANG_ALIAS[ lang.to_s.upcase ] then
      ln = LANG_ALIAS[ lang.to_s.upcase ]
    else
      ln = lang
    end

    if CHARSET_ALIAS[ charset.to_s.upcase ] then
      cs = CHARSET_ALIAS[ charset.to_s.upcase ]
    else
      cs = charset
    end

    if SUITABLE_CHARSET[ ln ] == nil || SUITABLE_CHARSET[ ln ].index( cs ) == nil
      res = false
    else
      res = true
    end

    # p ln, cs, res
    [ ln, cs, res ]
  end

  #=== codepage を取り出す
  #codepage は3〜5桁の整数として仮定
  def self.get_win_codepage
    cmd_pipe = IO.popen('cmd /c chcp','r')
    cmd_pipe.read =~ /([0-9]{3,5})/
    cp = $1
    cmd_pipe.close
    return cp
  end

  #=== codepage から LANG の設定文字列に変換
  def self.codepage_to_lang cp
    if cp == "932" then
      "ja_JP.sjis"
    else
      "en_US"
    end
  end

  #=== 言語、文字コードに関する変数を設定
  # 以下の順にチェックされ、一番最後に設定された値が採用される
  #   ・デフォルト
  #   ・codepage  (exerb 版で TERM 未設定または TERM=cygwin の場合のみ)
  #   ・LANG 環境変数
  #   ・TECSGEN_LANG 環境変数
  #   ・TECSGEN_FILE_LANG 環境変数 (ファイルの文字コードのみ)
  #   ・-k オプション (ファイルの文字コードのみ)
  def self.set_lang_var

    if $IN_EXERB && ( ENV[ 'TERM' ] == nil || ENV[ 'TERM' ] == "cygwin" ) then
      # exerb 版で端末 cygwin の時は codepage のみを見る
      cp = get_win_codepage
      lang = codepage_to_lang cp
      $LANG_FILE, $CHARSET_FILE, *dum = self.parse_lang( lang )
      $LANG_CONSOLE = $LANG_FILE
      $CHARSET_CONSOLE = $CHARSET_FILE

    elsif ENV[ 'LANG' ]then
      # 非 exerb 版では LANG 環境変数を見る
      # cygwin console では codepage に従って出力した方が平和なため

      $LANG_FILE, $CHARSET_FILE, *dum = self.parse_lang( ENV[ 'LANG' ] )
      $LANG_CONSOLE = $LANG_FILE
      $CHARSET_CONSOLE = $CHARSET_FILE
    end

    if ENV[ 'TECSGEN_LANG' ]then
      $LANG_FILE, $CHARSET_FILE, *dum = self.parse_lang( ENV[ 'TECSGEN_LANG' ] )
      $LANG_CONSOLE = $LANG_FILE
      $CHARSET_CONSOLE = $CHARSET_FILE
    end

    if ENV[ 'TECSGEN_FILE_LANG' ]then
      $LANG_FILE, $CHARSET_FILE, *dum = self.parse_lang( ENV[ 'TECSGEN_FILE_LANG' ] )
    end

    self.set_lang_by_option
  end

  #=== -k オプションからファイル用の言語、文字コード変数を設定
  def self.set_lang_by_option
    if $kcode == nil
      return
    end

    code = $kcode
    found = false
    res = $CODE_TYPE_ARRAY.index( code )
    if res == nil then
      print "-k: illegal kcode type #{code}. (#{$CODE_TYPE_ARRAY.join(", ")})\n"
      exit 1
    end

    case $kcode
    when "euc"
      $CHARSET_FILE = :eucJP
      $LANG_FILE = :ja_JP
    when "sjis"
      $CHARSET_FILE = :sjis
      $LANG_FILE = :ja_JP
    when "utf8"
      $CHARSET_FILE = :utf8
      $LANG_FILE = :ja_JP
    when "none"
      $CHARSET_FILE = nil
      $LANG_FILE = "en_US"
    end
  end

  #=== Kconv クラス用の変数を設定
  # 言語情報から Kconv に関する変数を設定
  def self.set_kconv_var

    # 文字コードの設定
    case $CHARSET_FILE           # string: "EUC" | "SJIS" | "NONE" | "UTF8"
    when :eucJP
      $KCODE_CDL = "EUC"
      $KCONV_CDL = Kconv::EUC
      $Ruby19_File_Encode = "ASCII-8BIT"
    when :sjis
      $KCODE_CDL = "SJIS"
      $KCONV_CDL = Kconv::SJIS
      $Ruby19_File_Encode = "Shift_JIS"
    when :utf8
      $KCODE_CDL = "UTF8"
      $KCONV_CDL = Kconv::UTF8
      $Ruby19_File_Encode = "ASCII-8BIT"
    else
      $KCODE_CDL = "BINARY"
      $KCONV_CDL = Kconv::BINARY
      $Ruby19_File_Encode = "ASCII-8BIT"
    end

    case $CHARSET_CONSOLE
    when :eucJP
      $KCODE_CONSOLE = "EUC"
      $KCONV_CONSOLE = Kconv::EUC
    when :sjis
      $KCODE_CONSOLE = "SJIS"
      $KCONV_CONSOLE = Kconv::SJIS
    when :utf8
      $KCODE_CONSOLE = "UTF8"
      $KCONV_CONSOLE = Kconv::UTF8
    else
      $KCODE_CONSOLE = "BINARY"
      $KCONV_CONSOLE = Kconv::BINARY
    end

    $KCODE_TECSGEN = "UTF8"      # string: "EUC"  このファイルの文字コード（オプションではなく定数）
    $KCONV_TECSGEN = Kconv::UTF8 # const: 
    set_kcode $KCODE_TECSGEN     # このファイルの文字コードを設定
  end

  #=== 一時的に KCODE を BINARY に変更する
  # EUC を UTF8 で読み込んだ場合に文字区切りを誤る問題の対応
  # コメントの読み飛ばしを誤る点が問題
  # ただし、SJIS の場合は、エスケープ文字の問題があるため、変更しない
  def self.set_kcode_binary

    #2.0
    if $b_no_kcode then
      return
    end

    $KCODE_BACK = $KCODE
    if $KCODE != "SJIS" then
      set_kcode "BINARY"
    end
  end

  #=== 一時的なあ KCODE の変更を元に戻す
  def self.reset_kcode
    set_kcode $KCODE_BACK
  end

  #####
  # $LANG_FILE        言語 (C は en_US に変換される)
  # $LANG_CONSOLE     言語 (C は en_US に変換される)
  # $CHARSET_FILE     ファイルの文字コード
  # $CHARSET_CONSOLE  コンソール文字コード

  # デフォルトの設定（正規化済みのこと）
  $LANG_FILE_DEFAULT  = :en_US
  $CHARSET_FILE_DEFAULT = nil
  $LANG_CONSOLE_DEFAULT = :en_US
  $CHARSET_CONSOLE_DEFAULT = nil

  $LANG_FILE = $LANG_FILE_DEFAULT
  $CHARSET_FILE = $CHARSET_FILE_DEFAULT
  $LANG_CONSOLE = $LANG_CONSOLE_DEFAULT
  $CHARSET_CONSOLE = $CHARSET_CONSOLE_DEFAULT

  # -k で指定可能なコード
  $CODE_TYPE_ARRAY = [ "euc", "sjis", "none", "utf8" ]

  # 言語を決定する
  self.set_lang_var

  # 言語、コードのチェックと正規化
  lang_file, charset_file, res =
    self.resolve_alias_and_check( $LANG_FILE, $CHARSET_FILE )
  if res == false then
    # lang_file, charset_file = lang_file_default, charset_file_default
    lang_file, charset_file = $LANG_FILE_DEFAULT, $CHARSET_FILE_DEFAULT
  end
  lang_console, charset_console, res =
    self.resolve_alias_and_check( $LANG_CONSOLE, $CHARSET_CONSOLE )
  if res == false then
    # lang_console, charset_console = lang_console_default, charset_console_default
    lang_console, charset_console = $LANG_CONSOLE_DEFAULT, $CHARSET_CONSOLE_DEFAULT
  end

  # メッセージモジュールをロード
  if require_tecsgen_lib( "tecslib/messages/messages_console_#{lang_console}.rb", false ) == false then
    require_tecsgen_lib( "tecslib/messages/messages_console_#{$LANG_CONSOLE_DEFAULT}.rb" )
    $LANG_CONSOLE, $CHARSET_CONSOLE = $LANG_CONSOLE_DEFAULT, $CHARSET_CONSOLE_DEFAULT
  else
    $LANG_CONSOLE, $CHARSET_CONSOLE = lang_console, charset_console
  end
  if require_tecsgen_lib( "tecslib/messages/messages_file_#{lang_file}.rb", false ) == false then
    require_tecsgen_lib( "tecslib/messages/messages_file_#{$LANG_FILE_DEFAULT}.rb" )
    $LANG_FILE, $CHARSET_FILE = $LANG_FILE_DEFAULT, $CHARSET_FILE_DEFAULT
  else
    $LANG_FILE, $CHARSET_FILE = lang_file, charset_file
  end

  # Kconv クラスのための変数を設定
  self.set_kconv_var 

  dbgPrint "LANG_FILE=#{$LANG_FILE}.#{$CHARSET_FILE}, LANG_CONSOLE=#{$LANG_CONSOLE}.#{$CHARSET_CONSOLE}\n"
  dbgPrint "KCODE_CDL=#{$KCODE_CDL}(#{$KCONV_CDL}) KCODE_CONSOLE=#{$KCODE_CONSOLE}(#{$KCONV_CONSOLE})\n"
  dbgPrint "Ruby19_File_Encode=#{$Ruby19_File_Encode}\n"

  #=== 単体テスト実行
  if $unit_test then
    print "unit test: set_lang_var\n"
    require "kconv"
    self.set_lang_var
    print "#{$LANG_FILE} #{$LANG_CONSOLE}\n"
    print "#{$CHARSET_FILE} #{$CHARSET_CONSOLE}\n"

    $LANG_FILE, $CHARSET_FILE, res = self.resolve_alias_and_check( $LANG_FILE, $CHARSET_FILE )
    print "#{$LANG_FILE} #{$CHARSET_FILE}, #{res}\n"
  end
end

#= Console クラス
# 文字コードを変換する
class Console
  def self.print str
    if $b_no_kcode && $KCONV_CONSOLE == Kconv::BINARY then 
      STDOUT.print str
    else
      STDOUT.print str.kconv( $KCONV_CONSOLE, $KCONV_TECSGEN )
    end
  end
  def self.puts str
    if $b_no_kcode && $KCONV_CONSOLE == Kconv::BINARY then 
      STDOUT.puts str
    else
      STDOUT.puts str.kconv( $KCONV_CONSOLE, $KCONV_TECSGEN )
    end
  end
end
