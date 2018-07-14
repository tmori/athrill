#!/usr/bin/env ruby
# -*- coding: utf-8 -*-
#
#  TECS Generator
#      Generator for TOPPERS Embedded Component System
#  
#   Copyright (C) 2008-2017 by TOPPERS Project
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
#   $Id: tecsgen.rb 2638 2017-05-29 14:05:52Z okuma-top $
#++

#= tecsgen  : TECS のジェネレータ
#
#Authors::    石川　拓也(HRP2Plugin)
#Authors::    原　　拓(動的結合仕様)
#Authors::    河田　智明(NotifierPlugin)
#Authors::    高田　広章(ASP3 への TECS 組込み)
#Authors::    高木　信尚(ランタイムヘッダ)
#Authors::    成瀬　有美(TECS flow 実装)
#Authors::    鵜飼　敬幸(ATK1Plugin)
#Authors::    大山　博司(ジェネレータ実装, RPCPlugin, TracePlugin, MrubyBridgePlugin)
#Authors::    山本　将也(ジェネレータ初期プロトタイプ実装)
#Authors::    小南　靖雄(テストコードの一部)
#Authors::    安積　卓也(ASP+TECS, EV3RT+TECS, mruby on TECS等実装)
#  Authors list is in i-ro-ha order.
#Version::   see version.rb
$Copyright = "Copyright(c) 2008-2017, TOPPERS project. All rights reserved."
$License   = "TOPPERS License"

# This doesn't work as expected in exerb version (Ruby 1.8.7?)
$tecsgen_base_path = File.dirname( File.expand_path __FILE__ )


###
#= Array  initializer の '{', '}' で囲まれた場合
# mikan AggregateInitializer など、クラスを変更すべきである
class Array
  #=== CDL の文字列を生成する
  def to_CDL_str
    str = '{ '
    delim = ''
    self.each{ |v|
      str += delim + v.to_CDL_str
      delim = ', '
    }
    str += ' }'
    return str
  end

  def show_tree( indent )
    indent.times{ print "  " }
    print( "Array\n" )
    self.each{ |m|
      m.show_tree( indent+1 )
    }
  end
end

#=== RUBY ライブラリをロードする
#
# -L, $RUBYLIB, システム(/usr/lib/rub..など) の順にサーチが行われる
# exerb 対応のため、上記パスにファイルが見つからない場合 require を実行してみる
#
# プラグインの場合は b_fatal = false を指定。ファイルがなくてもエラー出力後、処理続行
# b_fatal = false の場合 tecslib/core がサーチパスに追加される
#RETURN::Bool   : true=成功、 false=失敗   失敗した場合、Generator.error は呼び元で出力する
def require_tecsgen_lib( fname, b_fatal = true )
  dbgPrint( "require_lib: #{fname}\n")
  set_kcode $KCODE_TECSGEN
  begin  
    b_require = false
    b_exception = false

    # -L 、 $RUBYLIB で指定されたパスおよびシステムのパスからサーチ
    #   exerb では $LOAD_PATH は ["."] のみ入っているようだ
    ($library_path+$LOAD_PATH).each{ |path|
      [ "", "tecslib/plugin/" ].each { |lp|
        lib = File.expand_path( path ) + '/' + lp + fname
        
        if File.exist? lib then  # ファイル存否と他のエラーを区別するため存在確認をする
          begin
            require( lib )
            b_require = true
          rescue Exception => evar
            b_exception = true
            print_exception( evar )
          end
          break
        end
      }
      if b_require then
        break
      end
    }

    if b_require == false && b_exception == false then
      # exerb 対応 "." をサーチパスの最初に加える
      #   "tecslib/" は RPCPlugin.rb, TracePlugin.rb のために用意してある
      #   RPCPlugin.rb, TracePlugin.rb が tecslib 下でなければ不要になるが、このようにしておく
      ["","tecslib/plugin/"].each{ |lp|
        path = lp + fname
        begin
          require path
          return true
        rescue LoadError => e
          # p "LoadError to load #{fname}"
          # 2015.12.18 exerb 版でプラグインのロードでエラーが出るので、無視する. おそらく昔からエラーは出ていた
          # print_exception( e )
        rescue Exception => e
          b_exception = true
          # 文法エラーなどが発生
          print_exception( e )
          break
        end
      }
    end

    if b_require == false then
      # 見つからなかった
      if b_exception == false then
        STDERR << "tecsgen: Fail to load #{fname}. Check $RUBYLIB environment variable or -L option\n"
      end
      # tecsgen を構成するファイルの場合は中止する
      if b_fatal then
        STDERR << "tecsgen: Exit because of unrecoverble error\n"
        exit 1
      end
      return false
    end
    return true
  ensure
    # $KCODE を CDL の文字コードに戻しておく
    set_kcode $KCODE_CDL
  end
end

#=== 例外の表示
#evar:: Exception
def print_exception( evar )
# もしスタックトレースが出るまでい時間がかかるようならば、次をコメントアウトしてみるべし
  print "*** Begin Ruby exception message ***\n"
  puts( evar.to_s )

  if $debug then
    puts "#### stack trace ####"
    pp evar.backtrace
  end
  print "*** End Ruby exception message ***\n"
end

def dbgPrint( str )
  if $debug then
    print str
  end
end

def dbgPrintf( *param )
  if $debug then
    printf *param
  end
end

#=== エラーおよび警告のレポート
def print_report
  msg = nil

  if Generator.get_n_error != 0 then
    msg = "#{Generator.get_n_error} error"
    msg = "#{msg}s" if Generator.get_n_error >= 2
  end

  if Generator.get_n_warning != 0 then
    msg = "#{msg}  " if msg
    msg = "#{msg}#{Generator.get_n_warning} warning"
    msg = "#{msg}s" if Generator.get_n_warning >= 2
  end

  puts msg if msg
end

#=== $KCODE を設定
def set_kcode kcode
  if ! $b_no_kcode then
    $KCODE = kcode
  end
end
#----- class TECSGEN -------#
class TECSGEN

  @@current_tecsgen = nil

  def self.init( addtional_option_parser = nil )
    initialize_global_var
    analyze_option addtional_option_parser
    load_modules
    setup
        
    dbgPrint  "tecspath: #{$tecsgen_base_path}, __FILE__=#{__FILE__}\n"
    dbgPrint  "ARGV(remained): #{ARGV}, argments=#{$arguments}\n"
  end

  #----- initialize -------#
  def initialize
    @cell_list = nil
    @cell_list2 = nil
    @celltype_list = nil
    @root_namespace = nil

    #--- obsolete ---#   replaced to TOOL_INFO
    @cell_location_list = []
    @join_location_list = []
  end

  def run1
    @@current_tecsgen = self

    syntax_analisys ARGV
    semantics_analisys_1
    semantics_analisys_2

    @celltype_list = Celltype.get_celltype_list
    @cell_list = Cell.get_cell_list
    @cell_list2 = Cell.get_cell_list2

    @@current_tecsgen = nil
  end

  def run2
    @@current_tecsgen = self

    optimize_and_generate
    finalize

    @@current_tecsgen = nil
  end

  #-----  initialize_global_var -----#
  def self.initialize_global_var

    require 'optparse'
    #2.0 require 'runit/assert.rb'
    require 'kconv'
    $b_no_kcode = RUBY_VERSION >= "1.9.0" ? true : false
	       # Use Ruby 1.9 M17N code (use Ruby 1.8 code if false).
    if ! $b_no_kcode then
      require 'jcode'
    end
    require 'pp'
    # include RUNIT::Assert

    ### グローバル変数定義 ###

    # コマンドライン引数　 (Makefile.templ へ出力)
    $arguments = ""
    ARGV.each { |a| $arguments += " " + a }

    $unopt     = false     # bool:   disable optimizing
    $gen_base  = "gen"     # string: folder path to place generated files
    $gen       = $gen_base # string: folder path to place generated files
    $generate_all_template = false   # bool:   generarete template files for all celltypes (if non cell exist or system celltypes)
    $generate_no_template = false    # bool:   generarete no template file (neither celltype code nor Makefile)
    $idx_is_id = false     # bool:   all components are idx_is_id
    $unique_id = false     # bool:   assign unique id to each cell (otherwise begin from 1 for each celltype)
    $debug     = false     # bool:   tecsgen debug message
    $dryrun    = false     # bool:   dryrun mode: syntax is checked, but not generate any files
    $show_tree = false     # bool:   show parsing tree
    $verbose   = false     # bool:   verbose mode: show some messages
    $yydebug   = false     # bool:   yydebug: parser debug mode (need bnf-deb.tab.rb)
    $run_dir   = Dir.pwd   # string: tecsgen/tecscde start up directory
    $base_dir  = {  }      # string=>bool: base dir for import_path (key:base_dir, val:actually used or specified directly)
    $import_path = [ "." ] # string array : import/import_C path
    $import_path_opt = []  # [String]
    $library_path = [ $tecsgen_base_path ] # string array : path to dir where tecsgen.rb placed
    $define    = [ ]       # string array : define
    $ram_initializer = false # bool: generate ram initializer
    $region_list = {}      #string array : region path which is generated
    $generating_region = nil # Region:  Region to optimisze & generate code   # コマンドラインオプションではない
		 #           Cell#is_generate? にて参照される
    $unit_test = false     # bool:   unit test verification
    $kcode     = nil       # nil | String: Kanji code type "euc"|"sjis"|"none"|"utf8"
    $force_overwrite = false # bool:  force overwrite all files if file contents not differ
    $no_banner = false     # bool:   not print banner
    $print_version = false # bool:   print version
    $target    = "tecs"    # String: target name, ARGV[0] から再設定する（"tecs" は仮のターゲット)
    $no_default_import_path = false  # bool: no default import path
    $c_suffix  = "c"       # suffix for C progorams (for C++ source)
    $h_suffix  = "h"       # suffix for C progoram headers (for C++ source)

    if ENV['TECSGEN_DEFAULT_RAM'] then
      rom_ram_defalult = "ram"
    else
      rom_ram_defalult = "rom"
    end
    if rom_ram_defalult == "rom" then
      $rom       = true      # bool:   ROM support : generate CB separately
    else
      $rom       = false     # bool:   ROM support : generate CB separately
    end
    $b_cpp_specified = false
    if $cpp == nil then
      $cpp       = "gcc -E -DTECSGEN"
    end
    if ENV[ 'TECS_CPP' ]then
      $cpp = ENV[ 'TECS_CPP' ]
      $b_cpp_specified = true
    end
    if ENV[ 'TECSPATH' ] then
      $tecspath = ENV[ 'TECSPATH' ]
    else
      $tecspath = "#{$tecsgen_base_path}/tecs"
    end

    # # 文字コードの設定
    if $IN_EXERB then
      # KCODE_CDL, $KCONV_CDL を仮に設定する (tecs_lang.rb ですぐに再設定される)
      $KCODE_CDL = "SJIS"          # string: "EUC" | "SJIS" | "NONE" | "UTF8"
      $KCONV_CDL = Kconv::SJIS     # const: NONE には ASCII を対応させる
    else
      $KCODE_CDL = "EUC"           # string: "EUC" | "SJIS" | "NONE" | "UTF8"
      $KCONV_CDL = Kconv::EUC      # const: NONE には ASCII を対応させる
    end
    # $KCODE_TECSGEN, $KCONV_TECSGEN を仮に設定する (tecs_lang.rb ですぐに再設定される)
    $KCODE_TECSGEN = "UTF8"      # string: "EUC"  このファイルの文字コード（オプションではなく定数）
    $KCONV_TECSGEN = Kconv::UTF8 # const: 
    set_kcode( $KCODE_TECSGEN )  # このファイルの文字コードを設定

  end # initialize_global_var

  def self.analyze_option( additional_option_parser )

    ###  tecsgen コマンドオプション解析  ###
    ARGV.options {|parser|
      parser.banner = "Usage: tecsgen [options] files"
      parser.on('-D', '--define=def', 'define cpp symbol for import_C') { |define|
        $define << define
      }
      parser.on('-G', '--generate-region=path', 'generate region') { |path|
        if path =~ /^::/ then
          gen_path = path
        else
          gen_path = "::" + path
        end
        $region_list[ gen_path ] = true
      }
      parser.on('-I', '--import-path=path', 'imoprt/import_C path') { |path|
        $import_path << path
        $import_path_opt << path
      }
      parser.on('-L', '--library-path=path', 'path to dir where tecsgen.rb (obsolete, unnecessary to specify -L, those passes are gotten from tecsgen.rb') { |path|
        $library_path << path
      }
      parser.on('-R', '--RAM-initializer',   'generate RAM initializer. INITIALIZE_TECS() must be called before running any TECS code.' ){
        $ram_initializer = true
      }
      parser.on('-U', '--unoptimize', 'unoptimize') {
        $unopt = true
      }
      parser.on('-c', '--cpp=cpp_cmd', 'C pre-processor command used import_C (default: gcc -E -DTECSGEN), you can also specify by environment variable TECS_CPP' ){
        |arg|
        $cpp = arg
        $b_cpp_specified = true
      }
      parser.on('-d', '--dryrun',      'dryrun' ){
        $dryrun = true
      }
      parser.on('-f', '--force-overwrite', 'force overwrite all files') {
        $force_overwrite = true
      }
      parser.on('-g', '--gen=dir',     'generate dir') { |dir|
        $gen = $gen_base = dir
      }
      parser.on('-i', '--idx_is_id',   'set idx_is_id to all celltypes') {
        $idx_is_id = true
      }
      #  parser.on('-k', '--kcode=code',  'set kanji code: euc|sjis|none|utf8, none is default') { |code|
      parser.on('-k', '--kcode=code',  "set kanji code: euc|sjis|none|utf8") { |code|
        $kcode = code
      }
      #  old_mode は V1.0.C.22 で廃止
      #  parser.on('-o', '--old-mode',    'old mode' ){
      #    $old_mode = true
      #  }
      parser.on('-r', '--ram',     'RAM only' ){
        $rom = false
      }
      parser.on('-s', '--show-tree',   'show parsing tree' ){
        $show_tree = true
      }
      parser.on('-t', '--generator-debug', 'generator debug' ){
        $debug = true
        $verbose = true
      }
      parser.on('-u', '--unique-id', 'assign unique id for each cell' ){
        $unique_id = true
      }
      parser.on('-v', '--verbose',     'verbose mode' ){
        $verbose = true
      }
      parser.on('-y', '--yydebug',     'yydebug' ){
        $yydebug = true
      }
      parser.on( '--no-banner', 'not display banner') {
        $no_banner = true
      }
      parser.on( '--version', 'print version') {
        $print_version = true
      }
      parser.on( '--unit-test', 'unit verification (test tecsgen itself)') {
        $unit_test = true
      }
      parser.on( '--generate-all-template', 'generate all celltypes\' templates') {
        $generate_all_template = true
      }
      parser.on( '--generate-no-template',  'generate no template') {
        $generate_no_template = true
      }
      parser.on( '--no-default-import-path', 'no default import path' ){
        $no_default_import_path = true
      }
      parser.on( '--c-suffix=c', 'C program suffix (default: c)' ){
        | suffix |
        $c_suffix = suffix
      }
      parser.on( '--h-suffix=h', 'C program header suffix (default: h)' ){
        | suffix |
        $h_suffix = suffix
      }
      #  parser.on(  '--include_path_opt_format',  'cpp include path option format, default: "-I %s"' ){
      #  }
      parser.version = #{$version}
      parser.release = nil
      if additional_option_parser
        additional_option_parser.call( parser )
      end
      parser.parse!
    }

    if ARGV.empty? && ! $print_version && ! $unit_test
      ARGV.options{|parser|
        puts parser.help
        exit 1
      }
    end

  end # analyze_option

  def self.load_modules
    ### tecsgen モジュールのロード ####
    # -L でパス指定可能としたため、ここからロードを開始する

    #  tecsgen バージョンファイルのロード
    # これを実行するまで tecsgen のバージョンを表示できない
    # このファイルを誤って読み込むと、異なるバージョン名を表示してしまう
    require_tecsgen_lib 'tecslib/version.rb'
    if $tecscde_version then
      STDERR << "tecscde version #{$tecscde_version} (tecsgen version #{$version})  #{$Copyright}\n"
    elsif ! $no_banner || $print_version
      STDERR << "tecsgen  version #{$version}  #{$Copyright}\n"
    end
    if $verbose then
      STDERR << "ruby #{RUBY_VERSION} (#{RUBY_RELEASE_DATE} patchlevel #{RUBY_PATCHLEVEL}) [#{RUBY_PLATFORM}]\n"
    end
    if $print_version && ARGV.empty?
      exit
    end

    # 文字コード決定のため最初に読みこむ
    require_tecsgen_lib 'tecslib/core/tecs_lang.rb'

    unless $yydebug then
      require_tecsgen_lib 'tecslib/core/bnf.tab.rb'
    else
      require_tecsgen_lib 'tecslib/core/bnf-deb.tab.rb'
    end

    # syntaxobj.rb には Node が定義されているので、早い段階で require 
    require_tecsgen_lib 'tecslib/core/syntaxobj.rb'
    require_tecsgen_lib 'tecslib/core/pluginModule.rb'
    require_tecsgen_lib 'tecslib/core/plugin.rb'
    require_tecsgen_lib 'tecslib/core/messages.rb'
    require_tecsgen_lib 'tecslib/core/types.rb'
    require_tecsgen_lib 'tecslib/core/value.rb'
    require_tecsgen_lib 'tecslib/core/componentobj.rb'
    require_tecsgen_lib 'tecslib/core/expression.rb'
    require_tecsgen_lib 'tecslib/core/optimize.rb'
    require_tecsgen_lib 'tecslib/core/tecsgen.rb'
    require_tecsgen_lib 'tecslib/core/generate.rb'
    require_tecsgen_lib 'tecslib/core/gen_xml.rb'
    require_tecsgen_lib 'tecslib/core/tool_info.rb'
    require_tecsgen_lib 'tecslib/core/tecsinfo.rb'
    require_tecsgen_lib 'tecslib/plugin/CelltypePlugin.rb'
    require_tecsgen_lib 'tecslib/plugin/CompositePlugin.rb'
    require_tecsgen_lib 'tecslib/plugin/CellPlugin.rb'
    require_tecsgen_lib 'tecslib/plugin/SignaturePlugin.rb'
    require_tecsgen_lib 'tecslib/plugin/ThroughPlugin.rb'
    require_tecsgen_lib 'tecslib/plugin/DomainPlugin.rb'
    require_tecsgen_lib 'tecslib/plugin/MultiPlugin.rb'

    # C 言語パーサ
    require_tecsgen_lib 'tecslib/core/C_parser.tab.rb'
    require_tecsgen_lib 'tecslib/core/ctypes.rb'

    if $unit_test then
      exit 1
    end

  end # load_modules

  def self.setup
    # $import_path, $tecspath を調整
    TECSGEN.adjust_exerb_path

    # $import_path に環境変数 $TECSGEN およびその直下を追加
    if $no_default_import_path == false then
      # $TECSGEN および、その直下のディレクトリをパスに追加
      if $tecspath != "." then
        TECSGEN.add_import_path $tecspath
        dir = nil
        begin
          Dir.foreach( $tecspath ){ |f|
            if f != "." && f != ".." && File.directory?( $tecspath + '/' + f ) then
              TECSGEN.add_import_path( $tecspath + '/' + f )
            end
          }
        rescue
          # 無視
        end
      end
    end

    # デフォルト設定
    TECSGEN.set_default_config

    # $target の設定
    $target = ARGV[0]
    pos = $target.rindex( /[:\\\/]/ )
    if pos then
      $target = $target[pos+1..-1]  # ディレクトリ区切りを除いた文字列
    end
    pos = $target.rindex( /\./ )
    if pos then
      $target = $target[0..pos-1]   # 拡張子を取り除いた文字列
    end

    # gen ディレクトリの作成
    begin
      if ! File.directory?( $gen_base ) then
        Dir.mkdir( $gen_base )
      end
    rescue
      print( "Cannot mkdir #{$gen_base}. If the path has hierarchy, please create directory by manual.\n" )
      exit 1
    end
  end # setup

  #=== TECSGEN#get_celltype_list
  def get_celltype_list
    @celltype_list
  end

  #=== TECSGEN#get_cell_list
  def get_cell_list
    @cell_list
  end

  def get_root_namespace
    @root_namespace
  end
end # TECSGEN

# 複数のジェネレータインスタンスを生成することは、可能だが、以下の問題がある
#  クラス変数のリセットを確実に行う必要がある

if $TECSCDE != true then
  begin
    TECSGEN.init
    tecsgen = TECSGEN.new
    tecsgen.run1
    tecsgen.run2
  rescue => evar
    print_exception( evar )
    STDERR << "tecsgen: exit because of unrecoverble error.\n"
    STDERR << "   please retry after resolve early error.\n"
    STDERR << "   if no error has occured, please report to TOPPERS TECS WG (users@toppers.jp or com-wg@toppers.jp).\n"
    exit 1
  end
end
