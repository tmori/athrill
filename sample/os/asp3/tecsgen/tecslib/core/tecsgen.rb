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
#   $Id: tecsgen.rb 2638 2017-05-29 14:05:52Z okuma-top $
#++


class TECSGEN

  @@b_post_coded = false     # ポストコード生成開始後 true

  #=== import パス (-I) を末尾に追加
  # 既に登録済みであれば、追加しない
  def self.add_import_path path
    if $import_path.index( path ) == nil then
      dbgPrint "add_import_path: '#{path}'\n"
      $import_path << path
    end
  end

  #=== EXEB 版のパスの調整
  # 環境変数 TECSPATH が cygwin スタイルだと、exerb 版では扱えない
  # $import_path と $TECSPATH を調整する
  def self.adjust_exerb_path
    if $IN_EXERB then
      new_tecspath = cygpath( $tecspath, $tecspath )
      pattern = /\A#{$tecspath}/
      paths = []
      $import_path.each{ |path|
        # cygpath は '..' を簡約してしまうので、new_tecspath で置換する
        # paths << cygpath( path, path )
        paths << path.sub( pattern, new_tecspath )
      }
      $import_path = paths
      $tecspath = new_tecspath
    else
      return
    end
  end

  #=== $(TECSPATH) への置換
  #path::String   : ENV[ 'TECSPATH' ] に一致する部分があれば、 "$(TECSPATH)" に置換
  #b_global::Bool : true なら gsub で置換。false なら sub で置換
  def self.subst_tecspath( path, b_global = false )
    tp = $tecspath.dup
    tp.gsub!( /\\/, "\\\\\\\\" )
    pattern = /#{tp}/
      substr = "$(TECSPATH)"
    if b_global then
      str =  path.gsub( pattern, substr )
    else
      str = path.sub( pattern, substr )
    end
    dbgPrint "subst_tecspath #{path}, #{str}\n"
    return str
  end

  #=== path は絶対パスか?
  #path:: String   :
  # '/' または '$' で始まる場合、絶対パスと判定する
  def self.is_absolute_path?( path )
    pa = path[0..0]; pa2 = path[0..1]
    if pa == '/' || pa == '$' || pa2 =~ /[A-Za-z]:/ then
      res = true
    else
      res = false
    end
    dbgPrint "is_absolute( #{path} ) = #{res}  #{path[0]}\n"
    return res
  end

  #=== tecsgen のデフォルトを設定
  def self.set_default_config
    Makefile::set_default_config
  end

  ###
  #== Makefile.templ の出力内容を追加、変更するための操作
  module Makefile
    # 固定されている変数(add_var で変更できない)
    @@fixed_vars = { :INCLUDES => nil, :DEFINES => nil, :TARGET_BASE => nil, :BASE_DIR => nil }
    @@config_mode = false

    @@vars = { }
    @@vars_default = { }
    @@var_comments = { }

    @@objs = []
    @@ldflags = ""
    @@search_path = []
    @@pre_tecsgen_target = []
    @@post_tecsgen_target = []
    @@lines = []

    #=== OTHER_OBJS に追加する
    def self.add_obj obj
      @@objs << obj
    end
    #=== 追加する変数
    # プラグインからは、デフォルト値を変更できる
    # config により
    def self.add_var( var, val, comment = nil )
      if @@fixed_vars[ var ]
        raise "fixed var '#{var}' cannot be changed"
      end
      if @@config_mode then
        @@vars_default[ var.to_sym ] = val
        @@var_comments_default[ var.to_sym ] = comment
      else
        @@vars[ var.to_sym ] = val
        @@var_comments[ var.to_sym ] = comment
      end
    end
    #=== LDFLAGS に追加する
    def self.add_ldflag ldflag
      @@ldflags += " " + ldflag
    end
    #=== サーチパスを追加する
    # CFLAGS, vpath に追加する
    def self.add_search_path path
      @@search_path << path
    end
    #=== PRE_TECSGEN_TARGET に追加する
    # PRE_TECSGEN_TARGET に target を追加する
    def self.add_pre_tecsgen_target target
      @@pre_tecsgen_target << pre_tecsgen_target
    end
    #=== POST_TECSGEN_TARGET に追加する
    # POST_TECSGEN_TARGET に target を追加する
    def self.add_post_tecsgen_target target
      @@post_tecsgen_target << pre_tecsgen_target
    end
    #=== 追加する変数
    def self.add_line( line )
      @@lines << line.to_s + "\n"
    end

    def self.get_objs  # Array を返す
      return @@objs.uniq
    end
    def self.get_vars  # Array を返す
      if RUBY_VERSION >= '1.9' then
        return (@@vars.keys + @@vars_default.keys).sort.uniq
      else
        # V1.8 では、Symbol の sort ができないので、一旦 String に置換する
        return (@@vars.keys + @@vars_default.keys).map{|s| s.to_s }.sort.uniq.map!{|s| s.to_sym }
      end
    end
    def self.get_var_val( var )
      return @@vars[ var ] ? @@vars[ var ] : @@vars_default[ var ]
    end
    def self.get_var_comment( var )
      return @@var_comments[ var ]
    end
    def self.get_ldflags  # String を返す
      return @@ldflags
    end
    def self.get_search_path  # Array を返す
      return @@search_path.uniq
    end
    def self.get_pre_tecsgen_target  # Array を返す
      return @@pre_tecsgen_target.uniq
    end
    def self.get_post_tecsgen_target  # Array を返す
      return @@post_tecsgen_target.uniq
    end
    def self.get_lines  # 付加する行を得る
      return @@lines.uniq
    end

    #=== TECSGEN のデフォルト設定を行う
    # Makefile 
    # @@fixed_vars で定義されている変数は、変更できず、定数定義されている
    def self.set_default_config
      add_var( "TARGET", "$(TARGET_BASE).exe", "default target name"  )
      add_var( "TECSGEN", "tecsgen", "default TECS generator"  )
      add_var( "TIMESTAMP", "$(GEN_DIR)/tecsgen.timestamp", "Time Stamp"  )
      add_var( "CC", "gcc", "default C Compiler"  )
      add_var( "CFLAGS", '$(INCLUDES) $(DEFINES) -D  "Inline=static inline"', "default C Compiler options"  )
      add_var( "LD", "gcc", "default Liknker"  )
      add_var( "LDFLAGS", @@ldflags, "default Liknker Options"  )
      add_var( "SRC_DIR", "$(BASE_DIR)/src", "default source directory" )
      add_var( "_TECS_OBJ_DIR", "$(GEN_DIR)/", "default relocatable object (.o) directory" )
    end
  end

  def self.get_argv
    ARGV
  end

  #------ TECSGEN CDL analyze and generate ------#

  def syntax_analisys argv
    # ルート namespace (region) を生成
    @root_namespace = Region.new("::")

    ####  構文解析 (post コードを除く) ####
    # すべての cdl を import する
    argv.each{ |f|
      dbgPrint( "## Import: #{f}\n")
      Import.new( f, false, false )
    }

    # すべての構文解釈が完了したことの報告
    #   実際には、後からプラグインの生成する CDL のパースが行われる
    #   エラー行数の決定方法の変更のために行う
    Generator.end_all_parse
    dbgPrint( "## End all parse (except Post Code)\n")
  end # syntax_analisys

  #=== TECSGEN#semantics_analisys_1
  # semantics check. only cells here
  # other objects (signature, celltype, typedef, etc ) are checked while syntax analisys
  def semantics_analisys_1
    ####  意味解析１ (post コードを除く) ####
    dbgPrint( "## Creating reverse join \n")
    Cell.create_reverse_join

    #0 set_definition_join は2回呼び出される（1回目）
    dbgPrint( "## Checking all join\n")
    @root_namespace.set_definition_join
    # @root_namespace.set_require_join                   ### いったん見合わせ。重複エラーを見逃す
    # through プラグインで生成されたセルにも require も生成できる (set_definition_join の後ろで実施)

    ####  post コードの生成と構文解析 ####
    @@b_post_coded = true     # ポストコード生成開始後 true
    # 引数がなければ、プラグインのポストコードを出力しない
    if ARGV.length > 0 then
      dbgPrint( "## Generating Post Code\n")
      # プラグインのポストコードの出力と import
      tmp_file_name = "#{$gen}/tmp_plugin_post_code.cdl"
      file = nil
      begin
        file = CFile.open( tmp_file_name, "w" )
      rescue
        Generator.error( "G9999 fail to create #{tmp_file_name}" )
      end

      if file then
        # through プラグインのポストコード生成
        PluginModule.gen_plugin_post_code file

        begin
          file.close
        rescue
          Generator.error( "G9999 fail to close #{tmp_file_name}" )
        end
        dbgPrint( "## Import Post Code\n")
        Import.new( "#{tmp_file_name}" )
      end
    end

    ####  意味解析１ (post コード) ####
    dbgPrint( "## Creating reverse join (for post code) \n")
    Cell.create_reverse_join

    # Join の定義の設定とチェック
    #0 # 前方参照対応
    #0 set_definition_join は2回呼び出される（2回目）  post_code で生成された
    dbgPrint( "## Checking all join (for cells generated by Post Code\n")
    @root_namespace.set_definition_join
    @root_namespace.set_max_entry_port_inner_cell

    dbgPrint( "## Set require join\n")
    @root_namespace.set_require_join   # mikan post の前にも
    # ポストコードで生成されたセルの require のjoin を生成
    # mikan require で through が適用されて、ポストコードが必要となっても出力されない
  end # semantics_analisys_1

  def semantics_analisys_2
    ####  意味解析２ ####
    Cell.make_cell_list2
    dbgPrint( "## Set fixed join\n")
    Cell.create_reverse_require_join
    # create_reverse_require_join は set_detinition_join に埋め込むことができない
        # namespace に依存しない出現順で行う
        # mikan through プラグインが適用されポストコードに影響を与える場合が考慮できていない
        # mikan post code に影響のあるものであれば、早くに reverse_require_join の結合が必要
    dbgPrint( "## Setting port reference count\n")
    @root_namespace.set_port_reference_count

    dbgPrint( "## Checking all join\n")
    @root_namespace.check_join

    # mikan プラグインで生成されたコンポーネントの set_def_and_check_join

    dbgPrint( "## Checking referenced but undefined cell\n")
    @root_namespace.check_ref_but_undef
  end  #  semantics_analisys_2

  def optimize_and_generate
    #### Region link root ごとにオプティマイズおよび生成 ####
    Region.get_link_roots.each { |region|

      n_cells = region.get_n_cells

      dbgPrint "#{region.get_name} has #{n_cells} cells\n"
      if $verbose
        print "=====================================\n"
        print "=== Region.path_str: #{region.get_namespace_path.get_path_str}\n"
        print "=====================================\n"
      else
        dbgPrint "Region.path_str: #{region.get_namespace_path.get_path_str}\n"
      end

      if $region_list.length > 0 then
        if $region_list[ region.get_namespace_path.get_path_str ] then
          $region_list[ region.get_namespace_path.get_path_str ] = false
        else
          next
        end
      end

      # セルが一つもなければ生成しない
      # セルの生成がない場合
      if region.get_n_cells == 0 then
        if $region_list.length > 0 then
          Generator.warning( "W9999 $1: specified to generate but has no cell", region.get_name )
        end
        if region != @root_namespace then
          next
        end
      end

      $generating_region = region
      if Region.get_link_roots.length > 1 then
        if region.get_name == "::" then
          $gen = $gen_base
        else
          $gen = $gen_base + "/" + region.get_global_name.to_s
          begin
            if ! File.directory?( $gen ) then
              Dir.mkdir( $gen )
            end
          rescue
            print( "Cannot mkdir #{$gen}\n" )
            exit 1
          end
        end
      else
        $gen = $gen_base
      end

      dbgPrint( "## Unset optimize variables\n")
      @root_namespace.reset_optimize   # 最適化をリセットする

      if Generator.get_n_error == 0 then
        # エラーが発生していたら、設定しない
        dbgPrint( "## Set cell id\n")
        @root_namespace.set_cell_id_and_domain      # セルの ID とドメイン情報を設定（linkunit 毎に0からつける）

        # エラーが発生していたら、最適化は実施しない
        if ! $unopt then
          dbgPrint( "## Optimizing: Link Region=#{@root_namespace.get_name}\n")
          @root_namespace.optimize
        end
      end

      if $show_tree then
        # エラーが発生していても表示（エラー発生時は最適化されていないので注意）
        print "##### show_tree LinkRegion=#{region.get_name} #####\n"
        @root_namespace.show_tree(0)
        print "##### END       LinkRegion=#{region.get_name} #####\n\n"
      end

      # 構文解釈、意味解析でエラー発生していたら、コード生成をしない
      if Generator.get_n_error != 0 then
        print_report
        exit 1
      end

      #### コード生成 ####
      begin
        dbgPrint( "## Generating: Link Region=#{@root_namespace.get_name}\n")
        @root_namespace.generate
        dbgPrint( "## Generating Post: Link Region=#{@root_namespace.get_name}\n")
        @root_namespace.generate_post
      rescue
        # 通常ここへは来ない (generate, generate_post で処置される)
        Generator.error( "G9999 fail to generate" )
      end
    }
  end # optimize_and_generate

  def finalize
    dbgPrint( "## Generating XML\n")
    # Region.gen_xml @root_namespace
    ## 
    # Namespace.gen_XML  @root_namespace

    $region_list.each{ |region_path_str, val|
      if val == true then
        Generator.warning( "W9999 $1: not link root, -G ignored", region_path_str )
      end
    }

    # update する
    # APPFile で生成されたファイルは、もし変化があれば、ここで更新する
    # コード生成段階でエラーが発生すれば、更新しない
    # CFile で生成されたものは、更新されている
    if Generator.get_n_error == 0 then
      begin
        AppFile.update
      rescue => evar
        Generator.error( "G9999 Fail to update. (error occurred while renaming generated files)" )
        print_exception( evar )
      end
    end

    print_report
    if Generator.get_n_error != 0 then
      STDERR.print "error occurred while generating. some file can be corrupt in #{$gen_base}\n"
      exit 1
    end

    open( "#{$gen_base}/tecsgen.timestamp", "w" ){|io|}
  end # finalize

  def self.post_coded?
    @@b_post_coded
  end
end # class TECSGEN
