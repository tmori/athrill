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
#   $Id: pluginModule.rb 2633 2017-04-02 06:02:05Z okuma-top $
#++

#== プラグインをロードする側のモジュール
# @@loaded_plugin_list:: {Symbol=>Integer}
module PluginModule

  @@loaded_plugin_list = {}

  #=== プラグインをロードする
  # return:: PluginClass
  # V1.4.1 まで return:: true : 成功、 false : 失敗
  #
  # #{plugin_name}.rb をロードし、plugin_name クラスのオブジェクトを生成する．
  # plugin_name が MultiPlugin の場合、get_plugin により、superClass のプラグインオブジェクトをロードする．
  #
  # すでにロードされているものは、重複してロードしない
  # load 時の例外はこのメソッドの中でキャッチされて false が返される
  def load_plugin( plugin_name, superClass )

    dbgPrint "PluginModule: load_plugin: #{plugin_name}\n"
    begin
      unless @@loaded_plugin_list[ plugin_name.to_sym ] then
        @@loaded_plugin_list[ plugin_name.to_sym ] = 0
        if ( $verbose ) then
          print( "load '#{plugin_name}.rb'\n" )
        end
        # "#{plugin_name}.rb" をロード（システム用ではないので、fatal エラーにしない）
        if require_tecsgen_lib( "#{plugin_name}.rb", false ) == false then
          cdl_error( "P2001 $1.rb : fail to load plugin" , plugin_name )
          return nil
        end
      end

      plClass = Object.const_get plugin_name
      if ( plClass <= superClass ) then       # plClass inherits superClass
        return plClass
      elsif (plClass <= MultiPlugin) then     # plClass inherits MultiPlugin
        dbgPrint "pluginClass=#{plClass}\n"
        plugin_object = plClass.get_plugin superClass
        dbgPrint "pluginClass=#{plugin_object}\n"
        if plugin_object == nil then
          cdl_error( "P9999 '$1': MultiPlugin not support '$2'", plugin_name, superClass.name )
        end
        @@loaded_plugin_list[ plugin_name.to_sym ] = :MultiPlugin
        return plugin_object
      else
        cdl_error( "P2002 $1: not kind of $2" ,  plugin_name, superClass.name )
        return nil
      end
    rescue Exception => evar
      if $debug then
        p evar.class
        pp evar.backtrace
      end
      cdl_error( "P2003 $1: load failed" , plugin_name )
      return nil
    end
    # ここへは来ない
    return nil
  end

  #=== プラグインの gen_cdl_file を呼びして cdl ファイルを生成させ、解釈を行う
  def generate_and_parse plugin_object
    if plugin_object == nil     # プラグインのロードに失敗している（既にエラー）
      return
    end
    plugin_name = plugin_object.class.name.to_sym
    if @@loaded_plugin_list[ plugin_name ] == :MultiPlugin then
      p "#{plugin_name}: MultiPlugin"
      return
    elsif @@loaded_plugin_list[ plugin_name ] == nil then
      #raise "#{plugin_name} might have different name "
      ## プラグインのファイル名と、プラグインのクラス名が相違する場合
      #MultiPlugin の get_plugin で返されたケースでは nil になっている
      @@loaded_plugin_list[ plugin_name ] = 0
    end
    count = @@loaded_plugin_list[ plugin_name ]
    @@loaded_plugin_list[ plugin_name ] += 1
    tmp_file_name = "#{$gen}/tmp_#{plugin_name}_#{count}.cdl"

    begin
      tmp_file = CFile.open( tmp_file_name, "w" )
    rescue Exception => evar
      cdl_error( "P2004 $1: open error \'$2\'" , plugin_name, tmp_file_name )
      print_exception( evar )
    end
    dbgPrint "generate_and_parse: #{plugin_object.class}: gen_cdl_file\n"
    begin
      plugin_object.gen_cdl_file( tmp_file )
    rescue Exception => evar
      cdl_error( "P2005 $1: plugin error in gen_through_cell_code " , plugin_name )
      print_exception( evar )
    end
    begin
      tmp_file.close
    rescue Exception => evar
      cdl_error( "P2006 $1: close error \'$2\'" , plugin_name, tmp_file_name )
      print_exception( evar )
    end

    generator = Generator.new
    generator.set_plugin( plugin_object )
    generator.parse( [ tmp_file_name ] )
    generator.finalize
  end

  #=== プラグインが CDL の POST コードを生成
  # tmp_plugin_post_code.cdl への出力
  def self.gen_plugin_post_code file
    dbgPrint "PluginModule #{@@loaded_plugin_list}\n"
    @@loaded_plugin_list.each{ |plugin_name,count|
      if count == :MultiPlugin then
        next
      end
      dbgPrint "PluginModule: #{plugin_name}\n"
      eval_str = "#{plugin_name}.gen_post_code( file )"
      if $verbose then
        print "gen_plugin_post_code: #{eval_str}\n"
      end
      begin
        eval( eval_str )
      rescue Exception => evar
        Generator.error( "P2007 $1: fail to generate post code" , plugin_name )

        print_exception( evar )
      end
    }
  end

end
