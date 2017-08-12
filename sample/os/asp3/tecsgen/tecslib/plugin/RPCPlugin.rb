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
#   $Id: RPCPlugin.rb 2061 2014-05-31 22:15:33Z okuma-top $
#++

require_tecsgen_lib "lib/GenTransparentMarshaler.rb"
require_tecsgen_lib "lib/GenParamCopy.rb"

#= Transparent RPC プラグイン
# Transparent RPC チャンネルを生成する
# プラグイン引数は以下の RPCPluginArgProc を参照

# mikan through plugin: namespace が考慮されていない
# これを利用する場合、以下のように toppers_jsp.cdl sChannel.cdl を指定する必要がある
# tecsgen toppers_jsp.cdl sChannel.cdl your_description.cdl

# 以下を仮定（制限事項）
#  呼び元、呼び先のエンディアン、char, short, int_t, long_t, intptr_t のサイズが同じ
#  有符号、無符号でサイズが同じ

class RPCPlugin < ThroughPlugin

  include GenTransparentMarshaler
  include GenParamCopy

  # RPCPlugin 専用のオプション
  TransparentRPCPluginArgProc = RPCPluginArgProc.dup  # 複製を作って元を変更しないようにする
  TransparentRPCPluginArgProc[ "noClientSemaphore"  ] = Proc.new { |obj,rhs| obj.set_noClientSemaphore rhs }
  TransparentRPCPluginArgProc[ "semaphoreCelltype"  ] = Proc.new { |obj,rhs| obj.set_semaphoreCelltype rhs }

  #=== RPCPlugin の initialize
  #  説明は ThroughPlugin (plugin.rb) を参照
  def initialize( cell_name, plugin_arg, next_cell, next_cell_port_name, signature, celltype, caller_cell )
    super
    @b_noClientSemaphore = false
    @semaphoreCelltype = "tSemaphore"
    initialize_transparent_marshaler cell_name

    # オプション：GenTransparentMarshaler 参照
    @plugin_arg_check_proc_tab = TransparentRPCPluginArgProc
    parse_plugin_arg

    @rpc_channel_celltype_name = "tRPCPlugin_#{@TDRCelltype}_#{@channelCelltype}_#{@signature.get_global_name}"
    @rpc_channel_celltype_file_name = "#{$gen}/#{@rpc_channel_celltype_name}.cdl"

    if @signature.need_PPAllocator? then
      if @PPAllocatorSize == nil then
        cdl_error( "PPAllocatorSize must be speicified for oneway [in] array" )
        # @PPAllocatorSize = 0   # 仮に 0 としておく (cdl の構文エラーを避けるため)
      end
    end

#    @signature.each_param{ |func_decl, param_decl|
#      if func_decl.get_type.is_oneway? then
#        if ( param_decl.get_size || param_decl.get_count ) && param_decl.get_string then
#          cdl_error( "array of string not supported for oneway function in Transparent RPC" )  # mikan 文字列の配列
#        elsif param_decl.get_string == -1 then
#          cdl_error( "length unspecified string is not permited for oneway function in Transparent RPC" )  # mikan 長さ未指定文字列
#        end
#      end
#    }
  end

  #=== plugin の宣言コード (celltype の定義) 生成
  def gen_plugin_decl_code( file )

    ct_name = "#{@ct_name}_#{@channelCelltype}"

    # このセルタイプ（同じシグニチャ）は既に生成されているか？
    if @@generated_celltype[ ct_name ] == nil then
      @@generated_celltype[ ct_name ] = [ self ]
    else
      @@generated_celltype[ ct_name ] << self
      return
    end

    gen_marshaler_celltype

    if @PPAllocatorSize then
      alloc_cell = "  cell tPPAllocator PPAllocator {\n    heapSize = #{@PPAllocatorSize};\n  };\n"
      alloc_call_port_join = "    cPPAllocator = PPAllocator.ePPAllocator;\n"
    else
      alloc_cell = ""
      alloc_call_port_join = ""
    end

    if @b_noClientSemaphore == false then
      semaphore1 = <<EOT
  // Semaphore for Multi-task use ("specify noClientSemaphore" option to delete this)
  cell #{@semaphoreCelltype} Semaphore {
    count = 1;
    attribute = C_EXP( "TA_NULL" );
  };
EOT
      semaphore2 = "    cLockChannel = Semaphore.eSemaphore;\n"
    else
      semaphore1 = ""
      semaphore2 = ""
    end

    f = CFile.open( @rpc_channel_celltype_file_name, "w" )
    # 同じ内容を二度書く可能性あり (AppFile は不可)

    f.print <<EOT
import( "#{@marshaler_celltype_file_name}" );

[active]
composite #{@rpc_channel_celltype_name} {
  /* Interface */
  attr {
    PRI taskPriority;
  };
  call #{@signature.get_namespace_path} #{@call_port_name};
  entry #{@signature.get_namespace_path} eThroughEntry;
  call sTDR       cTDR;
  call sEventflag cEventflag;

  /* Implementation */
#{semaphore1}
  cell #{@marshaler_celltype_name} #{@signature.get_global_name}_marshaler{
    cTDR         => composite.cTDR;
    cEventflag   => composite.cEventflag;
#{semaphore2}  };
#{alloc_cell}  cell #{@unmarshaler_celltype_name} #{@signature.get_global_name}_unmarshaler{
    cTDR        => composite.cTDR;
    cEventflag  => composite.cEventflag;
    cServerCall => composite.#{@call_port_name};
#{alloc_call_port_join}  };
  cell tRPCDedicatedTaskMain RPCTaskMain{
    cMain = #{@signature.get_global_name}_unmarshaler.eUnmarshalAndCallFunction;
  };
  cell tTask Task {
    cBody     = RPCTaskMain.eMain;
    priority  = taskPriority;
    taskAttribute = C_EXP( "TA_ACT" );  /* mikan : marshaler starts at the beginning */
    stackSize     = 4096;
  };
  composite.eThroughEntry => #{@signature.get_global_name}_marshaler.eClientEntry;
};
EOT
    # mikan stackSize option & 最新 tecs_package 対応

    f.close
  end

  #===  through cell コードを生成
  #
  #
  def gen_through_cell_code( file )

    gen_plugin_decl_code( file )

    # セルを探す
    # path =["::",@next_cell.get_global_name]
    # cell = Namespace.find( path )
    cell = Namespace.find( @next_cell.get_namespace_path )

    file.print <<EOT
import( "#{@rpc_channel_celltype_file_name}" );

EOT

    nest = @region.gen_region_str_pre file
    indent_str = "  " * nest

    file.print <<EOT
#{indent_str}// 一方向チャンネルセル
#{indent_str}cell #{@channelCelltype} #{@channelCellName} {
#{indent_str}};

#{indent_str}// RPC チャンネルセル
EOT

    # #473 が解決された場合、composite リレーアロケータに変更すべき
    # アロケータの指定があるか？
    if cell.get_allocator_list.length > 0 then

      file.print "#{indent_str}[allocator("

      delim = ""
      cell.get_allocator_list.each do |type, eport, subsc, func, buf, alloc|

        file.print delim
        delim = ",\n#{indent_str}           "        # 最終行には出さない

        if subsc then        # 配列添数
          subsc_str = '[#{subsc}]'
        else
          subsc_str = ""
        end

        eport = "eThroughEntry" #RPCの受け口名に変更
        file.print  "#{eport}#{subsc_str}.#{func}.#{buf} = #{alloc}"
      end

      file.puts ")]"
    end

    file.print <<EOT
#{indent_str}cell #{@rpc_channel_celltype_name} #{@cell_name} {
#{indent_str}  #{@call_port_name} = #{@next_cell.get_name}.#{@next_cell_port_name};
#{indent_str}  cTDR         = #{@channelCellName}.eTDR;
#{indent_str}  cEventflag   = #{@channelCellName}.eEventflag;
#{indent_str}  taskPriority = #{@task_priority};
#{indent_str}};
EOT
    @region.gen_region_str_post file
  end


  #=== プラグイン引数 noClientSemaphore のチェック
  def set_noClientSemaphore rhs
    rhs = rhs.to_sym
    if rhs == :true then
      @b_noClientSemaphore = true
    elsif rhs == :false then
      @b_noClientSemaphore = false
    else
      cdl_error( "RPCPlugin: specify true or false for noClientSemaphore" )
    end
  end

  #=== プラグイン引数 semaphoreCelltype のチェック
  def set_semaphoreCelltype rhs
    @semaphoreCelltype = rhs.to_sym
    nsp = NamespacePath.analyze( @semaphoreCelltype.to_s )
    obj = Namespace.find( nsp )
    if ! obj.instance_of?( Celltype ) && ! obj.instance_of?( CompositeCelltype ) then
      cdl_error( "RPCPlugin: semaphoreCelltype '#{rhs}' not celltype or not defined" )
    end
  end
end


