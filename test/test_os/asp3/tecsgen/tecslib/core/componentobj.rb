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
#   $Id: componentobj.rb 2663 2017-07-08 23:09:53Z okuma-top $
#++

# STAGE:
# このメンテナンス状況はよろしくない
#  B    bnf.y.rb から呼出される
#  P    parse 段階で呼出される（bnf.y.rb から直接呼出されるわけではないが、構文木生成を行う）
#  S    P の中から呼出されるが、構文木生成するわけではなく意味チェックする
#  G    コード生成（この段階で、構文木は完全である．不完全ならエラーで打ちきられている）
#                                                   factory の第一引数 "format" の後ろの引数

# mikan 以下は ruby の mix in で実現できるかもしれない
# Nestable を継承した場合、クラス変数は Nestable のものが共有される（別にしたかった）
# class Nestable
#   @@nest_stack_index = -1
#   @@nest_stack = []
#   @@current_object = nil
# 
#   def self.push
#     @@nest_stack_index += 1
#     @@nest_stack[ @nest_stack_index ] = @@current_object
#     @@current_object = nil
#   end
# 
#   def pop
#     @@current_object = @@nest_stack[ @@nest_stack_index ]
#     @nest_stack_index -= 1
#     if @@nest_stack_index < -1 then
#       raise TooManyRestore
#     end
#   end
# end
#

class Signature < NSBDNode  # < Nestable
#  @name:: Symbol
#  @global_name:: Symbol
#  @function_head_list:: NamedList : FuncHead のインスタンスが要素
#  @func_name_to_id::  {String}  :  関数名を添字とする配列で id を記憶する．id は signature の出現順番 (1から始まる)
#  @context:: string : コンテキスト名
#  @b_callback:: bool: callback : コールバック用のシグニチャ
#  @b_deviate:: bool: deviate : 逸脱（pointer level mismatch を出さない）
#  @b_checked_as_allocator_signature:: bool:  アロケータシグニチャとしてチェック済み
#  @b_empty:: Bool: 空(関数が一つもない状態)
#  @descriptor_list:: nil | { Signature => ParamDecl }  最後の ParamDecl しか記憶しないことに注意
#  @generate:: [ Symbol, String, Plugin ]  = [ PluginName, option, Plugin ] Plugin は生成後に追加される

  include PluginModule

  @@nest_stack_index = -1
  @@nest_stack = []
  @@current_object = nil

  def self.push
    @@nest_stack_index += 1
    @@nest_stack[ @@nest_stack_index ] = @@current_object
    @@current_object = nil
  end

  def self.pop
    @@current_object = @@nest_stack[ @@nest_stack_index ]
    @@nest_stack_index -= 1
    if @@nest_stack_index < -1 then
      raise "TooManyRestore"
    end
  end

  # STAGE: P
  # このメソッドは parse 中のみ呼び出される
  def self.get_current
    @@current_object
  end

  #
  # STAGE: B
  def initialize( name )
    super()
    @name = name
    Namespace.new_signature( self )
    set_namespace_path # @NamespacePath の設定
    if "#{Namespace.get_global_name}" == "" then
      @global_name = @name
    else
      @global_name = :"#{Namespace.get_global_name}_#{@name}"
    end

    @func_name_to_id = {}
    @context = nil
    @b_callback = false
    @b_deviate = false
    @b_empty = false
    @b_checked_as_allocator_signature = false
    @descriptor_list = nil
    @generate = nil
    @@current_object = self
    set_specifier_list( Generator.get_statement_specifier )
  end

  #
  # STAGE: B
  def end_of_parse( function_head_list )
    @function_head_list = function_head_list

    # id を割付ける
    id = 1
    function_head_list.get_items.each{ |f|
      @func_name_to_id[ f.get_name ] = id
      f.set_owner self
      id += 1
    }
    if id == 1 then
      @b_empty = true
    end

    set_descriptor_list

    if @generate then
      signature_plugin
    end

    @@current_object = nil

    return self
  end

  #=== Signature# signature の指定子を設定
  # STAGE: B
  #spec_list::      [ [ :CONTEXT,  String ], ... ]
  #                     s[0]        s[1]
  def set_specifier_list( spec_list )
    return if spec_list == nil  # 空ならば何もしない

    spec_list.each { |s|
      case s[0]     # statement_specifier
      when :CALLBACK
        @b_callback = true
      when :CONTEXT         # [context("non-task")] etc
        if @context then
          cdl_error( "S1001 context specifier duplicate"  )
        end
        # @context = s[1].gsub( /\A\"(.*)\"$/, "\\1" )
        @context = CDLString.remove_dquote s[1]
        case @context
        when "non-task", "task", "any"
        else
          cdl_warning( "W1001 \'$1\': unknown context type. usually specifiy task, non-task or any" , @context )
        end
      when :DEVIATE
        @b_deviate = true
      when :GENERATE
        if @generate then
          cdl_error( "S9999 generate specifier duplicate"  )
        end
        @generate = [ s[1], s[2] ] # [ PluginName, "option" ]
      else
        cdl_error( "S1002 \'$1\': unknown specifier for signature" , s[0] )
      end
    }
  end

  def get_name
    @name
  end

  def get_global_name
    @global_name
  end

  def get_function_head_array
    if @function_head_list then
      return @function_head_list.get_items
    else
      return nil
    end
  end

  def get_function_head( func_name )
    return @function_head_list.get_item( func_name.to_sym )
  end

  #=== Signature# 関数名から signature 内の id を得る
  def get_id_from_func_name func_name
    @func_name_to_id[ func_name ]
  end

  #=== Signature# context を得る
  # context 文字列を返す "task", "non-task", "any"
  # 未指定時のデフォルトとして task を返す
  def get_context
    if @context then
      return @context
    else
      return "task"
    end
  end

  #=== Signature# signaure のすべての関数のすべてのパラメータをたどる
  #block:: ブロックを引数に取る
  # ブロックは2つの引数を受け取る  Decl, ParamDecl     ( Decl: 関数ヘッダ )
  # Port クラスにも each_param がある（同じ働き）
  def each_param # ブロック引数 { |func_decl, param_decl| }
    fha = get_function_head_array                       # 呼び口または受け口のシグニチャの関数配列
    return if fha == nil                                # nil なら文法エラーで有効値が設定されなかった

    pr = Proc.new   # このメソッドのブロック引数を pr に代入
    fha.each{ |fh|  # fh: FuncHead                      # 関数配列中の各関数頭部
      fd = fh.get_declarator                            # fd: Decl  (関数頭部からDeclarotorを得る)
      if fd.is_function? then                           # fd が関数でなければ、すでにエラー
        fd.get_type.get_paramlist.get_items.each{ |par| # すべてのパラメータについて
          pr.call( fd, par )
        }
      end
    }
  end

  #=== Signature# 正当なアロケータ シグニチャかテストする
  # alloc, dealloc 関数を持つかどうか、第一引き数がそれぞれ、整数、ポインタ、第二引き数が、ポインタへのポインタ、なし
  def is_allocator?

    # 一回だけチェックする
    if @b_checked_as_allocator_signature == true then
      return true
    end
    @b_checked_as_allocator_signature = true

    fha = get_function_head_array                       # 呼び口または受け口のシグニチャの関数配列
    if fha == nil then                                  # nil なら文法エラーで有効値が設定されなかった
      return false
    end

    found_alloc = false; found_dealloc = false
    fha.each{ |fh|  # fh: FuncHead                      # 関数配列中の各関数頭部
      fd = fh.get_declarator                            # fd: Decl  (関数頭部からDeclarotorを得る)
      if fd.is_function? then                           # fd が関数でなければ、すでにエラー
        func_name = fd.get_name.to_sym 
        if func_name == :alloc then
          found_alloc = true
          params = fd.get_type.get_paramlist.get_items
          if params then
            if ! params[0].instance_of?( ParamDecl ) ||
                ! params[0].get_type.get_original_type.kind_of?( IntType ) ||
                params[0].get_direction != :IN then
              # 第一引数が int 型でない
              if ! params[0].instance_of?( ParamDecl ) ||
                  ! params[0].get_type.kind_of?( PtrType ) ||
                  ! params[0].get_type.get_type.kind_of?( PtrType ) ||
                  params[0].get_type.get_type.get_type.kind_of?( PtrType ) ||
                  params[0].get_direction != :OUT then
                # 第一引数がポインタ型でもない
                cdl_error3( @locale, "S1003 $1: \'alloc\' 1st parameter neither [in] integer type nor [out] double pointer type", @name )
              end
            elsif ! params[1].instance_of?( ParamDecl ) ||
                ! params[1].get_type.kind_of?( PtrType ) ||
                ! params[1].get_type.get_type.kind_of?( PtrType ) ||
                params[1].get_type.get_type.get_type.kind_of?( PtrType ) ||
                params[0].get_direction != :IN then
              # (第一引数が整数で) 第二引数がポインタでない
              cdl_error3( @locale, "S1004 $1: \'alloc\' 2nd parameter not [in] double pointer" , @name )
            end
          else
            cdl_error3( @locale, "S1005 $1: \'alloc\' has no parameter, unsuitable for allocator signature" , @name )
          end
        elsif func_name == :dealloc then
          found_dealloc = true
          params = fd.get_type.get_paramlist.get_items
          if params then
            if ! params[0].instance_of?( ParamDecl ) ||
                ! params[0].get_type.kind_of?( PtrType ) ||
                params[0].get_type.get_type.kind_of?( PtrType ) ||
                params[0].get_direction != :IN then
              cdl_error3( @locale, "S1006 $1: \'dealloc\' 1st parameter not [in] pointer type" , @name )
#            elsif params[1] != nil then    # 第二引き数はチェックしない
#              cdl_error3( @locale, "S1007 Error message is changed to empty" )
#                 cdl_error3( @locale, "S1007 $1: \'dealloc\' cannot has 2nd parameter" , @name )
            end
          else
            cdl_error3( @locale, "S1008 $1: \'dealloc\' has no parameter, unsuitable for allocator signature" , @name )
          end
        end
        if found_alloc && found_dealloc then
          return true
        end
      end
    }
    if ! found_alloc then
      cdl_error3( @locale, "S1009 $1: \'alloc\' function not found, unsuitable for allocator signature" , @name )
    end
    if ! found_dealloc then
      cdl_error3( @locale, "S1010 $1: \'dealloc\' function not found, unsuitable for allocator signature" , @name )
    end
    return false
  end

  #=== Signature# シグニチャプラグイン (generate 指定子)
  def signature_plugin
    plugin_name = @generate[0]
    option = @generate[1]
    apply_plugin( plugin_name, option )
  end

  #== Signature#apply_plugin
  def apply_plugin plugin_name, option
    if is_empty? then
      cdl_warning( "S9999 $1 is empty. cannot apply signature plugin. ignored" , @name )
      return
    end

    plClass = load_plugin( plugin_name, SignaturePlugin )
    return if plClass == nil
    if $verbose then
      print "new through: plugin_object = #{plClass.class.name}.new( #{@name}, #{option} )\n"
    end

    begin
      plugin_object = plClass.new( self, option )
      plugin_object.set_locale @locale
    rescue Exception => evar
      cdl_error( "S1150 $1: fail to new" , plugin_name )
      print_exception( evar )
    end
    generate_and_parse plugin_object
end

  #== Signature# 引数で参照されている Descriptor 型のリストを
  #RETURN:: Hash { Signature => ParamDecl }:  複数の ParamDecl から参照されている場合、最後のものしか返さない
  def get_descriptor_list
    @descriptor_list
  end

  #== Signature# 引数で参照されている Descriptor 型のリストを作成する
  def set_descriptor_list
    desc_list = { }
    # p "has_desc #{@name}"
    fha = get_function_head_array                       # 呼び口または受け口のシグニチャの関数配列
    if fha == nil then                                  # nil の場合、自己参照によるケースと仮定
      @descriptor_list = desc_list
      return desc_list
    end
    fha.each{ |fh|
      fd = fh.get_declarator                            # fd: Decl  (関数頭部からDeclarotorを得る)
      if fd.is_function? then                           # fd が関数でなければ、すでにエラー
        params = fd.get_type.get_paramlist.get_items
        if params then
          params.each{ |param|
            t = param.get_type.get_original_type
            while( t.kind_of? PtrType )
              t = t.get_referto
            end
            # p "has_desc #{param.get_name} #{t}"
            if t.kind_of? DescriptorType then
              desc_list[ t.get_signature ] = param
              # p self.get_name, t.get_signature.get_name
              if t.get_signature == self then
               # cdl_error( "S9999 Descriptor argument '$1' is the same signature as this parameter '$2' included", @name, param.get_name )
              end
              dir = param.get_direction
              if dir != :IN && dir != :OUT && dir != :INOUT then
                cdl_error( "S9999 Descriptor argument '$1' cannot be specified for $2 parameter", param.get_name, dir.to_s.downcase )
              end
            end
          }
        end
      end
    }
    @descriptor_list = desc_list
  end

  #=== Signature# 引数に Descriptor があるか？
  def has_descriptor?
    if get_descriptor_list == nil then
      # end_of_parse が呼び出される前に has_descriptor? が呼び出された
      # 呼び出し元は DescriptorType#initialize
      # この場合、同じシグニチャ内の引数が Descriptor 型である
      return true
    elsif get_descriptor_list.length > 0 then
      return true
    else
      return false
    end
  end

  #=== Signature# コールバックか？
  # 指定子 callback が指定されていれば true 
  def is_callback?
    @b_callback
  end

  #=== Signature# 逸脱か？
  # 指定子 deviate が指定されていれば true 
  def is_deviate?
    @b_deviate
  end

  #=== Signature# 空か？
  def is_empty?
    @b_empty
  end

  #=== Signature# Push Pop Allocator が必要か？
  # Transparent RPC の場合 oneway かつ in の配列(size_is, count_is, string のいずれかで修飾）がある
  def need_PPAllocator?( b_opaque = false )
    fha = get_function_head_array                       # 呼び口または受け口のシグニチャの関数配列
    fha.each{ |fh|
      fd = fh.get_declarator
      if fd.get_type.need_PPAllocator?( b_opaque ) then
        # p "#{fd.get_name} need_PPAllocator: true"
        @b_need_PPAllocator = true
        return true
      end
      # p "#{fd.get_name} need_PPAllocator: false"
    }
    return false
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "Signature: name: #{@name} context: #{@context} deviate : #{@b_deviate} PPAllocator: #{@b_PPAllocator} #{self}"
    (indent+1).times { print "  " }
    puts "namespace_path: #{@NamespacePath}"
    (indent+1).times { print "  " }
    puts "function head list:"
    @function_head_list.show_tree( indent + 2 )
  end

end

module CelltypePluginModule
  #=== Celltype# セルタイププラグイン (generate 指定子)
  def celltype_plugin
    plugin_name = @generate[0]
    option = @generate[1]
    @generate[2] = apply_plugin( plugin_name, option )
  end

  #=== Celltype# セルタイププラグインをこのセルタイプに適用
  def apply_plugin( plugin_name, option )

    # plClass = load_plugin( plugin_name, CelltypePlugin )
    if kind_of? Celltype then
      plugin_class = CelltypePlugin
    elsif kind_of? CompositeCelltype then
      plugin_class = CompositePlugin
    else
      raise "unknown class #{self.class.name}"
    end
    
    plClass = load_plugin( plugin_name, plugin_class )
    return if plClass == nil
    if $verbose then
      print "new celltype plugin: plugin_object = #{plClass.class.name}.new( #{@name}, #{option} )\n"
    end

    begin
      plugin_object = plClass.new( self, option )
      @generate_list << [ plugin_name, option, plugin_object ]
      plugin_object.set_locale @locale
      generate_and_parse plugin_object
    rescue Exception => evar
      cdl_error( "S1023 $1: fail to new" , plugin_name )
      print_exception( evar )
    end

    # 既に存在するセルに new_cell を適用
    @cell_list.each{ |cell|
      apply_plugin_cell plugin_object, cell
    }

    return plugin_object
  end

  def apply_plugin_cell plugin, cell
    begin
      plugin.new_cell cell
    rescue Exception => evar
      cdl_error( "S1037 $1: celltype plugin fail to new_cell" , plugin.class.name )
      print_exception( evar )
    end
  end

  def celltype_plugin_new_cell cell
    @generate_list.each{ |generate|
      celltype_plugin = generate[2]
      begin
        celltype_plugin.new_cell cell
      rescue Exception => evar
        cdl_error( "S1037 $1: celltype plugin fail to new_cell" , celltype_plugin.class.name )
        print_exception( evar )
      end
    }
  end
end #CelltypePluginModule

class Celltype < NSBDNode # < Nestable
# @name:: Symbol
# @global_name:: Symbol
# @name_list:: NamedList item: Decl (attribute, var), Port
# @port:: Port[]
# @attribute:: Decl[]
# @var:: Decl[]
# @require:: [[cp_name,Celltype|Cell,Port],...]
# @factory_list::   Factory[]
# @ct_factory_list::    Factory[] :    celltype factory
# @cell_list:: Cell[] : 定義のみ (V1.0.0.2 以降)
# @singleton:: bool
# @idx_is_id:: bool
# @idx_is_id_act:: bool: actual value
# @active:: bool
# @b_reuse:: bool :  reuse 指定されて import された(template 不要)
# @generate:: [ Symbol, String, Plugin ]  = [ PluginName, option, Plugin ] Plugin は生成後に追加される
# @generate_list:: [ [ Symbol, String, Plugin ], ... ]   generate 文で追加された generate
#
# @n_attribute_ro:: int >= 0    none specified
# @n_attribute_rw:: int >= 0    # of [rw] specified attributes (obsolete)
# @n_attribute_omit : int >= 0  # of [omit] specified attributes
# @n_var:: int >= 0
# @n_var_size_is:: int >= 0     # of [size_is] specified vars # mikan count_is
# @n_var_omit:: int >= 0        # of [omit] specified vars # mikan var の omit は有？
# @n_var_init:: int >= 0        # of vars with initializer
# @n_call_port:: int >= 0       # dynamic ports are included
# @n_call_port_array:: int >= 0  # dynamic ports are included
# @n_call_port_omitted_in_CB:: int >= 0   最適化で省略される呼び口
# @n_call_port_dynamic:: int >= 0  #
# @n_call_port_array_dynamic:: int >= 0
# @n_call_port_ref_desc:: int >= 0  #
# @n_call_port_array_ref_desc:: int >= 0
# @n_entry_port:: int >= 0
# @n_entry_port_array:: int >= 0
# @n_entry_port_inline:: int >= 0
# @n_cell_gen:: int >= 0  生成するセルの数．コード生成の頭で算出する．意味解析段階では参照不可
# @id_base:: Integer : cell の ID の最小値(最大値は @id_base + @n_cell)
#
# @b_cp_optimized:: bool : 呼び口最適化実施
# @plugin:: PluginObject      このセルタイプがプラグインにより生成された CDL から生成された場合に有効。
#                              generate の指定は @generate にプラグインが保持される
#
# @included_header:: Hash :  include されたヘッダファイル
# @domain_roots::Hash { DomainTypeName(Symbol) => [ Region ] }  ドメインタイプ名と Region の配列 (optimize.rb で設定)
#                                               ルートリージョンはドメイン名が　nil

  include PluginModule
  include CelltypePluginModule
  
  @@nest_stack_index = -1
  @@nest_stack = []
  @@current_object = nil
  @@celltype_list = []

  def self.push
    @@nest_stack_index += 1
    @@nest_stack[ @@nest_stack_index ] = @@current_object
    @@current_object = nil
  end

  def self.pop
    @@current_object = @@nest_stack[ @@nest_stack_index ]
    @@nest_stack_index -= 1
    if @@nest_stack_index < -1 then
      raise "TooManyRestore"
    end
  end

  def initialize( name )
    super()
    @@current_object = self
    @name = name
    if "#{Namespace.get_global_name}" != "" then
      @global_name = :"#{Namespace.get_global_name}_#{@name}"
    else
      @global_name = name
    end

    @name_list = NamedList.new( nil, "symbol in celltype #{name}" )
    @port = []
    @attribute = []
    @var = []
    @require = []
    @factory_list = []
    @ct_factory_list = []
    @cell_list = []
    @singleton = false
    @active = false
    @generate = nil
    @generate_list = []

    @n_attribute_ro = 0
    @n_attribute_rw = 0
    @n_attribute_omit = 0
    @n_var = 0
    @n_var_omit = 0
    @n_var_size_is = 0
    @n_var_init = 0
    @n_call_port = 0
    @n_call_port_array = 0
    @n_call_port_omitted_in_CB = 0
    @n_call_port_dynamic = 0
    @n_call_port_array_dynamic = 0
    @n_call_port_ref_desc = 0
    @n_call_port_array_ref_desc = 0
    @n_entry_port = 0
    @n_entry_port_array = 0
    @n_entry_port_array_ns = 0
    @n_entry_port_inline = 0
    @n_cell_gen = 0

    @b_cp_optimized = false

    @plugin = Generator.get_plugin
      # plugin の場合 PluginObject が返される
    # 元の Generator から呼出された Generator の中でパースおよび意味チェックされている

    # if @plugin then
    #  # plugin 生成されるセルタイプは再利用ではない   #833 不具合修正
    #  @b_reuse = false
    # else
      @b_reuse = Generator.is_reuse?
    # end

    if $idx_is_id then
      @idx_is_id = true
    else
      @idx_is_id = false
    end
    @idx_is_id_act = @idx_is_id

    Namespace.new_celltype( self )
    set_namespace_path # @NamespacePath の設定
    set_specifier_list( Generator.get_statement_specifier )

    @included_header = {}
    @domain_roots = {}
    @@celltype_list << self
  end

  def get_name
    @name
  end

  #== Celltype#ドメインルートを返す
  # @domain_roots の説明を参照
  def get_domain_roots
    @domain_roots
  end

  # Celltype# end_of_parse
  def end_of_parse
    # 属性・変数のチェック
    check_attribute

    # アロケータ呼び口を内部生成
    generate_allocator_port

    # リレーアロケータ、内部アロケータの設定
    @port.each { |p|
      p.set_allocator_instance
    }

    if @n_entry_port == 0 && @active == false && @factory_list.length == 0 &&
        ( @singleton && @ct_factory_list.length == 0 || ! @singleton )then
      cdl_warning( "W1002 $1: non-active celltype has no entry port & factory" , @name )
    end

    if @generate then
      celltype_plugin
    end

    check_dynamic_join

    @@current_object = nil
  end

  def self.new_port( port )
    @@current_object.new_port( port )
  end

  def new_port( port )
    port.set_owner self
    @port << port
    @name_list.add_item( port )
    if port.get_port_type == :CALL then
      @n_call_port += 1
      @n_call_port_array += 1 if port.get_array_size != nil
      if port.is_dynamic? then
        @n_call_port_dynamic += 1
        @n_call_port_array_dynamic += 1 if port.get_array_size != nil
      end
      if port.is_ref_desc? then
        @n_call_port_ref_desc += 1
        @n_call_port_array_ref_desc += 1 if port.get_array_size != nil
      end
    else
      @n_entry_port += 1
      @n_entry_port_array += 1 if port.get_array_size != nil
      @n_entry_port_array_ns += 1 if port.get_array_size == "[]"
      @n_entry_port_inline += 1 if port.is_inline?
    end
    port.set_celltype self
  end

  def get_port_list
    @port
  end

  def self.new_attribute( attribute )
    @@current_object.new_attribute( attribute )
  end

  #=== Celltype# new_attribute for Celltype
  #attribute:: [Decl]
  def new_attribute( attribute )
    @attribute += attribute
    attribute.each { |a|
      a.set_owner self
      @name_list.add_item( a )
      if( a.is_omit? )then
        @n_attribute_omit += 1
      elsif( a.is_rw? )then
        @n_attribute_rw += 1
      else
        @n_attribute_ro += 1
      end
      if a.get_initializer then
        # 登録後にチェックしても問題ない（attr を参照できないので、自己参照しない）
        a.get_type.check_init( @locale, a.get_identifier, a.get_initializer, :ATTRIBUTE )
      end
    }
  end

  #=== Celltype# celltype の attribute/var のチェック
  # STAGE:  S
  #
  # このメソッドは celltype のパースが完了した時点で呼出される．
  def check_attribute
    # attribute の size_is 指定が妥当かチェック
    (@attribute+@var).each{ |a|
      if a.get_size_is then
        if ! a.get_type.kind_of?( PtrType ) then
          # size_is がポインタ型以外に指定された
          cdl_error( "S1011 $1: size_is specified for non-pointer type" , a.get_identifier )
        else

          # 参照する変数が存在し、計算可能な型かチェックする
          size = a.get_size_is.eval_const( @name_list )  # C_EXP の可能性あり
          init = a.get_initializer
          if init then
            if ! init.instance_of?( Array ) then
              # 初期化子が配列ではない
              cdl_error( "S1012 $1: unsuitable initializer, need array initializer" , a.get_identifier )
            elsif size.kind_of?( Integer ) && size < init.length then
              # size_is 指定された個数よりも初期化子の配列要素が多い
              cdl_error( "S1013 $1: too many initializer, $2 for $3" , a.get_identifier, init.length, size )
            # elsif a.get_size_is.eval_const( nil ) == nil  # C_EXP の可能性あり
            end

          end
        end
      else
        if a.get_type.kind_of?( PtrType ) then
          if a.get_initializer.instance_of?( Array ) ||
              ( a.get_initializer.instance_of?( Expression ) &&
                a.get_initializer.eval_const2(@name_list).instance_of?( Array ) ) then
            # size_is 指定されていないポインタが Array で初期化されていたら、エラー
            cdl_error( "S1159 $1: non-size_is pointer cannot be initialized with array initializer" , a.get_identifier )
          end
        end
      end
    }
  end

  def get_attribute_list
    @attribute
  end

  #=== Celltype# アロケータ呼び口を生成
  #    send, receive 引数のアロケータを呼出すための呼び口を生成
  def generate_allocator_port
    @port.each { |port|
      # ポートのすべてのパラメータを辿る
      port.each_param { |port, fd, par|
        case par.get_direction                        # 引数の方向指定子 (in, out, inout, send, receive )
        when :SEND, :RECEIVE
          if par.get_allocator then
            cp_name = :"#{port.get_name}_#{fd.get_name}_#{par.get_name}"     # アロケータ呼び口の名前
            #           ポート名          関数名         パラメータ名
            # alloc_sig_path = [ par.get_allocator.get_name ]  # mikan Namespace アロケータ呼び口のシグニチャ #1
            alloc_sig_path = par.get_allocator.get_namespace_path
            array_size = port.get_array_size            # 呼び口または受け口配列のサイズ
            created_port = Port.new( cp_name, alloc_sig_path, :CALL, array_size ) # 呼び口を生成
            created_port.set_allocator_port( port, fd, par )
            if port.is_optional? then
              created_port.set_optional
            end
            if port.is_omit? then
              created_port.set_omit
            end
            new_port( created_port )                    # セルタイプに新しい呼び口を追加
          # else
          #  already error "not found or not signature" in class ParamDecl
          end
        end
      }
    }
  end

  def get_name_list
    @name_list
  end

  def self.new_var( var )
    @@current_object.new_var( var )
  end

  #=== Celltype# 新しい内部変数
  #var:: [Decl]
  def new_var( var )
    @var += var
    var.each { |i|     # i: Decl
      i.set_owner self
      if i.is_omit? then
        @n_var_omit += 1
      else
        @n_var += 1
      end
      @name_list.add_item( i )

      # size_is 指定された配列? mikan  count_is
      if i.get_size_is then
        @n_var_size_is += 1
      end

      if i.get_initializer then
        i.get_type.check_init( @locale, i.get_identifier, i.get_initializer, :VAR, @name_list )
        @n_var_init += 1
      end
    }
  end

  def get_var_list
    @var
  end

  #=== Celltype# celltype の指定子を設定
  def set_specifier_list( spec_list )
    return if spec_list == nil

    spec_list.each { |s|
      case s[0]
      when :SINGLETON
        @singleton = true
      when :IDX_IS_ID
        @idx_is_id = true
        @idx_is_id_act = true
      when :ACTIVE
        @active = true
      when :GENERATE
        if @generate then
          cdl_error( "S1014 generate specifier duplicate"  )
        end
        @generate = [ s[1], s[2] ] # [ PluginName, "option" ]
      else
        cdl_error( "S1015 $1 cannot be specified for composite" , s[0] )
      end
    }
    if @singleton then
      @idx_is_id_act = false
    end
  end

  #
  def self.new_require( ct_or_cell_nsp, ep_name, cp_name = nil )
    @@current_object.new_require( ct_or_cell_nsp, ep_name.to_sym, cp_name )
  end

  def new_require( ct_or_cell_nsp, ep_name, cp_name )
    # Require: set_owner するものがない
    obj = Namespace.find( ct_or_cell_nsp )    #1
    if obj.instance_of? Celltype then
      # Celltype 名で指定
      ct = obj
    elsif obj.instance_of? Cell then
      # Cell 名で指定
      ct = obj.get_celltype
    elsif obj == nil then
      cdl_error( "S1016 $1 not found" , ct_or_cell_nsp.get_path_str )
      return
    else
      cdl_error( "S1017 $1 : neither celltype nor cell" , ct_or_cell_nsp.get_path_str )
      return
    end

    if( ! ct.is_singleton? ) then
      # シングルトンではない
      cdl_error( "S1018 $1 : not singleton cell" , obj.get_name )
    end

    # 受け口を探す
    obj2 = ct.find( ep_name )
    if( ( ! obj2.instance_of? Port ) || obj2.get_port_type != :ENTRY ) then
      cdl_error( "S1019 \'$1\' : not entry port" , ep_name )
      return
    elsif obj2.get_array_size then
      cdl_error( "S1020 \'$1\' : required port cannot be array" , ep_name )
      return
    end

    if obj2.get_signature == nil then
      # signature が未定義：既にエラー
      return
    end

    require_call_port_prefix = :_require_call_port
    if cp_name == nil then
      # 関数名重複チェック
      @require.each{ |req|
        unless req[0].to_s =~ /^#{require_call_port_prefix}/ then
          next     # 名前ありの require は関数名重複チェックしない
        end
        port = req[2]
        if port.get_signature == obj2.get_signature then
          # 同じ signature （すべて同じ関数名を持つ）個別に出すのではなく、まとめてエラーとする
          cdl_error( "S1021 $1 : require cannot have same signature with \'$2\'" , obj2.get_name, port.get_name )
          next
        end
        port.get_signature.get_function_head_array.each{ |f|
          # mikan ここは、namedList からの検索にならないの？（効率が悪い）
          obj2.get_signature.get_function_head_array.each{ |f2|
            if( f.get_name == f2.get_name ) then
              cdl_error( "S1022 $1.$2 : \'$3\' conflict function name in $4.$5" , obj.get_name, obj2.get_name, f.get_name, req[1].get_name, req[2].get_name )
            end
          }
        }
      }
    end

    if cp_name == nil then
      b_has_name = false
      cp_name = :"#{require_call_port_prefix}_#{ct.get_name}_#{obj2.get_name}"
    else
      b_has_name = true
    end
    # require を追加
    @require << [ cp_name, obj, obj2 ]  # [ lhs:cp_name, rhs:Celltype, rhs:Port ]

    # require port を追加 (呼び口として追加する。ただし require をセットしておく)
    port = Port.new( cp_name, obj2.get_signature.get_namespace_path, :CALL )
    port.set_require( b_has_name )
    self.new_port port
  end

  def self.new_factory( factory )
    @@current_object.new_factory( factory )
  end

  def new_factory( factory )
    factory.set_owner self
    if factory.get_f_celltype then
      @ct_factory_list << factory
    else
      @factory_list << factory
    end

    factory.check_arg( self )

  end

  #=== Celltype#dynamic の適合性チェック
  def check_dynamic_join
    @port.each{ |port|
      signature = port.get_signature
      next if signature == nil   # すでにエラー
      if port.is_dynamic? then
        dbgPrint( "[DYNAMIC] checking dynamic port: #{@global_name}.#{port.get_name}\n" )
        # print( "[DYNAMIC] checking dynamic port: #{@global_name}.#{port.get_name}\n" )
        next if find_ref_desc_port signature
        next if find_descriptor_param signature, :DYNAMIC
        cdl_warning( 'W9999 $1 cannot get information for dynamic port $2', @name, port.get_name )
      elsif port.is_ref_desc? then
        dbgPrint( "[DYNAMIC] checking ref_desc port: #{@global_name}.#{port.get_name}\n" )
        # print( "[DYNAMIC] checking ref_desc port: #{@global_name}.#{port.get_name}\n" )
        next if find_dynamic_port signature
        next if find_descriptor_param signature, :REF_DESC
        cdl_warning( 'W9999 $1 cannot put information from ref_desc port $2', @name, port.get_name )
      elsif port.get_signature then
        if port.get_signature.has_descriptor? then
          port.get_signature.get_descriptor_list.each{ |signature, param|
            dbgPrint( "[DYNAMIC] checking Descriptor parameter: #{@global_name}.#{port.get_name} ... #{param.get_name}\n" )
            # print( "[DYNAMIC] checking Descriptor parameter: #{@global_name}.#{port.get_name} ... #{param.get_name}\n" )
            if port.get_port_type == :CALL then
              if param.get_direction == :IN
                next if find_ref_desc_port signature
                next if find_descriptor_param signature, :DYNAMIC
              elsif param.get_direction == :OUT
                next if find_dynamic_port signature
                next if find_descriptor_param signature, :REF_DESC
              end
            else  # :ENTRY
              if param.get_direction == :IN
                next if find_dynamic_port signature
                next if find_descriptor_param signature, :REF_DESC
              elsif param.get_direction == :OUT
                next if find_ref_desc_port signature
                next if find_descriptor_param signature, :DYNAMIC
              end
            end
            cdl_warning( 'W9999 "$1" cannot handle Descriptor "$2" infromation for port "$3"', @name, param.get_name, port.get_name )
          }
        end
      end
    }
  end

  def find_dynamic_port signature
    dbgPrint "[DYNAMIC] find_dynamic_port signature=#{signature.get_name}"
    @port.each{ |port|
      dbgPrint "[DYNAMIC] port=#{port.get_name} signature=#{port.get_signature.get_name} dynamic=#{port.is_dynamic?}"
      return port if port.is_dynamic? && port.get_signature == signature
    }
    return nil
  end
  def find_ref_desc_port signature
    if signature == nil then  # すでにエラー
      return nil
    end
    dbgPrint "[DYNAMIC] find_ref_desc_port signature=#{signature.get_name}"
    @port.each{ |port|
      dbgPrint "[DYNAMIC] port=#{port.get_name} signature=#{port.get_signature.get_name} ref_desc=#{port.is_ref_desc?}"
      return port if port.is_ref_desc? && port.get_signature == signature
    }
    return nil
  end
  #=== Celltype#ディスクリプタ型でシグニチャが一致し dyn_ref に対応づく引数を探す
  #dyn_ref::Symbol: :DYNAMIC=ディスクリプタを得る手段となる引数を探す．:REF_DESC=渡す手段となる引数を探す
  def find_descriptor_param signature, dyn_ref
    param_list = []
    @port.each{ |port|
      port.each_param{ |port, func, param|
        type = param.get_type
        while type.kind_of? PtrType
          type = type.get_type
        end
        dbgPrint( "[DYNAMIC] dyn_ref=#{dyn_ref} port_type=#{port.get_port_type} dir=#{param.get_direction} paramName=#{param.get_name} paramType=#{type.class}\n" )
        # print( "[DYNAMIC] dyn_ref=#{dyn_ref} port_type=#{port.get_port_type} dir=#{param.get_direction} paramName=#{param.get_name} paramType=#{type.class}\n" )
        if type.kind_of? DescriptorType then
          if type.get_signature == signature then
            dir = param.get_direction
            if dir == :INOUT then
              dbgPrint( "[DYNAMIC] found INOUT Descriptor parameter: #{@global_name}.#{port.get_name} ... #{param.get_name}\n" )
              # print( "[DYNAMIC] found INOUT Descriptor parameter: #{@global_name}.#{port.get_name} ... #{param.get_name}\n" )
              return param
            elsif dyn_ref == :DYNAMIC then
              if dir == :IN && port.get_port_type == :ENTRY ||
                 dir == :OUT && port.get_port_type == :CALL then
                dbgPrint( "[DYNAMIC] found INBOUND Descriptor parameter: #{@global_name}.#{port.get_name} ... #{param.get_name}\n" )
                # print( "[DYNAMIC] found INBOUND Descriptor parameter: #{@global_name}.#{port.get_name} ... #{param.get_name}\n" )
                return param
              end
            elsif dyn_ref == :REF_DESC
              if dir == :IN && port.get_port_type == :CALL ||
                 dir == :OUT && port.get_port_type == :ENTRY then
                dbgPrint( "[DYNAMIC] found OUTBOUND Descriptor parameter: #{@global_name}.#{port.get_name} ... #{param.get_name}\n" )
                # print( "[DYNAMIC] found OUTBOUND Descriptor parameter: #{@global_name}.#{port.get_name} ... #{param.get_name}\n" )
                return param
              end
            else
              raise "unknown ref_desc"
            end
          end
        end
      }
    }
    return nil
  end

  #=== Celltype# celltype に新しい cell を追加
  #cell:: Cell
  # 新しいセルをセルタイプに追加．
  # セルの構文解釈の最後でこのメソドを呼出される．
  # シングルトンセルが同じ linkunit に複数ないかチェック
  def new_cell( cell )
    dbgPrint "Celltype#new_cell( #{cell.get_name} )\n"
    # Celltype では Cell の set_owner しない
    # シングルトンで、プロトタイプ宣言でない場合、コード生成対象リージョンの場合
    if @singleton  then
      @cell_list.each{ |c|
        if c.get_region.get_link_root == cell.get_region.get_link_root then
          cdl_error( "S1024 $1: multiple cell for singleton celltype" , @name )
        end
      }
    end
    @cell_list << cell

    # プラグインにより生成されたセルタイプか ?
    if @plugin then
      @plugin.new_cell cell
    end

    # セルタイププラグインの適用
    celltype_plugin_new_cell cell
  end

  #=== Celltype# セルタイプは INIB を持つか？
  # セルタイプが INIB を持つかどうかを判定する
  # $rom == false のとき:  INIB を持たない． （すべては CB に置かれる）
  # $rom == true のとき、INIB に置かれるものが一つでも存在すれば INIB を持つ
  #   INIB に置かれるものは
  #     attribute (omit のものは除く．現仕様では rw のものはない)
  #     size_is を伴う var
  #     呼び口（ただし、最適化で不要となるものは除く）
  def has_INIB?
#    print "name=#{@name} @n_attribute_ro=#{@n_attribute_ro}  @n_var_size_is=#{@n_var_size_is} @n_call_port=#{@n_call_port} @n_call_port_omitted_in_CB=#{@n_call_port_omitted_in_CB} @n_entry_port_array_ns=#{@n_entry_port_array_ns}\n"
    return $rom &&
      (@n_attribute_ro > 0 ||
       @n_var_size_is > 0 ||
       ( @n_call_port - @n_call_port_omitted_in_CB - (@n_call_port_dynamic-@n_call_port_array_dynamic) ) > 0 ||
       $ram_initializer && @n_call_port_dynamic ||
       @n_entry_port_array_ns > 0)
#    return $rom && (@n_attribute_ro > 0 || ( @n_call_port - @n_call_port_omitted_in_CB ) > 0)
  end

  #=== Celltype# セルタイプは CB を持つか？
  # $rom == true のとき、いかのものが置かれる．それらの一つでも存在すれば CB を持つ
  #   size_is が指定されていない var
  #   rw 指定された attribute (現仕様では存在しない)
  # $rom == false のとき、いかのものが置かれる．それらの一つでも存在すれば CB を持つ
  #   attribute
  #   var
  #   呼び口（ただし、最適化で不要となるものは除く）
  def has_CB?
    if $rom then
      return @n_attribute_rw > 0 || (@n_var-@n_var_size_is) > 0 || (@n_call_port_dynamic - @n_call_port_array_dynamic) > 0
      # return @n_attribute_rw > 0 || @n_var > 0
    else
      return @n_attribute_rw > 0 || @n_attribute_ro > 0 || @n_var > 0 || (@n_call_port-@n_call_port_omitted_in_CB) > 0 || @n_entry_port_array_ns > 0
    end
  end

  #=== Celltype# SET_CB_INIB_POINTER, INITIALIZE_CB が必要か
  def need_CB_initializer?
    @n_var_init > 0 || has_CB? || ( @n_call_port_dynamic && $ram_initializer )
  end

  #=== Celltype# 逆require の結合を生成する
  def create_reverse_require_join cell
    @port.each{ |p|
      p.create_reverse_require_join cell
    }
  end

  #=== Celltype# singleton セルを得る
  #region:: Region   : singleton を探す Region
  # 距離が最も近いものを返す
  # mikan 本当は region の範囲の singleton を探す必要がある
  def get_singleton_cell region
    cell = nil
    dist = 999999999 # mikan 制限値（これは十分すぎるほどデカイが）
    # require: celltype で指定
    @cell_list.each{ |c|
      # 到達可能で最も近いセルを探す（複数の singleton があるかもしれない）
      d = region.distance( c.get_region )
      #debug
      dbgPrint "distance #{d} from #{region.get_name} to #{c.get_name} in #{c.get_region.get_name}\n"
      # print "DOMAIN: distance #{d} from #{region.get_name} to #{c.get_name} in #{c.get_region.get_name}\n"
      if d != nil then
        if d < dist then
          cell = c
          dist = d
        end
      end
    }
    return cell
  end

  def find( name )
    @name_list.get_item( name )
  end

  #=== Celltype# セルタイププラグインを得る
  def get_celltype_plugin
    if @generate then
      return @generate[2]
    end
  end

  def get_global_name
    @global_name
  end

  def is_singleton?
    @singleton
  end

  def is_active?
    @active
  end

  def idx_is_id_act?
    @idx_is_id_act
  end

  #=== Celltype# アクティブではないか
  # このメソッドでは active の他に factory (singleton においては FACTORYを含む)がなければ inactive とする
  def is_inactive?
    if @active == false && @factory_list.length == 0 &&
        ( @singleton && @ct_factory_list.length == 0 || ! @singleton )then
      return true
    end
    return false
  end

  def get_id_base
    @id_base
  end

  def get_plugin
    @plugin
  end

  def get_require
    @require
  end

  #=== Celltype# コード生成する必要があるか判定
  # セルの個数が 0 ならセルタイプコードは生成不要
  def need_generate?
    @n_cell_gen > 0
  end

  #=== Celltype# require 呼び口の結合を行う
  # STAGE: S
  # セルタイプの require 呼び口について、結合を行う
  # セルが生成されないかチェックを行う
  def set_require_join
    @require.each{ |req|
      cp_name = req[0]
      cell_or_ct = req[1]
      port = req[2]
      @cell_list.each{ |c|
        c.set_require_join( cp_name, cell_or_ct, port )
      }
    }
  end

  def get_cell_list
    @cell_list
  end

  #=== Celltype# inline 受け口しかないか？
  # 受け口が無い場合、すべての受け口が inline とはしない
  def is_all_entry_inline?
    @n_entry_port == @n_entry_port_inline && @n_entry_port > 0
  end

  #=== Celltype.get_celltype_list
  def self.get_celltype_list
    @@celltype_list
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "Celltype: name=#{@name} global_name=#{@global_name}"  
    (indent+1).times { print "  " }
    puts "active=#{@active}, singleton=#{@singleton}, idx_is_id=#{@idx_is_id} plugin=#{@plugin.class} reuse=#{@b_reuse}"
    (indent+1).times { print "  " }
    puts "namespace_path: #{@NamespacePath}"
    (indent+1).times { print "  " }
    puts "port:"
    @port.each { |i| i.show_tree( indent + 2 ) }
    (indent+1).times { print "  " }
    puts "attribute:"
    @attribute.each { |i| i.show_tree( indent + 2 ) }
    (indent+1).times { print "  " }
    puts "var:"
    @var.each { |i| i.show_tree( indent + 2 ) }
#    (indent+1).times { print "  " }
#    puts "require:"   mikan
#    @require.each { |i| i.show_tree( indent + 2 ) }
    (indent+1).times { print "  " }
    puts "factory:"
    @factory_list.each { |i| i.show_tree( indent + 2 ) }
    (indent+1).times { print "  " }
    puts "@n_attribute_ro #{@n_attribute_ro}"
    (indent+1).times { print "  " }
    puts "@n_attribute_rw #{@n_attribute_rw}"
# @n_attribute_omit : int >= 0  # of [omit] specified cells
# @n_var:: int >= 0
# @n_var_size_is:: int >= 0     # of [size_is] specified cells # mikan count_is
# @n_var_omit:: int >= 0        # of [omit] specified  cells # mikan var の omit は有？
# @n_call_port:: int >= 0
# @n_call_port_array:: int >= 0
# @n_call_port_omitted_in_CB:: int >= 0   最適化で省略される呼び口
# @n_entry_port:: int >= 0
# @n_entry_port_array:: int >= 0
    (indent+1).times { print "  " }
    puts "@n_entry_port_inline #{@n_entry_port_inline}"
# @n_cell:: int >= 0  コード生成の頭で算出する．意味解析段階では参照不可
# @id_base:: Integer : cell の ID の最小値(最大値は @id_base + @n_cell)

  end
end

class Cell < NSBDNode # < Nestable
# @name:: Symbol : composite celltype の内側のインスタンスでは外のセル
# @global_name:: Symbol : C で使える名前（namespace を含む）
# @local_name:: str : cell celltype name { ... } の name
# @celltype:: Celltype | CompositeCelltype
# @join_list:: NamedList
# @reverse_join_list:: NamedList
# @b_defined:: definition flag (false if only prototype )
# @b_prototype:: bool:  prototype specified in current parsing cell. (@b_defined is used to determine whether definition done)
# @b_duplicate:: bool:  definition duplicate
# @b_checked::   bool:  set_definition_join が済んでいる場合 true
# @require_joined_list:: {cp_name=>true}:  set_require_join が済んでいる呼び口は true
# @f_ref:: refercenced from others
# @entry_array_max_subscript:: { @port=>Integer } : 受け口配列の配列添数の最大値（添数無し受け口配列対応）
# @plugin::     Plugin: avialble if cell is generated by plugin generated cdl code.
# @referenced_port_list:: { Port => Integer } : 受け口の参照数
#                                               すべての意味解析(through, composite展開)が終わった後に設定する
#                                               逆require ポートに対して複数の結合がないかチェックする
# @generate:: [ Symbol, String, Plugin ]  = [ PluginName, option, Plugin ] Plugin は生成後に追加される
#
# composite のためインスタンス変数
# @in_composite:: bool : true if in composite celltype
# @compositecelltypejoin_list:: NamedList : item= CompositeCelltypeJoin ( if @in_composite )
# @f_cloned:: bool : true if cloned (instantiate of composite consist cell)
# @my_clone:: Cell : Composite cell で in_composite = true の場合のみ有効（直前の clone を一時記憶）
# @cell_list:: Cell[local_name] : Composite cell で clone した後のリスト cell_list
# @cell_list2:: [ Cell ] : Composite cell で clone した後のリスト cell_list
#                          @cell_list2 は composite 内での出現順  
#
# region のためのインスタンス変数
# @region:: Region (属するregion)
#
# allocator のためのインスタンス変数
# @alloc_list::  [ [ NORMAL_ALLOC, ep_name, func_name, param_name, expr ], ... ]
#   受け口側のアロケータへの結合を記憶。以下のメソッドで追加される
#      set_specifier … cell 定義時にアロケータ指定されている場合
#      create_relay_allocator_join … リレーアロケータの場合
#
# ID のためのインスタンス変数（optimize.rb にて設定）
# @id:: Integer : コード生成直前に設定  (プロトタイプ宣言の場合は -1 のまま放置)
# @id_specified::Integer : 指定された id
# @restrict_list::{ entry_name => { func_name, [ region_name, ... ] } }

=begin
# Cell クラスは、以下のものを扱う
# 1)普通のセル
# 2)composite セルタイプのセル
# 3)composite セルタイプの内側のセル (@in_composite)   # composite の内側の composite セルタイプのセルもある
#
# 2) は CellOfComposite クラスとして分けたほうがよいかもしれない
#    expand (composite セルの展開) は CellOfComposite にのみ必要なメソッドである
#    get_real_cell, get_real_port など @celltype.instance_of?( CompositeCelltype ) の判定がなくすっきりする
#    ただ、分離しても、メンテナンスすべき範囲が切り離されてしまい、忘れやすくなる問題とのトレードオフかも
#
# 3) は CellInCompoiste クラスとして分けたほうがよいかもしれない
#    @in_composite で判定している処理を切り離せる (上記 2) よりも分離は容易ではない)
#    clone_for_composite は CellInCompoiste にのみ必要なメソッドである
#    ただし、clone 後に Cell, CellOfComposite に変化する必要があるので、clone ではなく new する実装に変更する必要がある
#
=end

  include PluginModule

  @@nest_stack_index = -1
  @@nest_stack = []
  @@current_object = nil

  # 定義されたすべてのセル（出現順. namespace に影響されない）
  @@cell_list = []     # composite の内部のセルを含まない
  @@cell_list2 = []    # composite の内部のセルを含む (元のセルを含む)
                       # 意味解析後 make_cell_list2 にて設定される

  def self.push
    @@nest_stack_index += 1
    @@nest_stack[ @@nest_stack_index ] = @@current_object
    @@current_object = nil
  end

  def self.pop
    @@current_object = @@nest_stack[ @@nest_stack_index ]
    @@nest_stack_index -= 1
    if @@nest_stack_index < -1 then
      raise "TooManyRestore"
    end
  end

  def initialize( ct_path, in_composite = false )
    super()
    @region = Region.get_current

    # celltype のplugin/存在をチェック
    object = Namespace.find( ct_path )    #1
    if object == nil then
      # mikan celltype の名前が不完全 "::ct1ct2" になる
      cdl_error( "S1027 \'$1\' celltype not found" , ct_path.get_path_str )
    elsif ! object.instance_of?( Celltype ) && ! object.instance_of?( CompositeCelltype ) then
      # mikan celltype の名前が不完全
      cdl_error( "S1028 \'$1\' not celltype" , ct_path.get_path_str )
    else
      @celltype = object
    end

    @in_composite = in_composite
    if @in_composite then
      @compositecelltypejoin_list = NamedList.new( nil, "in cell '#{@name}'" )
      @plugin = nil
    else
      @compositecelltypejoin_list = nil
      @plugin = Generator.get_plugin
    end

    @@current_object = self
    @b_defined = false
    @b_prototype = false
    @f_ref     = false
    @f_cloned  = false
    @alloc_list = []
    @id = -1
    @id_specified = nil
    @b_duplicate = false
    @b_checked = false
    @require_joined_list = {}
    @entry_array_max_subscript = {}
    @referenced_port_list = {}
    @restrict_list = {}

    @cell_list = {}
    @cell_list2 = []
  end

  def self.set_name( name )
    @@current_object.set_name( name )
  end

  def set_name( name )

    @name = name
    @local_name = name
    if "#{Namespace.get_global_name}" != "" then
      @global_name = :"#{Namespace.get_global_name}_#{name}"
    else
      @global_name = name
    end

    # この時点ではプロトタイプか、定義か分らないが、自己参照のために登録
    # set_f_def で再度登録しなおす
    # Celltype への登録は、end_of_parse で行う
    if @in_composite then
      cell_prev = CompositeCelltype.find( name )
      if cell_prev == nil then
        CompositeCelltype.new_cell_in_composite( self )
      end
    else
      # cell_prev = Namespace.find( [ name ] )   # 親まで捜しにいく
      cell_prev = Namespace.get_current.find( name )
      if cell_prev == nil then
        Namespace.new_cell( self )
        set_namespace_path # @NamespacePath の設定
      end
    end

    if cell_prev then
      if ! cell_prev.instance_of?( Cell ) then
        cdl_error( "S1029 $1 mismatch with previous one" , name )
        # celltype が一致しているか ?
      elsif get_celltype != cell_prev.get_celltype then
        cdl_error( "S1030 $1: celltype mismatch with previous one" , name )
      else
        # region が一致しているか?
        if ! cell_prev.get_region.equal? get_region then
          cdl_error( "S1031 $1 region \'$2\' mismatch  with previous one \'$3\'" , name, @region.get_name, cell_prev.get_region.get_name )
        end

        @@current_object = cell_prev
        # この時点では、まだプロトタイプ宣言か定義か分らない
        # 以前が定義であって、今回も定義の場合、重複エラーである
      end
    end

    @join_list = NamedList.new( nil, "in cell '#{@name}'" )
    @reverse_join_list = nil

    # debug
    dbgPrint "Cell new_cell: #{@global_name} #{@in_composite} #{self}\n"

    # 内部アロケータを @alloc_list に追加
    if @celltype.instance_of? CompositeCelltype then
      @celltype.get_internal_allocator_list.each{ |cell, cp_internal_name, port_name, fd_name, par_name, ext_alloc_ent|
        nsp = NamespacePath.new( @name, false )
        rhs = Expression.new( [:OP_DOT, [:IDENTIFIER, nsp], Token.new( ext_alloc_ent.to_s.to_sym, nil, nil, nil ) ] )  #1 構文解析段階なので locale 不要

        @alloc_list << [:NORMAL_ALLOC,port_name,nil,fd_name,par_name,rhs]
# print "add alloc_list: #{port_name}.#{fd_name}.#{par_name}=#{rhs.to_s}\n"
      }
    end
  end

  #=== Cell# cell の定義
  # 本体(join)の定義の直前に呼び出される
  def self.new_def
    @@current_object.new_def
  end

  #=== Cell# cell の定義
  # 本体(join)の定義の直前に呼び出される
  # このメソッドは cell tCelltype Cell { };  '{', '}' の部分がある場合に呼出される
  def new_def
    set_specifier_list( Generator.get_statement_specifier )

    # prototype 指定子ないか
    if ! @b_prototype then
      # 二重定義のチェック
      if @b_defined == true then
        cdl_error( "S1032 $1: duplicate cell" , @name )
        dbgPrint "previous: #{@prev_locale[0]}: line #{@prev_locale[1]} '#{@name}' defined here\n"

        # セルの重複定義エラーの処置
        # 前の定義は捨てる
        @join_list = NamedList.new( nil, "in cell '#{@name}'" )
      end

      @b_defined = true
      @prev_locale = @locale
    end
  end

  def self.new_join( join, b_regular = false )
    @@current_object.new_join( join, b_regular )
  end

  #===  Cell# 新しい結合をチェック
  # STAGE:   P
  #
  #join::  Join : 新しい結合
  #b_regular:: bool : 通常の場所 (bnf.y.rb)からならば true, それ以外(allocator, require) では false
  def new_join( join, b_regular = false )
    join.set_owner self

    # composite の新文法対応．
    # composite の中のセルで、attribute の定義である場合
    # cell 内の attr_ext = composite.attr; 構文を処理
    if @in_composite then
      if @celltype then
        if @celltype.find(join.get_name).instance_of?( Decl ) then # mikan a::b で指定されていたものがエラーにならないかも
          rhs = join.get_rhs
          if rhs.instance_of? Expression then
            ele = rhs.get_elements
            if( ele[0]==:IDENTIFIER )then    #  attr = attr_ext （右辺単項）か？ #1
              if( CompositeCelltype.has_attribute?(ele[1].get_name ) )then    # mikan a::b.ePort がエラーにならないかも
                ident = ele[1].get_name   # 右辺は attribute．
              else
                # 右辺は attribute にないのであれば、定数のはず
                # 定数は下へ渡す (cell の join にする)
                ident = nil
              end
            else
              if join.get_rhs.eval_const2(nil) == nil then   # 定数式ではないか？
                # 右辺が、単一のシンボルでない場合、現状は扱えない
                cdl_error( "S1033 rhs expression is not supported. Only attribute is permitted on current version."  )
                return
              else
                # 定数は下へ渡す (cell の join にする)
                ident = nil
              end
            end

            if ident then
              # attr = attr; のような参照はエラー (a = composite.a とする必要がある)
              if @celltype.find( ident ) then
                cdl_error( "S1034 $1 : cannot refer to $2\'s attribute here. Use \'composite.$3\' to refer to composite celltype\'s" , ident, @celltype.get_name, ident )
              end
            end
          elsif rhs.instance_of? Array then
            if rhs[0] == :COMPOSITE then   # 右辺は composite.attr の形式
              ident = rhs[1].to_sym
            else
              ident = nil    # 右辺は { 10, -10 } の形式
            end
          else
            ident = nil      # 右辺は C_EXP の形式
          end

          # ident が見つかった（右辺は単一の ident）
          if ident then
            # composite の旧文法の構文処理へ渡す．セル外の attr_ext = Cell.attr; 構文の処理に渡す
            #                        export_name, internal_cell_name, internal_cell_elem_name
            decl = CompositeCelltype.new_join( ident, @name, join.get_name, :ATTRIBUTE )  # mikan a::b.ePort がエラーにならないかも
            if ! decl.instance_of? Decl then
              return
            end
            ini = decl.get_initializer
            if ini == nil then
              return
            end
            # 以下の旧文法実装に渡す．
            # 旧文法では cell に初期値を与えることで、composite で属性の初期値を指定することができた
            # attribute で指定された初期値を cell の属性として処理させる
            join.change_rhs( ini )
          else
            # ident がない．定数式
          end
        else
          # celltype の属性として、この join の名前がない
          # 以下の join.set_definition の中でエラーとなる
        end
      else
        return    # celltype がない．すでにエラー
      end
    elsif join.get_rhs.instance_of? Array then
      rhs = join.get_rhs
      if rhs[0] == :COMPOSITE then
        # composite の中でないのに attr = composite.attr が使われた
        cdl_error( "S1035 composite : cannot specify out of composite celltype definition"  )
        return
      end
    end

    # 以下 composite 文法変更前からある処理

    # 既に左辺が同じ名前の初期化が存在するか？
    j = @join_list.get_item( join.get_name )   # mikan NamespacePath がパスを持っている
    if j.instance_of? Join then    # mikan ここでは j が Join or Nil 以外は、ないはず

      # debug
      dbgPrint "add_array_member: #{@name} port: #{j.get_port_name} rhs: #{j.get_rhs}, #{join.get_port_name} #{join.get_rhs}\n"
      # 呼び口配列（であると仮定して）要素を追加
      j.add_array_member join

    else
      dbgPrint "new_join: cell=#{@name} add_item=#{join.get_name}\n"
      # join
      @join_list.add_item( join )
    end

    # if get_owner then   # error S1030 発生時 get_owner が見つからなくて例外になる
    #   dbgPrint "Cell#new_join: #{get_owner.get_name}.#{@name}\n"
    # else
    #   dbgPrint "Cell#new_join: \"owner not fund\".#{@name}\n"
    # end
    if ! @in_composite then
     if join.get_cell
       dbgPrint "new_join: #{@name} #{@region.get_name} => #{join.get_cell.get_name} #{join.get_cell.get_region.get_path_string}\n"
     end
#     p "region: generate? #{@region.is_generate?}"
    end

  end

  #=== Cell.新しい逆結合
  def self.new_reverse_join( reverse_join )
    @@current_object.new_reverse_join( reverse_join )
  end

  #=== Cell#新しい逆結合
  def new_reverse_join( reverse_join )
    dbgPrint( "new_reverse_join name=#{reverse_join.get_name}\n")
    b_cb = false
    if @celltype then
      ep_name = reverse_join.get_name
      port = @celltype.find ep_name
      if port && port.get_signature
        if port.get_signature.is_callback? then
          b_cb = true
        end
      end
    end
    if ! @b_prototype && ! b_cb then
      cdl_error( "S9999 '$1': reverse join can be used in prototype cell, or with callback signature", @name )
    end
    if @reverse_join_list == nil then
#      @reverse_join_list = NamedList.new( reverse_join, "in cell '#{@name}'" )
      @reverse_join_list = [ reverse_join ]
    else
#      @reverse_join_list.add_item( reverse_join )
      @reverse_join_list << reverse_join
    end
  end

  #=== Cell#逆結合から結合を生成
  # STAGE: S
  def create_reverse_join
    if @b_checked then
      return
    end

    if @reverse_join_list then
#      @reverse_join_list.get_items.each{ |rj|
      @reverse_join_list.each{ |rj|
        # 逆結合の情報を得る
        ep_name = rj.get_name
        ep_subscript, cp_cell_nsp, cp_name, cp_subscript = rj.get_rhs_cell_and_port

        # 呼び口側のセルと、そのセルタイプ
        if ! @in_composite then
          cell = Namespace.find cp_cell_nsp
        else
          cell = CompositeCelltype.find cp_cell_nsp.to_s.to_sym
        end

        if ! cell.instance_of? Cell then
          cdl_error( "S9999 '$1': not cell for reverse join", cp_cell_nsp.get_path_str )
          next
        end
        ct = cell.get_celltype
        if ct == nil then
          next
        end

        if ! @in_composite then
          ep_cell_nsp = get_namespace_path
          ep_cell_nsp_str = ep_cell_nsp.get_path_str
        else
          ep_cell_nsp = NamespacePath.new @name, false
          ep_cell_nsp_str = @name
        end
        ep_subscript_val = ep_subscript ? ep_subscript.eval_const( nil ) : nil
        rhs = Expression.create_cell_join_expression( ep_cell_nsp, ep_subscript_val, ep_name, rj.get_locale )
        join = Join.new( cp_name, cp_subscript, rhs, rj.get_locale )
        cell.new_join( join )
        # join.set_definition( ct.find(join.get_name) )
        if cp_subscript then
          ss_str = "[#{cp_subscript}]"
        else
          ss_str = ""
        end
        dbgPrint "create_reverse_join: #{cell.get_name}.#{cp_name}#{ss_str} => #{ep_cell_nsp_str}.#{ep_name}\n"
      }
    end
  end

  def self.external_join( internal_cell_elem_name, export_name, b_composite )
    @@current_object.external_join( internal_cell_elem_name, export_name, b_composite )
  end

  #=== Cell# cell 内に記述する呼び口の外部結合
  # internal_cell_elem_name:: string : 呼び口名
  # export_name:: string: composite の外部に公開する呼び口名
  #  呼び口を外部結合する．
  #  このメソッドは、composite の中の cell でしか呼ばれない．
  def external_join( internal_cell_elem_name, export_name, b_composite )

    # cCall => composite.cCall; ではないか？
    if( b_composite == false )then
      # cCall => cCall; のような場合
      if @celltype.find( export_name ) then
        cdl_error( "S1036 $1 : cannot refer to $2\'s here. Use \'composite.$3\' to refer to composite celltype\'s" , export_name, @celltype.get_name, export_name )
      end
    end
    # composite の旧文法における、cell 外の cCall = Cell.cCall; の構文処理に渡す
    CompositeCelltype.new_join( export_name, @name,  internal_cell_elem_name, :CALL )
  end

  def self.end_of_parse f_def
    cell = @@current_object
    cell.end_of_parse f_def
    @@current_object = nil
    return cell
  end

  def end_of_parse f_def
    if @b_prototype then  # prototype 指定子あったか?
      f_def = false       # プロトタイプ宣言とする
      @b_prototype = false
    end
    if f_def == false then
      # cell tCelltype Cell; の形式の場合
      # f_def == true の場合 new_def で、呼出される
      set_specifier_list( Generator.get_statement_specifier )
    end
    set_f_def f_def

    if @generate then
      cell_plugin
    end
  end

  #=== Cell# プロトタイプ宣言(false)か定義(true)かを設定
  #    このメソッドは構文解釈の最後に呼出される
  #f_def::     bool     false if prototype, true if definition
  def set_f_def f_def
    if ! f_def then
      return
    end

    if ! @in_composite then
      # if @celltype.instance_of? Celltype then
      if @celltype then  # composite でも呼びだす, エラー時 nil
        @celltype.new_cell self
      end
      @@cell_list << self
    end
  end

  def set_f_ref
    dbgPrint "set_f_ref: #{@global_name}\n"
    @f_ref = true

    # composite の内部セルを参照されたことにする
    # 今のところ問題ないが、未参照であるべきものまで参照されたことになる
    if @cell_list then
      @cell_list.each{ |cn,cell|
        cell.set_f_ref
      }
    end
  end

  #=== Cell# cell の指定子を設定
  # STAGE:  B
  #
  #    bnf.y.rb の statement_specifiler_list
  #spec_list::      [ :ALLOCATOR, [ [ :NORMAL_ALLOC, ep_name, subscript, func_name, param_name, expr ], ... ] ]
  #                     s[0]      s[1]   a[0]        a[1]       a[2]        a[3]     a[4]       a[5]
  #    セルに指定されたアロケータ指定子
  #    a[1] の subscript はこのメソッドの中で Expression から Integer に評価される
  #    受け口側に生成されるアロケータ呼び口の結合を内部生成する
  #    呼び口側は Port の create_allocator_join にて生成
  #    リレーアロケータの場合 create_relay_allocator_join にて生成す
  def set_specifier_list( spec_list )
    return if spec_list == nil  # 空ならば何もしない

    dbgPrint( "set_spec_list: #{@name}\n" )
    b_generate = false   # generate が指定された

    spec_list.each{ |s|
      case s[0]             # statement_specifier
      when :ALLOCATOR       # [allocator(ePort.func.param=allocCell.eA,ePort.func2.param=allocCell.eA)]
        s[1].each { |a|     # alloc_list : allocator の内部の ',' で区切られた部分の配列
          cp_name = :"#{a[0+1]}_#{a[2+1]}_#{a[3+1]}"    # アロケータ呼び口の名前：'=' の左辺を '.' に変えて '_' で連結
          # p "#{a[0]} #{a[0+1]} #{a[2+1]} #{a[3+1]} #{cp_name}"
          if a[1+1] then
            subscript = a[1+1].eval_const nil
            a[1+1] = subscript
          else
            subscript = nil
          end
          # アロケータ呼び口の結合を生成
          join = Join.new( cp_name, subscript, a[4+1] )   # 構文解析段階なので locale 不要
          dbgPrint( "new allocator join #{cp_name} #{subscript} #{a[4+1]}\n" )
          Cell.new_join( join )
          @alloc_list << a 
        }
      when :ID          # [id(0)]
        if ! s[1].instance_of? Expression then
          cdl_error( "S1160 $1 must be constant for id", s[1].to_s )
        else
          id = s[1].eval_const nil
          if id == nil || Integer(id) != id then
            cdl_error( "S1161 $1 must be constant for id", s[1].to_s )
          elsif id == 0 then
            cdl_error( "S1162 $1: id cannot be 0", s[1].to_s )
          else
            @id_specified = id
          end
        end
      when :GENERATE      # [generate(CellPlugin,"option")]
        if @generate then
          cdl_error( "S1163 generate specifier duplicate"  )
        end
        @generate = [ s[1], s[2] ] # [ PluginName, "option" ]
        b_generate = true
      when :PROTOTYPE     # [prototype]
        @b_prototype = true
      when :RESTRICT      # [restrict]
        s[1].each{ |re|
          add_restrict re[0], re[1], re[2]
        }
      else
          cdl_error( "S1039 \'$1\': unknown specifier for cell" , s[0] )
      end
    }
    if @b_prototype then
      if b_generate then
         cdl_error( "S9999 '$1': generate and prototype specified simultaneously" , @name )
      end
      if @b_defined then
         cdl_error( "S9999 '$1': prototype specified after definition" , @name )
      end
    end
  end

  def get_allocator_list

    # 意味チェック(set_definition)されていない？
    # relay アロケータの場合、セルの意味チェックが行われていないと、@alloc_list が完成しない
    if @b_checked == false then
      set_definition_join
    end
    @alloc_list
  end

  def get_specified_id
    @id_specified
  end

  #=== id 指定子の値を設定
  # このメソッドは、プラグインで cell の生成順序を制御したい場合のために設けた
  # 通常の id 指定子では使っていない
  def set_specified_id id
    if Integer( id ) != id || id <= 0 then
      cdl_error( "S1164 '$1' set_specified_id: id not positive integer '$2'", @name, id )
    elsif @id_specified then
      cdl_error( "S1165 '$1' set_specified_id: id duplicate", @name )
    else
      @id_specified = id
    end
  end

  #=== Cell# セルプラグイン (generate 指定子)
  def cell_plugin
    plugin_name = @generate[0]
    option = @generate[1]
    @generate[2] = apply_plugin plugin_name, option
  end

  def apply_plugin plugin_name, option
    if ! @b_defined then
      cdl_error( "S9999 plugin cannot apply to prototype cell '$1'", @name )
    end

    plClass = load_plugin( plugin_name, CellPlugin )
    # return if plClass == nil # 従来と仕様が変わるので、継続する
    if $verbose then
      print "new cell plugin: plugin_object = #{plClass.class.name}.new( #{@name}, #{option} )\n"
    end

    begin
      plugin_object = plClass.new( self, option )
      plugin_object.set_locale @locale
      generate_and_parse plugin_object
    rescue Exception => evar
      cdl_error( "S1166 $1: fail to new", plugin_name )
      print_exception( evar )
    end
    return  plugin_object
  end

  def add_compositecelltypejoin join
    @compositecelltypejoin_list.add_item join
  end

  #=== Cell# cell を composite セルタイプのセル用に clone する
  #name::        string : 親 cell の名前  (cell tComposite cell1 での cell1)
  #global_name:: string : 親 cell の global_name 
  #join_array::  Join[] : composite の cell の join で、この cell に対応するもの
  #ct_name::     string : 親セルのセルタイプ名
  #region::      Region : 元のセルが属する region
  #このメソッドは CompositeCelltype の expand から呼出される
  def clone_for_composite( name, global_name, namespacePath, join_array, ct_name, region, plugin, locale )

    # debug
    dbgPrint "  CLONING Cell#clone_for_composite : cloning: #{@name} #{global_name}  b_defined=#{@b_defined} #{self}=>#{@my_clone} \n"
    dbgPrint "              my_name=#{@name} name=#{name} owner class=#{@owner.class.name}\n"

    @my_clone = self.clone

    # clone したセルの内部に持つ名前情報を調整する
    @my_clone.set_cloned( name, global_name, namespacePath, join_array, ct_name, region, plugin, locale )

    # @celltype == nil は以前にセルタイプ未定義エラー
    if @b_defined == true && @celltype != nil then
      if @celltype.instance_of?( Celltype ) then
        # celltype に登録（コード生成の対象となる）
        @celltype.new_cell( @my_clone )
      end
    end

    return @my_clone
  end

  #=== Cell# clone されたセルの内部に持つ名前情報を調整する
  #name::        string : 親 cell の名前  (cell tComposite cell1 での cell1)
  #global_name:: string : 親 cell の global_name
  #join_array::  Join[] : composite の cell の join で、この cell に対応するもの
  #parent_ct_name:: string : 親セルのセルタイプ名（composite セルタイプ）
  #  このメソッドはすぐ上の clone_for_composite から呼出され、clone されたセルを整える
  def set_cloned( name, global_name, namespacePath, join_array, parent_ct_name, region, plugin, locale )

    # debug
    dbgPrint "cell.set_cloned : global_name: #{global_name}  name: #{name}  @name: #{@name}\n"
    dbgPrint "set_cloned:  entry_array_max_subscript.len=#{@entry_array_max_subscript.length}\n"
    @global_name = :"#{global_name}_#{@name}"
    @name = :"#{name}_#{@name}"
    @NamespacePath = namespacePath.change_name @name
    @region = region
    @plugin = plugin
    @locale = locale

    @in_composite = false
    @b_checked = false
    @f_cloned = true

    # Namespace.new_cell( self )  # mikan namespace 対応
    region.new_cell( self )  # mikan  namespace に cell を置けないことを仮定

    # join_list : NamedList の clone を作る
    if @celltype then
      dbgPrint "set_cloned: #{@celltype.get_name} #{@name} #{region.get_name}\n"
    end
    @join_list = @join_list.clone_for_composite( parent_ct_name, name, locale )
    @referenced_port_list = {}

    @alloc_list = []
    @require_joined_list = {}
    @entry_array_max_subscript = @entry_array_max_subscript.dup
    @cell_list = {}
    @cell_list2 = []

    # このセルのグローバル名を与える
    # C_EXP の$id$ 置換はこのセルの名前になる
    join_array.each { |j|
      @join_list.change_item j
    }
  end

  #=== clone されたセルが composite の場合、内部セルを展開する
  #self:: clone されたセルでなければならない
  def expand_inner
    if ! @f_cloned then
      raise "expnad_inner: not cloned cell"
    end

    # clone しようとするセルが composit セルタイプ？
    if @celltype.instance_of?( CompositeCelltype ) then
      # composite cell を再帰的に展開
      @cell_list, @cell_list2 = @celltype.expand( @name, @global_name, @NamespacePath, @join_list, @region, @plugin, @locale )
    end
  end

  #=== Cell# clone された cell の join_list の右辺の変更
  #  呼び口の右辺の cell を他の clone された cell に置換え
  def change_rhs_port cloned_cell_list

    # debug
    dbgPrint "=====   Cell#change_rhs_port: name=#{@name}   =====\n"

    @join_list.get_items.each { |j|
      j.change_rhs_port( cloned_cell_list, @celltype )
    }
  end

  def get_f_def
    @b_defined
  end

  def get_f_ref
    if @f_ref then
      return true
    else
      return false
    end
  end

  def get_name
    @name
  end

  def get_local_name
    @local_name
  end

  def get_global_name
    @global_name
  end

  def get_region
    @region
  end

  def self.get_current
    @@current_object
  end

  #=== Cell# 生成されるセルか？
  # 最適化、コード生成中に、対象となる region に属する場合 true を返す
  def is_generate?
    if $generating_region == nil then
      # 構文解釈、意味解析段階で呼ばれると例外発生
      raise "is_generate? called before optimizing"
    end

    # print "Cell#is_generate?: #{@name} #{@region.get_name} #{$generating_region.get_name}\n"
    if $generating_region == @region.get_link_root then
      return true
    else
      return false
    end
  end

  #=== Cell# composite 内部の複製されたセルか？
  # composite 定義の内部のセル (@in_composite = true) ではない
  def is_cloned?
    @f_cloned
  end

  #=== Cell# composite 内部のセルか？
  def is_in_composite?
    @in_composite
  end

  # composite cell の port に対応する内部の cell の port の名前（リンク時に必要な名前）
  def get_real_global_name port_name
    if @celltype.instance_of?( CompositeCelltype ) then

      # debug
      dbgPrint "get_real_global_name: cell name: #{@name} #{@local_name} #{@global_name} #{port_name}\n"
      @cell_list.each{ |n,c|
        dbgPrint "   name: #{n}\n"
        dbgPrint " get_name: #{c.get_name} local_name: #{c.get_local_name}\n"  if c
        dbgPrint "\n\n"
      }

      cj = @celltype.find_export( port_name )

      # debug
      dbgPrint " composite join name: #{cj.get_name}  cell: #{cj.get_cell_name}  cell elem: #{cj.get_cell_elem_name}\n"

      name = @cell_list[ "#{cj.get_cell_name}" ].get_real_global_name( cj.get_cell_elem_name )
      return name

    else
      # debug
      dbgPrint "  get_real_global_name: cell name: #{@global_name}\n"

      return @global_name
    end
  end

  #=== Cell# セルの受け口 port_name に対する実際のセル名、受け口名を '_' で連結
  #    namespace 名 + '_' + セル名 + '_' + 受け口名   （このセルが composite ならば展開後のセル名、受け口名）
  def get_real_global_port_name port_name

    # composite か？
    if @celltype.instance_of?( CompositeCelltype ) then

      # debug
      dbgPrint "get_real_global_port_name: cell name: #{@name} #{@local_name} #{@global_name} #{port_name}\n"
      @cell_list.each{ |n,c|
        dbgPrint "   name: #{n}\n"
        dbgPrint " get_name: #{c.get_name} local_name: #{c.get_local_name}\n"  if c
        dbgPrint "\n"
      }

      # セルタイプ内で port_name の CompositeCelltypeJoin を探す（コード生成段階では必ず見つかる）
      cj = @celltype.find_export( port_name )

      # debug
      dbgPrint "   composite join name: #{cj.get_name}  cell: #{cj.get_cell_name}  cell elem: #{cj.get_cell_elem_name}\n"

      # composite の内部のセルに対し再帰的に get_real_global_port_name を適用
      name = @cell_list[ "#{cj.get_cell_name}" ].get_real_global_port_name( cj.get_cell_elem_name )
      return name

    else
      # debug
      dbgPrint "get_real_global_port_name:  cell name: #{@global_name}\n"

      return "#{@global_name}_#{port_name}"
    end
  end

  #=== Cell# PORT (celltype の定義) を得る
  def get_real_port( port_name )

    # composite か？
    if @celltype.instance_of?( CompositeCelltype ) then

      # セルタイプ内で port_name の CompositeCelltypeJoin を探す（コード生成段階では必ず見つかる）
      cj = @celltype.find_export( port_name )

      # composite の内部のセルに対し再帰的に get_real_port を適用
      port = @cell_list[ "#{cj.get_cell_name}" ].get_real_port( cj.get_cell_elem_name )
      return port
    else

      return @celltype.find( port_name )
    end
  end

  #=== Cell# cell を得る
  #    composite でなければ自分自身を返す
  def get_real_cell( port_name )

    # composite か？
    if @celltype.instance_of?( CompositeCelltype ) then

      # セルタイプ内で port_name の CompositeCelltypeJoin を探す（コード生成段階では必ず見つかる）
      cj = @celltype.find_export( port_name )

      # composite の内部のセルに対し再帰的に get_real_port を適用
      cell = @cell_list[ "#{cj.get_cell_name}" ].get_real_cell( cj.get_cell_elem_name )
      return cell
    else

      return self
    end
  end


  #=== Cell# 受け口のport の参照カウントをアップする
  #port_name:: Symbol  : ポート名
  def port_referenced port
    if @referenced_port_list[ port ] then
      @referenced_port_list[ port ] += 1
    else
      @referenced_port_list[ port ] = 1
    end

    # composite か？
    if @celltype.instance_of?( CompositeCelltype ) then

      # セルタイプ内で port_name の CompositeCelltypeJoin を探す（コード生成段階では必ず見つかる）
      cj = @celltype.find_export( port.get_name )

      dbgPrint " port_referenced: #{@celltype.get_name} #{@name} cj=#{cj&&(cj.get_name)||"nil"}\n"

      if cj then  # 既にエラー
        # composite の内部のセルに対し再帰的に get_real_port を適用
        cell = @cell_list[ "#{cj.get_cell_name}" ]
        if cell && cell.get_celltype then
          cell.port_referenced( cell.get_celltype.find( cj.get_cell_elem_name ) )
        end
      end
    end
  end

  def get_internal_port_name port_name
    if @celltype.instance_of?( CompositeCelltype ) then
      cj = @celltype.find_export( port_name )
#      return "#{@name}_#{cj.get_cell.get_internal_port_name cj.get_cell_elem_name}"
      return cj.get_cell.get_internal_port_name( cj.get_cell_elem_name )
    else

      # debug
      dbgPrint "  get_global_port_name: cell port: #{@global_name}_#{port_name}\n"

      return "#{@global_name}_#{port_name}"
    end
  end

  def get_celltype
    @celltype
  end

  def get_join_list
    @join_list
  end

  def set_id id
    @id = id
  end

  def get_id
    @id
  end

  def get_plugin
    @plugin
  end

  def get_cell_list2
    list = []
    @cell_list2.each{ |cell|
      list << cell
      list += cell.get_cell_list2
    }
    return list
  end

  #=== Cell# 受け口配列の添数の最大値を設定
  def set_entry_port_max_subscript( port, num )
    dbgPrint( "set_entry_port_max_subscript: #{@name}.#{port.get_name}: #{num}\n" )
    subscript = @entry_array_max_subscript[port]

    if subscript == nil || num > subscript then
      @entry_array_max_subscript[port] = num
      set_entry_inner_port_max_subscript( port, num )
    end
  end

  #=== Cell# composite の内側セルの受け口配列の添数の最大値を設定
  def set_entry_inner_port_max_subscript( port, num )
    if @cell_list == nil then
      return    # プロトタイプ宣言しかされていなくて、内側セルが展開されていない　or composite 展開前
    end

    # composite の内側のセルに伝播
    if @celltype.instance_of? CompositeCelltype then
      dbgPrint "set_entry_inner_port_max_subscript #{@name} #{@port} #{num} cell_list.len=#{@cell_list.length}\n"
      # @cell_list.each{ |c, p| print c, p, '\n' }

      cj = @celltype.find_export port.get_name
      if cj && @cell_list[ cj.get_cell_name.to_s ] then
        inner_cell = @cell_list[ cj.get_cell_name.to_s ]
        ct = inner_cell.get_celltype
        if ct then
          inner_port = ct.find( cj.get_cell_elem_name )
          inner_cell.set_entry_port_max_subscript( inner_port, num )
        end
      end
    end
  end

  #=== Cell# 受け口配列の添数の最大値を返す
  # 長さは +1 する
  # 1つもない場合は -1 を返す
  def get_entry_port_max_subscript( port )
    subscript = @entry_array_max_subscript[port]
    if subscript == nil then
      subscript = -1
    end
    return subscript
  end

  #=== Cell# リレーアロケータの結合を生成
  # STAGE: S
  # 呼び口側の結合を元に受け口側の結合を生成
  def create_relay_allocator_join

    # celltype がなければチェックしない（既にエラー）
    return if @celltype == nil

    # relay allocator を生成
    @celltype.get_port_list.each { |p|
      ail = p.get_allocator_instance
      if ail then
        dbgPrint "create_relay_allocator_join: #{@name}, #{p.get_name}\n"
        if p.get_array_size then
          # mikan relay allocator が array に対応できてもよいのでは？
          cdl_error( "S1040 array not supported for relay allocator"  )
          next
        end
        ail.each{ |name,ai2|
          # ai2 = [ :INTERNAL_ALLOC|:RELAY_ALLOC, func_name, param_name, rhs_cp_name, rhs_func_name, rhs_param_name ]
          if ai2[0] == :RELAY_ALLOC then
            dbgPrint "create_relay_allocator_join: #{@name}, #{name}\n"
            # 呼び口側の結合を取り出す
            ja = @join_list.get_item( :"#{ai2[3]}_#{ai2[4]}_#{ai2[5]}" )
            if ja == nil then
              # 見つからない場合
              found = false
              
              # composite 内で外部に結合されているか
              if @in_composite then
                @compositecelltypejoin_list.get_items.each { |cj|
                  dbgPrint( "create relay_allocator in_composite\n" )
                  dbgPrint("    #{cj.get_cell_name} #{@name} #{cj.get_cell_elem_name} #{ai2[3]}_#{ai2[4]}_#{ai2[5]}\n")
                  if cj.get_cell_name == @name &&
                      cj.get_cell_elem_name == :"#{ai2[3]}_#{ai2[4]}_#{ai2[5]}" then
                    found = true
                    dbgPrint "create_relay_allocator: found #{cj.get_cell_elem_name}\n"
                    break
                  end
                }
              end

              if found == false then
                cdl_error( "S1041 \'$1_$2_$3\': not joined. cannot create internal join for relay allocator" , ai2[3], ai2[4], ai2[5] )
                print( "      In cell #{get_name}\n" )
                # join が未結合であることのエラーは二度でる (S1043)
              end
              next    # 打ち切る
            end

            b_export = false
            # composite 内のセルでエクスポートされているかチェック
            #  mikan エクスポート側と、こちら側で、リレー先が一致するかチェックが必要
            if @compositecelltypejoin_list then
              # export されているか調べる
              @compositecelltypejoin_list.get_items.each{ |cj|
                # 属性名と composite の export する名前は一致するか
                if p.get_name == cj.get_cell_elem_name then
                  print "export : #{p.get_name}\n"
                  b_export = true    # 属性は export されているので、とりあえず未初期化とはしない
                  break
                end
              }
              # 
            end

            # mikan 配列
            am = nil
            if am then
              am.each{ |ja2|
                rhs = ja2.get_rhs
                subscript = ja2.get_subscript
                if b_export == false then
                  # CompositeCelltype の場合、内側のセルで生成させる
                  join = Join.new( :"#{p.get_name}_#{ai2[1]}_#{ai2[2]}", subscript, rhs, @loacle )
                  # p ( "#{p.get_name}_#{ai2[1]}_#{ai2[2]}", subscript, rhs )
                  new_join( join )
                  join.set_definition( @celltype.find(join.get_name) )
                  # mikan relay mismatch チェックができていない（下方を参照）
                end
                @alloc_list << [ :NORMAL_ALLOC, p.get_name, subscript, ai2[1], ai2[2], rhs ]
              }
            else
              if b_export == false then
                # CompositeCelltype の場合、内側のセルで生成させる
                join = Join.new( :"#{p.get_name}_#{ai2[1]}_#{ai2[2]}", nil, ja.get_rhs, @locale )
                new_join( join )
                join.set_definition( @celltype.find(join.get_name) )
                if @celltype.instance_of? CompositeCelltype then
                  jr = @join_list.get_item( :"#{ai2[3]}_#{ai2[4]}_#{ai2[5]}" )
                  if jr.get_rhs_cell2 != join.get_rhs_cell2 || jr.get_rhs_port2 != join.get_rhs_port2 then
                    cdl_error( "S1167 \'$1\': relay mismatch \'$2\'",
                                      "#{p.get_name}_#{ai2[1]}_#{ai2[2]}",
                                      "#{ai2[3]}_#{ai2[4]}_#{ai2[5]}" )
                    # 本当は composite の呼び口と受け口の間で行うべきだが、内部で多段接続されている場合
                  else
                    dbgPrint "relay success:  #{p.get_name}_#{ai2[1]}_#{ai2[2]}=>#{ai2[3]}_#{ai2[4]}_#{ai2[5]} #{jr.get_rhs_cell2.get_name}.#{jr.get_rhs_port2} \n"
                  end
                end
              end
              @alloc_list << [ :NORMAL_ALLOC, p.get_name, nil, ai2[1], ai2[2], ja.get_rhs ]
            end
            dbgPrint "create_relay_allocator_join: #{p.get_name}_#{ai2[1]}_#{ai2[2]} #{ai2[3]}_#{ai2[4]}_#{ai2[5]}\n"
          end
        }
      end
    }
  end

  #=== Cell# @@cell_list2 を作る
  # @@cell_list2 は、出現順に composite 内を含むセルのリスト
  def self.make_cell_list2
    @@cell_list.each{ |c|
      @@cell_list2 << c
      @@cell_list2 += c.get_cell_list2
    }
  end

  #=== Cell# @@cell_list2 を得る
  # composite 内を含む (compositeも含む)
  # 意味解析後に作成される
  def self.get_cell_list2
    @@cell_list2
  end

  #=== Cell# @@cell_list を得る
  #composite の中を含まない
  def self.get_cell_list
    @@cell_list
  end

  #=== Cell# reverse_join を生成する
  def self.create_reverse_join
    @@cell_list.each{ |c|
      ct = c.get_celltype
      # if c.is_generate? then
      if ct then
        c.create_reverse_join
      end
      # end
    }
  end

  #=== Cell# reverse_require_join を生成する
  def self.create_reverse_require_join
    @@cell_list2.each{ |c|
      ct = c.get_celltype
      # if c.is_generate? then
        if ct then
          # self への呼び口側の結合を生成
          ct.create_reverse_require_join c
        end
      # end
    }
  end

  #=== Cell# 受け口のport の参照カウントを設定する
  # self は呼び元のセル
  # 呼び先セルの受け口の参照カウントをアップする
  def set_port_reference_count
    @join_list.get_items.each { |j|
      if j.get_definition.instance_of? Port then
        am = j.get_array_member2
        if am then             # 呼び口配列
          am.each { |j2|
            next if j2 == nil    # optional で一部が欠落しているケース
            cell = j2.get_rhs_cell2
            next if cell == nil     # 右辺が見つからなかった．既にエラー
            port = cell.get_celltype.find( j2.get_rhs_port2 )
            dbgPrint( "set_port_reference_count: #{@name}.#{j2.get_name} => #{cell.get_name}.#{port.get_name}\n")
            cell.port_referenced port
          }
        else
          cell = j.get_rhs_cell2
          next if cell == nil     # 右辺が見つからなかった．既にエラー
          port = cell.get_celltype.find( j.get_rhs_port2 )
          dbgPrint( "set_port_reference_count: #{@name}.#{j.get_name} => #{cell.get_name}.#{port.get_name}\n")
          cell.port_referenced port
        end
      end
    }
  end

  #=== Cell# 結合(Join)のチェック
  #     Join は呼び口の結合または attribute の初期化
  #
  #  mikan このメソッドは、以下の４つのチェックからなるが、分割したほうがより適切な長さのメソッドになる
  #  ・リレーアロケータの生成 => create_relay_allocator_join
  #  ・未結合の呼び口のチェック
  #  ・ポインタ型が配列で初期化される場合のチェック
  #  ・未初期化の属性のチェック
  def check_join

    # celltype がなければチェックしない（既にエラー）
    return if @celltype == nil
    return if @b_defined == false
    return if @f_cloned == true    # 内部セルについては、composite の定義時にチェックされている

    # debug
    # if @compositecelltypejoin_list then
    #   p "check_join"
    #   @compositecelltypejoin_list.get_items.each { |cj| p "#{cj.get_name} #{cj.get_name.object_id}" }
    # end

    # 未結合の呼び口のチェック
    @celltype.get_port_list.each { |p|

      # 呼び口でなければ、チェックしない
      next if p.get_port_type != :CALL

      # debug
      dbgPrint "check_join: #{@name} #{get_celltype.get_name} #{p.get_name}\n"

      # 結合リストの中から呼び口名に一致するものを取りだす
      j = @join_list.get_item( p.get_name )

      if j == nil then
        # 未結合の呼び口

        # composite celltype の内部の場合、composite celltype が export する呼び口に結合されているか探す
        found = false
        if @in_composite then
          # composite celltype の export するものすべてから探す
          # （export するものの右辺値から探すために get_item ではダメ）
          @compositecelltypejoin_list.get_items.each{ |cj|
            # 呼び口名と composite の export する名前は一致するか
            if p.get_name == cj.get_cell_elem_name then
              found = true
            end
          }
        end

        # 呼び口配列の場合 optional で全ての要素が初期化されない場合に、ここへ来る
        if ! found && ! p.is_require? && ! p.is_optional? then
          if ! p.is_allocator_port? then
            cdl_error( "S1042 call port \'$1\' not initialized in cell \'$2\'" , p.get_name, @name )
          else
            cdl_error( "S1043 call port \'$1\' not initialized in cell \'$2\'. this call port is created by tecsgen. check allocator specifier" , p.get_name, @name )
          end
        end
      elsif p.get_array_size.kind_of? Integer then
        # 添数あり呼び口配列の場合、すべての添数要素が初期化されているかチェックする

        am = j.get_array_member2
        if( am )then
          # join は配列

          # 呼び口配列定義での配列の大きさ
          length = p.get_array_size

          # 配列の大きさが呼び口配列定義と結合定義で一致するか？
          if am.length != length then
            if ! p.is_optional? || am.length >= length then
              # optional の場合、要素数が少なすぎるのは OK
              cdl_error( "S1044 $1: array initializer too many or few, $2 for $3" , p.get_name, am.length, length )
            end

            # am の要素に nil を追加しておく (#_CPA_# のコード生成時、この配列要素数分生成)
            i = am.length
            while i < length
              am << nil
              i += 1
            end
          end

#          # 配列要素の抜けがないかチェック
#          if am.length < length then  # 満たない場合既にエラーだが要素のある範囲でチェック
#            length = am.length
#          end
          i = 0
          while( i < length )
            if am[i] == nil then
              if ! p.is_optional? then
                cdl_error( "S1045 $1[$2]: not initialized" , p.get_name, i )
              end
            else
              # 生成されないリージョンへの結合かチェック
              if ! @in_composite then
                am[i].check_region2
              end
            end
            i += 1
          end

        # else
        # join が非配列であれば、既にエラー
        end
      elsif j.get_array_member then
        # 添数なし呼び口配列の場合
        am = j.get_array_member2
        length = am.length
        i = 0
        while i < length
          if am[i] == nil then
            if ! p.is_optional? then
              cdl_error( "S1046 $1[$2]: not initialized" , p.get_name, i )
            end
          end
          i += 1
        end

        # 生成されないリージョンへの結合かチェック
        if ! @in_composite then
          am.each { |join|
            if join then
              join.check_region2
            end
          }
        end
      else
        # 呼び口［配列」でない場合

        # 生成されないリージョンへの結合かチェック
        if ! @in_composite then
          j.check_region2
        end

      end # j != nil
    }

    # ポインタ型が配列で初期化される場合のチェック
    (@celltype.get_attribute_list+@celltype.get_var_list).each { |a|
      if a.get_size_is then

        if a.instance_of? CompositeCelltypeJoin then
          # 既にエラーになっている
          # cdl_error( "S1047 size_is pointer cannot be exposed for composite attribute"  )
          next
        end

        if( ! a.get_type.kind_of?( PtrType ) ) then
          cdl_error( "S1048 $1: size_is specified for non-pointer type" , a.get_name )
        else
          size = a.get_size_is.eval_const( @join_list, @celltype.get_name_list )
          a.get_type.set_scs( a.get_size_is, nil, nil, nil, false )
          if( ! size.kind_of? Integer )then               # C_EXP の可能性あり
            # mikan 多分ここでのエラー発生は不要、eval_const の中で変数が存在しない、型が不適切などのエラーになるはず
            cdl_error( "S1049 $1: size_is arg not constant" , a.get_name )
          else
            j = @join_list.get_item( a.get_identifier )
            if j then
              ini = j.get_rhs
              if ini then
                if ! ini.instance_of?( Array ) then
                  cdl_error( "S1050 unsuitable initializer, need array initializer"  )
                elsif size < ini.length then
                  cdl_error( "S1051 too many initializer for array, $1 for $2" , ini.length, size )
                else
                  # a.get_type.set_scs( a.get_size_is, nil, nil )
                end
              end
            else
              # size_is 引数がセルで指定されていて、初期化子がセルタイプで指定されているケースのチェック
              ini = a.get_initializer
              if ini.instance_of? Expression
                ini = ini.eval_const( @celltype.get_name_list )
              end
              if ini.instance_of? Array then
                if( ini.length > size )then
                  cdl_error( "S1168 too many initializer for array, $1 for $2", ini.length, size )
                end
              end
            end
          end
        end
      else
        if ! a.instance_of? CompositeCelltypeJoin then
          # composite は size_is 指定できない
          if a.get_type.kind_of?( PtrType ) then
            j = @join_list.get_item( a.get_identifier )
            if j && j.get_rhs.instance_of?( Array ) then
              ## size_is 指定されていないポインタが Array で初期化されていたら、エラーとする
              cdl_error( "S1169 $1: non-size_is pointer cannot be initialized with array initializer" , a.get_identifier )
            end
          end
        end
      end
    }

    # 未初期化の属性をチェック
    @celltype.get_attribute_list.each { |a|
      b_init = false
      # self.show_tree 1
      if a.get_initializer then                               # セルタイプで初期化されている
        b_init = true
        # @in_composite で export されている場合には、この初期値は使われない
        # export されている、いないに関わらず、初期化されていることが保証される
      elsif @join_list.get_item( a.get_name ) then            # セルで初期化されている
        b_init = true
      elsif @in_composite && @compositecelltypejoin_list then
        # 属性が export されているか調べる。export されていれば未初期化とはしない
        # mikan リニアサーチ
        @compositecelltypejoin_list.get_items.each{ |cj|
          # 属性名と composite の export する名前は一致するか
          if a.get_name.to_sym == cj.get_cell_elem_name.to_sym then
            b_init = true    # 属性は export されているので、とりあえず未初期化とはしない
          end
        }
        if b_init then
          # size_is の引数がマッチするかチェックする
          # 内部セルの size_is をエクスポートする size_is とマッチするかチェックする
          # 内部セルとエクスポートで名前を変えている可能性があるので、内部セルの size_is の名前を変換した上でチェックする
          if a.get_size_is then
            ### p "attr: get_size_is"
            cj = @compositecelltypejoin_list.get_item a.get_name.to_sym
            if cj.get_port_decl.instance_of? Decl then
              ### p "attr: get_size_is 2"
              # cj_size_is は、外部公開される attr の size_is
              cj_size_is = cj.get_port_decl.get_size_is
              if cj_size_is == nil then
                cdl_error( "S1170 \'$1\' has size_is but export attr \'$2\' doesn't have", a.get_name, cj.get_name )
              end
              exprs = a.get_size_is.to_s
              ### p "exprs : ", exprs
              remain = exprs
              inner_to_export = {}
			  ### exprs に含まれる識別子を抜き出し、対応する export される名前を探す
              while remain != "" && remain != nil
                ### p "remain ", remain
                remain =~ /([^\w]*)([_A-Za-z][\w\d]*)/   # 変数名文字列を取り出す 
				if $2 == nil then
						break
				end
                arg_name = $2.to_sym
                remain = $'
                ### p exprs, $1, $2, $'
                # size_is に含まれる変数は、composite で export されているか
                cj2 = nil
                @compositecelltypejoin_list.get_items.each{ |cj2t|
                  if cj2t.get_cell_elem_name == arg_name then
                    cj2 = cj2t
                  end
                }
                if cj2 == nil then
                  cdl_error( "S1171 \'$1\' size_is argument of \'$2\' not exported", a.get_name, cj.get_name )
                  next
                end
                if cj2.get_port_decl.instance_of? Decl then
                   decl2 = cj2.get_port_decl
                   # 内部の名前と外部の名前の対応関係を記憶
                   inner_to_export[arg_name] = decl2.get_name
                # else cj2 は Port (既にエラー)
                end
              end
              # 内部の名前を外部の名前で置換
              inner_to_export.each{ |arg_name, exp_name|
                ### p "changing #{arg_name}=>#{exp_name}"
                # exprs.gsub!( Regexp.new("#{arg_name}[^0-9A-Za-z_]"), exp_name.to_s )
                exprs.gsub!( Regexp.new("#{arg_name}(\\W)"), exp_name.to_s+"\\1" )  # 文字列末尾にないケース
                exprs.gsub!( Regexp.new("#{arg_name}\\Z"), exp_name.to_s )          # 文字列末尾にあるケース
              }
              ### p "changed: #{exprs} #{cj_size_is.to_s}"
              if exprs != cj_size_is.to_s then
                cdl_error( "S1172 \'$1\' size_is argument mismatch with exporting one \'$2\'", a.get_name, cj.get_name )
              end
            # else cj は Port (既にエラー)
            end
          end
        end    
      end

      if b_init == false then
          cdl_error( "S1052 attribute \'$1\' not initialized in cell \'$2\'" , a.get_name, @name )
      end
   
    }
  end

  #=== Cell# 逆 require をチェックする
  # 逆 require 指定された受け口に複数の結合がないかチェックする
  # composite の内部セル (f_cloned=true) もチェックする
  def check_reverse_require
    # celltype がなければチェックしない（既にエラー）
    return if @celltype == nil
    return if @b_defined == false

    # p "check reverse require   #{@name}"
    # 逆require 指定された受け口に複数の結合がないかチェック
    @referenced_port_list.each{ |port,count|
      # p port.class, count
      # p port.get_name, port.get_port_type, port.get_signature.get_name
      if port.is_reverse_required? && count > 1 then
        cdl_warning( "W1009 $1: fixed join entry port has multi join", port.get_name )
      end
    }
  end

  #=== Cell# require 呼び口の結合を行う
  # STAGE: S
  #cp_name:: Symbol           : 呼び口名
  #cell_or_t:: Celltype|Cell  : celltype の require の右辺で指定されたセルタイプまたはセル
  #port::  Port               : celltype の Port オブジェクト
  def set_require_join( cp_name, cell_or_ct, port )

    # set_require_join は2度呼び出される
    # 2度目は post コードを生成した後       #####  いったん見合わせ（重複エラーを見逃す）
    # if @require_joined_list[ cp_name ] then
    #   return
    # else
    #   @require_joined_list[ cp_name ] = true
    # end

    dbgPrint "set_require_join: #{@name}.#{cp_name} = #{cell_or_ct.get_name}.#{port.get_name}\n"

    if cell_or_ct.instance_of? Celltype then
      # print "DOMAIN: not considered\n"
      cell = cell_or_ct.get_singleton_cell @region
      if cell == nil then
        cdl_error( "S1025 not found reachable cell for require \'$1\' in celltype \'$2\'" , port.get_name, cell_or_ct.get_name )
        return
      end
    else
      # require: cell で指定
      cell = cell_or_ct
      if @region.distance( cell.get_region ) == nil then
        cdl_error( "S1026 required cell \'$1\' not reachable" , cell.get_name )
      end
    end

    if @join_list.get_item( cp_name ) then
      cdl_warning( "W1003 $1 : require call port overridden in $2" , cp_name, @name )
    else
      # require の join を生成(呼び口の結合)
#      rhs = Expression.new( [ :OP_DOT, [ :IDENTIFIER, Token.new( cell.get_name, nil, nil, nil ) ],
      nsp = NamespacePath.new( cell.get_name, false, cell.get_namespace )
      nsp.set_locale @locale
      rhs = Expression.new( [ :OP_DOT, [ :IDENTIFIER, nsp ],
                              Token.new( port.get_name, nil, nil, nil ) ], @locale )   #1
      join = Join.new( cp_name, nil, rhs, @locale )
      self.new_join( join )

      join.set_definition( @celltype.find(join.get_name) )
    end
  end

  #=== Cell# Join の definition の設定とチェック
  # STAGE: S
  def set_definition_join
    return if @celltype == nil    # 既にエラー：打ち切る
    return if @b_defined == false # プロトタイプ宣言のみ
    return if @b_checked == true  # 既に設定（チェック）済み

    dbgPrint "set_definition_join in #{@name}\n"

    # relay allocator をたどって再入しないよう、先頭で @b_checked を true にする
    @b_checked = true

    if ! @f_cloned then
      check_restrict_list
      
      # compoiste セルのクローンされたものは、set_definition 不要
      # 元の join は既に definition されている
      # 元のセルにおいて、代入チェックされているので、二重にチェック(through適用)されてしまう
      @join_list.get_items.each{ |join|
        dbgPrint " set_definition_join: checking #{@name}.#{join.get_name}\n"
        if join.get_array_member then
          port = @celltype.find(join.get_name)
          join.get_array_member2.each { |am|
            if am == nil then   # 未結合の場合、エラーチェックは check_join
              if port && ! port.is_optional? then
                # テスト用にエラーメッセージ出力
                # cdl_error( "TEMPORAL set_definition_join: uninitialized array member"  )
              end
              next
            end
            am.set_definition( port )
          }
        else
          dbgPrint "set_definition_join: #{@name}.#{join.get_name} celltype=#{@celltype.get_name}\n"
          join.set_definition( @celltype.find(join.get_name) )
        end
      }
    end

    # リレー join は through プラグイン生成後にしかできない
    # through 後に結合先が入れ替えられる 
    create_relay_allocator_join

    # composite セルの展開
    if ! @in_composite && ! @f_cloned && @celltype.instance_of?( CompositeCelltype ) then
      # composite セルタイプ内の composite は展開しない
      # compoiste セル展開中の composite は展開しない (CompositeCelltype::expand 内で再帰的に expnad)
      expand
    end
  end

  #=== Cell# composite セルの展開
  # このセルが composite セルタイプ
  def expand

    #debug
    dbgPrint "=====    expanding   #{@name}     =====\n"

    # composite celltype の cell を展開
    @cell_list, @cell_list2 = @celltype.expand( @name, @global_name, @NamespacePath, @join_list, @region, @plugin, @locale )

    # プロトタイプが参照されている場合、子も参照されていることにする
    if @f_ref then
      dbgPrint "expand: set_f_ref\n"
      set_f_ref
    end
  end

  #=== Cell#内部セルの受け口添数最大値を設定
  def set_max_entry_port_inner_cell
    if @cell_list == nil then
      return
    end

    dbgPrint "set_max_entry_port_inner_cell name=#{@name} entry_array_max_subscript.len=#{@entry_array_max_subscript.length}\n"

    # プロトタイプ宣言で設定されていたものを反映する
    @entry_array_max_subscript.each{ |port,name|
      dbgPrint "set_entry_inner_port_max_subscript( #{port}, #{name} )\n"
      set_entry_inner_port_max_subscript( port, name )
    }
  end

  #=== Cell#restrict を追加
  def add_restrict( entry_name, func_name, region_name_list )
    if @restrict_list[ entry_name ] then
      if @restrict_list[ entry_name ][ func_name ] then
        @restrict_list[ entry_name ][ func_name ].each{ |rn|
          if region_name_list.include? rn then
            # p func_name
            name = func_name ? entry_name : entry_name+"."+func_name
            cdl_warning( "W9999 $1 restrict region duplicate $2", name, rn )
          end
        }
      else
        @restrict_list[ entry_name ][ func_name ] = region_name_list
      end
    else
      func_list = { }
      func_list[ func_name ] = region_name_list
      @restrict_list[ entry_name ] = func_list
    end
    # pp @restrict_list
  end

  #=== Cell#check_restrict_list
  def check_restrict_list
    @restrict_list.each{ |entry_name, func_hash|
      func_hash.each{ |func_name, region_list|
        region_list.each{ |rn|
          obj = Namespace.find [ rn ]
          if ( obj.kind_of? Region ) then
            if obj.get_domain_root != @region.get_domain_root then
            else
              cdl_warning( "W9999 $1 in same domain", rn )
            end
          else
            cdl_error( "S9999 $1 not region", region )
          end
        }
      }
    }
  end

  #=== Cell#callable?
  def callable?( callee_cell, entry_name, func_name )
    res = callee_cell.callable_from?( entry_name, func_name, self )
    dbgPrint "callable? #{callee_cell.get_namespace_path}.#{entry_name}.#{func_name} from #{@NamespacePath} is #{res}\n"
    return res
  end

  #=== Cell#callable_from? (private)
  def callable_from?( entry_name, func_name, caller_cell )
    if @restrict_list.length == 0 then
      return true
    end

    if @restrict_list[entry_name] then
      if @restrict_list[entry_name][nil] &&
         @restrict_list[entry_name][nil].include?( caller_cell.get_region.get_domain_root.get_name )then
        return true
      end
      if @restrict_list[entry_name][func_name] &&
         @restrict_list[entry_name][func_name].include?( caller_cell.get_region.get_domain_root.get_name )then
        return true
      else
        return false
      end
    else
      return true
    end
  end
  
  def show_tree( indent )
    indent.times { print "  " }
    puts "Cell: name: #{@name} in_composite: #{@in_composite} def: #{@b_defined} ref: #{@f_ref} cloned: #{@f_cloned}"
    (indent+1).times { print "  " }
    puts "Cell locale: #{@name}@#{@locale[0]}##{@locale[1]}"
    (indent+1).times { print "  " }
    puts "id: #{@id}  global_name: #{@global_name}  region: #{@region.get_name} plugin: #{@plugin.class.name} #{self}"
    (indent+1).times { print "  " }
    puts "namespace_path: #{@NamespacePath}"

    if @celltype then
      (indent+1).times { print "  " }
      puts "celltype: #{@celltype.get_name}"
    end
    @join_list.show_tree( indent + 1 )
    @entry_array_max_subscript.each{ |port, num|
      (indent+1).times { print "  " }
      puts "entry array #{port.get_name}: max subscript=#{num}"
    }
    if @cell_list then   # ここで @cell_list が nil なのは Bug
      (indent+1).times { print "  " }
      puts "cloned cell list:"
      @cell_list.each { |n,c|
        (indent+2).times { print "  " }
        puts "inner cell : #{n} = #{c.get_name}"
      }
    end
    if @compositecelltypejoin_list then
      @compositecelltypejoin_list.get_items.each{ |cj|
        cj.show_tree( indent+1 )
      }
    end
    if @alloc_list.length > 0 then
      (indent+1).times { print "  " }
      puts "allocator list: "
      @alloc_list.each { |a|
        cp_name = :"#{a[0+1]}_#{a[2+1]}_#{a[3+1]}"
        if a[1+1] then
          # subscript = "[#{a[1+1].eval_const nil}]"
          subscript = "[#{a[1+1]}]"
        else
          subscript = ""
        end
        # アロケータ呼び口の結合を生成
        (indent+2).times { print "  " }
        puts "#{cp_name}#{subscript} = #{a[4+1]}"
      }
    end
    @referenced_port_list.each{ |port,count|
      (indent+1).times { print "  " }
      puts( "#{port.get_name} : #{count} times referenced" )
    }
  end

end

class CompositeCelltype < NSBDNode # < Nestable
# @name:: str
# @global_name:: str
# @cell_list_in_composite:: NamedList   Cell
# @cell_list::Array :: [ Cell ] : cell of CompositeCelltype's cell
# @export_name_list:: NamedList : CompositeCelltypeJoin
# @port_list:: CompositeCelltypeJoin[]
# @attr_list:: CompositeCelltypeJoin[]
# @b_singleton:: bool : 'singleton' specified
# @b_active:: bool : 'active' specified
# @real_singleton:: bool : has singleton cell in this composite celltype
# @real_active:: bool : has active cell in this composite celltype
# @name_list:: NamedList item: Decl (attribute), Port エクスポート定義
# @internal_allocator_list:: [ [cell, internal_cp_name, port_name, func_name, param_name, ext_alloc_ent], ... ]
# @generate:: [ Symbol, String, Plugin ]  = [ PluginName, option, Plugin ] Plugin は生成後に追加される
# @generate_list:: [ [ Symbol, String, Plugin ], ... ]   generate 文で追加された generate

  @@nest_stack_index = -1
  @@nest_stack = []
  @@current_object = nil

  include CelltypePluginModule
  include PluginModule

  def self.push
    @@nest_stack_index += 1
    @@nest_stack[ @@nest_stack_index ] = @@current_object
    @@current_object = nil
  end

  def self.pop
    @@current_object = @@nest_stack[ @@nest_stack_index ]
    @@nest_stack_index -= 1
    if @@nest_stack_index < -1 then
      raise "TooManyRestore"
    end
  end

  def initialize( name )
    super()
    @name = name
    @cell_list_in_composite = NamedList.new( nil, "in composite celltype #{name}" )
    @cell_list = []
    @export_name_list = NamedList.new( nil, "export in composite celltype #{name}" )
    @name_list = NamedList.new( nil, "in composite celltype #{name}" )
    @@current_object = self

    @b_singleton = false
    @real_singleton = nil
    @b_active = false
    @real_active = nil
    if "#{Namespace.get_global_name}" == "" then
      @global_name = @name
    else
      @global_name = :"#{Namespace.get_global_name}_#{@name}"
    end

    Namespace.new_compositecelltype( self )
    set_namespace_path # @NamespacePath の設定

    @port_list = []
    @attr_list = []
    @internal_allocator_list = []
    @generate_list = []
    set_specifier_list( Generator.get_statement_specifier )
  end

  def self.end_of_parse
    @@current_object.end_of_parse
    @@current_object = nil
  end

  # CompositeCelltype#end_of_parse
  def end_of_parse
    # singleton に関するチェック
    if @b_singleton && @real_singleton == nil then
      cdl_warning( "W1004 $1 : specified singleton but has no singleton in this celltype" , @name )
    elsif ! @b_singleton && @real_singleton != nil then
      if ! @b_singleton then
        cdl_error( "S1053 $1 must be singleton. inner cell \'$2\' is singleton" , @name, @real_singleton.get_name )
      end
    end

    # active に関するチェック
    if @b_active && @real_active == nil then
      cdl_error( "S1054 $1 : specified active but has no active in this celltype" , @name )
    elsif ! @b_active && @real_active != nil then
      cdl_error( "S1055 $1 must be active. inner cell \'$2\' is active" , @name, @real_active.get_name )
    end

    # @allocator_instance を設定する
    @name_list.get_items.each{ |n|
      if n.instance_of? Port then
        n.set_allocator_instance
      end
    }

    # リレーアロケータの entry 側
    @port_list.each{ |p|
      if p.get_port_type == :ENTRY then
        if p.get_allocator_instance == nil then
          next
        end

        p.get_allocator_instance.each{ |name,ai|
          if ai[0] == :RELAY_ALLOC then
            self.new_join( :"#{p.get_name}_#{ai[4]}_#{ai[5]}", p.get_cell_name, :"#{p.get_cell_elem_name}_#{ai[4]}_#{ai[5]}", :CALL )
          end
        }
      end
    }
    # mikan relay が正しく抜けているかチェックされていない

    # callback 結合
    @cell_list_in_composite.get_items.each{ |c|
      ct = c.get_celltype
      if ct then
        c.create_reverse_join
      end
    }

    # 意味解析
    @cell_list_in_composite.get_items.each{ |c|
      c.set_definition_join
    }

    # cell の未結合の呼び口がないかチェック
    @cell_list_in_composite.get_items.each{ |c|
      c.check_join
      c.check_reverse_require
    }

    # 呼び口の結合について、export と内部結合の両方がないかチェック
    # リレーアロケータ、内部アロケータの設定
    @port_list.each{ |p|
      p.check_dup_init
    }

    # すべてのエクスポート定義に対応した呼び口、受け口、属性が存在するかチェック
    @name_list.get_items.each{ |n|
      if( @export_name_list.get_item( n.get_name ) == nil )then
        cdl_error( "S1056 $1 : cannot export, nothing designated" , n.get_name )
      end
    }

    # 内部アロケータを設定する
    @internal_allocator_list.each{ |cell, cp_internal_name, port_name, fd_name, par_name, ext_alloc_ent|
      res = ext_alloc_ent.get_allocator_rhs_elements( :INTERNAL_ALLOC )
      ep_name = res[0]
      cj = @export_name_list.get_item( ep_name )
      internal_alloc_name_from_port_def = cj.get_cell_name
      internal_alloc_ep_name_from_port_def = cj.get_cell_elem_name

      # puts "internal_allocator #{cell.get_name} #{cp_internal_name} #{port_name}.#{fd_name}.#{par_name}"
      cell.get_allocator_list.each{ |a|
        # puts "allocator_list of #{cell.get_name} #{a[0]} #{a[1]}.#{a[2]}.#{a[3]}.#{a[4]} #{a[5].to_s}"
        if cp_internal_name == :"#{a[1]}_#{a[3]}_#{a[4]}" then
          dbgPrint "internal_allocator {cp_internal_name} #{a[1]}_#{a[3]}_#{a[4]}\n"
          dbgPrint "internal_allocator: #{a[5]}, #{internal_alloc_name_from_port_def}.#{internal_alloc_ep_name_from_port_def}\n"
          if a[5].to_s != "#{internal_alloc_name_from_port_def}.#{internal_alloc_ep_name_from_port_def}" then
            cdl_error( "S1173 $1: allocator mismatch from $2's allocator", "#{port_name}.#{fd_name}.#{par_name}", cell.get_name  )
          end
        end
      }
    }

    # composite プラグイン
    if @generate then
      celltype_plugin
    end
  end

 ### CompositeCelltype#new_cell_in_composite
  def self.new_cell_in_composite( cell )
    @@current_object.new_cell_in_composite( cell )

  end

  def new_cell_in_composite( cell )
    cell.set_owner self  # Cell (in_omposite)
    @cell_list_in_composite.add_item( cell )
    if cell.get_celltype then    # nil ならば、すでにセルタイプなしエラー
      if cell.get_celltype.is_singleton? then
        @real_singleton = cell
      end
      if cell.get_celltype.is_active? then
        @real_active = cell
      end
    end
  end

 ### join
  def self.new_join( export_name, internal_cell_name,
			 internal_cell_elem_name, type )
    @@current_object.new_join( export_name, internal_cell_name,
					 internal_cell_elem_name, type )
    
  end

 ### CompositeCelltype#new_cell
  def new_cell cell
    @cell_list << cell

    # セルタイププラグインの適用
    celltype_plugin_new_cell cell
  end

  #=== CompositeCelltype# CompositeCelltypeJoin を作成
  # STAGE: B
  #export_name:: Symbol : 外部に公開する名前
  #internal_cell_name:: Symbol : 内部セル名
  #internal_cell_elem_name:: Symbol : 内部セルの要素名（呼び口名、受け口名、属性名のいずれか）
  #type::  :CALL, :ENTRY, :ATTRIBUTE のいずれか（構文要素としてあるべきもの）
  #RETURN:: Decl | Port : エクスポート定義
  # new_join は
  #   cCall => composite.cCall;     (セル内)
  #   attr = composite.attr;        (セル内)
  #   composite.eEnt => cell2.eEnt; (セル外)
  # の構文要素の出現に対して呼び出される
  def new_join( export_name, internal_cell_name,
		 internal_cell_elem_name, type )

    dbgPrint "new_join: #{export_name} #{internal_cell_name} #{internal_cell_elem_name}\n"

    cell = @cell_list_in_composite.get_item( internal_cell_name )
    if cell == nil then
      cdl_error( "S1057 $1 not found in $2" , internal_cell_name, @name )
      return
    end

    celltype = cell.get_celltype
    return if celltype == nil	# celltype == nil ならすでにエラー

    # 内部セルのセルタイプから対応要素を探す
    # このメソッドは、構文上、呼び口、受け口、属性が記述できる箇所から呼出される
    # 構文上の呼出し位置（記述位置）と、要素が対応したものかチェック
    obj = celltype.find( internal_cell_elem_name )
    if obj.instance_of?( Decl ) then
      if obj.get_kind == :VAR then
        cdl_error( "S1058 \'$1\' : cannot export var" , internal_cell_elem_name )
        return
      elsif type != :ATTRIBUTE then
        cdl_error( "S1059 \'$1\' : exporting attribute. write in cell or use \'=\' to export attribute" , export_name )
        # return 次のエラーを避けるために処理続行し、付け加えてみる
      end
    elsif obj.instance_of?( Port ) then
      if obj.get_port_type != type then
        cdl_error( "S1060 \'$1\' : port type mismatch. $2 type is allowed here." , export_name, type )
        # return 次のエラーを避けるために処理続行し、付け加えてみる
      end
    else
      cdl_error( "S1061 \'$1\' : not defined" , internal_cell_elem_name )
      dbgPrint "S1061 CompositeCelltypeJoin#new_join: #{export_name} => #{internal_cell_name}.#{internal_cell_elem_name} #{type}\n"
      return
    end

    # エクスポート定義と一致するかどうかチェック
    obj2 = @name_list.get_item( export_name )
    if( obj2 == nil )then
      cdl_error( "S1062 $1 has no export definition" , export_name )
    elsif obj2.instance_of?( Decl ) then
      if( ! obj.instance_of? Decl )then
        cdl_error( "S1063 $1 is port but previously defined as an attribute" , export_name )
      elsif ! obj.get_type.equal? obj2.get_type then
        cdl_error( "S1064 $1 : type \'$2$3\' mismatch with pprevious definition\'$4$5\'" , export_name, obj.get_type.get_type_str, obj.get_type.get_type_str_post, obj2.get_type.get_type_str, obj2.get_type.get_type_str_post )
      end
    elsif obj2.instance_of?( Port ) then
      if( obj.instance_of? Port )then
        if( obj.get_port_type != obj2.get_port_type )then
          cdl_error( "S1065 $1 : port type $2 mismatch with previous definition $3" , export_name, obj.get_port_type, obj2.get_port_type )
        elsif obj.get_signature != obj2.get_signature then
          if obj.get_signature != nil && obj2.get_signature != nil then
            # nil ならば既にエラーなので報告しない
            cdl_error( "S1066 $1 : signature \'$2\' mismatch with previous definition \'$3\'" , export_name, obj.get_signature.get_name, obj2.get_signature.get_name )
          end
        elsif obj.get_array_size != obj2.get_array_size then
          cdl_error( "S1067 $1 : array size mismatch with previous definition" , export_name )
        elsif obj.is_optional? != obj2.is_optional? then
          cdl_error( "S1068 $1 : optional specifier mismatch with previous definition" , export_name )
        elsif obj.is_omit? != obj2.is_omit? then
          cdl_error( "S9999 $1 : omit specifier mismatch with previous definition" , export_name )
        elsif obj.is_dynamic? != obj2.is_dynamic? then
          cdl_error( "S9999 $1 : dynamic specifier mismatch with previous definition" , export_name )
        elsif obj.is_ref_desc? != obj2.is_ref_desc? then
          cdl_error( "S9999 $1 : ref_desc specifier mismatch with previous definition" , export_name )
        end
      else
        cdl_error( "S1069 $1 is an attribute but previously defined as a port" , export_name )
      end
    end

    join = CompositeCelltypeJoin.new( export_name, internal_cell_name,
				 internal_cell_elem_name, cell, obj2 )
    join.set_owner self   # CompositeCelltypeJoin
    cell.add_compositecelltypejoin join

    # debug
    dbgPrint "compositecelltype join: add #{cell.get_name} #{export_name} = #{internal_cell_name}.#{internal_cell_elem_name}\n"

    if obj.instance_of?( Decl ) then
      # attribute
#      # 内部から外部へ複数の結合がないかチェック
#      found = false
#      @attr_list.each{ |a|
#        if a.get_name == join.get_name then
#          found = true
#          break
#        end
#      }
#      if found == false then
        @attr_list << join
#      end
    else
      # call/entry port
#      # 内部から外部へ複数の結合がないかチェック
#      found = false
#      @port_list.each{ |port|
#        if port.get_name == join.get_name then
#          found = true
#          break
#        end
#      }
#      if found == false then
        @port_list << join
#      end
    end

    # join を @export_name_list に登録（重複チェックとともに，後で行われる CompositeCelltypeJoin の clone に備える）
    if obj.instance_of?( Decl ) && @export_name_list.get_item( export_name ) then
      # 既に存在する。追加しない。新仕様では、@export_name_list に同じ名前が含まれることがある。
    elsif obj.instance_of?( Port ) && obj.get_port_type == :CALL && @export_name_list.get_item( export_name ) then
      # 既に存在する。追加しない。新仕様では、@export_name_list に同じ名前が含まれることがある。
    else
      @export_name_list.add_item( join )
    end

    # export するポートに含まれる send/receive パラメータのアロケータ(allocator)呼び口をセルと結合
    if obj2.instance_of? Port then
      obj2.each_param{ |port, fd, par|
        case par.get_direction                        # 引数の方向指定子 (in, out, inout, send, receive )
        when :SEND, :RECEIVE
          cp_name = :"#{port.get_name}_#{fd.get_name}_#{par.get_name}"     # アロケータ呼び口の名前
          #            ポート名         関数名         パラメータ名
          cp_internal_name = :"#{internal_cell_elem_name}_#{fd.get_name}_#{par.get_name}"

          # リレーアロケータ or 内部アロケータ指定がなされている場合、アロケータ呼び口を追加しない
          # この時点では get_allocator_instance では得られないため tmp を得る
          if port.get_allocator_instance_tmp then
            found = false
            port.get_allocator_instance_tmp.each { |s|
              if s[1] == fd.get_name && s[2] == par.get_name then
                found = true

                if s[0] == :INTERNAL_ALLOC then
                  # 内部アロケータの場合    # mikan これは内部のセルに直結する。外部のポートに改めるべき
                  @internal_allocator_list << [ cell, cp_internal_name, port.get_name, fd.get_name, par.get_name, s[3] ]
                end
              end
            }
            if found == true
              next
            end
          end

          # 外部アロケータの場合
          new_join( cp_name, internal_cell_name, cp_internal_name, :CALL )
        end
      }
    end

    # エクスポート定義を返す
    return obj2
  end

  def self.has_attribute? attr
    @@current_object.has_attribute? attr
  end

  def has_attribute? attr
    @name_list.get_item( attr ) != nil
  end

  def self.new_port port
    @@current_object.new_port port
  end

  #=== CompositeCelltype# new_port
  def new_port port
    port.set_owner self   # Port (CompositeCelltype)
    dbgPrint "new_port: #{@owner.get_name}.#{port.get_name}\n"
    @name_list.add_item port

    # export するポートに含まれる send/receive パラメータのアロケータ呼び口の export を生成してポートに追加
    # この時点では内部アロケータかどうか判断できないので、とりあえず生成しておく
    port.each_param { |port, fd, par|
      case par.get_direction                        # 引数の方向指定子 (in, out, inout, send, receive )
      when :SEND, :RECEIVE
        #### リレーアロケータ or 内部アロケータ指定がなされている場合、アロケータ呼び口を追加しない
        # 内部アロケータ指定がなされている場合、アロケータ呼び口を追加しない
        # この時点では get_allocator_instance では得られないため tmp を得る
        if port.get_allocator_instance_tmp then
          found = false
          port.get_allocator_instance_tmp.each { |s|
            if s[0] == :INTERNAL_ALLOC && s[1] == fd.get_name && s[2] == par.get_name then
              found = true
              break
            end
          }
          if found == true
            next
          end
        end

        if par.get_allocator then
          cp_name = :"#{port.get_name}_#{fd.get_name}_#{par.get_name}"     # アロケータ呼び口の名前
          #           ポート名          関数名         パラメータ名
          alloc_sig_path = [ par.get_allocator.get_name ]  # mikan Namespace アロケータ呼び口のシグニチャ
          array_size = port.get_array_size            # 呼び口または受け口配列のサイズ
          created_port = Port.new( cp_name, alloc_sig_path, :CALL, array_size ) # 呼び口を生成
          created_port.set_allocator_port( port, fd, par )
          if port.is_omit? then
            created_port.set_omit
          end
          new_port( created_port )           # セルタイプに新しい呼び口を追加
        # else
        #   already error
        end
      end
    }
  end

  def self.new_attribute attr
    @@current_object.new_attribute attr
  end

  #=== CompositeCelltype# new_attribute for CompositeCelltype
  #attribute:: [Decl]
  def new_attribute( attribute )
    attribute.each { |a|
      a.set_owner self   # Decl (CompositeCelltype)
      # V1.1.0.10 composite の attr の size_is は可となった
      # if a.get_size_is then
      #  cdl_error( "S1070 $1: size_is pointer cannot be exposed for composite attribute" , a.get_name )
      # end
      @name_list.add_item( a )
      if a.get_initializer then
        a.get_type.check_init( @locale, a.get_identifier, a.get_initializer, :ATTRIBUTE )
      end
    }
  end

  #=== CompositeCelltype# 逆require の結合を生成する
  def create_reverse_require_join cell
    @name_list.get_items.each{ |n|
      if n.instance_of? Port then
        n.create_reverse_require_join cell
      end
    }
  end

  # false : if not in celltype definition, nil : if not found in celltype
  def self.find( name )
    if @@current_object == nil then
      return false
    end
    @@current_object.find name
  end

  def find name
    dbgPrint "CompositeCelltype: find in composite: #{name}\n"
    cell = @cell_list_in_composite.get_item( name )
    return cell if cell

    dbgPrint "CompositeCelltype: #{name}, #{@name_list.get_item( name )}\n"
    return @name_list.get_item( name )

    # 従来仕様
#    cj = @export_name_list.get_item( name )
#p "#{name}, #{cj.get_port_decl}"
#    if cj then
#      return cj.get_port_decl
#    else
#      return nil
#    end
  end

  #=== CompositeCelltype# export する CompositeCelltypeJoin を得る
  #name:: string:
  # attribute の場合、同じ名前に対し複数存在する可能性があるが、最初のものしか返さない
  def find_export name
    return @export_name_list.get_item( name )
  end

  #=== CompositeCelltype# composite celltype の cell を展開
  #name:: string: Composite cell の名前
  #global_name:: string: Composite cell の global name (C 言語名)
  #join_list:: NamedList : Composite cell に対する Join の NamedList
  #RETURN:
  # [ { name => cell }, [ cell, ... ] ]
  #  戻り値 前は 名前⇒cloneされた内部セル、後ろは composite の出現順のリスト
  def expand( name, global_name, namespacePath, join_list, region, plugin, locale )

    # debug
    dbgPrint "expand composite: #{@name} name: #{name}  global_name: #{global_name}\njoin_list:\n"
    join_list.get_items.each{ |j|
      dbgPrint "   #{j.get_name} #{j}\n"
    }
  
    # 展開で clone されたセルのリスト、右辺は Cell (composite の場合 composite な cell の clone)
    clone_cell_list = {}
    clone_cell_list2 = []
    clone_cell_list3 = {}

    #  composite 内部のすべての cell について
    @cell_list_in_composite.get_items.each { |c|

      # debug
      dbgPrint "expand : cell #{c.get_name}\n"

      # Join の配列
      ja = []

      # CompositeCelltype が export する呼び口、受け口、属性のリストについて
      # @export_name_list.get_items.each{ |cj|	# cj: CompositeCelltypeJoin
      # 新仕様では、@export_name_list に入っていない attr がありうる
      (@port_list+@attr_list).each{ |cj|	# cj: CompositeCelltypeJoin

        # debug
        dbgPrint "        cj : #{cj.get_name}\n"

        # CompositeCelltypeJoin (export) の対象セルか？
        if cj.match?( c ) then

          # 対象セル内の CompositeCelltype の export する Join (attribute または call port)
          j = join_list.get_item( cj.get_name )

          # debug
          if j then
            dbgPrint "  REWRITE_EX parent cell: #{name} child cell: #{c.get_name}:  parent's export port: #{cj.get_name}  join: #{j.get_name}=>#{j.get_rhs.to_s}\n"
          else
            dbgPrint "expand : parent cell: #{name} child cell: #{c.get_name}:  parent's export port: #{cj.get_name}  join: nil\n"
          end

          if j then
            # 呼び口、属性の場合
            #  ComositeCell 用のもの(j) を対象セル用に clone (@through_list もコピーされる)
            # p "expand: cloning Join #{j.get_name} #{@name} #{name}"
            jc = j.clone_for_composite( @name, name, locale )
                                        # celltype_name, cell_name

            # debug
            # p "cn #{jc.get_name} #{cj.get_cell_elem_name}"

            # 対象セルの呼び口または属性の名前に変更
            jc.change_name( cj.get_cell_elem_name )

            # 対象セルに対する Join の配列
            ja << jc
          end

          # debug
          dbgPrint "\n"
        end
      }

      # debug
      dbgPrint "expand : clone #{name}_#{c.get_name}\n"

      # セルの clone を生成
#      clone_cell_list[ "#{name}_#{c.get_name}" ] =  c.clone_for_composite( name, global_name, ja )
      c2 =  c.clone_for_composite( name, global_name, namespacePath, ja, @name, region, plugin, locale )
      clone_cell_list[ "#{c.get_local_name}" ] = c2
      clone_cell_list2 << c2
      clone_cell_list3[ c ] = c2

    }

    clone_cell_list.each { |nm,c|
      dbgPrint "  cloned: #{nm} = #{c.get_global_name}\n"
      # join の owner を clone されたセルに変更する V1.1.0.25
      c.get_join_list.get_items.each{ |j|
        j.set_cloned( clone_cell_list[ "#{c.get_local_name}" ] )
      }
      dbgPrint "change_rhs_port: inner cell #{c.get_name}\n"
      c.change_rhs_port clone_cell_list3
    }
    clone_cell_list2.each { |c|
      c.expand_inner
    }
    return [ clone_cell_list, clone_cell_list2 ]
  end

  #=== CompositeCelltype 指定子リストの設定
  def set_specifier_list( spec_list )
    return if spec_list == nil

    spec_list.each { |s|
      case s[0]
      when :SINGLETON
        @b_singleton = true
      when :IDX_IS_ID
        cdl_warning( "W1005 $1 : idx_is_id is ineffective for composite celltype" , @name )
      when :ACTIVE
        @b_active = true
      when :GENERATE
        if @generate then
          cdl_error( "S9999 generate specifier duplicate"  )
        end
        @generate = [ s[1], s[2] ] # [ PluginName, "option" ]
      else
        cdl_error( "S1071 $1 cannot be specified for composite" , s[0] )
      end
    }
  end

  def get_name
    @name
  end

  def get_port_list
    @port_list
  end

  def get_attribute_list
    @attr_list
  end

  def get_var_list
    []   # 空の配列を返す
  end

  def get_internal_allocator_list
    @internal_allocator_list
  end

  #== CompositeCelltype# generate 指定子の情報
  # CompositeCelltype には generate が指定できないので nil を返す
  # Celltype::@generate を参照のこと
  def get_celltype_plugin
    nil
  end

  def is_singleton?
    @b_singleton
  end

  def is_active?
    @b_active
  end

  #=== CompositeCelltype# アクティブではない
  # active ではないに加え、全ての内部セルのセルタイプが inactive の場合に inactive
  # （内部のセルが active または factory を持っている）
  def is_inactive?
    if @b_active == false then
      @cell_list_in_composite.get_items.each{ |c|
        if c.get_celltype && c.get_celltype.is_inactive? == false then
          # c.get_celltype == nil の場合はセルタイプ未定義ですでにエラー
          return false
        end
      }
      return true
    else
      return false
    end
  end

  def get_id_base
    raise "get_id_base"
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "CompositeCelltype: name: #{@name}"
    (indent+1).times { print "  " }
    puts "active: #{@b_active}, singleton: #{@b_singleton}"
    @cell_list_in_composite.show_tree( indent + 1 )
    (indent+1).times { print "  " }
    puts "name_list"
    @name_list.show_tree( indent + 2 )
    (indent+1).times { print "  " }
    puts "export_name_list"
    @export_name_list.show_tree( indent + 2 )
    if @internal_allocator_list.length > 0 then
      (indent+1).times { print "  " }
      puts "internal_allocator_list:"
      @internal_allocator_list.each{  |a|
        (indent+1).times { print "  " }
        puts "  #{a[0].get_name} #{a[1]} #{a[2]} #{a[3]} #{a[4]}"
      }
    end
  end

end



#== 構文要素：口を表すクラス（セルタイプの呼び口、受け口）
class Port < BDNode
# @name::  str
# @signature:: Signature
# @port_type::  :CALL, :ENTRY
# @array_size:: nil: not array, "[]": sizeless, Integer: sized array
# @reverse_require_cell_path:: NamespacePath :     逆require呼び元セル  mikan namespace (呼び口のみ指定可能)
# @reverse_require_callport_name:: Symbol:  逆require呼び元セルの呼び口名
#
# set_allocator_port によって設定される．設定された場合、このポートはアロケータポートである。
# @allocator_port:: Port : この呼び口ができる元となった呼び口または受け口
# @allocator_func_decl:: Decl : この呼び口ができる元となった呼び口または受け口の関数
# @allocator_param_decl:: ParamDecl : この呼び口ができる元となった呼び口または受け口のパラメータ
#
# set_specifier によって設定される(
# @allocator_instance:: Hash : {"func_param" => [ :RELAY_ALLOC, func_name, param_name, rhs_cp_name, rhs_func_name, rhs_param_name ]}
#                                               [:INTERNAL_ALLOC, func_name, param_name, rhs_ep_name ]
# @allocator_instance_tmp:: Hash : {"func_param" => [:INTERNAL_ALLOC|:RELAY_ALLOC,  IDENTIFIER, IDENTIFIER, expression ],..}
#                                                                                    function    parameter   rhs
#
# @b_require:: bool : require により生成された call port の場合 true
# @b_has_name:: bool : require : 名前ありのリクワイア呼び口
# @b_inline:: bool : entry port のみ
# @b_omit:: bool : omit 指定子が指定された (call port のみ)
# @b_optional:: bool : call port のみ
# @b_ref_desc:: bool :  ref_desc キーワードが指定された
# @b_dynamic:: bool :  dynamic キーワードが指定された (呼び口のみ)
#
# optimize::
# @celltype:: 属するセルタイプ
#
# :CALL の場合の最適化
# @b_VMT_useless:: bool                     # VMT 関数テーブルを使用しない
# @b_skelton_useless:: bool                 # スケルトン関数不要   (true の時、受け口関数を呼出す)
# @b_cell_unique:: bool                     # 呼び先は唯一のセル
# @only_callee_port:: Port                  # 唯一の呼び先ポート
# @only_callee_cell:: Cell                  # 唯一の呼び先セル (@b_PEPDES_in_CB_useless = true の時有効)
#
# :ENTRY の場合の最適化（呼び口最適化と同じ変数名を使用）
# @b_VMT_useless:: bool                     # VMT 関数テーブルが不要
# @b_skelton_useless:: bool                 # スケルトン関数不要

  def initialize( name, sig_path, port_type, array_size = nil, reverse_require_cell_path = nil, reverse_require_entry_port_name = nil )
    super()
    @name = name
    @port_type = port_type

    if array_size == "[]" then
#      if port_type == :ENTRY then
#        cdl_error( "S1072 $1: entry port: sizeless array not supported in current version" , name )
#      end
      @array_size = array_size
    elsif array_size then
      if array_size.kind_of? Expression then
        @array_size = array_size.eval_const(nil)
      else
        @array_size = array_size   # これはアロケータ呼び口の場合（元の呼び口で既に評価済み）
      end
      if @array_size == nil then
        cdl_error( "S1073 Not constant expression $1" , array_size.to_s )
      end

      #if Integer( @array_size ) != @array_size || @array_size <= 0 then
      if ! @array_size.kind_of? Integer then
        cdl_error( "S1074 Not Integer $1" , array_size.to_s )
      end

    end

    object = Namespace.find( sig_path )    #1
    if object == nil then
      # mikan signature の名前が不完全
      cdl_error( "S1075 \'$1\' signature not found" , sig_path )
    elsif ! object.instance_of?( Signature ) then
      # mikan signature の名前が不完全
      cdl_error( "S1076 \'$1\' not signature" , sig_path )
    else
      @signature = object

    end

    # 逆require
    @reverse_require_cell_path       = nil
    @reverse_require_entry_port_name = nil
    if reverse_require_cell_path then
      if port_type == :CALL then
        cdl_error( "S1152 $1 call port cannot have fixed join", @name )
      else
        @reverse_require_cell_path       = reverse_require_cell_path
        @reverse_require_entry_port_name = reverse_require_entry_port_name

        # 受け口配列か？
        if array_size then
          cdl_error( "S1153 $1: cannot be entry port array for fixed join port", @name )
        end

        # 呼び口のセルタイプを探す
        ct_or_cell = Namespace.find( @reverse_require_cell_path )  #1
        if ct_or_cell.instance_of? Cell then
          ct = ct_or_cell.get_celltype
        elsif ct_or_cell.instance_of? Celltype then
          ct = ct_or_cell
          if ! ct.is_singleton? then
            cdl_error( "S1154 $1: must be singleton celltype for fixed join", @reverse_require_cell_path.to_s )
          end
        else
          ct = nil
          cdl_error( "S1155 $1: not celltype or not found", @reverse_require_cell_path.get_path_str)
        end

        if ct == nil then
          return    # 既にエラー
        end

        # 添え字なしの呼び口配列か？
        port = ct.find( @reverse_require_entry_port_name )
        if port == nil || port.get_port_type != :CALL
          cdl_error( "S1156 $1: not call port or not found", @reverse_require_entry_port_name )
        else
          if port.get_array_size != "[]" then
            cdl_error( "S1157 $1: sized array or not array", @reverse_require_entry_port_name )
          end
        end

      end
    end

    @b_require = false
    @b_has_name = false
    @b_inline = false
    @b_optional = false
    @b_omit = false
    @b_ref_desc = false
    @b_dynamic = false
    reset_optimize
  end

  #=== Port#最適化に関する変数をリセットする
  # Region ごとに最適化のやりなおしをするため、リセットする
  def reset_optimize
    if @port_type == :CALL then
      # call port optimize
      @b_VMT_useless = false                     # VMT 不要 (true の時 VMT を介することなく呼出す)
      @b_skelton_useless = false                 # スケルトン関数不要   (true の時、受け口関数を呼出す)
      @b_cell_unique = false                     # 唯一の呼び先セル
      @only_callee_port = nil                    # 唯一の呼び先ポート
      @only_callee_cell = nil                    # 唯一の呼び先セル
    else
      # entry port optimize
      if $unopt then
        # 最適化なし
        @b_VMT_useless = false                     # VMT 不要 (true の時 VMT を介することなく呼出す)
        @b_skelton_useless = false                 # スケルトン関数不要   (true の時、受け口関数を呼出す)
      else
        # 最適化あり
        @b_VMT_useless = true                      # VMT 不要 (true の時 VMT を介することなく呼出す)
        @b_skelton_useless = true                  # スケルトン関数不要   (true の時、受け口関数を呼出す)
      end
    end
end

  def set_celltype celltype
    @celltype = celltype
  end

  def get_name
    @name
  end

  def get_port_type
    @port_type
  end

  def get_signature
    @signature
  end

  def get_array_size
    @array_size
  end

  def get_celltype
    @celltype
  end

  #=== Port# アロケータポートの設定
  #port:: Port : send/receive のあった呼び口または受け口
  #fd:: Decl : 関数の declarator
  #par:: ParamDecl : send/receive のあった引数
  # この呼び口が生成されるもとになった呼び口または受け口の情報を設定
  def set_allocator_port( port, fd, par )
    @allocator_port = port
    @allocator_func_decl = fd
    @allocator_param_decl = par
  end

  def is_allocator_port?
    @allocator_port != nil
  end

  def get_allocator_port
    @allocator_port
  end

  def get_allocator_func_decl
    @allocator_func_decl
  end

  def get_allocator_param_decl
    @allocator_param_decl
  end

  def set_require( b_has_name )
    @b_require = true
    @b_has_name = b_has_name
  end

  def is_require?
    @b_require
  end

  #=== Port# require 呼び口が名前を持つ？
  # require 限定
  def has_name?
    @b_has_name
  end

  def is_optional?
    @b_optional
  end

  def set_optional
    @b_optional = true
  end

  #=== Port# omit 指定されている?
  def is_omit?
    @b_omit || ( @signature && @signature.is_empty? )
  end

  def set_omit
    @b_omit = true
  end

  def set_VMT_useless                     # VMT 関数テーブルを使用しない
   @b_VMT_useless = true
  end

  def set_skelton_useless                 # スケルトン関数不要   (true の時、受け口関数を呼出す)
    @b_skelton_useless = true
  end

  def set_cell_unique                     # 呼び先セルは一つだけ
    @b_cell_unique = true
  end

  #=== Port# 呼び口/受け口の指定子の設定
  # inline, allocator の指定
  def set_specifier spec_list
    spec_list.each { |s|
      case s[0]
      when :INLINE
        if @port_type == :CALL then
          cdl_error( "S1077 inline: cannot be specified for call port"  )
          next
        end
        @b_inline = true
      when :OMIT
        if @port_type == :ENTRY then
          cdl_error( "S9999 omit: cannot be specified for entry port"  )
          next
        end
        @b_omit = true
      when :OPTIONAL
        if @port_type == :ENTRY then
          cdl_error( "S1078 optional: cannot be specified for entry port"  )
          next
        end
        @b_optional = true
      when :REF_DESC
        if @port_type == :ENTRY then
          cdl_error( "S9999 ref_desc: cannnot be specified for entry port" )
          next
        end
        @b_ref_desc = true
      when :DYNAMIC
        if @port_type == :ENTRY then
          cdl_error( "S9999 dynamic: cannnot be specified for entry port" )
          next
        end
        @b_dynamic = true
      when :ALLOCATOR
        if @port_type == :CALL then
          cdl_error( "S1079 allocator: cannot be specified for call port"  )
        end
        if @allocator_instance_tmp then
          cdl_error( "S1080 duplicate allocator specifier"  )
          next
        end
        @allocator_instance_tmp = s[1]
      else
        raise "unknown specifier #{s[0]}"
      end
    }
    if ( @b_dynamic || @b_ref_desc ) then
      if @b_dynamic then
        dyn_ref = "dynamic"
      else
        dyn_ref = "ref_desc"
      end
      if @b_omit then     # is_omit? は is_empty? も含んでいるので使えない
        cdl_error( "S9999 omit cannot be specified with $1", dyn_ref  )
      elsif @signature && @signature.is_empty? then
        cdl_error( "S9999 $1 cannot be specified for empty signature", dyn_ref  )
      elsif @signature && @signature.has_descriptor? then
        # cdl_error( "S9999 $1 port '$2' cannot have Descriptor in its signature", dyn_ref, @name )
      end

    elsif @b_dynamic && @b_ref_desc then
      cdl_error( "S9999 both dynamic & ref_desc cannot be specified simultaneously"  )
    end
  end

  #=== Port# リレーアロケータ、内部アロケータのインスタンスを設定
  # 呼び口の前方参照可能なように、セルタイプの解釈の最後で行う
  def set_allocator_instance
    if @allocator_instance_tmp == nil then
      return
    end

    @allocator_instance = {}
    @allocator_instance_tmp.each { |ai|
      direction = nil
      alloc_type = ai[0]
      # ai = [ :INTERNAL_ALLOC|:RELAY_ALLOC, func_name, param_name, rhs ]
      case alloc_type
      when :INTERNAL_ALLOC
        if ! @owner.instance_of? CompositeCelltype then # ミスを防ぐために composite でなければとした
          cdl_error( "S1081 self allocator not supported yet"  )   # mikan これはサポートされているはず。要調査 12/1/15
          next
        end
        # OK
      when :RELAY_ALLOC
        # OK
      when :NORMAL_ALLOC
        # ここへ来るのは composite の受け口で右辺が "eEnt.func.param" 形式で指定されていた場合
        cdl_error( "S1174 $1 not suitable for lhs, suitable lhs: 'func.param'", "#{ai[1]}.#{ai[3]}.#{ai[4]}" )
        next
      else
        raise "Unknown allocator type #{ai[1]}"
      end

      # '=' 左辺(func_name,param_name)は実在するか?
      if @signature then       # signature = nil なら既にエラー
        fh = @signature.get_function_head( ai[1] )
        if fh == nil then
          cdl_error( "S1082 function \'$1\' not found in signature" , ai[1] )
          next
        end
        decl = fh.get_declarator
        if ! decl.is_function? then
          next   # 既にエラー
        end
        paramdecl = decl.get_type.get_paramlist.find( ai[2] )
        if paramdecl == nil then
          cdl_error( "S1083 \'$1\' not found in function \'$2\'" , ai[2], ai[1] )
          next
        end
        case paramdecl.get_direction
        when :SEND, :RECEIVE
          # OK
          direction = paramdecl.get_direction
        else
          cdl_error( "S1084 \'$1\' in function \'$2\' is not send or receive" , ai[2], ai[1] )
          next
        end
      end

      # 重複指定がないか?
      if @allocator_instance[ "#{@name}_#{ai[1]}_#{ai[2]}" ] then
        cdl_error( "S1085 duplicate allocator specifier for \'$1_$2\'" , ai[1], ai[2] )
      end

      # 右辺のチェック
      case alloc_type
      when :INTERNAL_ALLOC

        ele = ai[3].get_elements
        if( ele[0] != :IDENTIFIER )then
          cdl_error( "S1086 $1: rhs not in 'allocator_entry_port' form", ai[3].to_s )
          next
        end

        ep_name = ele[1]   # アロケータ受け口名
        ep = @owner.find ep_name.get_path[0]  # mikan "a::b"
        if ep == nil || ! ep.instance_of?( Port ) || ep.get_port_type != :ENTRY || ! ep.get_signature.is_allocator? then
          cdl_error( "S1175 $1 not found or not allocator entry port for $2" , ep_name, ai[1] )
        end
        # 右辺チェック終わり
        # ai2 = [ :INTERNAL_ALLOC, func_name, param_name, rhs_ep_name ]
        ai2 = [ ai[0], ai[1], ai[2], ep_name ]

      when :RELAY_ALLOC
        ele = ai[3].get_elements
        if( ele[0] != :OP_DOT ||
            ele[1][0] != :OP_DOT || ele[1][1][0] != :IDENTIFIER || ! ele[1][1][1].is_name_only? ||
            ! ele[1][2].instance_of?( Token ) || ! ele[2].instance_of?( Token ) )then   #1
          # [ :OP_DOT, [ :OP_DOT, [ :IDENTIFIER,  name_space_path ],  Token(1) ],  Token(2) ]
          #    ele[0]    ele[1][0]  ele[1][1][0]  ele[1][1][1]        ele[1][2]    ele[2]
          #      name_space_path.Token(1).Token(2) === call_port.func.param
          #  mikan Expression#analyze_cell_join_expression の変種を作成して置き換えるべき

          cdl_error( "S1176 rhs not in 'call_port.func.param' form for for $1_$2" , ai[1], ai[2] )   # S1086
          next
        end
        func_name = ele[1][2]; cp_name = ele[1][1][1].get_name; param_name = ele[2].to_sym
        cp = @owner.find cp_name    # リレーする先の呼び口
        if cp then
# mikan cp が呼び口であることのチェック（属性の場合もある）
# mikan 受け口から受け口へのリレーへの対応 (呼び口から呼び口へのリレーはありえない)  <=== 文法にかかわる事項（呼び口側でアロケータが決定される）
          sig = cp.get_signature
          if sig && @signature then
            fh = @signature.get_function_head( func_name )
            if fh == nil then
              cdl_error( "S1087 function \'$1\' not found in signature \'$2\'" , func_name, sig.get_name )
              next
            end
            decl = fh.get_declarator
            if ! decl.is_function? then
              next   # 既にエラー
            end
            paramdecl = decl.get_type.get_paramlist.find( param_name )
            if paramdecl == nil then
              cdl_error( "S1088 \'$1\' not found in function \'$2\'" , param_name, func_name )
              next
            end
            case paramdecl.get_direction
            when :SEND, :RECEIVE
              # OK
              if alloc_type == :RELAY_ALLOC && direction != paramdecl.get_direction then
                cdl_error( "S1089 relay allocator send/receive mismatch between $1.$2 and $3_$4.$5" , ai[1], ai[2], cp_name, func_name, param_name )
              end
            else
              cdl_error( "S1090 \'$1\' in function \'$2\' is not send or receive" , param_name, func_name )
              next
            end

            # else
            # sig == nil ならば既にエラー
          end
        else
          if @celltype then
            ct_name = @celltype.get_name
          else
            ct_name = "(None)"
          end
          cdl_error( "S1091 call port \'$1\' not found in celltype $2" , cp_name, ct_name )
          next
        end
        # 右辺チェック終わり
        # ai2 = [ :RELAY_ALLOC, func_name, param_name, rhs_cp_name, rhs_func_name, rhs_param_name ]
        ai2 = [ ai[0], ai[1], ai[2], cp_name, func_name, param_name ]
      end # case alloc_type

      @allocator_instance[ "#{@name}_#{ai[1]}_#{ai[2]}" ] = ai2
    }
  end

  def is_inline?
    @b_inline
  end

  def is_VMT_useless?                     # VMT 関数テーブルを使用しない
   @b_VMT_useless
  end

  def is_skelton_useless?                 # スケルトン関数不要   (true の時、受け口関数を呼出す)
    @b_skelton_useless
  end

  def is_cell_unique?                     # 呼び先のセルは一つ？
    @b_cell_unique
  end

  #=== Port# 受け口最適化の設定
  # この受け口を参照する呼び口が VMT, skelton を必要としているかどうかを設定
  # 一つでも呼び口が必要としている（すなわち b_*_useless が false）場合は、
  # この受け口の最適化を false とする
  def set_entry_VMT_skelton_useless( b_VMT_useless, b_skelton_useless )
    if ! b_VMT_useless then
      @b_VMT_useless = false
    end
    if ! b_skelton_useless then
      @b_skelton_useless = false
    end
  end

  #=== Port# 唯一の結合先を設定
  # 最適化で使用
  #  b_VMT_useless == true || b_skelton_useless == true の時に設定される
  #  optional の場合 callee_cell, callee_port が nil となる
  def set_only_callee( callee_port, callee_cell )
    @only_callee_port = callee_port
    @only_callee_cell = callee_cell
  end

  #=== Port# 唯一の結合先ポートを返す(compositeの場合実セル)
  # optional 呼び口で未結合の場合 nil を返す
  def get_real_callee_port
    if @only_callee_cell then
      return @only_callee_cell.get_real_port( @only_callee_port.get_name )
    end
  end

  #=== Port# 唯一の結合先セルを返す(compositeの場合実セル)
  # optional 呼び口で未結合の場合 nil を返す
  def get_real_callee_cell
    if @only_callee_cell then
      return @only_callee_cell.get_real_cell( @only_callee_port.get_name )
    end
  end

  def get_allocator_instance
    return @allocator_instance
  end

  def get_allocator_instance_tmp
    return @allocator_instance_tmp
  end

  #=== Port# 逆require の結合を生成する
  # STAGE: S
  def create_reverse_require_join cell
    if @reverse_require_cell_path == nil then
      return
    end

    # 呼び元セルを探す
    ct_or_cell = Namespace.find( @reverse_require_cell_path )   # mikan namespace    #1
    if ct_or_cell.instance_of? Cell then
      cell2 = ct_or_cell
      ct = cell2.get_celltype
      if ct == nil then
        return    # 既にエラー
      end
    elsif ct_or_cell.instance_of? Celltype then
      cell2 = ct_or_cell.get_singleton_cell( cell.get_region )
      if cell2 == nil then
        cdl_error( "S1158 $1: singleton cell not found for fixed join", ct_or_cell.get_name )
        return
      end
      ct = ct_or_cell
    else
      # 既にエラー：無視
      return
    end

    # 結合を生成する
    dbgPrint "create_reverse_require_join #{cell2.get_name}.#{@reverse_require_entry_port_name}[] = #{cell.get_name}.#{@name}"
    nsp = NamespacePath.new( cell.get_name, false, cell.get_namespace )
#    rhs = Expression.new( [ :OP_DOT, [ :IDENTIFIER, Token.new( cell.get_name, nil, nil, nil ) ],
    rhs = Expression.new( [ :OP_DOT, [ :IDENTIFIER, nsp ],
                            Token.new( @name, nil, nil, nil ) ], cell.get_locale )   #1
    join = Join.new( @reverse_require_entry_port_name, -1, rhs, cell.get_locale )
    cell2.new_join( join )
    join.set_definition( ct.find(join.get_name) )

  end

  #=== Port# signature のすべての関数のすべてのパラメータをたどる
  #block:: ブロックを引数として取る(ruby の文法で書かない)
  #  ブロックは3つの引数を受け取る(Port, Decl,      ParamDecl)    Decl: 関数ヘッダ
  # Signature クラスにも each_param がある（同じ働き）
  def each_param # ブロック引数{  |port, func_decl, param_decl| }
    return if @signature == nil                         # signature 未定義（既にエラー）
    fha = @signature.get_function_head_array            # 呼び口または受け口のシグニチャの関数配列
    return if fha == nil                                # nil なら文法エラーで有効値が設定されなかった

    pr = Proc.new   # このメソッドのブロック引数を pr に代入
    port = self
    fha.each{ |fh|  # fh: FuncHead                      # 関数配列中の各関数頭部
      fd = fh.get_declarator                            # fd: Decl  (関数頭部からDeclarotorを得る)
      if fd.is_function? then                           # fd が関数でなければ、すでにエラー
        fd.get_type.get_paramlist.get_items.each{ |par| # すべてのパラメータについて
          pr.call( port, fd, par )
        }
      end
    }
  end

  #=== Port# 逆require指定されている？
  def is_reverse_required?
    @reverse_require_cell_path != nil
  end

  #=== Port# is_dynamic?
  def is_dynamic?
    @b_dynamic
  end

  #=== Port# is_ref_desc?
  def is_ref_desc?
    @b_ref_desc
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "Port: name:#{@name} port_type:#{@port_type} require:#{@b_require} inline:#{@b_inline} omit:#{@b_omit} optional:#{@b_optional} ref_desc:#{@b_ref_desc} dynamic:#{@b_dynamic}"
    (indent+1).times { print "  " }
    if @signature then
      puts "signature: #{@signature.get_name} #{@signature}"
    else
      puts "signature: NOT defined"
    end
    if @array_size == "[]" then
      (indent+1).times { print "  " }
      puts "array_size: not specified"
    elsif @array_size then
      (indent+1).times { print "  " }
      puts "array_size: #{@array_size}"
    end
    if @allocator_instance then
      (indent+1).times { print "  " }
      puts "allocator instance:"
      @allocator_instance.each { |b,a|
        (indent+2).times { print "  " }
        puts "#{a[0]} #{a[1]} #{b} "
        # a[3].show_tree( indent+3 )
      }
    end
    (indent+1).times { print "  " }
    if @port_type == :CALL then
      puts "VMT_useless : #{@b_VMT_useless}  skelton_useless : #{@b_skelton_useless}  cell_unique : #{@b_cell_unique}"
    else
      puts "VMT_useless : #{@b_VMT_useless}  skelton_useless : #{@b_skelton_useless}"
    end
  end

end

#== Namespace
#
# root namespace だけ、Region クラスのインスタンスとして生成される
# root namespace は、root region を兼ねるため
#
# @cell_list は Region の場合にのみ持つ (mikan @cell_list 関連は Region に移すべき)
#
class Namespace < NSBDNode
# @name::  Symbol     # root の場合 "::" (String)
# @global_name:: str
# @name_list:: NamedList   Signature,Celltype,CompositeCelltype,Cell,Typedef,Namespace
# @struct_tag_list:: NamedList : StructType
# @namespace_list:: Namespace[] : Region は Namespace の子クラスであり、含まれる
# @signature_list:: Sginature[]
# @celltype_list:: Celltype[]
# @compositecelltype_list:: CompositeCelltype[]
# @cell_list:: Cell[]
# @typedef_list:: Typedef[]
# @decl_list:: ( Typedef | StructType | EnumType )[]   依存関係がある場合に備えて、順番どおりに配列に格納 mikan enum
# @const_decl_list:: Decl[]
# @cache_n_cells:: Integer :  get_n_cells の結果をキャッシュする
# @cache_generating_region:: Region :  get_n_cells の結果をキャッシュするしているリージョン

  # mikan namespace の push, pop

  # namespace 階層用のスタック
  @@namespace_stack = []      # @@namespace_stack[0] = "::" (generator.rb)
  @@namespace_sp = -1

  # Generator ネスト用のスタック (namespace 階層用のスタックを対比する)
  @@nest_stack_index = -1
  @@nest_stack = []

  @@root_namespace = nil

  # Generator ネスト用スタックの push, pop (クラスメソッド)
  def self.push
    dbgPrint "push Namespace\n"
    @@nest_stack_index += 1
    @@nest_stack[ @@nest_stack_index ] = [ @@namespace_stack, @@namespace_sp ]
    if @@root_namespace then
      @@namespace_sp = 0
      @@namespace_stack[ @@namespace_sp ] = @@root_namespace
    end
  end

  def self.pop
    dbgPrint "pop Namespace\n"
    @@namespace_stack, @@namespace_sp = @@nest_stack[ @@nest_stack_index ]
    @@nest_stack_index -= 1
    if @@nest_stack_index < -1 then
      raise "TooManyRestore"
    end
  end

  # namespace 階層用スタックの push, pop (インスタンスメソッド)
  def push ns
    @@namespace_sp += 1
    @@namespace_stack[ @@namespace_sp ] = self
    dbgPrint "Namespace.PUSH #{@@namespace_sp} #{@name}\n"
  end

  def pop
    dbgPrint "Namespace.POP #{@@namespace_sp} #{@name}\n"
    @@namespace_sp -= 1
    if @@namespace_sp < 0 then
      raise "StackUnderflow"
    end
  end

  def initialize( name )

    super()
    @name = name

    if( name == "::" )then
      if( @@root_namespace != nil )then
        # root は一回のみ生成できる
        raise "try to re-create root namespace"
      end
      @@root_namespace = self
      @NamespacePath = NamespacePath.new( name, true )
    else
      ns = @@namespace_stack[ @@namespace_sp ].find( name )
      if ns.kind_of? Namespace then
        dbgPrint "namespace: re-appear #{@name}\n"
        # 登録済み namespace の再登録
        ns.push ns
        return
      elsif ns then
        cdl_error( "S1151 $1: not namespace", @name )
        prev_locale = ns.get_locale
        puts "previous: #{prev_locale[0]}: line #{prev_locale[1]} \'#{name}\' defined here"
      end
      dbgPrint "namespace: 1st-appear #{@name}\n"
    end

    if @@namespace_sp >= 0 then   # root は除外
      @@namespace_stack[@@namespace_sp].new_namespace( self )
    end
    push self

    @global_name = Namespace.get_global_name    # stack 登録後取る
    @name_list = NamedList.new( nil, "symbol in namespace '#{@name}'" )
    @struct_tag_list = NamedList.new( nil, "struct tag" )

    @namespace_list = []
    @signature_list = []
    @celltype_list = []
    @compositecelltype_list = []
    @cell_list = []
    @typedef_list = []
    @decl_list = []
    @const_decl_list = []
    @cache_n_cells = nil
    @cache_generating_region = nil
    if @NamespacePath == nil then
      # root namespace の場合は設定済 (親 namespace が見つからず例外になる)
      set_namespace_path # @NamespacePath の設定
    end
  end

  def end_of_parse
    pop
  end

  def get_name
    @name
  end

  #=== Namespace:: global_name を得る
  # parse 中のみこのメソッドは使える
  # STAGE: P
  def self.get_global_name    # parse 中有効
    if @@namespace_sp <= 0 then
      return ""
    end

    path = @@namespace_stack[1].get_name.to_s
    i = 2
    while i <= @@namespace_sp
      path = path+"_"+@@namespace_stack[i].get_name.to_s
      i += 1
    end

    path
  end

  def get_global_name
    @global_name
  end

  #=== Namespace#セルの個数を得る
  # 子 region が linkunit, node 指定されていれば、含めない（別のリンク単位）
  # プロトタイプ宣言のもののみの個数を含めない
  # mikan namespace 下に cell を置けない仕様になると、このメソッドは Region のものでよい
  # mikan 上記の場合 instance_of? Namespace の条件判定は不要となる
  def get_n_cells
    if @cache_generating_region == $generating_region then
      # このメソッドは繰り返し呼び出されるため、結果をキャッシュする
      return @cache_n_cells
    end

    count = 0
    @cell_list.each{ |c|
      # 定義かプロトタイプ宣言だけかは、new_cell の段階で判断できないため、カウントしなおす
      if c.get_f_def == true then
        # print "get_n_cells: cell: #{c.get_name}\n"
        count += 1
      end
    }

    @namespace_list.each{ |ns|
      if ns.instance_of? Namespace then
        count += ns.get_n_cells
      else
        # ns は Region である
        rt = ns.get_region_type
        # print "get_n_cells: region: #{ns.get_name}: #{rt}\n"
        if rt == :NODE || rt == :LINKUNIT then
          # 別の linkunit なので加算しない
        else
          count += ns.get_n_cells
        end
      end
    }

    @cache_generating_region = $generating_region
    @cache_n_cells = count
    return count
  end

  #=== Namespace.find : in_path で示されるオブジェクトを探す
  #in_path:: NamespacePath
  #in_path:: Array : 古い形式
  #  path [ "::", "ns1", "ns2" ]   absolute
  #  path [ "ns1", "ns2" ]         relative
  def self.find( in_path )

    if in_path.instance_of? Array then
      # raise "Namespace.find: old fashion"

      path = in_path
      length = path.length
      return self.find_one( path[0] ) if length == 1

      name = path[0]
      if name == "::" then
        i = 1
        name = path[i]   # 構文的に必ず存在
        object = @@root_namespace.find( name )  # root
      else
        # 相対パス
        i = 0
        object = @@namespace_stack[@@namespace_sp].find_one( name ) # crrent
      end

    elsif in_path.instance_of? NamespacePath then
      path = in_path.get_path
      length = path.length

      if length == 0 then
        if in_path.is_absolute? then
          return @@root_namespace
        else
          raise "path length 0, not absolute"
        end
      end

      i = 0
      name = path[0]
      if in_path.is_absolute? then
        object = @@root_namespace.find( name )  # root
      else
        bns = in_path.get_base_namespace
        object = bns.find_one( name )           # crrent
      end
    else
      raise "unexpected path"
    end

    i += 1
    while i < length

      unless object.kind_of?( Namespace ) then
        # クラスメソッド内で cdl_error を呼び出すことはできない
        # また、前方参照対応後、正確な行番号が出ない問題も生じる
        # cdl_error( "S1092 \'$1\' not namespace" , name )
        # このメソッドから nil が帰った場合 "not found" が出るので、ここでは出さない
        return nil
      end

      object = object.find( path[i] )
      i += 1
    end

    return object
  end


  def find( name )
    @name_list.get_item(name)
  end

  #=== Namespace# namespace から探す。見つからなければ親 namespace から探す
  def self.find_one( name )
    return @@namespace_stack[@@namespace_sp].find_one( name )
  end

  def find_one( name )

    object = find( name )
    # これは出すぎ
    # dbgPrint "in '#{@name}' find '#{name}' object #{object ? object.class : "Not found"}\n"

    if object != nil then
      return object
    elsif @name != "::" then
      return @owner.find_one( name )
    else
      return nil
    end
  end

  def self.get_current
    @@namespace_stack[@@namespace_sp]
  end

  def self.find_tag( name )
    # mikan tag : namespace の path に対応しない
    # namespace の中にあっても、root namespace にあるものと見なされる
    # よって カレント namespace から根に向かって探す
    i = @@namespace_sp
    while i >= 0
      res = @@namespace_stack[i].find_tag( name )
      if res then
        return res
      end
      i -= 1
    end
  end

  def find_tag( name )
    @struct_tag_list.get_item( name )
  end

 ### namespace
  def self.new_namespace( namespace )
    @@namespace_stack[@@namespace_sp].new_namespace( namespace )
  end

  def new_namespace( namespace )
    dbgPrint "new_namespace: #{@name}:#{self} #{namespace.get_name}:#{namespace} \n"
    namespace.set_owner self   # Namespace (Namespace)

    @name_list.add_item( namespace )
    @namespace_list << namespace
  end

 ### signature
  def self.new_signature( signature )
    @@namespace_stack[@@namespace_sp].new_signature( signature )
  end

  def new_signature( signature )
    signature.set_owner self   # Signature (Namespace)
    @name_list.add_item( signature )
    @signature_list << signature
  end

 ### celltype
  def self.new_celltype( celltype )
    @@namespace_stack[@@namespace_sp].new_celltype( celltype )
  end

  def new_celltype( celltype )
    celltype.set_owner self   # Celltype (Namespace)
    @name_list.add_item( celltype )
    @celltype_list << celltype
  end

 ### compositecelltype
  def self.new_compositecelltype( compositecelltype )
    @@namespace_stack[@@namespace_sp].new_compositecelltype( compositecelltype )
  end

  def new_compositecelltype( compositecelltype )
    compositecelltype.set_owner self   # CompositeCelltype (Namespace)
    @name_list.add_item( compositecelltype )
    @compositecelltype_list << compositecelltype
  end

 ### cell (Namespace)
  def self.new_cell( cell )
    @@namespace_stack[@@namespace_sp].new_cell( cell )
  end

  def new_cell( cell )
    dbgPrint "Namespace.new_cell: #{@NamespacePath.get_path_str}::#{cell.get_name}\n"
    if ! is_root? && ! ( instance_of? Region ) then
      cdl_error( "S9999 '$1' cell cannot be placed under namespace", cell.get_name )
    end
    cell.set_owner self   # Cell (Namespace)
    @name_list.add_item( cell )
    @cell_list << cell
  end

  #=== Namespace# 参照されているが、未定義のセルを探す
  # プロトタイプ宣言だけで定義されていないケースをエラーとする
  # 受動の未結合セルについて警告する
  def check_ref_but_undef
    @cell_list.each { |c|
      if ! c.get_f_def then   # Namespace の @cell_list にはプロトタイプが含まれるケースあり
        if c.get_f_ref then
          cdl_error( "S1093 $1 : undefined cell" , c.get_namespace_path.get_path_str )
        elsif $verbose then
          cdl_warning( "W1006 $1 : only prototype, unused and undefined cell" , c.get_namespace_path.get_path_str )
        end
      else
        dbgPrint "check_ref_but_undef: #{c.get_global_name}\n"
        ct = c.get_celltype
        # if c.get_f_ref == false && c.is_generate? && ct && ct.is_inactive? then
        if c.get_f_ref == false && ct && ct.is_inactive? then
          cdl_warning( "W1007 $1 : non-active cell has no entry join and no factory" , c.get_namespace_path.get_path_str )
        end
      end
    }
    @namespace_list.each { |n|
      n.check_ref_but_undef
    }
  end

  #=== Namespace# セルの受け口の参照カウントを設定する
  def set_port_reference_count
    @cell_list.each { |c|
      c.set_port_reference_count
    }
    @namespace_list.each { |n|
      n.set_port_reference_count
    }
  end

 ### struct
  def self.new_structtype( struct )
    @@namespace_stack[@@namespace_sp].new_structtype( struct )
  end

  def new_structtype( struct )
    # struct.set_owner self   # StructType (Namespace) # StructType は BDNode ではない
    dup = @struct_tag_list.get_item(struct.get_name)
    if dup != nil then
      if struct.same? dup then
        # 同じものが typedef された
        # p "#{struct.get_name}"
        return
      end
    end

    @struct_tag_list.add_item( struct )
    @decl_list << struct
  end

 ### typedef
  def self.new_typedef( typedef )
    @@namespace_stack[@@namespace_sp].new_typedef( typedef )
  end

  def new_typedef( typedef )
    typedef.set_owner self   # TypeDef (Namespace)
    dup = @name_list.get_item(typedef.get_name)
    if dup != nil then
      typedef_type = typedef.get_declarator.get_type.get_original_type
      dup_type = dup.get_declarator.get_type.get_original_type
      # print "typedef: #{typedef.get_name} = #{typedef_type.get_type_str} #{typedef_type.get_type_str_post}\n"
      if typedef_type.get_type_str == dup_type.get_type_str &&
          typedef_type.get_type_str_post == dup_type.get_type_str_post then
        # 同じものが typedef された
        # ここへ来るのは C で関数ポインタを typedef しているケース
        # 以下のように二重に定義されている場合は type_specifier_qualifier_list として扱われる
        #    typedef long LONG; 
        #    typedef long LONG;
        # bnf.y.rb では declarator に TYPE_NAME を許さないので、ここへ来ることはない
        # p "#{typedef.get_declarator.get_type.get_type_str} #{typedef.get_name} #{typedef.get_declarator.get_type.get_type_str_post}"
        return
      end
      # p "prev: #{dup.get_declarator.get_type.get_type_str}#{dup.get_declarator.get_type.get_type_str_post} current:#{typedef.get_declarator.get_type.get_type_str} #{typedef.get_declarator.get_type.get_type_str_post}"
    end

    # p "typedef: #{typedef.get_name}  #{typedef.get_declarator.get_type.get_original_type.get_type_str}#{typedef.get_declarator.get_type.get_original_type.get_type_str_post}"
    # typedef.show_tree 0

    @name_list.add_item( typedef )
    @typedef_list << typedef
    @decl_list << typedef
  end

  def self.is_typename?( str )
    i = @@namespace_sp
    while i >= 0
      if @@namespace_stack[i].is_typename?( str ) then
        return true
      end
      i -= 1
    end
    false
  end

  def is_typename?( str )
    if @name_list.get_item( str ).instance_of?( Typedef ) then
      true
    else
      false
    end
  end

 ### const_decl
  def self.new_const_decl( decl )
    @@namespace_stack[@@namespace_sp].new_const_decl( decl )
  end

  def new_const_decl( decl )
    decl.set_owner self   # Decl (Namespace:const)
    if ! decl.is_const? then			# const 修飾さていること
      if decl.is_type?( PtrType ) then
        cdl_error( "S1094 $1: pointer is not constant. check \'const\'" , decl.get_name )
      else
        cdl_error( "S1095 $1: not constant" , decl.get_name )
      end
    elsif ! decl.is_type?( IntType ) && ! decl.is_type?( FloatType ) &&
        ! decl.is_type?( BoolType ) && ! decl.is_type?( PtrType ) then
                                            # IntType, FloatType であること
      cdl_error( "S1096 $1: should be int, float, bool or pointer type" , decl.get_name )
    elsif decl.get_initializer == nil then   # 初期値を持つこと
      cdl_error( "S1097 $1: has no initializer" , decl.get_name )
#    elsif decl.get_initializer.eval_const(nil) == nil then  #eval_const は check_init で呼出されるので二重チェック
#                                            # mikan 初期値が型に対し適切であること
#      cdl_error( "S1098 $1: has unsuitable initializer" , decl.get_name )
    else
      decl.get_type.check_init( @locale, decl.get_name, decl.get_initializer, :CONSTANT )
      @name_list.add_item( decl )
      @const_decl_list << decl
    end

  end

 ### region
  # def self.new_region( region )
  #   @@namespace_stack[@@namespace_sp].new_region( region )
  # end
# 
  # def new_region( region )
  #   region.set_owner self   # Rgion (Namespace)
  #   @name_list.add_item( region )
  # end

 ###

  #=== Namespace# すべてのセルの require ポートを設定
  # STAGE: S
  def set_require_join
    @celltype_list.each{ |ct|
      ct.set_require_join
    }
    # すべての namespace について require ポートをセット
    @namespace_list.each{ |ns|
      ns.set_require_join
    }
  end

  #=== Namespace# Join への definition の設定とチェック
  # セルタイプに属するすべてのセルに対して実施
  def set_definition_join
    # celltype のコードを生成
    @cell_list.each { |c|
      dbgPrint "set_definition_join #{c.get_name}\n"
      c.set_definition_join
    }
    @namespace_list.each{ |ns|
      ns.set_definition_join
    }
  end

  #=== Namespace# set_max_entry_port_inner_cell
  # セルタイプに属するすべてのセルに対して実施
  def set_max_entry_port_inner_cell
    # celltype のコードを生成
    @cell_list.each { |c|
      c.set_max_entry_port_inner_cell
    }
    @namespace_list.each{ |ns|
      ns.set_max_entry_port_inner_cell
    }
  end

  #=== Namespace# セルの結合をチェックする
  def check_join
    @cell_list.each { |c|
      dbgPrint "check_join #{c.get_name}\n"
      c.check_join
      c.check_reverse_require
    }
    @namespace_list.each{ |ns|
      ns.check_join
    }
  end

  #== Namespace# ルートか?
  # ルートネームスペース と ルートリージョンは同じ
  def is_root?
    @name == "::"
  end

  #== Namespace# ルートを得る
  # ルートリージョンとルートネームスペースは同じオブジェクト
  def self.get_root
    @@root_namespace
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "#{self.class}: name: #{@name} path: #{get_namespace_path.get_path_str}"
    @struct_tag_list.show_tree( indent + 1 )
    @name_list.show_tree( indent + 1 )
  end

end


class Join < BDNode
# @name:: string
# @subscript:: nil: not array, -1: subscript not specified, >=0: array_subscript
# @rhs:: Expression | initializer ( array of Expression | initializer (Expression | C_EXP) )
# @definition:: Port, Decl(attribute or var)
#
# available if definition is Port
# @cell_name:: string : 右辺のセルの名前
# @cell:: Cell  : 右辺のセル
# @celltype:: Celltype : 右辺のセルタイプ
# @port_name:: string : 右辺の受け口名
# @port:: Port : 右辺の受け口
# @array_member:: rhs array : available only for first appear in the same name
# @array_member2:: Join array : available only for first appear in the same name
# @rhs_subscript:: nil : not array, >=0: 右辺の添数
#

# @through_list::  @cp_through_list + @region_through_list
#  以下の構造を持つ（@cp_through_list の構造は共通）
# @cp_through_list::  呼び口に指定された through
#   [ [plugin_name, cell_name, plugin_arg], [plugin_name2, cell_name2, plugin_arg], ... ]
# @region_through_list::  region に指定された through
#   [ [plugin_name, cell_name, plugin_arg, region], [plugin_name2, cell_name2, plugin_arg, region2], ... ]
#
# @through_generated_list:: [Plugin_class object, ...]: @through_list に対応
# @region_through_generated_list:: [Plugin_class object, ...]: @region_through_list に対応
#

  include PluginModule

  #=== Join# 初期化
  #name:: string: 名前（属性名、呼び口名）
  #subscript:: Nil=非配列, -1="[]", N="[N]"
  #rhs:: Expression: 右辺の式
  def initialize( name, subscript, rhs, locale = nil )
    # dbgPrint "Join#new: #{name}, #{subscript} #{rhs.eval_const(nil)}\n"
    dbgPrint "Join#new: #{name}, #{subscript}\n"

    super()
    if locale then
      @locale = locale
    end

    @name = name
    if subscript.instance_of?( Expression ) then
       #mikan 配列添数が整数であることを未チェック
       @subscript = subscript.eval_const(nil)
       if @subscript == nil then
         cdl_error( "S1099 array subscript not constant"  )
       end
    else
       @subscript = subscript
    end

    @rhs = rhs
    @definition = nil

    # 配列要素を設定
    # 本当は、初出の要素のみ設定するのが適当
    # new_join で add_array_member の中で初出要素の array_member に対し設定する
    if @subscript == -1 then
      @array_member  = [self]
      @array_member2 = [self]
    elsif @subscript != nil then
      @array_member = []
      @array_member2 = []
      @array_member[@subscript]  = self
      @array_member2[@subscript] = self
    end

    @through_list = []
    @cp_through_list = []
    @region_through_list = []
    @through_generated_list = []
    @region_through_generated_list = []
  end

  #===  Join# 左辺に対応する celltype の定義を設定するとともにチェックする
  # STAGE:   S
  #
  #     代入可能かチェックする
  #definition:: Decl (attribute,varの時) または Port (callの時) または nil (definition が見つからなかった時)

  def set_definition( definition )

    dbgPrint "set_definition: #{@owner.get_name}.#{@name} = #{definition.class}\n"

    # 二重チェックの防止
    if @definition then
      # set_definition を個別に行うケースで、二重に行われる可能性がある（異常ではない）
      # 二重に set_definition が実行されると through が二重に適用されてしまう
      # cdl_warning( "W9999 $1, internal error: set_definition duplicate", @name )
      return
    end

    @definition = definition

    # mikan 左辺値、右辺値の型チェックなど
    if @definition.instance_of?( Decl ) then
      check_var_init
    elsif @definition.instance_of?( Port ) then
      check_call_port_init
      if @definition.get_port_type == :CALL then   # :ENTRY ならエラー。無視しない
        check_and_gen_through
        create_allocator_join  # through プラグイン生成した後でないと、挿入前のセルのアロケータを結合してしまう
      end
    elsif @definition == nil then
      cdl_error( "S1117 \'$1\' not in celltype", @name )
    else
      raise "UnknownToken"
    end
  end

  #=== Join# 変数の初期化チェック
  def check_var_init
    # attribute, var の場合
    if @definition.get_kind == :ATTRIBUTE then
#        check_cell_cb_init( definition.get_type, @rhs )
      # 右辺で初期化可能かチェック
      @definition.get_type.check_init( @locale, @definition.get_identifier, @rhs, :ATTRIBUTE )
    elsif @definition.get_kind == :VAR then
      # var は初期化できない
      cdl_error( "S1100 $1: cannot initialize var" , @name )
    else
      # Bug trap
      raise "UnknownDeclKind"
    end
  end

  #=== Join# 呼び口の初期化チェック
  def check_call_port_init
    ### Port

    # 左辺は受け口か（受け口を初期化しようとしている）？
    if @definition.get_port_type == :ENTRY then
      cdl_error( "S1101 \'$1\' cannot initialize entry port" , @name )
      return
    end

#      # 配列添数の整合性チェック
#      # 呼び口の定義で、非配列なら添数なし、添数なし配列なら添数なし、添数あり配列なら添数あり
    as = @definition.get_array_size
    if ( @subscript == nil && as != nil ) then
      cdl_error( "S1102 $1: must specify array subscript here" , @name )
    elsif ( @subscript != nil && as == nil ) then
      cdl_error( "S1103 $1: cannot specify array subscript here" , @name )
    end
#    if @subscript == nil then
#      if as != nil then
#        cdl_error( "S1103 $1: need array subscript" , @name )
#      end
#    elsif @subscript == -1 then
#      if as != "[]" then
#        cdl_error( "S1104 $1: need array subscript number. ex. \'[0]\'" , @name )
#      end
#    else # @subscript >0
#      if as == nil then
#        cdl_error( "S1105 $1: cannot specify array subscript here" , @name )
#      elsif as == "[]" then
#        cdl_error( "S1106 $1: cannot specify array subscript number. use \'[]\'" , @name )
#      end
#    end

    # mikan Expression の get_type で型導出させる方がスマート
    # mikan '=' の左辺が配列かどうか未チェック
    #(1) '=' の右辺は "Cell.ePort" の形式か？
    #     演算子は "."  かつ "." の左辺が :IDENTIFIER
    #     "." の右辺はチェック不要 (synatax 的に :IDENTIFIER)
    #(2) "Cell" は存在するか？（名前が一致するものはあるか）
    #(3) "Cell" は cell か？
    #(4) "Cell" の celltype は有効か？ (無効なら既にエラー）
    #(5) "ePort" は "Cell" の celltype 内に存在するか？
    #(6) "ePort" は entry port か？
    #(7) signature は一致するか

    # 右辺がない（以前の段階でエラー）
    return unless @rhs

    # cCall = composite.cCall; のチェック．この形式は属性用
    # 呼び口を export するには cCall => composite.cCall; の形式を用いる
    if @rhs.instance_of?( Array ) == true && @rhs[0] == :COMPOSITE then
      cdl_error( "S1107 to export port, use \'cCall => composite.cCall\'"  )
      return
    elsif ! @rhs.instance_of?( Expression ) then
      raise "Unknown bug. specify -t to find problem in source"
    end

    # 右辺の Expression の要素を取り出す
    ret = @rhs.analyze_cell_join_expression
    if ret == nil then   #1
      cdl_error( "S1108 $1: rhs not \'Cell.ePort\' form" , @name )
      return
    end

    nsp, @rhs_subscript, @port_name = ret[0], ret[1], ret[2]
    @cell_name = nsp.get_name     # mikan ns::cellname の形式の考慮

    # composite の定義の中なら object は結合先 cell か、見つからなければ nil が返る
    # composite の定義外なら false が返る
    object = CompositeCelltype.find( @cell_name )
    if object == false then
#     mikan 左辺が namespace に対応していないため。 path にして find
      # p nsp.get_path_str, nsp.get_path
      object = Namespace.find( nsp )    #1
      in_composite = false
    else
      if nsp.get_path.length != 1 then
        cdl_error( "$1 cannot have path", nsp.get_path_str )
      end
      in_composite = true
    end

    if object == nil then                                             # (2)
      cdl_error( "S1109 \'$1\' not found" , nsp.to_s )
    elsif ! object.instance_of?( Cell ) then                          # (3)
      cdl_error( "S1110 \'$1\' not cell" , nsp.to_s )
    else
      dbgPrint "set_definition: set_f_ref #{@owner.get_name}.#{@name} => #{object.get_name}\n"
      object.set_f_ref

      # 右辺のセルのセルタイプ
      celltype = object.get_celltype

      if celltype then                                                # (4)
        object2 = celltype.find( @port_name )
        if object2 == nil then                                        # (5)
          cdl_error( "S1111 \'$1\' not found" , @port_name )
        elsif ! object2.instance_of? Port \
             || object2.get_port_type != :ENTRY then                  # (6)
          cdl_error( "S1112 \'$1\' not entry port" , @port_name )
        elsif @definition.get_signature != object2.get_signature then # (7)
          cdl_error( "S1113 \'$1\' signature mismatch" , @port_name )
        elsif object2.get_array_size then
          # 受け口配列

          unless @rhs_subscript then
            # 右辺に添数指定がなかった
            cdl_error( "S1114 \'$1\' should be array" , @port_name )
          else

            as = object2.get_array_size
            if( as.kind_of?( Integer ) && as <= @rhs_subscript )then
              # 受け口配列の大きさに対し、右辺の添数が同じか大きい
              cdl_error( "S1115 $1[$2]: subscript out of range (< $3)" , @port_name, @rhs_subscript, as )
            else
              dbgPrint "Join OK #{@owner.get_name}.#{@name}[#{@rhs_subscript}] = #{object.get_name}.#{@port_name} #{self}\n"
              @cell = object
              @celltype = celltype
              @port = object2
              # 右辺のセルの受け口 object2 を参照済みにする
              # object2: Port, @definition: Port
              @cell.set_entry_port_max_subscript( @port, @rhs_subscript )
            end

            # debug
            dbgPrint "Join set_definition: rhs: #{@cell}  #{@cell.get_name if @cell}\n"

          end
        elsif @rhs_subscript then
          # 受け口配列でないのに右辺で添数指定されている
          cdl_error( "S1116 \'$1\' entry port is not array" , @port_name )
        else
          dbgPrint "Join OK #{@owner.get_name}.#{@name} = #{object.get_name}.#{@port_name} #{self}\n"
          @cell = object
          @port = object2
          @celltype = celltype

          # 右辺のセル object の受け口 object2 を参照済みにする
          # object2: Port, @definition: Port

          # debug
          # p "rhs:  #{@cell}  #{@cell.get_name}"
        end  # end of port (object2) チェック

        #else
        #  celltype == nil (すでにエラー)
      end  # end of celltyep チェック


      check_region( object )

    end  # end of cell (object) チェック

  end

  #=== Join# アロケータの結合を生成
  # STAGE: S
  #cell::  呼び口の結合先のセル
  #
  # ここでは呼び口側に生成されるアロケータ呼び口の結合を生成
  # 受け口側は Cell の set_specifier_list で生成
  #  a[*] の内容は Cell の set_specifier_list を参照
  def create_allocator_join

    cell = get_rhs_cell2   # 右辺のセルを得る
    port = get_rhs_port2

    if( cell && cell.get_allocator_list ) then      # cell == nil なら既にエラー

      dbgPrint "create_allocator_join: #{@owner.get_name}.#{@name}=>#{cell ? cell.get_name : "nil"}\n"

      cell.get_allocator_list.each { |a|

        if( a[0+1] == port && a[1+1] == @rhs_subscript )then
          # 名前の一致するものの結合を生成する
          # 過不足は、別途チェックされる
          cp_name = :"#{@name}_#{a[2+1]}_#{a[3+1]}"
          # p "creating allocator join #{cp_name} #{@subscript} #{a[1+1]}"
          join = Join.new( cp_name, @subscript, a[4+1], @locale )

          #debug
          dbgPrint "create_allocator_join: #{@owner.get_name}.#{cp_name} [#{@subscript}] #{@name}\n"
          @owner.new_join join
        else
          dbgPrint "create_allocator_join:3 not #{@owner.get_name}.#{a[0+1]} #{@name}\n"
        end
      }
    end
  end

  #=== Join# リージョン間の結合をチェック
  # リージョン間の through による @region_through_list の作成
  # 実際の生成は check_and_gen_through で行う
  # mikan Cell#distance とRegion へたどり着くまでための処理に共通部分が多い
  def check_region( object )

    #debug
    dbgPrint "check_region #{@owner.get_name}.#{@name} => #{object.get_name}\n"
    # print "DOMAIN: check_region #{@owner.get_name}.#{@name} => #{object.get_name}\n"

    # プラグインで生成されたなかでは生成しない
    # さもないとプラグイン生成されたものとの間で、無限に生成される
##    if Generator.get_nest >= 1 then
##    if Generator.get_plugin then     # mikan これは必要？ (意味解析段階での実行になるので不適切)
    if @owner.get_plugin.kind_of?( ThroughPlugin ) then
      # プラグイン生成されたセルの場合、結合チェックのみ
      return
    end

    # region のチェック
    r1 = @owner.get_region      # 呼び口セルの region
    r2 = object.get_region      # 受け口セルの region

    if ! r1.equal? r2 then      # 同一 region なら呼出し可能

      f1 = r1.get_family_line
      len1 = f1.length
      f2 = r2.get_family_line
      len2 = f2.length

      # 不一致になるところ（兄弟）を探す
      i = 1  # i = 0 は :RootRegion なので必ず一致
      while( i < len1 && i < len2 )
        if( f1[i] != f2[i] )then
          break
        end
        i += 1
      end

      sibling_level = i     # 兄弟となるレベル、もしくはどちらか一方が終わったレベル

      dbgPrint "sibling_level: #{i}\n"
      dbgPrint "from: #{f1[i].get_name}\n" if f1[i]
      dbgPrint "to: #{f2[i].get_name}\n" if f2[i]

      if f1[sibling_level] && f2[sibling_level] then
        b_to_through = true
      else
        b_to_through = false
      end


      # 呼び側について呼び元のレベルから兄弟レベルまで（out_through をチェックおよび挿入）
      i = len1 -1
      if b_to_through then
        end_level = sibling_level
      else
        end_level = sibling_level - 1
      end
      while i > end_level
      # while i > sibling_level
      # while i >= sibling_level
        dbgPrint "going out from #{f1[i].get_name} level=#{i}\n"
        region_count = f1[i].next_out_through_count
        out_through_list = f1[i].get_out_through_list   # [ plugin_name, plugin_arg ]
        domain = f1[i].get_domain_type
        if domain then
          domain_through = f1[i].get_domain_type.add_through_plugin( self, f1[i], f1[i-1], :OUT_THROUGH )
          if domain_through == nil then
            cdl_error( "S9999 $1: going out from regin '$2' not permitted by domain '$3'" , @name, f1[i].get_name, f1[i].get_domain_type.get_name )
          end
        elsif out_through_list.length == 0 then
          cdl_error( "S1118 $1: going out from region \'$2\' not permitted" , @name, f1[i].get_name )
        end

        out_through_list.each { |ol|
          if ol[0] then    # plugin_name が指定されていなければ登録しない
            plugin_arg = CDLString.remove_dquote ol[1]
            through = [ ol[0], :"Join_out_through_", plugin_arg, f1[i], f1[i-1], :OUT_THROUGH, region_count]
            @region_through_list << through
          end
        }
        if domain_through && domain_through.length > 0 then
          through = [ domain_through[0], :"Join_domain_out_through_", domain_through[1], f1[i], f1[i-1], :OUT_THROUGH, region_count ]
          @region_through_list << through
        end
        i -= 1
      end

      # 兄弟レベルにおいて（to_through をチェックおよび挿入）
      if f1[sibling_level] && f2[sibling_level] then
        dbgPrint "going from #{f1[sibling_level].get_name} to #{f2[sibling_level].get_name}\n"
        found = 0
        region_count = f1[i].next_to_through_count( f2[sibling_level].get_name )   # to_through の region カウント
        f1[sibling_level].get_to_through_list.each { |t|
          if t[0][0] == f2[sibling_level].get_name then   # region 名が一致するか ?
            if t[1] then    # plugin_name が指定されていなければ登録しない
              plugin_arg = CDLString.remove_dquote t[2]
              through = [ t[1], :"Join_to_through__", plugin_arg, f1[sibling_level], f2[sibling_level], :TO_THROUGH, region_count ]
              @region_through_list << through
            end
            found = 1
          end
        }
        domain = f1[sibling_level].get_domain_type
        if domain then
          domain_through = f1[sibling_level].get_domain_type.add_through_plugin( self, f1[sibling_level], f2[sibling_level], :TO_THROUGH )
          if domain_through == nil then
            cdl_error( "S9999 $1: going from regin '$2' not permitted by domain'$3'" , @name, f1[sibling_level].get_name, f2[sibling_level].get_domain_type.get_name )
          end
          if domain_through && domain_through.length > 0 then
            through = [ domain_through[0], :"Join_domain_to_through_", domain_through[1], f1[sibling_level], f2[sibling_level], :TO_THROUGH, region_count ]
            @region_through_list << through
          end
        elsif found == 0 then
          cdl_error( "S1119 $1: going from region \'$2\' to \'$3\' not permitted" , @name, f1[sibling_level].get_name, f2[sibling_level].get_name )
        end
      end

      # 受け側について兄弟レベルから受け側のレベルまで（in_through をチェックおよび挿入）
      if b_to_through then
        i = sibling_level + 1      # to_through を経た場合、最初の in_through は適用しない
      else
        i = sibling_level
      end
      while i < len2
        dbgPrint "going in to #{f2[i].get_name} level=#{i}\n"
        region_count = f2[i].next_in_through_count
        in_through_list = f2[i].get_in_through_list   # [ plugin_name, plugin_arg ]
        domain = f2[i].get_domain_type
        if domain then
          domain_through = f2[i].get_domain_type.add_through_plugin( self, f2[i-1], f2[i], :IN_THROUGH )
          if domain_through == nil then
            cdl_error( "S9999 $1: going in from regin '$2' to '$3' not permitted by domain '$4'",
                        @name, f2[i-1].get_name, f2[i].get_name, f2[i].get_domain_type.get_name )
          end
          if domain_through && domain_through.length > 0 then
            through = [ domain_through[0], :"Join_domain_in_through_", domain_through[1], f2[i-1], f2[i], :IN_THROUGH, region_count ]
            @region_through_list << through
          end
        elsif in_through_list.length == 0 then
          cdl_error( "S1120 $1: going in to region \'$2\' not permitted" , @name, f2[i].get_name )
        end
        in_through_list.each { |il|
          if il[0] then    # plugin_name が指定されていなければ登録しない
            plugin_arg = CDLString.remove_dquote il[1]
            through = [ il[0], :"Join_in_through_", plugin_arg, f2[i-1], f2[i],:IN_THROUGH, region_count ]
            @region_through_list << through
          end
        }
        i += 1
      end

    end
  end


  #=== Join# 生成しないリージョンへの結合かチェック
  # 右辺のセルが、生成されないリージョンにあればエラー
  # 右辺は、プラグイン生成されたセルがあれば、それを対象とする
  def check_region2
    lhs_cell = @owner

    # 生成しないリージョンのセルへの結合か？
    # if join.get_cell && ! join.get_cell.is_generate? then
    # if get_rhs_cell && ! get_rhs_cell.is_generate? then # composite セルがプロタイプ宣言の場合例外
    # print "Link root: (caller #{@owner.get_name}) '#{@owner.get_region.get_link_root.get_name}'"
    # print " #{@owner.get_region.get_link_root == get_rhs_region.get_link_root ? "==" : "!="} "
    # print "'#{get_rhs_region.get_link_root.get_name}'  (callee #{@cell_name})\n"

    if get_rhs_region then
      dbgPrint "check_region2 #{lhs_cell.get_name} => #{get_rhs_region.get_path_string}#{@rhs.to_s}\n"

      # if get_rhs_region.is_generate? != true then  #3
      if @owner.get_region.get_link_root != get_rhs_region.get_link_root then
        cdl_error( "S1121 \'$1\' in region \'$2\' cannot be directly joined $3 in  $4" , lhs_cell.get_name, lhs_cell.get_region.get_namespace_path.get_path_str, @rhs.to_s, get_rhs_region.get_namespace_path.get_path_str )
      end
    else
      # rhs のセルが存在しなかった (既にエラー)
    end
  end

  def get_definition
    @definition
  end

  #=== Join# specifier を設定
  # STAGE: B
  # set_specifier_list は、join の解析の最後で呼び出される
  # through 指定子を設定
  #  check_and_gen_through を呼出して、through 生成
  def set_specifier_list( specifier_list )

    specifier_list.each { |s|
      case s[0]
      when :THROUGH
        # set plugin_name
        plugin_name = s[1].to_s
        plugin_name[0] = "#{plugin_name[/^./].upcase}"     # 先頭文字を大文字に : ruby のクラス名の制約

        # set cell_name
        cell_name = :"#{s[1].to_s}_"

        # set plugin_arg
        plugin_arg = CDLString.remove_dquote s[2].to_s
        # plugin_arg = s[2].to_s.gsub( /\A"(.*)/, '\1' )   # 前後の "" を取り除く
        # plugin_arg.sub!( /(.*)"\z/, '\1' )

        @cp_through_list << [ plugin_name, cell_name, plugin_arg ]
      end
    }

  end

  #=== Join# through のチェックと生成
  # new_join の中の check_region で region 間の through が @region_through に設定される
  # set_specifier で呼び口の結合で指定された through が @cp_through 設定される
  # その後、このメソッドが呼ばれる
  def check_and_gen_through

    dbgPrint "check_and_gen_through #{@owner.get_name}.#{@name}\n"

    if ! @definition.instance_of? Port then
      cdl_error( "S1123 $1 : not port: \'through\' can be specified only for port" , @name )
      return
    end
    if @cp_through_list.length > 0 then
      # is_empty? must check before is_omit?
      if @definition.get_signature && @definition.get_signature.is_empty? then
        cdl_warning( "W9999 'through' is specified for empty signature, ignored"  )
        return
      elsif @definition.is_omit? then
        cdl_warning( "W9999 'through' is specified for omitted port, ignored"  )
        return
      end
    end

    @through_list = @cp_through_list + @region_through_list
      # 後から @cp_through_list と @region_through_list に分けたため、このような実装になった

    if @through_list then           # nil when the join is not Port
      len = @through_list.length    # through が連接している数
    else
      len = 0
    end
    cp_len = @cp_through_list.length

    if @owner.is_in_composite? && len > 0 then
      cdl_error( "S1177 cannot specify 'through' in composite in current version" )
      return
    end

    # 連続した through について、受け口側から順にセルを生成し解釈する
    i = len - 1
    while i >= 0

      through = @through_list[ i ]
      plugin_name           = through[ 0 ]
      generating_cell_name  = through[ 1 ]
      plugin_arg            = through[ 2 ]

      if i != len - 1 then

        begin
          next_cell_nsp       = @through_generated_list[ i + 1 ].get_cell_namespace_path
          next_port_name      = @through_generated_list[ i + 1 ].get_through_entry_port_name
        rescue Exception => evar
          cdl_error( "S1124 $1: plugin function failed: \'get_through_entry_port_name\'" , plugin_name )
          print_exception( evar )
          i -= 1
          next
        end

        next_cell = Namespace.find( next_cell_nsp )    #1
        if next_cell == nil then
          # p "next_cell_path: #{next_cell_nsp.get_path_str}"
          cdl_error( "S1125 $1: not generated cell \'$2\'" , @through_generated_list[ i + 1 ].class, next_cell_nsp.get_path_str )
          return
        end

      else
        # 最後のセルの場合、次のセルの名前、ポート名
        next_cell      = @cell
        next_port_name = @port_name

        if next_cell == nil then
          # 結合先がない
          return
        end
      end

      if i >= cp_len then
        # region_through_list 部分
        # region から @cell_name.@port_name への through がないか探す
        # rp = @through_list[i][3].find_cell_port_through_plugin( @cell_name, @port_name ) #762
        rp = @through_list[i][3].find_cell_port_through_plugin( @cell.get_global_name, @port_name )
           # @through_list[i] と @region_through_list[i-cp_len] は同じ
        # 共用しないようにするには、見つからなかったことにすればよい
        # rp = nil
      else
        # region 以外のものは共有しない
        # 呼び口側に指定されているし、plugin_arg が異なるかもしれない
        rp = nil
      end

      if rp == nil then
        plClass = load_plugin( plugin_name, ThroughPlugin )
        if( plClass ) then
          gen_through_cell_code_and_parse( plugin_name, i, next_cell, next_port_name, plClass )
        end
      else
        # 見つかったものを共用する
        @through_generated_list[ i ] = rp
      end

      if i >= cp_len then
        # @through_generated_list のうち @region_through_listに対応する部分
        @region_through_generated_list[ i - cp_len ] = @through_generated_list[ i ]
        if rp == nil then
          # 生成したものを region(@through_list[i][3]) のリストに追加
          # @through_list[i][3].add_cell_port_through_plugin( @cell_name, @port_name, @through_generated_list[i] ) #762
          @through_list[i][3].add_cell_port_through_plugin( @cell.get_global_name, @port_name, @through_generated_list[i] )
        end
      end

      if i == 0 then
        # 最も呼び口側のセルは、CDL 上の結合がないため、参照されたことにならない
        # mikan namespace 対応
        # cell = Namespace.find( [ @through_generated_list[0].get_cell_name] )    #1
        if @through_generated_list[0] == nil then
          return  # plugin_object の生成に失敗している
        end
        cell = Namespace.find( @through_generated_list[0].get_cell_namespace_path )    #1
        if cell.instance_of? Cell then
          cell.set_f_ref
        end
      end

      i -= 1
    end

  end

  @@through_count = { }
  def get_through_count name
    sym = name.to_sym
    if @@through_count[ sym ] then
      @@through_count[ sym ] += 1
    else
      @@through_count[ sym ] = 0
    end
    return @@through_count[ sym ]
  end

  #=== Join# through プラグインを呼び出して CDL 生成させるとともに、import する
  def gen_through_cell_code_and_parse( plugin_name, i, next_cell, next_port_name, plClass )

    through = @through_list[ i ]
    plugin_name           = through[ 0 ]
    generating_cell_name  = :"#{through[ 1 ]}_#{get_through_count through[ 1 ]}"
    plugin_arg            = through[ 2 ]
    if through[ 3 ] then
      # region 間の through の場合
      @@start_region      = through[ 3 ]
      if next_cell.get_region.equal? @@start_region then
        @@end_region      = @@start_region
      else
        @@end_region      = through[ 4 ]
      end
      @@through_type      = through[ 5 ]
      @@region_count      = through[ 6 ]
    else
      # 呼び口の through の場合
      @@start_region      = @owner.get_region    # 呼び口側セルの region
      @@end_region        = next_cell.get_region # 次のセルの region
      @@through_type      = :THROUGH             # 呼び口の through 指定
      @@region_count      = 0
    end
    @@plugin_creating_join = self
    caller_cell = @owner

    begin
      plugin_object = plClass.new( "#{generating_cell_name}".to_sym, plugin_arg.to_s, next_cell, "#{next_port_name}".to_sym, @definition.get_signature, @celltype, caller_cell )
      plugin_object.set_locale @locale
    rescue Exception => evar
      cdl_error( "S1126 $1: fail to new" , plugin_name )
      if @celltype && @definition.get_signature && caller_cell && next_cell then
        print "signature: #{@definition.get_signature.get_name} from: #{caller_cell.get_name} to: #{next_cell.get_name} of celltype: #{@celltype.get_name}\n"
      end
      print_exception( evar )
      return 
    end

    @through_generated_list[ i ] = plugin_object

    # Region に関する情報を設定
    # 後から追加したので、new の引数外で設定
    # plugin_object.set_through_info( start_region, end_region, through_type )

    generate_and_parse plugin_object
  end

  #プラグインへの引数で渡さないものを、一時的に記憶しておく
  # プラグインの initialize の中でコールバックして設定する
  @@plugin_creating_join = nil
  @@start_region = nil
  @@end_region = nil
  @@through_type = nil
  @@region_count = nil

  #=== Join# ThroughPlugin の追加情報を設定する
  # このメソッドは ThroughPlugin#initialize から呼び出される
  # plugin_object を生成する際の引数では不足する情報を追加する
  def self.set_through_info plugin_object
    plugin_object.set_through_info( @@start_region, @@end_region, @@through_type,
                                    @@plugin_creating_join,
                                    @@plugin_creating_join.get_cell,
                                    @@region_count )
  end

  def get_name
    @name
  end

  #=== Join#配列添数を得る
  # @subscript の説明を参照のこと
  def get_subscript
    @subscript
  end

  def get_cell_name         # 受け口セル名
    @cell_name
  end

  def get_celltype
    @celltype
  end

  def get_cell
    @cell
  end

  #=== Join# 右辺の実セルを得る
  #    実セルとは through で挿入されたもの、composite の内部など実際に結合される先
  #    このメソッドは　get_rhs_port と対になっている
  #    このメソッドは、意味解析段階では呼び出してはならない (対象セルの意味解析が済む前には正しい結果を返さない)
  def get_rhs_cell
    # through 指定あり？
    if @through_list[0] then
      # mikan through で生成したものが root namespace 限定
      if @through_generated_list[0] then
        # cell = Namespace.find( [ "::", @through_generated_list[0].get_cell_name.to_sym ] )    #1
        cell = Namespace.find( @through_generated_list[0].get_cell_namespace_path )    #1
        # cell が nil になるのはプラグインの get_cell_namespace_path が正しくないか、
        # プラグイン生成コードがエラーになっている。
        # できの悪いプラグインが多ければ、cell == nil をはじいた方がよい。
        return cell.get_real_cell( @through_generated_list[0].get_through_entry_port_name )
      else
        return nil            # generate に失敗している
      end
    elsif @cell then
      return @cell.get_real_cell( @port_name )
    else
      # 右辺が未定義の場合 @cell は nil (既にエラー)
      return nil
    end
  end

  #=== Join# 右辺のセルを得る
  # 右辺のセルを得る。ただし、composite 展開されていない
  # composite 展開されたものを得るには get_rhs_cell を使う
  # プロトタイプ宣言しかされていない場合には、こちらしか使えない
  # このメソッドは get_rhs_port2 と対になっている
  def get_rhs_cell2
    # through 指定あり？
    if @through_list[0] then
      # mikan through で生成したものが root namespace 限定
      # cell = Namespace.find( [ "::", @through_generated_list[0].get_cell_name ] )
      if @through_generated_list[0] then
        # cell = Namespace.find( [ @through_generated_list[0].get_cell_name ] )    #1
        cell = Namespace.find( @through_generated_list[0].get_cell_namespace_path )    #1
      else
        cell = @cell            # generate に失敗している
      end
    else
      cell = @cell
    end

    return cell
  end

  #=== Join# 右辺のセルを得る
  # through は適用しないが、composite は展開した後のセル
  # (意味解析が終わっていないと、composite 展開が終わっていない)
  # このメソッドは get_rhs_port3 と対になっている
  def get_rhs_cell3
    if @cell then
      return @cell.get_real_cell( @port_name )
    end
  end

  #=== Join# 右辺のセルのリージョンを得る
  # 右辺が未定義の場合、nil を返す
  # composite の場合、実セルではなく composite cell の region を返す(composite はすべて同じ region に属する)
  # composite の cell がプロトタイプ宣言されているとき get_rhs_cell/get_real_cell は ruby の例外となる
  def get_rhs_region
    # through 指定あり？
    if @through_list[0] then
      if @through_generated_list[0] then
        # mikan through で生成したものが root namespace 限定
        # cell = Namespace.find( [ "::", @through_generated_list[0].get_cell_name.to_sym ] )    #1
        cell = Namespace.find( @through_generated_list[0].get_cell_namespace_path )    #1
        if cell then
          return cell.get_region
        end
      else
        return nil       # generate に失敗している
      end
    elsif @cell then
      return @cell.get_region
    end
    # 右辺が未定義の場合 @cell は nil (既にエラー)
    return nil
  end

  def get_cell_global_name  # 受け口セル名（コンポジットなら展開した内側のセル）

    # debug
    dbgPrint "cell get_cell_global_name:  #{@cell_name}\n"
    # @cell.show_tree( 1 )

    if @cell then
      return @cell.get_real_global_name( @port_name )
    else
      return "NonDefinedCell?"
    end

  end

  #===  Join# 結合の右辺の受け口の名前
  #     namespace 名 + '_' + セル名 + '_' + 受け口名   （このセルが composite ならば展開後のセル名、受け口名）
  #subscript:: Integer  呼び口配列の時添数 または nil 呼び口配列でない時
  def get_port_global_name( subscript = nil )  # 受け口名（コンポジットなら展開した内側のセル）

    # debug
    dbgPrint "Cell get_port_global_name:  #{@cell_name}\n"

    # through 指定あり？
    if @through_list[0] then

      # mikan through で生成したものが root namespace 限定
      # cell = Namespace.find( [ "::", @through_generated_list[0].get_cell_name.to_sym ] )    #1
      cell = Namespace.find( @through_generated_list[0].get_cell_namespace_path )    #1

      # through で挿入されたセルで、実際に接続されるセル（compositeの場合内部の)の受け口の C 言語名前
      return cell.get_real_global_port_name( @through_generated_list[0].get_through_entry_port_name )
    else

      # 実際に接続されるセルの受け口の C 言語名前
      if @cell then
        return @cell.get_real_global_port_name( @port_name )
      else
        return "UndefinedCellsPort?"
      end

    end

  end

  def get_port_name
    @port_name
  end

  def get_rhs
    @rhs
  end

  def get_rhs_subscript
    @rhs_subscript
  end

  #=== Join# 右辺のポートを得る
  #    右辺が composite の場合は、内部の繋がるセルのポート, through の場合は挿入されたセルのポート
  #    このメソッドは get_rhs_cell と対になっている
  def get_rhs_port
    # through 指定あり？
    if @through_list[0] then
      # mikan through で生成したものが root namespace 限定
      # through で生成されたセルを探す
#      cell = Namespace.find( [ "::", @through_generated_list[0].get_cell_name.to_sym ] )    #1
      cell = Namespace.find( @through_generated_list[0].get_cell_namespace_path )    #1
      # cell のプラグインで生成されたポート名のポートを探す (composite なら内部の繋がるポート)
      return cell.get_real_port( @through_generated_list[0].get_through_entry_port_name )
    else
      # ポートを返す(composite なら内部の繋がるポートを返す)
      return @cell.get_real_port( @port_name )
    end
  end

  #=== Join# 右辺のポートを得る
  # 右辺のポートを得る。
  # これはプロトタイプ宣言しかされていない場合には、こちらしか使えない
  def get_rhs_port2
    # through 指定あり？
    if @through_list[0] then
      if @through_generated_list[0] then
        port = @through_generated_list[0].get_through_entry_port_name.to_sym
      else
        port = @port_name    # generate に失敗している
      end
    else
      port = @port_name
    end

    return port
  end

  #=== Join# 右辺のポートを得る
  # through は適用しないが、composite は展開した後のセルの対応するポート
  def get_rhs_port3
    if @cell then
      return @cell.get_real_port( @port_name )
    end
  end

  #=== Join# 呼び口配列の2番目以降の要素を追加する
  #     一番最初に定義された配列要素が全要素の初期値の配列を持つ
  #     このメソッドは非配列の場合も呼出される（join 重複エラーの場合）
  #join2:: Join  呼び口配列要素の Join
  def add_array_member join2

    # subscript2: join2 の左辺添数
    subscript2 = join2.get_subscript

    if @subscript == nil then		# not array : initialize duplicate
      # 非配列の場合、join が重複している
      cdl_error( "S1127 \'$1\' duplicate", @name )
      # print "add_array_member2: #{@owner.get_name}\n"

    elsif @subscript >= 0 then
      # 添数指定ありの場合
      if( subscript2 == nil || subscript2 < 0 ) then
        # join2 左辺は非配列または添数なし
        # 配列が不一致
        cdl_error( "S1128 \'$1\' inconsistent array definition", @name )
      elsif @array_member[subscript2] != nil then
        # 同じ添数が既に定義済み
        cdl_error( "S1129 \'$1\' redefinition of subscript $2" ,@name, subscript2 )
      else
        # 添数の位置に要素を追加
        @array_member[subscript2] = join2.get_rhs
        @array_member2[subscript2] = join2
#        p "0:#{join2.get_rhs}"
      end

    else
      # 添数指定なしの場合
      if( subscript2 == nil || subscript2 >= 0 ) then
        # join2 左辺は非配列または添数有
        # 配列が不一致
        cdl_error( "S1130 \'R1\' inconsistent array definition", @name )
      end

      # 添数なし配列の場合、配列要素を追加
      @array_member  << join2.get_rhs
      @array_member2 << join2
    end
  end

  def get_array_member
    @array_member
  end

  def get_array_member2
    @array_member2
  end

  def change_name name
    # debug
    dbgPrint "change_name: #{@name} to #{name}\n"

    @name = name

    if @array_member2 then
      i = 0
      while i < @array_member2.length
        if @array_member2[i] != self && @array_member[i] != nil then
          # @array_member2[i] が nil になるのは optional の時と、
          # Join の initialize で無駄に @array_member2 が設定されている場合
          # 無駄に設定されているものについては、再帰的に呼び出す必要はない（clone_for_composite では対策している）
          @array_member2[i].change_name( name )
        end
        i += 1
      end
    end
  end

  # composite cell を展開したセルの結合を clone したセルの名前に変更
  def change_rhs_port( clone_cell_list, celltype )
    dbgPrint "change_rhs_port: name=#{@name}\n"

    # debug
    if $debug then
#    if @name == :cCallB then
      # dbgPrint "change_rhs name: #{@name}  cell_name: #{@cell_name} #{@cell} #{self}\n"
      print "============\n"
      print "CHANGE_RHS change_rhs name: #{@owner.get_name}.#{@name}  rhs cell_name: #{@cell_name} #{@cell} #{self}\n"

      clone_cell_list.each{ |cell, ce|
        # dbgPrint "=== change_rhs:  #{cell.get_name}=#{cell} : #{ce.get_name}\n"
        print "   CHANGE_RHS  change_rhs:  #{cell.get_name}=#{cell} : #{ce.get_name}\n"
      }
      print "============\n"
    end

    c = clone_cell_list[@cell]
    return if c == nil

    # debug
    dbgPrint "  REWRITE cell_name:  #{@owner.get_name}   #{@cell_name} => #{c.get_global_name}, #{c.get_name}\n"

    # @rhs の内容を調整しておく（この内容は、subscript を除いて、後から使われていない）
    elements = @rhs.get_elements
    if elements[0] == :OP_SUBSC then  # 右辺：受け口配列？
      elements  = elements[1]
    end

    # 右辺が　cell.ePort の形式でない
    if elements[0] != :OP_DOT || elements[1][0] != :IDENTIFIER then   #1
      return
    else
      # セル名を composite 内部の名前から、外部の名前に入れ替える
      # elements[1][1] = Token.new( c.get_name, nil, nil, nil )
      elements[1][1] = NamespacePath.new( c.get_name, false, c.get_namespace )
    end

    @cell_name = c.get_name
    @cell = c
    # @definition = nil          # @definition が有効： チェック済み（とは、しない）

    if @array_member2 then

      # debug
      dbgPrint "array_member2.len : #{@array_member.length}\n"

      i = 0
      while i < @array_member2.length
        # @array_member2[i] が nil になるのは optional の時と、
        # Join の initialize で無駄に @array_member2 が設定されている場合
        # 無駄に設定されているものについては、再帰的に呼び出す必要はない（clone_for_composite では対策している）
        if @array_member2[i] != self && @array_member[i] != nil then
          dbgPrint "change_rhs array_member #{i}: #{@name}  #{@cell_name}\n"
          @array_member2[i].change_rhs_port( clone_cell_list, celltype )
        end
        i += 1
      end
    end

  end

  #=== Join# composite セル用にクローン
  #cell_global_name:: string : 親セルのグローバル名
  # 右辺の C_EXP に含まれる $id$, $cell$, $ct$ を置換
  # ここで置換するのは composite の attribute の C_EXP を composite セルタイプおよびセル名に置換するため
  # （内部セルの C_EXP もここで置換される）
  # @through_list などもコピーされるので、これが呼び出される前に確定する必要がある
  def clone_for_composite( ct_name, cell_name, locale, b_need_recursive = true )
    # debug
    dbgPrint "=====  clone_for_composite: #{@name} #{@cell_name} #{self}   =====\n"
    cl = self.clone

    if @array_member2 && b_need_recursive then
      cl.clone_array_member( ct_name, cell_name, self, locale )
    end

    rhs = CDLInitializer.clone_for_composite( @rhs, ct_name, cell_name, locale )
    cl.change_rhs rhs

    # debug
    dbgPrint "join cloned : #{cl}\n"
    return cl
  end

  def clone_array_member( ct_name, cell_name, prev, locale )
    # 配列のコピーを作る
    am  = @array_member.clone
    am2 = @array_member2.clone

    # 配列要素のコピーを作る
    i = 0
    while i < am2.length
      if @array_member2[i] == prev then
        # 自分自身である（ので、呼出すと無限再帰呼出しとなる）
        am2[i] = self
        am[i] = am2[i].get_rhs
      elsif @array_member2[i] then
#        am2[i] = @array_member2[i].clone_for_composite( ct_name, cell_name, locale, false )
        am2[i] = @array_member2[i].clone_for_composite( ct_name, cell_name, locale, true )
        am[i] = am2[i].get_rhs
      else
        # 以前のエラーで array_member2[i] は nil になっている
      end

      # debug
      dbgPrint "clone_array_member: #{@name} subsript=#{i} #{am2[i]} #{@array_member2[i]}\n"

      i += 1
    end

    # i = 0 は、ここで自分自身を設定
    # am2[0] = self

    @array_member  = am
    @array_member2 = am2

  end

  #=== Join# rhs を入れ換える
  #rhs:: Expression | initializer
  # 右辺を入れ換える．
  # このメソッドは、composite で cell の属性の初期値を attribute の値で置き換えるのに使われる
  # このメソッドは composite 内の cell の属性の初期値が定数ではなく式になった場合、不要になる
  def change_rhs rhs
    @rhs = rhs
  end

  #=== Join# clone された join の owner を変更
  def set_cloned( owner )
    dbgPrint "Join#set_cloned: #{@name}  prev owner: #{@owner.get_name} new owner: #{owner.get_name}\n"
    @owner = owner
    if @array_member2 then
      @array_member2.each{ |join|
        dbgPrint "Joinarray#set_cloned: #{@name}  prev owner: #{join.get_owner.get_name} new owner: #{owner.get_name}\n"
        join.set_owner owner
      }
    end
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "Join: name: #{@name} owner: #{@owner.get_name} id: #{self}"
    if @subscript == nil then
    elsif @subscript >= 0 then
      (indent+1).times { print "  " }
      puts "subscript: #{@subscript}"
    else
      (indent+1).times { print "  " }
      puts "subscript: not specified"
    end
    (indent+1).times { print "  " }
    puts "rhs: "
    if @rhs.instance_of?( Array )then
      @rhs.each{ |i|
        if i.instance_of?( Array )then
          i.each{ |j|
            j.show_tree( indent + 3 )
          }
        elsif i.instance_of? Symbol then
          (indent+2).times { print "  " }
          print i
          print "\n"
        else
          i.show_tree( indent + 2 )
        end
      }
    else
      @rhs.show_tree( indent + 2 )
      (indent+1).times { print "  " }
      if @definition then
        puts "definition:"
        @definition.show_tree( indent + 2 )
      else
        puts "definition: not found"
      end
    end
    if @definition.instance_of?( Port ) then
      (indent+2).times { print "  " }
      if @cell then
        puts "cell: #{@cell_name} #{@cell}  port: #{@port_name}  cell_global_name: #{@cell.get_global_name}"
      else
        puts "cell: #{@cell_name} port: #{@port_name}  (cell not found)"
      end
    end
    if @through_list then
      i = 0
      @through_list.each { |t|
        (indent+2).times { print "  " }
        puts "through: plugin name :  '#{t[0]}' arg : '#{t[2]}'"
        if @through_generated_list[i] then
          @through_generated_list[i].show_tree( indent+3 )
        end
        i += 1
      }
    end
    if @array_member2 then
      (indent+1).times { print "  " }
      puts "array member:"
      i = 0
      @array_member2.each { |j|
        if j then
          (indent+2).times { print "  " }
          puts "[#{i}]: #{j.get_name}  id: #{j} owner=#{j.get_owner.get_name}"
          j.get_rhs.show_tree(indent+3)
#          (indent+3).times { print "  " }
#          puts "cell global name: #{j.get_cell_global_name}"
#          puts "cell global name: #{j.get_rhs_cell.get_global_name}"
#          (indent+3).times { print "  " }
#          puts "port global name: #{j.get_port_global_name}"
#          puts "port global name: #{j.get_rhs_port.get_name}"
        else
          (indent+2).times { print "  " }
          puts "[#{i}]: [optional]  id: #{j}"
        end
        i += 1
      }
    end
  end

end

#== 逆結合
class ReverseJoin < BDNode
#@ep_name:: Symbol
#@ep_subscript:: Expression or nil
#@cell_nsp: NamespacePath
#@cp_name:: Symbol
#@cp_subscript:: Expression or nil
  def initialize( ep_name, ep_subscript, cell_nsp, cp_name, cp_subscript = nil )
    super()
    @ep_name = ep_name
    @ep_subscript = ep_subscript
    @cell_nsp = cell_nsp
    @cp_name = cp_name
    @cp_subscript = cp_subscript
  end

  def get_name
    @ep_name
  end

  def get_rhs_cell_and_port
    [ @ep_subscript, @cell_nsp, @cp_name, @cp_subscript ]
  end
end

# CLASS: CompositeCelltype 用の Join
# REM:   CompositeCelltype が export するもの
class CompositeCelltypeJoin < BDNode
# @export_name:: string     :  CompositeCelltype が export する名前（呼び口、受け口、属性）
# @internal_cell_name:: string : CompositeCelltype 内部のセルの名前
# @internal_cell_elem_name:: string : CompositeCelltype 内部のセルの呼び口、受け口、属性の名前
# @cell : Cell : Cell::  internal cell  : CompositeCelltyep 内部のセル（in_compositeセル）
# @port_decl:: Port | Decl
# @b_pseudo: bool : 

  def initialize( export_name, internal_cell_name,
		 internal_cell_elem_name, cell, port_decl )
    super()
    @export_name = export_name
    @internal_cell_name = internal_cell_name
    @internal_cell_elem_name = internal_cell_elem_name
    @cell = cell
    @port_decl = port_decl

  end

  #=== CompositeCelltypeJoin# CompositeCelltypeJoin の対象セルか？
  #cell::  Cell 対象かどうかチェックするセル
  #
  #     CompositeCelltypeJoin と cell の名前が一致するかチェックする
  #     port_decl が指定された場合は、現状使われていない
  def match?( cell, port_decl = nil )

    #debug
    if port_decl
      dbgPrint(  "match?"  )
      dbgPrintf( "  @cell:      %-20s      %08x\n", @cell.get_name, @cell.object_id )
      dbgPrintf( "  @port_decl: %-20s      %08x\n", @port_decl.get_name, @port_decl.object_id )
      dbgPrintf( "  cell:       %-20s      %08x\n", cell.get_name, cell.object_id )
      dbgPrintf( "  port_decl:  %-20s      %08x\n", port_decl.get_name, port_decl.object_id )
      dbgPrint(  "  cell_name: #{cell.get_name.class}=#{cell.get_name} cell_elem_name: #{port_decl.get_name.class}=#{port_decl.get_name}\n" )
      dbgPrint(  "  @cell_name: #{@cell.get_name.class}=#{@cell.get_name} cell_elem_name: #{@port_decl.get_name.class}=#{@port_decl.get_name}\n" )

    end

#    if @cell.equal?( cell ) && ( port_decl == nil || @port_decl.equal?( port_decl ) ) then
    # なぜ port_decl が一致しなければならなかったか忘れた。
    # recursive_composite で名前の一致に変更   060917
    if((@cell.get_name == cell.get_name) && (port_decl == nil || @port_decl.get_name == port_decl.get_name))then
      true
    else
      false
    end
  end

  def check_dup_init
    return if get_port_type != :CALL

    if @cell.get_join_list.get_item @internal_cell_elem_name then
      cdl_error( "S1131 \'$1.$2\' has duplicate initializer" , @internal_cell_name, @internal_cell_elem_name )
    end
  end

  def get_name
    @export_name
  end

  def get_cell_name
    @internal_cell_name
  end

  def get_cell
    @cell
  end

  def get_cell_elem_name
    @internal_cell_elem_name
  end

  # @port_decl が Port の場合のみ呼び出してよい
  def get_port_type
    if @port_decl then
      @port_decl.get_port_type
    end
  end

  def get_port_decl
    @port_decl
  end

  #=== CompositeCelltypeJoin#get_allocator_instance
  def get_allocator_instance
    if @port_decl.instance_of? Port then
      return @port_decl.get_allocator_instance
    elsif @port_decl
      raise "CompositeCelltypeJoin#get_allocator_instance: not port"
    else
      return nil
    end
  end

  # @port_decl が Port の場合のみ呼び出してよい
  def is_require?
    if @port_decl then
      @port_decl.is_require?
    end
  end

  # @port_decl が Port の場合のみ呼び出してよい
  def is_allocator_port?
    if @port_decl then
      @port_decl.is_allocator_port?
    end
  end

  # @port_decl が Port の場合のみ呼び出してよい
  def is_optional?
    if @port_decl then
      @port_decl.is_optional?
    end
  end

  #=== CompositeCelltypeJoin# 右辺が Decl ならば初期化子（式）を返す
  # このメソッドは Cell の check_join から初期値チェックのために呼び出される
  def get_initializer
    if @port_decl.instance_of? Decl then
      @port_decl.get_initializer
    end
  end

  def get_size_is
    if @port_decl.instance_of? Decl then
      @port_decl.get_size_is
    end
  end

  #=== CompositeCelltypeJoin# 配列サイズを得る
  #RETURN:: nil: not array, "[]": 大きさ指定なし, Integer: 大きさ指定あり
  def get_array_size
    @port_decl.get_array_size
  end

  #=== CompositeCelltypeJoin# signature を得る
  # @port_decl が Port の時のみ呼び出してもよい
  def get_signature
    @port_decl.get_signature
  end

  #=== CompositeCelltypeJoin# get_type
  def get_type
    if @port_decl.instance_of? Decl
      @port_decl.get_type
    end
  end

  #=== CompositeCelltypeJoin# get_initializer
  def get_initializer
    if @port_decl.instance_of? Decl
      @port_decl.get_initializer
    end
  end

  #=== CompositeCelltypeJoin# get_choice_list
  def get_choice_list
    if @port_decl.instance_of? Decl
      @port_decl.get_choice_list
    end
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "CompositeCelltypeJoin: export_name: #{@export_name} #{self}"
    (indent+1).times { print "  " }
    puts "internal_cell_name: #{@internal_cell_name}"
    (indent+1).times { print "  " }
    puts "internal_cell_elem_name: #{@internal_cell_elem_name}"
    if @port_decl then
      @port_decl.show_tree( indent + 1 )
    end
  end
end

class Factory < BDNode
# @name:: string
# @file_name:: string
# @format:: string
# @arg_list:: Expression の elements と同じ形式 [ [:IDENTIFIER, String], ... ]
# @f_celltype:: bool : true: celltype factory, false: cell factory

  @@f_celltype = false

  def initialize( name, file_name, format, arg_list )
    super()
    @f_celltype = @@f_celltype

    case name
    when :write
      # write 関数
      @name = name

      # write 関数の第一引数：出力先ファイル名
        # 式を評価する（通常単一の文字列であるから、単一の文字列が返される）
      @file_name = file_name.eval_const(nil).val  # file_name : Expression
      if ! @file_name.instance_of?( String ) then
        # 文字列定数ではなかった
        cdl_error( "S1132 $1: 1st parameter is not string(file name)" , @name )
        @file_name = nil
      end

      # write 関数の第二引数：フォーマット文字列
      @format    = format.eval_const(nil).val     # format : Expression
        # 式を評価する（通常単一の文字列であるから、単一の文字列が返される）
      if ! @format.instance_of?( String ) then
        # 文字列定数ではなかった
        cdl_error( "S1133 $1: 2nd parameter is not string(fromat)" , @name )
        @format = nil
      end

      # 第三引数以降を引数リストとする mikan 引数のチェック
      @arg_list = arg_list

    else
      cdl_error( "S1134 $1: unknown factory function" , name )
    end
    Celltype.new_factory( self )
  end

  def check_arg( celltype )
    if ! @arg_list then
      return
    end

    if @f_celltype then
      cdl_error( "S1135 celltype factory can\'t have parameter(s)"  )
      return
    end

    @arg_list.each{ |elements|

      case elements[0]
      when :IDENTIFIER  #1
        obj = celltype.find( elements[1] )
        if obj == nil then
          cdl_error( "S1136 \'$1\': not found" , elements[1] )
        elsif ! obj.instance_of?( Decl ) || obj.get_kind != :ATTRIBUTE then
          cdl_error( "S1137 \'$1\': not attribute" , elements[1] )
        end
      when :STRING_LITERAL
      else
        cdl_error( "S1138 internal error Factory.check_arg()"  )
      end

    }
  end

  def self.set_f_celltype( f_celltype )
    @@f_celltype = f_celltype
  end

  def get_f_celltype
    @f_celltype
  end

  def get_name
    @name
  end

  def get_file_name
    @file_name
  end

  def get_format
    @format
  end

  def get_arg_list
    @arg_list
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "Factory: name: #{@name}"
    if @arg_list then
      (indent+1).times { print "  " }
      puts "argument(s):"
      @arg_list.each { |l|
        (indent+2).times { print "  " }
        print "\"#{l}\"\n"
      }
    end
  end
end

#== Domain
#
# region の domain を記憶するクラス
class DomainType < Node
#@name::Symbol : ドメインタイプの名前 ex) HRP2
#@region::Region
#@plugin_name::Symbol : ex) HRP2Plugin
#@option::String : ex) "trusted", "nontrusted"
#@plugin::DomainPlugin の子クラス

  include PluginModule

  # ドメインに属する region の Hash
  # domain 指定が一度も行われない場合、このリストは空である
  # ルートリージョンは option = "OutOfDomain" で登録される (domain 指定が無ければ登録されない)
  @@domain_regions = { }  # { :domain_type => [ region, ... ] }

  def initialize( region, name, option )
    super()
    @name = name
    @plugin_name = (name.to_s + "Plugin").to_sym
    plClass = load_plugin( @plugin_name, DomainPlugin )
    @region = region
    @option = option

    if @@domain_regions[ name ] then
      if ! @@domain_regions[ name ].include?( region ) then
        @@domain_regions[ name ] << region
      end
    else
      @@domain_regions[ name ] = [ region ]
    end
  end

  def create_domain_plugin
    if ! @plugin then
      pluginClass = Object.const_get @plugin_name
      return if pluginClass == nil
      @plugin = pluginClass.new( @region, @name, @option )
    end
  end

  def add_through_plugin( join, from_region, to_region, through_type )
    # print( "DOMAIN: add_through_plugin: from=#{from_region.get_name}#{join.get_owner.get_name}.#{join.get_name} to=#{to_region}#{join.get_cell.get_name}.#{join.get_port_name} through_type=#{through_type}\n" )
    return @plugin.add_through_plugin( join, from_region, to_region, through_type )
  end

  def joinable?( from_region, to_region, through_type )
    # print( "DOMAIN: joinable? from_region=#{from_region.get_name} to_region=#{to_region} through_type=#{through_type}\n" )
    return @plugin.joinable?( from_region, to_region, through_type )
  end

  def get_name
    @name
  end

  #== Domain リージョンの Hash を得る
  # @@domain_regions の説明参照
  def self.get_domain_regions
    return @@domain_regions
  end

  def get_regions
    return @@domain_regions[ @name ]
  end

  def get_option
    @option
  end

  def show_tree( indent )
    (indent+1).times { print( "  " ) }
    puts "domain: name=#{@name} plugin=#{@plugin_name} option=#{@option}"
  end
end

#== Region クラス
# 
# Region は Namespace を継承している
# root region は特殊で、root namespace と同じである
#
# cell は region に属する
# region に属する cell のリストは Namespace クラスのインスタンス変数として記憶される
#
class Region < Namespace
# @name:: string
# @in_through_list:: [ [ plugin_name, plugin_arg ], ... ] : plungin_name = nil の時 in 禁止
# @out_through_list:: [ [ plugin_name, plugin_arg ], ... ] : plungin_name = nil の時 out 禁止
# @to_through_list:: [ [ dst_region, plugin_name, plugin_arg ], ... ]
# @cell_port_throug_plugin_list:: { "#{cell_name}.#{port_name}" => through_generated_list の要素 }
#    この region から cell_name.port_name への through プラグインで生成されたオブジェクト
# @region_type::Symbol : :NODE, :LINKUNIT, :DOMAIN, :CLASS
# @region_type_param::Symbol : domain, class の名前. node, linkunit では nil
# @link_root:: Region : linkUnit の根っことなる region (node, linkunit が指定された region)
# @family_line:: [ @region_root, ...,@region_me ]  家系
# @in_through_count:: Integer :  n 番目の in_through 結合 (n>=0)
# @out_through_count:: Integer : n 番目の out_through 結合 (n>=0)
# @to_through_count:: { :RegionName => Integer }: RegionName への n 番目の to_through 結合 (n>=0)
# @domain_type::DomainType : domain 指定されていない場合、nil
# @domain_root::Region : domain 指定されていなる Region (root の場合 nil)

  @@in_through_list  = []
  @@out_through_list = []
  @@to_through_list  = []
  @@region_type = nil
  @@region_type_param = nil
  @@domain_name = nil
  @@domain_option = nil    # Token が入る

  @@link_roots = []

  def initialize( name )
    # mikan name の Namespace 修飾
    # object = Namespace.find( [ name ] )   # 親まで捜しにいく
    if name != "::" then
      object = Namespace.get_current.find( name )    #1
    else
      # root リージョン
      object = nil
      @@region_type = :NODE
    end

    @in_through_list    = @@in_through_list
    @out_through_list   = @@out_through_list
    @to_through_list    = @@to_through_list
    @region_type        = @@region_type
    @region_type_param  = @@region_type_param

    if @@domain_name then
      domain_option = CDLString.remove_dquote @@domain_option.to_s
      @domain_type = DomainType.new( self, @@domain_name, domain_option )
      @@domain_name       = nil
      @@domain_option     = nil
    else
      @domain_type = nil
    end

    @@in_through_list   = []
    @@out_through_list  = []
    @@to_through_list   = []
    @@region_type       = nil
    @@region_type_param = nil

    @in_through_count = -1
    @out_through_count = -1
    @to_through_count = {}

    super( name )
    if object then

      if object.instance_of?( Region ) then
        dbgPrint "Region.new: re-appear #{@name}\n"

        # # Region path が前回出現と一致するか？
        # if @@region_stack[ @@region_stack_sp - 1 ] then
        #   my_path = @@region_stack[ @@region_stack_sp - 1 ].get_path_string.to_s + "." + @name.to_s
        # else
        #   my_path = @name.to_s
        # end
        # if my_path != object.get_path_string then
        #   cdl_error( "S1139 $1: region path mismatch. previous path: $2" , my_path, object.get_path_string )
        # end

        # 再出現
        # @@region_stack[@@region_stack_sp] = object

        # 再出現時に specifier が指定されているか？
        if( @in_through_list.length != 0 || @out_through_list.length != 0 || @to_through_list.length != 0 ||
            @region_type != nil || @domain_type != nil )then
          cdl_error( "S1140 $1: region specifier must place at first appearence" , name )
        end
        return

      else
        # エラー用ダミー定義

        # 異なる同名のオブジェクトが定義済み
        cdl_error( "S1141 $1 duplication, previous one : $2" , name, object.class )
        # @@region_stack[@@region_stack_sp] = self    # エラー時暫定 region
      end
    else
      # 初出現
      dbgPrint "Region.new: #{@name}\n"
      set_region_family_line

      if @region_type == :NODE || @region_type == :LINKUNIT then
        dbgPrint "new LinkRoot: #{@name}\n"
        @@link_roots << self
      end
    end

    @cell_port_throug_plugin_list = {}

# p @name
# p @in_through_list
# p @out_through_list
# p @to_through_list

  end

  def self.end_of_parse
    Namespace.get_current.create_domain_plugin
    Namespace.get_current.end_of_parse
  end

  def self.new_in_through( plugin_name = nil, plugin_arg = nil )
    @@in_through_list << [ plugin_name, plugin_arg ]
  end

  def self.new_out_through( plugin_name = nil, plugin_arg = nil )
    @@out_through_list << [ plugin_name, plugin_arg ]
  end

  def self.new_to_through( dst_region, plugin_name, plugin_arg )
    # p "New to_through #{dst_region}"
    @@to_through_list  << [ dst_region, plugin_name, plugin_arg ]
  end

  def self.set_type( type, param = nil )
    if @@region_type then
      Generator.error( "S1178 $1 region type specifier duplicate, previous $2", type, @@region_type )
    end
    @@region_type = type
    @@region_type_param = param
  end

  def self.set_domain( name, option )
    if @@domain_name then
      Generator.error( "S9999 $1 domain specifier duplicate, previous $2", type, @@region_type )
    end
    @@domain_name = name
    @@domain_option = option
  end

  #== Region ルートリージョンを得る
  # ルートリージョンは、ルートネームスペースと同じである
  def self.get_root
    Namespace.get_root
  end

  def set_region_family_line

    dbgPrint  "set_region_family_line: Region: #{@name}  \n"
    # root namespace (root region) の region type は :NODE
    if @name == "::" then
      @region_type = :NODE
    end

    if @region_type == :NODE || @region_type == :LINKUNIT then
      @link_root = self
    else
      @link_root = @owner.get_link_root
    end

    if @domain_type != nil || @owner == nil then
      @domain_root = self
    else
      @domain_root = @owner.get_domain_root
    end

    if @domain_type then
      # ルートリージョンが最初から @domain_type 設定されることはないの
      # で @owner == nil を調べる必要はない
      @owner.set_domain_type @domain_type
    end

    if @owner then
      @family_line = ( @owner.get_family_line.dup ) << self
    else
      @family_line = [ self ]    # root region
    end

=begin
    @family_line = []
    @link_root = nil

    # @family_line を作成する
    # @link_root もみつける
    # (上位にたどっていって Region で node または linkunit のいずれか先に見つかったものが @link_root となる)
    # root namespace は Region かつ node なので必ず @link_root は見つかる
    # mikan: self が node, linkUnit の場合、ここで期待したとおりに設定されないため、Region#initialize で再設定
    obj = self
    while 1
      if obj.instance_of? Region then
        @family_line << obj
        if @link_root == nil then
          if obj.get_region_type == :NODE || obj.get_region_type == :LINKUNIT then
            @link_root = obj
          end
        end
      else
        # さもなければ Namespace
        # namespace の下に region がある場合
      end

      # root namespace にたどり着けば終り
      break if obj.get_name == "::"

      obj = obj.get_owner
    end
    # print "#{@name}: linkRoot: #{@link_root.get_name}  (this can be wrong if #{@name} is node or linkunit, and corret later\n"
    @family_line.reverse!
=end

  end

  #== Region#ドメインを設定する
  def set_domain_type domain_type
    if @region_type == :NODE then
      if @domain_type then
        if @domain_type.get_name != domain_type.get_name then
          cdl_error( "S9999 '$1' node root cannot belong to both $2 and $3", @name, @domain_type.get_name, domain_type.get_name )
        end
      else
        @domain_type = DomainType.new( self, domain_type.get_name, "OutOfDomain" )
        @domain_type.create_domain_plugin
      end
    elsif @domain_type == nil then
      @owner.set_domain_type domain_type
    end
  end

  def self.get_link_roots
    @@link_roots
  end

  def get_family_line
    @family_line
  end

  def get_in_through_list
    @in_through_list
  end

  def get_out_through_list
    @out_through_list
  end

  def get_to_through_list
    @to_through_list
  end

  def get_link_root
    @link_root
  end

  def get_domain_type
    @domain_type
  end

  #== Region# domain の根っことなる region を得る
  # Region のインスタンスを返す
  # domain 指定子があれば、そのリージョンがドメインルートである
  # なければ、親リージョンのドメインルートとする
  def get_domain_root
    @domain_root
  end

  def get_path_string
    pstring = ""
    delim = ""
    @family_line.each{ |p|
      pstring = "#{pstring}#{delim}#{p.get_name}"
      delim = "."
    }
    dbgPrint "get_path_string: #{pstring}\n"
    pstring
  end

  def get_region_type
    @region_type
  end

  def get_name
    @name
  end

  def next_in_through_count
    @in_through_count += 1
  end

  def next_out_through_count
    @out_through_count += 1
  end

  def next_to_through_count( symRegionName )
    if @to_through_count[ symRegionName ] == nil then
      @to_through_count[ symRegionName ] = 0
    else
      @to_through_count[ symRegionName ] += 1
    end
  end

  #=== Region# 構文解析中の region を得る
  # 構文解析中 Namespace (あるいは子クラスの Region) の上位をたどって Region を見つける
  # cell が namespace 下におくことができなければ、ループをまわす必要はない
  def self.get_current
    # @@region_stack[@@region_stack_sp]
    region = Namespace.get_current
    while 1
      if region.instance_of? Region
        break
      end
      region = region.get_owner
    end
    return region
  end

  #=== Region# through プラグインで、この region から cell_name.port_name へのプラグインオブジェクトを登録
  # mikan namesppace 対応 (cell_name)
  def add_cell_port_through_plugin( cell_name, port_name, through_plugin_object )
    @cell_port_throug_plugin_list[ "#{cell_name}.#{port_name}" ] = through_plugin_object
  end

  def find_cell_port_through_plugin( cell_name, port_name )
    return @cell_port_throug_plugin_list[ "#{cell_name}.#{port_name}" ]
  end

  def create_domain_plugin
    if @domain_type then
      @domain_type.create_domain_plugin
    end
  end

  #=== Region# to_region への距離（unreachable な場合 nil)
  # mikan Cell#check_region とRegion へたどり着くまでための処理に共通性が高い
  # region#distance は require で用いられる
  def distance( to_region )

    r1 = self                   # 出発 region
    r2 = to_region              # 目的 region
    dist = 0

    if ! r1.equal? r2 then      # 同一 region なら呼出し可能

      # mikan namespace 対応
      f1 = r1.get_family_line
      len1 = f1.length
      f2 = r2.get_family_line
      len2 = f2.length

      # 不一致になるところ（兄弟）を探す
      i = 1  # i = 0 は :RootRegion なので必ず一致
      while( i < len1 && i < len2 )
        if( f1[i] != f2[i] )then
          break
        end
        i += 1
      end

      sibling_level = i     # 兄弟となるレベル、もしくはどちらか一方が終わったレベル

      # p "sibling_level: #{i}"
      # p "from: #{f1[i].get_name}" if f1[i]
      # p "to: #{f2[i].get_name}" if f2[i]

      # 呼び側について呼び元のレベルから兄弟レベルまで（out_through をチェックおよび挿入）
      i = len1 -1
      while i >= sibling_level
        dbgPrint "going out from #{f1[i].get_name} level=#{i}\n"
        # print "DOMAIN: going out from #{f1[i].get_name} level=#{i}\n"
        domain = f1[i].get_domain_type
        domain_ok = false
        if domain then
          if ! f1[i].get_domain_type.joinable?( f1[i], f1[i-1], :OUT_THROUGH ) then
            return nil
          end
          domain_ok = true
        end
        if ! domain_ok then
          out_through_list = f1[i].get_out_through_list   # [ plugin_name, plugin_arg ]
          if out_through_list.length == 0 then
            return nil
          end
        end
        i -= 1
        dist += 1
      end

      # 兄弟レベルにおいて（to_through をチェックおよび挿入）
      if f1[sibling_level] && f2[sibling_level] then
        dbgPrint "going from #{f1[sibling_level].get_name} to #{f2[sibling_level].get_name}\n"
        # print "DOMAIN: going from #{f1[sibling_level].get_name} to #{f2[sibling_level].get_name}\n"
        domain = f1[sibling_level].get_domain_type
        domain_ok = false
        if domain then
          if ! f1[i].get_domain_type.joinable?( f1[i], f1[i-1], :TO_THROUGH ) then
            return nil
          end
          domain_ok = true
        end
        if ! domain_ok then
          found = 0
          f1[sibling_level].get_to_through_list.each { |t|
            if t[0][0] == f2[sibling_level].get_name then   # region 名が一致するか ?
              found = 1
            end
          }
          if found == 0 then
            return nil
          end
        end
        dist += 1
      end

      # 受け側について兄弟レベルから受け側のレベルまで（in_through をチェックおよび挿入）
      i = sibling_level
      while i < len2
        dbgPrint "going in to #{f2[i].get_name} level=#{i}\n"
        # print "DOMAIN: going in to #{f2[i].get_name} level=#{i}\n"
        domain = f2[i].get_domain_type
        domain_ok = false
        if domain then
          if ! f2[i].get_domain_type.joinable?( f2[i-1], f2[i], :IN_THROUGH ) then
            return nil
          end
          domain_ok = true
        end
        if ! domain_ok then
          in_through_list = f2[i].get_in_through_list   # [ plugin_name, plugin_arg ]
          if in_through_list.length == 0 then
            return nil
          end
        end
        i += 1
        dist += 1
      end
    end

    dbgPrint "dsitance=#{dist} from #{r1.get_name} to #{r2.get_name}\n"

    return dist
  end

  def show_tree( indent )
    super
    (indent+1).times { print( "  " ) }
    puts "path: #{get_path_string}"
    (indent+1).times { print( "  " ) }
    puts "namespace: #{@namespace ? @namespace.get_name : "nil"}  owner: #{@owner.class}.#{@owner ? @owner.get_name : "nil"}"
    if @domain
      @domain.show_tree( indent+1 )
    end
  end
end

#== Importable class
# this module is included by Import_C and Import
module Importable
#@last_base_dir::String

  #=== Importable#find_file
  #file::String : file name to find
  #return::String | Nil: path to file or nil if not found
  #find file in 
  def find_file file
    $import_path.each{ |path|
      if path == "."
        pt = file
      else
        pt = "#{path}/#{file}"
      end
      if File.exist?( pt )
        if ! $base_dir[ Dir.pwd ]
          $base_dir[ Dir.pwd ] = true
        end
        if $verbose then
          print "#{file} is found in #{path}\n"
        end
        @last_base_dir = nil
        dbgPrint "base_dir=. while searching #{file}\n"
        return pt
      end
    }

    $base_dir.each_key{ |bd|
      $import_path.each{ |path|
#        if path =~ /\A\// || path =~ /\A[a-zA-Z]:/
          pt = "#{path}/#{file}"
#        else
#          pt = "#{bd}/#{path}/#{file}"
#        end
        begin
          Dir.chdir $run_dir
          Dir.chdir bd
          if File.exist?( pt )
            if $verbose then
              print "#{file} is found in #{bd}/#{path}\n"
            end
            @last_base_dir = bd
            dbgPrint "base_dir=#{bd} while searching #{file}\n"
            $base_dir[ bd ] = true
            return pt
          end
        rescue
        end
      }
    }
    @last_base_dir = nil
    dbgPrint "base_dir=. while searching #{file}\n"
    return nil
  end

  def get_base_dir
    return @last_base_dir
    $base_dir.each{ |bd, flag|
      if flag == true
        return bd
      end
    }
    return nil
  end
end

class Import_C < Node

  # ヘッダの名前文字列のリスト
  @@header_list = {}
  @@header_list2 = []
  @@define_list = {}

  include Importable

  #=== Import_C# import_C の生成（ヘッダファイルを取込む）
  #header:: Token : import_C の第一引数文字列リテラルトークン
  #define:: Token : import_C の第二引数文字列リテラルトークン
  def initialize( header, define = nil )
    super()
    # ヘッダファイル名文字列から前後の "" を取り除く
    # header = header.to_s.gsub( /\A"(.*)"\z/, '\1' )
    header = CDLString.remove_dquote header.to_s

    if define then
      # 前後の "" を取り除く
      # def_opt = define.to_s.gsub( /\A"(.*)/, '\1' )
      # def_opt.sub!( /(.*)"\z/, '\1' )
      def_opt = CDLString.remove_dquote define.to_s

      # "," を -D に置き換え
      def_opt = def_opt.gsub( /,/, " -D " )

      # 先頭に -D を挿入 # mikan 不適切な define 入力があった場合、CPP 時にエラー
      def_opt = def_opt.gsub( /^/, "-D " )

    end

    # コマンドライン指定された DEFINE 
    $define.each{ |define|
      if $IN_EXERB then
        q = ""
      else
        if define =~ /'/ then
          q = '"'
        else
          q = "'"
        end
      end
      def_opt = "#{def_opt} -D #{q}#{define}#{q}"
    }

    header_path = find_file header

=begin
    include_opt = ""
    found = false
    header_path = ""
    $import_path.each{ |path|
      include_opt = "#{include_opt} -I #{path}"
      if found == false then
        begin
          # ファイルの stat を取ってみる(なければ例外発生)
          File.stat( "#{path}/#{header}" )

          # cdl を見つかったファイルパスに再設定
          header_path = "#{path}/#{header}"
          found = true
        rescue => evar
          found = false
          # print_exception( evar )
        end
      end
    }

    if found == false then
=end
    if header_path == nil then
      cdl_error( "S1142 $1 not found in search path" , header )
      return
    end

    include_opt = ""
    if get_base_dir then
      base = get_base_dir + "/"
    else
      base = ""
    end
    $import_path.each{ |path|
      include_opt = "#{include_opt} -I #{base}#{path}"
    }

    # 読込み済み？
    if( @@header_list[ header ] ) then
      # 第二引数 define が以前と異なる
      if @@define_list[ header ].to_s != define.to_s then
        cdl_error( "S1143 import_C: arg2: mismatch with previous one"  )
      end
      # いずれにせよ読み込まない
      return
    end

    # ヘッダのリストを記録
    @@header_list[ header ] = header_path
    @@header_list2 << header
    @@define_list[ header ] = define

    if $verbose then
      print "import_C header=#{header_path}, define=#{define}\n"
    end

    begin
      tmp_C = "#{$gen}/tmp_C_src.c"
      file = File.open( tmp_C, "w" )
    rescue => evar
      cdl_error( "S1144 $1: temporary C source: open error" , tmp_C )
      print_exception( evar )
    end

    begin
      print_defines file

      file.print( "#include \"#{header}\"\n" )
    rescue => evar
      cdl_error( "S1145 $1: temporary C source: writing error" , tmp_C )
      print_exception( evar )
    ensure
      file.close
    end

    # CPP 出力用 tmp ファイル名
    tmp_header = header.gsub( /\//, "_" )
    tmp_header = "#{$gen}/tmp_#{tmp_header}"

    # CPP コマンドラインを作成
    cmd = "#{$cpp} #{def_opt} #{include_opt} #{tmp_C}"

    begin
      if( $verbose )then
        puts "CPP: #{cmd}"
      end

      # プリプロセッサコマンドを pipe として開く
          # cmd は cygwin/Linux では bash(sh) 経由で実行される
          # Exerb 版では cmd.exe 経由で実行される
          # この差は引き数の (), $, % などシェルの特別な文字の評価に現れるので注意
          cpp = IO.popen( cmd, "r:ASCII-8BIT" )
      begin
        tmp_file = nil
        tmp_file = File.open( tmp_header, "w:ASCII-8BIT" )
        cpp.each { |line|
          line = line.gsub( /^#(.*)$/, '/* \1 */' )
          tmp_file.puts( line )
        }
      rescue => evar
        cdl_error( "S1146 $1: error occured while CPP" , header )
        print_exception( evar )
      ensure
        tmp_file.close if tmp_file    # mikan File.open に失敗した時 tmp_file == nil は保証されている ?
        cpp.close
      end
    rescue => evar
      cdl_error( "S1147 $1: popen for CPP failed" , header )
      print_exception( evar )
    end

    # C 言語のパーサインスタンスを生成
    c_parser = C_parser.new

    # tmp_header をパース
    c_parser.parse( [tmp_header] )

    # 終期化　パーサスタックを戻す
    c_parser.finalize

  end

  def print_defines file
    if ! $b_no_gcc_extension_support then
      
    file.print <<EOT

#ifndef TECS_NO_GCC_EXTENSION_SUPPORT

/*
 * these extension can be eliminated also by spefcifying option
 * --no-gcc-extension-support for tecsgen.
 */
#ifdef __GNUC__

#ifndef __attribute__
#define __attribute__(x)
#endif

#ifndef __extension__
#define __extension__
#endif

#ifndef __builtin_va_list
#define __builtin_va_list va_list
#endif

#ifndef __asm__
#define __asm__(x)
#endif

#ifndef restrict
#define restrict
#endif

#endif /* ifdef __GNUC__ */
#endif /* TECS_NO_GCC_EXTENSION_SUPPORT */
EOT
    end

    file.print <<EOT

/* va_list is not supported in C_parser.y.rb */
typedef struct { int dummy; } va_list;

EOT
  end

  def self.get_header_list
    @@header_list
  end
  def self.get_header_list2
    @@header_list2
  end

end

class Import < Node
# @b_reuse::bool:       再利用．セルタイプの template 生成不要
# @b_reuse_real::bool:  実際に再利用
# @cdl::      string:   import する CDL
# @cdl_path:: string:   CDL のパス
# @b_imported:: bool:   import された(コマンドライン指定されていない)

  include Importable

  # ヘッダの名前文字列のリスト  添字：expand したパス、値：Import
  @@import_list = {}

  @@nest_stack_index = -1
  @@nest_stack = []
  @@current_object = nil

  def self.push object
    @@nest_stack_index += 1
    @@nest_stack[ @@nest_stack_index ] = @@current_object
    @@current_object = object
  end

  def self.pop
    @@current_object = @@nest_stack[ @@nest_stack_index ]
    @@nest_stack_index -= 1
    if @@nest_stack_index < -1 then
      raise "TooManyRestore"
    end
  end

  #=== Import# import を行う
  #cdl::      string   cdl へのパス．"" で囲まれていることを仮定
  #b_reuse::  bool     true: template を生成しない
  def initialize( cdl, b_reuse = false, b_imported = true )
    Import.push self
    @b_imported = b_imported
    super()
    @@current_import = self
    # ヘッダファイル名文字列から前後の "", <> を取り除くn
    @cdl = cdl.to_s.gsub( /\A["<](.*)[">]\z/, '\1' )

    # サーチパスから探す
    found = false
    @cdl_path = ""

    @b_reuse = b_reuse
    @b_reuse_real = @b_reuse || Generator.is_reuse?

    if( Generator.get_plugin ) &&( File.exist? "#{$gen}/#{@cdl}" ) then
      @cdl_path = "#{$gen}/#{@cdl}"
      found = true
    else
      path = find_file @cdl
      if path then
        found = true
        @cdl_path = path
      end
    end

    if found == false then
      cdl_error( "S1148 $1 not found in search path" , @cdl )
      return
    end

    # 読込み済みなら、読込まない
    prev = @@import_list[ File.expand_path( @cdl_path ) ]
    if( prev ) then
      if prev.is_reuse_real? != @b_reuse_real then
        cdl_warning( "W1008 $1: reuse designation mismatch with previous import" , @cdl )
      end
      return
    end

    # import リストを記録
    @@import_list[ File.expand_path( @cdl_path ) ] = self

    # plugin から import されている場合
    plugin = Generator.get_plugin

    # パーサインスタンスを生成(別パーサで読み込む)
    parser = Generator.new

    # plugin から import されている場合の plugin 設定
    parser.set_plugin plugin

    # reuse フラグを設定
    parser.set_reuse @b_reuse_real

    # cdl をパース
    parser.parse( [@cdl_path] )

    # 終期化　パーサスタックを戻す
    parser.finalize
    Import.pop
  end

  def self.get_list
    @@import_list
  end

  def get_cdl_path
    @cdl_path
  end

  def is_reuse_real?
    @b_reuse_real
  end

  def self.get_current
    @@current_object
  end

  def is_imported?
    @b_imported
  end

  #=== cdl の名前を返す
  # 引数で指定されている cdl 名。一部パスを含む可能性がある
  def get_cdl_name
    @cdl
  end
end

#== generate: signature, celltype, cell へのプラグインのロードと適用
class Generate < Node
#@plugin_name:: Symbol
#@object_nsp:: NamespacePath
#@option::         String '"', '"' で囲まれている
#@plugin_object:: Plugin

  include PluginModule

  def initialize( plugin_name, object_nsp, option )
    super()
    @plugin_name = plugin_name
    @object_nsp = object_nsp
    option = option.to_s    # option は Token
    @option = option
    @plugin_object = nil

    dbgPrint "generate: #{plugin_name} #{object_nsp.to_s} option=#{option}\n"

    object = Namespace.find( object_nsp )
    if object.kind_of?( Signature ) ||
       object.kind_of?( Celltype ) ||
       object.kind_of?( CompositeCelltype ) ||
       object.kind_of?( Cell )then
      @plugin_object = object.apply_plugin( @plugin_name, @option )
    elsif object then
      # V1.5.0 以前の仕様では、signature のみ可能だった
#      cdl_error( "S1149 $1 not signature" , signature_nsp )
      cdl_error( "S9999 generate: '$1' neither signature, celltype nor cell", object_nsp )
      return
    else
      cdl_error( "S9999 generate: '$1' not found", object_nsp )
    end
  end
end

#== 名前空間パス
class NamespacePath < Node
#@b_absolute::Bool
#@path::[ Symbol,... ]
#@namespace::Namespace:  @b_absolute == false のとき、基点となる namespace

  #=== NamespacePath# initialize
  #ident::Symbol           最初の名前, ただし "::" のみの場合は String
  #b_absolute:Bool         "::" で始まっている場合 true
  #namespace::Namespace    b_absolute = false かつ、構文解釈段階以外で呼び出す場合は、必ず指定すること 
  def initialize( ident, b_absolute, namespace = nil )
    super()

    if ident == "::" then   # RootNamespace
      @path = []
      @b_absolute = true
    else
      @path = [ ident ]
      @b_absolute = b_absolute
    end

    if namespace then
      @namespace = namespace
      if b_absolute == true then
        raise "NamespacePath#initialize: naamespace specified for absolute path"
      end
    else
      if b_absolute == false then
        @namespace = Namespace.get_current
      else
        @namespace = nil
      end
    end
  end

  #=== NamespacePath# append する
  #RETURN self
  # このメソッドは、元の NamespacePath オブジェクトを変形して返す
  def append!( ident )
    @path << ident
    return self
  end
  #=== NamespacePath# append する
  # このメソッドは、元の NamespacePath オブジェクトを変形しない
  #RETURN:: 複製した NamespacePath
  def append( ident )
    cl = self.clone
    cl.set_clone
    cl.append!( ident )
    return cl
  end

  def set_clone
    @path = @path.clone
  end

  def get_name
    @path[ @path.length - 1 ]
  end

  #=== NamespacePath#クローンを作成して名前を変更する
  def change_name name
    cl = self.clone
    cl.set_clone
    cl.change_name_no_clone name
    return cl
  end
  alias :change_name_clone :change_name

  #=== NamespacePath#名前を変更する
  # このインスタンスを参照するすべてに影響を与えることに注意
  def change_name_no_clone name
    @path[ @path.length - 1 ] = name
    nil
  end

  #=== NamespacePath:: path 文字列を得る
  # CDL 用の path 文字列を生成
  def to_s
    get_path_str
  end
  def get_path_str
    first = true
    if @b_absolute then
      path = "::"
    else
      path = ""
    end
    @path.each{ |n|
      if first then
        path = "#{path}#{n}"
        first = false
      else
        path += "::#{n}"
      end
    }
    return path
  end

  def is_absolute?
    @b_absolute
  end
  def is_name_only?
    @path.length == 1 && @b_absolute == false
  end

  #=== NamespacePath:: パスの配列を返す
  # is_absolute? true の場合、ルートからのパス
  #              false の場合、base_namespace からの相対
  # ルート namespace の場合、長さ０の配列を返す
  #
  def get_path
    @path
  end

  #=== NamespacePath#フルパスの配列を返す
  # 返された配列を書き換えてはならない
  def get_full_path
    if @b_absolute then
      return @path
    else
      return @namespace.get_namespace_path.get_full_path.clone + @path
    end
  end

  #=== NamespacePath:: 相対パスのベースとなる namespace
  # is_absolute? == false の時のみ有効な値を返す (true なら nil)
  def get_base_namespace
    @namespace
  end

  #=== NamespacePath:: C 言語グローバル名を得る
  def get_global_name
    if @b_absolute then
      global_name = ""
    else
      global_name = @namespace.get_global_name
    end

    @path.each{ |n|
      if global_name != "" then
        global_name = "#{global_name}_#{n}"
      else
        global_name = n.to_s
      end
    }
    global_name
  end

  #=== NamespacePath:: 分解して NamespacePath インスタンスを生成する
  #path_str:: String       : namespace または region のパス ex) "::path::A" , "::", "ident"
  #b_force_absolute:: Bool : "::" で始まっていない場合でも絶対パスに扱う
  #
  # NamespacePath は通常構文解析されて作成される
  # このメソッドは、オプションなどで指定される文字列を分解して NamespacePath を生成するのに用いる
  # チェックはゆるい。不適切なパス指定は、不適切な NamespacePath が生成される
  def self.analyze( path_str, b_force_absolute = false )

    if path_str == "::" then
      return self.new( "::", true )
    end

    pa = path_str.split( "::" )
    if pa[0] == "" then
      pa.shift
      b_absolute = true
    else
      if b_force_absolute then
        b_absolute = true
      else
        b_absolute = false
      end
    end

    if pa[0] then
      nsp = self.new( pa[0].to_sym, b_absolute )
    else
      nsp = self.new( "::", b_absolute )
    end
    pa.shift

    pa.each{ |a|
      if a then
        nsp.append! a.to_sym
      else
        nsp.append! "::"
      end
    }

    return nsp
  end

end

# 以下単体テストコード
if $unit_test then
  root_namespace = Namespace.new("::")

  puts( "===== Unit Test: NamespacePath ===== (componentobj.rb)")
  a = NamespacePath.new( :"ABC", true )
  printf( "Path: %-10s global_name: %s\n", a.get_path_str, a.get_global_name )

  a.append( :"DEF" )
  printf( "Path: %-10s global_name: %s\n", a.get_path_str, a.get_global_name )

  a = NamespacePath.new( :"abc", false )
  printf( "Path: %-10s global_name: %s\n", a.get_path_str, a.get_global_name )

  a.append( :"def" )
  printf( "Path: %-10s global_name: %s\n", a.get_path_str, a.get_global_name )

  puts ""
end
