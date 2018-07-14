# -*- coding: utf-8 -*-
#
#  TECS Generator
#      Generator for TOPPERS Embedded Component System
#  
#   Copyright (C) 2008-2016 by TOPPERS Project
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
#   $Id: syntaxobj.rb 2633 2017-04-02 06:02:05Z okuma-top $
#++

# mikan ruby の symbol と文字列の使い分けがどうしてもうまくいかないことが時々あるので name.to_sym を入れることにした

#== Node
#
# Node の直接の子クラス： C_EXP, Type, BaseVal, BDNode(ほとんどのものは BDNode の子クラス)
# Node に (BDNodeにも) 入らないもの: Token, Import, Import_C, Generate
#
# owner を持たないものが Node となる
# エラーは、cdl_error を通じて報告する (意味解析が構文解析後に行われる場合には、行番号が正しく出力できる
# 

class Node
#@locale::    [@file, @lineno, @col]

  def initialize
    @locale = Generator.current_locale
  end

  #=== エラーを出力する
  def cdl_error( message, *arg )
    Generator.error2( @locale, message, *arg )
  end

  #=== エラーを出力する
  #locale:: Array(locale info) : 構文解析中は無視される
  def cdl_error2( locale, message, *arg )
    Generator.error2( locale, message, *arg )
  end

  #=== エラーを出力する
  #locale:: Array(locale info)
  # 構文解析中 cdl_error2 では locale が無視されるため、別に locale を出力する
  def cdl_error3( locale, message, *arg )
    Generator.error(  message, *arg )
    Console.puts "check: #{locale[0]}: line #{locale[1]} for above error"
  end

  #=== ウォーニング出力する
  def cdl_warning( message, *arg )
    Generator.warning2( @locale, message, *arg )
  end

  #=== ウォーニング出力する
  def cdl_warning2( locale, message, *arg )
    Generator.warning2( locale, message, *arg )
  end

  #=== 情報を表示する
  def cdl_info( message, *arg )
    Generator.info2( @locale, message, *arg )
  end

  #=== 情報を表示する
  def cdl_info2( locale, message, *arg )
    Generator.info2( locale, message, *arg )
  end

  def get_locale
    @locale
  end

  def set_locale locale
    @locale = locale
  end

  def locale_str
    "locale=( #{@locale[0]}, #{@locale[1]} )"
  end
end

#== 双方向 Node (Bi Direction Node)
#
#  Node の子クラス
#  owner Node から参照されているもの (owner へのリンクも取り出せる)
#
#  get_owner で得られるもの
#    FuncHead => Signature
#    Decl => Namespace(const), Typedef(typedef),
#            Celltype, CompositeCelltype(attr,var)
#            Struct(member), ParamDecl(parameter), FuncHead(funchead)
#    Signature, Celltype, CompositeCelltype, Typedef => Namespace
#,   Namespace => Namespace, Generator.class (root Namespace の場合)
#    Cell => Region, CompositeCelltype(in_composite)
#    Port => Celltype, Composite
#    Factory => Celltype
#    Join => Cell
#    CompositeCelltypeJoin => CompositeCelltype
#    Region => Region, 
#    ParamDecl => ParamList
#    ParamList => FuncHead
#    Expression => Namespace
#    大半のものは new_* メソッドで owner Node に伝達される
#    そのメソッドが呼び出されたときに owner Node が記録される
#    new_* がないもの：
#            Decl(parameter), ParamDecl, ParamList, FuncHead, Expression 
#
#    Expression は、owner Node となるものが多くあるが、改造が困難であるため
#    Expression が定義されたときの Namespace を owner Node とする
#    StructType は Type の一種なので owner を持たない
#
class BDNode < Node
#@owner::Node
#@NamespacePath:: NamespacePath
#@Generator::
#@import::Import :  

  def initialize
    super
    @owner = nil
    @NamespacePath = nil
    @import = Import.get_current

  end

  #=== owner を設定する
  def set_owner owner
    dbgPrint "set_owner: #{owner.class.name}\n"
    @owner = owner
  end

  #=== owner を得る
  # class の説明を参照
  def get_owner
    if @owner == nil
      raise "Node have no owner #{self.class.name} #{get_name}"
    end
    @owner
  end
end

#== Namespace 名を持つ BDNode
# Namespace(Region), Signature, Celltype, CompositeCelltype, Cell
class NSBDNode < BDNode

  def initialize
    super
  end

  #=== 属する namespace を得る
  # owner を namespace にたどり着くまで上にたどる
  def get_namespace
    if @owner.kind_of? Namespace
      return @owner
    elsif @owner != nil then
      return @owner.get_namespace
    else
      # @owner == nil なら "::"
      if @name != "::" then
        raise "non-root namespace has no owner #{self.class.name}##{@name} #{self}"
      end
      return nil
    end
  end

  def set_namespace_path
    ns = get_namespace
    if ns then
      @NamespacePath = ns.get_namespace_path.append( get_name )
    else
      raise "get_namespace_path: no namespace found"
    end
  end

  #=== NamespacePath を得る
  def get_namespace_path
    return @NamespacePath
  end

  def is_imported?
    if @import then
      return @import.is_imported?
    else
      return false    # mikan: 仮 @import が nil になるケースが追求できていない
    end
  end
end

class NamedList
#  @names:: {} of items
#  @items:: [] of items : item の CLASS は get_name メソッドを持つこと get_name の戻り値は Symbol でなくてはならない
#                         NamedList を clone_for_composite する場合は、item にもメソッドが必要
#  @type:: string	エラーメッセージ

  def initialize( item, type )
    @names = {}
    @items = []
    @type = type
    add_item( item )
  end

  #=== 要素を加える
  # parse した時点で加えること(場所を記憶する)
  def add_item( item )

    if item then
      assert_name item
      name = item.get_name
      prev = @names[name]
      if prev then
        Generator.error( "S2001 \'$1\' duplicate $2" , name, @type )
        prev_locale = prev.get_locale
        puts "previous: #{prev_locale[0]}: line #{prev_locale[1]} \'#{name}\' defined here"
        return self
      end

      @names[name]=item
      @items << item
    end

    return self
  end

  def change_item( item )
    assert_name item
    name = item.get_name

    prev_one = @names[name]
    @names[name]=item

    @items = @items - [ prev_one ]
    @items << item
  end

  def del_item( item )
    assert_name item
    name = item.get_name
    @names.delete name

    @items = @items - [ item ]
  end

  def get_item( name )
    if ! name.kind_of? Symbol
      print "get_item: '#{name}', items are below\n"
      @names.each{ |nm,item|
        p nm
      }
      raise "get_item: #{name}: not Symbol"
    end
    if name then
      return @names[name.to_sym]
    else
      return nil
    end
  end

  def get_items
    return @items
  end

  #=== composite cell を clone した時に要素(JOIN) の clone する
  #
  # mikan このメソッドは Join に特化されているので NamedList から分離すべき
  def clone_for_composite( ct_name, cell_name, locale )
    cl = self.clone
    cl.set_cloned( ct_name, cell_name, locale )
    return cl
  end

  #=== clone された NamedList インスタンスの参照するもの(item)を clone
  #
  # mikan このメソッドは Join に特化されているので NamedList から分離すべき
  def set_cloned( ct_name, cell_name, locale )
    items = []
    names = {}
    @items.each { |i|
      dbgPrint "NamedList clone #{ct_name}, #{cell_name}, #{i.get_name}\n"

      cl = i.clone_for_composite( ct_name, cell_name, locale )
      names[cl.get_name] = cl
      items << cl
    }
    @items = items
    @names = names
  end

  def assert_name item
    if ! item.get_name.kind_of? Symbol
      raise "Not symbol for NamedList item"
    end
  end

  def show_tree( indent )
    @items.each { |i|
      i.show_tree( indent )
    }
  end

end

class Typedef < BDNode
# @declarator:: Decl

  def self.new_decl_list( type_spec_qual_list, decl_list )
    decl_list.each { |decl|
       Typedef.new( type_spec_qual_list, decl )
    }
  end

  def initialize( type_spec_qual_list, decl )
    super()
    decl.set_type( type_spec_qual_list )
    @declarator = decl
    decl.set_owner self    # Decl(Typedef)

    Namespace.new_typedef( self )
  end

  def get_name
    @declarator.get_name
  end

  def get_declarator
    @declarator
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "Typedef: #{locale_str}"
    @declarator.show_tree( indent + 1 )
  end
end

#== 関数頭部
# signature に登録される関数
class FuncHead <BDNode
#  @declarator:: Decl

  def initialize( declarator, type, b_oneway )
    super()
    declarator.set_type( type )
    @declarator = declarator
    @declarator.set_owner self  # Decl (FuncHead)

    if @declarator.get_type.kind_of?( FuncType ) then
      if b_oneway then
        @declarator.get_type.set_oneway( b_oneway )
      end
    end
    @declarator.get_type.check_struct_tag :FUNCHEAD

    # check if return type is pointer
    if declarator.get_type.kind_of? FuncType then
      if declarator.get_type.get_type.get_original_type.kind_of?( PtrType ) &&
          Signature.get_current.is_deviate? == false then
        cdl_warning( "W3004 $1 pointer type has returned. specify deviate or stop return pointer" , @declarator.get_identifier )
      end
    end
  end

  def get_name
    @declarator.get_name
  end

  def get_declarator
    @declarator
  end

  def is_oneway?
    if @declarator.is_function? then
      return @declarator.get_type.is_oneway?
    end
    return false
  end

  def is_function?
    @declarator.is_function?
  end

  #=== FuncHead# 関数の名前を返す
  def get_name
    return @declarator.get_name
  end

  #=== FuncHead# 関数の戻り値の型を返す
  # types.rb に定義されている型
  # 関数ヘッダの定義として不完全な場合 nil を返す
  def get_return_type
    if is_function? then
      return @declarator.get_type.get_type
    end
  end

  #=== FuncHead# 関数の引数のリストを返す
  # ParamList を返す
  # 関数ヘッダの定義として不完全な場合 nil を返す
  def get_paramlist
    if is_function? then
      return @declarator.get_type.get_paramlist
    end
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "FuncHead: #{locale_str}"
    @declarator.show_tree( indent + 1 )
  end
end

#=== 宣言
# @kind で示される各種の宣言
class Decl < BDNode

# @identifer:: String
# @global_name:: String | nil : String(@kind=TYPEDEF||:CONSTANT), nil(@kind=その他)
#                set_kind にて設定される
# @type:: ArrayType, FuncType, PtrType, IntType, StructType
#         VoidType, FloatType, DefinedType, BoolType
# @initializer:: constant_expression, mikan { initlist }
# @kind:: :VAR, :ATTRIBUTE, :PARAMETER, :TYPEDEF, :CONSTANT, :MEMBER, :FUNCHEAD(signatureの関数定義)
# @b_referenced:: bool
#
# 以下は、@kind が :VAR, :ATTRIBUTE のときに有効
# @rw:: bool     # 古い文法では attr に指定可能だった（消すには generate の修正も必要）
# @omit:: bool
# @choice_list:: [String]  attr 初期値の選択肢
# 以下は、@kind が :VAR, :ATTRIBUTE, :MEMBER のときに有効
# @size_is:: Expression or nil unless specified
# 以下は、@kind が :MEMBER のときに有効
# @count_is:: Expression or nil unless specified
#             attr, var の場合、count_is は指定できない
# @string:: Expression, -1 (length not specified) or nil (not specified)
#
# mikan  ParamDecl だけ別に設けたが、MemberDecl, AttrDecl なども分けるべきか(？)

  def initialize( identifier )
    super()
    @identifier = identifier
    @rw = false
    @omit = false
    @size_is = nil
    @count_is = nil
    @string  = nil
    @choice_list  = nil
    @b_referenced  = false
  end

  def set_initializer( initializer )
    @initializer = initializer
  end

  def get_initializer
    @initializer
  end

  def is_function?
    if @type.class == FuncType then
      return true
    else
      return false
    end
  end

  #== Decl の意味的誤りをチェックする
  def check
    # return nil if @type == nil

    # 構造体タグチェック（ポインタ型から構造体が参照されている場合は、タグの存在をチェックしない）
    @type.check_struct_tag @kind

    # 型のチェックを行う
    res = @type.check
    if res then
      cdl_error( "S2002 $1: $2" , @identifier, res )
    end

    # 不要の初期化子をチェックする
    if @initializer then
      case @kind
      when :PARAMETER, :TYPEDEF, :MEMBER, :FUNCHEAD
        cdl_error( "S2003 $1: $2 cannot have initializer" , @identifier, @kind.to_s.downcase )
      when :VAR, :ATTRIBUTE, :CONSTANT
        # p @initializer  ここでは代入可能かどうか、チェックしない
        # :VAR, :ATTRIBUTE, :CONSTANT はそれぞれでチェックする
        # return @type.check_init( @identifier, @initializer, @kind )
      else
        raise "unknown kind in Delc::check"
      end
    end

    if( @type.kind_of? ArrayType ) && ( @type.get_subscript == nil ) && ( @omit == false ) then
      if @kind == :ATTRIBUTE then
        cdl_error( "S2004 $1: array subscript must be specified or omit" , @identifier )
      elsif @kind == :VAR || @kind == :MEMBER then
        cdl_error( "S2005 $1: array subscript must be specified" , @identifier )
      end
    end

    return nil
  end

  #== ポインタレベルを得る
  # 戻り値：
  #   非ポインタ変数   = 0
  #   ポインタ変数     = 1
  #   二重ポインタ変数 = 2
  def get_ptr_level
    level = 0
    type = @type
    while 1
      if type.kind_of?( PtrType ) then
        level += 1
        type = type.get_referto
#      elsif type.kind_of?( ArrayType ) then  # 添数なし配列はポインタとみなす
#        if type.get_subscript == nil then
#          level += 1
#          type = type.get_type
#        else
#          break
#        end
        # mikan ポインタの添数あり配列のポインタレベルは０でよい？
      elsif type.kind_of?( DefinedType ) then
        type = type.get_type
        # p "DefinedType: #{type} #{type.class}"
      else
        break
      end
    end
    return level
  end

  def get_name
    @identifier
  end

  def get_global_name
    @global_name
  end

  def set_type( type )
    unless @type then
      @type = type
    else
      @type.set_type( type )             # 葉に設定
    end
  end

  def get_type
    @type
  end

  def get_identifier
    @identifier
  end

  # STAGE: B
  def set_kind( kind )
    @kind = kind
    case kind
    when :TYPEDEF, :CONSTANT
      if Namespace.get_global_name.to_s == "" then
        @global_name = @identifier
      else
        @global_name = :"#{Namespace.get_global_name}_#{@identifier}"
      end
    else
      @global_name = nil
    end
  end

  def get_kind
    @kind
  end

  def set_specifier_list( spec_list )
    spec_list.each{  |spec|
      case spec[0]
      when :RW
        @rw = true
      when :OMIT
        @omit = true
      when :SIZE_IS
        @size_is = spec[1]
      when :COUNT_IS
        @count_is = spec[1]
      when :STRING
        @string = spec[1]
      when :CHOICE
        @choice_list = spec[1]
      else
        raise "Unknown specifier #{spec[0]}"
      end
    }

    if @size_is || @count_is || @string
        @type.set_scs( @size_is, @count_is, @string, nil, false )
    end
  end

  def is_rw?
    @rw
  end

  def is_omit?
    @omit
  end

  def get_size_is
    @size_is
  end

  def get_count_is
    @count_is
  end

  def get_string
    @string
  end

  def get_choice_list
    @choice_list
  end

  def referenced
    @b_referenced = true
  end

  def is_referenced?
    @b_referenced
  end

  def is_type?( type )
    t = @type
    while 1
      if t.kind_of?( type ) then
        return true
      elsif t.kind_of?( DefinedType ) then
        t = t.get_type
      else
        return false
      end
    end
  end

  def is_const?
    type = @type
    while 1
      if type.is_const? then
        return true
      elsif type.kind_of?( DefinedType ) then
        type = type.get_type
      else
        return false
      end
    end
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "Declarator: name: #{@identifier} kind: #{@kind} global_name: #{@global_name} #{locale_str}"
    (indent+1).times { print "  " }
    puts "type:"
    @type.show_tree( indent + 2 )
    if @initializer then
      (indent+1).times { print "  " }
      puts "initializer:"
      @initializer.show_tree( indent + 2 )
    else
      (indent+1).times { print "  " }
      puts "initializer: no"
    end
    (indent+1).times { print "  " }
    puts "size_is: #{@size_is.to_s}, count_is: #{@count_is.to_s}, string: #{@string.to_s} referenced: #{@b_referenced} "
   
  end

end

# 関数パラメータの宣言
class ParamDecl < BDNode

# @declarator:: Decl:  Token, ArrayType, FuncType, PtrType
# @direction:: :IN, :OUT, :INOUT, :SEND, :RECEIVE
# @size:: Expr   (size_is 引数)
# @count:: Expr   (count_is 引数)
# @max:: Expr (size_is の第二引数)
# @b_nullable:: Bool : nullable 
# @string:: Expr or -1(if size not specified) （string 引数）
# @allocator:: Signature of allocator
# @b_ref:: bool : size_is, count_is, string_is 引数として参照されている
#
# 1. 関数型でないこと
# 2. ２次元以上の配列であって最も内側以外の添数があること
# 3. in, out, ..., size_is, count_is, ... の重複指定がないこと
# 4. ポインタレベルが適切なこと

  def initialize( declarator, specifier, param_specifier )
    super()
    @declarator = declarator
    @declarator.set_owner self  # Decl (ParamDecl)
    @declarator.set_type( specifier )
    @param_specifier = param_specifier
    @b_ref = false
    @b_nullable = false

    if @declarator.is_function? then		# (1)
      cdl_error( "S2006 \'$1\' function" , get_name )
      return
    end

    res = @declarator.check
    if res then					# (2)
      cdl_error( "S2007 \'$1\' $2" , get_name, res )
      return
    end

    @param_specifier.each { |i|
      case i[0]                                     # (3)
      when :IN, :OUT, :INOUT, :SEND, :RECEIVE
        if @direction == nil then
          @direction = i[0]
        elsif i[0] == @direction then
          cdl_warning( "W3001 $1: duplicate" , i[0] )
          next
        else
          cdl_error( "S2008 $1: inconsitent with previous one" , i[0] )
          next
        end

        case i[0]
        when :SEND, :RECEIVE
          @allocator = Namespace.find( i[1] )   #1
          if ! @allocator.instance_of?( Signature ) then
            cdl_error( "S2009 $1: not found or not signature" , i[1] )
            next
          elsif ! @allocator.is_allocator? then
            # cdl_error( "S2010 $1: not allocator signature" , i[1] )
          end
        end

      when :SIZE_IS
        if @size then
          cdl_error( "S2011 size_is duplicate"  )
        else
          @size = i[1]
        end
      when :COUNT_IS
        if @count then
          cdl_error( "S2012 count_is duplicate"  )
        else
          @count = i[1]
        end
      when :STRING
        if @string then
          cdl_error( "S2013 string duplicate"  )
        elsif i[1] then
          @string = i[1]
        else
          @string = -1
        end
      when :MAX_IS
        # max_is は、内部的なもの bnf.y.rb 参照
        # size_is で重複チェックされる
        @max = i[1]
      when :NULLABLE
        # if ! @declarator.get_type.kind_of?( PtrType ) then
        #  cdl_error( "S2026 '$1' nullable specified for non-pointer type", @declarator.get_name )
        # else
          @b_nullable = true
        # end
      end

    }

    if @direction == nil then
      cdl_error( "S2014 No direction specified. [in/out/inout/send/receive]"  )
    end

    if ( @direction == :OUT || @direction == :INOUT ) && @string == -1 then
      cdl_warning( "W3002 $1: this string might cause buffer over run" , get_name )
    end

    # mikan ポインタの配列（添数有）のレベルが０
    ptr_level = @declarator.get_ptr_level

    # p "ptr_level: #{@declarator.get_identifier} #{ptr_level}"
    # p @declarator

    #----  set req_level, min_level & max_level  ----#
    if !(@size||@count||@string) then	    # (4)
      req_level = 1
    elsif (@size||@count)&&@string then
      req_level = 2
    else
      req_level = 1
    end

    if @direction == :RECEIVE then
      req_level += 1
    end
    min_level = req_level
    max_level = req_level

    # IN without pointer specifier can be non-pointer type
    if @direction == :IN && !(@size||@count||@string) then
      min_level = 0
    end

    # if size_is specified and pointer refer to struct, max_level increase
    if @size then
      type = @declarator.get_type.get_original_type
      while type.kind_of? PtrType
        type = type.get_referto.get_original_type
      end
      if type.kind_of? StructType then
        max_level += 1
      end
    end
    #----  end req_level & max_level    ----#

    # p "req_level: #{req_level} ptr_level: #{ptr_level}"
    #if ptr_level < req_level && ! ( @direction == :IN && req_level == 1 && ptr_level == 0) then
    if ptr_level < min_level then
      cdl_error( "S2014 $1 need pointer or more pointer" , @declarator.get_identifier )
    elsif ptr_level > max_level then
      # note: 構文解析段階で実行のため get_current 可
      if Signature.get_current == nil || Signature.get_current.is_deviate? == false then
        cdl_warning( "W3003 $1 pointer level mismatch" , @declarator.get_identifier )
      end
    end

    type = @declarator.get_type
    while type.kind_of?( DefinedType )
      type = type.get_original_type
    end

    if ptr_level > 0 then
      # size_is, count_is, string をセット
      if @direction == :RECEIVE && ptr_level > 1 then
        type.get_type.set_scs( @size, @count, @string, @max, @b_nullable )
      else
        type.set_scs( @size, @count, @string, @max, @b_nullable )
      end

#p ptr_level
#type.show_tree 1

      # ポインタが指している先のデータ型を得る
      i = 0
      t2 = type
      while i < ptr_level
        t2 = t2.get_referto
        while t2.kind_of?( DefinedType )
          t2 = t2.get_original_type
        end
        i += 1
      end

# p @declarator.get_name
# t2.show_tree 1
# p t2.is_const?

      # const 修飾が適切かチェック
      if @direction == :IN then
        if ! t2.is_const? then
          cdl_error( "S2015 '$1' must be const for \'in\' parameter $2" , get_name, type.class )
        end
      else
        if t2.is_const? then
          cdl_error( "S2016 '$1' can not be const for $2 parameter" , get_name, @direction )
        end
      end
    else
      # 非ポインタタイプ
      if @size != nil || @count != nil || @string != nil || @max != nil || @b_nullable then
        type.set_scs( @size, @count, @string, @max, @b_nullable )
      end
    end

#    if ptr_level > 0 && @direction == :IN then
#      if type.is_const != :CONST
#    end

    # p self

  end

  def check_struct_tag kind
    @declarator.get_type.check_struct_tag :PARAMETER
  end

  def get_name
    @declarator.get_name
  end

  def get_size
    @size
  end

  def get_count
    @count
  end

  def get_string
    @string
  end

  def get_max
    @max
  end

  def clear_max
    # p "clear_max: #{@declarator.get_name} #{@max.to_s}"
    @max = nil
    @declarator.get_type.clear_max
  end

  def is_nullable?
    @b_nullable
  end

  def get_type
    @declarator.get_type
  end

  def get_direction
    @direction
  end

  def get_declarator
    @declarator
  end

  def get_allocator
    @allocator
  end

  def referenced
    @b_ref = true
  end

  def is_referenced?
    @b_ref
  end

  #=== PPAllocator が必要か
  # Transparent RPC の場合 in で size_is, count_is, string のいずれかが指定されている場合 oneway では PPAllocator が必要
  # Transparent PC で oneway かどうかは、ここでは判断しないので別途判断が必要
  # Opaque RPC の場合 size_is, count_is, string のいずれかが指定されている場合、PPAllocator が必要
  def need_PPAllocator?( b_opaque = false )
    if ! b_opaque then
#      if @direction == :IN && ( @size || @count || @string ) then
      if @direction == :IN && @declarator.get_type.get_original_type.kind_of?( PtrType ) then
        return true
      end
    else
      if (@direction == :IN || @direction == :OUT || @direction == :INOUT ) &&
          @declarator.get_type.get_original_type.kind_of?( PtrType ) then
        return true
      end
    end
    return false
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "ParamDecl: direction: #{@direction} #{locale_str}"
    @declarator.show_tree( indent + 1 )
    if @size then
      (indent+1).times { print "  " }
      puts "size:"
      @size.show_tree( indent + 2 )
    end
    if @count then
      (indent+1).times { print "  " }
      puts "count:"
      @count.show_tree( indent + 2 )
    end
    if @string then
      (indent+1).times { print "  " }
      puts "string:"
      if @string == -1 then
       (indent+2).times { print "  " }
        puts "size is not specified"
      else
        @string.show_tree( indent + 2 )
      end
    end
    if @allocator then
      (indent+1).times { print "  " }
      puts "allocator: signature: #{@allocator.get_name}"
    end    
  end
end

# 関数パラメータリスト
class ParamList < BDNode
# @param_list:: NamedList : item: ParamDecl

  def initialize( paramdecl )
    super()
    @param_list = NamedList.new( paramdecl, "parameter" )
    @param_list.get_items.each { |paramdecl|
      paramdecl.set_owner self   # ParamDecl
    }
  end

  def add_param( paramdecl )
    return if paramdecl == nil    # 既にエラー

    @param_list.add_item( paramdecl )
    paramdecl.set_owner self   # ParamDecl
  end

  def get_items
    @param_list.get_items
  end

  #=== size_is, count_is, string の引数の式をチェック
  # 変数は前方参照可能なため、関数頭部の構文解釈が終わった後にチェックする
  def check_param
    @param_list.get_items.each { |i|
      next if i == nil                      # i == nil : エラー時

      if i.get_type.class == VoidType then
        # 単一の void 型はここにはこない
        cdl_error( "S2027 '$1' parameter cannot be void type", i.get_name )
      end

      size = i.get_size			# Expression
      if size then
        val = size.eval_const( @param_list )
        if val == nil then			# 定数式でないか？
          # mikan 変数を含む式：単一の変数のみ OK
          type = size.get_type( @param_list )
          unless type.kind_of?( IntType ) then
            cdl_error( "S2017 size_is argument is not integer type"  )
          else
            size.check_dir_for_param( @param_list, i.get_direction, "size_is" )
          end
        else
          if val != Integer( val ) then
            cdl_error( "S2018 \'$1\' size_is parameter not integer" , i.get_declarator.get_identifier )
          elsif val <= 0 then
            cdl_error( "S2019 \'$1\' size_is parameter negative or zero" , i.get_declarator.get_identifier )
          end
        end
      end

      max = i.get_max
      if max then
        val2 = max.eval_const( @param_list )
        if val2 == nil then
          cdl_error( "S2028 '$1' max (size_is 2nd parameter) not constant", i.get_name )
        elsif val2 != Integer( val2 ) || val2 <= 0 then
          cdl_error( "S2029 '$1' max (size_is 2nd parameter) negative or zero, or not integer", i.get_name )
        end
      end

      if val != nil && val2 != nil then
        if val < val2 then
          cdl_warning( "W3005 '$1' size_is always lower than max. max is ignored", i.get_name )
          i.clear_max
        else
          cdl_error( "S2030 '$1' both size_is and max are const. size_is larger than max", i.get_name )
        end
      end

      count = i.get_count			# Expression
      if count then
        val = count.eval_const( @param_list )
        if val == nil then			# 定数式でないか？
          # mikan 変数を含む式：単一の変数のみ OK
          type = count.get_type( @param_list )
          unless type.kind_of?( IntType ) then
            cdl_error( "S2020 count_is argument is not integer type"  )
          else
            count.check_dir_for_param( @param_list, i.get_direction, "count_is" )
          end
        else
          if val != Integer( val ) then
            cdl_error( "S2021 \'$1\' count_is parameter not integer" , i.get_declarator.get_identifier )
          elsif val <= 0 then
            cdl_error( "S2022 \'$1\' count_is parameter negative or zero" , i.get_declarator.get_identifier )
          end
        end
      end

      string = i.get_string			# Expression
      if string != -1 && string then
        val = string.eval_const( @param_list )
        if val == nil then			# 定数式でないか？
          # mikan 変数を含む式：単一の変数のみ OK
          type = string.get_type( @param_list )
          unless type.kind_of?( IntType ) then
            cdl_error( "S2023 string argument is not integer type"  )
          else
            string.check_dir_for_param( @param_list, i.get_direction, "string" )
          end
        else
          if val != Integer( val ) then
            cdl_error( "S2024 \'$1\' string parameter not integer" , i.get_declarator.get_identifier )
          elsif val <= 0 then
            cdl_error( "S2025 \'$1\' string parameter negative or zero" , i.get_declarator.get_identifier )
          end
        end
      end
    }
  end

  def check_struct_tag kind
    @param_list.get_items.each{ |p|
      p.check_struct_tag kind
    }
  end

  #=== Push Pop Allocator が必要か？
  # Transparent RPC の場合 (oneway かつ) in の配列(size_is, count_is, string のいずれかで修飾）がある
  def need_PPAllocator?( b_opaque = false )
    @param_list.get_items.each { |i|
      if i.need_PPAllocator?( b_opaque ) then
        return true
      end
    }
    false
  end

  def find( name )
    @param_list.get_item( name )
  end

  #== ParamList# 文字列化
  #b_name:: Bool: パラメータ名を含める
  def to_str( b_name )
    str = "("
    delim = ""
    @param_list.get_items.each{ |paramdecl|
      decl = paramdecl.get_declarator
      str += delim + decl.get_type
      if b_name then
        str += " " + decl.get_name
      end
      str += decl.get_type_post
      delim = ", "
    }
    str += ")"
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "ParamList: #{locale_str}"
    @param_list.show_tree( indent + 1 )
  end
end

#== CDL の文字列リテラルを扱うためのクラス
# CDL の文字列リテラルそのものではない
class CDLString
  # エスケープ文字を変換
  def self.escape str
    str = str.dup
    str.gsub!( /\\a/, "\x07" )
    str.gsub!( /\\b/, "\x08" )
    str.gsub!( /\\f/, "\x0c" )
    str.gsub!( /\\n/, "\x0a" )
    str.gsub!( /\\r/, "\x0d" )
    str.gsub!( /\\t/, "\x08" )
    str.gsub!( /\\v/, "\x0b" )
    str.gsub!( /(\\[Xx][0-9A-Fa-f]{1,2})/, '{printf \"\\1\"}' )
    str.gsub!( /(\\[0-7]{1,3})/, '{printf \"\\1\"}' )
    str.gsub!( /\\(.)/, "\\1" )   # mikan 未定義のエスケープシーケンスを変換してしまう (gcc V3.4.4 では警告が出される)
    return str
  end

  #=== CDLString#前後の " を取り除く
  def self.remove_dquote str
    s = str.sub( /\A"/, "" )
    s.sub!( /"\z/, "" )
    return s
  end
end

#== CDL の初期化子を扱うためのクラス
# CDL の初期化子そのものではない
class CDLInitializer
  #=== 初期化子のクローン
  # 初期化子は Expression, C_EXP, Array のいずれか
  def self.clone_for_composite( rhs, ct_name, cell_name, locale )
    if rhs.instance_of? C_EXP then
      # C_EXP の clone を作るとともに置換
      rhs = rhs.clone_for_composite( ct_name, cell_name, locale )
    elsif rhs.instance_of? Expression then
      rhs = rhs.clone_for_composite
    elsif rhs.instance_of? Array then
      rhs = clone_for_compoiste_array( rhs, ct_name, cell_name, locale )
    else
      raise "unknown rhs for join"
    end
    return rhs
  end

  #=== 初期化子（配列）のクローン
  # 要素は clone_for_composite を持つものだけ
  def self.clone_for_compoiste_array( array, ct_name, cell_name, locale )
    # "compoiste.identifier" の場合 (CDL としては誤り)
    if array[0] == :COMPOSITE then
      return array.clone
    end

    new_array = array.map{ |m|
      clone_for_composite( m, ct_name, cell_name, locale )
    }
    return new_array
  end
end
