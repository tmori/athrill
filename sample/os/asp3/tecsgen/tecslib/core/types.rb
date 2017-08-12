# -*- coding: utf-8 -*-
#
#  TECS Generator
#      Generator for TOPPERS Embedded Component System
#
#   Copyright (C) 2008-2015 by TOPPERS Project
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
#   $Id: types.rb 2633 2017-04-02 06:02:05Z okuma-top $
#++

#= HasType: @type を内部に持つ型のためのモジュール
#  @b_cloned::Bool  : true if @type is cloned
#
# このモジュールは DefinedType, PtrType, ArrayType に include される
# 本当は typedef された時の Decl の要素のみ clone すればよいのだが、get_type, get_original_type で
# 取り出されたとき、set_scs, set_qualifier されたときに無条件で clone する (無駄にメモリを使用する)
# ただし、clone するのは一回のみである (二回 clone すると別の型を参照してしまう)
#
# initialize で clone しても、共有されているときに clone されない
#
module HasType
  def initHasType
    @b_cloned = false
  end

  #=== HasType# @type をクローンする
  def clone_type
#    if @b_cloned == false then
      @type = @type.clone
      @b_cloned = true
#    end
  end
end

class Type < Node
# @b_const    : bool
# @b_volatile : bool

  def initialize
    super
  end

  def set_qualifier( qualifier )
    case qualifier
    when :CONST
#      if @b_const then
#        cdl_error( "T1001 const duplicate"  )
#      end
      @b_const = true
    when :VOLATILE
#      if @b_volatile then
#        cdl_error( "T1002 volatile duplicate"  )
#      end
      @b_volatile = true
    else
      raise "Unknown qualifier #{qualifier}"
    end
  end

  def is_const?
    if @b_const then
      return true
    else
      return false
    end
  end

  def is_volatile?
    if @b_volatile then
      return true
    else
      return false
    end
  end

  def is_void?
    if self.kind_of? DefinedType then
      return @type.is_void?
    elsif self.kind_of? VoidType then
      return true
    else
      return false
    end
  end

  #=== size_is, count_is, string を設定
  # 派生クラスでオーバーライドする（デフォルトではエラー）
  def set_scs( size, count, string, max = nil, b_nullable = false )
    str = ""
    delim = ""
    if size then
      str = "size_is"
      delim = ", "
    end
    if count then
      str = "#{str}#{delim}count_is"
      delim = ", "
    end
    if string then
      str = "#{str}#{delim}string"
      delim = ", "
    end
    if b_nullable then
      str = "#{str}#{delim}nullable"
      delim = ", "
    end
    cdl_error( "T1003 $1: unsuitable specifier for $2" , str, self.class )
  end

  def clear_max
    raise "clear_max called: #{self.class}"
  end

  def get_type_str
    str = ""
    if @b_const then
      str = "#{str}const "
    end
    if @b_volatile then
      str = "#{str}volatile "
    end
    return str
  end

  #=== 型をチェック
  #    正当な型定義かどうか、チェックする
  def check
    # 型に誤りがあれば、エラー文字列を返す
  end

  #=== struct の tag をチェック
  #    正当な型定義かどうか、チェックする
  #kind:: Decl の @kind を参照
  def check_struct_tag kind
    # tag が存在しなければエラーを出力する
    # 配列型では、要素の型を再帰的にチェック
    # ポインタ型では、指す先の tag チェックはしない
    # 関数型ではパラメータリストのすべてについて行う
  end

  #===  初期化可能かチェック
  #     attribute など初期化可能かチェックする（型に対し正当な初期化子が与えられているか）
  #ident::        string                被代入変数命
  #initialize::   Expression, Array of initializer or C_EXP
  #               代入値、C_EXP が与えられるのは IntType の場合のみ
  #kind::         symbol (:ATTRIBUTE, :VAR, :CONSTNAT )
  #attribute::    NameList              kind == :VAR のとき参照できる attribute
  #
  #     locale を第一引数として取るのは、以下の理由による。
  #     このメソッドは、変数への代入が行われる「行」に対して呼び出されるが、
  #     Type クラスのインスタンスは、変数が定義された「行」を記憶している。
  #
  # STAGE: S
  def check_init( locale, ident, initializer, kind, attribute = nil )
    #
  end

  #=== const_val を指定の型にキャストする
  # 派生クラスでオーバーライドしていないとエラー
  def cast( const_val )
    cdl_error( "T1004 cannot cast to $1" , self.class )
  end

  #=== 型が一致するかのチェック
  # 型名の字面でチェック．
  # typedef された型も字面で一致を見るため、元の型が同じでも型名が異なれば不一致となる
  def equal? type2
    return ( get_type_str == type2.get_type_str ) && ( get_type_str_post == type2.get_type_str_post )
  end

  #=== bit size を得る
  # IntType, FloatType 以外は0
  def get_bit_size
    return 0
  end

  #=== 元の型を得る
  # typedef された型の場合、その元の型を返す.
  # それ以外は、自分自身を返す．
  # (DefinedType では本メソッドがオーバーライドされる)
  def get_original_type
    return self
  end

  #=== 内部にポインタ型を持つ
  # ポインタ型、またはポインタ型メンバを持つ構造体、または要素がポインタ型を持つ配列
  def has_pointer?
    false
  end

  #=== size_is, count_is, string 指定されたポインタを持つか
  # size_is, count_is, string 指定されたポインタ型、またはそれをメンバに持つ構造体、またはそれをを要素に持つ配列
  def has_sized_pointer?
    false
  end
  
  #=== 長さ指定のない string を持つ
  # なさ指定のない string 指定されたポインタ型、またはそれをメンバに持つ構造体、またはそれを要素に持つ配列
  def has_unsized_string?
    false
  end

  def show_tree indent
    indent.times { print "  " }
    puts "const=#{@b_const} volatile=#{@b_volatile} #{locale_str}"
    indent.times { print "  " }
    puts "has_pointer=#{has_pointer?} has_sized_pointer=#{has_sized_pointer?} has_unsized_string=#{has_unsized_string?}"
  end
end

class DefinedType < Type
#  @type_name::string
#  @typedef::Typedef
#  @type:: kind_of Type

  include HasType

  def initialize( type_name )
    super()
    @type_name = type_name

    # mikan type_name が path になっていないため暫定
    @typedef = Namespace.find( [ type_name ] )  #1

#    if @type.class != Typedef then
#      raise NotTypedef
    #    end
    if @typedef == nil then
      cdl_error( "T1005 \'$1\' not defined" , type_name )
    elsif @typedef.class != Typedef then
      cdl_error( "T1006 \'$1\' not type name. expecting type name here" , type_name )
    end
    @type = @typedef.get_declarator.get_type
    initHasType
  end

  def get_type
    clone_type
    return @type
  end

  def get_type_str
    "#{super}#{@type_name}"
  end

  def get_type_str_post
    ""
  end

  def get_size
    return @type.get_size
  end

  def is_nullable?
    return @type.is_nullable?
  end

  #=== qualifier(const, volatile) の設定
  def set_qualifier( qualifier )
    clone_type
    @type.set_qualifier( qualifier )
    super
  end

  def set_scs( size, count, string, max = nil, b_nullable = false )
    clone_type
    @type.set_scs( size, count, string, max, b_nullable )
  end

  def clear_max
    @type.clear_max
  end

  def get_original_type
    clone_type
    return @type.get_original_type
  end

  def check	# 意味的誤りがあれば、文字列を返す
    nil    # typedef の段階で意味チェックされている
  end

  def check_init( locale, ident, initializer, kind, attribute = nil )
    get_type.check_init( locale, ident, initializer, kind, attribute )
  end

  #=== 内部にポインタ型を持つ
  # ポインタ型、またはポインタ型メンバを持つ構造体、または要素がポインタ型を持つ配列
  def has_pointer?
    @type.has_pointer?
  end

  #=== size_is, count_is, string 指定されたポインタを持つか
  # size_is, count_is, string 指定されたポインタ型、またはそれをメンバに持つ構造体、またはそれをを要素に持つ配列
  def has_sized_pointer?
    @type.has_sized_pointer?
  end
  
  #=== 長さ指定のない string を持つ
  # なさ指定のない string 指定されたポインタ型、またはそれをメンバに持つ構造体、またはそれを要素に持つ配列
  def has_unsized_string?
    @type.has_unsized_string?
  end

  def show_tree( indent )
    indent.times { print "  " }
    if @typedef == nil then
      puts "DefinedType: #{@type_name} is missing, const=#{@b_const} volatile=#{@b_volatile} #{locale_str}"
    else
      puts "DefinedType: #{@type_name}, const=#{@b_const} volatile=#{@b_volatile}"
    end
    super( indent + 1 )
    @typedef.show_tree( indent + 1 )
    @type.show_tree( indent + 1 )
  end
end

class VoidType < Type

  def check	# 意味的誤りがあれば、文字列を返す
    nil
  end

  def check_init( locale, ident, initializer, kind, attribute = nil )
    cdl_error2( locale, "T1007 $1: void type variable cannot have initializer" , ident )
  end

  def get_type_str
    "#{super}void"
  end

  def get_type_str_post
    ""
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "VoidType #{locale_str}"
    super( indent + 1 )
  end
end

class BoolType < Type

  def check	# 意味的誤りがあれば、文字列を返す
    nil
  end

  def get_type_str
    "#{super}bool_t"
  end

  def get_type_str_post
    ""
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "BoolType #{locale_str}"
    super( indent + 1 )
  end
end

class IntType < Type
#  @bit_size::		-11: char, -1: char_t, -2: short, -3: int, -4: long, -5: long long
#			8, 16, 32, 64, 128
#  @sign::		:SIGNED, :UNSIGNED, nil

  def initialize( bit_size )
    super()
    @bit_size = bit_size
    @sign = nil
  end

  def set_sign( sign, b_uint = false )
    if @sign then
      if @sign != sign then
        cdl_error( "T1008 ambigous signed or unsigned"   )
      end
    elsif b_uint == false && @bit_size > 0 then
      if sign == :SIGNED then
        cdl_warning( "W2001 signed int$1_t: obsolete. use int$2_t" , @bit_size, @bit_size )
      else
        cdl_warning( "W2002 unsinged int$1_t: obsolete. use uint$2_t" , @bit_size, @bit_size )
      end
    end

    @sign = sign

    if @sign != :SIGNED && @sign != :UNSIGNED then
      raise "set_sign: unknown sign: #{@sign}"
    end
  end

  def check	# 意味的誤りがあれば、文字列を返す
    nil
  end

  def check_init( locale, ident, initializer, kind, attribute = nil )
    val = initializer  # C_EXP, Array
    if val.instance_of?( Expression ) then
      val = val.eval_const2( nil, attribute )
      # 評価の結果 C_EXP や Array となる可能性がある
    end

    if val.instance_of? Token then    # StringVal 導入により、もはや Token は来ないはず
      # val が Token の場合 == の右辺が String だとエラーを起こす (#198)
      cdl_error2( locale, "T1009 $1: $2: not integer" , ident, val )
      return
    elsif val.is_a? C_EXP then
      # #192 var が attribute を参照し、attribute の右辺が C_EXP の場合
      # const の右辺が C_EXP の場合も
      return
    elsif val.kind_of? FloatVal then
      cdl_error2( locale, "T1011 $1: need cast to assign float to integer" , ident )
      return
    elsif val.instance_of? Array then
      cdl_error2( locale, "T1017 $1: unsuitable initializer for scalar type" , ident )
      return
    elsif val == nil then
      cdl_error2( locale, "T1010 $1: initializer is not constant" , ident )
      return
    end

    if ! val.kind_of? IntegerVal then
      cdl_error2( locale, "T1012 $1: $2: not integer" , ident, val )
      return
    end

    val = val.to_i
    max = get_max
    min = get_min
    dbgPrint "sign=#{@sign} ident=#{ident} val=#{val} max=#{max} min=#{min}\n"

    if max != nil then
      if val > max then
        if @sign == :SIGNED || @sign == nil then
          cdl_error2( locale, "T1013 $1: too large (max=$2)" , ident, max )
        else
          cdl_error2( locale, "T1016 $1: too large (max=$2)" , ident, max )
        end
      end
    end

    if min != nil then
      if val < min
        if @sign == :SIGNED || @sign == nil then
          cdl_error2( locale, "T1014 $1: too large negative value (min=$2)" , ident, min )
        else
          cdl_error2( locale, "T1015 $1: negative value for unsigned" , ident )
        end
      end
    end
  end

  #=== IntType# 最大値、最小値をチェックしてクリップする
  # キャスト演算を行う
  #in_val:: IntegerVal, FloatVal:  この型にキャストする値
  #from_type:: Symbol:  :IntType, :FloatType  IntType の場合はビット数でクリップ、FloatType の場合は最大値でクリップ
  def check_and_clip( in_val, from_type = :IntType )
    bit_size = get_bit_size
    if bit_size == -1 then
      bit_size = 8
    end
    val = in_val.to_i
    if get_max && val > get_max then
      if from_type == :IntType then
        rval = ((1 << bit_size)-1) & val   # bit 数でクリップ
      else
        rval = get_max                         # 最大値でクリップ (FloatType)
      end
      cdl_warning( "W2003 $1: too large to cast to $2, clipped($3)" , in_val, get_type_str, rval )
    elsif get_min && val < get_min then
      if from_type == :IntType then
        rval = ((1 << bit_size)-1) & val
      else
        rval = get_min
      end
      if @sign == :SIGNED || @sign == nil then
        cdl_warning( "W2004 $1: too small to cast to $2, clipped($3)" , in_val, get_type_str, rval )
      else    # @sign == :UNSIGNED || @sign == nil (char の場合)
        cdl_warning( "W2005 $1: negative value for unsigned: convert to $2" , in_val, rval )
      end
    else
      rval = val
    end
    return rval
  end

  def get_min
    if @sign == :SIGNED || @sign == nil then
      if @bit_size == -1 then
        bit_sz = 8   # char_t は、有符号に扱う
      else
        bit_sz = @bit_size
      end
      case bit_sz
      when 8, 16, 32, 64, 128
        return  - ( 1 << ( bit_sz - 1 ))
      else # -1, -2, -3, -4, -5, -11
        return nil
      end
    else   # @sign == :UNSIGNED
      return 0
    end
  end

  def get_max
    if @bit_size == -1 then
      if @sign == nil then
        return 255   # char_t は、無符号に扱う
      else
        bit_sz = 8
      end
    else
      bit_sz = @bit_size
    end
    if @sign == :SIGNED || @sign == nil then
      case bit_sz
      when 8, 16, 32, 64, 128
        return ( 1 << ( bit_sz - 1 )) -1
      else # -1, -2, -3, -4, -5, -11
        return nil
      end
    else   # @sign == :UNSIGNED
      case bit_sz
      when 8, 16, 32, 64, 128
        return ( 1 << bit_sz ) - 1
      else # -2, -3, -4, -5, -11
        return nil
      end
    end
  end

  #=== IntType# C 言語における型名（修飾子付き）
  def get_type_str
    str = super

    # NEW_MODE
    case @sign
    when :SIGNED
      sign = ""
      signL = "signed "
    when :UNSIGNED
      sign = "u"
      signL = "unsigned "
    else
      sign = ""
      signL = ""
    end

    # p "get_type_str: sign:#{@sign} signL=#{signL}"

    case @bit_size
    when -1      # char_t 型
      if @sign == :SIGNED then
        sign = "s"
      end
      str = "#{str}#{sign}char_t"
    when -11     # char 型(obsolete)
      str = "#{str}#{signL}char"
    when -2      # short 型
      str = "#{str}#{signL}short"
    when -3      # int 型
      str = "#{str}#{signL}int"
    when -4      # long 型
      str = "#{str}#{signL}long"
    when -5      # long long 型
      str = "#{str}#{signL}long long"
    when 8, 16, 32, 64, 128     # int16, int32, int64, int128 型
      str = "#{str}#{sign}int#{@bit_size}_t"
    end

    return str
  end

  #=== IntType# C 言語における型名（後置文字列）
  def get_type_str_post
    ""
  end

  #=== IntType#bit_size を得る
  #    返される値は @bit_size の仕様を参照
  def get_bit_size
    return @bit_size
  end

  #=== IntType# sign を得る
  # @sign の説明を参照
  def get_sign
    @sign
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "IntType bit_size=#{@bit_size} sign=#{@sign} const=#{@b_const} volatile=#{@b_volatile} #{locale_str}"
    super( indent + 1 )
  end
end

class FloatType < Type
#  @bit_size::         32, 64, (80), -32, -64

  def initialize( bit_size )
    super()
    @bit_size = bit_size
  end

  def check	# 意味的誤りがあれば、文字列を返す
    nil
  end

  # mikan Float 型の C_EXP 対応 (generate.rb にも変更必要)
  def check_init( locale, ident, initializer, kind, attribute = nil )
    # 型に対する初期値に誤りがあれば、エラー文字列を返す
    val = initializer
    if val.instance_of?( Expression ) then
      val = val.eval_const2( nil, attribute )
      # 評価の結果 C_EXP や Array となる可能性がある
    end

    if val.instance_of? Token then
      # val が Token の場合 == の右辺が String だとエラーを起こす
      cdl_error2( locale, "T1018 $1: $2: not number" , ident, val )
      return
    elsif val.instance_of? Array then
      cdl_error2( locale, "T1020 $1: unsuitable initializer for scalar type" , ident )
      return
    elsif val.instance_of? C_EXP then
      return
    elsif val == nil then
      cdl_error2( locale, "T1019 $1: initializer is not constant" , ident )
      return
    elsif ! val.kind_of?( IntegerVal ) && ! val.kind_of?( FloatVal ) then
      cdl_error2( locale, "T1037 $1: not number" , ident )
      return
    end
    # else
    #   cdl_error2( locale, "T1020 $1: unsuitable initializer for scalar type" , ident )
    #   return
    # end
    return
  end

  def get_type_str
    str = super

    case @bit_size
    when 32
      str = "#{str}float32_t"
    when 64
      str = "#{str}double64_t"
    when -32
      str = "#{str}float"
    when -64
      str = "#{str}double"
    end
    return str
  end

  def get_type_str_post
    ""
  end

  def get_bit_size
    @bit_size
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "FloatType bit_size=#{@bit_size} qualifier=#{@qualifier} #{locale_str}"
    super( indent + 1 )
  end
end

class EnumType < Type # mikan
#  @bit_size::		-1: enum
#			8, 16, 32, 64, 128
#  @element::		[]
#  @element_val::	[]

  def initialize( bit_size )
    super()
    @bit_size = bit_size
  end

  def check
   # mikan enum check
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "EnumType bit_size=#{@bit_size} qualifier=#{@qualifier} #{locale_str}"
    super( indent + 1 )
    # mikan element
  end
end

class StructType < Type
#  @tag::
#  @b_define::  true if define, false if refer
#  @members_decl:: NamedList
#  @definition:: StructType
#  @b_has_pointer_member:: bool : メンバにポインタ型がある
#  @b_has_sized_pointer_member:: bool : メンバにポインタ型がある
#  @b_has_unsized_string_member:: bool : メンバにポインタ型がある
#  @b_hasTag:: bool : タグがある
#  @member_types_symbol:: Symbol : tag が無い時のみ設定 (それ以外では nil)

  @@structtype_current_stack = []
  @@structtype_current_sp = -1

  # tag なし struct
  @@no_struct_tag_num = 0
  @@no_tag_struct_list = {}

  def initialize( tag = nil )
    super()
    @tag = tag
    if tag then
      @b_hasTag = true
    else
      @b_hasTag = false
    end
    @@structtype_current_sp += 1
    @@structtype_current_stack[@@structtype_current_sp] = self
    @b_has_pointer_member = false
    @b_has_sized_pointer_member = false
    @b_has_unsized_string_member = false
    @member_types_symbol = nil
  end

  def self.set_define( b_define )
    @@structtype_current_stack[@@structtype_current_sp].set_define( b_define )
  end

  def set_define( b_define )
    @b_define = b_define
    if @b_define then
      @members_decl = NamedList.new( nil, "in struct #{@tag}" )
      # if @tag then    登録タイミングを終わりに変更 V1.0.2.19
      #  Namespace.new_structtype( self )
      # end
    else
      @definition = Namespace.find_tag( @tag )
      # check_struct_tag に移す V1.0.2.19
      # if @definition == nil then
      #  cdl_error( "T1021 \'$1\': struct not defined" , @tag )
      # end
    end
  end

  def self.new_member( member_decl )
    @@structtype_current_stack[@@structtype_current_sp].new_member( member_decl )
  end

  def new_member( member_decl )
    member_decl.set_owner self   # Decl (StructType)
    @members_decl.add_item( member_decl )
    if member_decl.get_type.has_pointer?
      @b_has_pointer_member = true
    end
    if member_decl.get_type.has_sized_pointer?
      @b_has_sized_pointer_member = true
    end
    if member_decl.get_type.has_unsized_string?
      @b_has_unsized_string_member = true
    end
  end

  def check	# 意味的誤りがあれば、文字列を返す
    nil
  end

  #=== 構造体のタグをチェック
  #  declarator の時点でチェックする
  #kind:: Decl の @kind を参照
  def check_struct_tag kind
    if @tag == nil
      return
    end

    st = Namespace.find_tag( @tag )
    if st == nil then
      cdl_error( "T1022 struct $1: not defined" , @tag )
    end
  end

  # mikan Float 型の C_EXP 対応 (generate.rb にも変更必要)
  def check_init( locale, ident, initializer, kind, attribute = nil )

    st = Namespace.find_tag( @tag )
    if st == nil then
      cdl_error2( locale, "T1023 struct $1: not defined" , @tag )
      return
    end

    # 初期化子が式の場合、型（タグ）が一致するかチェック
    if initializer.instance_of?( Expression ) then
      t = initializer.get_type( attribute )
      # print "Check init #{t.class} #{t.get_name}\n"
      if ! t.kind_of?( StructType ) then
        if t then
          str = t.get_type_str
        else
          str = "unknown"
        end
        cdl_error2( locale, "T1038 $1: initializer type mismatch. '$2' & '$3'" , ident, get_type_str, str )
      elsif @tag != t.get_name then
        cdl_error2( locale, "T1039 $1: struct tag mismatch $2 and $3" , ident, @tag, t.get_name )
      end
      initializer = initializer.eval_const( attribute )
    end

    if initializer.instance_of?( Array ) then
      i = 0
      st.get_members_decl.get_items.each { |d|
        if initializer[i] then
          d.get_type.check_init( locale, "#{ident}.#{d.get_identifier}", initializer[i], kind )
        end
        i += 1
      }
    else
      cdl_error2( locale, "T1024 $1: unsuitable initializer for struct" , ident )
    end
  end

  def self.end_of_parse()
    @@structtype_current_stack[@@structtype_current_sp].end_of_parse
    @@structtype_current_sp -= 1
  end

  def end_of_parse()
    if @members_decl == nil   # @b_define = false またはメンバーのない構造体（エラー）
      return
    end
    @members_decl.get_items.each{ |md|
      size = md.get_size_is
      if size then
        val = size.eval_const( @members_decl )
        if val == nil then
          type = size.get_type( @members_decl )
          if ! type.kind_of?( IntType ) then
            cdl_error( "T1025 size_is argument is not integer type"  )
          end
        end
      end
      count = md.get_count_is
      if count then
        val = count.eval_const( @members_decl )
        if val == nil then
          type = count.get_type( @members_decl )
          if ! type.kind_of?( IntType ) then
            cdl_error( "T1026 count_is argument is not integer type"  )
          end
        end
      end
      string = md.get_string
      if string == -1 then
        # 長さ指定なし
      elsif string then
        val = string.eval_const( @members_decl )
        if val == nil then
          type = string.get_type( @members_decl )
          if ! type.kind_of?( IntType ) then
            cdl_error( "T1027 string argument is not integer type"  )
          end
        end
      end
    }

    if @tag == nil then
      @member_types_symbol = get_member_types_symbol
      # print "member_types_symbol = #{get_member_types_symbol}\n"
      if @@no_tag_struct_list[ @member_types_symbol ] then
        @tag = @@no_tag_struct_list[ @member_types_symbol ]
      else
        @tag = :"TAG_#{@@no_struct_tag_num}_TECS_internal__"
	    @@no_struct_tag_num += 1
        @@no_tag_struct_list[ @member_types_symbol ] = @tag
        Namespace.new_structtype( self )
      end
    else
      if @b_define then
        Namespace.new_structtype( self )
      end
    end
  end

  def get_name
    @tag
  end

  def get_type_str      # mikan struct get_type_str
    str = super

    if @b_hasTag then
      # typedef struct tag StructType; の形式の場合
      # struct の本体は、別に生成される
      return "#{str}struct #{@tag}"

    else
      # typedef struct { int a; } StructType; の形式の場合
      str += "struct {"
      @members_decl.get_items.each{ |i|
        str += sprintf( "%s %s%s;", "#{i.get_type.get_type_str}", "#{i.get_name}", "#{i.get_type.get_type_str_post}" )
      }
      str += "} "

      return str

    end
  end

  def get_type_str_post
    ""
  end

  def get_members_decl
    return @members_decl if @members_decl

    st = Namespace.find_tag( @tag )
    if st then
      return st.get_members_decl
    end

    return nil
  end

  def has_pointer?
    if @definition
      return @definition.has_pointer?
    else
      return @b_has_pointer_member
    end
  end

  def has_sized_pointer?
    if @definition
      return @definition.has_sized_pointer?
    else
      return @b_has_sized_pointer_member
    end
  end
  
  def has_unsized_string?
    if @definition
      return @definition.has_unsized_string?
    else
      return @b_has_unsized_string_member
    end
  end

  #=== 同じ構造体かどうかチェックする
  # tag のチェックは行わない
  # すべてのメンバの名前と型が一致することを確認する
  def same? another
    md = another.get_members_decl
    if @members_decl == nil || md == nil
      return false
    end

    md1 = @members_decl.get_items
    md2 = md.get_items
    if( md1.length != md2.length )
      return false
    end

    i = 0
    while i < md1.length
      if md1[i].get_name != md2[i].get_name ||
          md1[i].get_type.get_type_str != md2[i].get_type.get_type_str ||
          md1[i].get_type.get_type_str_post != md2[i].get_type.get_type_str_post
        return false
      end
      i += 1
    end

    return true
  end
  
  def get_member_types_symbol
    mts = ''
    @members_decl.get_items.each { |member|
      mts += member.get_type.get_type_str + member.get_type.get_type_str_post + ';'
    }
    return mts.to_sym
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "StructType tag: #{@tag} qualifier=#{@qualifier} has_pointer=#{@b_has_pointer_member} #{locale_str}"
    super( indent + 1 )
    if @b_define then
      @members_decl.show_tree( indent + 1 )
    end
  end
end

class FuncType < Type
#  @paramlist::  ParamList
#  @type:: return type : PtrType, ArrayType, FuncType, IntType, FloatType, ...
#  @b_oneway:: bool: true, false
#  @has_in:: bool :  has [in] parameter
#  @has_inout:: bool : has [inout] parameter
#  @has_out:: bool : has [out] parameter
#  @has_send:: bool : has [send] parameter
#  @has_receive:: bool : has [receive] parameter

  def initialize( paramlist = nil )
    super()

    @has_in = false
    @has_inout = false
    @has_out = false
    @has_send = false
    @has_receive = false

    @paramlist = paramlist
    @b_oneway = false
    if paramlist then
      paramlist.check_param
    else
      @paramlist = ParamList.new( nil )
    end
    @paramlist.set_owner self  # ParamList
    @paramlist.get_items.each{ |p|
      case p.get_direction
      when :IN
        @has_in = true
      when :INOUT
        @has_inout = true
      when :OUT
        @has_out = true
      when :SEND
        @has_send = true
      when :RECEIVE
        @has_receive = true
      else
        raise "unkown direction"
      end
    }

  end

  def check	# 意味的誤りがあれば、文字列を返す
    if @type.class == ArrayType then	# 配列を返す関数
      return "function returning array"
    elsif @type.class == FuncType then	# 関数を返す関数
      return "function returning function"
    end
    return @type.check   # 関数の return する型のチェック

    # パラメータの型のチェックは ParamList#check_param で行う
  end

  def check_struct_tag kind
    @type.check_struct_tag kind
    # ParamDecl でもチェックされるので、ここではチェックしない
    # @paramlist.check_struct_tag kind
  end

  def check_init( locale, ident, initializer, kind, attribute = nil )
    cdl_error2( locale, "T1028 $1: cannot initialize function pointer" , ident )
    return
  end

  def set_type( type )
    unless @type then
      @type = type
    else
      @type.set_type( type )
    end
  end

  #=== return type を返す
  #
  # return type を返す
  # get_return_type とすべきだった
  def get_type
    @type
  end

  def get_type_str
    return @type.get_type_str
  end

  def get_type_str_post
    # 型だけを返す (仮引数の名前を含めない)
    @paramlist.to_str( false )
  end

  def get_paramlist
    @paramlist
  end

  def set_oneway( b_oneway )
    @b_oneway = b_oneway

    if ( ( @type.get_type_str != "ER" && @type.get_type_str != "void" ) || @type.get_type_str_post != "" ) then
      cdl_error( "T1029 oneway function cannot return type \'$1$2\', \'void\' or \'ER\' is permitted" , @type.get_type_str, @type.get_type_str_post )
    end

    if @paramlist then
      @paramlist.get_items.each{ |p|
        if p.get_direction != :IN && p.get_direction != :SEND then
          cdl_error( "T1030 oneway function cannot have $1 parameter for \'$2\'" , p.get_direction, p.get_name )
        end
      }
    end
  end

  def is_oneway?
    @b_oneway
  end

  #=== Push Pop Allocator が必要か？
  # Transparent RPC の場合 oneway かつ in の配列(size_is, count_is, string のいずれかで修飾）がある
  def need_PPAllocator?( b_opaque = false )
    if @b_oneway || b_opaque then
      return @paramlist.need_PPAllocator?( b_opaque )
    else
      return false
    end
  end

  #=== パラメータが in, inout, out, send, receive を持つか
  def has_in?
    @has_in
  end
  def has_inout?
    @has_inout
  end
  def has_out?
    @has_out
  end
  def has_send?
    @has_send
  end
  def has_receive?
    @has_receive
  end

  #=== 入力方向のパラメータを持つか
  def has_inward?
    @has_in || @has_inout || @has_send
  end
  #=== 出力方向のパラメータを持つか
  def has_outward?
    @has_inout || @has_out || @has_receive
  end

  def show_tree( indent )
    indent.times { print "  " }
    if @b_oneway then
      puts "FunctType:  oneway=true #{locale_str}"
    else
      puts "FunctType:  oneway=false #{locale_str}"
    end
    super( indent + 1 )
    if @paramlist then
      @paramlist.show_tree( indent + 1 )
    end
    (indent+1).times { print "  " }
    puts "return type:"
    @type.show_tree( indent + 2 )
  end
end

class ArrayType < Type
#  @type:: element type :  ArrayType, FuncType, IntType, FloatType, ...
#  @subscript:: Expression, nil if '[]'

  include HasType

  def initialize( subscript = nil )
    super()
    @subscript = subscript
    initHasType
  end

  def set_type( type )
    unless @type then
      @type = type
    else
      @type.set_type( type )
    end
  end

  #=== Array#qualifier(const, volatile) の設定
  def set_qualifier( qualifier )
    clone_type
    @type.set_qualifier( qualifier )
    super
  end

  # 配列要素が const なら const
  def is_const?
    @type.is_const?
  end

  # 配列要素が volatile なら volatile
  def is_volatile?
    @type.is_volatile?
  end

  def get_type
    @type
  end

  def get_subscript
    @subscript
  end

  def get_type_str
    return "#{@type.get_type_str}"
  end

  def get_type_str_post
    if @subscript
      "[#{@subscript.eval_const(nil)}]#{@type.get_type_str_post}"
    else
      "[]#{@type.get_type_str_post}"
    end
    # "[#{@subscript.to_s}]#{@type.get_type_str_post}"
  end

  def check	# 意味的誤りがあれば、文字列を返す
    if @type.class == FuncType then		# 関数の配列
      return "array of function"
    elsif @type.class == ArrayType then	# 添数なし配列の配列
      unless @type.get_subscript then
        return "subscript not specified"
      end
    end

    return @type.check    # 配列要素の型をチェック
  end

  def check_struct_tag kind
    @type.check_struct_tag kind
  end

  def check_init( locale, ident, initializer, kind, attribute = nil )
    if ( initializer.instance_of?( Array ) ) then
      # 要素数が指定されている場合、初期化要素数をチェック
      if @subscript then
        n_sub = @subscript.eval_const( nil )
        if n_sub then
          if initializer.length > n_sub then
            cdl_error2( locale, "T9999 $1: too many initializer, $2 for $3" , ident, initializer.length, n_sub )
          end
        end
      end
      index = 0
      initializer.each{ |i|
        @type.check_init( locale, "#{ident}[#{index}]", i, kind, attribute = nil )
        index += 1
      }
    else
      cdl_error2( locale, "T1031 $1: unsuitable initializer for array" , ident )
    end
  end

  def has_pointer?
    @type.has_pointer?
  end

  def has_sized_pointer?
    @type.has_sized_pointer?
  end

  def has_unsized_string?
    @type.has_unsized_string?
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "ArrayType: #{locale_str}"
    super( indent + 1 )
    (indent+1).times { print "  " }
    puts "type:"
    @type.show_tree( indent + 2 )
    (indent+1).times { print "  " }
    puts "subscript:"
    if @subscript then
      @subscript.show_tree( indent + 2 )
    else
      (indent+2).times { print "  " }
      puts "no subscript"
    end
  end
end

class PtrType < Type
#  @type:: refer to : PtrType, FuncType, ArrayType, IntType, FloatType, ...
#  @size:: Expr, or nil if not specified
#  @count:: Expr, or nil if not specified
#  @string:: Expr or -1(if size not specified) （string 引数）, or nil if not specified

  include HasType

  def initialize( referto = nil )
    super()
    @type   = referto
    @size   = nil
    @count  = nil
    @string = nil
    initHasType
  end

  def set_type( type )
    unless @type then
      @type = type
    else
      @type.set_type( type )	# 枝先の type を設定
    end
  end

  def get_type_str
    if @type.kind_of?( ArrayType ) || @type.kind_of?( FuncType ) then
      parenthes = "("
    else
      parenthes = ""
    end
    return "#{@type.get_type_str}#{parenthes}*"
  end

  def get_type_str_post
    if @type.kind_of?( ArrayType ) || @type.kind_of?( FuncType ) then
      parenthes = ")"
    else
      parenthes = ""
    end
    "#{parenthes}#{@type.get_type_str_post}"
  end

  def check	# 意味的誤りがあれば、文字列を返す
    return nil if @type == nil
    @type.check
  end

  def check_struct_tag kind
    if kind != :MEMBER  # 構造体メンバーの場合、ポインタの先の構造体タグをチェックしない
      @type.check_struct_tag kind
    end
  end

  def check_init( locale, ident, initializer, kind, attribute = nil )
    if ( initializer.instance_of?( Expression ) ) then
      val = initializer.eval_const2( nil, attribute )
      if val.kind_of? PointerVal then
        type = val.get_type  # PtrType
        t1 = self
        t2 = type
        while( t1.kind_of?( PtrType ) && t2.kind_of?( PtrType ) )
          t1 = t1.get_type
          t2 = t2.get_type
          if ( t1.class == t2.class ) && ( t1.get_bit_size == t2.get_bit_size ) then
          elsif ( t1.kind_of?( CDefinedType) || t2.kind_of?( CDefinedType ) )&& t1.get_type_str == t2.get_type_str && t1.get_type_str_post && t2.get_type_str_post then
            # int8_t などが、一方は .h に定義されているケース
          else
            cdl_error2( locale, "T1032 $1: incompatible pointer type" , ident )
            break
          end
        end
      elsif val.kind_of? IntegerVal then
        if val.to_i != 0 then
          cdl_error2( locale, "T1033 $1: need cast to assign integer to pointer" , ident )
        end
      elsif val.kind_of? StringVal then
        # 文字列定数
        # mikan L"wide string"
        if @type.get_bit_size != -1 && @type.get_bit_size != -11 then  # -1: char_t
          cdl_error2( locale, "T1034 $1: unsuitable string constant" , ident )
        end
      elsif ( val.instance_of?( Array ) ) then
        i = 0
        val.each { |ini|
          @type.check_init( locale, "#{ident}[#{i}]", ini, kind, attribute = nil )
          i += 1
        }
      elsif val.instance_of?( C_EXP ) then

      else
        cdl_error2( locale, "T1035 $1: unsuitable initializer for pointer" , ident )
      end
    elsif ( initializer.instance_of?( Array ) ) then
      if @size == nil && @count == nil then
        cdl_error2( locale, "T9999 $1: non-size_is pointer cannot have array initializer", ident )
      end

      i = 0
      initializer.each { |ini|
        @type.check_init( locale, "#{ident}[#{i}]", ini, kind, attribute = nil )
        i += 1
      }
    elsif( initializer.instance_of?( C_EXP ) ) then

    else
      cdl_error2( locale, "T1036 $1: unsuitable initializer for pointer" , ident )
    end
  end

  def get_referto
    clone_type
    @type
  end

  def set_scs( size, count, string, max, b_nullable )
    @size = size
    @count = count
    @max = max
    @b_nullable = b_nullable

    # string は最も左側の ptr に作用する
    if @type.kind_of?( PtrType ) then
      # ptr_level が 2 以上であることは ParamDecl#initializer でチェックされる
      clone_type
      @type.set_scs( nil, nil, string, nil, false )
    elsif @type.kind_of?( VoidType ) && ( size || count || string ) then
      str = ""
      if size then
        str = "size_is"
      end
      if count then
        if str then
          str += ", "
        end
        str += "count_is"
      end
      if string then
        if str then
          str += ", "
        end
        str += "string"
      end

      cdl_error( "T1040 $1 specified for void pointer type", str )
    else
      @string = string
    end

    if (@size != nil) && (@b_nullable != false) then
      cdl_error( "T9999 size_is & nullable cannot be specified simultaneously. If size is zero, pointer must be null")
    end
  end

  def clear_max
    @max = nil
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

  #=== PtrType# size_is の最大値
  def get_max
    @max
  end

  def is_nullable?
    return @b_nullable
  end

  def get_type
    clone_type
    @type
  end

  def has_pointer?
    true
  end

  def has_sized_pointer?
    @size != nil || @count != nil || @string.instance_of?( Expression ) || @type.has_sized_pointer?
  end

  def has_unsized_string?
    @string == -1 || @type.has_unsized_string?
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "PtrType: qualifier=#{@qualifier}, nullable=#{@b_nullable} #{locale_str}"
    super( indent + 1 )
    (indent+1).times { print "  " }
    if @size then
      print "size=#{@size.to_s}, "
    else
      print "size=nil, "
    end
    if @max then
      print "max=#{@size.to_s}, "
    else
      print "max=nil, "
    end
    if @count then
      print "count=#{@count.to_s}, "
    else
      print "count=nil, "
    end
    if @string then
      if @string.instance_of?( Expression ) then
        print "string=#{@string.to_s}\n"
      else
        print "string=yes\n"
      end
    else
      print "string=nil\n"
    end

    (indent+1).times { print "  " }
    puts "type:"
    @type.show_tree( indent + 2 )
  end

end

#==  DescriptorType クラス
# 動的結合で渡すデスクリプタ型
class DescriptorType < Type
# @sinagure_nsp::NamespacePath

  def initialize( signature_nsp )
    # p "Desc #{signature_nsp.to_s}"
    obj = Namespace.find signature_nsp
    if ! obj.kind_of? Signature then
      cdl_error( "T9999 '$1': not signature or not found", signature_nsp.to_s )
      @signature_nsp = signature_nsp
    else
      if obj.has_descriptor? then
       # cdl_error( "T9999 '$1': has Descriptor in function parameter", signature_nsp.to_s )
      end
      @signature_nsp = obj.get_namespace_path
    end
  end

  def get_type_str
    "Descriptor( #{@signature_nsp.get_global_name} )"
  end

  def get_type_str_post
    ""
  end

  def set_qualifier( qualifier )
    cdl_error( "T9999 '$1' cannot be specified for Descriptor", qualfier.to_s )
  end

  def check
  end

  def check_init( locale, ident, initializer, kind, attribute = nil )
    case kind
    when :PARAMETER
      # 引数は初期化できない
    else
      cdl_error2( locale, "T9999 Descriptor cannot be used for $1", kind)
    end
  end

  #== DescriptorType#
  def get_signature
    Namespace.find @signature_nsp
  end
end

# 以下単体テストコード
if $unit_test then
  puts( "===== Unit Test: IntType ===== (types.rb)")
  sizes = [ 8, 16, 32, 64 ]
  sizes.each{ |n|
    int = IntType.new n
    printf( "%8s  max: %d  min:%d\n", "int#{n}_t", int.get_max, int.get_min )
  }
  puts ""
end
