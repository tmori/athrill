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
#   $Id: value.rb 2061 2014-05-31 22:15:33Z okuma-top $
#++

#= BaseVal 整数、浮動小数などの値を扱うクラスの基底クラス
#
# TECS の CDL で扱う値は、以下に分類される
# ・整数
# ・浮動小数
# ・文字列
# ・ブール値
# 集成型（構造体、配列）と C_EXP はここでは扱わない
#
# このクラスで定義済みの演算子は、エラーとなる
# 型により演算可能な場合、演算子をオーバーライドする
#
class BaseVal < Node
  def ~@
    unsupport "~"
  end
  def -@
    unsupport "unary -"
  end
  def +@
    unsupport "unary +"
  end
  def not # ! val
    unsupport "!"
  end

  def * val
    unsupport "*"
  end
  def / val
    unsupport "/"
  end
  def % val
    unsupport "%"
  end
  def + val
    unsupport "+"
  end
  def - val
    unsupport "-"
  end
  def << val
    unsupport "<<"
  end
  def >> val
    unsupport ">>"
  end
  def > val
    unsupport ">"
  end
  def < val
    unsupport "<"
  end
  def >= val
    unsupport ">="
  end
  def <= val
    unsupport "<="
  end
  def eq val # == val
    unsupport "=="
  end
  def neq val # != val
    unsupport "!="
  end
  def & val
    unsupport "&"
  end
  def ^ val
    unsupport "^"
  end
  def | val
    unsupport "|"
  end
  def lAND val  # && val
    unsupport "&&"
  end
  def lOR val   # || val
    unsupport "||"
  end
  def cast( type )
    unsupport "CAST"
  end

  def unsupport op
    cdl_error( "V1001 $1: unable for $2" , op, self.class )
  end

  def to_s
    raise "to_s not overridden"
  end

  def to_b
    cdl_error( "V1002 $1: cannot cast to bool (implicitly)" , self.class )
    false
  end
  def to_i
    cdl_error( "V1003 $1: cannot cast to integer (implicitly)" , self.class )
    1
  end
  def to_f
    cdl_error( "V1004 $1: cannot cast to float (implicitly)" , self.class )
    1.0
  end
end

#= Pointer 値 (IntegerVal の Pointer 版)
#
# ポインタ値は、CDL で直接生成されることはない
# 整数値のキャスト演算により生成される
class PointerVal < BaseVal
#@int_val:: IntegerVal: IntegerVal でなくてはならない
#@ptr_type:: PtrType: ポインタの指す先の型

  def initialize( int_val, ptr_type )
    super()
    @int_val = int_val
    @ptr_type = ptr_type
  end

  #=== ポインタの指す先の型を得る
  # PointerVal 専用のメソッド
  def get_type
    @ptr_type
  end

  def cast type
    t = type.get_original_type   # typedef の元を得る
    if t.kind_of? IntType then
      val = t.check_and_clip( @int_val, :IntType )
      return IntegerVal.new( val )
    elsif t.kind_of? FloatType then
      cdl_error( "V1005 Cannot cast pointer to float"  )
      return FloatVal.new( @int_val )
    elsif t.kind_of? PtrType then
      return PointerVal.new( @int_val, type )
    else
      cdl_error( "V1006 pointer value cannot cast to $1" , type.class )
      return nil
    end
  end

  def to_s
    "(#{@ptr_type.get_type_str}#{@ptr_type.get_type_str_post})#{sprintf("0x%X", @int_val)}"
  end

  def to_b
    cdl_error( "V1007 convert pointer value to bool"  )
    false
  end
  def to_i
    cdl_error( "V1008 convert pointer value to integer without cast"  )
    @val.to_i
  end
end

#= IntegerVal: 整数値を扱うクラス
class IntegerVal < BaseVal
#@val:: Integer: value
#@str:: string: literal
#@sign:: Symbol: :SIGNED | :UNSIGNED
#@size:: Symbol: :NORMAL | :SHORT | :LONG | :LONGLONG

  def initialize( val, str = nil, sign = :SIGNED, size = :NORMAL )
    super()
    @val = val.to_i
    @str = str
    @sign = sign
    @size = size
  end

  def ~@
    IntegerVal.new( ~ @val )
  end
  def -@
    IntegerVal.new( - @val )
  end
  def +@
    self
  end
  def not # !
      BoolVal.new( self.to_b )
  end

  def * val
    if val.kind_of? FloatVal then
      return FloatVal.new( @val.to_f * val.to_f )
    else
      return IntegerVal.new( @val * val.to_i )
    end
  end
  def / val
    if val.kind_of? FloatVal then
      v2 = val.to_f   # to_f を2回評価しない
      if v2 == 0.0 then
        cdl_error( "V1009 / : divieded by zero"  )
        return FloatVal.new( 1.0 )
      end
      return FloatVal.new( @val.to_f / v2 )
    else
      v2 = val.to_i   # to_i を2回評価しない
      if v2 == 0 then
        cdl_error( "V1010 / : divieded by zero"  )
        return IntegerVal.new( 1 )
      end
      return IntegerVal.new( @val / v2 )
    end
  end
  def % val
    if val.kind_of? FloatVal then
      v2 = val.to_f   # to_f を2回評価しない
      if v2 == 0.0 then
        cdl_error( "V1011 % : divieded by zero"  )
        return FloatVal.new( 1.0 )
      end
      return FloatVal.new( @val.to_f % v2 )
    else
      v2 = val.to_i   # to_i を2回評価しない
      if v2 == 0 then
        cdl_error( "V1012 % : divieded by zero"  )
        return IntegerVal.new( 1 )
      end
      return IntegerVal.new( @val % v2 )
    end
  end
  def + val
    if val.kind_of? FloatVal then
      return FloatVal.new( @val.to_f + val.to_f )
    else
      return IntegerVal.new( @val + val.to_i )
    end
  end
  def - val
    if val.kind_of? FloatVal then
      return FloatVal.new( @val.to_f - val.to_f )
    else
      return IntegerVal.new( @val - val.to_i )
    end
  end
  def << val
    return IntegerVal.new( @val << val.to_i )
  end
  def >> val
    return IntegerVal.new( @val >> val.to_i )
  end
  def > val
    if val.kind_of? FloatVal then
      return BoolVal.new( @val.to_f > val.to_f )
    else
      return BoolVal.new( @val > val.to_i )
    end
  end
  def < val
    if val.kind_of? FloatVal then
      return BoolVal.new( @val.to_f < val.to_f )
    else
      return BoolVal.new( @val < val.to_i )
    end
  end
  def >= val
    if val.kind_of? FloatVal then
      return BoolVal.new( @val.to_f >= val.to_f )
    else
      return BoolVal.new( @val >= val.to_i )
    end
  end
  def <= val
    if val.kind_of? FloatVal then
      return BoolVal.new( @val.to_f <= val.to_f )
    else
      return BoolVal.new( @val <= val.to_i )
    end
  end
  def eq val # == val
    if val.kind_of? FloatVal then
      return BoolVal.new( @val.to_f == val.to_f )
    else
      return BoolVal.new( @val == val.to_i )
    end
  end
  def neq val # != val
    if val.kind_of? FloatVal then
      return BoolVal.new( @val.to_f != val.to_f )
    else
      return BoolVal.new( @val != val.to_i )
    end
  end
  def & val
    IntegerVal.new( @val & val.to_i )
  end
  def ^ val
    IntegerVal.new( @val ^ val.to_i )
  end
  def | val
    IntegerVal.new( @val | val.to_i )
  end
  def lAND val  # && val
    BoolVal.new( self.to_b && val.to_b )
  end
  def lOR val   # || val
    BoolVal.new( self.to_b || val.to_b )
  end
  def cast( type )
    t = type.get_original_type   # typedef の元を得る
    if t.kind_of? IntType then
      val = t.check_and_clip( @val, :IntType )
      return IntegerVal.new( val )
    elsif t.kind_of? FloatType then
      return FloatVal.new( @val )
    elsif t.kind_of? PtrType then
      return PointerVal.new( @val, type )
    elsif t.kind_of? BoolType then
      return BoolVal.new( @val.to_b )
    else
      cdl_error( "V1013 integer value cannot cast to $1" , type.class )
      return nil
    end
  end

  def to_s
    if @str then
      @str
    else
      @val.to_s
    end
  end
  def to_b
    @val != 0
  end
  def to_i
    @val
  end
  def to_f
    @val.to_f
  end
end

#= BoolVal: bool 値を扱うクラス
class BoolVal < BaseVal
#@val:: bool: true, false

  def initialize( val )
    super()
    if val == true || val == false
      @val = val
    elsif val.to_i != 0
      @val = true
    else
      @val = false
    end
    # raise "No boolean val" if val != true && val != false
  end

  def not # ! val
      BoolVal.new( ! @val )
  end
  def eq val # == val
    if val.kind_of? BoolVal then
      return BoolVal.new( self.to_i == val.to_i )
    else
      cdl_error( "V1014 comparing bool value with \'$1\'" , val.class )
      return BoolVal.new( false )
    end
  end
  def neq val # != val
    if val.kind_of? BoolVal then
      return BoolVal.new( self.to_i != val.to_i )
    else
      cdl_error( "V1015 comparing bool value with \'$1\'" , val.class )
      return BoolVal.new( false )
    end
  end
  def lAND val  # && val
    BoolVal.new( self.to_b && val.to_b )
  end
  def lOR val   # || val
    BoolVal.new( self.to_b || val.to_b )
  end
  def cast( type )
    t = type.get_original_type   # typedef の元を得る
    if @val then
      val = 1
    else
      val = 0
    end
    if t.kind_of? IntType then
      return IntegerVal.new( val )
    elsif t.kind_of? FloatType then
      return FloatVal.new( val )
    elsif t.kind_of? BoolType then
      return self
    else
      cdl_error( "V1016 bool value cannot cast to $1" , type.class )
      return nil
    end
  end

  def to_s
    if @val
      return "true"
    else
      return "false"
    end
  end
  def to_b
    @val
  end
  def to_i
    return 0 if @val == false
    return 1
  end
  def to_f
    return 0.0 if @val == false
    return 1.0
  end

  attr_reader :val
end

#= FloatVal: 実数値を扱うクラス
class FloatVal < BaseVal
#@val:: Float
  def initialize( val )
    super()
    @val = val.to_f
  end

  def -@
    FloatVal.new( - @val )
  end
  def +@
    self
  end
  def * val
    FloatVal.new( @val * val.to_f )
  end
  def / val
    v2 = val.to_f   # to_f を2回評価しない
    if v2 == 0.0 then
      cdl_error( "V1017 / : divieded by zero"  )
      return FloatVal.new( 1.0 )
    end
    return FloatVal.new( @val.to_f / v2 )
  end
  def % val
    v2 = val.to_f   # to_f を2回評価しない
    if v2 == 0.0 then
      cdl_error( "V1018 % : divieded by zero"  )
      return FloatVal.new( 1.0 )
    end
    return FloatVal.new( @val.to_f % v2 )
  end
  def + val
    FloatVal.new( @val + val.to_f )
  end
  def - val
    FloatVal.new( @val - val.to_f )
  end
  def > val
    BoolVal.new( @val > val.to_f )
  end
  def < val
    BoolVal.new( @val < val.to_f )
  end
  def >= val
    BoolVal.new( @val >= val.to_f )
  end
  def <= val
    BoolVal.new( @val <= val.to_f )
  end
  def eq val # == val
    BoolVal.new( @val == val.to_f )
  end
  def neq val # != val
    BoolVal.new( @val != val.to_f )
  end
  def cast( type )
    t = type.get_original_type   # typedef の元を得る
    if t.kind_of? IntType then
      val = t.check_and_clip( @val, :FloatType )
      return IntegerVal.new( val )
    elsif t.kind_of? FloatType then
      return self
    else
      cdl_error( "V1019 floating value cannot cast to $1" , type )
      return self
    end
  end

  def to_b
    cdl_error( "V1020 convert floating value to bool without cast"  )
    @val.to_i
  end
  def to_i
    cdl_error( "V1021 convert floating value to integer without cast"  )
    @val.to_i
  end
  def to_s
    @val.to_s
  end
  def to_f
    @val
  end
end

#= 文字列リテラルを扱うクラス
class StringVal < BaseVal
#@str:: Token:
#@specifier:: Symbol: :WIDE, :NORMAL

  def initialize( str, spec = :NORMAL )
    super()
    @str = str
    @specifier = spec   # mikan L"str" wide 文字列未対応
  end

  #===
  #
  # string の cast はできない mikan ポインタ型への cast はできるべき
  def cast type
    t = type.get_original_type   # typedef の元を得る
    if t.kind_of? IntType then
      cdl_error( "V1022 string cannot cast to integer"  )
    elsif t.kind_of? FloatType then
      cdl_error( "V1023 string cannot cast to float"  )
    elsif t.kind_of? PtrType then
      cdl_error( "V1024 string cannot cast to pointer"  )
    else
      cdl_error( "V1025 string cannot cast to $1" , type )
    end
  end
    
  def to_s
    @str.to_s
  end

  def val
    @str.to_s   # Token で扱われていた名残 (val を取り出す)
  end
end
