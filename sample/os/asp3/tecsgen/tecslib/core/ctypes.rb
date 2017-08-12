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
#   $Id: ctypes.rb 2622 2017-01-04 13:19:33Z okuma-top $
#++

# CType は C_parser で定義される型を扱う CIntType, CFloatType などに include するもの
# CIntType は IntType を継承するなど、C の型では TECS の型を継承する
module CType

  #=== 構文要素 type_specifier が複数指定されている場合に merge する
  # merge は const(CIntType) unsigned(CIntTtype), long(CIntType), などと他の型をマージする
  # const, unsigned, long などは、単体で int (CIntType) 型になりうる
  #
  # mikan C の文法を厳密にはチェックしていない  long struct 等もできてしまう
  def merge another

    # p "self: #{self.class} kind_of( IntType ): #{self.kind_of?( IntType )}  another: #{another.class}"

    # signed, unsigned が Symbol として来る事は無くなった
    # if another.instance_of? Symbol then
    #   # ここで Symbol は :SIGNED, :UNSIGNED のいずれか
    # 
    #   # CIntType か？
    #   if self.instance_of? CIntType then
    #     self.set_sign another
    #     return self
    #   else
    #     cdl_error( "C1001 $1: mismatch, suitable for int types" , another )
    #     return self
    #   end
    # elsif self.instance_of?( CIntType ) && another.instance_of?( CIntType )then
    if self.instance_of?( CIntType ) && another.instance_of?( CIntType )then
      if another.get_bit_size != -3 then
        if @bit_size == -4 && another.get_bit_size == -4 then
          @bit_size = -5  # long long
        else
          # self は int 型、another の bit_size が (int 以外であれば)そちらにする
          # mikan 上記以外で 両方 -3 でなければ、本来エラー
          @bit_size = another.get_bit_size
        end
      end

      if another.get_sign then
        # another で sign が指定されていれば、そちらのものを採用する mikan 矛盾のチェック
        @sign = another.get_sign
      end

#      if another.get_qualifier then
#        # another で qualifier が指定されていれば、そちらのものを採用する mikan 矛盾のチェック
#        @qualifier = another.get_qualifier
#      end
      if another.is_const? then
        @b_const = true
      end
      if another.is_volatile? then
        @b_volatile = true
      end

      return self
    elsif self.instance_of?( CIntType ) then
      return another.merge self
    elsif self.instance_of?( CDefinedType ) then
      # mikan unsigned などとの merge の不正検出
      if another.is_const? then
        @b_const = true
      end
      if another.is_volatile? then
        @b_volatile = true
      end

#      if self.get_type.get_type_str == another.get_type_str &&
#          self.get_type.get_type_str_post == another.get_type_str_post
#        # p "typedef #{another.get_type_str} #{self.get_type_str}#{another.get_type_str_post} ;"
#      else
#        cdl_error( "C1002 $1 not compatible with previous one $2" , self.get_type_str, another.get_type_str )
#      end
      return self
    elsif self.instance_of?( CStructType ) then
      if another.is_const? then
        @b_const = true
      end
      if another.is_volatile? then
        @b_volatile = true
      end
      return self
    elsif self.instance_of?( CFloatType ) then
      # mikan long double
      #   TECS には long double を表現する手段がない (double80_t を定義すればよいか?)
#      cdl_warning( "C1003 $1 & $2 incompatible (\'long double\' is not supported.). Treated as $3." , self.class, another.class, self.class )
      cdl_warning( "W9999 $1 & $2 incompatible (\'long double\' is not supported.). Treated as $3." , self.get_type_str, another.get_type_str, self.get_type_str )
      return self
    else
      raise "merge: unknown type"
    end
  end

  #=== qualifier を設定する
  #     元の Type クラスでは矛盾チェックしない（TECSの本来の文法では重複指定できないため）
  def set_qualifier( qual )

    if @qualifier then
      cdl_error( "C1004 $1: qualifier redefined. previous one: $2" , qual, @qualifier )
    end
    super( qual )
  end

end

class CDefinedType < DefinedType

  include CType

  def initialize( type_name )
    super( type_name )
  end
end

class CVoidType < VoidType

  include CType

end

class CBoolType < BoolType

  include CType

end

class CIntType < IntType

  include CType

  def initialize( bit_size )
    #p super.class   mikan super.class が Symbol だ、なぜ？
    super( bit_size )
  end

  def set_sign( sign, b_uint = false )
    super( sign, b_uint )
    # p "CInt: set_sign: #{get_type_str} #{sign}"
  end
end

class CFloatType < FloatType

  include CType

  def initialize( bit_size )
    super
  end

end

class CEnumType < EnumType # mikan

  include CType

  def initialize( bit_size )
    super( bit_size )
  end

end

class CStructType < StructType

  include CType


  def initialize( tag = nil )
    super( tag )
  end
end

class CFuncType < FuncType

  include CType

  def initialize( paramlist = nil )
    super( paramlist )
  end

end

class CArrayType < ArrayType

  include CType

  def initialize( subscript = nil )
    super( subscript )
  end
end

class CPtrType < PtrType

  include CType

  def initialize( referto = nil )
    super( referto )
  end

end
