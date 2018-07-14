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
#   $Id: GenParamCopy.rb 2061 2014-05-31 22:15:33Z okuma-top $
#++

#= ParamCopy
#
# パラメータコピーするマーシャラ／アンマーシャラコードを生成するメソッド print_param を提供する．
# RPCPlugin, OpaqueRPCPlugin に include される．
# RPCPlugin (トランスペアレント) では、oneway 関数で in のポインタ引数の場合に限って print_param が用いられる．
#
module  GenParamCopy

  #=== 引数の転送コードを生成

  def print_param( name, type, file, nest, dir, outer, outer2, b_marshal, b_get, alloc_cp = nil, alloc_cp_extra = nil, name_list = nil )
    if type.get_original_type.kind_of?( ArrayType ) && b_get && dir != :OUT then
      indent = "	" * nest
      subsc = type.get_subscript
      if subsc == nil
        raise "Unsubscripted Array Not Supported"
      else        
        size_str = subsc.to_str( name_list, outer, outer2 )
        file.print <<EOT
#{indent}if((ercd_=#{alloc_cp}(sizeof(#{type.get_type.get_type_str}#{type.get_type.get_type_str_post})*(#{size_str}),(void **)&#{outer}#{name}#{outer2}#{alloc_cp_extra}))!=E_OK)\t/* GenParamCopy 1 */
#{indent}	goto error_reset;
EOT
        if ( dir == :SEND || dir == :RECEIVE ) && type.get_type.has_pointer? then
          # send, receive の場合は、エラーリセットに備え NULL にする
          file.print <<EOT
#{indent}memset( (void *)#{outer}#{name}#{outer2}#{alloc_cp_extra}, 0, sizeof(#{type.get_type.get_type_str}#{type.get_type.get_type_str_post})*(#{size_str});   /* GenParamCopy Alloc1 */
EOT
        end
      end
    end
    print_param0( name, type, file, nest, dir, outer, outer2, b_marshal, b_get, alloc_cp, alloc_cp_extra, name_list )
  end

  def print_param0( name, type, file, nest, dir, outer, outer2, b_marshal, b_get, alloc_cp = nil, alloc_cp_extra = nil, name_list = nil )
    indent = "	" * nest

    case type
    when DefinedType
      print_param0( name, type.get_type, file, nest, dir, outer, outer2, b_marshal, b_get, alloc_cp, alloc_cp_extra )
    when BoolType, IntType, FloatType
      case type
      when BoolType
        type_str = "Bool"
      when IntType
        bit_size = type.get_bit_size
        case type.get_sign
        when :UNSIGNED
          sign = "U"
        when :SIGNED
          if bit_size == -1 || bit_size == -11 then
            sign = "S"   # signed char の場合のみ S がつく
          else
            sign = ""
          end
        else
          sign = ""
        end

        case bit_size
        when -1, -11  # -1: char_t, -11: char
          type_str = "#{sign}Char"
        when -2
          type_str = "#{sign}Short"
        when -3
          type_str = "#{sign}Int"
        when -4
          type_str = "#{sign}Long"
        when -5
          type_str = "Intptr"
        when 8, 16, 32, 64, 128
          type_str = "#{sign}Int#{bit_size}"
        else
          raise "unknown bit_size '#{bit_size}' for int type "
        end

      when FloatType
        bit_size = type.get_bit_size
        if bit_size == 32 then
          type_str = "Float32"
        else
          type_str = "Double64"
        end

      end

      if( b_get )then
        file.print indent
        file.print "if( ( ercd_ = cTDR_get#{type_str}( &(#{outer}#{name}#{outer2}) ) ) != E_OK )\t/* GenParamCopy 2 */\n"
        file.print indent
        file.print "	goto error_reset;\n"
      else
        file.print indent
        file.print "if( ( ercd_ = cTDR_put#{type_str}( #{outer}#{name}#{outer2} ) ) != E_OK )\t/* GenParamCopy 3 */\n"
        file.print indent
        file.print "	goto error_reset;\n"
      end

    when PtrType

      count = type.get_count; size = type.get_size; string = type.get_string
      if count || size || string then
        nest = print_nullable_pre( name, type, file, nest, dir, outer, outer2, b_marshal, b_get )
        indent = "\t" * nest
        loop_counter_type = IntType.new(32)   # mikan 型を size_is, count_is の引数の型とする
        file.print "#{indent}{\t/* GenParamCopy 4 */\n"
        file.print "#{indent}	#{loop_counter_type.get_type_str}  i__#{nest}, length__#{nest};\n"

        if size || count then
          if size then
            size_str = size.to_str( name_list, outer, outer2 )
          end

          if count then
            count_str = count.to_str( name_list, outer, outer2 )
          else
            # size_is は必須. count_is はオプション
            count_str = size_str
          end
          file.print "#{indent}	length__#{nest} = #{count_str};\t/* GenParamCopy 5 */\n"

          # size_is に max 指定がある場合、length が max を超えているかチェックするコードを生成
              # alloc_cp == nil のとき dir は INOUT, OUT のはず (条件が冗長)。試験が終わっているので、次回見直し時に外す
          if b_get && type.get_max != nil && ! ( ( dir == :INOUT || dir == :OUT ) && alloc_cp == nil ) then
            file.print "#{indent}	if( length__#{nest} > #{type.get_max.to_s} ){\t/* GenParamCopy max check 1 */\n"
            file.print "#{indent}		ercd_ = E_PAR;\n"
            file.print "#{indent}		goto error_reset;\n"
            file.print "#{indent}	}\n"
          end

        else # if string then
          case type.get_type.get_bit_size
          when -1              # char_t
            b_size = 8
          when 8, 16, 32, 64   # intN_t, uintN_t
            b_size = type.get_type.get_bit_size
          else
            cdl_error( "R9999 $1: string specifier cannot be specified to '$2' in current implementation",
                       name, type.get_type.get_type_str + type.get_type.get_type_str_post )
          end
          if ! b_get then
            if  string.instance_of? Expression then
              len = string.to_str( name_list, outer, outer2 )
              file.print "#{indent}	length__#{nest} = STRNLEN#{b_size}(#{outer}#{name}#{outer2},(#{len}-1))+1;\t/* GenParamCopy 6 */\n"
              file.print "#{indent}	if( length__#{nest} < #{len})\tlength__#{nest} += 1;\n"
            else
              file.print "#{indent}	length__#{nest} = STRLEN#{b_size}(#{outer}#{name}#{outer2})+1;\t/* GenParamCopy 7 */\n"
            end
            size_str = "length__#{nest}"     # string の場合、strnlen 以上の領域を確保しない
          else
            if ( dir == :INOUT ) then
              if ( string.instance_of? Expression ) then
                len = string.to_str( name_list, outer, outer2 )
                size_str = "#{len}"              # string(len) の場合 len を確保する
              else
                raise "unsuscripted string used for inout parameter #{name}"
              end
            else
              size_str = "length__#{nest}"     # string の場合、strnlen 以上の領域を確保しない
            end
          end
          print_param0( "length__#{nest}", loop_counter_type, file, nest + 1, dir, nil, nil, b_marshal, b_get )
        end

        if b_get && ( dir == :IN || dir == :INOUT || dir == :SEND || dir == :RECEIVE ) && alloc_cp then
          file.print <<EOT
#{indent}	if((ercd_=#{alloc_cp}(sizeof(#{type.get_type.get_type_str}#{type.get_type.get_type_str_post})*(#{size_str}),(void **)&#{outer}#{name}#{outer2}#{alloc_cp_extra}))!=E_OK)\t/* GenParamCopy 8 */
#{indent}		goto error_reset;
EOT
          if ( dir == :SEND || dir == :RECEIVE ) && type.get_type.has_pointer? then
            # send, receive の場合は、エラーリセットに備え NULL にする
            file.print <<EOT
#{indent}	memset( (void *)#{outer}#{name}#{outer2}#{alloc_cp_extra}, 0, sizeof(#{type.get_type.get_type_str}#{type.get_type.get_type_str_post})*(#{size_str}) );   /* GenParamCopy Alloc2 */
EOT
          end
        end
        file.print "#{indent}	for( i__#{nest} = 0; i__#{nest} < length__#{nest}; i__#{nest}++ ){\t/* GenParamCopy 9 */\n"

        print_param0( name, type.get_type, file, nest + 2, dir, outer, "#{outer2}[i__#{nest}]", b_marshal, b_get, alloc_cp, alloc_cp_extra )
        file.print "#{indent}	}\n"
        file.print "#{indent}}\n"

        nest = print_nullable_post( name, type, file, nest, dir, outer, outer2, b_marshal, b_get )
        indent = "\t" * nest
      else

        # nullable (pre)
        nest = print_nullable_pre( name, type, file, nest, dir, outer, outer2, b_marshal, b_get )
        indent = "	" * nest

        # allocate memory for getting value
        if b_get && ( dir == :IN || dir == :INOUT || dir == :SEND || dir == :RECEIVE ) && alloc_cp then
          file.print <<EOT
#{indent}if((ercd_=#{alloc_cp}(sizeof(#{type.get_type.get_type_str}#{type.get_type.get_type_str_post}),(void **)&#{outer}#{name}#{outer2}#{alloc_cp_extra}))!=E_OK)\t/* GenParamCopy 10 */
#{indent}	 goto error_reset;
EOT
          if ( dir == :SEND || dir == :RECEIVE ) && type.get_type.has_pointer? then
            # send, receive の場合は、エラーリセットに備え NULL にする
            file.print <<EOT
#{indent}memset( (void *)#{outer}#{name}#{outer2}#{alloc_cp_extra}, 0, sizeof(#{type.get_type.get_type_str}#{type.get_type.get_type_str_post}) );   /* GenParamCopy Alloc3 */
EOT
          end
        end

        print_param0( name, type.get_type, file, nest, dir, "(*#{outer}", "#{outer2})", b_marshal, b_get, alloc_cp, alloc_cp_extra )
        # nullable (post)
        nest = print_nullable_post( name, type, file, nest, dir, outer, outer2, b_marshal, b_get )
        indent = "	" * nest
      end
    when StructType
      members_decl =type.get_members_decl
      members_decl.get_items.each { |m|
        if m.is_referenced?
          print_param0( m.get_name, m.get_type, file, nest, dir, "#{outer}#{name}#{outer2}.", nil, b_marshal, b_get, alloc_cp, alloc_cp_extra, members_decl )
        end
      }
      members_decl.get_items.each { |m|
        if ! m.is_referenced?
          print_param0( m.get_name, m.get_type, file, nest, dir, "#{outer}#{name}#{outer2}.", nil, b_marshal, b_get, alloc_cp, alloc_cp_extra, members_decl )
        end
      }

    when VoidType
    when EnumType  # mikan EnumType
    when FuncType  # mikan FuncType
    when ArrayType # mikan ArrayType
      subsc = type.get_subscript
      if subsc == nil
        raise "Unsubscripted Array Not Supported"
      else        
        size_str = subsc.to_str( name_list, outer, outer2 )

        loop_counter_type = IntType.new(32)   # mikan 型を size_is, count_is の引数の型とする
        file.print "#{indent}{\t/* GenParamCopy 11 */\n"
        file.print "#{indent}	#{loop_counter_type.get_type_str}  i__#{nest}, length__#{nest} = #{size_str};\n"

        file.print "#{indent}	for( i__#{nest} = 0; i__#{nest} < length__#{nest}; i__#{nest}++ ){\n"
        print_param0( name, type.get_type, file, nest + 2, dir, outer, "#{outer2}[i__#{nest}]", b_marshal, b_get, alloc_cp, alloc_cp_extra )
        file.print "#{indent}	}\n"
        file.print "#{indent}}\n"
      end
    end
  end

  #=== nullable (pre)
  def print_nullable_pre( name, type, file, nest, dir, outer, outer2, b_marshal, b_get )
    if type.is_nullable? then
      indent = "	" * nest
      if dir == :OUT then  # OUT の場合 print_out_nullable で NULL かどうかの情報を渡す
        # 'null or not' is sent in the function 'print_out_nullable'
        if b_get then
          file.print "#{indent}if( #{outer}#{name}#{outer2} ){\t/* GenParamCopy Null 10 */\n"
        else
          file.print "#{indent}if( ! b_#{name}_null_ ){\t/* GenParamCopy Null 11 */\n"
        end
        nest += 1
      else # dir = :IN, :INOUT, :SEND, :RECEIVE
        if b_get then
          file.print "#{indent}{\n"
          if ! ( dir == :INOUT && b_marshal == true ) then
            file.print <<EOT
#{indent}	int8_t  b_null_;
#{indent}	if((ercd_=cTDR_getInt8( &b_null_ )) != E_OK )\t/* GenParamCopy Null 20 */
#{indent}		 goto error_reset;
#{indent}	if( ! b_null_ ){
EOT
          else # dir = :INOUT, b_marshal = true, b_get = true の場合、NULL かどうかの情報を渡さない
            file.print <<EOT
#{indent}	int8_t  b_null_ = (#{outer}#{name}#{outer2} == NULL);\t/* GenParamCopy Null 21 */
#{indent}	if( ! b_null_ ){
EOT
          end
        else
          file.print <<EOT
#{indent}{
#{indent}	int8_t  b_null_ = (int8_t)(#{outer}#{name}#{outer2} == NULL);\t/* GenParamCopy Null 31 */
EOT

          if ! ( dir == :INOUT && b_marshal == false ) then
            # dir = :INOUT, b_marshal = false, b_get = false の場合
            file.print <<EOT
#{indent}	if((ercd_=cTDR_putInt8( b_null_ )) != E_OK )\t/* GenParamCopy Null 32 */
#{indent}		 goto error_reset;
EOT
          end

          file.print <<EOT
#{indent}	if( ! b_null_ ){\t/* GenParamCopy Null 33 */
EOT
        end
        nest += 2
      end
    end
    return nest
  end

  #== nullable (post)
  def print_nullable_post( name, type, file, nest, dir, outer, outer2, b_marshal, b_get )
    if type.is_nullable? then
      if dir == :OUT then  # OUT の場合 print_out_nullable で NULL かどうかの情報を渡す
        nest -= 1
        indent = "	" * nest
        file.print "#{indent}}  /* ! b_#{name}_null_   GenParamCopy Null 50 */\n"
      else # ( dir == :IN || dir == :INOUT || dir == :SEND || dir == :RECEIVE )
        nest -= 2
        indent = "	" * nest
        if b_get then
          if ! ( dir == :INOUT && b_marshal == true ) then
            file.print <<EOT

#{indent}	} else { /* null  GenParamCopy Null 51 */
#{indent}		#{outer}#{name}#{outer2} = NULL;
#{indent}	}  /* ! b_null_ */
EOT
          else # dir = :INOUT, b_marshal = true   # inout の out 方向
            file.print "#{indent}	}  /* ! b_null_  GenParamCopy Null 52 */\n"
          end
        else
          file.print "#{indent}	}\t/* GenParamCopy Null 53 */\n"
        end
        file.print "#{indent}}\t/* GenParamCopy Null 54 */\n"
      end
    end
    return nest
  end
end
