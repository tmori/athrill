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
#   $Id: expression.rb 2633 2017-04-02 06:02:05Z okuma-top $
#++

class Expression < Node
#  @elements   # array

  def initialize( elements, locale = nil )
    super()
    if locale then
      @locale = locale
    end

    @elements = elements
  end

  def print
    # puts "expr: #{@elements}"
    puts "expr_string: #{to_s}"
  end

  #=== Expression# to_s
  # C 言語ソース向きの文字列を生成 (globa_name)
  def to_s
    elements_to_s( @elements )
  end

  #=== Expression# to_str
  # C 言語ソース向きの文字列を生成 (globa_name)
  def to_str( name_list, pre, post )
    elements_to_s( @elements, name_list, pre, post )
  end

  #=== Expression#to_CDL_str
  # CDL 表現の文字列を生成
  def to_CDL_str
    return to_s
  end

  #=== 定数式として評価する(トライしてみる)
  #
  # このメソッドは、定数式を評価する
  # ・attribute, var の初期化子
  # ・size_is, count_is 引数
  # ・配列の添数
  #
  # name_list(NamedList|Nil): 式から参照可能なリスト．
  # NamedList の要素は  size_is, count_is の引数評価の場合 ParamDecl (関数仮引数)
  #
  # name_list2(NamedList|Nil) : NamedList の要素は Decl (attribute, var) である．省略時 nil
  # 
  # RETURN: 評価した定数．評価できなかった場合は nil を返す
  #
  # 型は get_type で、評価する（定数として求められないときに使用できる）
  # Array を返すのは attr{ int *a = {1, 2, 3}; int *b = a; }; の b の右辺を評価した場合

  def eval_const( name_list, name_list2 = nil )
    val = elements_eval_const( @elements, name_list, name_list2, 0 )
    if val.kind_of? IntegerVal then
      return val.to_i
    elsif val.kind_of? FloatVal then
      return val.to_f
    elsif val.kind_of? BoolVal then
      return val.to_i
    elsif val.kind_of? PointerVal then
      return val.to_i           # mikan エラー V1008 が発生してしまう
      # elsif val.kind_of? EnumVal then
      # enum mikan
    else
      # C_EXP, Array または nil ：そのまま返す
      return val
    end
  end

  #=== 定数式として評価する2(トライしてみる)
  #
  # IntegerVal, FloatVal をそのまま返す（eval_const では Integer, Float に変換）
  def eval_const2( name_list, name_list2 = nil, nest = 0 )
    val = elements_eval_const( @elements, name_list, name_list2, nest )
  end

  #=== 式の型を評価する
  #
  # eval_const で値が得られない場合、型を導出可能であれば型を得る
  # param を含んだ式は定数値を求められないが、型を得ることはできる
  # 未定義変数を含んだ型は、得ることができない (ダミー型定義が返る)
  def get_type( namedList )        # 名前空間の NamedList を指定
    elements_get_type( @elements, namedList )
  end

  def check_dir_for_param( namedList, dir, spec )
    elements_check_dir_for_param( @elements, namedList, dir, spec )
  end

  def get_elements
    @elements
  end

  def show_tree( indent )
    # mikan override してしまった print を呼出す方法がわからないのでこうした
    str = ""
    indent.times { str += "  " }
    puts "#{str}#{to_s}"
  end

## private

  #=== 式を文字列に変換
  #name_list:: attribute (Celltype::@attribute_list), struct の @member_list を仮定している
  def elements_to_s( elements, name_list = nil, pre = nil, post = nil )
    if elements.instance_of? Token then
      return elements.to_s    # OP_DOT, OP_REF の右辺
    end

    case elements[0]
    when :IDENTIFIER
      nsp = elements[1]
      # if nsp.is_name_only? && name_list && name_list.get_item( nsp.get_name ) then
      if nsp.is_name_only? && name_list && name_list.get_item( nsp.get_name ) then
        return "#{pre}#{nsp.get_name}#{post}"
      else
        # return  elements[1].get_global_name
        return nsp.get_path_str
      end
    when :INTEGER_CONSTANT, :FLOATING_CONSTANT, :OCTAL_CONSTANT, :HEX_CONSTANT, :CHARACTER_LITERAL, :STRING_LITERAL_LIST, :BOOL_CONSTANT
      return  elements[1].to_s
    when :PARENTHESES
      return "(#{elements_to_s(elements[1],name_list,pre,post)})"
    when :OP_SUBSC
      return "#{elements_to_s(elements[1],name_list,pre,post)}[#{elements[2].to_s}]"
    when :OP_DOT
      return "#{elements_to_s(elements[1],name_list,pre,post)}.#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_REF
      return "#{elements_to_s(elements[1],name_list,pre,post)}->#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_SIZEOF_EXPR
      return "sizeof(#{elements_to_s(elements[1],name_list,pre,post)})"
    when :OP_SIZEOF_TYPE
      return "sizeof(#{elements[1]}) mikan"
    when :OP_U_AMP
      return "&#{elements_to_s(elements[1],name_list,pre,post)}"
    when :OP_U_ASTER
      return "*#{elements_to_s(elements[1],name_list,pre,post)}"
    when :OP_U_PLUS
      return "+#{elements_to_s(elements[1],name_list,pre,post)}"
    when :OP_U_MINUS
      return "-#{elements_to_s(elements[1],name_list,pre,post)}"
    when :OP_U_TILDE
      return "~#{elements_to_s(elements[1],name_list,pre,post)}"
    when :OP_U_EXCLAM
      return "!#{elements_to_s(elements[1],name_list,pre,post)}"
    when :CAST
      return "(#{elements[1].get_type_str})#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_MULT
      return "#{elements_to_s(elements[1],name_list,pre,post)}*#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_DIV
      return "#{elements_to_s(elements[1],name_list,pre,post)}/#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_REMAIN
      return "#{elements_to_s(elements[1],name_list,pre,post)}%#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_ADD
      return "#{elements_to_s(elements[1],name_list,pre,post)}+#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_SUB
      return "#{elements_to_s(elements[1],name_list,pre,post)}-#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_LSFT
      return "#{elements_to_s(elements[1],name_list,pre,post)}<<#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_RSFT
      return "#{elements_to_s(elements[1],name_list,pre,post)}>>#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_LT
      return "#{elements_to_s(elements[1],name_list,pre,post)}<#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_GT
      return "#{elements_to_s(elements[1],name_list,pre,post)}>#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_LE
      return "#{elements_to_s(elements[1],name_list,pre,post)}<=#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_GE
      return "#{elements_to_s(elements[1],name_list,pre,post)}>=#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_EQ
      return "#{elements_to_s(elements[1],name_list,pre,post)}==#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_NE
      return "#{elements_to_s(elements[1],name_list,pre,post)}!=#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_AND
      return "#{elements_to_s(elements[1],name_list,pre,post)}&#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_EOR
      return "#{elements_to_s(elements[1],name_list,pre,post)}^#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_OR
      return "#{elements_to_s(elements[1],name_list,pre,post)}|#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_LAND
      return "#{elements_to_s(elements[1],name_list,pre,post)}&&#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_LOR
      return "#{elements_to_s(elements[1],name_list,pre,post)}||#{elements_to_s(elements[2],name_list,pre,post)}"
    when :OP_CEX
      return "#{elements_to_s(elements[1],name_list,pre,post)}?#{elements_to_s(elements[2],name_list,pre,post)}:#{elements_to_s(elements[3],name_list,pre,post)}"
    else
      raise "Unknown expression element: #{elemets[0]}. try -t and please report"
    end
    return ""
  end

  #=== Expression# 逆ポーランド文字列化
  #param_list:: ParamlList  関数の引数リスト
  def get_rpn( param_list = nil, name_list2 = nil )
    return elements_rpn( @elements, param_list, name_list2 )
  end

  #=== Expression# 逆ポーランド文字列化 (private)
  #name_list:: ParamlList  関数の引数リスト
  def elements_rpn( elements, name_list = nil, name_list2 = nil )
    if elements.instance_of? Token then
      print "rpn: #{elements.to_s}\n"
      return elements.to_s    # OP_DOT, OP_REF の右辺
    end

    case elements[0]
    when :IDENTIFIER
      nsp = elements[1]
      #if nsp.is_name_only? && name_list && name_list.find( nsp.get_name ) then
      if nsp.is_name_only? then
        count = 0
        # p "search: #{nsp.get_name}"
        name_list.get_items.each{ |nm,val|
          # p "    : #{nm.get_name} #{nsp.get_name.class} #{nm.get_name.class}"
          if nsp.get_name == nm.get_name then
            return " $#{count}"
          end
          count += 1
        }
        raise "not found parameter"
      else
        # return  elements[1].get_global_name
        raise "not unexpected parameter"
      end
    when :INTEGER_CONSTANT, :FLOATING_CONSTANT, :OCTAL_CONSTANT, :HEX_CONSTANT, :CHARACTER_LITERAL, :STRING_LITERAL_LIST, :BOOL_CONSTANT
      return elements[1].to_s
    when :PARENTHESES
      return elements_rpn( elements[1], name_list, name_list2 )
    when :OP_SUBSC
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[1], "", name_list, name_list2 ) + " []"
    when :OP_DOT
      return elements_rpn( elements[1], name_list, name_list2 ) + " ."
    when :OP_REF
      return elements_rpn( elements[1], name_list, name_list2 ) + " ->"
    when :OP_SIZEOF_EXPR
      return elements_rpn( elements[1], name_list, name_list2 ) + " #s"
    when :OP_SIZEOF_TYPE
      return elements_rpn( elements[1], name_list, name_list2 ) + " #S"
    when :OP_U_AMP
      return elements_rpn( elements[1], name_list, name_list2 ) + " #&"
    when :OP_U_ASTER
      return elements_rpn( elements[1], name_list, name_list2 ) + " #*"
    when :OP_U_PLUS
      return elements_rpn( elements[1], name_list, name_list2 ) + " #+"
    when :OP_U_MINUS
      return elements_rpn( elements[1], name_list, name_list2 ) + " #-"
    when :OP_U_TILDE
      return elements_rpn( elements[1], name_list, name_list2 ) + " #~"
    when :OP_U_EXCLAM
      return elements_rpn( elements[1], name_list, name_list2 ) + " #!"
    when :CAST
      return elements_rpn( elements[1], name_list, name_list2 ) + " #(" + elements_rpn( elements[2], "", name_list, name_list2 ) + ")"
    when :OP_MULT
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " *"
    when :OP_DIV
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " /"
    when :OP_REMAIN
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " %"
    when :OP_ADD
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " +"
    when :OP_SUB
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " -"
    when :OP_LSFT
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " <<"
    when :OP_RSFT
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " >>"
    when :OP_LT
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " <"
    when :OP_GT
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " >"
    when :OP_LE
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " <="
    when :OP_GE
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " >="
    when :OP_EQ
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " =="
    when :OP_NE
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " !="
    when :OP_AND
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " &"
    when :OP_EOR
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " ^"
    when :OP_OR
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " |"
    when :OP_LAND
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " &&"
    when :OP_LOR
      return elements_rpn( elements[1], name_list, name_list2 ) + " " + elements_rpn( elements[2], "", name_list, name_list2 ) + " ||"
    when :OP_CEX
      return elements_rpn( elements[1], name_list, name_list2 ) + " " +
             elements_rpn( elements[2], name_list, name_list2 ) + " " +
             elements_rpn( elements[3], name_list, name_list2 ) + " ?:"
    else
      raise "Unknown expression element: #{elemets[0]}. try -t and please report"
    end
    return ""
  end

  # 定数式(elements)を評価する
  #
  # このメソッドは Expression クラスのメソッドである必要はない（関数化できる）
  #
  # elements は式の要素
  #
  # name_list, name_list2 は eval_const を参照
  #
  # RETURN: 評価した定数、評価できなかった場合は nil を返す

  MAX_NEST_LEVEL = 64    # 簡易のループ検出（参照のネストを 64 まで許可する）
  def elements_eval_const( elements, name_list, name_list2 = nil, nest = nil )

    case elements[0]
    when :IDENTIFIER
      nsp = elements[1]

      # #809 の修正しかけ (別の問題が解決しきれていない)
      # nest += 1     # 参照がループになっていないかのチェック
      #               # mikan 本当にループしているかどうかではなく、単純に多数の参照を繰り返していることで判定している
      # if nest > MAX_NEST_LEVEL then
      #   cdl_error( "E9999: '$1' too many reference (maybe loop) max=$1" , nsp.to_s, MAX_NEST_LEVEL )
      #   return
      # end
      if nsp.is_name_only? then
        if name_list then
          object = name_list.get_item( nsp.get_name )
        end

        if object == nil && name_list2 then
          object = name_list2.get_item( nsp.get_name )
        end
      end

      # 見つからなければ定数定義から探す
      if object == nil then
        object = Namespace.find( nsp )# mikan namespace の対応 #1
      end

# この実装は、もう少し整理されるべき
# これが呼出されるのは、以下の場合
#   ・attribute, var の右辺式の評価
#   ・size_is 引数の評価：関数パラメータの場合とattribute, var の場合がある
# 以下のエラーチェックでは、これらがごっちゃになって誤りを検出しようとしている

      # IDENTIFIER は見つからなかった？
      if object == nil then
        cdl_error( "E1001 $1: not found" , nsp.get_path_str )
        # raise  "E1001"  # bug trap
        return nil
      elsif object.instance_of?( Join ) then
        # Join の場合： cell の中の attribute, var, call のどれかが見つかった
        # Decl (attribute, var) でない？
        if ! object.get_definition.instance_of?( Decl ) then
          cdl_error( "E1002 $1: not constant (port)" , nsp.get_path_str )
          return nil
        end
        return object.get_rhs.eval_const2( name_list, name_list2, nest )
      elsif ! object.instance_of?( Decl ) then
        # Decl でない場合： 定数でもない
        if ( ! object.instance_of?( ParamDecl ) ) then
                                                      # mikan paramdecl は無視する
                                                      # ParamList から呼ばれたとき
          cdl_error( "E1003 $1: not constant" , nsp.get_path_str )
        else
          # ParamDecl
          object.referenced
        end
        return nil
      else # Decl
        object.referenced
        if object.get_initializer == nil then
          # 初期化子の存在しない変数   # mikan ここへくるのは、通常ありえないはず（未検証）
          return IntegerVal.new( 0 )
        else
          # Decl の右辺の評価
          # mikan size_is 引数に現れる変数の型が適切かのチェックする
          if object.get_initializer.instance_of?( Expression ) || object.get_initializer.instance_of?( C_EXP ) then
            return object.get_initializer.eval_const2( name_list, name_list2, nest )
          else
            # Array の場合
            return object.get_initializer
          end
        end
      end
    when :BOOL_CONSTANT
      if( elements[1].instance_of?( TrueClass ) )then
        return BoolVal.new( true )
      elsif( elements[1].instance_of?( FalseClass ) )then
        return  BoolVal.new( false )
      else
        throw( "BOOL constant error" )
      end
    when :INTEGER_CONSTANT
      return IntegerVal.new( elements[1].val )
    when :FLOATING_CONSTANT
      return FloatVal.new( elements[1].val )
    when :OCTAL_CONSTANT
      return IntegerVal.new( elements[1].val.oct, elements[1].val )
    when :HEX_CONSTANT
      return IntegerVal.new( elements[1].val.hex, elements[1].val )
    when :CHARACTER_LITERAL
      str =  elements[1].val.gsub(/'/, "" )
#2.0      if str.jlength == 1
      if $b_no_kcode then
        len = str.length
      else
        len = str.jlength
      end
      if len == 1 then
        sum = 0
        str.each_byte { |b| sum = sum * 256 + b }
        return IntegerVal.new( sum, elements[1].val )
      else
#2.0        if str[0] == 92 then
        if str[0] == 92 || str[0] == "\\" then
          case str[1]
#2.0          when 48 # '0'
          when 48, "0" # '0'
            return IntegerVal.new( 0, elements[1].val )
#2.0          when 110 # 'n'
          when 110, "n" # 'n'
            return IntegerVal.new( 10, elements[1].val )
#2.0          when 114 # 'r'
          when 114, "r" # 'r'
            return IntegerVal.new( 13, elements[1].val )
#2.0          when 116 # 't'
          when 116, "t" # 't'
            return IntegerVal.new( 15, elements[1].val )
#2.0          when 92 # '\\'
          when 92, '\\' # '\\'
            return IntegerVal.new( 92, elements[1].val )
          end
        end
      end
#2.0      printf( "c=%c\n", str[1] )
      printf( "len=%d c=%c\n", len, str[1] )
      raise Error

    when :STRING_LITERAL_LIST
      return  StringVal.new( elements[1] )
    when :PARENTHESES
      return elements_eval_const( elements[1], name_list, name_list2, nest );
    when :OP_SUBSC
      cdl_error( "E1004 cannot evaluate \'[]\' operator"  )
      return nil
    when :OP_DOT
      cdl_error( "E1005 cannot evaluate \'.\' operator"  )
      return nil
    when :OP_REF
      cdl_error( "E1006 cannot evaluate \'->\' operator"  )
      return nil
    when :OP_SIZEOF_EXPR
      if Generator.parsing_C? then
        cdl_info( "I9999 cannot evaluate \'sizeof\' operator. this might causes later error."  )
      else
        cdl_error( "E1007 cannot evaluate \'sizeof\' operator"  )
      end
      return nil
    when :OP_SIZEOF_TYPE
      if Generator.parsing_C? then
        cdl_info( "I9999 cannot evaluate \'sizeof\' operator. this might causes later error."  )
      else
        cdl_error( "E1008 cannot evaluate \'sizeof\' operator"  )
      end
      return nil
    when :OP_U_AMP
      cdl_error( "E1009 cannot evaluate \'&\' operator"  )
      return nil
    when :OP_U_ASTER
      # cdl_error( "E1010 cannot evaluate \'*\' operator"  )
      val = elements_eval_const(elements[1], name_list, name_list2, nest)
      return nil if ! evaluable?( val )
      return val
    when :OP_U_PLUS
      val = elements_eval_const(elements[1], name_list, name_list2, nest)
      return nil if ! evaluable?( val )
      if val.respond_to?( "+@" ) then
        return + val
      else
        cdl_error( "E1011 cannot evaluate unary + for $1" , val.class )
        return nil
      end
    when :OP_U_MINUS
      val = elements_eval_const(elements[1], name_list, name_list2, nest)
      return nil if ! evaluable?( val )
      if val.respond_to?( "-@" ) then
        return - val
      else
        return nil
      end
    when :OP_U_TILDE
      val = elements_eval_const(elements[1], name_list, name_list2, nest)
      return nil if ! evaluable?( val )
# p "val.respond_to?( \"-@\" )=#{val.respond_to?( "-@" )} #{val.class}"
# p "val.respond_to?( \"~@\" )=#{val.respond_to?( "~@" )}"
#2.0      if val.respond_to?( "~@" ) then  # Ruby 1.9, 2.0 preview 版では例外が発生してしまう
      if val.kind_of? IntegerVal then
        return ~ val
      else
        return nil
      end
    when :OP_U_EXCLAM
      val = elements_eval_const(elements[1], name_list, name_list2, nest)
      return nil if ! evaluable?( val )
      val = val.cast( BoolType.new )
      if val.respond_to?( "not" ) then
        return val.not
      else
        return nil
      end
      return nil
    when :CAST
      val = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if val == nil
      return nil if ! evaluable?( val )
      return val.cast( elements[1] )
    when :OP_MULT
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs * rhs
    when :OP_DIV
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs / rhs
    when :OP_REMAIN
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs % rhs
    when :OP_ADD
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs + rhs
    when :OP_SUB
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs - rhs
    when :OP_LSFT
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs << rhs
    when :OP_RSFT
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs >> rhs
    when :OP_LT
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs < rhs
    when :OP_GT
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs > rhs
    when :OP_LE
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs <= rhs
    when :OP_GE
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs >= rhs
    when :OP_EQ
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs.eq( rhs )
    when :OP_NE
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs.neq( rhs )
    when :OP_AND
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs & rhs
    when :OP_EOR
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs ^ rhs
    when :OP_OR
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs | rhs
    when :OP_LAND
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs.lAND( rhs )
    when :OP_LOR
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      # return nil if( rhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, lhs )
      return lhs.lOR( rhs )
    when :OP_CEX
      lhs = elements_eval_const(elements[1], name_list, name_list2, nest)
      mhs = elements_eval_const(elements[2], name_list, name_list2, nest)
      rhs = elements_eval_const(elements[3], name_list, name_list2, nest)
      # return nil if( rhs == nil || mhs == nil || lhs == nil )
      return nil if ! evaluable?( rhs, mhs, lhs )
      if lhs.cast( BoolType.new ).val then
          return mhs
      else
          return rhs
      end
    end
    return nil
  end

  def elements_get_type( elements, namedList )
    type = elements_get_type_sub( elements, namedList )
    # 返された方が DefinedType の場合 元の型を返す
    if type.kind_of?( DefinedType ) then
      type = type.get_type
    end
    return type
  end

  def elements_get_type_sub( elements, namedList )
    case elements[0]
    when :IDENTIFIER
      nsp = elements[1]
      if nsp.is_name_only? then
        paramdecl = namedList.get_item( nsp.get_name )
      else
        paramdecl = nil
      end
      unless paramdecl then
        cdl_error( "E1012 $1: not found in parameter list" , nsp.get_path_str )
        return IntType.new(32)        # dummy result
      end
      return paramdecl.get_type
# mikan get_type
#    when :INTEGER_CONSTANT
#    when :FLOATING_CONSTANT
#    when :OCTAL_CONSTANT
#    when :HEX_CONSTANT
#    when :CHARACTER_LITERAL
#    when :STRING_LITERAL_LIST
#    when :PARENTHESES
#    when :OP_SUBSC
#    when :OP_DOT
#    when :OP_REF
#    when :OP_SIZEOF_EXPR
#    when :OP_SIZEOF_TYPE
#    when :OP_U_AMP
    when :OP_U_ASTER
      type = elements_get_type( elements[1], namedList )
      unless type.kind_of?( PtrType ) then
        cdl_error( "E1013 \'*\': operand is not pointer value"  )
        return IntType.new( 8 )    # IntType を返しておく
      end
      return type.get_referto

    when :OP_U_PLUS, :OP_U_MINUS
      # mikan operand が適切な型かチェックしていない
      return elements_get_type( elements[1], namedList )

    when :OP_ADD, :OP_SUB, :OP_MULT, :OP_DIV, :OP_REMAIN
      # mikan operand が適切な型かチェックしていない＆左辺の型を採用している
      return elements_get_type( elements[1], namedList )

    when :OP_U_TILDE
      # mikan operand が整数かチェックしていない
      return elements_get_type( elements[1], namedList )
    when :OP_AND, :OP_EOR, :OP_OR, :OP_LSFT, :OP_RSFT
      # mikan operand が整数かチェックしていない
      return BoolType.new
    when :OP_U_EXCLAM
      # mikan operand が整数かチェックしていない
      return BoolType.new

    when :OP_LT, :OP_GT, :OP_LE, :OP_GE, :OP_EQ, :OP_NE, :OP_LAND, :OP_LOR, :OP_CEX, :CAST
      cdl_error( "E1014 $1: elements_get_type: sorry not supported" , elements[0] )
    end

    return nil
  end

  # 式が size_is, count_is, string の引数である場合の方向のチェック
  def elements_check_dir_for_param( elements, namedList, dir, spec )
   # dir ： 元の引数の方向
   # direct: size_is などの引数の変数の方向

    case elements[0]
    when :IDENTIFIER
      nsp = elements[1]
      if nsp.is_name_only? then
        paramdecl = namedList.get_item( nsp.get_name )
      else
        paramdecl = nil
      end

      return unless paramdecl      # if nil already error in element_get_type

      direct = paramdecl.get_direction
      judge = false
      case spec
      when "size_is", "string"
        case dir
        when :IN, :OUT, :INOUT, :SEND
          judge = true if ( direct == :IN || direct == :INOUT )
          req_direct = "in or inout"
        when :RECEIVE
          judge = true if ( direct == :OUT || direct == :INOUT )
          req_direct = "out or inout"
        end

      when "count_is"
        case dir
        when :IN, :SEND
          judge = true if ( direct == :IN || direct == :INOUT )
          req_direct = "in or inout"
        when :OUT, :RECEIVE     # mikan out で count_is のみ指定されている場合 in でなくてはならない
          judge = true if ( direct == :OUT || direct == :INOUT )
          req_direct = "out or inout"
        when :INOUT
          judge = true if ( direct == :INOUT )
          req_direct = "inout"
        end
      end

      if judge == false then
        cdl_error( "E1015 \'$1\': direction mismatch for $2, $3 required" , nsp.get_path_str, spec, req_direct )
      end

    when :INTEGER_CONSTANT, :FLOATING_CONSTANT, :OCTAL_CONSTANT, :HEX_CONSTANT, :CHARACTER_LITERAL, :STRING_LITERAL_LIST
      return true

    # 単項演算子
    when :OP_U_ASTER, :OP_SIZEOF_EXPR, :OP_SIZEOF_TYPE, :OP_U_PLUS, :OP_U_MINUS, :OP_U_TILDE, :OP_U_EXCLAM, :CAST, :OP_U_AMP, :PARENTHESES,
      elements_check_dir_for_param( elements[1], namedList, dir, spec )

    # 2項演算子
    when :OP_SUBSC, :OP_DOT, :OP_REF, :OP_MULT, :OP_DIV, :OP_REMAIN, :OP_ADD, :OP_SUB, :OP_LSFT, :OP_RSFT, :OP_LT, :OP_GT, :OP_LE, :OP_GE, :OP_EQ, :OP_NE, :OP_AND, :OP_EOR, :OP_OR, :OP_LAND, :OP_LOR
      return elements_check_dir_for_param( elements[1], namedList, dir, spec ) && elements_check_dir_for_param( elements[2], namedList, dir, spec )

    # 3項演算子
    when :OP_CEX
      return elements_check_dir_for_param( elements[1], namedList, dir, spec ) && elements_check_dir_for_param( elements[2], namedList, dir, spec ) && elements_check_dir_for_param( elements[3], namedList, dir, spec )

    else
      cdl_error( "E1016 $1: elements_check_dir_for_param: sorry not supported" , elements[0] )
    end

  end

  #Express# get_allocator_rhs_elem
  #alloc_type::Symbol  :NORMAL_ALLOC|:INTERNAL_ALLOC|:RELAY_ALLOC
  #式がアロケータ指定子の右辺として妥当かチェックし、正しければ分解した値を返す
  #return:
  #  :NORMAL_ALLOC      [ cell_nsp, ep_name ]               # rhs = cell_nsp.ep_name    ex) Alloc.eAlloc
  #  :INTERNAL_ALLOC    [ ep_name ]                         # rhs = ep_name             ex) eAlloc
  #  :RELAY_ALLOC       [ cp_name, func_name, param_name ]  # rhs = cp_name.func_name.param_name
  def get_allocator_rhs_elements( alloc_type )
    ele = @elements
    case alloc_type 
    when :NORMAL_ALLOC
      if ele[0] != :OP_DOT || ele[1][0] != :IDENTIFIER then   #1
        cdl_error( "E1017 $1: rhs not \'Cell.ePort\' form" , ele[0].to_s )
        return nil
      end
      cell_nsp  = elements[1][1]
      port_name = elements[2].val
      return [ cell_nsp, port_name ]
    when :INTERNAL_ALLOC
      if( ele[0] == :IDENTIFIER )then
        if ele[1].is_name_only? then
          return [ ele[1].get_path[0] ]  # mikan a::b
        else
          cdl_error( "E1018 $1: namespace cannot be specified", ele[1].to_s )
        end
      else
        cdl_error( "E1019 $1: rhs not in 'allocator_entry_port' form", ele[1].to_s )
      end
    when :RELAY_ALLOC
      if( ele[0] != :OP_DOT ||
          ele[1][0] != :OP_DOT || ele[1][1][0] != :IDENTIFIER || ! ele[1][1][1].is_name_only? ||
          ! ele[1][2].instance_of?( Token ) || ! ele[2].instance_of?( Token ) )then   #1
        cdl_error( "E1020 rhs not in 'call_port.func.param' form ($1)" , ele[0].to_s )   # S1086
      end
      func_name = ele[1][2]; cp_name = ele[1][1][1].get_name; param_name = ele[2].to_sym
      return [ cp_name, func_name, param_name ]
    end
    return  nil
  end

  #Expression#Expression のクローンを作成する
  def clone_for_composite
    cl = self.clone
    elements = clone_elements @elements
    cl.set_elements elements
    return cl
  end

  #Expression#elements のクローンを作成
  #elements::Array
  # このメソッドは、Array のディープコピーを行う
  def clone_elements elements
    elements = elements.clone
    elements.map!{ |ele|
      if ele.instance_of? Array
        clone_elements ele
      else
        ele
      end
    }
    return elements
  end

  def set_elements elements
    @elements = elements
  end

  #=== Expression#セル結合の式を解析する
  # Cell.eEntry  => [ :OP_DOT, [ :IDENTIFIER, token ], token ]
  # Cell.eEntry[expression] => [ :OP_SUBSC, [ :OP_DOT, [ :IDENTIFIER, token ], token ], expression ]
  # Return: [ NamespacePath(cell_name), Integer(subscript) or nil, Token(port_name)]
  def analyze_cell_join_expression
    # 右辺の Expression の要素を取り出す
    elements = @elements
    if elements[0] == :OP_SUBSC then  # 右辺：受け口配列？
      # elements = [ :OP_SUBSC, [ :OP_DOT, [ :IDENTIFIER, token ], token ], expression ]
      subscript = elements[2].eval_const(nil)  # 受け口配列の添数
      elements  = elements[1]          # mikan 配列だった場合
    else
      subscript = nil
    end

    # elements = [ :OP_DOT, [ :IDENTIFIER, token ], token ]
    if elements[0] != :OP_DOT || elements[1][0] != :IDENTIFIER then   #1
      return nil
    end

    nsp = elements[1][1]         # NamespacePath
    port_name = elements[2].val

    return [ nsp, subscript, port_name]
  end

  #=== Expression# セルへの結合の式を生成する
  #nsp:: NamespacePath
  #subscript:: Integer
  #port_name:: Symbol
  # analyze_cell_join_expression と対になっている
  def self.create_cell_join_expression( nsp, subscript, port_name, locale = nil )
    if ! port_name.instance_of?( Symbol ) then
      raise "port_name: not Symbol"
    end

    if subscript then
      elements = [ :OP_SUBSC, [ :OP_DOT, [ :IDENTIFIER, nsp ],
                                Token.new( port_name, nil, nil, nil ) ],
                   Expression.create_integer_constant( subscript, @locale ) ]
    else
      elements = [ :OP_DOT, [ :IDENTIFIER, nsp ], Token.new( port_name, nil, nil, nil ) ]
    end
    return Expression.new( elements, locale )
  end

  #=== Expression#整数定数の式を生成する
  #val:: Integer : 値： 整数
  def self.create_integer_constant( val, locale = nil )
    if val != Integer( val ) || val < 0 then
      raise "create_integer_constant: not integer or negative: #{val}"
    end
    Expression.new( [ :INTEGER_CONSTANT, Token.new( val, nil, nil, nil ) ], locale )
  end

  #=== Expression#単一の識別子の式を解析する
  # Identifier  => [ :IDENTIFIER, token ]
  # Return: NamespacePath(Identifier)
  def analyze_single_identifier
    # 右辺の Expression の要素を取り出す
    elements = @elements
    if elements[0] == :IDENTIFIER
      return elements[1]
    else
      return nil
    end
  end

  #=== Expression#
  #nsp:: NamespacePath :  参照するもの識別子
  def self.create_single_identifier( nsp, locale )
    if ! nsp.instance_of?( NamespacePath ) then
      raise "create_single_identifier: not NamespacePath: #{nsp.to_s}"
    end
    Expression.new( [ :IDENTIFIER, nsp ] )
  end

  #=== 評価可能かチェックする
  #*v:: 可変個引数（任意の型）
  # すべてが BaseVal の子クラス（値）であれば、評価可能と判断する
  def evaluable?( *v )
    v.each{ |val|
      if ! val.kind_of?( BaseVal ) then
        return false
      end
    }
    return true
  end

  private :elements_to_s, :elements_eval_const,  :elements_get_type

end


class C_EXP < Node
# @c_exp_string : string

  #c_exp_string::String
  #b_renew::Bool  : true なら C_EXP の clone 作成（エスケープ処理等をしない）
  def initialize( c_exp_string, b_renew = false )
    if b_renew then
      @c_exp_string = c_exp_string
    else
      # 前後の " を取り除く
      # str = c_exp_string.to_s.sub( /^\"(.*)\"$/, "\\1" )
      str = CDLString.remove_dquote c_exp_string.to_s
      @c_exp_string = CDLString.escape str
    end
  end

  #=== composite 用に C_EXP を clone する
  #ct_name::
  #cell_name::
  # composite の attribute に現れる C_EXP を文字列置換して生成しなおす．
  # この文字列置換は、意味解釈段階で行う．
  # 他の C_EXP の文字列置換は、コード生成段階で行う．
  def clone_for_composite( ct_name, cell_name, locale )
    dbgPrint "C_EXP: #{ct_name} #{cell_name} #{@c_exp_string}\n"

    @locale = locale
    str = @c_exp_string.gsub( /(^|[^\$])\$ct\$/, "\\1#{ct_name}" )
    str = str.          gsub( /(^|[^\$])\$cell\$/, "\\1#{cell_name}" )
    str = str.          gsub( /(^|[^\$])\$id\$/, "\\1#{ct_name}_#{cell_name}" )
    return C_EXP.new( str, true )
  end

  def get_c_exp_string
    @c_exp_string
  end

  #=== C_EXP を評価する
  # C_EXP の引き数文字列を返す
  # 本来 C_EXP は eval_const する対象ではないが、便宜上 eval_const で対応
  def eval_const( name_list, name_list2 = nil )
     return self
  end
  def eval_const2( name_list, name_list2 = nil, nest = nil )
     return self
  end

  def to_s
    @c_exp_string
  end

  def to_CDL_str
    return "C_EXP( \"#{to_s}\" )"
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "C_EXP: #{@c_exp_string}"
  end
end

