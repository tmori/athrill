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
#   $Id: bnf.y.rb 2633 2017-04-02 06:02:05Z okuma-top $
#++

class Generator
rule
# トップレベルの構文要素はcomponent_description
all: component_description

# Expr
##########################  式  ##########################
# K&Rの文法(プログラミング言語C 第2版 付録)と一部異なる
# argument_expression_list(関数引数), assignment_expression(代入)がない
# 式の result は、すべて配列で第一要素が識別シンボル、第二要素以下が引数

primary_expression
        : namespace_identifier
		{ result = [ :IDENTIFIER, val[0] ] }     #1ok
        | TRUE
		{ result = [ :BOOL_CONSTANT, true ] }
        | FALSE
		{ result = [ :BOOL_CONSTANT, false ] }
        | INTEGER_CONSTANT
		{ result = [ :INTEGER_CONSTANT, val[0] ] }
        | FLOATING_CONSTANT
		{ result = [ :FLOATING_CONSTANT, val[0] ] }
        | OCTAL_CONSTANT
		{ result = [ :OCTAL_CONSTANT, val[0] ] }
        | HEX_CONSTANT
		{ result = [ :HEX_CONSTANT, val[0] ] }
        | CHARACTER_LITERAL
		{ result = [ :CHARACTER_LITERAL, val[0] ] }
        | string_literal_list
		{ result = [ :STRING_LITERAL_LIST, val[0] ] }
        | '(' expression ')'
		{ result = [ :PARENTHESES, val[1].get_elements ] }

string_literal_list
        : STRING_LITERAL
        | string_literal_list STRING_LITERAL
		{
			# 連接した文字列を1つの文字列にまとめる
			str = "\"" + val[0].val.gsub( /\"(.*)\"/, "\\1" ) + val[1].val.gsub( /\"(.*)\"/, "\\1" ) + "\""
			result = Token.new( str, val[0].file, val[0].lineno, val[0].col )
		}

# 関数呼び出しと後置インクリメント、デクリメント演算子がない
postfix_expression
        : primary_expression
        | postfix_expression '[' expression ']'
		{ result = [ :OP_SUBSC, val[0], val[2] ] }
        | postfix_expression '.' IDENTIFIER
		{ result = [ :OP_DOT, val[0], val[2] ] }
        | postfix_expression '->' IDENTIFIER
		{ result = [ :OP_REF, val[0], val[2] ] }

# 前置インクリメント、デクリメント演算子がない
unary_expression
        : postfix_expression
        | unary_operator cast_expression
		{ result = [ val[0], val[1] ] }
        | SIZEOF unary_expression
		{ result = [ :OP_SIZEOF_EXPR, val[1] ] }
        | SIZEOF '(' type_name ')'
		{ result = [ :OP_SIZEOF_TYPE, val[1] ] }

unary_operator
        : '&'	{ result = :OP_U_AMP }
        | '*'	{ result = :OP_U_ASTER }
        | '+'	{ result = :OP_U_PLUS }
        | '-'	{ result = :OP_U_MINUS }
        | '~'	{ result = :OP_U_TILDE }
        | '!'	{ result = :OP_U_EXCLAM }

cast_expression
        : unary_expression
        | '(' type_name ')' cast_expression
		{  result = [ :CAST, val[1], val[3] ] }

multiplicative_expression
        : cast_expression
        | multiplicative_expression '*' cast_expression
		{ result = [ :OP_MULT, val[0], val[2] ]  }
        | multiplicative_expression '/' cast_expression
		{ result = [ :OP_DIV, val[0], val[2] ]  }
        | multiplicative_expression '%' cast_expression
		{ result = [ :OP_REMAIN, val[0], val[2] ]  }

additive_expression
        : multiplicative_expression
        | additive_expression '+' multiplicative_expression
		{ result = [ :OP_ADD, val[0], val[2] ]  }
        | additive_expression '-' multiplicative_expression
		{ result = [ :OP_SUB, val[0], val[2] ]  }

shift_expression
        : additive_expression
        | shift_expression '<<' additive_expression
		{ result = [ :OP_LSFT, val[0], val[2] ]  }
        | shift_expression '>>' additive_expression
		{ result = [ :OP_RSFT, val[0], val[2] ]  }

relational_expression
        : shift_expression
        | relational_expression '<' shift_expression
		{ result = [ :OP_LT, val[0], val[2] ]  }
        | relational_expression '>' shift_expression
		{ result = [ :OP_GT, val[0], val[2] ]  }
        | relational_expression '<=' shift_expression
		{ result = [ :OP_LE, val[0], val[2] ]  }
        | relational_expression '>=' shift_expression
		{ result = [ :OP_GE, val[0], val[2] ]  }

equality_expression
        : relational_expression
        | equality_expression '==' relational_expression
		{ result = [ :OP_EQ, val[0], val[2] ]  }
        | equality_expression '!=' relational_expression
		{ result = [ :OP_NE, val[0], val[2] ]  }

and_expression
        : equality_expression
        | and_expression '&' equality_expression
		{ result = [ :OP_AND, val[0], val[2] ]  }

exclusive_or_expression
        : and_expression
        | exclusive_or_expression '^' and_expression
		{ result = [ :OP_EOR, val[0], val[2] ]  }

inclusive_or_expression
        : exclusive_or_expression
        | inclusive_or_expression '|' exclusive_or_expression
		{ result = [ :OP_OR, val[0], val[2] ]  }

logical_and_expression
        : inclusive_or_expression
        | logical_and_expression '&&' inclusive_or_expression
		{ result = [ :OP_LAND, val[0], val[2] ]  }

logical_or_expression
        : logical_and_expression
        | logical_or_expression '||' logical_and_expression
		{ result = [ :OP_LOR, val[0], val[2] ]  }

conditional_expression
        : logical_or_expression
        | logical_or_expression '?' expression ':' conditional_expression
		{ result = [ :OP_CEX, val[0], val[2].get_elements, val[4] ]  }


# コンマ演算子が使えない
expression
        : conditional_expression
		{
			result = Expression.new( val[0] )
			# result.print
		}

constant_expression
        : conditional_expression
		{
			result = Expression.new( val[0] )
			# result.print

			# res = result.eval_const( nil )
			# if res then
			#   puts "val: #{res}"
			# else
			#   puts "val: nil"
			# end
		}



# Types
##########################  宣言  ##########################
# 宣言もK&Rと一部異なる

# declarationはセルの属性で使われる
# K&Rとの違い: storage classが指定できない、型が省略できない
declaration
        : type_specifier_qualifier_list init_declarator_list ';'
		{
			val[1].each { |i|	# i: Decl
				i.set_type( val[0] )
			}
			result = val[1]
		}

# declaration_specifiersは関数のパラメータで使われるが、
# type_specifier_qualifier_listで十分かもしれない
# Oyama
# const, volatile は単独で型にならないので変形
# const と volatile が同居することはないので、繰返し指定できないようにした
# type_specifier も繰返し指定できる必要はない (singed など単独で型にはならない)
declaration_specifiers
        : type_specifier
        | type_qualifier type_specifier
		{
			val[1].set_qualifier( val[0] )
			result = val[1]
		}

init_declarator_list
        : init_declarator
		{ result = [val[0]] }
        | init_declarator_list ',' init_declarator
		{ result << val[2] }

init_declarator
        : declarator
        | declarator '=' initializer
		{ val[0].set_initializer( val[2] ) }

# INT8から下はK&Rにない
# Oyama
# signed, unsigned は単独で型にならないので、構文要素として分離
type_specifier
        : VOID	{ result = VoidType.new }
        | FLOAT32_T	{ result = FloatType.new(32) }
        | DOUBLE64_T	{ result = FloatType.new(64) }
        | struct_specifier
        | enum_specifier
        | TYPE_NAME	{ result = DefinedType.new( val[0].val ) }
#        | IDENTIFIER	{ result = DefinedType.new( val[0].val ) }   # reduce/reduce conflict が起こってしまう
        | sign_int_type
        | char_type
        | BOOL_T	{ result = BoolType.new }
#        | BOOL	{
#			Generator.warning( "W5001 bool: obsolete type. use bool_t"  )
#			result = BoolType.new
#		}
        | FLOAT	{
			Generator.warning( "W5002 float: obsolete type. use float32_t"  )
			result = FloatType.new(32)
		}
        | DOUBLE {
			Generator.warning( "W5003 double: obsolete type. use double64_t"  )
			result = FloatType.new(64)
		}
        | DESCRIPTOR '(' namespace_identifier ')' {          # namespace_identifier: signature name
			result = DescriptorType.new( val[2] )
		}

char_type
        : CHAR_T	{ result = IntType.new( -1 ) }
        | SCHAR_T
		{
			result = IntType.new( -1 )
			result.set_sign( :SIGNED, true )
		}
        | UCHAR_T
		{
			result = IntType.new( -1 )
			result.set_sign( :UNSIGNED, true )
		}

int_type
        : CHAR	{
			# Generator.warning( "W5004 char: obsolete type. use char_t"  )
			result = IntType.new( -11 )
		}
        | SHORT	{ result = IntType.new( -2 ) }
        | INT		{ result = IntType.new( -3 ) }
        | LONG	{ result = IntType.new( -4 ) }
#        | INTPTR	{ result = IntType.new( -5 ) }
#        | INT8	{
#			Generator.warning( "W5005 int8: obsolete. use int8_t"  )
#			result = IntType.new(  8 )
#		}
#        | INT16	{
#			Generator.warning( "W5006 int16: obsolete. use int16_t"  )
#			result = IntType.new( 16 )
#		}
#        | INT32	{
#			Generator.warning( "W5007 int32: obsolete. use int32_t"  )
#			result = IntType.new( 32 )
#		}
#        | INT64
#		{
#			Generator.warning( "W5008 int64: obsolete. use int64_t"  )
#			result = IntType.new( 64 )
#		}
#        | INT128
#		{
#			Generator.warning( "W5009 int64: obsolete. use int64_t"  )
#			result = IntType.new( 128 )
#		}
        | INT8_T	{ result = IntType.new( 8 ) }
        | INT16_T	{ result = IntType.new( 16 ) }
        | INT32_T	{ result = IntType.new( 32 ) }
        | INT64_T	{ result = IntType.new( 64 ) }
        | INT128_T	{ result = IntType.new( 128 ) }
        | UINT8_T
		{
			result = IntType.new( 8 )
			result.set_sign( :UNSIGNED, true )
		}
        | UINT16_T
		{
			result = IntType.new( 16 )
			result.set_sign( :UNSIGNED, true )
		}
        | UINT32_T
		{
			result = IntType.new( 32 )
			result.set_sign( :UNSIGNED, true )
		}
        | UINT64_T
		{
			result = IntType.new( 64 )
			result.set_sign( :UNSIGNED, true )
		}
        | UINT128_T
		{
			result = IntType.new( 128 )
			result.set_sign( :UNSIGNED, true )
		}

sign    # TECS では signed, unsigned 単独では型にできない
        : SIGNED	{ result = :SIGNED }
        | UNSIGNED	{ result = :UNSIGNED }

# result[0] :CHAR などのトークン、result[1] :CONST, :VOLATILE など
sign_int_type
        : sign int_type
		{
			val[1].set_sign( val[0] )
			result = val[1]
		}
        | int_type

# K&Rのstruct_or_union_specifierに相当するが、unionは使えない
struct_specifier		# mikan
        : STRUCT struct_tag '{'
		{ StructType.set_define( true )  }
	   struct_declaration_list '}'
		{
			StructType.end_of_parse
			result = val[1]
		}
        | STRUCT
		{
			# tag が無い場合、内部名を与える
			result = StructType.new( :"$TAG_#{@@no_struct_tag_num}" )
			@@no_struct_tag_num += 1
			StructType.set_define( true )
		}
	   '{' struct_declaration_list '}'
		{
			StructType.end_of_parse
			result = val[1]
		}
        | STRUCT struct_tag   # mikan struct_tag は namespace 対応が必要
		{
			StructType.set_define( false )
			StructType.end_of_parse
			result = val[1]
		}

struct_declaration_list
        : struct_declaration
        | struct_declaration_list struct_declaration

struct_tag:
	IDENTIFIER
		{ result = StructType.new( val[0].val ) }

# ポインタ修飾子を追加
struct_declaration
        :                                type_specifier_qualifier_list struct_declarator_list ';'
		{
			val[1].each { |i|	# i: Decl
				i.set_type( val[0] )
				i.set_kind( :MEMBER )
				i.check
				StructType.new_member( i )
			}
			result = val[1]
		}
        | spec_L pointer_specifier_list spec_R type_specifier_qualifier_list struct_declarator_list ';'
		{
			val[4].each { |i|	# i: Decl
				i.set_type( val[3] )
				i.set_kind( :MEMBER )
  				i.set_specifier_list val[1]
				i.check
				StructType.new_member( i )
			}
			result = val[4]
		}

pointer_specifier_list
        : pointer_specifier                               { result = [ val[0] ] }
        | pointer_specifier_list ',' pointer_specifier    { result <<  val[2] }

pointer_specifier
        : STRING				{ result = [:STRING,-1] }
        | STRING   '(' expression ')'	{ result = [:STRING,val[2]] }
        | SIZE_IS  '(' expression ')'	{ result = [:SIZE_IS,val[2]] }
        | COUNT_IS '(' expression ')'	{ result = [:COUNT_IS,val[2]] }


# K&Rのspecifier_qualifier_listと同じ
# 名前がまぎらわしかったのでtype_を付けた
type_specifier_qualifier_list
# Oyama type_specifier を繰り返して指定することはなくなった (sign_int_type としたため）
#        : type_specifier type_specifier_qualifier_list
        : type_specifier
        | type_qualifier type_specifier_qualifier_list
		{
			val[1].set_qualifier( val[0] )
			result = val[1]
		}
# mikan Oyama type_qualifier だけでは型指定にならない : 構文エラーとするより、意味エラーとした方が親切
#        | type_qualifier


struct_declarator_list
        : struct_declarator
		{ result = [ val[0] ] }
        | struct_declarator_list ',' struct_declarator
		{ result << val[2] }

# ビットフィールドは使えない
struct_declarator
        : declarator

# enumの種類を追加
enum_specifier		# mikan
        : enum_type            '{' enumerator_list '}'
        | enum_type IDENTIFIER '{' enumerator_list '}'
        | enum_type IDENTIFIER

enum_type
        : ENUM	{ result = EnumType.new( -1 ) }
        | ENUM8	{ result = EnumType.new( 8 ) }
        | ENUM16	{ result = EnumType.new( 16 ) }
        | ENUM32	{ result = EnumType.new( 32 ) }
        | ENUM64	{ result = EnumType.new( 64 ) }
        | ENUM128	{ result = EnumType.new( 128 ) }

enumerator_list
        : enumerator
        | enumerator_list ',' enumerator

enumerator
        : IDENTIFIER
        | IDENTIFIER '=' constant_expression

type_qualifier
        : CONST	{ result = :CONST }
        | VOLATILE	{ result = :VOLATILE }

declarator
        : pointer direct_declarator
		{
			val[1].set_type( val[0] )
			result = val[1]
		}
        | direct_declarator

direct_declarator		# mikan
        : IDENTIFIER
		{ result = Decl.new( val[0].val ) }
        | '(' declarator ')'
		{ result = val[1] }
        | direct_declarator '[' constant_expression ']'
		{
			val[0].set_type( ArrayType.new( val[2] ) )
			result = val[0]
		}
        | direct_declarator '[' ']'
		{
			val[0].set_type( ArrayType.new )
			result = val[0]
		}
        | direct_declarator '(' parameter_type_list ')'
		{
			val[0].set_type( FuncType.new( val[2] ) )
			result = val[0]
		}
#        | direct_declarator '(' identifier_list ')'  # これは何のために必要？ 060211
        | direct_declarator '(' ')'
		{
			Generator.warning( "W5010 need 'void' for no parameter"  )
			val[0].set_type( FuncType.new )
			result = val[0]
		}

pointer
        : '*'
		{ result = PtrType.new }
        | '*' type_qualifier
		{
			result = PtrType.new
			result.set_qualifier( val[1] )
		}
        | '*' pointer
		{
			val[1].set_type(PtrType.new)
			result = val[1]
		}
        | '*' type_qualifier pointer
		{
			ptrtype = PtrType.new
			ptrtype.set_qualifier( val[1] )
			val[2].set_type( ptrtype )
			result = val[2]
		}


parameter_type_list
        : parameter_list
        | parameter_list ',' '...'
		# mikan 可変長パラメータ

parameter_list
        : parameter_declaration
		{ result = ParamList.new( val[0] ) }
        | parameter_list ',' parameter_declaration
		{
			val[0].add_param( val[2] )
			# result = val[0] 不要
		}


# パラメータ修飾子を追加
parameter_declaration
#        : spec_L parameter_specifier_list spec_R declaration_specifiers declarator
        : parameter_specifier_list_bracket declaration_specifiers  declarator
		{
			val[2].set_kind( :PARAMETER )
			paramdecl = ParamDecl.new( val[2], val[1], val[0] )
			val[2].check
			result = paramdecl
		}

	# 以下はエラーとする
        | declaration_specifiers declarator # parameter_specifier なしは扱わない
		{
			Generator.error( "G1001 need specifier for \'$1\'" , val[1].get_name )
			result = nil
		}
        | declaration_specifiers	# 仮引数なしは、とりあえず扱わない 060210
		{
			unless val[0].instance_of?( VoidType ) then
				Generator.error( "G1002 need parameter name"  )
			end
			result = nil
		}
#        | '[' parameter_specifier_list ']' declaration_specifiers # 同 060210
        | parameter_specifier_list_bracket declaration_specifiers # 同 060210
		{
			unless val[1].instance_of?( VoidType ) then
				Generator.error( "G1003 need parameter name"  )
			end
			result = nil
		}

parameter_specifier_list_bracket
        :  spec_L parameter_specifier_list spec_R { result = val[1] }
#        :  '[' parameter_specifier_list  ']'  { result = val[1] }

parameter_specifier_list
        : parameter_specifier	{ result = val[0] }
        | parameter_specifier_list ',' parameter_specifier
		{ result = result + val[2] }

parameter_specifier
        : IN					{ result = [ [:IN]  ] }
        | OUT					{ result = [ [:OUT] ] }
        | INOUT					{ result = [ [:INOUT] ] }
        | SEND     '(' namespace_identifier ')'	{ result = [ [:SEND,   val[2]] ] }   #1ok allocator
        | RECEIVE  '(' namespace_identifier ')'	{ result = [ [:RECEIVE,val[2]] ] }   #1ok allocator
        | STRING				{ result = [ [:STRING,nil] ] }
        | STRING   '(' expression ')'	{ result = [ [:STRING,  val[2]] ] }
        | SIZE_IS  '(' expression ')'	{ result = [ [:SIZE_IS, val[2]] ] }
        | SIZE_IS  '(' expression ',' constant_expression ')'
		{
			result = [ [:SIZE_IS,val[2]], [:MAX_IS, val[4]] ]
		}
        | COUNT_IS '(' expression ')'	{ result = [ [:COUNT_IS,val[2]] ] }
        | NULLABLE		{ result = [ [:NULLABLE] ] }

type_name
        : type_specifier_qualifier_list
        | type_specifier_qualifier_list abstract_declarator
		{
			if val[1] then
				val[1].set_type( val[0] )
				result = val[1]
			else
				# エラー：仮で val[0] を返す
				result = val[0]
			end
		}
		# mikan abstract_declarator が pointer 以外ではうまく動かない、とりあえず '*' CAST のみ救った

abstract_declarator		# mikan
        : pointer
        | direct_abstract_declarator
        | pointer direct_abstract_declarator

direct_abstract_declarator
        : '(' abstract_declarator ')'
		{ result = val[1] }  # 関数ポインタ型を救う
        | '[' ']'
		{
			Generator.error( "G1004 impossible array type"  )
			result = nil
		}
        | '[' constant_expression ']'
		{
			Generator.error( "G1005 impossible array type"  )
			result = nil
		}
        | direct_abstract_declarator '[' ']'
		{
			Generator.error( "G1006 impossible array type"  )
			result = nil
		}
        | direct_abstract_declarator '[' constant_expression ']'
		{
			Generator.error( "G1007 impossible array type"  )
			result = nil
		}
        | '(' ')'
		{
			Generator.error( "G1008 impossible function type"  )
			result = nil
		}
        | '(' parameter_type_list ')'
        | direct_abstract_declarator '(' ')'
		{
			Generator.warning( "W5011 need 'void' for no parameter"  )
			val[0].set_type( FuncType.new )
			result = val[0]
		}
        | direct_abstract_declarator '(' parameter_type_list ')'
		{
			val[0].set_type( FuncType.new( val[2] ) )
			result = val[0]
		}

# assignment_expressionをconstant_expressionに変更
initializer			# mikan
        : constant_expression
		{ result = val[0] }
        | '{' initializer_list '}'
		{ result = val[1] }
        | '{' initializer_list ',' '}'
		{ result = val[1] }
#        | C_EXP '(' STRING_LITERAL ')'
        | C_EXP '(' string_literal_list ')'
		{ result = C_EXP.new( val[2] ) }

initializer_list
        : initializer
		{
			result = [ val[0] ]
		}
        | initializer_list ',' initializer
		{
			val[0] << val[2]
			result = val[0]
		}


##########################  ここからはCDL独自  ##########################

#トップレベルの構文規則
#コンポーネント記述
component_description
        : component_description specified_statement
        | component_description location_information
        | component_description tool_info
        | 

specified_statement
        : statement
        | spec_L statement_specifier_list spec_R statement
		{
			obj = val[3]
			if obj.kind_of?( Cell ) || obj.kind_of?( Signature ) || obj.kind_of?( Celltype ) || obj.kind_of?( CompositeCelltype )then
                                # cell, signature 以外は、指定子を置けない
			else
              Generator.get_statement_specifier   # クリア
              Generator.error( "G1009 unexpected specifier"  )
			end
		}
		# これと同じ記述が composite_celltype にもある

statement
        : typedef
        | const_statement
        | namespace
        | signature
        | celltype
        | cell
        | composite_celltype
        | enum_specifier ';'
        | struct_specifier ';'
        | region
        | import
        | import_C
        | generate_statement
        | error   # エラー回復ポイント

	
statement_specifier_list
        : statement_specifier
		{ Generator.add_statement_specifier val[0]	}
        | statement_specifier_list ',' statement_specifier
		{ Generator.add_statement_specifier val[2] }

statement_specifier
        : ALLOCATOR '(' alloc_list ')'                       # cell
		{ result = [ :ALLOCATOR, val[2] ] }
        | CALLBACK                                           # signature
		{ result = [ :CALLBACK ] }
        | CONTEXT '(' string_literal_list ')'                # signature
		{ result = [ :CONTEXT, val[2].val ] }
        | DEVIATE                                            # signature
		{ result = [ :DEVIATE ] }
        | ID '(' constant_expression ')'                     # cell
		{ result = [ :ID, val[2] ] }
        | PROTOTYPE                                          # cell
		{ result = [ :PROTOTYPE ] }
        | RESTRICT  '(' restrict_list ')'                    # cell
		{ result = [ :RESTRICT, val[2] ] }
        | SINGLETON  { result = [:SINGLETON] }               # celltype, composite
        | IDX_IS_ID  { result = [:IDX_IS_ID] }               # celltype, composite (composite: no-effective)
        | ACTIVE     { result = [:ACTIVE] }                  # celltype, composite
        | GENERATE '(' plugin_name ',' plugin_arg  ')'       # celltype, cell
		{ result = [:GENERATE, val[2].val, val[4].val] }

alloc_list
        : alloc			{ result = [ val[0] ] }
        | alloc_list ',' alloc	{ result << val[2] }

alloc
        : IDENTIFIER '.' IDENTIFIER '.' IDENTIFIER '=' initializer
		{  result = [ :NORMAL_ALLOC, val[0], nil, val[2], val[4], val[6] ] }
        | IDENTIFIER '[' constant_expression ']' '.' IDENTIFIER '.' IDENTIFIER '=' initializer
		{  result = [ :NORMAL_ALLOC, val[0], val[2], val[5], val[7], val[9] ] }
# mikan 将来的な拡張 ('*' でまとめて指定可能とする)
#        | IDENTIFIER '.' IDENTIFIER '.' '*'        '=' initializer
#		{  result = [ val[0], val[ ], val[ ], val[ ] ] }
#        | IDENTIFIER '.' '*' '.' '*'               '=' initializer
#		{  result = [ val[0], val[ ], val[ ], val[ ] ] }

restrict_list
        : restrict
		{	result = [val[0]]		}
        | restrict_list ',' restrict
		{	result << val[2]		}

restrict
        : port_name '=' '{' region_name_list '}'
		{	result = [ val[0].val, nil, val[3] ]		}
        | port_name '.' IDENTIFIER '=' '{' region_name_list '}'
		{	result = [ val[0].val, val[2].val, val[5] ]		}

region_name_list
        : IDENTIFIER
		{	result = [val[0].val]		}
        | region_name_list ',' IDENTIFIER
		{	result << val[2].val		}

const_statement
        : declaration   # 定数定義
		{
			val[0].each { |decl|
				decl.set_kind( :CONSTANT )
				Namespace.new_const_decl( decl )
				decl.check
			}
		}

import_C
        : IMPORT_C '(' STRING_LITERAL ')' ';'
		{
			@@import_C = true
			Import_C.new( val[2] )
			@@import_C = false
		}
        | IMPORT_C '(' STRING_LITERAL ',' STRING_LITERAL ')' ';'
		{
			@@import_C = true
			Import_C.new( val[2], val[4] )
			@@import_C = false
		}

import
        : IMPORT '(' STRING_LITERAL ')' ';'
		{ Import.new( val[2] ) }
        | IMPORT '(' AB_STRING_LITERAL ')' ';'
		{ Import.new( val[2], true ) }

generate_statement
#        : GENERATE '(' plugin_name ',' namespace_identifier ',' STRING_LITERAL ')' ';'  #1ok signature plugin
        : GENERATE '(' plugin_name ',' namespace_identifier ',' plugin_arg ')' ';'  #1ok signature plugin
		{ Generate.new( val[2].val, val[4], val[6] ) }

typedef
        : TYPEDEF type_specifier_qualifier_list declarator_list ';'
		{
			val[2].each{ |i|       # i:Decl
				i.set_kind( :TYPEDEF )
    		}
			Typedef.new_decl_list( val[1], val[2] )
			val[2].each{ |i|       # i:Decl
				i.check
			}
		}
        | TYPEDEF '[' typedef_specifier ']' type_specifier_qualifier_list declarator_list ';'
		{
			val[5].each{ |i|       # i:Decl
				i.set_kind( :TYPEDEF )
			}
			Typedef.new_decl_list( val[4], val[5] )
			val[5].each{ |i|       # i:Decl
				i.check
			}
		}
		# mikan   typedef_specifier 未処置


typedef_specifier
        : STRING
        | STRING '(' expression ')'

declarator_list
        : declarator
		{ result = [ val[0] ] }
        | declarator_list ',' declarator
		{ result << val[2] }

namespace
        : NAMESPACE namespace_name '{' statement_list '}' ';'
		{val[1].end_of_parse}

namespace_name
        : IDENTIFIER
		{result = Namespace.new(val[0].val)}
		# namespace インスタンスに statement を記憶させるためここで生成

statement_list
        : specified_statement
        | statement_list specified_statement

namespace_identifier
        : IDENTIFIER		{ result = NamespacePath.new( val[0].val, false ) }
        | '::' IDENTIFIER	{ result = NamespacePath.new( val[1].val, true ) }
        | namespace_identifier '::' IDENTIFIER
		{ result = val[0].append!( val[2].val ) }

#シグニチャ
signature
        : SIGNATURE signature_name '{' function_head_list '}' ';'
		{ result = val[1].end_of_parse( val[3] ) }

signature_name
        : IDENTIFIER
		{result = Signature.new( val[0].val ) }

function_head_list     # result:  function_head の配列
#        : function_head
#		{ result = NamedList.new( val[0], "function" ) }
        :
		{ result = NamedList.new( nil, "function" ) }
        | function_head_list function_head
		{ result = val[0].add_item( val[1] ) }

function_head
        :               type_specifier_qualifier_list declarator ';'
		{
			# val[1]: Decl
			if val[1].is_function? then
				result = FuncHead.new( val[1], val[0], false )
				val[1].set_kind :FUNCHEAD
				val[1].check
			else
				# mikan 関数の配列も以下のメッセージになる
				Generator.error( "G1010 Not function"  )
				result = nil
			end
		}
        | spec_L ONEWAY spec_R type_specifier_qualifier_list declarator ';'
		{
			if val[4].is_function? then
				result = FuncHead.new( val[4], val[3], true )
			else
				Generator.error( "G1011 Not function"  )
				result = nil
			end
		}


#セルタイプ
celltype
        : CELLTYPE celltype_name '{' celltype_statement_list '}' ';'
		{
			val[1].end_of_parse
			result = val[1]
		}

celltype_name
        : IDENTIFIER
		{ result = Celltype.new(val[0].val) }

celltype_statement_list
        : specified_celltype_statement
        | celltype_statement_list specified_celltype_statement

specified_celltype_statement
        : celltype_statement
		{
			if val[0].kind_of? Port then
				Celltype.new_port( val[0] )
			end
		}
        | spec_L celltype_statement_specifier_list spec_R celltype_statement
		{
			if val[3].kind_of? Port then
				val[3].set_specifier val[1]  # 設定順序あり
				Celltype.new_port( val[3] )
			else
				# Port 以外では指定子はエラー
				Generator.error( "G1012 $1 : cannot put specifier here" , val[1] )
			end
		}

celltype_statement
        : port
        | attribute
        | var
        | require
        | factory
#        | error       # エラー回復ポイント  (#513 無限ループに陥るケースがあるので、ここでのエラー回復は取りやめ)

celltype_statement_specifier_list
        : celltype_statement_specifier
		{ result = [ val[0] ] }
        | celltype_statement_specifier_list ',' celltype_statement_specifier
		{ result << val[2] }

celltype_statement_specifier
        : INLINE { result = [ :INLINE ] }
        | ALLOCATOR '(' alloc_list2 ')'    { result = [ :ALLOCATOR, val[2] ] }
        | OPTIONAL { result = [ :OPTIONAL ] }
        | REF_DESC { result = [ :REF_DESC ] }
        | DYNAMIC { result = [ :DYNAMIC ] }
        | OMIT { result = [ :OMIT ] }

alloc_list2
        : alloc2			{ result = [ val[0] ] }    # 受け口のアロケータ指定
        | alloc				{ result = [ val[0] ] }    # 内部セルのアロケータ指定
        | alloc_list2 ',' alloc2	{ result << val[2] }
        | alloc_list2 ',' alloc		{ result << val[2] }

alloc2
        : IDENTIFIER '.' IDENTIFIER '=' initializer    # 内部アロケータ (デバドラ向きアロケータ)指定
		{  result = [ :INTERNAL_ALLOC, val[0].val, val[2].val, val[4] ] }
        | IDENTIFIER '.' IDENTIFIER '<=' initializer   # 多段リレーモデル向きアロケータ指定
		{  result = [ :RELAY_ALLOC, val[0].val, val[2].val, val[4] ] }


#呼び口、受け口
port
        : port_type namespace_signature_name port_name ';'
		{ result = Port.new( val[2].val, val[1], val[0] ) }
        | port_type namespace_signature_name port_name '[' ']' ';'
		{ result = Port.new( val[2].val, val[1], val[0], "[]" ) }
        | port_type namespace_signature_name port_name '[' array_size ']' ';'
		{ result = Port.new(val[2].val, val[1], val[0], val[4]) }
        | port_type namespace_signature_name port_name '<=' namespace_identifier '.' IDENTIFIER ';'    #1ok reverse require
		{ result = Port.new( val[2].val, val[1], val[0], nil, val[4], val[ 6 ].val ) }

port_type
        : CALL	{ result = :CALL }
        | ENTRY	{ result = :ENTRY }

namespace_signature_name
        : namespace_identifier  #1ok
		{ result = val[0] }

port_name
        : IDENTIFIER

array_size
        : constant_expression


#属性
attribute
        : ATTRIBUTE '{' attribute_declaration_list '}' ';'
		{ result = nil }

attribute_declaration_list
        : attribute_declaration
		{ Celltype.new_attribute( val[0] ) }
        | attribute_declaration_list attribute_declaration
		{ Celltype.new_attribute( val[1] ) }


attribute_declaration
        :                             declaration
		{
			val[0].each{ |i|       # i:Decl
				i.set_kind( :ATTRIBUTE )
				i.check
			}
			result = val[0]
		}
        | spec_L attribute_specifier spec_R declaration
		{
			val[3].each{ |i|       # i:Decl
				i.set_kind( :ATTRIBUTE )   # 設定順序あり
				i.set_specifier_list( [val[1]] )
				i.check
			}
			result = val[3]
		}

attribute_specifier
        : OMIT     { result = [:OMIT] }
        | SIZE_IS  '(' expression ')'	{ result = [:SIZE_IS,val[2]] }
        | CHOICE '=' '{' choice_list '}' {  result = [:CHOICE,val[3]] }

choice_list
        : choice_list ',' choice_element {  result << val[2] }
        | choice_element                 {  result = [ val[0] ] }

choice_element
        : STRING_LITERAL

#内部変数
var
        : VAR '{' var_declaration_list '}' ';'
		{ result = nil }

var_declaration_list
        : var_declaration
   		{ Celltype.new_var( val[0] ) }
        | var_declaration_list var_declaration
   		{ Celltype.new_var( val[1] ) }

var_declaration
        : declaration
		{
			val[0].each{ |i|       # i:Decl
				i.set_kind( :VAR )
				i.check
			}
		}
        | spec_L var_specifier spec_R declaration
		{
			val[3].each{ |i|       # i:Decl
				i.set_kind( :VAR )   # 設定順序あり
				i.set_specifier_list( [val[1]] )
				i.check
			}
			result = val[3]
		}

var_specifier
        : SIZE_IS  '(' expression ')'	{ result = [:SIZE_IS,val[2]] }

# リクワイア
require
        : REQUIRE namespace_identifier '.' IDENTIFIER';'                            # mikan namespace #1
		{
			Celltype.new_require( val[1], val[3] )
		}
        | REQUIRE IDENTIFIER '=' namespace_identifier '.' IDENTIFIER';'             #1
		{
			Celltype.new_require( val[3], val[5], val[1].val )
		}

#ファクトリ
factory
        : factory_head '{' factory_function_list '}' ';'

factory_head
        : FACTORY     { Factory.set_f_celltype( false ) }
        | CTFACTORY   { Factory.set_f_celltype( true ) }

factory_function_list
        :                                         # 空
        | factory_function_list factory_function

factory_function
        : factory_function_name '(' constant_expression ',' constant_expression  ')' ';'
		{ Factory.new( val[0].val, val[2], val[4], nil    ) }
        | factory_function_name '(' constant_expression ',' constant_expression ',' arg_list ')' ';'
		{ Factory.new( val[0].val, val[2], val[4], val[6] ) }

factory_function_name
        : IDENTIFIER

arg_list     # factory の write 関数の第三引数以降
        : IDENTIFIER
		{ result = [ [ :IDENTIFIER, val[0].val ] ] }
        | arg_list ',' IDENTIFIER
		{ result << [ :IDENTIFIER, val[2].val ] }
        | STRING_LITERAL
		{ result = [ [ :STRING_LITERAL, val[0].val ] ] }
        | arg_list ',' STRING_LITERAL
		{ result << [ :STRING_LITERAL, val[2].val ] }

#セル生成
cell
        : CELL namespace_celltype_name cell_name '{'
		{ Cell.new_def }
          join_list '}' ';'
		{ result = Cell.end_of_parse true }
        | CELL namespace_celltype_name cell_name ';'   # oyama プロトタイプ宣言
			{ result = Cell.end_of_parse false }

namespace_celltype_name
        : namespace_identifier
		{ result = Cell.new(val[0]) }

cell_name
        : IDENTIFIER
		{ result = Cell.set_name(val[0].val) }

join_list
        :   # 空行  061007
        | join_list specified_join
        | join_list reverse_join

specified_join
        :  spec_L join_specifier_list spec_R join
		{ val[3].set_specifier_list( val[1] )  }
	|  join
		{ val[0].set_specifier_list( [] ) }

join_specifier_list
        : join_specifier_list ',' join_specifier
		{ result << val[2] }
        | join_specifier
		{ result = [val[0]] }

join_specifier
        : THROUGH '(' plugin_name ',' plugin_arg ')'
		{ result = [ :THROUGH, val[2], val[4] ] }

plugin_name
        : IDENTIFIER   { result = val[0] }

plugin_arg
        : string_literal_list

join
#        : cae_name                     '=' expression ';'
#		{
#			result = Join.new( val[0].val, nil, val[2] )
#			Cell.new_join( result, true )
#		}
#        | cae_name '[' ']'             '=' expression ';'
        : cae_name '[' ']'             '=' expression ';'
		{
			result = Join.new( val[0].val,  -1, val[4] )
			Cell.new_join( result, true )
		 }
        | cae_name '[' array_index ']' '=' expression ';'
		{
			result = Join.new( val[0].val, val[2], val[5] )
			Cell.new_join( result, true )
		}
        | cae_name '=' initializer ';'     # 初期化子： '{', '}' も可
		{
			result = Join.new( val[0].val, nil, val[2] )
			Cell.new_join( result, true )
		}
        | cae_name '=' COMPOSITE '.' IDENTIFIER ';'
		{
			result = Join.new( val[0].val, nil, [ :COMPOSITE, val[4] ] )
			Cell.new_join( result, true )
		}

cae_name #  cae: callport, attribute, entryport
        : IDENTIFIER

reverse_join
        # non-array <= non-array
        : cae_name '<=' namespace_identifier '.' IDENTIFIER ';'
		{
			rj = ReverseJoin.new( val[0].val, nil, val[2], val[4].val )
			Cell.new_reverse_join( rj )
		}
        # non-array <= array
        | cae_name '<=' namespace_identifier '.' IDENTIFIER '[' expression ']' ';'
		{
			rj = ReverseJoin.new( val[0].val, nil, val[2], val[4].val, val[6] )
			Cell.new_reverse_join( rj )
		}
        # array <= non-array
        | cae_name '[' array_index ']' '<=' namespace_identifier '.' IDENTIFIER ';'
		{
			rj = ReverseJoin.new( val[0].val, val[2], val[5], val[7].val )
			Cell.new_reverse_join( rj )
		}
        # array <= array
        | cae_name '[' array_index ']' '<=' namespace_identifier '.' IDENTIFIER '[' expression ']' ';'
		{
			rj = ReverseJoin.new( val[0].val, val[2], val[5], val[7].val, val[9] )
			Cell.new_reverse_join( rj )
		}


array_index
        : constant_expression

#複合種
composite_celltype
        : COMPOSITE composite_celltype_name '{' composite_celltype_statement_list '}' ';'
		{
			CompositeCelltype.end_of_parse
			result = val[1]
		}

composite_celltype_name
        : IDENTIFIER
		{ result = CompositeCelltype.new(val[0].val) }

composite_celltype_statement_list
        : specified_composite_celltype_statement
        | composite_celltype_statement_list specified_composite_celltype_statement

specified_composite_celltype_statement
        : composite_celltype_statement
		{
			if val[0].kind_of?( Port ) then
				CompositeCelltype.new_port( val[0] )   # 遅延して登録
			end
		}
        | spec_L composite_celltype_statement_specifier_list spec_R composite_celltype_statement
		{
			if val[3].kind_of?( Port ) then
				# port 以外 val[3] に有効な値が入っていないので、以下のメソッドを適用できない
				# 現状 port, cell 以外は指定子を受け付けない
				# （しかし将来他の文も指定子を受け付ける可能性があるので、この位置に記述する）
				val[3].set_specifier( Generator.get_statement_specifier )
				CompositeCelltype.new_port( val[3] )   # 遅延して登録 (set_specifier 後)
			elsif val[3].kind_of?( Cell ) then
				# Cell.end_of_parse にて設定
			else
              Generator.get_statement_specifier   # クリア
              Generator.error( "G1013 unexpected specifier"  )
			end
		}

composite_celltype_statement
        : composite_port
        | composite_attribute
        | internal_cell
        | export_join
#        | error       # エラー回復ポイント  (#513 無限ループに陥るケースがあるので、ここでのエラー回復は取りやめ)

composite_celltype_statement_specifier_list
        : composite_celltype_statement_specifier
		{
			Generator.add_statement_specifier val[0]
			result = [ val[0] ]
		}
        | composite_celltype_statement_specifier_list ',' composite_celltype_statement_specifier
		{
			Generator.add_statement_specifier val[2]
			result = val[0] << val[2]
		}

composite_celltype_statement_specifier
        : ALLOCATOR '(' alloc_list2 ')'		{ result = [ :ALLOCATOR, val[2] ] }
        | OMIT	{ result = [ :OMIT ] }
        | OPTIONAL	{ result = [ :OPTIONAL ] }
        | REF_DESC	{ result = [ :REF_DESC ] }
        | DYNAMIC	{ result = [ :DYNAMIC ] }

composite_port
        : port
		{
			# CompositeCelltype.new_port( val[0] )
			result = val[0]
		}

#属性
composite_attribute
        : ATTRIBUTE '{' composite_attribute_declaration_list '}' ';'
		{ result = nil }

composite_attribute_declaration_list
        : attribute_declaration
		{ CompositeCelltype.new_attribute( val[0] ) }
        | composite_attribute_declaration_list attribute_declaration
		{ CompositeCelltype.new_attribute( val[1] ) }

internal_cell
        : CELL internal_namespace_celltype_name
             internal_cell_name '{'
		{ Cell.new_def }
             internal_join_list '}' ';'
		{ result = Cell.end_of_parse true }
        | CELL internal_namespace_celltype_name	internal_cell_name ';'
		{ result = Cell.end_of_parse false }


internal_namespace_celltype_name
        : namespace_identifier
		{ Cell.new(val[0],true) }

internal_cell_name
        : IDENTIFIER
		{ Cell.set_name(val[0].val) }


internal_join_list
        :   # 空行  061007
        | internal_join_list specified_join
        | internal_join_list external_join
        | internal_join_list reverse_join

external_join  # cell 内に記述する呼び口の外部結合
        : internal_cell_elem_name '=>' COMPOSITE '.' export_name ';'
		{	Cell.external_join( val[0].val, val[4].val, true )	}
        | internal_cell_elem_name '=>' export_name ';'
		{	Cell.external_join( val[0].val, val[2].val, false )	}
        # 以前の文法では、呼び口側も cell の外に記述していた
        # その時の実装を

export_join    # cell 外に記述する受け口の外部結合
        : export_name '=>' internal_ref_cell_name '.' internal_cell_elem_name ';'
		{
			CompositeCelltype.new_join( val[0].val,
						val[2].val, val[4].val, :ENTRY )
		}
        | COMPOSITE '.' export_name '=>' internal_ref_cell_name '.' internal_cell_elem_name ';'
		{
			CompositeCelltype.new_join( val[2].val,
						val[4].val, val[6].val, :ENTRY )
		}

export_name
        : IDENTIFIER

internal_ref_cell_name
        : IDENTIFIER

internal_cell_elem_name
        : IDENTIFIER

# リージョン
region
        : spec_L region_specifier_list spec_R REGION region_name '{'  region_statement '}' ';'
		{ Region.end_of_parse }
        |                  REGION region_name '{'  region_statement '}' ';'
		{ Region.end_of_parse }


region_specifier_list
        : region_specifier
        | region_specifier_list ',' region_specifier

region_specifier
        : IN_THROUGH '(' plugin_name ',' plugin_arg ')'
		{ Region.new_in_through( val[2].val, val[4].val ) }
        | IN_THROUGH '(' ')'   # in 許可
		{ Region.new_in_through }
        | OUT_THROUGH '(' plugin_name ',' plugin_arg ')'
		{ Region.new_out_through( val[2].val, val[4].val ) }
        | OUT_THROUGH '(' ')'  # out 許可
		{ Region.new_out_through() }
        | TO_THROUGH '(' namespace_region_name ',' plugin_name ',' plugin_arg ')'
		{ Region.new_to_through( val[2], val[4].val, val[6].val ) }
        | TO_THROUGH '('namespace_region_name ')'  # to 許可
		{ Region.new_to_through( val[2], nil, nil ) }
        | NODE
		{ Region.set_type( :NODE ) }
        | LINKUNIT
		{ Region.set_type( :LINKUNIT ) }
        | DOMAIN '(' IDENTIFIER ',' STRING_LITERAL ')'
		{ Region.set_domain( val[2].val, val[4] ) }
        | CLASS '(' IDENTIFIER ')'
		{ Region.set_type( :CLASS, val[2].val ) }

region_name
        : IDENTIFIER
		{ result = Region.new( val[0].val ) }

region_statement
        :
        | region_statement region_cell
        | region_statement region

region_cell
        : cell
        | spec_L statement_specifier_list spec_R cell
		{
			obj = val[3]
			if obj.kind_of?( Cell ) then
			else
              Generator.get_statement_specifier   # クリア
              Generator.error( "G9999 unexpected specifier"  )
			end
		}
#        | spec_L region_cell_specifier_list spec_R cell

# region_cell_specifier_list
#         : region_cell_specifier
# 		{ Generator.add_statement_specifier val[0] }
#         | region_cell_specifier_list region_cell_specifier
# 		{ Generator.add_statement_specifier val[2] }

# region_cell_specifier
#         : ALLOCATOR '(' alloc_list ')'
# 		{ result = [ :ALLOCATOR, val[2] ] }


namespace_region_name
         : :IDENTIFIER
		{ result = [ val[0].val ] }  # mikan 配列である必要はない
#        : namespace_identifier

# 指定子の括弧 (in, out などのキーワード切り替えのため分離)
spec_L
        : '['  { set_in_specifier }
spec_R
        : ']'  { unset_in_specifier }

# location information
location_information
        :  __LOCATION_INFORMATION__ '{'  cell_location_join_location_list '}'

cell_location_join_location_list
        : cell_location_join_location_list cell_location
        | cell_location_join_location_list join_location
        |

cell_location
        : __CELL__ namespace_identifier '(' constant_expression ',' constant_expression ','  constant_expression ','  constant_expression ')' '{' port_location_list '}'
		{
			TECSGEN::Cell_location.new( val[1], val[3], val[5], val[7], val[9], val[12] )
		}

port_location_list
        : port_location_list port_location
		{
			result = val[0] << val[1]
		}
        |
		{ result = [] }

port_location
        : IDENTIFIER '(' IDENTIFIER ',' constant_expression ')'
		{ result = [ val[0], val[2], val[3] ] }

join_location    # mikan port array
        : __JOIN__ '(' namespace_identifier '.' IDENTIFIER '=>' namespace_identifier '.' IDENTIFIER ')'  '{' bar_list '}'
		{
            TECSGEN::Join_location.new( val[2], val[4], val[6], val[8], val[11] )
		}

bar_list
        : bar_list IDENTIFIER '(' constant_expression ')'
          {
            result = val[0] << [ val[1], val[3] ]
          }
        | { result = [] }


#  JSON object
tool_info        : TOOL_INFO '(' JSON_string ')' JSON_object { TOOL_INFO.new( val[2].to_sym, val[4] ) }
JSON_object      : '{' JSON_property_list   '}'              {  result = val[1] }
JSON_property_list : JSON_string ':' JSON_value              { result = { val[0].to_sym => val[2] } }
                 | JSON_property_list ',' JSON_string ':' JSON_value
                                                             { val[0][ val[2].to_sym ] = val[4] }
JSON_value       : JSON_string | JSON_number | JSON_object | JSON_array
                 | TRUE { result=val[0].val } | FALSE  { result=val[0].val } # JSON_NULL # null not suppoted
JSON_array       : '[' JSON_array_list ']'                   { result = val[1]  }
                 | '['  ']'                                  { result = []  }
JSON_array_list  : JSON_value                                { result = [ val[0] ] }
                 | JSON_array_list ',' JSON_value            { val[0] << val[2] }
JSON_string      : STRING_LITERAL                            { result = val[0].val.gsub!( /\"(.*)\"/, "\\1" ) }
JSON_number      : INTEGER_CONSTANT                          { result = val[0].val.to_i }
                 | FLOATING_CONSTANT                         { result = val[0].val.to_f }
                 | '-' INTEGER_CONSTANT                      { result = - val[0].val.to_i }
                 | '-' FLOATING_CONSTANT                     { result = - val[0].val.to_f }
                 | '+' INTEGER_CONSTANT                      { result = val[0].val.to_i }
                 | '+' FLOATING_CONSTANT                     { result = val[0].val.to_f }


end

---- inner

  RESERVED = {
    # keyword
    'namespace' => :NAMESPACE,
    'signature' => :SIGNATURE,
    'celltype' => :CELLTYPE,
    'cell' => :CELL,
    'attr' => :ATTRIBUTE,
    'var' => :VAR,
    'call' => :CALL,
    'entry' => :ENTRY,
    'composite' => :COMPOSITE,
    'require' => :REQUIRE,
    'factory' => :FACTORY,
    'FACTORY' => :CTFACTORY,
    'typedef' => :TYPEDEF,
    'struct' => :STRUCT,
    'region' => :REGION,
    'import' => :IMPORT,
    'import_C' => :IMPORT_C,
    'generate' => :GENERATE,
    '__tool_info__' => :TOOL_INFO,

    # types
    'void'    => :VOID,

    'volatile'=> :VOLATILE,
    'const'   => :CONST,

    'signed'  => :SIGNED,
    'unsigned'=> :UNSIGNED,

    'int8_t'    => :INT8_T,
    'int16_t'   => :INT16_T,
    'int32_t'   => :INT32_T,
    'int64_t'   => :INT64_T,
    'int128_t'  => :INT128_T,
    'uint8_t'   => :UINT8_T,
    'uint16_t'  => :UINT16_T,
    'uint32_t'  => :UINT32_T,
    'uint64_t'  => :UINT64_T,
    'uint128_t' => :UINT128_T,

    'float32_t'   => :FLOAT32_T,
    'double64_t'  => :DOUBLE64_T,
    'bool_t'      => :BOOL_T,
    'char_t'  => :CHAR_T,
    'schar_t'  => :SCHAR_T,
    'uchar_t'  => :UCHAR_T,

    # unrecommened types
    'int'     => :INT,
#   'intptr'  => :INTPTR,
    'short'   => :SHORT,
    'long'    => :LONG,

    # obsolete types
    'char'    => :CHAR,
#    'int8'    => :INT8,
#    'int16'   => :INT16,
#    'int32'   => :INT32,
#    'int64'   => :INT64,
#    'int128'  => :INT128,
#    'float'   => :FLOAT,
#    'double'  => :DOUBLE,
#    'bool'    => :BOOL,

    'enum'    => :ENUM,
    'enum8'   => :ENUM8,
    'enum16'  => :ENUM16,
    'enum32'  => :ENUM32,
    'enum64'  => :ENUM64,

    'true'    => :TRUE,
    'false'   => :FALSE,

    'C_EXP'   => :C_EXP,

    'Descriptor'   => :DESCRIPTOR,

    # location information for TECSCDE
    '__location_information__' => :__LOCATION_INFORMATION__,
    '__cell__' => :__CELL__,
    '__join__' => :__JOIN__,
  }

  # 指定子 '[]' 内でのみ使用できるキーワード
  RESERVED2 = {
    # specifier
    'id' => :ID,

    # signature
    'callback' => :CALLBACK,
    'context' => :CONTEXT,
    'deviate' => :DEVIATE,

    # celltype
    'singleton' => :SINGLETON,
    'idx_is_id' => :IDX_IS_ID,
    'active' => :ACTIVE,

    # port (entry)
    'inline' => :INLINE,
    'ref_desc' => :REF_DESC,   # call も可

    # port (call)
    'optional' => :OPTIONAL,
    'dynamic' => :DYNAMIC,

    # port (call), attribute
    'omit' => :OMIT,

    # attribute
    'choice' => :CHOICE,

    # cell
    'allocator' => :ALLOCATOR,
    'prototype' => :PROTOTYPE,
    'restrict'  => :RESTRICT,

    # FuncType
    'oneway' => :ONEWAY,

    # parameter (basic)
    'in' => :IN,
    'out' => :OUT,
    'inout' => :INOUT,
    'send' => :SEND,
    'receive' => :RECEIVE,

    # parameter 
    'size_is' => :SIZE_IS,
    'count_is' => :COUNT_IS,
    'string' => :STRING,
    'nullable' => :NULLABLE,

    'through' => :THROUGH,
    'in_through' => :IN_THROUGH,
    'out_through' => :OUT_THROUGH,
    'to_through' => :TO_THROUGH,

    'node' => :NODE,
    'linkunit' => :LINKUNIT ,
    'domain' => :DOMAIN,
    'class' => :CLASS,
  }

  # 再帰的なパーサのためのスタック
  @@generator_nest = -1
  @@generator_stack = []
  @@current_locale = []

  # import_C 中である
  @@import_C = false

  # すべての構文解析が完了した
  @@b_end_all_parse = false

  # tag なし struct
  @@no_struct_tag_num = 0

  def self.parse( file_name, plugin = nil, b_reuse = false )
    # パーサインスタンスを生成(別パーサで読み込む)
    parser = Generator.new

    # plugin から import されている場合の plugin 設定
    parser.set_plugin plugin

    # reuse フラグを設定
    parser.set_reuse b_reuse

    # cdl をパース
    parser.parse( [file_name] )

    # 終期化　パーサスタックを戻す
    parser.finalize
  end

  def finalize

    # mikan Namespace.pop
    Namespace.pop
    Signature.pop
    Celltype.pop
    Cell.pop
    CompositeCelltype.pop
  end

  def set_plugin( plugin )
    @plugin = plugin
  end

  def self.get_plugin
    if @@generator_stack[@@generator_nest] then
      # tecsgen 引数の cdl が import される場合は nil
      return @@generator_stack[@@generator_nest].get_plugin
    else
      return nil
    end
  end

  def get_plugin
    @plugin
  end

  def set_reuse( b_reuse )
    @b_reuse = b_reuse
  end

  def self.is_reuse?
    if @@generator_stack[@@generator_nest] then
      # tecsgen 引数の cdl が import される場合は nil
      return @@generator_stack[@@generator_nest].is_reuse?
    else
      return false
    end
  end

  def is_reuse?
    @b_reuse
  end

  def parse(files)

    # mikan Namespace.push
    Namespace.push
    Signature.push
    Celltype.push
    Cell.push
    CompositeCelltype.push

    @@generator_nest += 1
    @@generator_stack[@@generator_nest] = self
    @in_specifier = false

    begin

      @q = []
      b_in_comment = false
      b_in_string = false

      # euc のコメントを utf8 として扱うと、コメントの終わりを誤る問題の対策
      TECS_LANG::set_kcode_binary

      files.each {|file|
        lineno = 1
        begin
          string = ""
#2.0          IO.foreach(file) {|line|
          TECSIO.foreach(file) {|line|
            col = 1
#            line.rstrip!         改行含む文字列を扱うようになったので、ここで空白を取り除けなくなった

            until line.empty?

              if b_in_comment
                case line
                  # コメント終了
                when /\A\*\//
                  b_in_comment = false
                when /\A./
                  ;
                when /\s+/     # line.rstrip! を止めたため \n などの空白文字とまっちするルールが必要になった
                  ;
                end
              elsif b_in_string
                if line =~ /\A(?:[^"\\]|\\.)*"/
                  string = "#{string}#{$&}"
                  @q <<  [:STRING_LITERAL, Token.new(string, file, lineno, col)]
                  b_in_string = false
                elsif line =~ /\A.*\\\n/     # 改行 \n は '.' にマッチしない
                  string += $&
                elsif line =~ /\A.*\n/     # 改行 \n は '.' にマッチしない
                  string += line
                  # この位置では error メソッドは使えない (token 読出し前)
                  puts "error: #{file} line #{lineno}: string literal has newline without escape"
                  @@n_error += 1
                end
              else
                case line
                # 空白、プリプロセスディレクティブ
                when /\A\s+/
                  ;
                # 識別子
                when /\A[a-zA-Z_]\w*/
                  word = $&
                  @q << [RESERVED[word] || :IDENTIFIER, Token.new(word.intern, file, lineno, col)]
                # 16 進数定数
                when /\A0x[0-9A-Fa-f]+/
                  @q << [:HEX_CONSTANT, Token.new($&, file, lineno, col)]
                # 8 進数定数
                when /\A0[0-7]+/
                  @q << [:OCTAL_CONSTANT, Token.new($&, file, lineno, col)]
                # 浮動小数定数
                when /\A[0-9]+\.([0-9]*)?([Ee][+-]?[0-9]+)?/
                  @q << [:FLOATING_CONSTANT, Token.new($&, file, lineno, col)]
                # 整数定数
                when /\A\d+/
                  @q << [:INTEGER_CONSTANT, Token.new($&.to_i, file, lineno, col)]
                # 文字定数
                when /\A'(?:[^'\\]|\\.)'/
                  @q << [:CHARACTER_LITERAL, Token.new($&, file, lineno, col)]
                # 文字列
#                "#include  #include #include \"../systask/logtask.cfg\"       最後の " 忘れ)で無限ループ
#                when /\A"(?:[^"\\]+|\\.)*"/
                when /\A"(?:[^"\\]|\\.)*"/   # これはうまく行くようだ
                  @q << [:STRING_LITERAL, Token.new($&, file, lineno, col)]
                # 文字列 (改行あり)
                when /\A"(?:[^"\\]|\\.)*\\\n$/
                  string = $&
                  b_in_string = true
                # 文字列 (改行あり, escape なし)
                when /\A("(?:[^"\\]|\e\.)*)\n$/
                  string = $1 + "\\\n"
                  b_in_string = true
                  # この位置では error メソッドは使えない (token 読出し前) # mikan cdl_error ではない
                  puts "error: #{file} line #{lineno}: string literal has newline without escape"
                  @@n_error += 1
                # 山括弧で囲まれた文字列
                when /\A<[0-9A-Za-z_\. \/]+>/   # AB: angle bracke
                  @q << [:AB_STRING_LITERAL, Token.new($&, file, lineno, col)]
                # 行コメント
                when /\A\/\/.*$/
                  # 読み飛ばすだけ
                # コメント開始
                when /\A\/\*/
                  b_in_comment = true
                # '>>', '<<' など
                when /\A>>/, /\A<</, /\A==/, /\A!=/, /\A\&\&/, /\A\|\|/
                  @q << [$&, Token.new($&, file, lineno, col)]
                when /\A::/, /\A=>/, /\A<=/, /\A>=/
                  @q << [$&, Token.new($&, file, lineno, col)]
                # '(', ')' など一文字の記号、または未知の記号
                when /\A./
                  @q << [$&, Token.new($&, file, lineno, col)]
                else
                  raise
                end
              end

              line = $'
              col += $&.length
            end

            lineno += 1
          }

        rescue => evar
          Generator.error( "G1014 while open or reading \'$1\'" , file )
          if $debug then
            p puts( evar )
            pp $@
          end
        end
      }

      # 終了の印
      @q << nil

      @yydebug = true
      do_parse

    ensure
      @@generator_nest -= 1
      TECS_LANG::reset_kcode
    end

  end

  def next_token
    token = @q.shift

    if token then
      @@current_locale[@@generator_nest] = token[1].locale

      if token[0] == :IDENTIFIER then
        # TYPE_NAME トークンへ置換え
        if Namespace.is_typename?( token[1].val ) then
          token[0] = :TYPE_NAME
        elsif @in_specifier && RESERVED2[ token[1].val.to_s ] then
          # 指定子キーワード（ '[', ']' 内でのみ有効)
          token[0] = RESERVED2[ token[1].val.to_s ]
        end
      end

      if $debug then     # 070107 token 無効時ここを通さないようした (through 対応 -d の時に例外発生) 
        locale = @@current_locale[@@generator_nest]
        if token then
          print( "#{locale[0]}: line #{locale[1]} : #{token[0]} '#{token[1].val}'\n" )
        else
          print( "#{locale[0]}: line #{locale[1]} : EOF\n" )
        end
      end
    else
      token = [ false, false ]
    end

    token
  end

  def on_error(t, v, vstack)
    # p t, token_to_str(t), vstack
    if token_to_str(t) == "$end" then
      Generator.error( "G1015 Unexpected EOF"  )
    else
      Generator.error( "G1016 syntax error near \'$1\'" , v.val )
    end
  end

  def self.current_locale
    @@current_locale[ @@generator_nest ]
  end

  @@n_error = 0
  @@n_warning = 0
  @@n_info = 0

  # このメソッドは構文解析、意味解析からのみ呼出し可（コード生成でエラー発生は不適切）
  def self.error( msg, *arg )
    locale = nil
    self.error2( locale, msg, *arg )
  end

  def self.error2( locale, msg, *arg )
    @@n_error += 1

    msg = TECSMsg.get_error_message( msg )
    # $1, $2, ... を arg で置換
    count = 1
    arg.each{ |a|
      str = TECSIO.str_code_convert( msg, a.to_s )
      msg.sub!( /\$#{count}/, str )
      count += 1
    }

    # import_C の中でのエラー？
    if @@import_C then
      C_parser.error( msg )
    else

      # Node の記憶する 位置 (locale) を使用した場合、変更以前に比べ、
      # 問題発生箇所と異なる位置にエラーが出るため、構文解析中のエラー
      # は、解析中の位置を出力する．(new_XXX で owner が子要素のチェッ
      # クをすると owner の行番号が出てしまう点で、ずれが生じている)

      if @@b_end_all_parse == false || locale == nil then
        locale = @@current_locale[ @@generator_nest ]
      end
      if locale then
        Console.puts "error: #{locale[0]}: line #{locale[1]} #{msg}"
      else
        Console.puts "error: #{msg}"
      end
    end
  end

  # このメソッドは構文解析、意味解析からのみ呼出し可（コード生成でウォーニング発生は不適切）
  def self.warning( msg, *arg )
    locale = nil
    self.warning2( locale, msg, *arg )
  end

  def self.warning2( locale, msg, *arg )
    @@n_warning += 1

    msg = TECSMsg.get_warning_message( msg )
    # $1, $2, ... を arg で置換
    count = 1
    arg.each{ |a|
      str = TECSIO.str_code_convert( msg, a.to_s )
      msg.sub!( /\$#{count}/, str )
      count += 1
    }

    # import_C の中でのウォーニング？
    if @@import_C then
      C_parser.warning( msg )
    else
      if @@b_end_all_parse == false || locale == nil then
        locale = @@current_locale[ @@generator_nest ]
      end
      if locale then
        Console.puts "warning: #{locale[0]}: line #{locale[1]} #{msg}"
      else
        Console.puts "warning: #{msg}"
      end
    end
  end

  # このメソッドは構文解析、意味解析からのみ呼出し可
  def self.info( msg, *arg )
    locale = nil
    self.info2( locale, msg, *arg )
  end

  def self.info2( locale, msg, *arg )
    @@n_info += 1

    msg = TECSMsg.get_info_message( msg )
    # $1, $2, ... を arg で置換
    count = 1
    arg.each{ |a|
      str = TECSIO.str_code_convert( msg, a.to_s )
      msg.sub!( /\$#{count}/, str )
      count += 1
    }

    # import_C の中でのウォーニング？
    if @@import_C then
      C_parser.info( msg )
    else
      if @@b_end_all_parse == false || locale == nil then
        locale = @@current_locale[ @@generator_nest ]
      end
      if locale then
        Console.puts "info: #{locale[0]}: line #{locale[1]} #{msg}"
      else
        Console.puts "info: #{msg}"
      end
    end
  end

  def self.get_n_error
    @@n_error
  end

  def self.get_n_warning
    @@n_warning
  end

  def self.get_n_info
    @@n_info
  end

  def self.get_nest
    @@generator_nest
  end

  def self.parsing_C?
    @@import_C
  end

  #===  '[' specifier 始め
  def set_in_specifier
    # p "set_in_specifier"
    @in_specifier = true
  end

  #=== ']' specifier 終わり
  def unset_in_specifier
    # p "unset_in_specifier"
    @in_specifier = false
  end

  # statement_specifier は構文解釈途中で参照したいため
  @@statement_specifier_stack = []
  def self.add_statement_specifier( ss )
    if( @@statement_specifier_stack[ @@generator_nest ] == nil )then
      @@statement_specifier_stack[ @@generator_nest ] = [ ss ]
    else
      @@statement_specifier_stack[ @@generator_nest ] << ss
    end
  end


  def self.get_statement_specifier
    spec_list = @@statement_specifier_stack[ @@generator_nest ]
    @@statement_specifier_stack[ @@generator_nest ] = nil
    return spec_list
  end

  #=== すべての構文解析が完了したことを報告
  def self.end_all_parse
    @@b_end_all_parse = true
  end

---- footer


# ファイル => INCLUDE("header")の配列
Include = Hash.new {|hash, key| hash[key] = []}


class Token

  attr_accessor :val, :file, :lineno, :col

  def initialize(val, file, lineno, col)
    @val = val
    @file = file
    @lineno = lineno
    @col = col

  end

  def to_s
    @val.to_s
  end

  def to_sym
    @val.to_sym
  end

  def get_name
    @val
  end

  def locale
    [@file, @lineno, @col]
  end

  def eql?(other)
    if other.is_a? Symbol
      @val == other
    elsif other.is_a? Token
      @val == other.val
    elsif other.is_a? String
      @val.to_s == other
    else
      raise ArgumentError
    end
  end

  alias == eql?

  def show_tree( indent )
    indent.times { print "  " }
    print "#{@val}\n"
  end

end

#= TECSIO
#  Ruby2.0(1.9) 対応に伴い導入したクラス
#  SJIS 以外では、ASCII-8BIT として入力する
class TECSIO
  def self.foreach(file) # ブロック引数 { |line| }
    pr = Proc.new   # このメソッドのブロック引数を pr に代入
    if $b_no_kcode then
	  msg = "E".encode $Ruby19_File_Encode
      if( $Ruby19_File_Encode == "Shift_JIS" )

        # Shift JIS は、いったん Windows-31J として読み込ませ、Shift_JIS に変換させる．
        # コメント等に含まれる SJIS に不適切な文字コードは '?' または REPLACEMENT CHARACTER に変換される．
        # EUC や UTF-8 で記述された CDL が混在していても、Ruby 例外が発生することなく処理を進めることができる．
        # 文字コード指定が SJIS であって、文字列リテラルの中に、文字コードがSJIS 以外の非 ASCII が含まれている場合、
        # Ruby 1.8 の tecsgen では文字コード指定に影響なく処理されたものが、Ruby 1.9 以降では '?' に置き換わる可能性がある．

        mode = "r:Windows-31J"
      else
        mode = "r:#{$Ruby19_File_Encode}"
      end
      # mode = "r"
    else
	  msg = "E"
      mode = "r"
    end

    f = File.open( file, mode )
    begin
      f.each{ |line|
        # dbgPrint line
        line = str_code_convert( msg, line )
        pr.call( line )
      }
    ensure
      f.close
    end
  end

  #=== 文字コードが相違する場合一致させる
  # msg と str の文字コードが相違する場合、str を msg の文字コードに変換する
  # 変換不可の文字コードは '?' (utf-8 の場合 U+FFFD (REPLACEMENT CHARACTER )) に変換
  #
  # このメソッドは、エラーメッセージ出力でも使用されていることに注意．
  #
  #msg_enc::Encode | String
  def self.str_code_convert( msg, str )
    if $b_no_kcode == false then
      return str                          # Ruby V1.8 まで
    end
    if msg.encoding != str.encoding then
      option = { :invalid => :replace, :undef => :replace }   # 例外を発生させず、'?' に変換する(utf-8 は 0xfffd)
      # return str.encode( msg.encoding, option )
      str = str.encode( "utf-8", option )
      return str.encode( msg.encoding, option )
    else
      return str
    end
  end
end
