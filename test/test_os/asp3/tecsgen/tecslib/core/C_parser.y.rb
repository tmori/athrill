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
#   $Id: C_parser.y.rb 2633 2017-04-02 06:02:05Z okuma-top $
#++

class C_parser
rule
# トップレベルの構文要素は C_parser
all: C_parser

# Expr
##########################  式  ##########################
# K&Rの文法(プログラミング言語C 第2版 付録)と一部異なる
# argument_expression_list(関数引数), assignment_expression(代入)がない
# 式の result は、すべて配列で第一要素が識別シンボル、第二要素以下が引数

primary_expression
        : namespace_identifier
		{ result = [ :IDENTIFIER, val[0] ] }     #1ok
#        : IDENTIFIER	# mikan namespace への対応
#		{ result = [ :IDENTIFIER, val[0] ] }
#        | TRUE
#		{ result = [ :BOOL_CONSTANT, true ] }
#        | FALSE
#		{ result = [ :BOOL_CONSTANT, false ] }
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
		{ result = [val[0]] }
        | string_literal_list STRING_LITERAL
		{ result << val[1] }

# 関数呼び出しと後置インクリメント、デクリメント演算子がない
postfix_expression
        : primary_expression
        | primary_expression '(' argument_list ')'
        | primary_expression type_qualifier '(' argument_list ')'    # intended __asm volatile ( "   MNEMONIC  OPERAND" );
        | postfix_expression '[' expression ']'
		{ result = [ :OP_SUBSC, val[0], val[2] ] }
        | postfix_expression '.' IDENTIFIER
		{ result = [ :OP_DOT, val[0], val[2] ] }
        | postfix_expression '->' IDENTIFIER
		{ result = [ :OP_REF, val[0], val[2] ] }
        | postfix_expression '++'	{ result = val[0] }   # ++, -- は無視する
        | postfix_expression '--'	{ result = val[0] }

argument_list
        :
        | expression
        | argument_list ',' expression


# 前置インクリメント、デクリメント演算子がない
unary_expression
        : postfix_expression
        | unary_operator cast_expression
		{ result = [ val[0], val[1] ] }
        | SIZEOF unary_expression
		{ result = [ :OP_SIZEOF_EXPR, val[1] ] }
        | SIZEOF '(' type_name ')'
		{ result = [ :OP_SIZEOF_TYPE, val[1] ] }
		| '++' unary_expression			{ result = val[1] }   # ++, -- は無視する
		| '--' unary_expression			{ result = val[1] }

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
#			res = result.eval_const( nil )

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
        : declaration_specifiers init_declarator_list ';'
#        : type_specifier_qualifier_list init_declarator_list ';'

# declaration_specifiersは関数のパラメータで使われるが、
# type_specifier_qualifier_listで十分かもしれない

declaration_specifiers
        : storage_class
		{
			result = CIntType.new( -3 )    # storage class は無視
		}
        | type_specifier
        | type_qualifier
		{
			result = CIntType.new( -3 )
			result.set_qualifier val[0]
		}
        | storage_class declaration_specifiers
		{
			result = val[1]                # storage class は無視
		}
        | type_specifier declaration_specifiers
		{
			result = val[1].merge val[0]
		}
        | type_qualifier declaration_specifiers
		{
			val[1].set_qualifier val[0]
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

type_specifier
        : VOID	{ set_no_type_name true; result = CVoidType.new }
        | FLOAT	{ set_no_type_name true; result = CFloatType.new(-32) }
        | DOUBLE	{ set_no_type_name true; result = CFloatType.new(-64) }
        | BOOL	{ set_no_type_name true; result = CBoolType.new }
        | struct_specifier	{ set_no_type_name true; result = val[0] } # set_no_type_name true は struct_tag でも呼ばれる
        | union_specifier	{ set_no_type_name true; result = CVoidType.new }  # void が宣言されたとする
        | enum_specifier	{ set_no_type_name true; result = CVoidType.new }  # void が宣言されたとする
        | TYPE_NAME	{ set_no_type_name true; result = CDefinedType.new( val[0].val ) }

        | CHAR	{ set_no_type_name true; result = CIntType.new(-11 ) }
        | SHORT	{ set_no_type_name true; result = CIntType.new( -2 ) }
        | INT		{ set_no_type_name true; result = CIntType.new( -3 ) }
        | LONG	{ set_no_type_name true; result = CIntType.new( -4 ) }
        | SIGNED
		{
			set_no_type_name true
			result = CIntType.new( -3 )
			result.set_sign :SIGNED
		}
        | UNSIGNED
		{
			set_no_type_name true
			result = CIntType.new( -3 )
			result.set_sign :UNSIGNED
		}

# mikan K&Rのstruct_or_union_specifierに相当するが、unionは使えない, bit field にも対応しない
struct_specifier		# mikan
#        : STRUCT struct_tag '{'
        : struct_term struct_tag '{'
		{ StructType.set_define( true )  }
	   struct_declaration_list '}'
		{
			StructType.end_of_parse
			result = val[1]
		}
#        | STRUCT
        | struct_term
		{
			result = CStructType.new()
			StructType.set_define( true )
		}
	   '{' struct_declaration_list '}'
		{
			StructType.end_of_parse
			result = val[1]
		}
#        | STRUCT struct_tag   # mikan struct_tag は namespace 対応が必要
        | struct_term struct_tag   # mikan struct_tag は namespace 対応が必要
		{
			StructType.set_define( false )
			StructType.end_of_parse
			result = val[1]
		}

struct_term
        : STRUCT { set_no_type_name true }

struct_declaration_list
        : struct_declaration
        | struct_declaration_list struct_declaration

struct_tag:
        IDENTIFIER
		{
			result = CStructType.new( val[0].val )
			set_no_type_name true
		}

# ポインタ修飾子を追加
struct_declaration
        : declaration_specifiers struct_declarator_list ';'
#        :                                type_specifier_qualifier_list struct_declarator_list ';'
		{
			val[1].each { |i|	# i: Decl
				i.set_type( val[0] )
				i.set_kind( :MEMBER )
				i.check
				CStructType.new_member( i )
			}
			result = val[1]
		}
        | union_specifier ';'                       # 無名
        | struct_specifier ';'                       # 無名



# K&Rのspecifier_qualifier_listと同じ
# 名前がまぎらわしかったのでtype_を付けた
type_specifier_qualifier_list
        : type_specifier
        | type_specifier type_specifier_qualifier_list
		{
			result = val[1].merge val[0]
		}
        | type_qualifier
		{
			result = CIntType.new( -3 )
			result.set_qualifier val[0]
		}
        | type_qualifier type_specifier_qualifier_list
		{
			val[1].set_qualifier val[0]
                        result = val[1]
		}

struct_declarator_list
        : struct_declarator
		{ result = [ val[0] ] }
        | struct_declarator_list ',' struct_declarator
		{ result << val[2] }

# ビットフィールドは使えない
struct_declarator
        : declarator



union_specifier
#        : UNION union_tag '{' union_declaration_list '}'
#        | UNION '{' union_declaration_list '}'
#        | UNION union_tag   # mikan struct_tag は namespace 対応が必要
        : union_term union_tag '{' union_declaration_list '}'
        | union_term '{' union_declaration_list '}'
        | union_term union_tag   # mikan struct_tag は namespace 対応が必要

union_term
        : UNION { set_no_type_name true }

union_declaration_list
        : union_declaration
        | union_declaration_list union_declaration

union_tag:
	IDENTIFIER

union_declaration
        : declaration_specifiers union_declarator_list ';'
		| union_specifier ';'                       # 無名
		| struct_specifier ';'                      # 無名

union_declarator_list
        : union_declarator
        | union_declarator_list ',' union_declarator

# ビットフィールドは使えない
union_declarator
        : declarator



# enumの種類を追加
enum_specifier		# mikan
        : enum_type            '{' enumerator_list '}'
        | enum_type IDENTIFIER '{' enumerator_list '}'
        | enum_type IDENTIFIER

enum_type
        : ENUM	{ result = CEnumType.new( -1 ) }
#        | ENUM8	{ result = CEnumType.new( 8 ) }
#        | ENUM16	{ result = CEnumType.new( 16 ) }
#        | ENUM32	{ result = CEnumType.new( 32 ) }
#        | ENUM64	{ result = CEnumType.new( 64 ) }
#        | ENUM128	{ result = CEnumType.new( 128 ) }

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
        | pointer TYPE_NAME     # 関数ポインタの typedef が二重定義の場合
		{
			result = Decl.new( val[1].val )
			result.set_type( val[0] )
		}

direct_declarator		# mikan
        : IDENTIFIER
		{ result = Decl.new( val[0].val ) }
        | '(' declarator ')'
		{ result = val[1] }
        | direct_declarator '[' constant_expression ']'
		{
			val[0].set_type( CArrayType.new( val[2] ) )
			result = val[0]
		}
        | direct_declarator '[' ']'
		{
			val[0].set_type( CArrayType.new )
			result = val[0]
		}
        | direct_declarator '(' parameter_type_list ')'
		{
		# 	Generator.warning( "W6001 need 'void' for no parameter"  )
			val[0].set_type( CFuncType.new )
			result = val[0]
		}

#        | direct_declarator '(' identifier_list ')'  # これは何のために必要？ 060211
        | direct_declarator '(' ')'
		{
		# 	Generator.warning( "W6002 need 'void' for no parameter"  )
			val[0].set_type( CFuncType.new )
			result = val[0]
		}

pointer
        : '*'
		{ result = CPtrType.new }
        | '*' type_qualifier
		{
			result = CPtrType.new
			result.set_qualifier( val[1] )
		}
        | '*' pointer
		{
			val[1].set_type(CPtrType.new)
			result = val[1]
		}
        | '*' type_qualifier pointer
		{
			ptrtype = CPtrType.new
			ptrtype.set_qualifier( val[1] )
			val[2].set_type( ptrtype )
			result = val[2]
		}


parameter_type_list
        : parameter_list
        | parameter_list ',' '.' '.' '.'
		# mikan 可変長パラメータ,  ... の間のスペースが許される（手抜き）

parameter_list
        : parameter_declaration
#		{ result = ParamList.new( val[0] ) }
        | parameter_list ',' parameter_declaration
#		{
#			val[0].add_param( val[2] )
#			# result = val[0] 不要
#		}


# パラメータ修飾子を追加
parameter_declaration
        : declaration_specifiers declarator
#		{
#			decl = ParamDecl.new( val[1], val[0], [] )
#			val[1].set_kind( :PARAMETER )
#			result = decl
#		}

	# 以下はエラーとする
#        | declaration_specifiers	# 仮引数なしは、とりあえず扱わない 060210
#		{
#			unless val[0].kind_of?( VoidType ) then
#				Generator.error( "B1001 need parameter name"  )
#			end
#			result = nil
#		}
        | declaration_specifiers abstract_declarator	# 仮引数なし
        | declaration_specifiers						# 仮引数なし


#identifier_list       # 060211  不用になった
#        : IDENTIFIER
#        | identifier_list ',' IDENTIFIER

type_name
        : type_specifier_qualifier_list
        | type_specifier_qualifier_list abstract_declarator

abstract_declarator		# mikan
        : pointer
        | direct_abstract_declarator
        | pointer direct_abstract_declarator

direct_abstract_declarator
        : '(' abstract_declarator ')'
        | '[' ']'
        | '[' constant_expression ']'
        | direct_abstract_declarator '[' ']'
        | direct_abstract_declarator '[' constant_expression ']'
        | '(' ')'
		{
			Generator.warning( "W6003 need 'void' for no parameter"  )
		}
        | '(' parameter_type_list ')'
        | direct_abstract_declarator '(' ')'
		{
			Generator.warning( "W6004 need 'void' for no parameter"  )
		}
        | direct_abstract_declarator '(' parameter_type_list ')'

# assignment_expressionをconstant_expressionに変更
initializer			# mikan
        : constant_expression
		{ result = val[0] }
        | '{' initializer_list '}'
		{ result = val[1] }
        | '{' initializer_list ',' '}'
		{ result = val[1] }
	| C_EXP '(' STRING_LITERAL ')'
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
C_parser
        :
        | C_parser extension_statement

extension_statement
        : statement
        | EXTENSION statement

statement
        : typedef
        | func_def
        | enum_specifier ';'
        | struct_specifier ';'
        | declaration
        | ';'
        | error   # エラー回復ポイント

typedef
        : TYPEDEF type_specifier_qualifier_list declarator_list ';'
		{
			val[2].each{ |i|
			   i.set_kind( :TYPEDEF )
			}
			Typedef.new_decl_list( val[1], val[2] )
                        # val[1].show_tree 0
		}

declarator_list
        : declarator
		{ result = [ val[0] ] }
        | declarator_list ',' declarator
		{ result << val[2] }

func_def
        : declaration_specifiers declarator compoundstatement

infunc_statement_list
        :
        | infunc_statement_list infunc_statement

infunc_statement
        : declaration
        | ifstatement
        | whilestatement
        | dowhilestatement
        | forstatement
        | switchstatement
        | labelstatement
        | compoundstatement
        | gotostatement
        | expressionstatement
        | ';'

ifstatement
        : IF '(' expression ')' infunc_statement
        | IF '(' expression ')' infunc_statement ELSE infunc_statement

whilestatement
        : WHILE '(' expression ')' infunc_statement

dowhilestatement
        : DO infunc_statement WHILE '(' expression ')' ';'

forstatement
        : FOR '(' expression ';' expression ';' expression ')' infunc_statement

switchstatement
        : SWITCH '(' expression ')'  infunc_statment

labelstatement
        : IDENTIFIER ':' infunc_statement
        | CASE constant_expression ':' infunc_statement
        | DEFAULT ':' infunc_statement

compoundstatement
        : '{' infunc_statement_list '}'

gotostatement
        : GOTO IDENTIFIER ';'
        | CONTINUE ';'
        | BREAK ';'
        | RETURN expression ';'
        | RETURN ';'

expressionstatement
        : expression ';'
        | unary_expression assignment_operator expression ';'

assignment_operator
        : '='
        | '+='
        | '-='
        | '*='
        | '/='
        | '%='
        | '<<='
        | '>>='
        | '&='
        | '|='
        | '^='

storage_class
        : __INLINE__
        | INLINE
        | __INLINE
        | CINLINE
        | EXTERN
        | STATIC
        | AUTO
        | REGISTER

namespace_identifier
        : IDENTIFIER		{ result = NamespacePath.new( val[0].val, false ) }
        | '::' IDENTIFIER	{ result = NamespacePath.new( val[1].val, true ) }
        | namespace_identifier '::' IDENTIFIER
		{ result = val[0].append!( val[2].val ) }

end

---- inner

  RESERVED = {
    # keyword
    'typedef' => :TYPEDEF,
    'struct' => :STRUCT,
    'union' => :UNION,
    'sizeof' => :SIZEOF,
    'throw' => :THROW,

    # specifier
    # types
    'void'    => :VOID,
    'char'    => :CHAR,
    'short'   => :SHORT,

    'volatile'=> :VOLATILE,
    'const'   => :CONST,
    'extern'   => :EXTERN,

    'long'    => :LONG,
    'float'   => :FLOAT,
    'double'  => :DOUBLE,
    'signed'  => :SIGNED,
    'unsigned'=> :UNSIGNED,

    'int'     => :INT,
    'enum'    => :ENUM,

    'if'      => :IF,
    'else'    => :ELSE,
    'while'   => :WHILE,
    'do'      => :DO,
    'for'     => :FOR,
    'case'    => :CASE,
    'default' => :DEFAULT,
    'goto'    => :GOTO,
    'continue' => :CONTINUE,
    'break'   => :BREAK,
    'return'  => :RETURN,
    '__inline__'  => :__INLINE__,
    'inline'  => :INLINE,
    '__inline'  => :__INLINE,
    'Inline'  => :CINLINE,        # inline starting with Capital letter
    'static'  => :STATIC,
    'register' => :REGISTER,
    'auto'    => :AUTO,
    '__extension__'    => :EXTENSION,

  }

  @@generator_nest = -1
  @@generator_stack = []
  @@current_locale = []

  def finalize

    # mikan Namespace.pop
    Celltype.pop
    Cell.pop
    CompositeCelltype.pop
    Region.pop

  end

  def set_plugin( plugin )
    @plugin = plugin
  end

  def self.get_plugin
    @@generator_stack[@@generator_nest].get_plugin
  end

  def get_plugin
    @plugin
  end

  def parse(files)

    # mikan Namespace.push
    Celltype.push
    Cell.push
    CompositeCelltype.push
    Region.push

    @@generator_nest += 1
    @@generator_stack[@@generator_nest] = self
    @b_no_type_name = false

   begin

    @q = []
    comment = false

    # euc のコメントを utf8 として扱うと、コメントの終わりを誤る問題の対策
    TECS_LANG::set_kcode_binary

    # 800U, 0xffLL など (整数リテラルに共通の修飾子)
    integer_qualifier = "([Uu][Ll][Ll]|[Uu][Ll]|[Uu]|[Ll][Ll]|[Ll])?"

    files.each {|file|
      lineno = 1
     begin
#2.0       IO.foreach(file) {|line|
       TECSIO.foreach(file) {|line|
        col = 1
        line.rstrip!

        until line.empty?

          if comment
            case line
            # コメント終了
            when /\A\*\//
              comment = false
            when /\A./
              ;
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
            when /\A0x[0-9A-Fa-f]+#{integer_qualifier}/
              @q << [:HEX_CONSTANT, Token.new($&, file, lineno, col)]
            # 8 進数定数
            when /\A0[0-7]+#{integer_qualifier}/
              @q << [:OCTAL_CONSTANT, Token.new($&, file, lineno, col)]
            # 浮動小数定数
            when /\A[0-9]+\.([0-9]*)?([Ee][+-]?[0-9]+)?/
              @q << [:FLOATING_CONSTANT, Token.new($&, file, lineno, col)]
            # 整数定数
            when /\A\d+#{integer_qualifier}/
            # when /\A\d+/
              @q << [:INTEGER_CONSTANT, Token.new($&.to_i, file, lineno, col)]
            # 文字
            when /\A'(?:[^'\\]|\\.)'/
              @q << [:CHARACTER_LITERAL, Token.new($&, file, lineno, col)]
            # 文字列
#              "#include  #include #include \"../systask/logtask.cfg\"       最後の " 忘れ)で無限ループ
#            when /\A"(?:[^"\\]+|\\.)*"/
            when /\A"(?:[^"\\]|\\.)*"/   # これはうまく行くようだ
              @q << [:STRING_LITERAL, Token.new($&, file, lineno, col)]
            # 行コメント
            when /\A\/\/.*$/
              # 読み飛ばすだけ
            # コメント開始
            when /\A\/\*/
              comment = true
            when /\A>>=/, /\A<<=/, /\A>>/,  /\A<</
              @q << [$&, Token.new($&, file, lineno, col)]
            when /\A\+=/, /\A\-=/, /\A\*=/, /\A\/=/, /\A%=/, /\A&=/, /\A\|=/, /\A\^=/
              @q << [$&, Token.new($&, file, lineno, col)]
            when /\A::/, /\A==/, /\A!=/, /\A>=/, /\A<=/, /\A\->/, /\A\+\+/, /\A\-\-/
              @q << [$&, Token.new($&, file, lineno, col)]
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
       Generator.error( "B1002 while open or reading \'$1\'" , file )
       print_exception( evar )
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

      case token[1].val
      when ";", ":", ",", "(", ")", "{", "}"
        set_no_type_name false
      when ".", "->"
        set_no_type_name true
      end

      # TYPE_NAME トークンへ置き換え
      if @b_no_type_name == false
        if token[0] == :IDENTIFIER && Namespace.is_typename?( token[1].val ) then
          token[0] = :TYPE_NAME
          locale = @@current_locale[@@generator_nest]
#print( "#{locale[0]}: line #{locale[1]} : #{token[0]} '#{token[1].val}: type_name'\n" )
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
    end

    token
  end

  def on_error(t, v, vstack)
    if v == "$" then
     Generator.error( "B1003 Unexpected EOF"  )
    else
     Generator.error( "B1004 syntax error near \'$1\'" , v.val )
    end

  end

  def self.current_locale
    @@current_locale[ @@generator_nest ]
  end

  @@n_error = 0
  @@n_warning = 0
  @@n_info = 0

  # このメソッドは構文解析、意味解析からのみ呼出し可（コード生成でエラー発生は不適切）
  def self.error( msg )
    @@n_error += 1
    locale = @@current_locale[ @@generator_nest ]

    if locale then
      Console.puts "error: #{locale[0]}: line #{locale[1]} #{msg}"
    else
      Console.puts "error: #{msg}"
    end
  end

  # このメソッドは構文解析、意味解析からのみ呼出し可（コード生成でウォーニング発生は不適切）
  def self.warning( msg )
    @@n_warning += 1
    locale = @@current_locale[ @@generator_nest ]
    Console.puts "warning: #{locale[0]}: line #{locale[1]} #{msg}"
  end

  # このメソッドは構文解析、意味解析からのみ呼出し可
  def self.info( msg )
    @@n_info += 1
    locale = @@current_locale[ @@generator_nest ]
    Console.puts "info: #{locale[0]}: line #{locale[1]} #{msg}"
  end

  def self.get_n_error
    @@n_error
  end

  def self.get_n_warning
    @@n_warning
  end

  def self.get_nest
    @@generator_nest
  end

  def set_no_type_name b_no_type_name
    locale = @@current_locale[ @@generator_nest ]
#print "b_no_type_name=#{b_no_type_name} #{locale[0]}: line #{locale[1]}\n"
    @b_no_type_name = b_no_type_name
  end

---- footer

