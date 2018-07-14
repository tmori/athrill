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
#   $Id: messages_console_ja_JP.rb 2633 2017-04-02 06:02:05Z okuma-top $
#++


# Console Messages for ja_JP
class TECSMsg

###
# エラーメッセージ
@@error_message = {}

### C_parser.y.rb
# B1001 need parameter name"
@@error_message[ :"B1001" ] = "パラメータ名が必要です" 

# B1002 while open or reading \'$1\'"
@@error_message[ :"B1002" ] = "\'$1\' のオープンまたは読み込みに失敗しました" 

# B1003 Unexpected EOF"
@@error_message[ :"B1003" ] = "予期しない EOF です"

# B1004 syntax error near \'$1\'"
@@error_message[ :"B1004" ] = "\'$1\' 付近の構文エラーです"

### ctypes.rb
# C1001 $1: mismatch, suitable for integer types"
@@error_message[ :"C1001" ] = "$1: 整合していません, それは整数型に適しています"

# C1002 $1 not compatible with previous one $2"
@@error_message[ :"C1002" ] = "$1 はそれ以前のものである $2 と両立しません"

# C1003 $1 & $2 incompatible  (\'long double\' not support)"
@@error_message[ :"C1003" ] = "$1 及び $2 は両立しません (\'long double\' は未サポート)"

# C1004 $1: qualifier respecified. previous one: $2"
@@error_message[ :"C1004" ] = "$1: 修飾子が再度指定されました. その前の修飾子: $2"

### expression.rb
# E1001 $1: not found"
@@error_message[ :"E1001" ] = "$1: 見つかりません"

# E1002 $1: not constant (port)"
@@error_message[ :"E1002" ] = "$1: 定数ではありません (port)"

# E1003 $1: not constant"
@@error_message[ :"E1003" ] = "$1: 定数ではありません"

# E1004 cannot evaluate \'[]\' operator"
@@error_message[ :"E1004" ] = "\'[]\' 演算子を評価できません"

# E1005 cannot evaluate \'.\' operator"
@@error_message[ :"E1005" ] = "\'.\' 演算子を評価できません"

# E1006 cannot evaluate \'->\' operator"
@@error_message[ :"E1006" ] = "\'->\' 演算子を評価できません"

# E1007 cannot evaluate \'sizeof\' operator"
@@error_message[ :"E1007" ] = "\'sizeof\' 演算子を評価できません"

# E1008 cannot evaluate \'sizeof\' operator"
@@error_message[ :"E1008" ] = "\'sizeof\' 演算子を評価できません"

# E1009 cannot evaluate \'&\' operator"
@@error_message[ :"E1009" ] = "\'&\' 演算子を評価できません"

# E1010 cannot evaluate \'*\' operator"
@@error_message[ :"E1010" ] = "\'*\' 演算子を評価できません"

# E1011 cannot evaluate unary + for $1"
@@error_message[ :"E1011" ] = "$1 に対する単項の + を評価できません"

# E1012 $1: not found in parameter list"
@@error_message[ :"E1012" ] = "$1: パラメータリストの中に見つかりません"

# E1013 \'*\': operand is not pointer value"
@@error_message[ :"E1013" ] = "\'*\': オペランドはポインタ値ではありません"

# E1014 $1: elements_get_type: sorry not supported"
@@error_message[ :"E1014" ] = "$1: elements_get_type: 申し訳ありません．サポートされていません"

# E1015 \'$1\': direction mismatch for $2, $3 required"
@@error_message[ :"E1015" ] = "\'$1\': $2 に対する方向が整合していません．$3 が必要です"

# E1016 $1: elements_check_dir_for_param: sorry not supported"
@@error_message[ :"E1016" ] = "$1: elements_check_dir_for_param: 申し訳ありません．サポートされていません"

# E1017 $1: rhs not \'Cell.ePort\' form"
@@error_message[ :"E1017" ] = "$1: 右辺が \'Cell.ePort\' 形式ではありません"

# E1018 $1: namespace cannot be specified"
@@error_message[ :"E1018" ] = "$1: namespace を指定できません"

# E1019 $1: rhs not in 'allocator_entry_port' form"
@@error_message[ :"E1019" ] = "$1: 右辺が 'allocator_entry_port' 形式ではありません"

# E1020 rhs not in 'call_port.func.param' form for $1_$2"
@@error_message[ :"E1020" ] = "右辺が 'call_port.func.param' 形式でありません ($1_$2)"

### bnf.y.rb
# G1001 need specifier for \'$1\'"
@@error_message[ :"G1001" ] = "\'$1\' に対する指定子が必要です"

# G1002 need parameter name"
@@error_message[ :"G1002" ] = "パラメータ名が必要です"

# G1003 need parameter name"
@@error_message[ :"G1003" ] = "パラメータ名が必要です"

# G1004 impossible array type 0"
@@error_message[ :"G1004" ] = "あり得ない配列型 0 です"

# G1005 impossible array type 1"
@@error_message[ :"G1005" ] = "あり得ない配列型 1 です"

# G1006 impossible array type 2"
@@error_message[ :"G1006" ] = "あり得ない配列型 2 です"

# G1007 impossible array type 3"
@@error_message[ :"G1007" ] = "あり得ない配列型 3 です"

# G1008 impossible function type"
@@error_message[ :"G1008" ] = "あり得ない関数型です"

# G1009 unexpected specifier"
@@error_message[ :"G1009" ] = "予期しない指定子です"

# G1010 Not function"
@@error_message[ :"G1010" ] = "関数ではありません"

# G1011 Not function"
@@error_message[ :"G1011" ] = "関数ではありません"

# G1012 $1 : cannot put specifier here"
@@error_message[ :"G1012" ] = "$1 : ここに指定子を置くことはできません"

# G1013 unexpected specifier"
@@error_message[ :"G1013" ] = "予期しない指定子です"

# G1014 while open or reading \'$1\'"
@@error_message[ :"G1014" ] = "\'$1\' のオープンまたは読み込みに失敗しました"

# G1015 Unexpected EOF"
@@error_message[ :"G1015" ] = "予期しない EOF です"

# G1016 syntax error near \'$1\'"
@@error_message[ :"G1016" ] = "\'$1\' 付近の構文エラーです"

### plugin.rb
# P1001 plugin arg: cannot find identifier in $1"
@@error_message[ :"P1001" ] = "plugin arg: $1 の中に識別子を見つけることができません"

# P1002 plugin arg: expecting \'=\' not \'$1\'"
@@error_message[ :"P1002" ] = "plugin arg: \'$1\' ではなく \'=\' が必要です"

# P1003 plugin arg: unexpected $1"
@@error_message[ :"P1003" ] = "plugin arg: 予期しない $1 です"

# P1004 $1: unknown plugin argument\'s identifier\n  $2 are acceptable for RPCPlugin."
@@error_message[ :"P1004" ] = "$1: 不明なプラグイン引数の識別子\n  $2 が RPC プラグインに対して受け入れ可能です"

### pluginModule.rb
# P2001 $1.rb : fail to load plugin"
@@error_message[ :"P2001" ] = "$1.rb : プラグインの読み込みに失敗しました"

# P2002 $1: not kind of $2"
@@error_message[ :"P2002" ] = "$1: $2 の一種ではありません"

# P2003 $1: load failed"
@@error_message[ :"P2003" ] = "$1: 読み込みに失敗しました"

# P2004 $1: open error \'$2\'"
@@error_message[ :"P2004" ] = "$1: オープンエラー \'$2\' が発生しました"

# P2005 $1: plugin error in gen_through_cell_code "
@@error_message[ :"P2005" ] = "$1: gen_through_cell_code 内のプラグインエラーが発生しました"

# P2006 $1: close error \'$2\'"
@@error_message[ :"P2006" ] = "$1: \'$2\' のクローズエラーが発生しました"

# P2007 $1: fail to generate post code"
@@error_message[ :"P2007" ] = "$1: ポストコードの生成に失敗しました"

### componentobj.rb
# S1001 context specifier duplicate"
@@error_message[ :"S1001" ] = "コンテキスト指定子が重複しています"

# S1002 \'$1\': unknown specifier for signature"
@@error_message[ :"S1002" ] = "\'$1\': シグニチャに対する不明な指定子です"

# S1003 $1: \'alloc\' 1st parameter neither [in] integer type nor [out] double pointer type
@@error_message[ :"S1003" ] = "$1: \'alloc\' 関数の第1パラメータが in の整数型でも out の二重ポインタ型でもありません．"

# S1004 $1: \'alloc\' 2nd parameter not [in] double pointer"
@@error_message[ :"S1004" ] = "$1: \'alloc\' 関数の第2パラメータが in のポインタへのポインタではありません．"

# S1005 $1: \'alloc\' has no parameter, unsuitable for allocator signature"
@@error_message[ :"S1005" ] = "$1: \'alloc\' 関数がパラメータを持っていません．アロケータシグニチャに対して不適当です．"

# S1006 $1: \'dealloc\' 1st parameter not [in] pointer type"
@@error_message[ :"S1006" ] = "$1: \'dealloc\' 関数の第1パラメータが in のポインタ型ではありません．"

# S1007 "
     # S1007 $1: \'dealloc\' cannot has 2nd parameter"
# @@error_message[ :"S1007" ] = "$1: \'dealloc\' 関数は第2パラメータを持つことができません．"

# S1008 $1: \'dealloc\' has no parameter, unsuitable for allocator signature"
@@error_message[ :"S1008" ] = "$1: \'dealloc\' 関数がパラメータを持っていません．アロケータシグニチャに対して不適当です．"

# S1009 $1: \'alloc\' function not found, unsuitable for allocator signature"
@@error_message[ :"S1009" ] = "$1: \'alloc\' 関数が見つかりません．アロケータシグニチャに対して不適当です．"

# S1010 $1: \'dealloc\' function not found, unsuitable for allocator signature"
@@error_message[ :"S1010" ] = "$1: \'dealloc\' 関数が見つかりません．アロケータシグニチャに対して不適当です．"

# S1011 $1: size_is specified for non-pointer type"
@@error_message[ :"S1011" ] = "$1: 非ポインタ型に対して size_is が指定されました"

# S1012 $1: unsuitable initializer, need array initializer"
@@error_message[ :"S1012" ] = "$1: 不適当な初期化子です．配列の初期化子が必要です．"

# S1013 $1: too many initializer, $2 for $3"
@@error_message[ :"S1013" ] = "$1: 初期化子が多すぎます．$3 とするところを $2 としました．"

# S1014 generate specifier duplicate"
@@error_message[ :"S1014" ] = "generate 指定子が重複しました．"

# S1015 $1 cannot be specified for composite"
@@error_message[ :"S1015" ] = "$1 は複合セルに対して指定できません．"

# S1016 $1 not found"
@@error_message[ :"S1016" ] = "$1 が見つかりません"

# S1017 $1 : neither celltype nor cell"
@@error_message[ :"S1017" ] = "$1 : セルタイプでもセルでもありません"

# S1018 $1 : not singleton cell"
@@error_message[ :"S1018" ] = "$1 : シングルトンセルではありません"

# S1019 \'$1\' : not entry port"
@@error_message[ :"S1019" ] = "\'$1\' : 受け口ではありません"

# S1020 \'$1\' : required port cannot be array"
@@error_message[ :"S1020" ] = "\'$1\' : 要求された口(port)は配列に出来ません．"

# S1021 $1 : require cannot have same signature with \'$2\'"
@@error_message[ :"S1021" ] = "$1 : リクワイアは \'$2\' と同じシグニチャを持つことが出来ません．"

# S1022 $1.$2 : \'$3\' conflict function name in $4.$5"
@@error_message[ :"S1022" ] = "$1.$2 : \'$3\' は $4.$5 における関数名と衝突しています．"

# S1023 $1: fail to new"
@@error_message[ :"S1023" ] = "$1: new に失敗しました．"

# S1024 $1: multiple cell for singleton celltype"
@@error_message[ :"S1024" ] = "$1: シングルトンセルタイプに対し複数のセルが存在します．"

# S1025 not found reachable cell for require \'$1\' in celltype \'$2\'"
@@error_message[ :"S1025" ] = "セルタイプ \'$2\' 内のリクワイア \'$1\' へ到達可能なセルが見つかりません．"

# S1026 required cell \'$1\' not reachable"
@@error_message[ :"S1026" ] = "必要とされるセル \'$1\' は到達可能ではありません．"

# S1027 \'$1\' celltype not found"
@@error_message[ :"S1027" ] = "セルタイプ \'$1\' が見つかりません"

# S1028 \'$1\' not celltype"
@@error_message[ :"S1028" ] = "\'$1\' はセルタイプではありません"

# S1029 $1 mismatch with previous one"
@@error_message[ :"S1029" ] = "$1 がそれ以前のものと整合していません"

# S1030 $1: celltype mismatch with previous one"
@@error_message[ :"S1030" ] = "$1: セルタイプがそれ以前のものと整合していません"

# S1031 $1 region \'$2\' mismatch  with previous one \'$3\'"
@@error_message[ :"S1031" ] = "セル $1 のリージョン \'$2\' はそれ以前のリージョン \'$3\' と整合していません．"

# S1032 $1: duplicate cell"
@@error_message[ :"S1032" ] = "$1: セルが重複しています"

# S1033 rhs expression is not supported. Only attribute is permitted on current version."
@@error_message[ :"S1033" ] = "右辺式はサポートされていません．属性のみが現在のバージョンで使用できます．"

# S1034 $1 : cannot refer to $2\'s attribute here. Use \'composite.$3\' to refer to composite celltype\'s"
@@error_message[ :"S1034" ] = "$1 : ここでは $2 の属性を参照できません. 複合セルタイプの属性を参照するため \'composite.$3\' をお使いください．"

# S1035 composite : cannot specify out of composite celltype definition"
@@error_message[ :"S1035" ] = "composite : 複合セルタイプ定義の外側では指定できません．"

# S1036 $1 : cannot refer to $2\'s here. Use \'composite.$3\' to refer to composite celltype\'s"
@@error_message[ :"S1036" ] = "$1 : ここでは$2のものを参照できません．複合セルタイプのものを参照するため \'composite.$3\' をお使いください．"

# S1037 $1: celltype plugin fail to new_cell"
@@error_message[ :"S1037" ] = "$1: セルタイププラグインが new_cell に失敗しました．"

# S1038 $1.$2: self allocator not supported for array entry port"
@@error_message[ :"S1038" ] = "$1.$2: セルフアロケータは受け口配列に対してサポートされていません．"

# S1039 \'$1\': unknown specifier for cell"
@@error_message[ :"S1039" ] = "\'$1\': セルに対する不明な指定子です．"

# S1040 array not supported for relay allocator"
@@error_message[ :"S1040" ] = "配列はリレーアロケータに対してサポートされていません．"

# S1041 \'$1_$2_$3\': not joined. cannot create internal join for relay allocator"
@@error_message[ :"S1041" ] = "\'$1_$2_$3\': 結合されません. リレーアロケータに対して内部結合を生成できません．"

# S1042 call port \'$1\' not initialized in cell \'$2\'"
@@error_message[ :"S1042" ] = "呼び口 \'$1\' はセル \'$2\' の中で初期化されません．"

# S1043 call port \'$1\' not initialized in cell \'$2\'. this call port is created by tecsgen. check allocator specifier"
@@error_message[ :"S1043" ] = "呼び口 \'$1\' はセル \'$2\' の中で初期化されません．この呼び口は tecsgen により生成されます．アロケータ指定子を確認してください．"

# S1044 $1: array initializer too many or few, $2 for $3"
@@error_message[ :"S1044" ] = "$1: 配列の初期化子が多すぎるかまたは少なすぎます．$3 とするところを $2 にしました．"

# S1045 $1[$2]: not initialized"
@@error_message[ :"S1045" ] = "$1[$2]: 初期化されません．"

# S1046 $1[$2]: not initialized"
@@error_message[ :"S1046" ] = "$1[$2]: 初期化されません"

# S1047 size_is pointer cannot be exposed for composite attribute"
@@error_message[ :"S1047" ] = "size_is を指定されたポインタ変数は，複合セルタイプの属性名に関しては，外部へ公開することができません．"

# S1048 $1: size_is specified for non-pointer type"
@@error_message[ :"S1048" ] = "$1: size_is が非ポインタ型に対して指定されました．"

# S1049 $1: size_is arg not constant"
@@error_message[ :"S1049" ] = "$1: size_is 引数が定数ではありません．"

# S1050 unsuitable initializer, need array initializer"
@@error_message[ :"S1050" ] = "不適当な初期化子です．配列の初期化子が必要です．"

# S1051 too many initializer for array, $1 for $2"
@@error_message[ :"S1051" ] = "配列に対する初期化子が多すぎます．$2 とするところを $1 にしました．"

# S1052 attribute \'$1\' not initialized in cell \'$2\'"
@@error_message[ :"S1052" ] = "属性 \'$1\' はセル \'$2\' の中で初期化されません．"

# S1053 $1 must be singleton. inner cell \'$2\' is singleton"
@@error_message[ :"S1053" ] = "$1 はシングルトンである必要があります．内部セル \'$2\' はシングルトンです．"

# S1054 $1 : specified active but has no active in this celltype"
@@error_message[ :"S1054" ] = "$1 : アクティブに指定されましたが，このセルタイプの中にはアクティブなセルがありません．"

# S1055 $1 must be active. inner cell \'$2\' is active"
@@error_message[ :"S1055" ] = "$1 は active である必要があります. 内部セル \'$2\' がアクティブです．"

# S1056 $1 : cannot export, nothing designated"
@@error_message[ :"S1056" ] = "$1 : 外部に公開することができません．何も指定されていません．"

# S1057 $1 not found in $2"
@@error_message[ :"S1057" ] = "$1 が $2 の中に見つかりません．"

# S1058 \'$1\' : cannot export var"
@@error_message[ :"S1058" ] = "\'$1\' : 変数を外部に公開することができません．"

# S1059 \'$1\' : exporting attribute. write in cell or use \'=\' to export attribute"
@@error_message[ :"S1059" ] = "\'$1\' : 属性を公開しています. セルの中に記述するかまたは属性の公開のために \'=\' を使って下さい．"

# S1060 \'$1\' : port type mismatch. $2 type is allowed here."
@@error_message[ :"S1060" ] = "\'$1\' : 口(port)の型が整合していません. ここでは $2 型が使用できます．"

# S1061 \'$1\' : not defined"
@@error_message[ :"S1061" ] = "\'$1\' : 定義されていません．"

# S1062 $1 has no export definition"
@@error_message[ :"S1062" ] = "$1 には外部定義がありません．"

# S1063 $1 is port but previously defined as an attribute"
@@error_message[ :"S1063" ] = "$1 は口(port)ですがそれより前に属性として定義されました．"

# S1064 $1 : type \'$2$3\' mismatch with pprevious definition\'$4$5\'"
@@error_message[ :"S1064" ] = "$1 : 型 \'$2$3\' はそれ以前の定義 \'$4$5\' と整合してません．"

# S1065 $1 : port type $2 mismatch with previous definition $3"
@@error_message[ :"S1065" ] = "$1 : 口(port)型 $2 はそれ以前の定義 $3 と整合していません．"

# S1066 $1 : signature \'$2\' mismatch with previous definition \'$3\'"
@@error_message[ :"S1066" ] = "$1 : シグニチャ \'$2\' はそれ以前の定義 \'$3\' と整合していません．"

# S1067 $1 : array size mismatch with previous definition"
@@error_message[ :"S1067" ] = "$1 : 配列のサイズがそれ以前の定義と整合していません．"

# S1068 $1 : optional specifier mismatch with previous definition"
@@error_message[ :"S1068" ] = "$1 : オプショナルな指定子がそれ以前の定義と整合していません．"

# S1069 $1 is an attribute but previously defined as a port"
@@error_message[ :"S1069" ] = "$1 は属性ですがそれより前に口(port)として定義されました．"

# S1070 $1: size_is pointer cannot be exposed for composite attribute"
@@error_message[ :"S1070" ] = "$1: size_is を指定されたポインタ変数は，複合セルタイプの属性名に関しては，公開することができません．"

# S1071 $1 cannot be specified for composite"
@@error_message[ :"S1071" ] = "$1 は複合セルに対して指定できません．"

# S1072 $1: entry port: sizeless array not supported in current version"
@@error_message[ :"S1072" ] = "$1: 受け口: 現在の版ではサイズ未指定の配列はサポートされていません．"

# S1073 Not constant expression $1"
@@error_message[ :"S1073" ] = "$1 は定数式ではありません．"

# S1074 Not Integer $1"
@@error_message[ :"S1074" ] = "$1 は整数ではありません"

# S1075 \'$1\' signature not found"
@@error_message[ :"S1075" ] = "\'$1\' シグニチャが見つかりません．"

# S1076 \'$1\' not signature"
@@error_message[ :"S1076" ] = "\'$1\' はシグニチャではありません．"

# S1077 inline: cannot be specified for call port"
@@error_message[ :"S1077" ] = "inline: 呼び口に対して指定できません．"

# S1078 optional: cannot be specified for entry port"
@@error_message[ :"S1078" ] = "optional: 受け口に対して指定できません．"

# S1079 allocator: cannot be specified for call port"
@@error_message[ :"S1079" ] = "allocator: 呼び口に対して指定できません．"

# S1080 duplicate allocator specifier"
@@error_message[ :"S1080" ] = "アロケータ指定子が重複しています．"

# S1081 self allocator not supported yet"
@@error_message[ :"S1081" ] = "セルフアロケータはまだサポートされていません"

# S1082 function \'$1\' not found in signature"
@@error_message[ :"S1082" ] = "関数 \'$1\' がシグニチャで見つかりません"

# S1083 \'$1\' not found in function \'$2\'"
@@error_message[ :"S1083" ] = "\'$1\' が関数 \'$2\' で見つかりません"

# S1084 \'$1\' in function \'$2\' is not send or receive"
@@error_message[ :"S1084" ] = "\'$2\' 内の \'$1\' が基本指定子 send または receive ではありません．"

# S1085 duplicate allocator specifier for \'$1_$2\'"
@@error_message[ :"S1085" ] = "\'$1_$2\' に対する重複したアロケータ指定子です．"

# S1086 rhs not call_port.func.param for $1_$2"
@@error_message[ :"S1086" ] = "$1_$2 に対して not call_port.func.param の形式ではありません．"

# S1087 function \'$1\' not found in signature \'$2\'"
@@error_message[ :"S1087" ] = "関数 \'$1\' がシグニチャ \'$2\' の中に見つかりません．"

# S1088 \'$1\' not found in function \'$2\'"
@@error_message[ :"S1088" ] = "\'$1\' が関数 \'$2\' の中に見つかりません．"

# S1089 relay allocator send/receive mismatch between $1.$2 and $3_$4.$5"
@@error_message[ :"S1089" ] = "リレーアロケータ send/receive は $1.$2 及び $3_$4.$5 の間で整合していません．"

# S1090 \'$1\' in function \'$2\' is not send or receive"
@@error_message[ :"S1090" ] = "関数 \'$2\' 内の \'$1\' が send または receive ではありません．"

# S1091 call port \'$1\' not found in celltype $2"
@@error_message[ :"S1091" ] = "呼び口 \'$1\' がセルタイプ $2 の中に見つかりません．"

# S1092 \'$1\' not namespace"
@@error_message[ :"S1092" ] = "\'$1\' はネームスペースではありません．"

# S1093 $1 : undefined cell"
@@error_message[ :"S1093" ] = "$1 : 未定義のセルです"

# S1094 $1: pointer is not constant. check \'const\'"
@@error_message[ :"S1094" ] = "$1: ポインタが定数ではありません．\'const\' を確認してください"

# S1095 $1: not constant"
@@error_message[ :"S1095" ] = "$1: 定数ではありません"

# S1096 $1: should be int, float, bool or pointer type"
@@error_message[ :"S1096" ] = "$1: int, float, bool またはポインタ型である必要があります"

# S1097 $1: has no initializer"
@@error_message[ :"S1097" ] = "$1: 初期化子を持っていません"

# S1098 $1: has unsuitable initializer"
@@error_message[ :"S1098" ] = "$1: 不適当な初期化子があります．"

# S1099 array subscript not constant"
@@error_message[ :"S1099" ] = "配列添数が定数ではありません．"

# S1100 $1: cannot initialize var"
@@error_message[ :"S1100" ] = "$1: 変数を初期化できません．"

# S1101 \'$1\' cannot initialize entry port"
@@error_message[ :"S1101" ] = "\'$1\' 受け口を初期化できません．"

# S1102 $1: must specify array subscript here"
@@error_message[ :"S1102" ] = "$1: ここでは配列添数を指定する必要があります．"

# S1103 $1: need array subscript"
@@error_message[ :"S1103" ] = "$1: 配列添数が必要です．"

# S1104 $1: need array subscript number. ex. \'[0]\'"
@@error_message[ :"S1104" ] = "$1: 配列添数の数値が必要です．(例) \'[0]\'"

# S1105 $1: cannot specify array subscript here"
@@error_message[ :"S1105" ] = "$1: ここでは配列添数を指定することができません．"

# S1106 $1: cannot specify array subscript number. use \'[]\'"
@@error_message[ :"S1106" ] = "$1: 配列添数の数値を指定することが出来ません．\'[]\' をお使いください．"

# S1107 to export port, use \'cCall => composite.cCall\'"
@@error_message[ :"S1107" ] = "口(port)を外部に公開するため, \'cCall => composite.cCall\' をお使いください．"

# S1108 $1: rhs not \'Cell.ePort\' form"
@@error_message[ :"S1108" ] = "$1: 右辺が \'Cell.ePort\' の形式ではありません．"

# S1109 \'$1\' not found"
@@error_message[ :"S1109" ] = "\'$1\' が見つかりません．"

# S1110 \'$1\' not cell"
@@error_message[ :"S1110" ] = "\'$1\' セルではありません．"

# S1111 \'$1\' not found"
@@error_message[ :"S1111" ] = "\'$1\' が見つかりません．"

# S1112 \'$1\' not entry port"
@@error_message[ :"S1112" ] = "\'$1\' 受け口ではありません．"

# S1113 \'$1\' signature mismatch"
@@error_message[ :"S1113" ] = "\'$1\' シグニチャが整合していません．"

# S1114 \'$1\' should be array"
@@error_message[ :"S1114" ] = "\'$1\' 配列である必要があります．"

# S1115 $1[$2]: subscript out of range (< $3)"
@@error_message[ :"S1115" ] = "$1[$2]: 配列の添数が範囲外の値です．(< $3)"

# S1116 \'$1\' entry port is not array"
@@error_message[ :"S1116" ] = "\'$1\' 受け口が配列ではありません．"

# S1117 \'$1\' not in celltype"
@@error_message[ :"S1117" ] = "\'$1\' はセルタイプ内にありません．"

# S1118 $1: going out from region \'$2\' not permitted"
@@error_message[ :"S1118" ] = "$1: リージョン $2 から out することができません．"

# S1119 $1: going from region \'$2\' to \'$3\' not permitted"
@@error_message[ :"S1119" ] = "$1: リージョン $2 からリージョン $3へ結合できません．"

# S1120 $1: going in to region \'$2\' not permitted"
@@error_message[ :"S1120" ] = "$1: リージョン \'$2\' へ in することができません．"

# S1121 \'$1\' in region \'$2\' cannot be directly joined $3 in  $4"
@@error_message[ :"S1121" ] = "リージョン \'$2\' 内の \'$1\' は直接 $4 内の $3 に結合できません．"

# S1122 $1 : not port: \'through\' can be specified only for port"
@@error_message[ :"S1122" ] = "$1 :口(port) ではありません: \'through\' は口(port)に対してのみ指定できます．"

# S1123 $1 : not port: \'through\' can be specified only for port"
@@error_message[ :"S1123" ] = "$1 :口(port) ではありません: \'through\' は口(port)に対してのみ指定できます．"

# S1124 $1: plugin function failed: \'get_through_entry_port_name\'"
@@error_message[ :"S1124" ] = "$1: プラグイン関数が失敗しました．: \'get_through_entry_port_name\'"

# S1125 $1: not generated cell \'$2\'"
@@error_message[ :"S1125" ] = "$1: 生成されたセル \'$2\' ではありません．"

# S1126 $1: fail to new"
@@error_message[ :"S1126" ] = "$1: new に失敗しました．"

# S1127 \'$1\' duplicate"
@@error_message[ :"S1127" ] = "\'$1\' が重複しています．"

# S1128 \'$1\' inconsistent array definition"
@@error_message[ :"S1128" ] = "\'$1\' は一貫しない配列定義です．"

# S1129 \'$1\' redefinition of subscript $1"
@@error_message[ :"S1129" ] = "\'$1\' は配列添数 $1 の再定義です．"

# S1130 \'$1\' inconsistent array definition"
@@error_message[ :"S1130" ] = "\'$1\' は一貫しない配列定義です．"

# S1131 \'$1.$2\' has duplicate initializer"
@@error_message[ :"S1131" ] = "\'$1.$2\' には重複した初期化子があります．"

# S1132 $1: 1st parameter is not string(file name)"
@@error_message[ :"S1132" ] = "$1: 第1パラメータが string(ファイル名) ではありません．"

# S1133 $1: 2nd parameter is not string(fromat)"
@@error_message[ :"S1133" ] = "$1: 第2パラメータが string(ファイル名) ではありません．"

# S1134 $1: unknown factory function"
@@error_message[ :"S1134" ] = "$1: 不明なファクトリ関数です．"

# S1135 celltype factory can\'t have parameter(s)"
@@error_message[ :"S1135" ] = "セルタイプファクトリはパラメータを持つことができません．"

# S1136 \'$1\': not found"
@@error_message[ :"S1136" ] = "\'$1\': 見つかりません．"

# S1137 \'$1\': not attribute"
@@error_message[ :"S1137" ] = "\'$1\': 属性ではありません．"

# S1138 internal error Factory.check_arg()"
@@error_message[ :"S1138" ] = "Factory.check_arg() の内部エラーです．"

# S1139 $1: region path mismatch. previous path: $2"
@@error_message[ :"S1139" ] = "$1: リージョンパスが整合していません．以前のパス: $2"

# S1140 $1: region specifier must place at first appearence"
@@error_message[ :"S1140" ] = "$1: リージョン指定子は最初に配置する必要があります．"

# S1141 $1 duplication, previous one : $2"
@@error_message[ :"S1141" ] = "$1 が重複しています, 以前の値 : $2"

# S1142 $1 not found in search path"
@@error_message[ :"S1142" ] = "$1 が検索パスの中に見つかりません．"

# S1143 import_C: arg2: mismatch with previous one"
@@error_message[ :"S1143" ] = "import_C: arg2: 以前のものと整合していません．"

# S1144 $1: temporary C source: open error"
@@error_message[ :"S1144" ] = "$1: 一時的な C ソース: オープンエラーです．"

# S1145 $1: temporary C source: writing error"
@@error_message[ :"S1145" ] = "$1: 一時的な C ソース: 書込みエラーです．"

# S1146 $1: error occured while CPP"
@@error_message[ :"S1146" ] = "$1: CPP 実行中にエラーが発生しました．"

# S1147 $1: popen for CPP failed"
@@error_message[ :"S1147" ] = "$1: CPPに対する popen が失敗しました．"

# S1148 $1 not found in search path"
@@error_message[ :"S1148" ] = "$1 が検索パスの中に見つかりません．"

# S1149 $1 not signature"
@@error_message[ :"S1149" ] = "$1 シグニチャではありません．"

# S1150 $1: fail to new"
@@error_message[ :"S1150" ] = "$1: new に失敗しました．"

# S1151 $1: not namespace"
@@error_message[ :"S1151" ] = "$1: ネームスペースではありません．"

# S1152 $1: call port cannot have fixed join"
@@error_message[ :"S1152" ] = "$1: 呼び口は固定結合を持つことができません．"

# "S1153 $1: cannot be entry port array for fixed join port"
@@error_message[ :"S1153" ] = "$1: 固定結合の口に対する受け口配列はあり得ません．"

# "S1154 $1: must be singleton celltype for fixed join"
@@error_message[ :"S1154" ] = "$1: 固定結合に対するシングルトンセルタイプである必要があります．"

# "S1155 $1: not celltype or not found"
@@error_message[ :"S1155" ] = "$1: セルタイプではないかまたは見つかりません．"

# "S1156 $1: not call port or not found"
@@error_message[ :"S1156" ] = "$1: 呼び口ではないかまたは見つかりません．"

# "S1157 $1: sized array or not array"
@@error_message[ :"S1157" ] = "$1: サイズ指定された配列かまたは配列ではありません．"

# "S1158 $1: singleton cell not found for fixed join"
@@error_message[ :"S1158" ] = "$1: シングルトンセルが固定結合に対して見つかりません．"

# S1159 $1: non-size_is pointer cannot be initialized with array initializer"
@@error_message[ :"S1159" ] = "$1: size_is 指定されていないポインタは、配列初期化子で初期化できません"

# S1160 $1 must be constant for id"
@@error_message[ :"S1160" ] = "$1 id は定数でなければなりません"

# S1161 $1 must be constant for id"
@@error_message[ :"S1161" ] = "$1 id は定数でなければなりません"

# S1162 $1: id cannot be 0"
@@error_message[ :"S1162" ] = "$1: id に 0 を指定できません"

# S1163 generate specifier duplicate"
@@error_message[ :"S1163" ] = "generate 指定子が重複しています"

# S1164 '$1' set_specified_id: id not positive integer '$2'"
@@error_message[ :"S1164" ] = "'$1' id が正整数でありません '$2'"

# S1165 '$1' set_specified_id: id duplicate"
@@error_message[ :"S1165" ] = "'$1' id 指定子が重複しています"

# S1166 $1: fail to new"
@@error_message[ :"S1166" ] = "$1: プラグインの生成に失敗しました"

# S1167 \'$1\': relay mismatch \'$2\'"
@@error_message[ :"S1167" ] = "\'$1\': リレーにマッチしません \'$2\'"

# S1168 too many initializer for array, $1 for $2"
@@error_message[ :"S1168" ] = "配列初期化子が多すぎます $1 for $2"

# S1169 $1: non-size_is pointer cannot be initialized with array initializer"
@@error_message[ :"S1169" ] = "$1: size_is 指定されていないポインタに配列初期化子を指定できません"

# S1170 \'$1\' has size_is but export attr \'$2\' doesn't have"
@@error_message[ :"S1170" ] = "\'$1\' に size_is 指定されていますがエクスポートする \'$2\' に指定されていません"

# S1171 \'$1\' size_is argument of \'$2\' not exported"
@@error_message[ :"S1171" ] = "\'$1\' \'$2\' size_is 引数がエクスポートされていません"

# S1172 \'$1\' size_is argument mismatch with exporting one \'$2\'"
@@error_message[ :"S1172" ] = "\'$1\' size_is 引数がエクスポートされている \'$2\' と一致しません"

# S1173 $1: allocator mismatch from $2's allocator"
@@error_message[ :"S1173" ] = "$1: アロケータが $2' のアロケータと一致しません"

# S1174 $1 not suitable for lhs, suitable lhs: 'func.param'"
@@error_message[ :"S1174" ] = "$1 左辺が不適切です。適切な左辺は 'func.param' の形式です"

# S1175 $1 not found or not allocator entry port for $2"
@@error_message[ :"S1175" ] = "$1  $2 のアロケータ受け口が見当たらないか、アロケータ受け口ではありません"

# S1176 rhs not in 'call_port.func.param' form for $1_$2"
@@error_message[ :"S1176" ] = "右辺が 'call_port.func.param' 形式でありません ($1_$2)"

# S1177 cannot specify 'through' in composite in current version"
@@error_message[ :"S1177" ] = "現在のバージョンでは、コンポジットセルに 'through'を指定できません"

# S1178 $1 region type specifier duplicate, previous $2"
@@error_message[ :"S1178" ] = "$1 リージョンタイプ指定子が重複しています。以前は $2 でした"

### syntaxobj.rb
# S2001 \'$1\' duplicate $2"
@@error_message[ :"S2001" ] = "\'$1\': 重複しています ($2)"

# S2002 $1: $2"
@@error_message[ :"S2002" ] = "$1: $2"

# S2003 $1: $2 cannot have initializer"
@@error_message[ :"S2003" ] = "$1: $2 は初期化子をもつことができません．"

# S2004 $1: array subscript must be specified or omit"
@@error_message[ :"S2004" ] = "$1: 配列添数は指定されるかまたは省略される必要があります．"

# S2005 $1: array subscript must be specified"
@@error_message[ :"S2005" ] = "$1: 配列添数は指定される必要があります．"

# S2006 \'$1\' function"
@@error_message[ :"S2006" ] = "\'$1\' 関数です．"

# S2007 \'$1\' $2"
@@error_message[ :"S2007" ] = "\'$1\' $2"

# S2008 $1: inconsitent with previous one"
@@error_message[ :"S2008" ] = "$1: 以前のものと一貫していません．"

# S2009 $1: not found or not signature"
@@error_message[ :"S2009" ] = "$1: 見つからないかまたはシグニチャではありません．"

# S2010 $1: not allocator signature"
@@error_message[ :"S2010" ] = "$1: アロケータシグニチャではありません．"

# S2011 size_is duplicate"
@@error_message[ :"S2011" ] = "size_is が重複しています．"

# S2012 count_is duplicate"
@@error_message[ :"S2012" ] = "count_is が重複しています．"

# S2013 string duplicate"
@@error_message[ :"S2013" ] = "string が重複しています．"

# S2014 $1 need pointer or more pointer"
@@error_message[ :"S2014" ] = "$1 は一つ以上のポインタが必要です．"

# S2015 $1 must be const for \'in\' parameter $2"
@@error_message[ :"S2015" ] = "'$1' は \'in\' パラメータ $2 に対して定数である必要があります．"

# S2016 $1 can not be const for $2 parameter"
@@error_message[ :"S2016" ] = "'$1' は $2 パラメータに対して定数指定できません． "

# S2017 size_is argument is not integer type"
@@error_message[ :"S2017" ] = "size_is 引数が整数型ではありません．"

# S2018 \'$1\' size_is parameter not integer"
@@error_message[ :"S2018" ] = "\'$1\' size_is パラメータは整数ではありません．"

# S2019 \'$1\' size_is parameter negative or zero"
@@error_message[ :"S2019" ] = "\'$1\' size_is パラメータが負数またはゼロです．"

# S2020 count_is argument is not integer type"
@@error_message[ :"S2020" ] = "count_is 引数は整数型ではありません．"

# S2021 \'$1\' count_is parameter not integer"
@@error_message[ :"S2021" ] = "\'$1\' count_is パラメータは整数型ではありません．"

# S2022 \'$1\' count_is parameter negative or zero"
@@error_message[ :"S2022" ] = "\'$1\' count_is パラメータは負数またはゼロです．"

# S2023 string argument is not integer type"
@@error_message[ :"S2023" ] = "string 引数は整数型ではありません．"

# S2024 \'$1\' string parameter not integer"
@@error_message[ :"S2024" ] = "\'$1\' string パラメータは整数型ではありません．"

# S2025 \'$1\' string parameter negative or zero"
@@error_message[ :"S2025" ] = "\'$1\' string パラメータは負数またはゼロです．"

# S2026 '$1' nullable specified for non-pointer type"
@@error_message[ :"S2026" ] = "'$1' ポインタ型以外に nullable が指定されました"

# S2027 '$1' parameter cannot be void type"
@@error_message[ :"S2027" ] = "'$1' 引数を void 型にできません"

# S2028 '$1' max (size_is 2nd parameter) not constant"
@@error_message[ :"S2028" ] = "'$1' max (size_is の第二引数)が定数でありません"

# S2029 '$1' max (size_is 2nd parameter) negative or zero, or not integer"
@@error_message[ :"S2029" ] = "'$1' max (size_is の第二引数) が負、ゼロまたは整数ではありません"

# S2030 '$1' both size_is and max are const. size_is larger than max"
@@error_message[ :"S2030" ] = "'$1' size_is max の両方が定数で. size_is の方が max を超えています"

### optimize.rb
# S3001 $1: id too large $2 (max=$3)"
@@error_message[ :"S3001" ] = "$1: id に $2 は大きすぎます (最大値は$3)"

# S3002 $1: id too large $2 (max=$3)"
@@error_message[ :"S3002" ] = "$1: id too large $2 (max=$3)"

# S3003 $1: id number '$2' conflict with $3"
@@error_message[ :"S3003" ] = "$1: id 番号 '$2' は $3 と衝突しています"

### types.rb
# T1001 const duplicate"
@@error_message[ :"T1001" ] = "const が重複しています．"

# T1002 volatile duplicate"
@@error_message[ :"T1002" ] = "volatile が重複しています．"

# T1003 $1: unsuitable specifier for $2"
@@error_message[ :"T1003" ] = "$1: $2 に対する不適切な指定子です．"

# T1004 cannot cast to $1"
@@error_message[ :"T1004" ] = "$1 へキャストできません．"

# T1005 \'$1\' not defined"
@@error_message[ :"T1005" ] = "\'$1\' は定義されていません．"

# T1006 \'$1\' not type name. expecting type name here"
@@error_message[ :"T1006" ] = "\'$1\' は型名ではありません．ここは型名が必要です．"

# T1007 $1: void type variable cannot have initializer"
@@error_message[ :"T1007" ] = "$1: void 型変数は初期化子をもつことができません．"

# T1008 ambigous signed or unsigned"
@@error_message[ :"T1008" ] = "符号付きであるかまたは符号無しであるかが曖昧です．"

# T1009 $1: $2: not integer"
@@error_message[ :"T1009" ] = "$1: $2: 整数ではありません．"

# T1010 $1: initializer is not constant"
@@error_message[ :"T1010" ] = "$1: 初期化子は定数ではありません．"

# T1011 $1: need cast to assign float to integer"
@@error_message[ :"T1011" ] = "$1: 浮動小数点型から整数型へ割り当てのためにキャストが必要です．"

# T1012 $1: $2: not integer"
@@error_message[ :"T1012" ] = "$1: $2: 整数ではありません．"

# T1013 $1: too large (max=$2)"
@@error_message[ :"T1013" ] = "$1: 値が大きすぎます (max=$2)"

# T1014 $1: too large negative value (min=-$2)"
@@error_message[ :"T1014" ] = "$1: 負数の絶対値が大きすぎます．(min=-$2)"

# T1015 $1: negative value for unsigned"
@@error_message[ :"T1015" ] = "$1: 符号無し整数型に対し負数値が使用されています．"

# T1016 $1: too large (max=$2)"
@@error_message[ :"T1016" ] = "$1: 値が大きすぎます．(max=$2)"

# T1017 $1: unsuitable initializer for scalar type"
@@error_message[ :"T1017" ] = "$1: スカラー型に対する不適当な初期化子です．"

# T1018 $1: $2: not number"
@@error_message[ :"T1018" ] = "$1: $2: 数値ではありません．"

# T1019 $1: initializer is not constant"
@@error_message[ :"T1019" ] = "$1: 初期化子は定数ではありません．"

# T1020 $1: unsuitable initializer for scalar type"
@@error_message[ :"T1020" ] = "$1: スカラー型に対する不適当な初期化子です．"

# T1021 \'$1\': struct not defined"
@@error_message[ :"T1021" ] = "\'$1\': 構造体が定義されていません．"

# T1022 struct $1: not defined"
@@error_message[ :"T1022" ] = "構造体 $1: 定義されていません．"

# T1023 struct $1: not defined"
@@error_message[ :"T1023" ] = "構造体 $1: 定義されていません．"

# T1024 $1: unsuitable initializer for struct"
@@error_message[ :"T1024" ] = "$1: 構造体に対する不適当な初期化子です．"

# T1025 size_is argument is not integer type"
@@error_message[ :"T1025" ] = "size_is 引数 は整数型ではありません．"

# T1026 count_is argument is not integer type"
@@error_message[ :"T1026" ] = "count_is 引数 は整数型ではありません．"

# T1027 string argument is not integer type"
@@error_message[ :"T1027" ] = "string 引数は整数型ではありません．"

# T1028 $1: cannot initialize function pointer"
@@error_message[ :"T1028" ] = "$1: 関数ポインタを初期化することはできません．"

# T1029 oneway function cannot return type \'$1$2\', \'void\' or \'ER\' is permitted"
@@error_message[ :"T1029" ] = "一方向関数は型 \'$1$2\' を返すことができません．\'void\' または \'ER\' が使用できます．"

# T1030 oneway function cannot have $1 parameter for \'$2\'"
@@error_message[ :"T1030" ] = "一方向関数は \'$2\' のために $1 パラメータを持つことができません．"

# T1031 $1: unsuitable initializer for array"
@@error_message[ :"T1031" ] = "$1: 配列に対する不適当な初期化子です．"

# T1032 $1: incompatible pointer type"
@@error_message[ :"T1032" ] = "$1: 互換性のないポインタ型です．"

# T1033 $1: need cast to assign integer to pointer"
@@error_message[ :"T1033" ] = "$1: 整数型をポインタへ割り当てるためにはキャストが必要です．"

# T1034 $1: unsuitable string constant"
@@error_message[ :"T1034" ] = "$1: 不適当な文字列定数です．"

# T1035 $1: unsuitable initializer for pointer"
@@error_message[ :"T1035" ] = "$1: ポインタに対する不適当な初期化子です．"

# T1036 $1: unsuitable initializer for pointer"
@@error_message[ :"T1036" ] = "$1: ポインタに対する不適当な初期化子です．"

# T1037 $1: not number"
@@error_message[ :"T1037" ] = "$1: 数値ではありません"

# T1038 $1: initializer type mismatch. '$2' & '$3'"
@@error_message[ :"T1038" ] = "$1: 初期化子の型が一致しません. '$2' と '$3'"

# T1039 $1: struct tag mismatch $2 and $3"
@@error_message[ :"T1039" ] = "$1: 構造体タグが $2 と $3 で一致しません"

# T1040 $1 specified for void pointer type"
@@error_message[ :"T1040" ] = "$1 void ポインタ型に指定されました"

### gen_xml.rb
# T2001 fail to create XML file $1"
@@error_message[ :"T2001" ] = "XML ファイル $1 の生成に失敗しました"

# TEMPORAL set_definition_join: uninitialized array member"
@@error_message[ :"TEMPORAL" ] = "set_definition_join: 配列メンバが初期化されていません．"

# V1001 $1: unable for $2"
@@error_message[ :"V1001" ] = "$1: unable for $2"

# V1002 $1: cannot cast to bool (implicitly)"
@@error_message[ :"V1002" ] = "$1: ブール型への(暗黙的な)キャストを行うことができません．"

# V1003 $1: cannot cast to integer (implicitly)"
@@error_message[ :"V1003" ] = "$1: 整数型への(暗黙的な)キャストを行うことができません．"

# V1004 $1: cannot cast to float (implicitly)"
@@error_message[ :"V1004" ] = "$1: 浮動小数点型への(暗黙的な)キャストを行うことができません．"

# V1005 Cannot cast pointer to float"
@@error_message[ :"V1005" ] = "ポインタ型を浮動小数点型へキャストを行うことができません．"

# V1006 pointer value cannot cast to $1"
@@error_message[ :"V1006" ] = "ポインタ値は $1 へキャストを行うことができません．"

# V1007 convert pointer value to bool"
@@error_message[ :"V1007" ] = "ポインタ値をブール型へ変換しました．"

# V1008 convert pointer value to integer without cast"
@@error_message[ :"V1008" ] = "ポインタ値をキャスト無しで整数型へ変換しました．"

# V1009 / : divieded by zero"
@@error_message[ :"V1009" ] = "/ : ゼロ除算エラーです．"

# V1010 / : divieded by zero"
@@error_message[ :"V1010" ] = "/ : ゼロ除算エラーです．"

# V1011 % : divieded by zero"
@@error_message[ :"V1011" ] = "% : ゼロ除算エラーです．"

# V1012 % : divieded by zero"
@@error_message[ :"V1012" ] = "% : ゼロ除算エラーです．"

# V1013 integer value cannot cast to $1"
@@error_message[ :"V1013" ] = "整数値は $1 へキャストを行うことができません．"

# V1014 comparing bool value with \'$1\'"
@@error_message[ :"V1014" ] = "ブール値を \'$1\' と比較しました．"

# V1015 comparing bool value with \'$1\'"
@@error_message[ :"V1015" ] = "ブール値を \'$1\' と比較しました．"

# V1016 bool value cannot cast to $1"
@@error_message[ :"V1016" ] = "ブール値は $1 へキャストを行うことができません．"

# V1017 / : divieded by zero"
@@error_message[ :"V1017" ] = "/ : ゼロ除算エラーです．"

# V1018 % : divieded by zero"
@@error_message[ :"V1018" ] = "% : ゼロ除算エラーです．"

# V1019 floating value cannot cast to $1"
@@error_message[ :"V1019" ] = "浮動小数点値は $1 へのキャストを行うことができません．"

# V1020 convert floating value to bool without cast"
@@error_message[ :"V1020" ] = "浮動小数点値をキャスト無しでブール型へ変換しました．"

# V1021 convert floating value to integer without cast"
@@error_message[ :"V1021" ] = "浮動小数点型をキャスト無しで整数型へ変換しました．"

# V1022 string cannot cast to integer"
@@error_message[ :"V1022" ] = "文字列は整数型にキャストできません"

# V1023 string cannot cast to float"
@@error_message[ :"V1023" ] = "文字列は浮動小数点型にキャストできません"

# V1024 string cannot cast to pointer"
@@error_message[ :"V1024" ] = "文字列はポインタ型にキャストできません"

# V1025 string cannot cast to $1"
@@error_message[ :"V1025" ] = "文字列は $1 にキャストできません"

###
# warning メッセージ
@@warning_message = {}

### componentobj.rb
# W1001 \'$1\': unknown context type. usually specifiy task, non-task or any"
@@warning_message[ :"W1001" ] = "\'$1\': 不明なコンテキスト型です．通常 task, non-task または any を指定して下さい．"

# W1002 $1: non-active celltype has no entry port & factory"
@@warning_message[ :"W1002" ] = "$1: 非アクティブなセルタイプが受け口及びファクトリを持っていません．"

# W1003 $1 : require call port overridden in $2"
@@warning_message[ :"W1003" ] = "$1 : リクワイア呼び口が $2 でオーバーライドされました．"

# W1004 $1 : specified singleton but has no singleton in this celltype"
@@warning_message[ :"W1004" ] = "$1 : specified singleton but has no singleton in this celltype"

# W1005 $1 : idx_is_id is ineffective for composite celltype"
@@warning_message[ :"W1005" ] = "$1 : idx_is_id は複合セルタイプに対しては無効です．"

# W1006 $1 : only prototype, unused and undefined cell"
@@warning_message[ :"W1006" ] = "$1 : プロトタイプのみ，未使用，かつ未定義のセルです．"

# W1007 $1 : non-active cell has no entry join and no factory"
@@warning_message[ :"W1007" ] = "$1 : 非アクティブセルに受け口の結合及びファクトリがありません．"

# W1008 $1: reuse designation mismatch with previous import"
@@warning_message[ :"W1008" ] = "$1: 再使用の指示が以前のインポートのと整合しません．"

# "W1009 $1: fixed join entry port has multi join"
@@warning_message[ :"W1009" ] = "$1: 固定結合受け口が複数の結合を持っています．"

### types.rb
# W2001 signed int$1_t: obsolete. use int$2_t"
@@warning_message[ :"W2001" ] = "signed int$1_t: 推奨されません．int$2_t をお使いください．"

# W2002 unsinged int$1_t: obsolete. use uint$2_t"
@@warning_message[ :"W2002" ] = "unsinged int$1_t: 推奨されません．uint$2_t をお使いください．"

# W2003 $1: too large to cast to $2, clipped($3)"
@@warning_message[ :"W2003" ] = "$1: $2 へキャストするのに大きすぎます．切り詰めました($3)"

# W2004 $1: too small to cast to $2, clipped($3)"
@@warning_message[ :"W2004" ] = "$1: $2 へキャストするのに小さすぎます．切り詰めました($3)"

# W2005 $1: negative value for unsigned: convert to $2"
@@warning_message[ :"W2005" ] = "$1: 符号無し整数型に対して負数値です : $2 へ変換しました．"

### syntaxobj.rb
# W3001 $1: duplicate"
@@warning_message[ :"W3001" ] = "$1: 重複しています．"

# W3002 $1: this string might cause buffer over run"
@@warning_message[ :"W3002" ] = "$1: この文字列はバッファーオーバーランを引き起こすかもしれません．"

# W3003 $1 pointer level mismatch"
@@warning_message[ :"W3003" ] = "$1 ポインターレベルの不整合です．"

# W3004 $1 pointer type has returned. specify deviate or stop return pointer"
@@error_message[ :"W3004" ] = "$1 ポインタ型が返されました。deviate を指定するかポインタを返すのを止めてください"

# W3005 '$1' size_is always lower than max. max is ignored"
@@error_message[ :"W3005" ] = "'$1' size_is は常に max より小さい. max は無視されます"

### bnf.y.rb
# W5001 bool: obsolete type. use bool_t"
@@warning_message[ :"W5001" ] = "bool: 推奨されないデータ型です．bool_t をお使いください．"

# W5002 float: obsolete type. use float32_t"
@@warning_message[ :"W5002" ] = "float: 推奨されないデータ型です．float32_t をお使いください．"

# W5003 double: obsolete type. use double64_t"
@@warning_message[ :"W5003" ] = "double: 推奨されないデータ型です．double64_t をお使いください．"

# W5004 char: obsolete type. use char_t"
@@warning_message[ :"W5004" ] = "char: 推奨されないデータ型です．char_t をお使いください．"

# W5005 int8: obsolete. use int8_t"
@@warning_message[ :"W5005" ] = "int8: 推奨されません．int8_t をお使いください．"

# W5006 int16: obsolete. use int16_t"
@@warning_message[ :"W5006" ] = "int16: 推奨されません．int16_t をお使いください．"

# W5007 int32: obsolete. use int32_t"
@@warning_message[ :"W5007" ] = "int32: 推奨されません．int32_t をお使いください．"

# W5008 int64: obsolete. use int64_t"
@@warning_message[ :"W5008" ] = "int64: 推奨されません．int64_t をお使いください．"

# W5009 int64: obsolete. use int64_t"
@@warning_message[ :"W5009" ] = "int64: 推奨されません．int64_t をお使いください．"

# W5010 need 'void' for no parameter"
@@warning_message[ :"W5010" ] = "パラメータが存在しない場合 'void' が必要です"

# W5011 need 'void' for no parameter"
@@warning_message[ :"W5011" ] = "パラメータが存在しない場合 'void' が必要です"

### C_parser.y.rb
# W6001 need 'void' for no parameter"
@@warning_message[ :"W6001" ] = "パラメータが存在しない場合 'void' が必要です"

# W6002 need 'void' for no parameter"
@@warning_message[ :"W6002" ] = "パラメータが存在しない場合 'void' が必要です"

# W6003 need 'void' for no parameter"
@@warning_message[ :"W6003" ] = "パラメータが存在しない場合 'void' が必要です"

# W6004 need 'void' for no parameter"
@@warning_message[ :"W6004" ] = "パラメータが存在しない場合 'void' が必要です"

###
# info メッセージ
@@info_message = {}

end
