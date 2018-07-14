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
#   $Id: messages_console_en_US.rb 2633 2017-04-02 06:02:05Z okuma-top $
#++


# Console Messages for en_US 
class TECSMsg

###
# エラーメッセージ
@@error_message = {}

### C_parser.y.rb
# B1001 need parameter name"
@@error_message[ :"B1001" ] = "need parameter name"

# B1002 while open or reading \'$1\'"
@@error_message[ :"B1002" ] = "while open or reading \'$1\'"

# B1003 Unexpected EOF"
@@error_message[ :"B1003" ] = "Unexpected EOF"

# B1004 syntax error near \'$1\'"
@@error_message[ :"B1004" ] = "syntax error near \'$1\'"

### ctypes.rb
# C1001 $1: mismatch, suitable for integer types"
@@error_message[ :"C1001" ] = "$1: mismatch, suitable for integer types"

# C1002 $1 not compatible with previous one $2"
@@error_message[ :"C1002" ] = "$1 not compatible with previous one $2"

# C1003 $1 & $2 incompatible  (\'long double\' not support)"
@@error_message[ :"C1003" ] = "$1 & $2 incompatible  (\'long double\' not support)"

# C1004 $1: qualifier respecified. previous one: $2"
@@error_message[ :"C1004" ] = "$1: qualifier respecified. previous one: $2"

### expression.rb
# E1001 $1: not found"
@@error_message[ :"E1001" ] = "$1: not found"

# E1002 $1: not constant (port)"
@@error_message[ :"E1002" ] = "$1: not constant (port)"

# E1003 $1: not constant"
@@error_message[ :"E1003" ] = "$1: not constant"

# E1004 cannot evaluate \'[]\' operator"
@@error_message[ :"E1004" ] = "cannot evaluate \'[]\' operator"

# E1005 cannot evaluate \'.\' operator"
@@error_message[ :"E1005" ] = "cannot evaluate \'.\' operator"

# E1006 cannot evaluate \'->\' operator"
@@error_message[ :"E1006" ] = "cannot evaluate \'->\' operator"

# E1007 cannot evaluate \'sizeof\' operator"
@@error_message[ :"E1007" ] = "cannot evaluate \'sizeof\' operator"

# E1008 cannot evaluate \'sizeof\' operator"
@@error_message[ :"E1008" ] = "cannot evaluate \'sizeof\' operator"

# E1009 cannot evaluate \'&\' operator"
@@error_message[ :"E1009" ] = "cannot evaluate \'&\' operator"

# E1010 cannot evaluate \'*\' operator"
@@error_message[ :"E1010" ] = "cannot evaluate \'*\' operator"

# E1011 cannot evaluate unary + for $1"
@@error_message[ :"E1011" ] = "cannot evaluate unary + for $1"

# E1012 $1: not found in parameter list"
@@error_message[ :"E1012" ] = "$1: not found in parameter list"

# E1013 \'*\': operand is not pointer value"
@@error_message[ :"E1013" ] = "\'*\': operand is not pointer value"

# E1014 $1: elements_get_type: sorry not supported"
@@error_message[ :"E1014" ] = "$1: elements_get_type: sorry not supported"

# E1015 \'$1\': direction mismatch for $2, $3 required"
@@error_message[ :"E1015" ] = "\'$1\': direction mismatch for $2, $3 required"

# E1016 $1: elements_check_dir_for_param: sorry not supported"
@@error_message[ :"E1016" ] = "$1: elements_check_dir_for_param: sorry not supported"

# E1017 $1: rhs not \'Cell.ePort\' form"
@@error_message[ :"E1017" ] = "$1: rhs not \'Cell.ePort\' form"

# E1018 $1: namespace cannot be specified"
@@error_message[ :"E1018" ] = "$1: namespace cannot be specified"

# E1019 $1: rhs not in 'allocator_entry_port' form"
@@error_message[ :"E1019" ] = "$1: rhs not in 'allocator_entry_port' form"

# E1020 rhs not in 'call_port.func.param' form for $1_$2"
@@error_message[ :"E1020" ] = "rhs not in 'call_port.func.param' form for $1_$2"

### bnf.y.rb
# G1001 need specifier for \'$1\'"
@@error_message[ :"G1001" ] = "need specifier for \'$1\'"

# G1002 need parameter name"
@@error_message[ :"G1002" ] = "need parameter name"

# G1003 need parameter name"
@@error_message[ :"G1003" ] = "need parameter name"

# G1004 impossible array type 0"
@@error_message[ :"G1004" ] = "impossible array type 0"

# G1005 impossible array type 1"
@@error_message[ :"G1005" ] = "impossible array type 1"

# G1006 impossible array type 2"
@@error_message[ :"G1006" ] = "impossible array type 2"

# G1007 impossible array type 3"
@@error_message[ :"G1007" ] = "impossible array type 3"

# G1008 impossible function type"
@@error_message[ :"G1008" ] = "impossible function type"

# G1009 unexpected specifier"
@@error_message[ :"G1009" ] = "unexpected specifier"

# G1010 Not function"
@@error_message[ :"G1010" ] = "Not function"

# G1011 Not function"
@@error_message[ :"G1011" ] = "Not function"

# G1012 $1 : cannot put specifier here"
@@error_message[ :"G1012" ] = "$1 : cannot put specifier here"

# G1013 unexpected specifier"
@@error_message[ :"G1013" ] = "unexpected specifier"

# G1014 while open or reading \'$1\'"
@@error_message[ :"G1014" ] = "while open or reading \'$1\'"

# G1015 Unexpected EOF"
@@error_message[ :"G1015" ] = "Unexpected EOF"

# G1016 syntax error near \'$1\'"
@@error_message[ :"G1016" ] = "syntax error near \'$1\'"

### plugin.rb
# P1001 plugin arg: cannot find identifier in $1"
@@error_message[ :"P1001" ] = "plugin arg: cannot find identifier in $1"

# P1002 plugin arg: expecting \'=\' not \'$1\'"
@@error_message[ :"P1002" ] = "plugin arg: expecting \'=\' not \'$1\'"

# P1003 plugin arg: unexpected $1"
@@error_message[ :"P1003" ] = "plugin arg: unexpected $1"

# P1004 $1: unknown plugin argument\'s identifier\n  $2 are acceptable for RPCPlugin."
@@error_message[ :"P1004" ] = "$1: unknown plugin argument\'s identifier\n  $2 are acceptable for RPCPlugin."

### pluginModule.rb
# P2001 $1.rb : fail to load plugin"
@@error_message[ :"P2001" ] = "$1.rb : fail to load plugin"

# P2002 $1: not kind of $2"
@@error_message[ :"P2002" ] = "$1: not kind of $2"

# P2003 $1: load failed"
@@error_message[ :"P2003" ] = "$1: load failed"

# P2004 $1: open error \'$2\'"
@@error_message[ :"P2004" ] = "$1: open error \'$2\'"

# P2005 $1: plugin error in gen_through_cell_code "
@@error_message[ :"P2005" ] = "$1: plugin error in gen_through_cell_code "

# P2006 $1: close error \'$2\'"
@@error_message[ :"P2006" ] = "$1: close error \'$2\'"

# P2007 $1: fail to generate post code"
@@error_message[ :"P2007" ] = "$1: fail to generate post code"

### componentobj.rb
# S1001 context specifier duplicate"
@@error_message[ :"S1001" ] = "context specifier duplicate"

# S1002 \'$1\': unknown specifier for signature"
@@error_message[ :"S1002" ] = "\'$1\': unknown specifier for signature"

# S1003 $1: \'alloc\' 1st parameter neither [in] integer type nor [out] double pointer type
@@error_message[ :"S1003" ] = "$1: \'alloc\' 1st parameter neither [in] integer type nor [out] double pointer type"

# S1004 $1: \'alloc\' 2nd parameter not [in] double pointer"
@@error_message[ :"S1004" ] = "$1: \'alloc\' 2nd parameter not [in] double pointer"

# S1005 $1: \'alloc\' has no parameter, unsuitable for allocator signature"
@@error_message[ :"S1005" ] = "$1: \'alloc\' has no parameter, unsuitable for allocator signature"

# S1006 $1: \'dealloc\' 1st parameter not [in] pointer type"
@@error_message[ :"S1006" ] = "$1: \'dealloc\' 1st parameter not [in] pointer type"

# S1007 "
     # S1007 $1: \'dealloc\' cannot has 2nd parameter"
# @@error_message[ :"S1007" ] = "$1: \'dealloc\' cannot has 2nd parameter"

# S1008 $1: \'dealloc\' has no parameter, unsuitable for allocator signature"
@@error_message[ :"S1008" ] = "$1: \'dealloc\' has no parameter, unsuitable for allocator signature"

# S1009 $1: \'alloc\' function not found, unsuitable for allocator signature"
@@error_message[ :"S1009" ] = "$1: \'alloc\' function not found, unsuitable for allocator signature"

# S1010 $1: \'dealloc\' function not found, unsuitable for allocator signature"
@@error_message[ :"S1010" ] = "$1: \'dealloc\' function not found, unsuitable for allocator signature"

# S1011 $1: size_is specified for non-pointer type"
@@error_message[ :"S1011" ] = "$1: size_is specified for non-pointer type"

# S1012 $1: unsuitable initializer, need array initializer"
@@error_message[ :"S1012" ] = "$1: unsuitable initializer, need array initializer"

# S1013 $1: too many initializer, $2 for $3"
@@error_message[ :"S1013" ] = "$1: too many initializer, $2 for $3"

# S1014 generate specifier duplicate"
@@error_message[ :"S1014" ] = "generate specifier duplicate"

# S1015 $1 cannot be specified for composite"
@@error_message[ :"S1015" ] = "$1 cannot be specified for composite"

# S1016 $1 not found"
@@error_message[ :"S1016" ] = "$1 not found"

# S1017 $1 : neither celltype nor cell"
@@error_message[ :"S1017" ] = "$1 : neither celltype nor cell"

# S1018 $1 : not singleton cell"
@@error_message[ :"S1018" ] = "$1 : not singleton cell"

# S1019 \'$1\' : not entry port"
@@error_message[ :"S1019" ] = "\'$1\' : not entry port"

# S1020 \'$1\' : required port cannot be array"
@@error_message[ :"S1020" ] = "\'$1\' : required port cannot be array"

# S1021 $1 : require cannot have same signature with \'$2\'"
@@error_message[ :"S1021" ] = "$1 : require cannot have same signature with \'$2\'"

# S1022 $1.$2 : \'$3\' conflict function name in $4.$5"
@@error_message[ :"S1022" ] = "$1.$2 : \'$3\' conflict function name in $4.$5"

# S1023 $1: fail to new"
@@error_message[ :"S1023" ] = "$1: fail to new"

# S1024 $1: multiple cell for singleton celltype"
@@error_message[ :"S1024" ] = "$1: multiple cell for singleton celltype"

# S1025 not found reachable cell for require \'$1\' in celltype \'$2\'"
@@error_message[ :"S1025" ] = "not found reachable cell for require \'$1\' in celltype \'$2\'"

# S1026 required cell \'$1\' not reachable"
@@error_message[ :"S1026" ] = "required cell \'$1\' not reachable"

# S1027 \'$1\' celltype not found"
@@error_message[ :"S1027" ] = "\'$1\' celltype not found"

# S1028 \'$1\' not celltype"
@@error_message[ :"S1028" ] = "\'$1\' not celltype"

# S1029 $1 mismatch with previous one"
@@error_message[ :"S1029" ] = "$1 mismatch with previous one"

# S1030 $1: celltype mismatch with previous one"
@@error_message[ :"S1030" ] = "$1: celltype mismatch with previous one"

# S1031 $1 region \'$2\' mismatch  with previous one \'$3\'"
@@error_message[ :"S1031" ] = "$1 region \'$2\' mismatch  with previous one \'$3\'"

# S1032 $1: duplicate cell"
@@error_message[ :"S1032" ] = "$1: duplicate cell"

# S1033 rhs expression is not supported. Only attribute is permitted on current version."
@@error_message[ :"S1033" ] = "rhs expression is not supported. Only attribute is permitted on current version."

# S1034 $1 : cannot refer to $2\'s attribute here. Use \'composite.$3\' to refer to composite celltype\'s"
@@error_message[ :"S1034" ] = "$1 : cannot refer to $2\'s attribute here. Use \'composite.$3\' to refer to composite celltype\'s"

# S1035 composite : cannot specify out of composite celltype definition"
@@error_message[ :"S1035" ] = "composite : cannot specify out of composite celltype definition"

# S1036 $1 : cannot refer to $2\'s here. Use \'composite.$3\' to refer to composite celltype\'s"
@@error_message[ :"S1036" ] = "$1 : cannot refer to $2\'s here. Use \'composite.$3\' to refer to composite celltype\'s"

# S1037 $1: celltype plugin fail to new_cell"
@@error_message[ :"S1037" ] = "$1: celltype plugin fail to new_cell"

# S1038 $1.$2: self allocator not supported for array entry port"
@@error_message[ :"S1038" ] = "$1.$2: self allocator not supported for array entry port"

# S1039 \'$1\': unknown specifier for cell"
@@error_message[ :"S1039" ] = "\'$1\': unknown specifier for cell"

# S1040 array not supported for relay allocator"
@@error_message[ :"S1040" ] = "array not supported for relay allocator"

# S1041 \'$1_$2_$3\': not joined. cannot create internal join for relay allocator"
@@error_message[ :"S1041" ] = "\'$1_$2_$3\': not joined. cannot create internal join for relay allocator"

# S1042 call port \'$1\' not initialized in cell \'$2\'"
@@error_message[ :"S1042" ] = "call port \'$1\' not initialized in cell \'$2\'"

# S1043 call port \'$1\' not initialized in cell \'$2\'. this call port is created by tecsgen. check allocator specifier"
@@error_message[ :"S1043" ] = "call port \'$1\' not initialized in cell \'$2\'. this call port is created by tecsgen. check allocator specifier"

# S1044 $1: array initializer too many or few, $2 for $3"
@@error_message[ :"S1044" ] = "$1: array initializer too many or few, $2 for $3"

# S1045 $1[$2]: not initialized"
@@error_message[ :"S1045" ] = "$1[$2]: not initialized"

# S1046 $1[$2]: not initialized"
@@error_message[ :"S1046" ] = "$1[$2]: not initialized"

# S1047 size_is pointer cannot be exposed for composite attribute"
@@error_message[ :"S1047" ] = "size_is pointer cannot be exposed for composite attribute"

# S1048 $1: size_is specified for non-pointer type"
@@error_message[ :"S1048" ] = "$1: size_is specified for non-pointer type"

# S1049 $1: size_is arg not constant"
@@error_message[ :"S1049" ] = "$1: size_is arg not constant"

# S1050 unsuitable initializer, need array initializer"
@@error_message[ :"S1050" ] = "unsuitable initializer, need array initializer"

# S1051 too many initializer for array, $1 for $2"
@@error_message[ :"S1051" ] = "too many initializer for array, $1 for $2"

# S1052 attribute \'$1\' not initialized in cell \'$2\'"
@@error_message[ :"S1052" ] = "attribute \'$1\' not initialized in cell \'$2\'"

# S1053 $1 must be singleton. inner cell \'$2\' is singleton"
@@error_message[ :"S1053" ] = "$1 must be singleton. inner cell \'$2\' is singleton"

# S1054 $1 : specified active but has no active in this celltype"
@@error_message[ :"S1054" ] = "$1 : specified active but has no active in this celltype"

# S1055 $1 must be active. inner cell \'$2\' is active"
@@error_message[ :"S1055" ] = "$1 must be active. inner cell \'$2\' is active"

# S1056 $1 : cannot export, nothing designated"
@@error_message[ :"S1056" ] = "$1 : cannot export, nothing designated"

# S1057 $1 not found in $2"
@@error_message[ :"S1057" ] = "$1 not found in $2"

# S1058 \'$1\' : cannot export var"
@@error_message[ :"S1058" ] = "\'$1\' : cannot export var"

# S1059 \'$1\' : exporting attribute. write in cell or use \'=\' to export attribute"
@@error_message[ :"S1059" ] = "\'$1\' : exporting attribute. write in cell or use \'=\' to export attribute"

# S1060 \'$1\' : port type mismatch. $2 type is allowed here."
@@error_message[ :"S1060" ] = "\'$1\' : port type mismatch. $2 type is allowed here."

# S1061 \'$1\' : not defined"
@@error_message[ :"S1061" ] = "\'$1\' : not defined"

# S1062 $1 has no export definition"
@@error_message[ :"S1062" ] = "$1 has no export definition"

# S1063 $1 is port but previously defined as an attribute"
@@error_message[ :"S1063" ] = "$1 is port but previously defined as an attribute"

# S1064 $1 : type \'$2$3\' mismatch with pprevious definition\'$4$5\'"
@@error_message[ :"S1064" ] = "$1 : type \'$2$3\' mismatch with pprevious definition\'$4$5\'"

# S1065 $1 : port type $2 mismatch with previous definition $3"
@@error_message[ :"S1065" ] = "$1 : port type $2 mismatch with previous definition $3"

# S1066 $1 : signature \'$2\' mismatch with previous definition \'$3\'"
@@error_message[ :"S1066" ] = "$1 : signature \'$2\' mismatch with previous definition \'$3\'"

# S1067 $1 : array size mismatch with previous definition"
@@error_message[ :"S1067" ] = "$1 : array size mismatch with previous definition"

# S1068 $1 : optional specifier mismatch with previous definition"
@@error_message[ :"S1068" ] = "$1 : optional specifier mismatch with previous definition"

# S1069 $1 is an attribute but previously defined as a port"
@@error_message[ :"S1069" ] = "$1 is an attribute but previously defined as a port"

# S1070 $1: size_is pointer cannot be exposed for composite attribute"
@@error_message[ :"S1070" ] = "$1: size_is pointer cannot be exposed for composite attribute"

# S1071 $1 cannot be specified for composite"
@@error_message[ :"S1071" ] = "$1 cannot be specified for composite"

# S1072 $1: entry port: sizeless array not supported in current version"
@@error_message[ :"S1072" ] = "$1: entry port: sizeless array not supported in current version"

# S1073 Not constant expression $1"
@@error_message[ :"S1073" ] = "Not constant expression $1"

# S1074 Not Integer $1"
@@error_message[ :"S1074" ] = "Not Integer $1"

# S1075 \'$1\' signature not found"
@@error_message[ :"S1075" ] = "\'$1\' signature not found"

# S1076 \'$1\' not signature"
@@error_message[ :"S1076" ] = "\'$1\' not signature"

# S1077 inline: cannot be specified for call port"
@@error_message[ :"S1077" ] = "inline: cannot be specified for call port"

# S1078 optional: cannot be specified for entry port"
@@error_message[ :"S1078" ] = "optional: cannot be specified for entry port"

# S1079 allocator: cannot be specified for call port"
@@error_message[ :"S1079" ] = "allocator: cannot be specified for call port"

# S1080 duplicate allocator specifier"
@@error_message[ :"S1080" ] = "duplicate allocator specifier"

# S1081 self allocator not supported yet"
@@error_message[ :"S1081" ] = "self allocator not supported yet"

# S1082 function \'$1\' not found in signature"
@@error_message[ :"S1082" ] = "function \'$1\' not found in signature"

# S1083 \'$1\' not found in function \'$2\'"
@@error_message[ :"S1083" ] = "\'$1\' not found in function \'$2\'"

# S1084 \'$1\' in function \'$2\' is not send or receive"
@@error_message[ :"S1084" ] = "\'$1\' in function \'$2\' is not send or receive"

# S1085 duplicate allocator specifier for \'$1_$2\'"
@@error_message[ :"S1085" ] = "duplicate allocator specifier for \'$1_$2\'"

# S1086 rhs not call_port.func.param for $1_$2"
@@error_message[ :"S1086" ] = "rhs not call_port.func.param for $1_$2"

# S1087 function \'$1\' not found in signature \'$2\'"
@@error_message[ :"S1087" ] = "function \'$1\' not found in signature \'$2\'"

# S1088 \'$1\' not found in function \'$2\'"
@@error_message[ :"S1088" ] = "\'$1\' not found in function \'$2\'"

# S1089 relay allocator send/receive mismatch between $1.$2 and $3_$4.$5"
@@error_message[ :"S1089" ] = "relay allocator send/receive mismatch between $1.$2 and $3_$4.$5"

# S1090 \'$1\' in function \'$2\' is not send or receive"
@@error_message[ :"S1090" ] = "\'$1\' in function \'$2\' is not send or receive"

# S1091 call port \'$1\' not found in celltype $2"
@@error_message[ :"S1091" ] = "call port \'$1\' not found in celltype $2"

# S1092 \'$1\' not namespace"
@@error_message[ :"S1092" ] = "\'$1\' not namespace"

# S1093 $1 : undefined cell"
@@error_message[ :"S1093" ] = "$1 : undefined cell"

# S1094 $1: pointer is not constant. check \'const\'"
@@error_message[ :"S1094" ] = "$1: pointer is not constant. check \'const\'"

# S1095 $1: not constant"
@@error_message[ :"S1095" ] = "$1: not constant"

# S1096 $1: should be int, float, bool or pointer type"
@@error_message[ :"S1096" ] = "$1: should be int, float, bool or pointer type"

# S1097 $1: has no initializer"
@@error_message[ :"S1097" ] = "$1: has no initializer"

# S1098 $1: has unsuitable initializer"
@@error_message[ :"S1098" ] = "$1: has unsuitable initializer"

# S1099 array subscript not constant"
@@error_message[ :"S1099" ] = "array subscript not constant"

# S1100 $1: cannot initialize var"
@@error_message[ :"S1100" ] = "$1: cannot initialize var"

# S1101 \'$1\' cannot initialize entry port"
@@error_message[ :"S1101" ] = "\'$1\' cannot initialize entry port"

# S1102 $1: must specify array subscript here"
@@error_message[ :"S1102" ] = "$1: must specify array subscript here"

# S1103 $1: need array subscript"
@@error_message[ :"S1103" ] = "$1: need array subscript"

# S1104 $1: need array subscript number. ex. \'[0]\'"
@@error_message[ :"S1104" ] = "$1: need array subscript number. ex. \'[0]\'"

# S1105 $1: cannot specify array subscript here"
@@error_message[ :"S1105" ] = "$1: cannot specify array subscript here"

# S1106 $1: cannot specify array subscript number. use \'[]\'"
@@error_message[ :"S1106" ] = "$1: cannot specify array subscript number. use \'[]\'"

# S1107 to export port, use \'cCall => composite.cCall\'"
@@error_message[ :"S1107" ] = "to export port, use \'cCall => composite.cCall\'"

# S1108 $1: rhs not \'Cell.ePort\' form"
@@error_message[ :"S1108" ] = "$1: rhs not \'Cell.ePort\' form"

# S1109 \'$1\' not found"
@@error_message[ :"S1109" ] = "\'$1\' not found"

# S1110 \'$1\' not cell"
@@error_message[ :"S1110" ] = "\'$1\' not cell"

# S1111 \'$1\' not found"
@@error_message[ :"S1111" ] = "\'$1\' not found"

# S1112 \'$1\' not entry port"
@@error_message[ :"S1112" ] = "\'$1\' not entry port"

# S1113 \'$1\' signature mismatch"
@@error_message[ :"S1113" ] = "\'$1\' signature mismatch"

# S1114 \'$1\' should be array"
@@error_message[ :"S1114" ] = "\'$1\' should be array"

# S1115 $1[$2]: subscript out of range (< $3)"
@@error_message[ :"S1115" ] = "$1[$2]: subscript out of range (< $3)"

# S1116 \'$1\' entry port is not array"
@@error_message[ :"S1116" ] = "\'$1\' entry port is not array"

# S1117 \'$1\' not in celltype"
@@error_message[ :"S1117" ] = "\'$1\' not in celltype"

# S1118 $1: going out from region \'$2\' not permitted"
@@error_message[ :"S1118" ] = "$1: going out from region \'$2\' not permitted"

# S1119 $1: going from region \'$2\' to \'$3\' not permitted"
@@error_message[ :"S1119" ] = "$1: going from region \'$2\' to \'$3\' not permitted"

# S1120 $1: going in to region \'$2\' not permitted"
@@error_message[ :"S1120" ] = "$1: going in to region \'$2\' not permitted"

# S1121 \'$1\' in region \'$2\' cannot be directly joined $3 in  $4"
@@error_message[ :"S1121" ] = "\'$1\' in region \'$2\' cannot be directly joined $3 in  $4"

# S1122 $1 : not port: \'through\' can be specified only for port"
@@error_message[ :"S1122" ] = "$1 : not port: \'through\' can be specified only for port"

# S1123 $1 : not port: \'through\' can be specified only for port"
@@error_message[ :"S1123" ] = "$1 : not port: \'through\' can be specified only for port"

# S1124 $1: plugin function failed: \'get_through_entry_port_name\'"
@@error_message[ :"S1124" ] = "$1: plugin function failed: \'get_through_entry_port_name\'"

# S1125 $1: not generated cell \'$2\'"
@@error_message[ :"S1125" ] = "$1: not generated cell \'$2\'"

# S1126 $1: fail to new"
@@error_message[ :"S1126" ] = "$1: fail to new"

# S1127 \'$1\' duplicate"
@@error_message[ :"S1127" ] = "\'$1\' duplicate"

# S1128 \'$1\' inconsistent array definition"
@@error_message[ :"S1128" ] = "\'$1\' inconsistent array definition"

# S1129 \'$1\' redefinition of subscript $1"
@@error_message[ :"S1129" ] = "\'$1\' redefinition of subscript $1"

# S1130 \'$1\' inconsistent array definition"
@@error_message[ :"S1130" ] = "\'$1\' inconsistent array definition"

# S1131 \'$1.$2\' has duplicate initializer"
@@error_message[ :"S1131" ] = "\'$1.$2\' has duplicate initializer"

# S1132 $1: 1st parameter is not string(file name)"
@@error_message[ :"S1132" ] = "$1: 1st parameter is not string(file name)"

# S1133 $1: 2nd parameter is not string(fromat)"
@@error_message[ :"S1133" ] = "$1: 2nd parameter is not string(fromat)"

# S1134 $1: unknown factory function"
@@error_message[ :"S1134" ] = "$1: unknown factory function"

# S1135 celltype factory can\'t have parameter(s)"
@@error_message[ :"S1135" ] = "celltype factory can\'t have parameter(s)"

# S1136 \'$1\': not found"
@@error_message[ :"S1136" ] = "\'$1\': not found"

# S1137 \'$1\': not attribute"
@@error_message[ :"S1137" ] = "\'$1\': not attribute"

# S1138 internal error Factory.check_arg()"
@@error_message[ :"S1138" ] = "internal error Factory.check_arg()"

# S1139 $1: region path mismatch. previous path: $2"
@@error_message[ :"S1139" ] = "$1: region path mismatch. previous path: $2"

# S1140 $1: region specifier must place at first appearence"
@@error_message[ :"S1140" ] = "$1: region specifier must place at first appearence"

# S1141 $1 duplication, previous one : $2"
@@error_message[ :"S1141" ] = "$1 duplication, previous one : $2"

# S1142 $1 not found in search path"
@@error_message[ :"S1142" ] = "$1 not found in search path"

# S1143 import_C: arg2: mismatch with previous one"
@@error_message[ :"S1143" ] = "import_C: arg2: mismatch with previous one"

# S1144 $1: temporary C source: open error"
@@error_message[ :"S1144" ] = "$1: temporary C source: open error"

# S1145 $1: temporary C source: writing error"
@@error_message[ :"S1145" ] = "$1: temporary C source: writing error"

# S1146 $1: error occured while CPP"
@@error_message[ :"S1146" ] = "$1: error occured while CPP"

# S1147 $1: popen for CPP failed"
@@error_message[ :"S1147" ] = "$1: popen for CPP failed"

# S1148 $1 not found in search path"
@@error_message[ :"S1148" ] = "$1 not found in search path"

# S1149 $1 not signature"
@@error_message[ :"S1149" ] = "$1 not signature"

# S1150 $1: fail to new"
@@error_message[ :"S1150" ] = "$1: fail to new"

# S1151 $1: not namespace"
@@error_message[ :"S1151" ] = "$1: not namespace"

# S1152 $1: call port cannot have fixed join"
@@error_message[ :"S1152" ] = "$1: call port cannot have fixed join"

# "S1153 $1: cannot be entry port array for fixed join port"
@@error_message[ :"S1153" ] = "$1: cannot be entry port array for fixed join port"

# "S1154 $1: must be singleton celltype for fixed join"
@@error_message[ :"S1154" ] = "$1: must be singleton celltype for fixed join"

# "S1155 $1: not celltype or not found"
@@error_message[ :"S1155" ] = "$1: not celltype or not found"

# "S1156 $1: not call port or not found"
@@error_message[ :"S1156" ] = "$1: not call port or not found"

# "S1157 $1: sized array or not array"
@@error_message[ :"S1157" ] = "$1: sized array or not array"

# "S1158 $1: singleton cell not found for fixed join"
@@error_message[ :"S1158" ] = "$1: singleton cell not found for fixed join"

# S1159 $1: non-size_is pointer cannot be initialized with array initializer"
@@error_message[ :"S1159" ] = "$1: non-size_is pointer cannot be initialized with array initializer"

# S1160 $1 must be constant for id"
@@error_message[ :"S1160" ] = "$1 must be constant for id"

# S1161 $1 must be constant for id"
@@error_message[ :"S1161" ] = "$1 must be constant for id"

# S1162 $1: id cannot be 0"
@@error_message[ :"S1162" ] = "$1: id cannot be 0"

# S1163 generate specifier duplicate"
@@error_message[ :"S1163" ] = "generate specifier duplicate"

# S1164 '$1' set_specified_id: id not positive integer '$2'"
@@error_message[ :"S1164" ] = "'$1' set_specified_id: id not positive integer '$2'"

# S1165 '$1' set_specified_id: id duplicate"
@@error_message[ :"S1165" ] = "'$1' set_specified_id: id duplicate"

# S1166 $1: fail to new"
@@error_message[ :"S1166" ] = "$1: fail to new"

# S1167 \'$1\': relay mismatch \'$2\'"
@@error_message[ :"S1167" ] = "\'$1\': relay mismatch \'$2\'"

# S1168 too many initializer for array, $1 for $2"
@@error_message[ :"S1168" ] = "too many initializer for array, $1 for $2"

# S1169 $1: non-size_is pointer cannot be initialized with array initializer"
@@error_message[ :"S1169" ] = "$1: non-size_is pointer cannot be initialized with array initializer"

# S1170 \'$1\' has size_is but export attr \'$2\' doesn't have"
@@error_message[ :"S1170" ] = "\'$1\' has size_is but export attr \'$2\' doesn't have"

# S1171 \'$1\' size_is argument of \'$2\' not exported"
@@error_message[ :"S1171" ] = "\'$1\' size_is argument of \'$2\' not exported"

# S1172 \'$1\' size_is argument mismatch with exporting one \'$2\'"
@@error_message[ :"S1172" ] = "\'$1\' size_is argument mismatch with exporting one \'$2\'"

# S1173 $1: allocator mismatch from $2's allocator"
@@error_message[ :"S1173" ] = "$1: allocator mismatch from $2's allocator"

# S1174 $1 not suitable for lhs, suitable lhs: 'func.param'"
@@error_message[ :"S1174" ] = "$1 not suitable for lhs, suitable lhs: 'func.param'"

# S1175 $1 not found or not allocator entry port for $2"
@@error_message[ :"S1175" ] = "$1 not found or not allocator entry port for $2"

# S1176 rhs not in 'call_port.func.param' form for $1_$2"
@@error_message[ :"S1176" ] = "rhs not in 'call_port.func.param' form for $1_$2"

# S1177 cannot specify 'through' in composite in current version"
@@error_message[ :"S1177" ] = "cannot specify 'through' in composite in current version"

# S1178 $1 region type specifier duplicate, previous $2"
@@error_message[ :"S1178" ] = "$1 region type specifier duplicate, previous $2"

### syntaxobj.rb
# S2001 \'$1\' duplicate $2"
@@error_message[ :"S2001" ] = "\'$1\' duplicate $2"

# S2002 $1: $2"
@@error_message[ :"S2002" ] = "$1: $2"

# S2003 $1: $2 cannot have initializer"
@@error_message[ :"S2003" ] = "$1: $2 cannot have initializer"

# S2004 $1: array subscript must be specified or omit"
@@error_message[ :"S2004" ] = "$1: array subscript must be specified or omit"

# S2005 $1: array subscript must be specified"
@@error_message[ :"S2005" ] = "$1: array subscript must be specified"

# S2006 \'$1\' function"
@@error_message[ :"S2006" ] = "\'$1\' function"

# S2007 \'$1\' $2"
@@error_message[ :"S2007" ] = "\'$1\' $2"

# S2008 $1: inconsitent with previous one"
@@error_message[ :"S2008" ] = "$1: inconsitent with previous one"

# S2009 $1: not found or not signature"
@@error_message[ :"S2009" ] = "$1: not found or not signature"

# S2010 $1: not allocator signature"
@@error_message[ :"S2010" ] = "$1: not allocator signature"

# S2011 size_is duplicate"
@@error_message[ :"S2011" ] = "size_is duplicate"

# S2012 count_is duplicate"
@@error_message[ :"S2012" ] = "count_is duplicate"

# S2013 string duplicate"
@@error_message[ :"S2013" ] = "string duplicate"

# S2014 $1 need pointer or more pointer"
@@error_message[ :"S2014" ] = "$1 need pointer or more pointer"

# S2015 $1 must be const for \'in\' parameter $2"
@@error_message[ :"S2015" ] = "'$1' must be const for \'in\' parameter $2"

# S2016 $1 can not be const for $2 parameter"
@@error_message[ :"S2016" ] = "'$1' can not be const for $2 parameter"

# S2017 size_is argument is not integer type"
@@error_message[ :"S2017" ] = "size_is argument is not integer type"

# S2018 \'$1\' size_is parameter not integer"
@@error_message[ :"S2018" ] = "\'$1\' size_is parameter not integer"

# S2019 \'$1\' size_is parameter negative or zero"
@@error_message[ :"S2019" ] = "\'$1\' size_is parameter negative or zero"

# S2020 count_is argument is not integer type"
@@error_message[ :"S2020" ] = "count_is argument is not integer type"

# S2021 \'$1\' count_is parameter not integer"
@@error_message[ :"S2021" ] = "\'$1\' count_is parameter not integer"

# S2022 \'$1\' count_is parameter negative or zero"
@@error_message[ :"S2022" ] = "\'$1\' count_is parameter negative or zero"

# S2023 string argument is not integer type"
@@error_message[ :"S2023" ] = "string argument is not integer type"

# S2024 \'$1\' string parameter not integer"
@@error_message[ :"S2024" ] = "\'$1\' string parameter not integer"

# S2025 \'$1\' string parameter negative or zero"
@@error_message[ :"S2025" ] = "\'$1\' string parameter negative or zero"

# S2026 '$1' nullable specified for non-pointer type"
@@error_message[ :"S2026" ] = "'$1' nullable specified for non-pointer type"

# S2027 '$1' parameter cannot be void type"
@@error_message[ :"S2027" ] = "'$1' parameter cannot be void type"

# S2028 '$1' max (size_is 2nd parameter) not constant"
@@error_message[ :"S2028" ] = "'$1' max (size_is 2nd parameter) not constant"

# S2029 '$1' max (size_is 2nd parameter) negative or zero, or not integer"
@@error_message[ :"S2029" ] = "'$1' max (size_is 2nd parameter) negative or zero, or not integer"

# S2030 '$1' both size_is and max are const. size_is larger than max"
@@error_message[ :"S2030" ] = "'$1' both size_is and max are const. size_is larger than max"

### optimize.rb
# S3001 $1: id too large $2 (max=$3)"
@@error_message[ :"S3001" ] = "$1: id too large $2 (max=$3)"

# S3002 $1: id too large $2 (max=$3)"
@@error_message[ :"S3002" ] = "$1: id too large $2 (max=$3)"

# S3003 $1: id number '$2' conflict with $3"
@@error_message[ :"S3003" ] = "$1: id number '$2' conflict with $3"

### types.rb
# T1001 const duplicate"
@@error_message[ :"T1001" ] = "const duplicate"

# T1002 volatile duplicate"
@@error_message[ :"T1002" ] = "volatile duplicate"

# T1003 $1: unsuitable specifier for $2"
@@error_message[ :"T1003" ] = "$1: unsuitable specifier for $2"

# T1004 cannot cast to $1"
@@error_message[ :"T1004" ] = "cannot cast to $1"

# T1005 \'$1\' not defined"
@@error_message[ :"T1005" ] = "\'$1\' not defined"

# T1006 \'$1\' not type name. expecting type name here"
@@error_message[ :"T1006" ] = "\'$1\' not type name. expecting type name here"

# T1007 $1: void type variable cannot have initializer"
@@error_message[ :"T1007" ] = "$1: void type variable cannot have initializer"

# T1008 ambigous signed or unsigned"
@@error_message[ :"T1008" ] = "ambigous signed or unsigned"

# T1009 $1: $2: not integer"
@@error_message[ :"T1009" ] = "$1: $2: not integer"

# T1010 $1: initializer is not constant"
@@error_message[ :"T1010" ] = "$1: initializer is not constant"

# T1011 $1: need cast to assign float to integer"
@@error_message[ :"T1011" ] = "$1: need cast to assign float to integer"

# T1012 $1: $2: not integer"
@@error_message[ :"T1012" ] = "$1: $2: not integer"

# T1013 $1: too large (max=$2)"
@@error_message[ :"T1013" ] = "$1: too large (max=$2)"

# T1014 $1: too large negative value (min=-$2)"
@@error_message[ :"T1014" ] = "$1: too large negative value (min=-$2)"

# T1015 $1: negative value for unsigned"
@@error_message[ :"T1015" ] = "$1: negative value for unsigned"

# T1016 $1: too large (max=$2)"
@@error_message[ :"T1016" ] = "$1: too large (max=$2)"

# T1017 $1: unsuitable initializer for scalar type"
@@error_message[ :"T1017" ] = "$1: unsuitable initializer for scalar type"

# T1018 $1: $2: not number"
@@error_message[ :"T1018" ] = "$1: $2: not number"

# T1019 $1: initializer is not constant"
@@error_message[ :"T1019" ] = "$1: initializer is not constant"

# T1020 $1: unsuitable initializer for scalar type"
@@error_message[ :"T1020" ] = "$1: unsuitable initializer for scalar type"

# T1021 \'$1\': struct not defined"
@@error_message[ :"T1021" ] = "\'$1\': struct not defined"

# T1022 struct $1: not defined"
@@error_message[ :"T1022" ] = "struct $1: not defined"

# T1023 struct $1: not defined"
@@error_message[ :"T1023" ] = "struct $1: not defined"

# T1024 $1: unsuitable initializer for struct"
@@error_message[ :"T1024" ] = "$1: unsuitable initializer for struct"

# T1025 size_is argument is not integer type"
@@error_message[ :"T1025" ] = "size_is argument is not integer type"

# T1026 count_is argument is not integer type"
@@error_message[ :"T1026" ] = "count_is argument is not integer type"

# T1027 string argument is not integer type"
@@error_message[ :"T1027" ] = "string argument is not integer type"

# T1028 $1: cannot initialize function pointer"
@@error_message[ :"T1028" ] = "$1: cannot initialize function pointer"

# T1029 oneway function cannot return type \'$1$2\', \'void\' or \'ER\' is permitted"
@@error_message[ :"T1029" ] = "oneway function cannot return type \'$1$2\', \'void\' or \'ER\' is permitted"

# T1030 oneway function cannot have $1 parameter for \'$2\'"
@@error_message[ :"T1030" ] = "oneway function cannot have $1 parameter for \'$2\'"

# T1031 $1: unsuitable initializer for array"
@@error_message[ :"T1031" ] = "$1: unsuitable initializer for array"

# T1032 $1: incompatible pointer type"
@@error_message[ :"T1032" ] = "$1: incompatible pointer type"

# T1033 $1: need cast to assign integer to pointer"
@@error_message[ :"T1033" ] = "$1: need cast to assign integer to pointer"

# T1034 $1: unsuitable string constant"
@@error_message[ :"T1034" ] = "$1: unsuitable string constant"

# T1035 $1: unsuitable initializer for pointer"
@@error_message[ :"T1035" ] = "$1: unsuitable initializer for pointer"

# T1036 $1: unsuitable initializer for pointer"
@@error_message[ :"T1036" ] = "$1: unsuitable initializer for pointer"

# T1037 $1: not number"
@@error_message[ :"T1037" ] = "$1: not number"

# T1038 $1: initializer type mismatch. '$2' & '$3'"
@@error_message[ :"T1038" ] = "$1: initializer type mismatch. '$2' & '$3'"

# T1039 $1: struct tag mismatch $2 and $3"
@@error_message[ :"T1039" ] = "$1: struct tag mismatch $2 and $3"

# T1040 $1 specified for void pointer type"
@@error_message[ :"T1040" ] = "$1 specified for void pointer type"

### gen_xml.rb
# T2001 fail to create XML file $1"
@@error_message[ :"T2001" ] = "fail to create XML file $1"

# TEMPORAL set_definition_join: uninitialized array member"
@@error_message[ :"TEMPORAL" ] = "set_definition_join: uninitialized array member"

# V1001 $1: unable for $2"
@@error_message[ :"V1001" ] = "$1: unable for $2"

# V1002 $1: cannot cast to bool (implicitly)"
@@error_message[ :"V1002" ] = "$1: cannot cast to bool (implicitly)"

# V1003 $1: cannot cast to integer (implicitly)"
@@error_message[ :"V1003" ] = "$1: cannot cast to integer (implicitly)"

# V1004 $1: cannot cast to float (implicitly)"
@@error_message[ :"V1004" ] = "$1: cannot cast to float (implicitly)"

# V1005 Cannot cast pointer to float"
@@error_message[ :"V1005" ] = "Cannot cast pointer to float"

# V1006 pointer value cannot cast to $1"
@@error_message[ :"V1006" ] = "pointer value cannot cast to $1"

# V1007 convert pointer value to bool"
@@error_message[ :"V1007" ] = "convert pointer value to bool"

# V1008 convert pointer value to integer without cast"
@@error_message[ :"V1008" ] = "convert pointer value to integer without cast"

# V1009 / : divieded by zero"
@@error_message[ :"V1009" ] = "/ : divieded by zero"

# V1010 / : divieded by zero"
@@error_message[ :"V1010" ] = "/ : divieded by zero"

# V1011 % : divieded by zero"
@@error_message[ :"V1011" ] = "% : divieded by zero"

# V1012 % : divieded by zero"
@@error_message[ :"V1012" ] = "% : divieded by zero"

# V1013 integer value cannot cast to $1"
@@error_message[ :"V1013" ] = "integer value cannot cast to $1"

# V1014 comparing bool value with \'$1\'"
@@error_message[ :"V1014" ] = "comparing bool value with \'$1\'"

# V1015 comparing bool value with \'$1\'"
@@error_message[ :"V1015" ] = "comparing bool value with \'$1\'"

# V1016 bool value cannot cast to $1"
@@error_message[ :"V1016" ] = "bool value cannot cast to $1"

# V1017 / : divieded by zero"
@@error_message[ :"V1017" ] = "/ : divieded by zero"

# V1018 % : divieded by zero"
@@error_message[ :"V1018" ] = "% : divieded by zero"

# V1019 floating value cannot cast to $1"
@@error_message[ :"V1019" ] = "floating value cannot cast to $1"

# V1020 convert floating value to bool without cast"
@@error_message[ :"V1020" ] = "convert floating value to bool without cast"

# V1021 convert floating value to integer without cast"
@@error_message[ :"V1021" ] = "convert floating value to integer without cast"

# V1022 string cannot cast to integer"
@@error_message[ :"V1022" ] = "string cannot cast to integer"

# V1023 string cannot cast to float"
@@error_message[ :"V1023" ] = "string cannot cast to float"

# V1024 string cannot cast to pointer"
@@error_message[ :"V1024" ] = "string cannot cast to pointer"

# V1025 string cannot cast to $1"
@@error_message[ :"V1025" ] = "string cannot cast to $1"

###
# warning メッセージ
@@warning_message = {}

### componentobj.rb
# W1001 \'$1\': unknown context type. usually specifiy task, non-task or any"
@@warning_message[ :"W1001" ] = "\'$1\': unknown context type. usually specifiy task, non-task or any"

# W1002 $1: non-active celltype has no entry port & factory"
@@warning_message[ :"W1002" ] = "$1: non-active celltype has no entry port & factory"

# W1003 $1 : require call port overridden in $2"
@@warning_message[ :"W1003" ] = "$1 : require call port overridden in $2"

# W1004 $1 : specified singleton but has no singleton in this celltype"
@@warning_message[ :"W1004" ] = "$1 : specified singleton but has no singleton in this celltype"

# W1005 $1 : idx_is_id is ineffective for composite celltype"
@@warning_message[ :"W1005" ] = "$1 : idx_is_id is ineffective for composite celltype"

# W1006 $1 : only prototype, unused and undefined cell"
@@warning_message[ :"W1006" ] = "$1 : only prototype, unused and undefined cell"

# W1007 $1 : non-active cell has no entry join and no factory"
@@warning_message[ :"W1007" ] = "$1 : non-active cell has no entry join and no factory"

# W1008 $1: reuse designation mismatch with previous import"
@@warning_message[ :"W1008" ] = "$1: reuse designation mismatch with previous import"

# "W1009 $1: fixed join entry port has multi join"
@@warning_message[ :"W1009" ] = "$1: fixed join entry port has multi join"

### types.rb
# W2001 signed int$1_t: obsolete. use int$2_t"
@@warning_message[ :"W2001" ] = "signed int$1_t: obsolete. use int$2_t"

# W2002 unsinged int$1_t: obsolete. use uint$2_t"
@@warning_message[ :"W2002" ] = "unsinged int$1_t: obsolete. use uint$2_t"

# W2003 $1: too large to cast to $2, clipped($3)"
@@warning_message[ :"W2003" ] = "$1: too large to cast to $2, clipped($3)"

# W2004 $1: too small to cast to $2, clipped($3)"
@@warning_message[ :"W2004" ] = "$1: too small to cast to $2, clipped($3)"

# W2005 $1: negative value for unsigned: convert to $2"
@@warning_message[ :"W2005" ] = "$1: negative value for unsigned: convert to $2"

### syntaxobj.rb
# W3001 $1: duplicate"
@@warning_message[ :"W3001" ] = "$1: duplicate"

# W3002 $1: this string might cause buffer over run"
@@warning_message[ :"W3002" ] = "$1: this string might cause buffer over run"

# W3003 $1 pointer level mismatch"
@@warning_message[ :"W3003" ] = "$1 pointer level mismatch"

# W3004 $1 pointer type has returned. specify deviate or stop return pointer"
@@error_message[ :"W3004" ] = "$1 pointer type has returned. specify deviate or stop return pointer"

# W3005 '$1' size_is always lower than max. max is ignored"
@@error_message[ :"W3005" ] = "'$1' size_is always lower than max. max is ignored"

### bnf.y.rb
# W5001 bool: obsolete type. use bool_t"
@@warning_message[ :"W5001" ] = "bool: obsolete type. use bool_t"

# W5002 float: obsolete type. use float32_t"
@@warning_message[ :"W5002" ] = "float: obsolete type. use float32_t"

# W5003 double: obsolete type. use double64_t"
@@warning_message[ :"W5003" ] = "double: obsolete type. use double64_t"

# W5004 char: obsolete type. use char_t"
@@warning_message[ :"W5004" ] = "char: obsolete type. use char_t"

# W5005 int8: obsolete. use int8_t"
@@warning_message[ :"W5005" ] = "int8: obsolete. use int8_t"

# W5006 int16: obsolete. use int16_t"
@@warning_message[ :"W5006" ] = "int16: obsolete. use int16_t"

# W5007 int32: obsolete. use int32_t"
@@warning_message[ :"W5007" ] = "int32: obsolete. use int32_t"

# W5008 int64: obsolete. use int64_t"
@@warning_message[ :"W5008" ] = "int64: obsolete. use int64_t"

# W5009 int64: obsolete. use int64_t"
@@warning_message[ :"W5009" ] = "int64: obsolete. use int64_t"

# W5010 need 'void' for no parameter"
@@warning_message[ :"W5010" ] = "need 'void' for no parameter"

# W5011 need 'void' for no parameter"
@@warning_message[ :"W5011" ] = "need 'void' for no parameter"

### C_parser.y.rb
# W6001 need 'void' for no parameter"
@@warning_message[ :"W6001" ] = "need 'void' for no parameter"

# W6002 need 'void' for no parameter"
@@warning_message[ :"W6002" ] = "need 'void' for no parameter"

# W6003 need 'void' for no parameter"
@@warning_message[ :"W6003" ] = "need 'void' for no parameter"

# W6004 need 'void' for no parameter"
@@warning_message[ :"W6004" ] = "need 'void' for no parameter"

###
# info メッセージ
@@info_message = {}

end
