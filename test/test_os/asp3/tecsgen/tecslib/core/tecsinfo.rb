# -*- coding: utf-8 -*-
#
#  TECS Generator
#      Generator for TOPPERS Embedded Component System
#  
#   Copyright (C) 2017 by TOPPERS Project
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
#   $Id: tecsinfo.rb 2663 2017-07-08 23:09:53Z okuma-top $
#++

# TECS 情報セルの生成
module TECSInfo
  # region は Link root のこと
  def self.print_info f, region
    # p "region: "+ region.get_name.to_s
    nest = region.gen_region_str_pre f
    indent0 = "    " * nest
    indent = "    " * ( nest + 1 )
    f.print <<EOT
#{indent0}region rTECSInfo {
EOT
    # mikan 全部生成するのではなく、region 下のセルのセルタイプと、そこから参照されるシグニチャ、セルタイプに限定して出力する．
    # しかし、意味解析後に出力するため、これは容易ではない．最適化とコード生成は、リンクルートごとに行われる．
    Namespace.print_info f, indent
    region.get_link_root.print_info f, indent

    f.print "\n#{indent}/*** TYPE information cell ***/\n"
    Type.print_info_post f, indent

    f.print "\n#{indent}/*** TECS information cell ***/\n"
    f.print <<EOT
#{indent}cell nTECSInfo::tTECSInfoSub TECSInfoSub {
#{indent}    cNamespaceInfo = _RootNamespaceInfo.eNamespaceInfo;
#{indent}    cRegionInfo    = _LinkRootRegionInfo.eRegionInfo;
#{indent}} /* TECSInfoSub */;
#{indent0}}; /* rTECSInfo */
EOT
    region.gen_region_str_post f
  end
end

class Namespace
  # RootRegion と LinkRegion は同じ Region クラスのオブジェクトである
  # 子ネームスペースは Namespace クラスの、子リージョンは Region クラスのオブジェクトである
  # これは、意味解析段階で呼び出されるため、リンクユニットごとに出しわけることができない
  # 出しわけるには、2パスにする必要がある
  def print_info_ns_sub f, indent
    if @name == "::" then
      name = "_Root"
    else
      name = @global_name
    end
    f.print "\n#{indent}/*** #{get_namespace_path} namespace information cell ***/\n"
    f.print <<EOT
#{indent}cell nTECSInfo::tNamespaceInfo #{name}NamespaceInfo{
#{indent}    name = "#{@name}";
EOT
    if @signature_list.length > 0 then
      f.print "\n#{indent}    /* SIGNATURE info */\n"
    end
    @signature_list.each{ |sig|
      f.print <<EOT
#{indent}    cSignatureInfo[] = #{sig.get_global_name}SignatureInfo.eSignatureInfo;
EOT
    }
    if @celltype_list.length > 0 then
      f.print "\n#{indent}    /* CELLTYPE info */\n"
    end
    @celltype_list.each{ |ct|
      if ct.get_cell_list.length > 0 then
        f.print <<EOT
#{indent}    cCelltypeInfo[] = #{ct.get_global_name}CelltypeInfo.eCelltypeInfo;
EOT
      end
    }
    if @namespace_list.length > 0 then
      f.print "\n#{indent}    /* NAMESPACE info */\n"
    end
    @namespace_list.each{ |ns|
      if ns.instance_of? Namespace then
        f.print <<EOT
#{indent}    cNamespaceInfo[] = #{ns.get_global_name}NamespaceInfo.eNamespaceInfo;
EOT
      end
    }
    f.print <<EOT
#{indent}};   /* cell nTECSInfo::tNamespaceInfo #{name}NamespaceInfo */
EOT
  end

  def print_info_ns f, indent
    # p "print_info: #{self.get_global_name}"
    print_info_ns_sub f, indent
    @signature_list.each { |sig|
      sig.print_info f, indent
    }
    @celltype_list.each { |ct|
      if ct.get_cell_list.length > 0 then
        ct.print_info f, indent
      end
    }
    @namespace_list.each { |ns|
      if ns.instance_of? Namespace then   # region を含めない
        ns.print_info_ns f, indent
      end
    }
  end

  def self.print_info( f, indent )
    @@root_namespace.print_info_ns f, indent
  end

  #=== Namespace# 構造体メンバーのオフセット定義
  def print_struct_define f
    f.print "\n/***** Offset of members of structures  *****/\n"
    @struct_tag_list.get_items.each{ |tag, sttype|
      # print "sttype: #{tag.get_name} #{sttype}\n"
      tag.get_members_decl.get_items.each{ |decl|
        f.printf "#define OFFSET_OF_%-30s  (%s)\n",
                 "#{tag.get_ID_str}_#{decl.get_name}",
                 "(uint32_t)(intptr_t)&(((#{tag.get_type_str}#{tag.get_type_str_post}*)0)->#{decl.get_name})"
      }
    }
  end

  def print_celltype_define_offset f
    @celltype_list.each { |ct|
      if ct.get_cell_list.length > 0 then
        ct.print_define_offset f
      end
    }
    @namespace_list.each { |ns|
      if ns.instance_of? Namespace then   # region を含めない
        ns.print_celltype_define_offset f
      end
    }
  end

  def print_celltype_define f
    @celltype_list.each { |ct|
      if ct.get_cell_list.length > 0 then
        ct.print_celltype_define f
      end
    }
    @namespace_list.each { |ns|
      if ns.instance_of? Namespace then   # region を含めない
        ns.print_celltype_define f
      end
    }
  end

  def print_call_define f
    @celltype_list.each { |ct|
      if ct.get_cell_list.length > 0 then
        ct.print_call_define f
      end
    }
    @namespace_list.each { |ns|
      if ns.instance_of? Namespace then   # region を含めない
        ns.print_call_define f
      end
    }
  end

  def print_entry_define f
    @celltype_list.each { |ct|
      if ct.get_cell_list.length > 0 then
        ct.print_entry_define f
      end
    }
    @namespace_list.each { |ns|
      if ns.instance_of? Namespace then   # region を含めない
        ns.print_entry_define f
      end
    }
  end
end

class Region
  def print_info_region_sub f, indent
    if get_link_root == self then
      name = "_LinkRoot"
    else
      name = @global_name
    end
    f.print "\n#{indent}/*** #{get_namespace_path} region information cell ***/\n"
    f.print <<EOT
#{indent}cell nTECSInfo::tRegionInfo #{name}RegionInfo{
#{indent}    name = "#{@name}";
EOT
    @cell_list.each{ |cell|
      if cell.get_global_name != :rTECSInfo_TECSInfoSub then
        f.print "#{indent}    cCellInfo[] = #{cell.get_global_name}CellInfo.eCellInfo;\n"
      end
    }
    f.print "#{indent}};\n"
    @cell_list.each{ |cell|
      cell.print_info f, indent
    }
  end

  def print_info_region( f, indent )
    self.print_info_region_sub f, indent
    @namespace_list.each { |region|
      if region.instance_of? Region then
        region.print_info_region f, indent
      end
    }
  end
  
  def print_info( f, indent )
    #p "print_info: #{self.get_global_name}"
    self.print_info_region f, indent
  end

  def self.print_cell_define f
    region.get_link_root.print_cell_define f
    region.get_link_root.get_region{ |region|
      if region.instance_of? Region then
        region.print_cell_define_offset f
      end

    }
  end

  def print_cell_define f
    ct_list = {}
    @cell_list.each{ |cell|
      ct_list[ cell.get_celltype ] = true
    }
    f.print "#define TOPPERS_CB_TYPE_ONLY\n"
    ct_list.each{ |ct, val|
      f.print "#include \"#{ct.get_global_name}_tecsgen.h\"\n"
    }
    f.print "\n"
    @cell_list.each{ |cell|
      name_array = cell.get_celltype.get_name_array cell
      if cell.get_celltype.has_CB?
        cb = "(void*)#{name_array[8]}"
      else
        cb = "0"
      end
      if cell.get_celltype.has_INIB?
        inib = "(void*)&#{name_array[5]}"
        inib_proto = "extern #{cell.get_celltype.get_global_name}_INIB #{name_array[11]};\n"
      else
        inib = "0"
        inib_proto = ""
      end
      if cell.get_global_name != :rTECSInfo_TECSInfoSub then
        f.print <<EOT
#define  #{cell.get_global_name}__CBP   #{cb}
#{inib_proto}#define  #{cell.get_global_name}__INIBP #{inib}
EOT
      end
    }
    @namespace_list.each { |region|
      if region.instance_of? Region then
        region.print_cell_define f
      end
    }
  end
end

class Celltype
  def print_info f, indent
    f.print <<EOT
#{indent}cell nTECSInfo::tCelltypeInfo #{@global_name}CelltypeInfo {
#{indent}    name             = "#{@name}";
#{indent}    b_singleton      = #{@singleton};
#{indent}    b_IDX_is_ID_act  = C_EXP( "#{@global_name}__IDX_is_ID_act" );
#{indent}    sizeOfCB         = C_EXP( "#{@global_name}__sizeOfCB" );
#{indent}    sizeOfINIB       = C_EXP( "#{@global_name}__sizeOfINIB" );
#{indent}    n_cellInLinkUnit = C_EXP( "#{@global_name}__NCELLINLINKUNIT" );
#{indent}    n_cellInSystem   = #{@cell_list.length};
EOT
    @port.each{ |port|
      if port.get_port_type == :ENTRY then
        f.print <<EOT
#{indent}    cEntryInfo[]    = #{@global_name}_#{port.get_name}EntryInfo.eEntryInfo;
EOT
      end
    }
    @port.each{ |port|
      if port.get_port_type == :CALL then
        f.print <<EOT
#{indent}    cCallInfo[]     = #{@global_name}_#{port.get_name}CallInfo.eCallInfo;
EOT
      end
    }
    @attribute.each{ |decl|
      f.print <<EOT
#{indent}    cAttrInfo[]     = #{@global_name}_#{decl.get_name}VarDeclInfo.eVarDeclInfo;
EOT
    }
    @var.each{ |decl|
      f.print <<EOT
#{indent}    cVarInfo[]      = #{@global_name}_#{decl.get_name}VarDeclInfo.eVarDeclInfo;
EOT
    }
    f.print <<EOT
#{indent}};
EOT
    @port.each{ |port|
      if port.get_port_type == :ENTRY then
        port.print_info f, @global_name, indent
      end
    }
    @port.each{ |port|
      if port.get_port_type == :CALL then
        port.print_info f, @global_name, indent
      end
    }
    @attribute.each{ |decl|
      decl.print_info f, @global_name, indent
    }
    @var.each{ |decl|
      decl.print_info f, @global_name, indent
    }
  end

  def print_define_offset f
    # intptr_t に一回キャストするのは 64bit 版を考量してのこと．しかし 32bit としているので 4GB を超える構造体等は扱えない
    if @n_cell_gen > 0 then
      f.print <<EOT

#include "#{@global_name}_tecsgen.h"
EOT
      @attribute.each{ |decl|
        if has_INIB? then
          inib_cb = "INIB"
        else
          inib_cb = "CB"
        end
        if ! decl.is_omit? then
          offset = "(uint32_t)(intptr_t)&(((#{@global_name}_#{inib_cb}*)0)->#{decl.get_name})"
        else
          offset = "0xffffffff"
        end
        f.printf "#define OFFSET_OF_%-30s  (%s)\n", "#{@global_name}_#{decl.get_name}", offset
      }
      @var.each{ |decl|
        if decl.get_size_is then
          inib_cb = "INIB"
        else
          inib_cb = "CB"
        end
          f.printf "#define OFFSET_OF_%-30s  (%s)\n", "#{@global_name}_#{decl.get_name}", "(uint32_t)(intptr_t)&(((#{@global_name}_#{inib_cb}*)0)->#{decl.get_name})"
      }
    else
      f.print <<EOT

// #include "#{@global_name}_tecsgen.h"   // no cell exist
EOT
      # 生成されないセルタイプ
      @attribute.each{ |decl|
        f.printf "#define OFFSET_OF_%-30s  (%s)\n", "#{@global_name}_#{decl.get_name}", "0xffffffff"
      }
      @var.each{ |decl|
        f.printf "#define OFFSET_OF_%-30s  (%s)\n", "#{@global_name}_#{decl.get_name}", "0xffffffff"
      }
    end
  end

  def print_celltype_define f
    if has_INIB? then
      size_INIB = "(sizeof(#{@global_name}_INIB))"
    else
      size_INIB = "(0)"
    end
    if has_CB? then
      size_CB = "(sizeof(#{@global_name}_CB))"
    else
      size_CB   = "(0)"
    end

    f.printf "\n#include \"#{@global_name}_tecsgen.h\"\n"
    f.printf "#define %-50s (#{@idx_is_id_act})\n",     "#{@global_name}__IDX_is_ID_act"
    f.printf "#define %-50s (#{size_CB})\n",            "#{@global_name}__sizeOfCB"
    f.printf "#define %-50s (#{size_INIB})\n",          "#{@global_name}__sizeOfINIB"
    f.printf "#define %-30s (%d)\n", "#{@global_name}__NCELLINLINKUNIT", @n_cell_gen
  end

  def print_call_define f
    if @n_cell_gen > 0 then
      f.print <<EOT

#include "#{@global_name}_tecsgen.h"
EOT
    else
      f.print <<EOT

// #include "#{@global_name}_tecsgen.h"   // no cell exist
EOT
    end
    @port.each{ |port|
      next if port.get_port_type == :ENTRY
      if port.is_omit? || ( port.is_VMT_useless? && port.is_cell_unique? ) || @n_cell_gen == 0 then
        place = "CALL_PLACE_NON"
      elsif port.is_dynamic?
        if port.get_array_size then
          place = "CALL_PLACE_INIB_DES"
        else
          place = "CALL_PLACE_CB_DES"
        end
      elsif ! has_INIB? then
        if port.is_VMT_useless? then
          place = "CALL_PLACE_CB_IDX"
        else
          place = "CALL_PLACE_CB_DES"
        end
      else
        if port.is_VMT_useless? then
          place = "CALL_PLACE_INIB_IDX"
        else
          place = "CALL_PLACE_INIB_DES"
        end
      end
      if ( port.is_VMT_useless? && port.is_cell_unique? ) || @n_cell_gen == 0 then
        offset = "0xffffffff"
      else
        if port.is_dynamic? || ! has_INIB? then
          cb_inib = "CB"
        else
          cb_inib = "INIB"
        end
        offset = "(uint32_t)(intptr_t)&((#{@global_name}_#{cb_inib}*)0)->#{port.get_name}"
      end
      array_size = port.get_array_size
      if array_size == "[]" then
        array_size = "0xffffffff"
      elsif array_size == nil then
        array_size = "0"
      end

      f.printf "#define %-50s (#{offset})\n",                   "#{@global_name}_#{port.get_name}__offset"
      f.printf "#define %-50s (#{array_size})\n",               "#{@global_name}_#{port.get_name}__array_size"
      f.printf "#define %-50s (#{place})\n",                    "#{@global_name}_#{port.get_name}__place"
      f.printf "#define %-50s (#{port.is_VMT_useless?})\n",     "#{@global_name}_#{port.get_name}__b_VMT_useless"
      f.printf "#define %-50s (#{port.is_skelton_useless?})\n", "#{@global_name}_#{port.get_name}__b_skelton_useless"
      f.printf "#define %-50s (#{port.is_cell_unique?})\n",     "#{@global_name}_#{port.get_name}__b_cell_unique"
    }
  end

  def print_entry_define f
    @port.each{ |port|
      next if port.get_port_type == :CALL
      array_size = port.get_array_size
      if array_size == "[]" then
        array_size = "0xffffffff"
      elsif array_size == nil then
        array_size = "0"
      end

      f.printf "#define %-50s (#{array_size})\n",               "#{@global_name}_#{port.get_name}__array_size"
    }
  end
end

class Port
  def print_info f, ct_global, indent
    return if @signature == nil     # signature not found error in cdl
    if @port_type == :ENTRY then
      f.print <<EOT
#{indent}cell nTECSInfo::tEntryInfo #{ct_global}_#{@name}EntryInfo{
#{indent}    name            = "#{@name}";
#{indent}    cSignatureInfo  = #{@signature.get_global_name}SignatureInfo.eSignatureInfo;
#{indent}    b_inline        = #{@b_inline};
#{indent}    array_size      = C_EXP( "#{ct_global}_#{@name}__array_size" );
#{indent}};
EOT
    else
      f.print <<EOT
#{indent}cell nTECSInfo::tCallInfo #{ct_global}_#{@name}CallInfo{
#{indent}    name            = "#{@name}";
#{indent}    cSignatureInfo  = #{@signature.get_global_name}SignatureInfo.eSignatureInfo;
#{indent}    offset            = C_EXP( "#{ct_global}_#{@name}__offset" );
#{indent}    array_size        = C_EXP( "#{ct_global}_#{@name}__array_size" );
#{indent}    b_optional        = #{@b_optional};
#{indent}    b_omit            = #{@b_omit};
#{indent}    b_dynamic         = #{@b_dynamic};
#{indent}    b_ref_desc        = #{@b_ref_desc};
#{indent}    b_allocator_port  = #{@allocator_port!=nil ? true : false};
#{indent}    b_require_port    = #{@b_require};
#{indent}    place             = C_EXP( "#{ct_global}_#{@name}__place" );
#{indent}    b_VMT_useless     = C_EXP( "#{ct_global}_#{@name}__b_VMT_useless" );
#{indent}    b_skelton_useless = C_EXP( "#{ct_global}_#{@name}__b_skelton_useless" );
#{indent}    b_cell_unique     = C_EXP( "#{ct_global}_#{@name}__b_cell_unique" );

#{indent}};
EOT
    end
  end
end

class Cell
  def print_info f, indent
    if @global_name == :rTECSInfo_TECSInfoSub then
      return
    end
    f.print <<EOT

#{indent}/*** #{@global_name} cell information ****/
#{indent}cell nTECSInfo::tCellInfo #{@global_name}CellInfo {
#{indent}    name            = "#{@name}";
#{indent}    cbp             = C_EXP( \"#{@global_name}__CBP\" );
#{indent}    inibp           = C_EXP( \"#{@global_name}__INIBP\" );
#{indent}    cCelltypeInfo   = #{@celltype.get_global_name}CelltypeInfo.eCelltypeInfo;
#{indent}};
EOT
  end
end

class Signature
  def print_info f, indent
    f.print <<EOT

#{indent}/*** #{@global_name} signature information ****/
#{indent}cell nTECSInfo::tSignatureInfo #{@global_name}SignatureInfo {
#{indent}    name            = "#{@name}";
EOT
    @function_head_list.get_items.each{ |fh|
      f.print <<EOT
#{indent}    cFunctionInfo[] = #{@global_name}_#{fh.get_name}FunctionInfo.eFunctionInfo;
EOT
    }
    f.print <<EOT
#{indent}};
EOT
    @function_head_list.get_items.each{ |fh|
      fh.print_info f, indent
    }
  end
end

class FuncHead
  def print_info f, indent
    sig_name = @owner.get_global_name
    func_name = get_name
    f.print <<EOT
#{indent}cell nTECSInfo::tFunctionInfo #{sig_name}_#{func_name}FunctionInfo {
#{indent}    name            = "#{@owner.get_global_name}_#{@name}";
#{indent}    bOneway         = #{is_oneway?};
EOT
    get_paramlist.get_items.each{ |param|
      f.print <<EOT
#{indent}    cParamInfo[]    = #{sig_name}_#{func_name}_#{param.get_name}ParamInfo.eParamInfo;
EOT
    }
    f.print <<EOT
#{indent}    cReturnTypeInfo = #{get_return_type.get_ID_str}TypeInfo.eTypeInfo;
#{indent}};
EOT
    get_paramlist.get_items.each{ |param|
      dbgPrint "param_list #{sig_name}, #{func_name}, #{param.get_name}\n"
      param.print_info f, sig_name, func_name, get_paramlist, indent
    }
    get_return_type.print_info f, indent
  end
end

class ParamDecl
  def print_info f, signature_global_name, func_name, paramdecl_list, indent
    if @size then
      size = "\"#{@size.get_rpn( paramdecl_list )}\""
    else
      size = "(char_t*)0";
    end
    if @count then
      count = "\"#{@count.get_rpn( paramdecl_list )}\""
    else
      count = "(char_t*)0";
    end
    if @string then
      if @string == -1 then
        string = '""'
      else
        string =  "\"#{@string.get_rpn( paramdecl_list )}\""
      end
    else
      string = "(char_t*)0";
    end
    f.print <<EOT
#{indent}cell nTECSInfo::tParamInfo #{signature_global_name}_#{func_name}_#{get_name}ParamInfo {
#{indent}    name            = "#{get_name}";
#{indent}    dir             = PARAM_DIR_#{@direction};
#{indent}    sizeIsExpr      = #{size};
#{indent}    countIsExpr     = #{count};
#{indent}    stringExpr      = #{string};
#{indent}    cTypeInfo       = #{get_type.get_ID_str}TypeInfo.eTypeInfo;
#{indent}};
EOT
    get_type.print_info f, indent
  end
end

class Decl
  def print_info f, parent_ID_str, indent
    if @size_is then
      size = "\"mikan\"";
    else
      size = "(char_t*)0";
    end
    f.print <<EOT
#{indent}cell nTECSInfo::tVarDeclInfo #{parent_ID_str}_#{get_name}VarDeclInfo {
#{indent}    name            = "#{get_name}";
#{indent}    sizeIsExpr      = #{size};
#{indent}    declType        = DECLTYPE_STMEMBER;
#{indent}    offset          = C_EXP( "OFFSET_OF_#{parent_ID_str}_#{get_name}" );
#{indent}    cTypeInfo       = #{get_type.get_ID_str}TypeInfo.eTypeInfo;
#{indent}};
EOT
    get_type.print_info f, indent
  end
end

class Type
  @@typeinfo_printed = {}
  def print_info f, indent
    # Type の info は、最後にまとめて出力するので、ここでは記録するだけ
    if @@typeinfo_printed[ get_ID_str ] then
      return
    end
    # p "ID Str: #{get_ID_str}"
    @@typeinfo_printed[ get_ID_str ] = self
    if self.kind_of? PtrType then
      get_referto.print_info f, indent
    elsif self.kind_of? ArrayType then
      get_type.print_info f, indent
    elsif self.kind_of? DefinedType then
      get_type.print_info f, indent
    elsif self.kind_of? StructType then
      get_members_decl.get_items.each{ |decl|
        decl.get_type.print_info f, indent
      }
    end
  end

  def self.print_info_post f, indent
    @@typeinfo_printed.each{ |nm, type|
      type.print_info_post f, indent
    }
  end

  def print_info_post f, indent
    bit_size = get_bit_size
    if bit_size <= 0 then
      bit_size = "C_EXP( \"sizeof(#{get_type_str}#{get_type_str_post})\" )"
    end
    if self.class.superclass == Type then     # 親クラスが Type の場合 types.rb のクラス
      type_name = self.class.name
    else
      type_name = self.class.superclass.name  # ctypes.rb のクラス (親クラスが types.rb のクラス)
    end
    # p "type: #{type_name}, #{self.class.name}"

    # p "class=#{self.class.name} size=#{bit_size}"
    f.print <<EOT
#{indent}cell nTECSInfo::t#{type_name}Info #{get_ID_str}TypeInfo{
#{indent}    name           = "#{get_type_str}#{get_type_str_post}";
#{indent}    typeKind       = TECSTypeKind_#{type_name};
#{indent}    size           = #{bit_size};
#{indent}    b_const        = #{is_const?};
#{indent}    b_volatile     = #{is_volatile?};
EOT
    if self.kind_of? PtrType then
      f.print "#{indent}    cTypeInfo      = #{get_referto.get_ID_str}TypeInfo.eTypeInfo;\n"
    elsif self.kind_of? ArrayType then
      f.print "#{indent}    cTypeInfo      = #{get_type.get_ID_str}TypeInfo.eTypeInfo;\n"
    elsif self.kind_of? DefinedType then
      f.print "#{indent}    cTypeInfo      = #{get_type.get_ID_str}TypeInfo.eTypeInfo;\n"
    elsif self.kind_of? StructType then
      get_members_decl.get_items.each{ |decl|
        f.print "#{indent}    cTypeInfo[]    = #{decl.get_type.get_ID_str}TypeInfo.eTypeInfo;\n"
      }
    elsif self.kind_of? DescriptorType then
      f.print "#{indent}    cSignatureInfo   = #{get_signature.get_global_name}SignatureInfo.eSignatureInfo;\n"
    end
    
    f.print <<EOT
#{indent}};
EOT
  end

  #=== Type# 型文字列の識別子化
  #型文字列に含まれる識別子として用いることのできない文字を用いることのできる文字列に置き換える
  # 空白 => __
  # * => _Ptr_
  # [] => _Array_
  # Descriptor() => Descriptor_of_
  def get_ID_str
    # puts "get_ID_str: #{self.class.name}"
    if kind_of? PtrType then
      str = get_referto.get_ID_str + "_Ptr_"
    elsif kind_of? ArrayType then
      str = get_type.get_ID_str + "_Array" + get_subscript.eval_const( nil ).to_s + "_"
    elsif kind_of? StructType then
      str = "struct #{@tag}"
    elsif kind_of? DescriptorType then
      str = "Descriptor_of_" + get_signature.get_global_name.to_s
    else
      str = get_type_str + get_type_str_post
    end
    # p "before: #{str}"
    str.gsub!( / /, "__" )
    # p "after: #{str}"
    return str
  end
end

