# -*- coding: utf-8 -*-
#
#  TECS Generator
#      Generator for TOPPERS Embedded Component System
#  
#   Copyright (C) 2008-2017 by TOPPERS Project
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
#   $Id: generate.rb 2663 2017-07-08 23:09:53Z okuma-top $
#++

def ifdef_macro_only f
  f.print <<EOT
#ifndef TOPPERS_MACRO_ONLY

EOT
end

def ifndef_macro_only f
  f.print <<EOT
#ifndef TOPPERS_MACRO_ONLY

EOT
end

def endif_macro_only f
  f.print <<EOT
#endif /* TOPPERS_MACRO_ONLY */

EOT
end

def ifndef_cb_type_only f
  f.print <<EOT
#ifndef TOPPERS_CB_TYPE_ONLY

EOT
end

def ifdef_cb_type_only f
  f.print <<EOT
#ifdef TOPPERS_CB_TYPE_ONLY

EOT
end

def endif_cb_type_only f
  f.print <<EOT
#endif /* TOPPERS_CB_TYPE_ONLY */

EOT
end

def begin_extern_C f
  f.print  <<EOT
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
EOT
end

def end_extern_C f
  f.print  <<EOT
#ifdef __cplusplus
}
#endif /* __cplusplus */
EOT
end

def print_note( f, b_complete = true )
  if b_complete
    f.print "/*\n"
  else
    f.print " *\n"
  end
  f.print TECSMsg.get( :note )
  if b_complete
    f.print " */\n"
  else
    f.print " *\n"
  end
end

def print_Makefile_note f
  f.print TECSMsg.get( :Makefile_note )
end


def print_indent( f, n )
  f.print "    " * n
end

# celltype_private.h を生成

class Namespace
  def generate

    begin
      # root namespace ならば makefile を出力する(全セルタイプに関わるものだけ)
      # 先に出力する
      if @name == "::" then

        gen_makefile_template
        gen_makefile_tecsgen
        if $generating_region.get_n_cells == 0 then
          dbgPrint "only makefile_template #{@name}\n"
          return
        end
      end

      dbgPrint "generating region: #{$generating_region.get_name} namespace=#{@name} gen_dir=#{$gen}\n"
      # global_tecsgen.h (typedef, struct, const) の生成
      gen_global_header

      # signature のコードを生成
      @signature_list.each { |s|
        s.generate
      }

      # celltype のコードを生成
      @celltype_list.each { |t|
        t.generate
      }

      # サブネームスペースのコードを生成
      @namespace_list.each { |n|
        n.generate
      }

    rescue => evar
      # もしスタックトレースが出るまでい時間がかかるようならば、次をコメントアウトしてみるべし
      cdl_error( "H1001 tecsgen: fatal internal error during code generation"  )
      print_exception( evar )
    end
  end

  def generate_post

    if $generating_region.get_n_cells == 0 then
      return
    end

    begin
      # global_tecsgen.h (typedef, struct, const) の終わりのガードコード生成
      gen_global_header_post

      # signature のコードを生成
      @signature_list.each { |s|
        s.generate_post
      }

      # celltype のコードを生成
      @celltype_list.each { |t|
        t.generate_post
      }

      # サブネームスペースのコードを生成
      @namespace_list.each { |n|
        n.generate_post
      }

    rescue => evar
      cdl_error( "H1002 tecsgen: fatal internal error during post code generation"  )
      print_exception( evar )
    end
  end

###  
  def gen_global_header

    # global_tecs.h の生成
    f = AppFile.open( "#{$gen}/global_tecsgen.#{$h_suffix}" )

    if @name == "::" then
      print_note f

      # ガードコードを出力
      f.print <<EOT
#ifndef GLOBAL_TECSGEN_H
#define GLOBAL_TECSGEN_H

EOT

      # import_C で指定されたヘッダファイルの #include を出力
      if Import_C.get_header_list2.length > 0 then
        # ヘッダ include の出力
        f.printf TECSMsg.get( :IMP_comment ), "#_IMP_#"
        Import_C.get_header_list2.each{ |h|
          f.printf( "#include \"#{h}\"\n" )
        }
        f.printf( "/**/\n\n" )
      end

      ifndef_macro_only f
    end

    # typedef, struct, enum を生成
    @decl_list.each { |d|

      # d は Typedef, StructType, EnumType のいずれか
      if d.instance_of?( Typedef ) then

        # Typedef の場合、declarator の @type が　CType でないか
        if ! d.get_declarator.get_type.kind_of?( CType ) then
          d.gen_gh f
        end
      elsif ! d.kind_of?( CType ) then

        # CType ではない (StructType または EnumType)
        d.gen_gh f
#     else
#       ここに該当するのは CStructType, CEnumType
      end
    }

    if @name == "::" then

      if $ram_initializer then
        # Proc to judge the necessity of CB initializer
        b_inline_only_or_proc = Proc.new{ |ct|
          # print ct.get_name, ": ",  ct.need_CB_initializer?, "\n"
          ct.need_CB_initializer?
        }
        gen_celltype_names( f, "extern void ", "_CB_initialize();\n", true, b_inline_only_or_proc )
        gen_celltype_names( f, "extern void ", "_CB_initialize();\n", false, b_inline_only_or_proc )
        f.print "\n#define INITIALIZE_TECS() \\\n"
        gen_celltype_names( f, "\t", "_CB_initialize();\\\n", true, b_inline_only_or_proc )
        gen_celltype_names( f, "\t", "_CB_initialize();\\\n", false, b_inline_only_or_proc )
        f.print( "/* INITIALIZE_TECS terminator */\n\n" )
      else
        f.print "\n#define INITIALIZE_TECS() \n"
      end
      f.print "#define INITIALZE_TECSGEN() INITIALIZE_TECS()  /* for backward compatibility */\n\n"

      f.print( "/* Descriptor for dynamic join */\n" )
      f.print( "#define Descriptor( signature_global_name )  DynDesc__ ## signature_global_name\n" )
      f.print( "#define is_descriptor_unjoined( desc )  ((desc).vdes==NULL)\n\n" )
      endif_macro_only f
    end

    # const を生成  mikan
    @const_decl_list.each { |d|
      f.printf( "#define %-14s ((%s%s)%s)\n", d.get_global_name,
                d.get_type.get_type_str, d.get_type.get_type_str_post,
                d.get_initializer.eval_const2(nil) )
    }

    f.close

  end

  def gen_global_header_post

    # global_tecs.h を開く
    f = AppFile.open( "#{$gen}/global_tecsgen.#{$h_suffix}" )

    if @name == "::" then
      f.print <<EOT

#endif /* GLOBAL_TECSGEN_H */
EOT
    end

    f.close

  end

  #=== Makefile.tecsgen, Makefile.templ の出力
  # 全セルタイプ名を出力する部分を出力
  #    （本メソッドは root namespace に対して呼出す）
  #     個々のセルタイプのメークルールは Celltype クラスで出力
  def gen_makefile
    gen_makefile_template
    gen_makefile_tecsgen
  end

  def gen_makefile_template

    return if $generate_no_template

    ### Makefile.templ の生成
    f = AppFile.open( "#{$gen}/Makefile.templ" )

    print_Makefile_note f

    # Makefile の変数の出力
    f.printf TECSMsg.get( :MVAR_comment ), "#_MVAR_#"
    f.printf "# fixed variable (unchangeable by config or plugin)\n"

    # TARGET の出力 (第一引数 $target に region 名および .exe を付加)
    target = $target
    if $generating_region != @@root_namespace then
      # 子 region のリンクターゲットの場合
      target += "-#{$generating_region.get_global_name}"
    end
    f.print "TARGET_BASE = #{target}\n"

    if $generating_region == @@root_namespace then
      f.print "BASE_DIR = .\n"
      vpath_lead = ""
    else
      f.print "BASE_DIR = ..\n"
      vpath_lead = "../"
    end

    f.print "GEN_DIR = $(BASE_DIR)/#{$gen}\n"

    f.print "INCLUDES ="
    search_path = $import_path + TECSGEN::Makefile.get_search_path
    search_path.each{ |path|
      if TECSGEN.is_absolute_path? path then
        f.print( TECSGEN.subst_tecspath " -I #{path}" )
      else
        f.print " -I $(BASE_DIR)/#{path}"
      end
    }
    f.print " -I $(GEN_DIR)\n"
    f.print "DEFINES ="
    $define.each{ |define| f.print " -D #{define}" }
    f.print "\n\n"
    f.printf "# end of fixed variable (unchangeable by config or plugin)\n"


    vpath_add = ""
    search_path.each{ |path|
      if path != "." then
        if TECSGEN.is_absolute_path? path then
          vpath_add += " " + TECSGEN.subst_tecspath( path )
        else
          vpath_add += " " + vpath_lead + path
        end
      end
    }
    objs_add = ""
    TECSGEN::Makefile.get_objs.each{ |obj| objs_add += " " + obj }
    ld_flag_add = TECSGEN::Makefile.get_ldflags
    var_add = ""
    TECSGEN::Makefile.get_vars.each{ |var|
      var_add += "#" + TECSGEN::Makefile.get_var_comment( var ) + "\n"
      a = TECSGEN::Makefile.get_var_val( var ).to_s
      b = var.to_s
      c = var_add.to_s
      var_add += var.to_s + " =" + " " + TECSGEN::Makefile.get_var_val( var ).to_s + "\n\n"
    }
    pre_tecsgen_target = ""
    TECSGEN::Makefile.get_pre_tecsgen_target{ |target| pre_tecsgen_target += " " + target }
    post_tecsgen_target = ""
    TECSGEN::Makefile.get_post_tecsgen_target{ |target| post_tecsgen_target += " " + target }

#
# LD = gcc
# LDFLAGS =#{ld_flag_add}
# SRC_DIR = $(BASE_DIR)/src
# _TECS_OBJ_DIR = $(GEN_DIR)/
# #   _TECS_OBJ_DIR   # should end with '/'

    f.print <<EOT
#{var_add}

# Pre-tecsgen target
PRE_TECSGEN_TARGET =#{pre_tecsgen_target}

# Post-tecsgen target
POST_TECSGEN_TARGET =#{post_tecsgen_target}

# vpath for C sources and headers
vpath %.#{$c_suffix} $(SRC_DIR) $(GEN_DIR) #{vpath_add}
vpath %.#{$h_suffix} $(SRC_DIR) $(GEN_DIR) #{vpath_add}

# Other objects (out of tecsgen)
OTHER_OBJS =#{objs_add}                      # Add objects out of tecs care.
# OTHER_OBJS = $(_TECS_OBJ_DIR)vasyslog.o
EOT

    # make ルールの出力
    f.printf( TECSMsg.get( :MRUL_comment), "#_MRUL_#" )

    f.print <<EOT
allall: tecs
\tmake all     # in order to include generated Makefile.tecsgen & Makefile.depend

EOT

    if $generating_region.get_n_cells != 0 then
      all_target = "$(TARGET)"
    else
      all_target = ""
    end

    if $generating_region == @@root_namespace then
      if Region.get_link_roots.length > 1 then
        all_target += " sub_regions"
      end
      timestamp = " $(TIMESTAMP)"
    else
      timestamp = ""
    end

    f.print "all : #{all_target}\n\n"
    f.printf TECSMsg.get( :MDEP_comment ), "#_MDEP_#"
    f.print "-include $(GEN_DIR)/Makefile.tecsgen\n"
    if $generating_region.get_n_cells != 0 then
      # Makefile.depend の include
      f.print "-include $(GEN_DIR)/Makefile.depend\n\n"

      f.print "$(TARGET) :#{timestamp} $(CELLTYPE_COBJS) $(TECSGEN_COBJS) $(PLUGIN_COBJS) $(OTHER_OBJS)\n"
      f.print "	$(LD) -o $(TARGET) $(TECSGEN_COBJS) $(CELLTYPE_COBJS) $(PLUGIN_COBJS) $(OTHER_OBJS) $(LDFLAGS)\n\n"
    end

    if Region.get_link_roots.length > 1 && $generating_region == @@root_namespace then
      f.print "\nsub_regions:$(TIMESTAMP)\n"
      Region.get_link_roots.each {|region|
        if region.get_global_name != "" then  # Root region: この Makefile 自身
          f.print "\tcd #{region.get_global_name}; make all\n"
        end
      }
      f.print "\n"
    end

    # clean: ターゲット
    f.print "clean :\n"
    if $generating_region == @@root_namespace then
      Region.get_link_roots.each {|region|
        if region.get_global_name != "" then  # Root region: この Makefile 自身
          f.print "\tcd #{region.get_global_name}; make clean\n"
        end
      }
    end
	f.print "	rm -f $(CELLTYPE_COBJS) $(TECSGEN_COBJS) $(PLUGIN_COBJS) $(OTHER_OBJS) $(TARGET) #{timestamp}\n"
    if $generating_region == @@root_namespace then
      f.print "	rm -rf $(GEN_DIR)\n"
    end
    f.print "\n"

    # tecs: ターゲット
    if $generating_region == @@root_namespace then
      f.print "tecs : $(PRE_TECSGEN_TARGET) $(TIMESTAMP) $(POST_TECSGEN_TARGET)\n\n"
      f.print "$(TIMESTAMP) : $(TECS_IMPORTS)\n"
      f.print "	$(TECSGEN) #{TECSGEN.subst_tecspath( $arguments, true )}\n"
      # f.print "	touch $(TIMESTAMP)\n\n"

    else
      f.print "tecs:\n"
      f.print "\t@echo \"run 'make tecs' in root region\"\n\n"
    end

    f.print "# generic target for objs\n"
    f.print "$(_TECS_OBJ_DIR)%.o : %.#{$c_suffix}\n"
    f.print "	$(CC) -c $(CFLAGS) -o $@ $<\n\n"

    lines = TECSGEN::Makefile.get_lines
    if lines.length > 0 then
      f.print( "# additional lines\n" )
      lines.each{ |line|
        f.print  line, "\n"
      }
      f.print( "# end additional lines\n\n" )
    end

    f.close
  end

  def gen_makefile_tecsgen
    ### Makefile.tecsgen の生成
    f = AppFile.open( "#{$gen}/Makefile.tecsgen" )

    f.print( "TECS_IMPORT_CDLS =" )
    Import.get_list.each{ |cdl_expand_path, import|
      path = import.get_cdl_path
      if TECSGEN.is_absolute_path? path then
        path = TECSGEN.subst_tecspath path
      end
      f.print " "
      f.print( path )
    }
    f.print( "\n" )
    f.print( "TECS_IMPORT_HEADERS =" )
    Import_C.get_header_list.each{ |header,path|
      if TECSGEN.is_absolute_path? path then
        path = TECSGEN.subst_tecspath path
      end
      f.print " "
      f.print path
    }
    f.print "\n"
    f.print "TECS_IMPORTS = $(TECS_IMPORT_CDLS) $(TECS_IMPORT_HEADERS)\n\n"

    f.print "SIGNATURE_HEADERS = \\\n"
    if $generating_region.get_n_cells != 0 then
      @signature_list.each{ |s|
        f.print "\t$(GEN_DIR)/#{s.get_global_name}_tecsgen.#{$h_suffix} \\\n"
      }
    end
    f.print "# SIGNATURE_HEADERS terminator\n\n"

    b_inline_only_or_proc = Proc.new { |ct|  true    }
    f.print "CELLTYPE_TECSGEN_HEADERS = \\\n"
    gen_celltype_names( f, "\t$(GEN_DIR)/", "_tecsgen.h \\\n", true, b_inline_only_or_proc )
    gen_celltype_names( f, "\t$(GEN_DIR)/", "_tecsgen.h \\\n", false, b_inline_only_or_proc )
    f.print "# CELLTYPE_TECSGEN_HEADERS terminator\n\n"
    f.print "CELLTYPE_FACTORY_HEADERS = \\\n"
    gen_celltype_names( f, "\t$(GEN_DIR)/", "_factory.h \\\n", true, b_inline_only_or_proc )
    gen_celltype_names( f, "\t$(GEN_DIR)/", "_factory.h \\\n", false, b_inline_only_or_proc )
    f.print "# CELLTYPE_FACTORY_HEADERS terminator\n\n"
    f.print "# TECS_HEADERS:  headers generated by tecsgen\n"
    f.print "TECS_HEADERS = $(SIGNATURE_HEADERS) $(CELLTYPE_TECSGEN_HEADERS) $(CELLTYPE_FACTORY_HEADERS)\n\n"
    b_inline_only_or_proc = true
    f.print "TECS_INLINE_HEADERS = \\\n"
    gen_celltype_names( f, "\t", "_tecsgen.h \\\n", false, b_inline_only_or_proc )
    f.print "# TECS_INLINE_HEADERS terminator\n\n"
    f.print "PLUGIN_INLINE_HEADERS = \\\n"
    gen_celltype_names( f, "\t", "_tecsgen.h \\\n", true, b_inline_only_or_proc )
    f.print "# PLUGIN_INLINE_HEADERS terminator\n\n"

    ### set domain variables ###
    domain_type = nil
    domain_regions = nil
    DomainType.get_domain_regions.each{ |dt, regions|
      # domain_type は一つのノードには、一つしかないので、このループは、必ず一回しか回らない
      domain_regions = regions
      domain_type = dt
    }
    if domain_regions == nil then
      # in case no 'domain' specified at region
      domain_regions = [ Region.get_root ]
    end

    hasDomainProc = Proc.new{
      if domain_regions.length > 1 || domain_regions[0] != Region.get_root then
        true
      else
        false
      end
    }
    decideDomainNameProc = Proc.new { |region|
      if region.is_root? then
        if hasDomainProc.call then
          dn = "_Root_"
        else
          dn = ""
        end
      else
        dn = "_#{region.get_namespace_path.get_global_name}"
      end
    }

    f.print( "# TECS_COBJS: all objects of TECS, include both user written code and tecsgen automatically generated code\n" )
    f.print( "TECS_COBJS = $(TECSGEN_COBJS) $(PLUGIN_COBJS) $(CELLTYPE_COBJS)\n\n" )

    ### in case domain is used ###
    if hasDomainProc.call then

      f.print( "# TECS_DOMAINS: list of domain names (names of 'domain' spacified region)\n" )
      f.print( "TECS_DOMAINS = " )
      domain_regions.each{ |r|
        if r.get_domain_type.get_option != "OutOfDomain" then
          f.print( " #{r.get_namespace_path.get_global_name}" )
        end
      }
      f.print( "\n\n" )

      f.print( "# TECS_COBJS: objects from sources which are automatically generated by tecsgen\n" )
      f.print( "TECSGEN_COBJS = \\\n" )
      domain_regions.each{ |r|
        f.print( "	$(TECSGEN#{decideDomainNameProc.call r}_COBJS) \\\n" )
      }
      f.print( "# TECSGEN_COBJS terminator\n\n" )

      f.print( "# PLUGIN_COBJS: objects from sources which are automatically generated by plugin(s)\n" )
      f.print( "PLUGIN_COBJS = \\\n" )
      domain_regions.each{ |r|
        f.print( "	$(PLUGIN#{decideDomainNameProc.call r}_COBJS) \\\n" )
      }
      f.print( "# PLUGIN_COBJS terminator\n\n" )

      f.print( "CELLTYPE_COBJS = \\\n" )
      domain_regions.each{ |r|
        f.print( "	$(CELLTYPE#{decideDomainNameProc.call r}_COBJS) \\\n" )
      }
      f.print( "# CELLTYPE_COBJS terminator\n\n" )

      f.print( "TECSGEN_SRCS = \\\n" )
      domain_regions.each{ |r|
        f.print( "	$(TECSGEN#{decideDomainNameProc.call r}_SRCS) \\\n" )
      }
      f.print( "# TECSGEN_SRCS terminator\n\n" )

      f.print( "PLUGIN_SRCS = \\\n" )
      domain_regions.each{ |r|
        f.print( "	$(PLUGIN#{decideDomainNameProc.call r}_SRCS) \\\n" )
      }
      f.print( "# PLUGIN#_SRCS terminator\n\n" )
    end

    ###
    f.print( "# TECS_COBJS: objects from sources which are automatically generated by tecsgen\n" )
    domain_regions.each{ |r|
      nsp = decideDomainNameProc.call( r )
      f.print( "TECSGEN#{nsp}_COBJS = \\\n" )
      gen_celltype_names_domain( f, "	$(_TECS_OBJ_DIR)", "_tecsgen.o \\\n", domain_type, r, false )
      f.print( "# TECSGEN#{nsp}_COBJS terminator\n\n" )
    }

    f.print( "# PLUGIN_COBJS: objects from sources which are automatically generated by plugin(s)\n" )
    domain_regions.each{ |r|
      nsp = decideDomainNameProc.call( r )
      f.print( "PLUGIN#{nsp}_COBJS = \\\n" )
      gen_celltype_names_domain( f, "	$(_TECS_OBJ_DIR)", "_tecsgen.o \\\n", domain_type, r, true )
      gen_celltype_names_domain2( f, "	$(_TECS_OBJ_DIR)", ".o \\\n", domain_type, r, true, false )
      f.print( "# PLUGIN#{nsp}_COBJS terminator\n\n" )
    }

    f.print( "# CELLTYPE_COBJS: objects of celltype code written by user\n" )
    domain_regions.each{ |r|
      nsp = decideDomainNameProc.call( r )
      f.print( "CELLTYPE#{nsp}_COBJS = \\\n" )
      gen_celltype_names_domain2( f, "	$(_TECS_OBJ_DIR)", ".o \\\n", domain_type, r, false, false )
      f.print( "# CELLTYPE#{nsp}_COBJS terminator\n\n" )
    }

    f.print( "# TECSGEN_SRCS: sources automatically generated by tecsgen\n" )
    domain_regions.each{ |r|
      nsp = decideDomainNameProc.call( r )
      f.print( "TECSGEN#{nsp}_SRCS = \\\n" )
      gen_celltype_names_domain( f, "	$(GEN_DIR)/", "_tecsgen.#{$c_suffix} \\\n", domain_type, r, false )
      f.print( "# TECSGEN#{nsp}_SRCS terminator\n\n" )
    }

    f.print( "# PLUGIN_SRCS: sources automatically generated by plugin\n" )
    domain_regions.each{ |r|
      nsp = decideDomainNameProc.call( r )
      f.print( "PLUGIN#{nsp}_SRCS = \\\n" )
      gen_celltype_names_domain( f, "	$(GEN_DIR)/", "_tecsgen.#{$c_suffix} \\\n", domain_type, r, true )
      gen_celltype_names_domain2( f, "	$(GEN_DIR)/", ".#{$c_suffix} \\\n", domain_type, r, true, false )
      f.print( "# PLUGIN#{nsp}_SRCS terminator\n\n" )
    }

    f.close

  end

  #=== すべてのセルタイプの名前を出力
  #f::       FILE:   出力先ファイル
  #prepend:: string: 前置文字列
  #append::  string: 後置文字列
  #b_plguin::  bool:   plugin により生成されたセルタイプを出力
  ##b_inline_only::  bool:   true ならば inline の entry port のみのセルタイプを含める
  #b_inline_only_or_proc::  bool|Proc:   true ならば inline の entry port のみ、かつインアクティブなセルタイプを含める
  #                                      Proc ならば Proc を実行した結果 true ならば含める
  #  namespace "::" から呼出される
  def gen_celltype_names( f, prepend, append, b_plugin, b_inline_only_or_proc = true )
    dbgPrint "gen_celltype_names #{@name}\n"

    @celltype_list.each { |ct|
      next if ! ct.need_generate?
#      next if b_inline_only == false && ct.is_all_entry_inline?
      next if b_inline_only_or_proc == false && ct.is_all_entry_inline? && ! ct.is_active?
      # print "Proc:", b_inline_only_or_proc.kind_of?( Proc ), b_inline_only_or_proc.class, "\n"
      next if b_inline_only_or_proc.kind_of?( Proc ) && ( b_inline_only_or_proc.call( ct ) == false )
      if ( b_plugin && ct.get_plugin ) || ( ! b_plugin && ! ct.get_plugin ) then
        f.print " #{prepend}#{ct.get_global_name}#{append}"
      end
    }
    @namespace_list.each { |ns|
      ns.gen_celltype_names( f, prepend, append, b_plugin, b_inline_only_or_proc )
    }

  end

  #=== すべてのセルタイプの名前を出力
  #region:: Region:
  # gen_celltype_names とgen_celltype_names_domain の相違：
  #   region を domain_roots に含む場合、出力する．
  #   または、region を含まないが、domain_roots が複数かつルートリージョンの場合、出力する．
  # それ以外は、gen_celltype_names の説明を参照
  def gen_celltype_names_domain( f, prepend, append, domain_type, region, b_plugin, b_inline_only = true )
    dbgPrint "gen_celltype_names #{@name}\n"

    @celltype_list.each { |ct|
      next if ! ct.need_generate?
#      next if b_inline_only == false && ct.is_all_entry_inline?
      next if b_inline_only == false && ct.is_all_entry_inline? && ! ct.is_active?
      if ( b_plugin && ct.get_plugin ) || ( ! b_plugin && ! ct.get_plugin ) then
        regions = ct.get_domain_roots[ domain_type ]
        if regions.include?( region ) then
          # p "BBB celltype:#{ct.get_name} domain:#{domain_type} append:#{append}"
          if region.is_root? then
            nsp = ""
          else
            nsp = "_#{region.get_namespace_path.get_global_name}"
          end
          f.print " #{prepend}#{ct.get_global_name}#{nsp}#{append}"
        elsif region.is_root? then
          # the case of domain_roots >= 2 && no cell in root region
          if regions.length > 1 then
            f.print " #{prepend}#{ct.get_global_name}#{append}"
          end
        end
      end
    }
    @namespace_list.each { |ns|
      ns.gen_celltype_names_domain( f, prepend, append, domain_type, region, b_plugin, b_inline_only )
    }
  end
  #== Namespace#すべてのセルタイプの名前を出力
  # セルタイプコードのための名前出力
  # gen_celltype_names_domain と gen_celltype_names_domain2 の相違
  # ・どれか一つのリージョンにしか出さない
  #     domain_roots が1つだけで、指定リージョンを含む
  #     domain_roots が2つ以上で、指定リージョンがルートリージョン 
  # ・ドメイン名を付加しない
  def gen_celltype_names_domain2( f, prepend, append, domain_type, region, b_plugin, b_inline_only = true )
    dbgPrint "gen_celltype_names #{@name}\n"

    @celltype_list.each { |ct|
      next if ! ct.need_generate?
#      next if b_inline_only == false && ct.is_all_entry_inline?
      next if b_inline_only == false && ct.is_all_entry_inline? && ! ct.is_active?
      if ( b_plugin && ct.get_plugin ) || ( ! b_plugin && ! ct.get_plugin ) then
        # p "BBB celltype:#{ct.get_name} domain:#{domain_type} append:#{append}"
        regions = ct.get_domain_roots[ domain_type ]
        if regions.include?( region ) && regions.length == 1 then
          f.print " #{prepend}#{ct.get_global_name}#{append}"
        elsif region.is_root? then
          # the case of domain_roots >= 2 && no cell in root region
          if regions.length > 1 then
            f.print " #{prepend}#{ct.get_global_name}#{append}"
          end
        end
      end
    }
    @namespace_list.each { |ns|
      ns.gen_celltype_names_domain2( f, prepend, append, domain_type, region, b_plugin, b_inline_only )
    }
  end

  #=== Namespace#すべてのシグニチャをたどる
  def travers_all_signature # ブロック引数 { |signature|  }
    proc = Proc.new    # このメソッドのブロック引数
    @signature_list.each{ |sig|
      proc.call sig
    }
    @namespace_list.each{ |ns|
      ns.travers_all_signature_proc proc
    }
  end
  def travers_all_signature_proc proc
    @signature_list.each{ |sig|
      proc.call sig
    }
    @namespace_list.each{ |ns|
      ns.travers_all_signature_proc proc
    }
  end
end

class Typedef
  def gen_gh f

#    print "Typedef.gen_gh\n"
#    show_tree 1

    f.printf( "typedef %-14s %s%s;\n",
		"#{@declarator.get_type.get_type_str}",
		"#{@declarator.get_name}",
		"#{@declarator.get_type.get_type_str_post}")
  end
end

class StructType < Type
  def gen_gh f

#    print "StructType.gen_gh\n"
#    show_tree 1

    if ! @b_define then
      return
    end

    f.print "struct #{@tag} {\n"

    @members_decl.get_items.each{ |i|
      f.printf( "                %-14s %s%s;\n", "#{i.get_type.get_type_str}", "#{i.get_name}", "#{i.get_type.get_type_str_post}" )
    }

    f.print "};\n"

  end
end

class Signature

  def generate
    generate_signature_header
  end

  def generate_post
    generate_signature_header_post
  end

  def generate_signature_header
    f = AppFile.open("#{$gen}/#{@global_name}_tecsgen.#{$h_suffix}")

    print_note f
    gen_sh_guard f
    gen_sh_info f
    gen_sh_include f

    ifndef_macro_only f
    gen_sh_func_tab f
    endif_macro_only f
    gen_sh_func_id f

    f.close
  end

  def generate_signature_header_post
    f = AppFile.open("#{$gen}/#{@global_name}_tecsgen.#{$h_suffix}")
    gen_sh_endif f
    f.close
  end


#####  signature header

  def gen_sh_guard f
    f.print("#ifndef #{@global_name}_TECSGEN_H\n")
    f.print("#define #{@global_name}_TECSGEN_H\n\n")
  end

  def gen_sh_info f
    f.print <<EOT
/*
 * signature   :  #{@name}
 * global name :  #{@global_name}
 * context     :  #{get_context}
 */

EOT
  end

  def gen_sh_include f
    dl = get_descriptor_list
    if dl.length > 0 then
      f.printf TECSMsg.get(:SDI_comment), "#_SDI_#"
      dl.each{ |dt,param|
        f.print <<EOT
/* pre-typedef incomplete-type to avoid error in case of mutual or cyclic reference */
#ifndef Descriptor_of_#{dt.get_global_name}_Defined
#define  Descriptor_of_#{dt.get_global_name}_Defined
typedef struct { struct tag_#{dt.get_global_name}_VDES *vdes; } Descriptor( #{dt.get_global_name} );
#endif
EOT
#        f.print "#include \"#{dt.get_global_name}_tecsgen.#{$h_suffix}\"\n"
      }
      f.print "\n"
    end

  end

  def gen_sh_func_tab f

    # シグニチャディスクリプタの出力
    f.printf TECSMsg.get(:SD_comment), "#_SD_#"
    f.print "struct tag_#{@global_name}_VDES {\n"
    f.print "    struct tag_#{@global_name}_VMT *VMT;\n"
    f.print "};\n\n"

    # シグニチャ関数テーブルの出力
    f.printf TECSMsg.get(:SFT_comment), "#_SFT_#"
    f.print( "struct tag_#{@global_name}_VMT {\n" )
    get_function_head_array.each{ |fun|
      f.print "    "
      functype = fun.get_declarator.get_type
      f.printf( "%-14s", functype.get_type_str )

      f.print( " (*#{fun.get_name}__T)(" )
      f.print( " const struct tag_#{@global_name}_VDES *edp" ) unless @singleton

      if functype.get_paramlist then
        items = functype.get_paramlist.get_items
        len = items.length
      else
        # ここで nil になるのは、引数なしの時に void がなかった場合
        items = []
        len = 0
      end

      i = 0
      items.each{ |param|
        f.print ", "
        f.print( param.get_type.get_type_str )
        f.print( " " )
        f.print( param.get_name )
        f.print( param.get_type.get_type_str_post )
        i += 1
      }
      f.print " );\n"
    }
    if get_function_head_array.length == 0 then
      f.print( "    void   (*dummy__)(void);\n" )
    end

    f.print "};\n\n"
    f.printf TECSMsg.get(:SDES_comment), "#_SDES_#"
    f.print <<EOT
#ifndef Descriptor_of_#{@global_name}_Defined
#define  Descriptor_of_#{@global_name}_Defined
typedef struct { struct tag_#{@global_name}_VDES *vdes; } Descriptor( #{@global_name} );
#endif
EOT
  end

  #=== Signature# 関数の ID の define を出力
  def gen_sh_func_id f
    f.print "/* function id */\n"
    get_function_head_array.each{ |fun|
      f.printf( "#define\tFUNCID_%-31s (%d)\n", "#{@global_name}_#{fun.get_name}".upcase, get_id_from_func_name( fun.get_name ) )
    }
    f.print "\n"
  end

  def gen_sh_endif f
    f.print("#endif /* #{@global_name}_TECSGEN_H */\n")
  end

end

class Celltype

  def generate

    if need_generate?    # セルのないセルタイプは生成しない

      generate_private_header
      generate_factory_header
      generate_cell_code
      generate_template_code
      generate_inline_template_code
      generate_celltype_factory_code
      generate_cell_factory_code
      generate_makefile

    elsif $generate_all_template   # テンプレートコード生成オプション

      generate_template_code
      generate_inline_template_code

      # generate_makefile_template は Makefile に追記するものだから、呼び出さない

    end

  end

  def generate_post
    return if ! need_generate?    # セルのないセルタイプは生成しない

    generate_private_header_post
    generate_factory_header_post
  end

  def generate_private_header

    f = AppFile.open("#{$gen}/#{@global_name}_tecsgen.#{$h_suffix}")

    print_note f

    gen_ph_guard f
    gen_ph_info f
    gen_ph_include f
    #
    ifndef_macro_only f
    begin_extern_C f
    gen_ph_cell_cb_type f
    gen_ph_INIB_as_CB f
    gen_ph_extern_cell f          # セルタイプグルコード以外は参照しない
    gen_ph_typedef_idx f          # mikan 参照するものができていない
    gen_ph_ep_fun_prototype f
    end_extern_C f
    endif_macro_only f
	#
    gen_ph_include_cb_type f

    if @n_entry_port_inline == 0 then
      # inline がなければ CB_TYPE_ONLY とする
      # inline ありの場合、いったん define しておいて、後ですべて undef する
      ifndef_cb_type_only f
    end

    gen_ph_base f
    gen_ph_valid_idx f
    gen_ph_n_cp f                 if @n_call_port_array > 0
    gen_ph_n_ep f                 if @n_entry_port_array > 0
    gen_ph_test_optional_call_port f
    gen_ph_get_cellcb f
    gen_ph_attr_access f          if @n_attribute_rw > 0 || @n_attribute_ro > 0 || @n_var > 0
    gen_ph_cp_fun_macro f         if @n_call_port > 0
#    gen_ph_abstract_ep_des_type f

    if @n_entry_port_inline == 0 then
      endif_cb_type_only f
    end

    ifndef_macro_only f
    begin_extern_C f
#    gen_ph_cell_cb_type f
#    gen_ph_INIB_as_CB f
#    gen_ph_extern_cell f          # セルタイプグルコード以外は参照しない
    # gen_ph_typedef_idx f          # mikan 参照するものができていない
#    gen_ph_ep_fun_prototype f
    gen_ph_ep_skel_prototype f

    #--- CB_TYPE_ONLY の場合、ref_desc, set_desc 関数は含めない (マクロ参照するため)
    if @n_entry_port_inline == 0 then
      ifndef_cb_type_only f
    end
    gen_ph_ref_desc_func f
    gen_ph_set_desc_func f
    if @n_entry_port_inline == 0 then
      endif_cb_type_only f
    end
    end_extern_C f
    endif_macro_only f

    # 短縮形などのマクロ出力
    if @n_entry_port_inline == 0 then
      ifndef_cb_type_only f
    end
    gen_ph_valid_idx_abbrev f
    gen_ph_get_cellcb_abbrev f
    gen_ph_attr_access_abbrev f   if @n_attribute_rw > 0 || @n_attribute_ro > 0 || @n_var > 0
    gen_ph_cp_fun_macro_abbrev f  if @n_call_port > 0
    gen_ph_ref_desc_macro_abbrev f
    gen_ph_set_desc_macro_abbrev f
    gen_ph_test_optional_call_port_abbrev f
    gen_ph_ep_fun_macro f         if @n_entry_port > 0
    gen_ph_foreach_cell f         # FOREACH マクロの出力
    gen_ph_cb_initialize_macro f   # CB 初期化マクロの出力．消費しないので ram_initializer フラグに関わらず出力
    gen_ph_dealloc_code f, ""
    gen_ph_dealloc_code f, "_RESET"
    if @n_entry_port_inline == 0 then
      endif_cb_type_only f
    end

    f.close
  end

  def generate_private_header_post

    f = AppFile.open("#{$gen}/#{@global_name}_tecsgen.#{$h_suffix}")

    ifndef_macro_only f
    gen_ph_inline f
    endif_macro_only f

    if @n_entry_port_inline > 0 then
      ifdef_cb_type_only f
      f.printf TECSMsg.get( :UDF_comment ), "#_UDF_#"
      f.print "#undef VALID_IDX\n"
      f.print "#undef GET_CELLCB\n"
      f.print "#undef CELLCB\n"
      f.print "#undef CELLIDX\n"
      f.print "#undef #{@name}_IDX\n"

      f.print "#undef FOREACH_CELL\n"
      f.print "#undef END_FOREACH_CELL\n"
      f.print "#undef INITIALIZE_CB\n"
      f.print "#undef SET_CB_INIB_POINTER\n"

      @attribute.each { |a|
        f.print( "#undef ATTR_#{a.get_name}\n" )
        f.print( "#undef #{@global_name}_ATTR_#{a.get_name}\n" )
        f.print( "#undef #{@global_name}_GET_#{a.get_name}\n" )
      }
      @var.each { |v|
        f.print( "#undef VAR_#{v.get_name}\n" )
        f.print( "#undef VAR_#{v.get_name}\n" )
        f.print( "#undef #{@global_name}_VAR_#{v.get_name}\n" )
        f.print( "#undef #{@global_name}_GET_#{v.get_name}\n" )
        f.print( "#undef #{@global_name}_SET_#{v.get_name}\n" )
      }
      @port.each { |p|
        next if p.get_port_type != :CALL

        # is_...joined は omit するケースでも出力されるため、omit を検査する前に出力
        if p.is_optional? then
          f.print( "#undef is_#{p.get_name}_joined\n" )
        end

        next if p.is_omit?

        p.get_signature.get_function_head_array.each{ |fun|
          f.print( "#undef #{@global_name}_#{p.get_name}_#{fun.get_name}\n" )
          if ! p.is_require? || p.has_name? then
            f.print( "#undef #{p.get_name}_#{fun.get_name}\n" )
          else
            f.print( "#undef #{fun.get_name}\n" )
          end
        }
        if p.is_dynamic? then
          f.print( "#undef #{p.get_name}_set_descriptor\n" )
          if p.is_optional? then
            f.print( "#undef #{p.get_name}_unjoin\n" )
          end
        elsif p.is_ref_desc? then
          f.print( "#undef #{p.get_name}_refer_to_descriptor\n" )
          f.print( "#undef #{p.get_name}_ref_desc\n" )
        end
      }
      @port.each { |p|
        next if p.get_port_type != :ENTRY
        next if p.is_omit?
        p.get_signature.get_function_head_array.each{ |fun|
          f.print( "#undef #{p.get_name}_#{fun.get_name}\n" )
        }
      }

      gen_ph_dealloc_code( f, "", true )
      gen_ph_dealloc_code( f, "_RESET", true )

      endif_cb_type_only f
    end

    gen_ph_endif f

    f.close
  end

  #=== CELLTYPE_tecsgen.c を生成
  def generate_cell_code
    fs = { }
    f = nil
    @domain_roots.each{ |domain_type_name, regions|
      regions.each{ |r|
        if r.is_root? then
          nsp = ""
        else
          nsp = "_#{r.get_namespace_path.get_global_name}"
        end
        # p "celltype:#{@name} dn:#{domain_type_name} nsp:#{nsp}"
        fs[r] = AppFile.open("#{$gen}/#{@global_name}#{nsp}_tecsgen.#{$c_suffix}")
        if r.is_root? then
          f = fs[r]
        end
      }
    }

    # in case that domain_roots does not include root region
    if f == nil then
      regions = nil
      @domain_roots.each{ |domain_type_name, regions_|
        regions = regions_   # domain_type_name is unique
      }
      if regions.length > 1 then 
        # if domain_roots.length >= 2 && no cell in root region
        # shared code are placed in root region
        f = AppFile.open("#{$gen}/#{@global_name}_tecsgen.#{$c_suffix}")
      else
        # shared code are placed in unique region
        f = fs[ regions[0] ]
      end
    end

    # すべての _tecsgen.c に出力
    print_note f
    gen_cell_private_header f
    gen_cell_factory_header f
    gen_cell_ep_des_type f

    # すべての _tecsgen.c に出力
    fs.each{ |r,f2|
      if f == f2 then
        next
      end
      print_note f2
      gen_cell_private_header f2
      gen_cell_factory_header f2
      gen_cell_ep_des_type f2
    }

    # 一つの _tecsgen.c に出力
    gen_cell_skel_fun f
    gen_cell_fun_table f
    gen_cell_var_init f

    # セルごとに _tecsgen.c に出力
    gen_cell_ep_vdes fs
    gen_cell_ep_vdes_array fs
    gen_cell_cb_out_init fs         # INITIALIZE_CB で参照されるため ram_initializer=false でも消せない
    gen_cell_cb fs
    gen_cell_extern_mt fs
    gen_cell_ep_des fs

    # 一つの _tecsgen.c に出力
    gen_cell_cb_tab f
    if $ram_initializer then
      gen_cell_cb_initialize_code f
    end

    fs.each{ |r,f2|
      f2.close
      if f == f2 then
        f = nil
      end
    }
    if f then
      f.close
    end

  end

#####  celltype header

  def gen_ph_guard( f, post = "TECSGEN" )
    f.print("#ifndef #{@global_name}_#{post}_H\n")
    f.print("#define #{@global_name}_#{post}_H\n\n")
  end

  def gen_ph_info f

    yn_idx_is_id = "no"
    yn_idx_is_id = "yes"  if @idx_is_id
    yn_idx_is_id_act = "no"
    yn_idx_is_id_act = "yes"  if @idx_is_id_act
    yn_singleton = "no"
    yn_singleton = "yes"  if @singleton
    yn_rom       = "no"
    yn_rom       = "yes"  if $rom
    yn_cb_init   = "no"
    yn_cb_init   = "yes"  if need_CB_initializer?
    # @singleton = false    # mikan singleton  060827

    f.print <<EOT
/*
 * celltype          :  #{@name}
 * global name       :  #{@global_name}
 * idx_is_id(actual) :  #{yn_idx_is_id}(#{yn_idx_is_id_act})
 * singleton         :  #{yn_singleton}
 * has_CB            :  #{has_CB?}
 * has_INIB          :  #{has_INIB?}
 * rom               :  #{yn_rom}
 * CB initializer    :  #{yn_cb_init}
 */

EOT
  end

  def gen_ph_include f
    # ランタイムヘッダの include
#    f.printf TECSMsg.get( :IRTH_comment), "#_IRTH_#"
#    f.print "#include \"tecs.#{$h_suffix}\"\n\n"

    # グローバルヘッダの include
    f.printf TECSMsg.get( :IGH_comment ), "#_IGH_#"
    f.print "#include \"global_tecsgen.#{$h_suffix}\"\n\n"

    # シグニチャヘッダの include
    f.printf TECSMsg.get( :ISH_comment ), "#_ISH_#"
    @port.each { |p|
      next if p.is_omit?
      hname = "#{p.get_signature.get_global_name}_tecsgen.#{$h_suffix}".to_sym
      if header_included?( hname ) == false then
        f.print "#include \"#{hname}\"\n"
      end
    }
    f.print "\n"
  end

  def gen_ph_include_cb_type f

    if ! @b_cp_optimized then
      return
    end

    # 最適化のため参照するセルタイプの CB 型の定義を取込む
    # _CB_TYPE_ONLY を定義した上で include する
    f.printf( TECSMsg.get( :ICT_comment ), "#_ICT_#" )

    f.print( "#ifndef  TOPPERS_CB_TYPE_ONLY\n" )
    f.print( "#define  #{@global_name}_CB_TYPE_ONLY\n" )
    f.print( "#define TOPPERS_CB_TYPE_ONLY\n" )
    f.print( "#endif  /* TOPPERS_CB_TYPE_ONLY */\n" )

    @port.each { |p|
      next if p.get_port_type != :CALL
      next if p.is_omit?

      if p.is_skelton_useless? || p.is_cell_unique? || p.is_VMT_useless? then
        # 最適化コード (optimize) # スケルトン不要など
        p2 = p.get_real_callee_port
        if p2 then
          ct = p2.get_celltype
          hname = "#{ct.get_global_name}_tecsgen.#{$h_suffix}".to_sym
          if header_included?( hname ) == false then
            f.print( "#include \"#{hname}\"\n" )
          end
        # else
          # optional で未結合
        end
      end

    }
    f.print( "#ifdef  #{@global_name}_CB_TYPE_ONLY\n" )
    f.print( "#undef TOPPERS_CB_TYPE_ONLY\n" )
    f.print( "#endif /* #{@global_name}_CB_TYPE_ONLY */\n" )

#    @port.each { |p|
#      next if p.get_port_type != :CALL
#      if p.is_skelton_useless? || p.is_cell_unique? || p.is_VMT_useless? then
#        # 最適化コード (optimize) # スケルトン不要など
#        p2 = p.get_real_callee_port
#        ct = p2.get_celltype
#        ct.gen_ph_typedef_idx f
#        ct.gen_ph_INIB_as_CB f
#        ct.gen_ph_extern_cell f
#        f.print "\n"
#      end
#    }

  end


  def gen_ph_base f
    return if @singleton

    # ID の基数および個数の define を出力
    f.printf("#define %-20s %10s  /* %s #_NIDB_# */\n", "#{@global_name}_ID_BASE", "(#{@id_base})", TECSMsg.get(:NIDB_comment))
    f.printf("#define %-20s %10s  /* %s  #_NCEL_# */\n\n", "#{@global_name}_N_CELL", "(#{@n_cell_gen})", TECSMsg.get(:NCEL_comment))
  end

  def gen_ph_valid_idx f
    return if @singleton

    # mikan  最適化
    # IDX 正当性チェックマクロの出力
    f.printf( TECSMsg.get( :CVI_comment ), "#_CVI_#" )
    if @idx_is_id_act then
      f.print("#define #{@global_name}_VALID_IDX(IDX) (#{@global_name}_ID_BASE <= (IDX) && (IDX) < #{@global_name}_ID_BASE+#{@global_name}_N_CELL)\n\n")
    else
      f.print("#define #{@global_name}_VALID_IDX(IDX) (1)\n\n")
    end

  end

  def gen_ph_valid_idx_abbrev f
    return if @singleton

    # IDX 正当性チェックマクロ（短縮形）の出力
    f.printf( TECSMsg.get( :CVIA_comment ), "#_CVIA_#")
    f.print("#define VALID_IDX(IDX)  #{@global_name}_VALID_IDX(IDX)\n\n")

  end

  #=== 呼び口配列の大きさを得るマクロの出力
  #
  #セルタイプヘッダへ呼び口の個数を出力
  def gen_ph_n_cp f

    b_comment = false
    @port.each { |p|
      next if p.get_port_type != :CALL
      next if p.is_omit?
      next if p.get_array_size == nil

      if ! b_comment then
        f.printf( TECSMsg.get( :NCPA_comment ), "#_NCPA_#" )
        b_comment = true
      end

      if p.get_array_size != "[]" then       # 固定長配列
        f.print( "#define N_CP_#{p.get_name}    (#{p.get_array_size})\n" )
        f.print( "#define NCP_#{p.get_name}     (#{p.get_array_size})\n" )
      else                                   # 可変長配列
        if @singleton then
          if has_INIB? then
            inib = "INIB"
          else
            inib = "CB"
          end
          f.print( "#define N_CP_#{p.get_name}  (#{@global_name}_SINGLE_CELL_#{inib}.n_#{p.get_name})\n" )
          f.print( "#define NCP_#{p.get_name}   (#{@global_name}_SINGLE_CELL_#{inib}.n_#{p.get_name})\n" )
          # mikan singleton ならば、固定長化できる
        else
          if has_CB? && has_INIB? then
            inib = "->_inib"
          else
            inib = ""
          end
          f.print( "#define N_CP_#{p.get_name}(p_that)  ((p_that)#{inib}->n_#{p.get_name})\n" )
          f.print( "#define NCP_#{p.get_name}           (N_CP_#{p.get_name}(p_cellcb))\n" )
        end
      end
    }
  end

  #=== 受け口配列の大きさを得るマクロの出力
  #
  #セルタイプヘッダへ受け口の個数を出力
  def gen_ph_n_ep f

    b_comment = false
    @port.each { |p|
      next if p.get_port_type != :ENTRY
      # next if p.is_omit?                       # 受け口配列の個数は省略しない
      next if p.get_array_size == nil

      if ! b_comment then
        f.printf( TECSMsg.get( :NEPA_comment ), "#_NEPA_#" )
        b_comment = true
      end

      if p.get_array_size != "[]" then       # 固定長配列
        f.print( "#define NEP_#{p.get_name}     (#{p.get_array_size})\n" )
      else                                   # 可変長配列
        if @singleton then
          if has_INIB? then
            inib = "INIB"
          else
            inib = "CB"
          end
          f.print( "#define NEP_#{p.get_name}   (#{@global_name}_SINGLE_CELL_#{inib}.n_#{p.get_name})\n" )
          # mikan singleton ならば、固定長化できる
        else
          if has_CB? && has_INIB? then
            inib = "->_inib"
          else
            inib = ""
          end
          f.print( "#define NEP_#{p.get_name}           ((p_cellcb)#{inib}->n_#{p.get_name})\n" )
        end
      end
    }
  end

  #=== optional な呼び口が結合されているかテストするコードの生成
  def gen_ph_test_optional_call_port f
    b_comment = false

    if @singleton then
      if has_INIB? then
        inib = "INIB"
      else
        inib = "CB"
      end
    else
      if has_CB? && has_INIB? then
        inib = "->_inib"
      else
        inib = ""
      end
    end

    @port.each { |p|
      next if p.get_port_type != :CALL
      next if ! p.is_optional?
      # next if p.is_omit?  # omit でも test コードは生成する

      if b_comment == false then
        f.printf( TECSMsg.get( :TOCP_comment ), "#_TOCP_#" )
        b_comment = true
      end

      if @singleton then
        param = ""
        delim = ""
      else
        param = "p_that"
        delim = ","
      end
      if p.get_array_size != nil then
        param = param + delim + "subscript"
      end
      f.print( "#define #{@global_name}_is_#{p.get_name}_joined(#{param}) \\\n" )

      if p.is_omit? then
        f.print( "   omit  is_#{p.get_name}_joined\n" )
        next
      end

      # 関数名の出力(標準：受け口ディスクリプタから VMT の関数名、最適化：受け口関数 or 受け口ディスクリプタ)
      # mikan  全部つながっているかどうかで (1) を判定する
      if ! p.is_VMT_useless? then
        if p.is_dynamic? then
          if @singleton then
            inib_tmp = "CB"
          else
            inib_tmp = ""
          end
        else
          inib_tmp = inib
        end

        # 標準コード
        if p.get_array_size == nil then
          if @singleton then
            f.print( "\t  (#{@global_name}_SINGLE_CELL_#{inib_tmp}.#{p.get_name}!=0)\n" )
          else
            f.print( "\t  ((p_that)#{inib_tmp}->#{p.get_name}!=0)\n" )
          end
        else
          # 配列の場合
          if @singleton then
            f.print( "\t  ((#{@global_name}_SINGLE_CELL_#{inib_tmp}.#{p.get_name}!=0) \\\n" )
            f.print( "\t  &&(#{@global_name}_SINGLE_CELL_#{inib_tmp}.#{p.get_name}[subscript]!=0))\n" )
          else
            f.print( "\t  (((p_that)#{inib_tmp}->#{p.get_name}!=0)\\\n" )
            f.print( "\t  &&((p_that)#{inib_tmp}->#{p.get_name}[subscript]!=0))\n" )
          end
        end
      else
        # 最適化コード (optimize) # VMT 不要（配列要素すべて同じ）
        p2 = p.get_real_callee_port
        if p2 then
          ct = p2.get_celltype
          if p.is_skelton_useless? then
            # 受け口関数を直接呼出す
            f.print( "\t  (1)\n" )
          else
            # 受け口スケルトン関数を直接呼出す
            f.print( "\t  (1)\n" )
          end
        else
          # optional で未結合
          f.print( "\t  (0)    /* not joined */\n" )
        end
      end
    }
  end

  #=== optional な呼び口が結合されているかテストするコードの生成（短縮形）
  def gen_ph_test_optional_call_port_abbrev f
    b_comment = false

    @port.each { |p|
      next if p.get_port_type != :CALL
      next if ! p.is_optional?
      # next if p.is_omit?  # omit でも test コードは生成する

      if b_comment == false then
        f.printf( TECSMsg.get( :TOCPA_comment ), "#_TOCPA_#" )
        b_comment = true
      end

      if @singleton then
        param = ""
        delim = ""
      else
        param = "p_cellcb"
        delim = ","
      end

      if p.get_array_size == nil then
        subscript = ""
      else
        subscript = "subscript"
        param = param + delim + subscript
      end
      f.print( "#define is_#{p.get_name}_joined(#{subscript})\\\n\t\t#{@global_name}_is_#{p.get_name}_joined(#{param})\n" )
    }
  end

  #=== CELLCB へのポインタを得るマクロを出力
  #   セルタイプヘッダへ出力
  def gen_ph_get_cellcb f
    f.printf( TECSMsg.get( :GCB_comment ), "#_GCB_#" )
    if ( ! has_CB? && ! has_INIB? ) || @singleton then
      f.print( "#define #{@global_name}_GET_CELLCB(idx) ((void *)0)\n" )
    elsif @idx_is_id_act then   # mikan 単一のセルの場合の最適化, idx_is_id でない場合
      f.print( "#define #{@global_name}_GET_CELLCB(idx) (#{@global_name}_CB_tab[(idx) - #{@global_name}_ID_BASE])\n" )
    else
      f.print( "#define #{@global_name}_GET_CELLCB(idx) (idx)\n" )
    end
  end

  #=== CELLCB へのポインタを得るマクロ（短縮形）を出力
  #  セルタイプヘッダへ出力
  def gen_ph_get_cellcb_abbrev f
    f.printf( TECSMsg.get( :GCBA_comment ), "#_GCBA_#" )
    f.print("#define GET_CELLCB(idx)  #{@global_name}_GET_CELLCB(idx)\n\n")

    f.printf( TECSMsg.get( :CCT_comment ), "#_CCT_#" )
    f.print( "#define CELLCB\t#{@global_name}_CB\n\n" )

    f.printf( TECSMsg.get( :CTIXA_comment ), "#_CTIXA_#" )
    f.print( "#define CELLIDX\t#{@global_name}_IDX\n\n" )

    if @name != @global_name then
      f.print( "#define #{@name}_IDX  #{@global_name}_IDX\n" )
    end
  end

  #===  attribute, var をアクセスするマクロを出力
  #    セルタイプヘッダへ出力
  def gen_ph_attr_access f
    if @n_attribute_rw > 0 || @n_attribute_ro > 0 then
      f.printf( TECSMsg.get( :AAM_comment ), "#_AAM_#" )
    end

    @attribute.each { |a|

      next if a.is_omit?

      # mikan const_value の場合
      f.print( "#define " )
      if @singleton then
        if has_INIB? then
          inib = "INIB"
        else
          inib = "CB"
        end
        f.printf( "%-20s", "#{@global_name}_ATTR_#{a.get_name}" )
        f.print( "\t(#{@global_name}_SINGLE_CELL_#{inib}.#{a.get_name})\n" )
        # mikan ここでは cell ではなく celltype の名前
      else
        if ! a.is_rw? && has_CB? && has_INIB? then
          inib = "->_inib"
        else
          inib = ""
        end
        f.printf( "%-20s", "#{@global_name}_ATTR_#{a.get_name}( p_that )" )
        f.print( "\t((p_that)#{inib}->#{a.get_name})\n" )
      end
    }
    f.print( "\n" )

    @attribute.each { |a|

      next if a.is_omit?

      if @singleton then
        if has_INIB? then
          inib = "INIB"
        else
          inib = "CB"
        end
      else
        if ! a.is_rw? && has_CB? && has_INIB? then
          inib = "->_inib"
        else
          inib = ""
        end
      end

      # mikan const_value の場合
      f.print( "#define " )
      if @singleton then
        f.printf( "%-20s", "#{@global_name}_GET_#{a.get_name}()" )
        f.print( "\t(#{@global_name}_SINGLE_CELL_#{inib}.#{a.get_name})\n" )
        # mikan ここでは cell ではなく celltype の名前
      else
        f.printf( "%-20s", "#{@global_name}_GET_#{a.get_name}(p_that)" )
        f.print( "\t((p_that)#{inib}->#{a.get_name})\n" )
      end

      if a.is_rw? then
        f.print( "#define " )
        if @singleton then
          f.printf( "%-20s", "#{@global_name}_SET_#{a.get_name}(val)" )
          f.print( "\t(#{@global_name}_SINGLE_CELL_#{inib}.#{a.get_name} = (val))\n" )
          # mikan ここでは cell ではなく celltype の名前
        else
          f.printf( "%-20s", "#{@global_name}_SET_#{a.get_name}(p_that,val)" )
          f.print( "\t((p_that)#{inib}->#{a.get_name}=(val))\n" )
        end
      end

    }
    f.print( "\n" )

    if @n_var > 0 then
      f.printf( TECSMsg.get( :VAM_comment ), "#_VAM_#" )
    end

    @var.each { |v|

      next if v.is_omit?

      if @singleton then
        if v.get_size_is && has_INIB? then
          inib = "INIB"
        else
          inib = "CB"
        end
      else
        if v.get_size_is && has_CB? && has_INIB? then
          inib = "->_inib"
        else
          inib = ""
        end
      end

      # mikan const_value の場合
      f.print( "#define " )
      if @singleton then
        f.printf( "%-20s", "#{@global_name}_VAR_#{v.get_name}" )
        f.print( "\t(#{@global_name}_SINGLE_CELL_#{inib}.#{v.get_name})\n" )
        # mikan ここでは cell ではなく celltype の名前
      else
        f.printf( "%-20s", "#{@global_name}_VAR_#{v.get_name}(p_that)" )
        f.print( "\t((p_that)#{inib}->#{v.get_name})\n" )
      end
    }
    f.print( "\n" )
    @var.each { |v|

      next if v.is_omit?

      # mikan const_value の場合
      f.print( "#define " )
      if @singleton then
        f.printf( "%-20s", "#{@global_name}_GET_#{v.get_name}()" )
        f.print( "\t(#{@global_name}_SINGLE_CELL_CB.#{v.get_name})\n" )
        # mikan ここでは cell ではなく celltype の名前
      else
        f.printf( "%-20s", "#{@global_name}_GET_#{v.get_name}(p_that)" )
        f.print( "\t((p_that)->#{v.get_name})\n" )
      end

      f.print( "#define " )
      if @singleton then
        f.printf( "%-20s", "#{@global_name}_SET_#{v.get_name}(val)" )
        f.print( "\t(#{@global_name}_SINGLE_CELL_CB.#{v.get_name}=(val))\n" )
        # mikan ここでは cell ではなく celltype の名前
      else
        f.printf( "%-20s", "#{@global_name}_SET_#{v.get_name}(p_that,val)" )
        f.print( "\t((p_that)->#{v.get_name}=(val))\n" )
      end
    }
    f.print( "\n" )

  end

  #===  attribute/var アクセスマクロ（短縮形）コードの生成
  def gen_ph_attr_access_abbrev f
    if @n_attribute_rw > 0 || @n_attribute_ro > 0 then
      f.printf( TECSMsg.get( :AAMA_comment ), "#_AAMA_#" )
    end

    @attribute.each { |a|
      next if a.is_omit?

      # mikan const_value の場合
      f.print( "#define " )
      f.printf( "%-20s", "ATTR_#{a.get_name}" )
      f.print( " #{@global_name}_ATTR_#{a.get_name}" )
      if ! @singleton then
        f.print( "( p_cellcb )" )
      end
      f.print "\n"
    }
    f.print( "\n" )

    if @n_var > 0 then
      f.printf( TECSMsg.get( :VAMA_comment ), "#_VAMA_#" )
    end

    @var.each { |v|
      next if v.is_omit?

      # mikan const_value の場合
      f.print( "#define " )
      f.printf( "%-20s", "VAR_#{v.get_name}" )
      f.print( " #{@global_name}_VAR_#{v.get_name}" )
      if ! @singleton then
        f.print( "( p_cellcb )" )
      end
      f.print( "\n" )
    }
    f.print( "\n" )

  end

  def gen_ph_cp_fun_macro f
    if @n_call_port >0 then
      f.printf( TECSMsg.get( :CPM_comment ) , "#_CPM_#" )
    end

    @port.each { |p|
      next if p.get_port_type != :CALL
      next if p.is_omit?

      p.get_signature.get_function_head_array.each{ |fun|
        if @singleton then
          if has_INIB? then
            inib = "INIB"
          else
            inib = "CB"
          end
          if p.is_dynamic? && p.get_array_size == nil then
            # dynamic call port (not array)
            inib = "CB"
          end
        else
          if has_CB? && has_INIB? then
            inib = "->_inib"
          else
            inib = ""
          end
          if p.is_dynamic? && p.get_array_size == nil then
            # dynamic call port (not array)
            inib = ""
          end
        end

        f.print( "#define #{@global_name}_#{p.get_name}_#{fun.get_name}(" )
        ft = fun.get_declarator.get_type
        delim = ""

        if ! @singleton then
          f.print( "#{delim} p_that" )
          delim = ","
        end

        if p.get_array_size then
          f.print( "#{delim} subscript" )
          delim = ","
        end

        ft.get_paramlist.get_items.each{ |param|
          f.print( "#{delim} #{param.get_name}" )
          delim = ","
        }
        f.print( " ) \\\n" )

        subsc = ""
        subsc = "[subscript]" if p.get_array_size
        delim = ""

        # 関数名の出力(標準：受け口ディスクリプタから VMT の関数名、最適化：受け口関数 or 受け口ディスクリプタ)
        if ! p.is_VMT_useless? then
          # 標準コード
          if @singleton then
            f.print( "\t  #{@global_name}_SINGLE_CELL_#{inib}.#{p.get_name}" )
          else
            f.print( "\t  (p_that)#{inib}->#{p.get_name}" )
          end
          f.print( "#{subsc}->VMT->#{fun.get_name}__T( \\\n" )
        else
          # 最適化コード (optimize) # VMT 不要
          p2 = p.get_real_callee_port
          if p2 then
            ct = p2.get_celltype
            if p.is_skelton_useless? then
              # 受け口関数を直接呼出す
              f.print( "\t  #{ct.get_global_name}_#{p2.get_name}_#{fun.get_name}( \\\n" )
            else
              # 受け口スケルトン関数を直接呼出す
              f.print( "\t  #{ct.get_global_name}_#{p2.get_name}_#{fun.get_name}_skel( \\\n" )
              # print "skelton: #{@name} #{ct.get_global_name}_#{p2.get_name}\n"
            end
          else
            # optional で未結合
            f.print( "\t  ((#{fun.get_declarator.get_type.get_type.get_type_str} (*)()" )
            f.print( "#{fun.get_declarator.get_type.get_type.get_type_str_post})0)()\n" )
            f.print( "\t  /* optional no entry port joined */\n" )
            if ! p.is_optional? then
              raise "unjoined but not optional celltype: #{@name} #{p.get_name}"
            end
          end
        end

        b_join = true    # optional で結合していない場合 false

        # 受け口情報の出力(標準：受け口ディスクリプタ、最適化：IDX など)
        if ! p.is_skelton_useless? && ! p.is_cell_unique? then
          # 標準コード
          if @singleton then
            f.print( "\t  #{@global_name}_SINGLE_CELL_#{inib}.#{p.get_name}#{subsc}" )
            delim = ","
          else
            f.print( "\t   (p_that)#{inib}->#{p.get_name}#{subsc}" )
            delim = ","
          end
        else
          # 最適化コード (optimize) # スケルトン不要
          c2 = p.get_real_callee_cell               # 唯一のセル(でない場合もある、複数セルがある場合)
          p2 = p.get_real_callee_port               # 唯一のポート(でない場合は、ない)
          if p2 then
            ct = p2.get_celltype                    # 呼び先のセルタイプ
            if ! ct.is_singleton? then
              if ct.has_CB? || ct.has_INIB? then
                if p.is_cell_unique? then
                  name_array = ct.get_name_array( c2 )
                  f.print( "\t   #{name_array[7]}" )
                else
                  # CELLCB IDX を渡す (標準コードと同じだが、扱う型は異なる)
                  # p.is_skelton_useless? == true/false ともに同じ
                  f.print( "\t   (p_that)#{inib}->#{p.get_name}#{subsc}" )
                end
              else
                f.print( "\t   (#{ct.get_global_name}_IDX)0" )
              end
              delim = ","
            else
              f.print( "\t   " )
            end
          else
            # optional で未結合
            b_join = false
          end
        end

        if b_join then
          ft.get_paramlist.get_items.each{ |param|
            f.print( "#{delim} (#{param.get_name})" )
            delim = ","
          }
          f.print( " )\n" )
        end
      }
    }
    f.print( "\n" )
  end

  #=== ref_desc 指定された呼び口に対するディスクリプタ参照関数の生成
  def gen_ph_ref_desc_func f
    if @n_call_port_ref_desc >0 then
      f.printf( TECSMsg.get( :CRD_comment ), "#_CRD_#" )
    end

    if has_CB? && has_INIB? then
      inib = "->_inib"
    else
      inib = ""
    end
    @port.each { |p|
      next if p.get_port_type != :CALL
      next if ! p.is_ref_desc?

      if @singleton then
        p_that = ""
        p_cellcb = ""
        delim = ""
        if has_INIB? then
          cb = "#{@global_name}_SINGLE_CELL_INIB."
        else
          cb = "#{@global_name}_SINGLE_CELL_CB."
        end
      else
        p_that = "#{@global_name}_CB  *p_that"
        p_cellcb = "    #{@global_name}_CB *p_cellcb = p_that;\n"
        delim = ", "
        cb = "p_that#{inib}->"
      end

      if p.get_array_size
        array = "#{delim}int_t  i "
        array2 = "[ i ]"
        assert = "    assert( 0 <= i && i < NCP_#{p.get_name} );\n"
      else
        array = ""
        array2 = ""
        assert = ""
      end
      f.print <<EOT
/* [ref_desc] #{p.get_name} */
Inline Descriptor( #{p.get_signature.get_global_name} )
#{@global_name}_#{p.get_name}_refer_to_descriptor( #{p_that}#{array} )
{
    Descriptor( #{p.get_signature.get_global_name} )  des;
#{p_cellcb}    /* cast is ncecessary for removing 'const'  */
#{assert}    des.vdes = (struct tag_#{p.get_signature.get_global_name}_VDES *)#{cb}#{p.get_name}#{array2};
    return des;
}

EOT
    }
  end

  #=== dynamic 指定された呼び口に対するディスクリプタ設定関数の生成
  def gen_ph_set_desc_func f
    if @n_call_port_dynamic >0 then
      f.printf( TECSMsg.get( :SDF_comment ), "#_SDF_#" )
    end

    @port.each { |p|
      next if p.get_port_type != :CALL
      next if ! p.is_dynamic?
      if has_CB? && has_INIB? && p.get_array_size then
        inib = "->_inib"
      else
        inib = ""
      end
      if @singleton then
        # p "main== #{@global_name} #{p.get_name} #{p.get_array_size}"
        p_that = ""
        p_that2 = ""
        p_cellcb = ""
        if p.get_array_size && $rom then
          cb = "#{@global_name}_SINGLE_CELL_INIB."
        else
          cb = "#{@global_name}_SINGLE_CELL_CB."
        end
      else
        p_that = "#{@global_name}_CB  *p_that, "
        p_that2 = "#{@global_name}_CB  *p_that "
        p_cellcb = "    #{@global_name}_CB *p_cellcb = p_that;\n"
        cb = "(p_cellcb)->#{inib}"
      end

      if p.get_array_size then
        array = "int_t  i, "
        array2 = "[ i ]"
        array3 = " int_t  i "
        assert2 = "    assert( 0 <= i && i < NCP_#{p.get_name} );\n"
      else
        array = ""
        array2 = ""
        array3 = ""
        assert2 = ""
      end
      f.print <<EOT
/* [dynamic] #{p.get_name} */
Inline void
#{@global_name}_#{p.get_name}_set_descriptor( #{p_that}#{array}Descriptor( #{p.get_signature.get_global_name} ) des )
{
#{p_cellcb}    assert( des.vdes != NULL );
#{assert2}    #{cb}#{p.get_name}#{array2} = des.vdes;
}

EOT

      if p.is_optional? then
        if p_that2 != "" && array3 != "" then
          delim = ", "
        else
          delim = ""
        end
        f.print <<EOT
/* [dynamic,optional] #{p.get_name} */
Inline void
#{@global_name}_#{p.get_name}_unjoin( #{p_that2}#{delim}#{array3} )
{
#{p_cellcb}    #{cb}#{p.get_name}#{array2} = NULL;
}

EOT
      end
    }
  end

  #=== send/receive で受け取ったメモリ領域を dealloc するマクロコード
  #f:: File
  #b_undef:: bool : true = #undef コードの生成,  false = #define コードの生成
  def  gen_ph_dealloc_code( f, append_name, b_undef = false )
    b_msg = false
    @port.each{ |p|
      next if p.is_omit?

      p.each_param{ |port, fd, par|
        case par.get_direction                        # 引数の方向指定子 (in, out, inout, send, receive )
        when :SEND
          # next if port.get_port_type == :CALL
          type = par.get_declarator.get_type
          pre = "("
          post = ")"
        when :RECEIVE
          # next if port.get_port_type == :ENTRY
          type = par.get_declarator.get_type.get_type
#          pre = "(*"
#          post = ")"
          pre = "("
          post = ")"
        else
          next
        end

        #                      ポート名         関数名         パラメータ名
        dealloc_func_name = "#{port.get_name}_#{fd.get_name}_#{par.get_name}_dealloc"
        dealloc_macro_name = dealloc_func_name.upcase
        name = par.get_name

        if b_undef == false
          if (type.get_size || type.get_count) && type.get_type.has_pointer?
            count_str = "count__"
            count_str2 = ", count__"
          else
            count_str = nil
            count_str2 = nil
          end
          if ! b_msg
            f.print "\n"
            f.printf TECSMsg.get( :DAL_comment ), "#_DAL_#  #{append_name}"
            b_msg = true
          end
          f.print "#define #{dealloc_macro_name}#{append_name}(#{name}#{count_str2})"
          if append_name == "_RESET" then
            gen_dealloc_code_for_type( f, type, dealloc_func_name, pre, name, post, 0, true, count_str )
          else
            gen_dealloc_code_for_type( f, type, dealloc_func_name, pre, name, post, 0, false, count_str )
          end
          f.print "\n"
        else
          f.print "#undef #{dealloc_macro_name}#{append_name}\n"
        end
      }
    }
  end

  def gen_ph_cp_fun_macro_abbrev f
    if @n_call_port >0 then
      f.printf( TECSMsg.get( :CPMA_comment ), "#_CPMA_#" )
    end

    @port.each { |p|
      next if p.get_port_type != :CALL
      # next if p.is_omit?  呼び出すとエラーを起こすコードを生成

      p.get_signature.get_function_head_array.each{ |fun|
        if p.is_VMT_useless? && ! @singleton then
          dummy_p_cell_access_pre = "((void)p_cellcb, "
          dummy_p_cell_access_post = ")"
        else
          dummy_p_cell_access_pre = ""
          dummy_p_cell_access_post = ""
        end
 
        if ! p.is_require? || p.has_name? then
          f.print( "#define #{p.get_name}_#{fun.get_name}(" )
        else
          f.print( "#define #{fun.get_name}(" )
        end
        ft = fun.get_declarator.get_type
        delim = ""

#        if ! @singleton then
#          f.print( "#{delim} p_that" )
#          delim = ","
#        end

        if p.get_array_size then
          f.print( "#{delim} subscript" )
          delim = ","
        end

        ft.get_paramlist.get_items.each{ |param|
          f.print( "#{delim} #{param.get_name}" )
          delim = ","
        }
        f.print( " ) \\\n" )

        if p.is_omit? then
          f.print( "          #{dummy_p_cell_access_pre}omitted #{p.get_name}_#{fun.get_name}(" )
        else
          f.print( "          #{dummy_p_cell_access_pre}#{@global_name}_#{p.get_name}_#{fun.get_name}(" )
        end
        ft = fun.get_declarator.get_type
        delim = ""

        if ! @singleton then
          f.print( "#{delim} p_cellcb" )
          delim = ","
        end

        if p.get_array_size then
          f.print( "#{delim} subscript" )
          delim = ","
        end

        ft.get_paramlist.get_items.each{ |param|
          f.print( "#{delim} #{param.get_name}" )
          delim = ","
        }
        f.print( " )#{dummy_p_cell_access_post}\n" )
      }
    }
    f.print( "\n" )
  end

  def gen_ph_ref_desc_macro_abbrev f
    if @n_call_port_ref_desc >0 then
      f.printf( TECSMsg.get( :CRDA_comment ), "#_CRDA_#" )
    end

    @port.each { |p|
      next if p.get_port_type != :CALL
      next if ! p.is_ref_desc?

      if @singleton then
        p_cellcb = ""
        delim = ""
      else
        p_cellcb = "p_cellcb"
        delim = ", "
      end

      if p.get_array_size then
        array = " i "
        array2 = "#{delim}i"
      else
        array = ""
        array2 = ""
      end

      f.printf( "#define %s_refer_to_descriptor(#{array})\\\n          %s_refer_to_descriptor( #{p_cellcb}#{array2} )\n",
                "#{p.get_name}",
                "#{@global_name}_#{p.get_name}" )
      f.printf( "#define %s_ref_desc(#{array})\\\n          %s_refer_to_descriptor(#{array})\n",   
                "#{p.get_name}",
                "#{p.get_name}" )
    }
    f.print( "\n" )
  end

  def gen_ph_set_desc_macro_abbrev f
    if @n_call_port_dynamic >0 then
      f.printf( TECSMsg.get( :SDMA_comment ), "#_SDMA_#" )
    end

    if @singleton then
      p_cellcb = ""
      delim = ""
    else
      p_cellcb = "p_cellcb"
      delim = ", "
    end
    @port.each { |p|
      next if p.get_port_type != :CALL
      next if ! p.is_dynamic?

      if p.get_array_size then
        subsc = "i, "
        subsc2 = "i"
        subsc3 = delim + subsc2
      else
        subsc = ""
        subsc2 = ""
        subsc3 = ""
      end
      f.printf( "#define %s_set_descriptor( #{subsc}desc )\\\n          %s_set_descriptor( #{p_cellcb}#{delim}#{subsc}desc )\n",
                "#{p.get_name}",
                "#{@global_name}_#{p.get_name}" )
      f.printf( "#define %s_unjoin( #{subsc2} )\\\n          %s_unjoin( #{p_cellcb}#{subsc3} )\n",
                "#{p.get_name}",
                "#{@global_name}_#{p.get_name}" )
    }
    f.print( "\n" )
  end

  def gen_ph_ep_fun_macro f
    if @n_entry_port >0 then
      f.printf( TECSMsg.get( :EPM_comment ), "#_EPM_#" )
    end

    @port.each { |p|
      next if p.get_port_type != :ENTRY
      next if p.is_omit?

      p.get_signature.get_function_head_array.each{ |fun|
        f.printf( "#define %-16s %s\n",
                  "#{p.get_name}_#{fun.get_name}",
                  "#{@global_name}_#{p.get_name}_#{fun.get_name}" )
      }
    }
    f.print( "\n" )

  end

  def gen_ph_typedef_idx f
    f.printf( TECSMsg.get( :CTIX_comment ), "#_CTIX_#" )
    if @idx_is_id_act then
      f.print( "typedef ID #{@global_name}_IDX;\n" )
    else
      if has_CB? then
        f.print( "typedef struct tag_#{@global_name}_CB *#{@global_name}_IDX;\n" )
      elsif has_INIB? then
        f.print( "typedef const struct tag_#{@global_name}_INIB *#{@global_name}_IDX;\n" )
      else
        f.print( "typedef int   #{@global_name}_IDX;\n" )
      end
    end

  end

  def gen_ph_idx_type f
    if @idx_is_id_act then
      f.print( "ID" )
    else
      if has_CB? then
        f.print( "struct tag_#{@global_name}_CB *" )
      elsif has_INIB? then
        # f.print( "struct tag_#{@global_name}_INIB *" )  # const を出力していない
        f.print( "const struct tag_#{@global_name}_INIB *" )
      else
        f.print( "int" )
      end
    end

  end

  def gen_ph_ep_fun_prototype f
    if @n_entry_port >0 then
      f.printf( TECSMsg.get( :EPP_comment ), "#_EPP_#" )
    end

    @port.each { |p|
      next if p.get_port_type != :ENTRY
      next if p.is_omit?

      f.print( "/* #{p.get_signature.get_global_name} */\n" )

      p.get_signature.get_function_head_array.each{ |fun|
        if p.is_inline? then
          f.print( "Inline " )
        end
        functype = fun.get_declarator.get_type
        f.printf( "%-12s", functype.get_type_str )

        f.print( " #{@global_name}_#{p.get_name}_#{fun.get_name}(" )
        if @singleton then
           delim = ""
        else
          f.print( "#{@global_name}_IDX idx" )
          delim = ","
        end

        if p.get_array_size then
          f.print( "#{delim} int_t subscript" )     # mikan singleton 時の ',' の始末
          delim = ","
        end

        if functype.get_paramlist then
          items = functype.get_paramlist.get_items
          len = items.length
        else
          # ここで nil になるのは、引数なしの時に void がなかった場合
          items = []
          len = 0
        end
        i = 0
        items.each{ |param|
          f.print "#{delim} "
          delim = ","
          f.print( param.get_type.get_type_str )
          f.print( " " )
          f.print( param.get_name )
          f.print( param.get_type.get_type_str_post )
          i += 1
        }
        f.print( ");\n" )

      }
    }
  end

  def gen_ph_ep_skel_prototype f
    # 受け口スケルトン関数のプロトタイプ宣言を出力
    if @n_entry_port >0 then
      f.printf( TECSMsg.get( :EPSP_comment ), "#_EPSP_#" )
    end
    @port.each { |p|
      next if p.get_port_type != :ENTRY
      next if p.is_omit?
#      if p.is_skelton_useless? || ! p.is_VMT_useless? then    # 受け口最適化
      if p.is_skelton_useless? then    # 受け口最適化
#        f.print( "/* #{p.get_name} : omitted by entry port optimize */\n" )
        next
      end

      f.print( "/* #{p.get_name} */\n" )

      p.get_signature.get_function_head_array.each{ |fun|
        functype = fun.get_declarator.get_type
        f.printf "%-14s", functype.get_type_str

        f.print " #{@global_name}_#{p.get_name}_#{fun.get_name}_skel("
        f.print " const struct tag_#{p.get_signature.get_global_name}_VDES *epd"
        delim = ","

        if functype.get_paramlist then
          items = functype.get_paramlist.get_items
          len = items.length
        else
          # ここで nil になるのは、引数なしの時に void がなかった場合
          items = []
          len = 0
        end
        i = 0
        items.each{ |param|
          f.print "#{delim} "
          delim = ","
          f.print param.get_type.get_type_str
          f.print " "
          f.print param.get_name
          f.print param.get_type.get_type_str_post
          i += 1
        }
        f.print ");\n"
      }
    }
    f.print( "\n" )
  end

  def gen_ph_cell_cb_type f

    if ( $rom )then
      # 定数部は ROM, 変数部は RAM

      if has_INIB? then

        f.printf( TECSMsg.get( :CIP_comment ), "#_CIP_#" )
        f.print( "typedef const struct tag_#{@global_name}_INIB {\n" )

        gen_cell_cb_type_port( f, :INIB )
        gen_cell_cb_type_attribute( f, :INIB )

        f.print( "}  #{@global_name}_INIB;\n" )

      end

      if has_CB? then
        f.printf( TECSMsg.get( :CCTPA_comment ), "#_CCTPA_#" )
        f.print( "typedef struct tag_#{@global_name}_CB {\n" )
        if has_INIB? then
          f.print "    #{@global_name}_INIB  *_inib;\n"
        end
        gen_cell_cb_type_port( f, :CB_DYNAMIC )
        gen_cell_cb_type_attribute( f, :CB )
        gen_cell_cb_type_var f
        f.print( "}  #{@global_name}_CB;\n" )
      end

      if ! has_CB? && ! has_INIB? then
        f.printf( TECSMsg.get( :CCDP_comment ), "#_CCDP_#" )
        f.print( "typedef struct tag_#{@global_name}_CB {\n" )
        f.print( "    int  dummy;\n" )
        f.print( "} #{@global_name}_CB;\n" )
     end

    else
      # 全て RAM
      f.printf( TECSMsg.get( :CCTPO_comment ), "#_CCTPO_#" )

      f.print( "typedef struct tag_#{@global_name}_CB {\n" )

      gen_cell_cb_type_port( f, :CB )
      gen_cell_cb_type_attribute( f, :CB )
      gen_cell_cb_type_var f

      f.print( "}  #{@global_name}_CB;\n" )
    end
  end


  #===   attribute の型宣言出力
  #inib_cb::  :INIB または :CB
  def gen_cell_cb_type_attribute( f, inib_cb )
    if inib_cb == :INIB && @n_attribute_ro > 0 then
      f.print "    /* attribute(RO) #_ATO_# */ \n"
    elsif inib_cb == :CB then
      if $rom then
        if @n_attribute_rw > 0 then
          f.print "    /* attribute(RW) #_ATW_# */ \n"
        end
      else
        if @n_attribute_rw > 0 || @n_attribute_ro > 0 then
          f.print "    /* attribute #_AT_# */ \n"
        end
      end
    end

    @attribute.each{ |a|
      next if a.is_omit?
      next if inib_cb == :INIB && a.is_rw?
      next if has_INIB? && inib_cb == :CB && ! a.is_rw?

      if a.get_type.kind_of?( PtrType ) && ! a.get_type.is_const? && a.get_size_is then
        const_str = "const "
      else
        const_str = ""
      end
      f.print "    "
      f.printf( "#{const_str}%-14s", a.get_type.get_type_str )
      f.print " #{a.get_name}#{a.get_type.get_type_str_post};\n"
    }
    @var.each { |v|
      next if v.is_omit?
      next if v.get_size_is == nil
      next if $rom && inib_cb == :CB      # size_is 指定されたものは INIB にのみ出力する

      f.print "    "
      f.printf( "%-14s", v.get_type.get_type_str )
      f.print " #{v.get_name}#{v.get_type.get_type_str_post};\n"
    }
  end

  def gen_cell_cb_type_var f
    # 変数の出力
    if @n_var > 0 then
      f.print "    /* var #_VA_# */ \n"
    end

    @var.each{ |v|

      next if v.is_omit?
      next if v.get_size_is != nil    # size_is 指定された var は attribute へ出力する

      f.print "    "
      f.printf( "%-14s", v.get_type.get_type_str )
      f.print " #{v.get_name}#{v.get_type.get_type_str_post};\n"
    }
  end

  def gen_cell_cb_type_port( f, inib_cb )
    gen_cell_cb_type_call_port( f, inib_cb )
    gen_cell_cb_type_entry_port( f, inib_cb )
  end

  def gen_cell_cb_type_call_port( f, inib_cb )
    # 呼び口
    if @n_call_port >0 then
      f.print "    /* call port #_TCP_# */\n"
    end

    @port.each{ |p|
      next if p.get_port_type != :CALL
      next if p.is_omit?
      next if inib_cb == :INIB && p.is_dynamic? && p.get_array_size == nil && ! $ram_initializer
      next if inib_cb == :CB_DYNAMIC && ( ! p.is_dynamic? || p.get_array_size != nil )
      # bprint "cb_type #{inib_cb} #{p.get_name} dynamic=#{p.is_dynamic?}\n"

      ptr = p.get_array_size ? '*' : ''

      if ! p.is_cell_unique? then
        const = p.is_dynamic?  ? '' : 'const'
        if inib_cb == :INIB && p.is_dynamic? && p.get_array_size == nil && $ram_initializer then
          init = '_init_'
          const2 = 'const'
        else
          init = ''
          const2 = 'const'
        end

        if ! p.is_skelton_useless? then
          # 標準形
          if inib_cb == :INIB && p.is_dynamic? && p.get_array_size != nil && $ram_initializer then
            f.print( "    struct tag_#{p.get_signature.get_global_name}_VDES #{ptr}#{const2}*#{p.get_name;}_init_;\n" )
          end
          f.print( "    struct tag_#{p.get_signature.get_global_name}_VDES #{ptr}#{const}*#{p.get_name;}#{init};\n" )
#          f.print( "    struct tag_#{p.get_signature.get_global_name}_VDES #{ptr}*#{p.get_name;};\n" )
          if p.get_array_size == "[]" then
            f.print( "    int_t n_#{p.get_name};\n" )
          end
        else
          # 最適化 skelton 関数を呼出さない(受け口関数を直接呼出す)
          # 呼び先セルタイプの CB の IDX 型
          if p.get_real_callee_cell then
            f.print( "    " )
            p.get_real_callee_cell.get_celltype.gen_ph_idx_type f
            f.print( " #{ptr}#{p.get_name;};\n" )
            # 相互参照に備えて、typedef した型を使わない
            # f.print( "    #{p.get_real_callee_cell.get_celltype.get_global_name}_IDX #{ptr}#{p.get_name;};\n" )
            if p.get_array_size == "[]" then
              f.print( "    int_t n_#{p.get_name};\n" )
            end
          #else
          #  optional で未結合
          end
        end
      # else
        # 最適化 一つしかセルがない場合、受け口ディスクリプタまたは受け側の IDX は呼び口関数マクロに埋め込まれる
      end
    }
  end

  #=== Celltype#受け口配列添数を記憶する変数の定義
  def gen_cell_cb_type_entry_port( f, inib_cb )
    # 呼び口
    if @n_entry_port >0 then
      f.print "    /* call port #_NEP_# */ \n"
    end

    @port.each{ |p|
      # next if p.is_omit?                       # 受け口配列の個数は省略しない
      if p.get_port_type == :ENTRY && p.get_array_size == "[]"
        f.print( "    int_t n_#{p.get_name};\n" )
      end
    }
  end

  def gen_ph_extern_cell f
    if @singleton then
      f.printf( TECSMsg.get( :SCP_comment ),  "#_SCP_#" )
      if has_CB? then
        f.print "extern  #{@global_name}_CB  #{@global_name}_SINGLE_CELL_CB;\n"
      end
      if has_INIB? then
        f.print "extern  #{@global_name}_INIB  #{@global_name}_SINGLE_CELL_INIB;\n"
      end

#     @ordered_cell_list.each{ |c|
#        f.print "extern  #{@global_name}_CB  #{@global_name}_#{c.get_name}_CB;\n"
#      }

      f.print "\n"
    elsif @idx_is_id_act then
      f.print "extern #{@global_name}_CB  *#{@global_name}_CB_tab[];\n"
    else
      f.print "extern #{@global_name}_CB  #{@global_name}_CB_tab[];\n"
    end
  end

  def gen_ph_INIB_as_CB f

    if ! has_CB? && has_INIB? then
      f.printf( TECSMsg.get( :DCI_comment ),  "#_DCI_#" )
      f.print "#define #{@global_name}_CB_tab           #{@global_name}_INIB_tab\n"
      f.print "#define #{@global_name}_SINGLE_CELL_CB   #{@global_name}_SINGLE_CELL_INIB\n"
      f.print "#define #{@global_name}_CB               #{@global_name}_INIB\n"
      f.print "#define tag_#{@global_name}_CB           tag_#{@global_name}_INIB\n"
      f.print "\n"
    end

  end

  #===  イテレータコード (FOREACH_CELL)の生成
  #      singleton では出力しない
  def gen_ph_foreach_cell f

    return if @singleton

    if has_CB? || has_INIB? then

      if need_CB_initializer?
        necessity = ""
      else
        necessity = "//"
      end

      f.printf( TECSMsg.get( :FEC_comment ), "#_FEC_#" )

      if @idx_is_id_act then
        amp = ''
      else
        amp = '&'
      end
      f.print <<EOT
#define FOREACH_CELL(i,p_cb)   \\
    for( (i) = 0; (i) < #{@global_name}_N_CELL; (i)++ ){ \\
       #{necessity}(p_cb) = #{amp}#{@global_name}_CB_tab[i];

#define END_FOREACH_CELL   }

EOT
    else
      f.printf( TECSMsg.get( :NFEC_comment ), "#_NFEC_#" )
      f.print <<EOT
#define FOREACH_CELL(i,p_cb)   \\
    for((i)=0;(i)<0;(i)++){

#define END_FOREACH_CELL   }

EOT
    end
  end


  #===  変数var初期化コード
  #
  def gen_ph_cb_initialize_macro f

    f.printf( TECSMsg.get( :CIM_comment ), "#_CIM_#" )

    @var.each { |v|
      init = v.get_initializer
      if init.instance_of? Array then
        type = v.get_type
        if( type.kind_of? PtrType )then
          # PtrType は ArrayType にすり替える

          # 初期化子の要素数とする (後は 0 である)
          t2 = ArrayType.new( Expression.create_integer_constant( init.length, nil ) )
          t2.set_type( type.get_type )
          type = t2
        end
        f.print "extern const #{type.get_type_str} #{@global_name}_#{v.get_name}_VAR_INIT#{type.get_type_str_post};\n"
      end
    }
    if @singleton then
      arg = "()"
      p_that = ""
      that = "#{@global_name}_SINGLE_CELL_CB."
    else
      arg = "(p_that)"
      p_that = "(p_that)"
      that = "(p_that)->"
    end

    if @n_cell_gen > 0 && need_CB_initializer? then
      b_var_init = false
      f.print "#define INITIALIZE_CB#{arg}"
      @var.each { |v|
        init = v.get_initializer
        next if init == nil

        b_var_init = true
        type = v.get_type.get_original_type
        f.print "\\\n"
#        print v.get_name, type.class, "\n"
#        if init.instance_of? Array || type.kind_of?( StructType ) then
        if init.instance_of? Array then
          if(type.kind_of?( ArrayType ) || type.kind_of?( PtrType ))then
            pre = "&"
            post = "[0]"
          elsif type.kind_of? StructType then
            pre = "&"
            post = ""
#          elsif type.kind_of? PtrType then
#            pre = ""
#            post = ""
          end
          f.print "\tmemcpy((void*)#{pre}#{@global_name}_VAR_#{v.get_name}#{p_that}#{post}, "
          f.print "(void*)#{pre}#{@global_name}_#{v.get_name}_VAR_INIT#{post}, sizeof(#{@global_name}_#{v.get_name}_VAR_INIT));"
        elsif init.instance_of? C_EXP then
          f.print "\t#{that}#{v.get_name} = #{init.get_c_exp_string};"
        else
          pre = "#{get_global_name}_ATTR_"
          if @singleton then
            post = ""
          else
            post = "#{p_that}"
          end
          f.print "\t#{that}#{v.get_name} = #{init.to_str( @name_list, pre, post )};"
        end
      }

      # dynamic call port の初期化コード
      b_dyn_port = false
      @port.each{ |p|
        next if p.get_port_type != :CALL
        if p.is_dynamic? && $ram_initializer then
          if p.get_array_size == nil then
            f.print "\\\n\t#{that}#{p.get_name} = #{that}_inib->#{p.get_name}_init_;"
          else
            if @singleton || p.get_array_size != "[]" then
              p_that = ""
            else
              p_that = "(p_that)"
            end
            if @singleton then
              that = "#{@global_name}_SINGLE_CELL_INIB."
            else
              that = "(p_that)->"
            end
            if has_CB? then
              init = '_init->'
            else
              init = ''
            end
            f.printf( "\\\n%-80s\\\n", '     {' )
            f.printf( "%-80s\\\n", '        int_t   j;' )
            f.printf( "%-80s\\\n", "        for( j = 0; j < N_CP_#{p.get_name}#{p_that}; j++ ){" )
            f.printf( "%-80s\\\n", "            #{that}#{p.get_name}[j] = #{that}#{init}#{p.get_name}_init_[j];" )
            f.printf( "%-80s\\\n", '        }' )
            f.printf( "%-80s", '       }' )
          end
          b_dyn_port = true
        end
      }
      if b_dyn_port then
        f.print( "\n" )
      end
      
      if b_var_init == false && b_dyn_port == false && ! @singleton then
        f.print "\t(void)(p_that);"
      end
      f.print "\n"

      f.print "#define SET_CB_INIB_POINTER(i,p_that)\\\n"
      if has_CB? && has_INIB? then
        if @singleton then
          f.print "\t#{that}_inib = &#{@global_name}_SINGLE_CELL_INIB;\n\n"
        elsif @idx_is_id_act
          f.print "\t#{that}_inib = #{@global_name}_INIB_tab[(i)];\n\n"
        else
          f.print "\t#{that}_inib = &#{@global_name}_INIB_tab[(i)];\n\n"
        end
      else
        f.print "\t/* empty */\n"
      end

    # else
    #   セルが一つもなければ出力しない
    end

  end


  def gen_ph_inline f
    # inline ポートが一つでもあれば、inline.h の include
    if @n_entry_port_inline > 0 then
      f.printf( TECSMsg.get( :INL_comment ), "#_INL_#" )
      f.print( "#include \"#{@global_name}_inline.#{$h_suffix}\"\n\n" )
    end
  end

  def gen_ph_endif( f, post = "TECSGEN" )
    f.print("#endif /* #{@global_name}_#{post}H */\n")
  end


##### celltype factory header
  def generate_factory_header

    f = AppFile.open("#{$gen}/#{@global_name}_factory.#{$h_suffix}")
    f.print("#ifndef #{@name}_FACTORY_H\n")
    f.print("#define #{@name}_FACTORY_H\n")
    f.close
  end

  def generate_factory_header_post

    f = AppFile.open("#{$gen}/#{@global_name}_factory.#{$h_suffix}")

    plugin_obj = get_celltype_plugin
    if plugin_obj
      plugin_obj.gen_factory f
    end

    f.print("#endif /* #{@name}_FACTORY_H */\n")
    f.close
  end


#####  celltype glue code
  def gen_cell_private_header f
    f.print "#include \"#{@global_name}_tecsgen.#{$h_suffix}\"\n"
  end

  def gen_cell_factory_header f
    f.print "#include \"#{@global_name}_factory.#{$h_suffix}\"\n\n"
  end

  def gen_cell_ep_des_type f
    if @n_entry_port > 0 then
      f.printf( TECSMsg.get( :EDT_comment ), "#_EDT_#" )
    end

    @port.each { |p|
      next if p.get_port_type != :ENTRY
      next if p.is_omit?
      if p.is_skelton_useless?    # 受け口最適化
        f.print( "/* #{p.get_name} : omitted by entry port optimize */\n\n" )
        next
      end

      f.print( "/* #{p.get_name} */\n" )
      f.print( "struct tag_#{@global_name}_#{p.get_name}_DES {\n" )
      f.print( "    const struct tag_#{p.get_signature.get_global_name}_VMT *vmt;\n" )
      if has_CB? || has_INIB? then
        f.print( "    #{@name}_IDX  idx;\n" )
      else
        # CB も INIB も存在しない (ので、idx として整数で初期化しておく)
        f.print( "    int           idx;\n" )
      end
      if p.get_array_size then
        f.print( "    int_t  subscript;\n" )
      end
      f.print( "};\n\n" )
    }
  end

  def gen_cell_skel_fun f
    if @n_entry_port >0 then
      f.printf( TECSMsg.get( :EPSF_comment ), "#_EPSF_#" )
    end

    @port.each { |p|
      next if p.get_port_type != :ENTRY
      next if p.is_omit?
      if p.is_skelton_useless? then    # 受け口最適化
        f.print( "/* #{p.get_name} : omitted by entry port optimize */\n" )
        next
      end

      f.print( "/* #{p.get_name} */\n" )

      p.get_signature.get_function_head_array.each{ |fun|
        functype = fun.get_declarator.get_type
        f.printf "%-14s", functype.get_type_str

        f.print " #{@global_name}_#{p.get_name}_#{fun.get_name}_skel("
        f.print " const struct tag_#{p.get_signature.get_global_name}_VDES *epd"
        delim = ","

        if functype.get_paramlist then
          items = functype.get_paramlist.get_items
          len = items.length
        else
          # ここで nil になるのは、引数なしの時に void がなかった場合
          items = []
          len = 0
        end
        i = 0
        items.each{ |param|
          f.print "#{delim} "
          delim = ","
          f.print param.get_type.get_type_str
          f.print " "
          f.print param.get_name
          f.print param.get_type.get_type_str_post
          i += 1
        }
        f.print ")\n"

        f.print "{\n"
        if ( ! @singleton || p.get_array_size != nil ) then
          f.print "    struct tag_#{@global_name}_#{p.get_name}_DES *lepd\n"
          f.print "        = (struct tag_#{@global_name}_#{p.get_name}_DES *)epd;\n"
        end

        if functype.get_type_str == "void" then # mikan "void" の typedef に未対応
          f.print "    "
        else
          f.print "    return "
        end

        f.print "#{@global_name}_#{p.get_name}_#{fun.get_name}("
        if @singleton then
          delim = ""
        else
          f.print " lepd->idx"
          delim = ","
        end

        if p.get_array_size then
          f.print "#{delim} lepd->subscript"
          delim = ","
        end

        items.each{ |param|
          f.print "#{delim} "
          delim = ","
          f.print param.get_name
          i += 1
        }
        f.print " );\n"

        f.print "}\n"
      }
    }
    if @n_entry_port >0 then
      f.print( "\n" )
    end
  end

  def gen_cell_fun_table f
    if @n_entry_port >0 then
      f.printf( TECSMsg.get( :EPSFT_comment ), "#_EPSFT_#" )
    end

    @port.each { |p|
      next if p.get_port_type != :ENTRY
      next if p.is_omit?
      if p.is_VMT_useless? then    # 受け口最適化
        f.print "/* #{p.get_name} : omitted by entry port optimize */\n"
        next
      end

      f.print "/* #{p.get_name} */\n"

      # f.print "static const struct tag_#{p.get_signature.get_global_name}_VMT"
      f.print "const struct tag_#{p.get_signature.get_global_name}_VMT"
      f.print " #{@global_name}_#{p.get_name}_MT_ = {\n"

      p.get_signature.get_function_head_array.each{ |fun|
        f.print "    #{@global_name}_#{p.get_name}_#{fun.get_name}_skel,\n"
      }

      f.print "};\n"
    }
    f.print "\n"

  end

  def gen_cell_ep_vdes fs
    if @n_cell_gen  >0 then
      fs.each{ |r,f| f.printf( TECSMsg.get( :CPEPD_comment ), "#_CPEPD_#" ) }
    end

    # このセルタイプのすべてのセルについて
    @ordered_cell_list.each{ |c|
      if c.is_generate? then                           # 生成対象か？

        f = fs[ c.get_region.get_domain_root ]

        # 結合のリスト (NamedList)
        jl = c.get_join_list

        # 全ての結合リストについて
        jl.get_items.each{ |j|

          # 左辺の定義を得る
          definition = j.get_definition

          # 呼び口ではない？　（属性）
          next unless definition.instance_of? Port

          port = find j.get_name # celltype の Port (こちらに最適化情報がある)
          # port = definition    # definition は composite の Port が得られることがある
          next if port.is_cell_unique?
          next if port.is_omit?

          # 配列要素を得る（受け口配列でなければ nil が返る）
          am = j.get_array_member2

          # 呼び口配列か?
          if am  then
            i = 0
            while( i < am.length )
              j = am[i]
              if j then
                if am[i].get_rhs_cell.get_celltype == self then
                  # 同じセルタイプへ結合している場合(VDES では type conflict になる)
                  p = am[i].get_rhs_port
                  des_type = "const struct tag_#{@global_name}_#{p.get_name}_DES"
                else
                  des_type = "struct tag_#{definition.get_signature.get_global_name}_VDES"
                end

                # 右辺は受け口配列か？
                if j.get_rhs_subscript then

                  # 受け口の配列添数
                  subscript = j.get_rhs_subscript

                  f.printf( "extern %s %s%d;\n",
                            des_type,
                            "#{j.get_port_global_name(i)}_des",
                            subscript)
                else
                  f.printf( "extern %s %s;\n",
                            des_type,
                            "#{j.get_port_global_name(i)}_des")
                end
              #else if j == nil
              #  optioanl で配列要素が初期化されていない
              end
              i += 1
            end
          else
            dbgPrint "me=#{@name} callee=#{j.get_rhs_cell.get_celltype.get_name} #{j.get_cell.get_celltype.get_name} \n"
            if j.get_rhs_cell.get_celltype == self then
              # 同じセルタイプへ結合している場合(VDES では type conflict になる)
              p = j.get_rhs_port
              des_type = "const struct tag_#{@global_name}_#{p.get_name}_DES"
            else
              des_type = "struct tag_#{definition.get_signature.get_global_name}_VDES"
            end

            if j.get_rhs_subscript then
              # 受け口配列
              subscript = j.get_rhs_subscript
              f.printf( "extern %s %s%d;\n",
                        des_type,
                        "#{j.get_port_global_name}_des",
                        subscript)
            else
              f.printf( "extern %s %s;\n",
                        des_type,
                        "#{j.get_port_global_name}_des" )
            end
          end
          # mikan   cell の namespace 未対応、Join で Cell オブジェクトを引当ておく必要あり
        }

        f.print "\n"
      end
    }
  end

  def gen_cell_ep_vdes_array fs
    if @n_cell_gen >0 then
      fs.each{ |r, f| f.printf( TECSMsg.get( :CPA_comment ), "#_CPA_#" ) }  # mikan 呼び口配列が無い場合も出てしまう
    end

    @ordered_cell_list.each{ |c|
      if c.is_generate? then
        f = fs[ c.get_region.get_domain_root ]

        jl = c.get_join_list
        # ループを回す変数を jl から @port に変更
        # dynamic, optional 
#        jl.get_items.each{ |j|
        @port.each { |port|
          next if port.get_port_type != :CALL
          dbgPrint( "gen_cell_ep_vdes_array: #{c.get_name}.#{port.get_name}\n" )

#          definition = j.get_definition
#          next unless definition.instance_of? Port
          j = jl.get_item( port.get_name )

          # port = definition    # definition は composite の Port が得られることがある
          # port = find j.get_name # celltype の Port (こちらに最適化情報がある)
          next if port.is_cell_unique?
          next if port.is_omit?

          b_array = false
          am = nil
          if j then
            am = j.get_array_member2
            if am then
              b_array = true
            end
          else
            if port.get_array_size == "[]" then
              # this case is dynamic optional and nothing joined
              next
            elsif port.get_array_size then
              b_array = true
            end
          end
          if b_array  then
#          if am then
            # 左辺は配列
            const = ( port.is_dynamic? && ! $ram_initializer ) ? '' : 'const '
            init = ( port.is_dynamic? && $ram_initializer ) ? '_init_' : ''

            if ! port.is_skelton_useless? then
              f.printf( "struct %s * #{const}%s_%s[] = {\n",
                        "tag_#{port.get_signature.get_global_name}_VDES",
                        "#{c.get_global_name}",
                        "#{port.get_name}" + init )
            else

#              スケルトン関数不要最適化の場合、この配列は参照されない
              # mikan このケースがテストされていない
              f.printf( "#{const}%s_IDX  %s_%s[] = {\n",
#                        "#{j.get_celltype.get_global_name}",   # 右辺 composite に対応できない
                        "#{j.get_rhs_cell.get_celltype.get_global_name}",
                        "#{c.get_global_name}",
                        "#{j.get_name}" )
            end
            if port.get_array_size == "[]" then
              length = am.length
            else
              length = port.get_array_size
            end
            # am.each { |j|
            i = 0
            while i < length
              if am == nil then
                f.print( "    0,\n" )
                i += 1
                next
              end
              j = am[i]
              i += 1

              if j then
                # 同一セルタイプの結合の場合、VDES 型へのキャストが必要
                if j.get_rhs_cell.get_celltype == self then
                  definition = j.get_definition
                  des_type_cast = "(struct tag_#{definition.get_signature.get_global_name}_VDES *)"
                else
                  des_type_cast = ""
                end

                
                if j.get_rhs_subscript then
                  # 右辺配列の場合(最適化はない)
                  subscript = j.get_rhs_subscript
                  f.printf( "    %s%d,\n",
                            "#{des_type_cast}&#{j.get_port_global_name}_des",
                            subscript)
                  # p "1: #{j.get_port_global_name}_des"
                  # p "2: #{j.get_cell_global_name}_#{j.get_port_name}_des"

                else
                  # 右辺非配列の場合 */
                  if ! port.is_skelton_useless? then
                    f.printf( "    %s,\n",
                              "#{des_type_cast}&#{j.get_port_global_name}_des" )
                  else
                    cell = j.get_rhs_cell
                    name_array = cell.get_celltype.get_name_array( cell )
                    f.printf( "    #{name_array[7]},\n" )
                  end
                end
              else
                # optional で呼び口配列要素が初期化されていない
                f.printf( "    0,\n" )
              end
            # }
            end
            # mikan   cell の namespace 未対応、Join で Cell オブジェクトを引当ておく必要あり
            f.print "};\n"
            # dynamic の呼び口配列
            if port.is_dynamic? && $ram_initializer then
              f.printf( "struct %s * %s_%s[ #{length} ];\n",
                        "tag_#{port.get_signature.get_global_name}_VDES",
                        "#{c.get_global_name}",
                        "#{port.get_name}" )
            end
          end
        }

        f.print "\n"
      end
    }
  end

  #=== CB を初期化するプログラムの生成
  def gen_cell_cb_initialize_code f
    if ! need_CB_initializer?
      return
    end
    f.printf( TECSMsg.get( :CIC_comment ), "#_CIC_#" )
    f.print <<EOT
void
#{@global_name}_CB_initialize()
{
EOT
    if @singleton then
      f.print <<EOT
    SET_CB_INIB_POINTER(i,p_cb)
    INITIALIZE_CB()
EOT
    else
      f.print <<EOT
    #{@global_name}_CB	*p_cb;
    int		i;
    FOREACH_CELL(i,p_cb)
        SET_CB_INIB_POINTER(i,p_cb)
        INITIALIZE_CB(p_cb)
    END_FOREACH_CELL
EOT
    end

    f.print <<EOT
}
EOT
  end

  # === CB/INIB の外で初期化される変数の出力
  def gen_cell_cb_out_init fs

    # セルがなければ、出力しない
    if @n_cell_gen == 0 then
      return
    end

    fs.each{ |r, f| f.printf( TECSMsg.get( :AVAI_comment ), "#_AVAI_#" ) }

    # attribute, var のポインタ型の参照する配列を生成
    @ordered_cell_list.each{ |c|
      next if  ! c.is_generate?

      f = fs[ c.get_region.get_domain_root ]
      name_array = get_name_array( c )

      ct = c.get_celltype
      jl = c.get_join_list

      # attribute, var のポインタ変数が配列により初期化される場合の、配列を出力
      av_list = ct.get_attribute_list + ct.get_var_list
      if av_list.length != 0 then
        av_list.each{ |a|    # a: Decl
          j = jl.get_item( a.get_identifier )
          if j then
            init = j.get_rhs
          else
            init = a.get_initializer
          end

          if( a.is_type?( PtrType ) && ( (init && init.instance_of?( Array )) || init == nil ) )then
            ptr_type = a.get_type
            size = ptr_type.get_size

            if size then
              # 式を評価する(attribute, var に含まれる変数を参照可能)
              sz = size.eval_const( c.get_join_list, c.get_celltype.get_name_list )
              # 式を生成しなおす (変数を含まない形にする)  不完全な形で Token を生成 (エラー発生しないから)
              size = Expression.new( [:INTEGER_CONSTANT, Token.new(sz, nil, 0, 0)] )
              array_type = ArrayType.new( size )
              type = a.get_type.get_referto
              if ! type.is_const? && a.get_kind == :ATTRIBUTE then
                type.set_qualifier :CONST
              end
              array_type.set_type( type )
              if a.get_kind == :ATTRIBUTE then
                f.print "const "
              end
              f.printf( "#{a.get_type.get_referto.get_type_str} #{name_array[3]}_#{a.get_identifier}_INIT[%d]#{a.get_type.get_referto.get_type_str_post}", sz )
								# name_array[3]: cell_CB_INIT
              if !( $ram_initializer && a.get_kind == :VAR ) then
                # -R (ram initializer 使用) の場合 var は初期化コードを出力しない
                if( init )then
                  str = " = #{gen_cell_cb_init( f, c, name_array, array_type, init, a.get_identifier, 1, true )}"
                  str = str.sub( /\}$/, "};\n" )
                else
                  str = ";\n"
                end
                f.print( str )
              else
                f.print( ";\n" )
              end
            end
          end
        }
      end
    }
  end

  #=== var の初期値の ROM 部への
  def gen_cell_var_init f
    # var の{ }で囲まれた初期値指定があるか調べる
    n_init = 0
    @var.each { |v|
      init = v.get_initializer
      if init && init.instance_of?( Array ) then
        n_init += 1
      end
    }

    if n_init > 0 then
      f.printf( TECSMsg.get( :AVI_comment ), "#_AVI_#" )
      @var.each { |v|
        init = v.get_initializer
        if init && init.instance_of?( Array ) then
          type = v.get_type
          org_type = v.get_type.get_original_type

          if( org_type.kind_of? PtrType )then
            # PtrType は ArrayType にすり替える

            # 初期化子の要素数だけとする（後は 0)
            t2 = ArrayType.new( Expression.create_integer_constant( init.length, nil ) )
            t2.set_type( type.get_type )
            type = t2
            org_type = t2
          end

          c = @ordered_cell_list[0]   # 仮の cell (実際には使われない)
          name_array = get_name_array( c )
          # f.print "const #{type0.get_type_str}\t#{@global_name}_#{v.get_name}_VAR_INIT#{type0.get_type_str_post} = "
          f.print "const #{type.get_type_str}\t#{@global_name}_#{v.get_name}_VAR_INIT#{type.get_type_str_post} = "
          if org_type.kind_of? StructType then
            # celltype の default の初期値あり
            str = gen_cell_cb_init( f, c, name_array, type, init, v.get_identifier, 1, true )
          elsif( org_type.kind_of?( PtrType ) || org_type.kind_of?( ArrayType ) ) then
            str = "{ "
            type = org_type.get_type
            # mikan ポインタではなく、配列型としないと、ポインタ変数の領域の分、損する
            init.each { |i|
              str += gen_cell_cb_init( f, c, name_array, type, i, v.get_identifier, 1, true )
              str += ", "
            }
            str += "}"
          else
            p type.class
            raise "Unknown Type"
          end
          f.print str
          f.print ";\n"
        end
      }
      f.print "\n"
    end

  end

  def gen_cell_cb fs
    if has_INIB? then
      if @n_cell_gen > 0 then
        fs.each{ |r, f| f.printf( TECSMsg.get( :INIB_comment ), "#_INIB_#" ) }
      end
      if @singleton then
        fs.each{ |r, f| f.print "#{@global_name}_INIB #{@global_name}_SINGLE_CELL_INIB = \n" }
        indent = 0
      elsif ! @idx_is_id_act then
        fs.each{ |r, f| f.print "#{@global_name}_INIB #{@global_name}_INIB_tab[] = {\n" }
        indent = 1
      else
        indent = 0
      end

      @ordered_cell_list.each{ |c|
        next if ! c.is_generate?

        f = fs[ c.get_region.get_domain_root ]

        name_array = get_name_array( c )

        unless @singleton then
          print_indent( f, indent )
          f.print "/* cell: #{name_array[2]}:  #{name_array[1]} id=#{c.get_id} */\n"
			    # name_array[2]: cell_CB_name
        end

        print_indent( f, indent )
        if @idx_is_id_act then
          f.print "const #{@global_name}_INIB #{name_array[5]} = "
        end
        f.print "{\n"

        gen_cell_cb_port( c, indent, f, name_array, :INIB )
        gen_cell_cb_attribute( c, indent, f, name_array, :INIB )

        unless @singleton then
          # 1 つの cell INIB の終わり
          if @idx_is_id_act then
            f.print( "};\n\n" )
          else
            f.print( "    },\n" )
          end
        end
      }
      if ! @idx_is_id_act then
        fs.each{ |r, f| f.print( "};\n\n" ) }
      end
    end  # has_INIB?

    if has_CB? then
      if @n_cell_gen >0 then
        fs.each{ |r, f| f.printf( TECSMsg.get( :CB_comment ),  "#_CB_#" ) }
      end

      # RAM initializer を使用しない、または ROM 化しない
      if $ram_initializer == false || $rom == false then
        if @singleton then
          fs.each{ |r, f| f.print "struct tag_#{@global_name}_CB #{@global_name}_SINGLE_CELL_CB = \n" }
          indent = 0
        elsif ! @idx_is_id_act then
          fs.each{ |r, f| f.print "struct tag_#{@global_name}_CB #{@global_name}_CB_tab[] = {\n" }
          indent = 1
        else
          indent = 0
        end

        @ordered_cell_list.each{ |c|
          next if ! c.is_generate?

          f = fs[ c.get_region.get_domain_root ]

          name_array = get_name_array( c )

          unless @singleton then
            print_indent( f, indent )
            f.print "/* cell: #{name_array[2]}:  #{name_array[1]} id=#{c.get_id} */\n"
            # name_array[2]: cell_CB_name
          end

          print_indent( f, indent )
          if @idx_is_id_act then
            f.print "#{@global_name}_CB #{name_array[2]} = "
          end
          f.print "{\n"

          if has_INIB? then
            print_indent( f, indent + 1 )
            f.printf( "&%-39s /* _inib */\n", "#{name_array[5]}," )
          end

          #if ! has_INIB? then
          if $rom == false then
            gen_cell_cb_port( c, indent, f, name_array, :CB_ALL )
          else
            gen_cell_cb_port( c, indent, f, name_array, :CB_DYNAMIC )
          end

          gen_cell_cb_attribute( c, indent, f, name_array, :CB )
          gen_cell_cb_var( c, indent, f, name_array )

          unless @singleton then
            # 1 つの cell CB の終わり
            if @idx_is_id_act then
              f.print( "};\n\n" )
            else
              f.print( "    },\n" )
            end
          end
        }
        if ! @idx_is_id_act then
          fs.each{ |r, f| f.print( "};\n\n" ) }
        end
      else
        if @singleton then
          fs.each{ |r, f| f.print "struct tag_#{@global_name}_CB #{@global_name}_SINGLE_CELL_CB;\n" }
          indent = 0
        elsif @idx_is_id_act then
          @ordered_cell_list.each{ |c|
            next if ! c.is_generate?

            f = fs[ c.get_region.get_domain_root ]

            name_array = get_name_array( c )
            f.print "/* cell: #{name_array[2]}:  #{name_array[1]} id=#{c.get_id} */\n"
            f.print "#{@global_name}_CB #{name_array[2]} = {};\n"
          }
        else
          fs.each{ |r, f| f.print "struct tag_#{@global_name}_CB #{@global_name}_CB_tab[#{@n_cell_gen}];\n" }
        end
      end
    end  # has_CB?
  end

  def gen_cell_cb_tab f
    indent = 0
    if @idx_is_id_act then
      if has_INIB? && ( $ram_initializer || ! has_CB? ) then
        f.print "/* ID to INIB table #_INTAB_# */\n"
        @ordered_cell_list.each{ |c|
          if c.is_generate? && ( c.get_region.get_domain_root != Region.get_root ) then # 生成対象かつ、ルート以外か
            name_array = get_name_array( c )
            print_indent( f, indent + 1 )
            f.print "extern #{@global_name}_INIB  #{name_array[5]};\n"
          end
        }

        f.print "#{@global_name}_INIB *#{@global_name}_INIB_tab[] ={\n"
        @ordered_cell_list.each{ |c|
          if c.is_generate? then                           # 生成対象か？
            name_array = get_name_array( c )
            print_indent( f, indent + 1 )
            f.print "&#{name_array[5]},\n"
          end
        }
        f.print "};\n"
      end
      if has_CB? then
        f.print "/* ID to CB table #_CBTAB_# */\n"
        @ordered_cell_list.each{ |c|
          if c.is_generate? && ( c.get_region.get_domain_root != Region.get_root ) then # 生成対象かつ、ルート以外か
            name_array = get_name_array( c )
            print_indent( f, indent + 1 )
            f.print "extern #{@global_name}_CB  #{name_array[2]};\n"
          end
        }

        f.print "#{@global_name}_CB *#{@global_name}_CB_tab[] ={\n"
        @ordered_cell_list.each{ |c|
          if c.is_generate? then                           # 生成対象か？
            name_array = get_name_array( c )
            print_indent( f, indent + 1 )
            f.print "&#{name_array[2]},\n"
          end
        }
        f.print "};\n"
      end
    end
  end


  #=== name_array を生成
  # IN:   cell  : Cell
  #       index : CB, INIB 配列の添数
  # RETURN: name_array
  #   name_array[0] = @name           # celltype name
  #   name_array[1] = cell.get_name   # cell name
  #   name_array[2] = cell_CB_name    # cell_CB_name
  #   name_array[3] = cell_CB_INIT    # cell_CB_INIT # CB の外側で初期化が必要な配列の名前
  #   name_array[4] = cell_CB_proto   # CB name for prototype
  #   name_array[5] = cell_INIB       # INIB name
  #   name_array[6] = cell_ID         # ID
  #   name_array[7] = cell_IDX        # IDX
  #   name_array[8] = cell_CBP        # CB pointer
  #   name_array[9] = @global_name    # celltype global name
  #   name_array[10] = cell.get_global_name # cell global name
  #   name_array[11] = cell_INIB_proto #INIB name for proto type
  
  def get_name_array( cell )

    if @singleton then
      cell_CB_name = "#{@global_name}_SINGLE_CELL_CB"
      cell_CB_INIT = cell_CB_name
      cell_CB_proto = "#{@global_name}_SINGLE_CELL_CB"
      cell_INIB_name = "#{@global_name}_SINGLE_CELL_INIB"
      cell_INIB_proto = cell_INIB_name
      cell_ID = 0
    else
      if ! @idx_is_id_act then
        index = cell.get_id - cell.get_celltype.get_id_base
        cell_CB_name = "#{@global_name}_CB_tab[#{index}]"
        cell_CB_INIT = "#{@global_name}_#{cell.get_name}_CB"
        cell_CB_proto = "#{@global_name}_CB_tab[]"
        cell_INIB_name = "#{@global_name}_INIB_tab[#{index}]"
        cell_INIB_proto = "#{@global_name}_INIB_tab[]"
      else
        cell_CB_name = "#{cell.get_global_name}_CB"
        cell_CB_INIT = cell_CB_name
        cell_CB_proto = cell_CB_name
        cell_INIB_name = "#{cell.get_global_name}_INIB"
        cell_INIB_proto = cell_INIB_name
      end
      cell_ID = cell.get_id
    end
    if @idx_is_id_act then
      cell_IDX = cell_ID
    else
      cell_IDX = "&#{cell_CB_name}"
    end

    if has_CB? then
      cell_CBP = "&#{cell_CB_name}"
    elsif has_INIB? then
      cell_CBP = "&#{cell_INIB_name}"
    else
      cell_CBP = "NULL"    # CB も INIB もなければ NULL に置換
    end

    name_array = []
    name_array[0] = @name           # celltype name
    name_array[1] = cell.get_name   # cell name
    name_array[2] = cell_CB_name    # cell_CB_name
    name_array[3] = cell_CB_INIT    # cell_CB_INIT
    name_array[4] = cell_CB_proto   # CB name for prototype
    name_array[5] = cell_INIB_name  # cell INIB name
    name_array[6] = cell_ID         # cell ID
    name_array[7] = cell_IDX        # cell IDX
    name_array[8] = cell_CBP        # cell CBP
    name_array[9] = @global_name    # celltype global name
    name_array[10] = cell.get_global_name # cell global name
    name_array[11] = cell_INIB_proto # INIB name for prototype

    return name_array
  end

  #=== attribute と size_is 指定された var (ポインタ)の初期化データを出力
  #
  # ROM 化サポートの有無、および出力対象が CB か INIB かにより出力される内容が異なる
  def gen_cell_cb_attribute( cell, indent, f, name_array, cb_inib )
    ct = self
    jl = cell.get_join_list

    if cb_inib == :INIB then
      return if @n_attribute_ro == 0 && @n_var_size_is == 0
      print_indent( f, indent + 1 )
      f.print "/* attribute(RO) */ \n"
    elsif $rom then  # && cb_inib == CB
      # CB で rw と var
      return if @n_attribute_rw == 0
      print_indent( f, indent + 1 )
      f.print "/* attribute(RW) */ \n"
    else  # cb_inib == CB && $rom == false
      # CB に全部
      return if @n_attribute_rw == 0 && @n_attribute_ro == 0 && @n_var_size_is == 0
      print_indent( f, indent + 1 )
      f.print "/* attribute */ \n"
    end

    attribute = ct.get_attribute_list
    attribute.each{ |a|              # a: Decl
      next if a.is_omit?
      if cb_inib == :INIB && a.is_rw? == true then
        # $rom == true でしか、ここへ来ない
        next
      elsif cb_inib == :CB && $rom && ! a.is_rw? then
        next
      end

      j = jl.get_item( a.get_identifier )
      if j then
        # cell の初期値指定あり
        gen_cell_cb_init( f, cell, name_array, a.get_type, j.get_rhs, a.get_identifier, indent + 1 )
      elsif a.get_initializer then
        # celltype の default の初期値あり
        gen_cell_cb_init( f, cell, name_array, a.get_type, a.get_initializer, a.get_identifier, indent + 1 )
      else
        # 初期値未指定
        gen_cell_cb_init( f, cell, name_array, a.get_type, nil, a.get_identifier, indent + 1 )
      end
    }
    @var.each{ |v|
      next if v.is_omit?
      next if v.get_size_is == nil   # size_is 指定がある場合 attribute の一部として出力

      if v.get_initializer && $ram_initializer == false then
        gen_cell_cb_init( f, cell, name_array, v.get_type, v.get_initializer, v.get_identifier, indent + 1 )
      else
        # 初期値未指定 または RAM initializer 使用
        gen_cell_cb_init( f, cell, name_array, v.get_type, nil, v.get_identifier, indent + 1 )
      end
    }
  end

  #=== var の初期化データを出力
  def gen_cell_cb_var( cell, indent, f, name_array )
    jl = cell.get_join_list
    var = get_var_list
    if @n_var - @n_var_size_is > 0 then
      print_indent( f, indent + 1 )
      f.print "/* var */ \n"
      var.each{ |v|

        next if v.is_omit?
        next if v.get_size_is      # size_is 指定がある場合 attribute の一部として出力

        if v.get_initializer && $ram_initializer == false then
          gen_cell_cb_init( f, cell, name_array, v.get_type, v.get_initializer, v.get_identifier, indent + 1 )
        else
          # 初期値未指定 または RAM initializer 使用
          gen_cell_cb_init( f, cell, name_array, v.get_type, nil, v.get_identifier, indent + 1 )
        end
      }
    end
  end

  #inib_cb::Symbol: :INIB, :CB_ALL, :CB_DYNAMIC
  def gen_cell_cb_port( cell, indent, f, name_array, inib_cb = :INIB )
    gen_cell_cb_call_port( cell, indent, f, name_array, inib_cb )
    gen_cell_cb_entry_port( cell, indent, f, name_array )
  end

  #=== 呼び口の初期化コードの生成
  def gen_cell_cb_call_port( cell, indent, f, name_array, inib_cb )
    jl = cell.get_join_list

    port = get_port_list
    if inib_cb == :INIB && ( @n_call_port - @n_call_port_omitted_in_CB -
                             ( $ram_initializer ? 0 : (@n_call_port_dynamic-@n_call_port_array_dynamic) )  > 0 ) ||
       inib_cb == :CB_ALL && @n_call_port > 0 ||
       inib_cb == :CB_DYNAMIC && (@n_call_port_dynamic - @n_call_port_array_dynamic) > 0 then
      print_indent( f, indent + 1 )
      f.print "/* call port (#{inib_cb}) #_CP_# */ \n"
      port.each{ |p|
        next if p.get_port_type != :CALL
        next if p.is_omit?
        next if p.is_cell_unique?        # 最適化（単一セルで呼び口マクロに埋め込まれる）
        next if inib_cb == :INIB && p.is_dynamic? && p.get_array_size == nil && ! $ram_initializer
        next if inib_cb == :CB_DYNAMIC && ( ! p.is_dynamic? || p.get_array_size != nil )

        j = jl.get_item( p.get_name )
        print_indent( f, indent + 1 )

        # debug
        if j == nil then
          dbgPrint "cell_cb_call_port: #{p.get_name} array size=#{p.get_array_size}\n"
          # optional 呼び口
          # cdl_error( "H1003 internal error: cell \'$1\' port \'$2\': initializer not found\n" , cell.get_name, p.get_name )
          # exit( 1 )
          if p.get_array_size then
            if p.is_dynamic? then
              if inib_cb == :INIB then
                if  $ram_initializer then
                  f.printf( "%-40s /* #_CCP7_# _init_ */\n",  "#{cell.get_global_name}_#{p.get_name}_init_," )
                  print_indent( f, indent + 1 )
                end
                f.printf( "%-40s /* #_CCP7B_# */\n",  "#{cell.get_global_name}_#{p.get_name}," )
              elsif $rom == false then
                f.printf( "%-40s /* #_CCP8_# */\n",  "#{cell.get_global_name}_#{p.get_name}," )
              end
            else
              f.printf( "%-40s /* #_CCP9_# */\n",  "0," )
            end
            if p.get_array_size == "[]" then
              # 添数省略の呼び口配列
              print_indent( f, indent + 1 )
              f.printf( "%-40s /* %s #_CCP6_# */\n", "0,", "length of #{p.get_name} (n_#{p.get_name})" )
            end
          else
            f.printf( "%-40s /* #_CCP5_# */\n",  "0," )
          end
          next
        end

        am = j.get_array_member2
        if am then
          # 呼び口配列の場合
          if inib_cb == :INIB && p.is_dynamic? && p.get_array_size != nil && $ram_initializer then
            f.printf( "%-40s /* #_CCP3_# _init_ */\n",  "#{cell.get_global_name}_#{j.get_name}_init_," )
            print_indent( f, indent + 1 )
          end
          f.printf( "%-40s /* #_CCP3B_# */\n",  "#{cell.get_global_name}_#{j.get_name}," )
          if p.get_array_size == "[]" then
            # 添数省略の呼び口配列
            print_indent( f, indent + 1 )
            f.printf( "%-40s /* %s #_CCP4_# */\n", "#{am.length},", "length of #{p.get_name} (n_#{p.get_name})" )
          end
        else
          # 同一セルタイプの結合の場合、VDES 型へのキャストが必要
          #print "CCP0/CCP1 #{p.get_name}, #{j.get_rhs_cell.get_celltype.get_name}, #{@name}\n"
          if j.get_rhs_cell.get_celltype == self then
            definition = j.get_definition
            des_type_cast = "(struct tag_#{definition.get_signature.get_global_name}_VDES *)"
          else
            des_type_cast = ""
          end

          init = ( p.is_dynamic? && inib_cb == :INIB ) ? "_init_" : ""

          if j.get_rhs_subscript then
            # 受け口配列の場合
            subscript = j.get_rhs_subscript
            f.printf( "%-40s /* %s #_CCP0_# */\n",
                      # "&#{j.get_cell_global_name}_#{j.get_port_name}_des#{subscript},",
                      "#{des_type_cast}&#{j.get_port_global_name}_des#{subscript},",
                      p.get_name.to_s + init )
          else
            # 呼び口配列でも、受け口配列でもない
            if ! p.is_skelton_useless? then
              f.printf( "%-40s /* %s #_CCP1_# */\n",
                        "#{des_type_cast}&#{j.get_port_global_name}_des,",
                        p.get_name.to_s + init )
            else
              # スケルトン不要最適化（CB (INIB) へのポインタを埋め込む）
              c = j.get_rhs_cell                    # 呼び先セル
              ct = c.get_celltype                   # 呼び先セルタイプ
              if ct.has_INIB? || ct.has_CB? then
                name_array = ct.get_name_array( c )   # 呼び先セルタイプで name_array を得る
                f.printf( "%-40s /* %s #_CCP2_# */\n", "#{name_array[7]},", p.get_name )
              else
                # 呼び先は CB も INIB も持たない（NULL に初期化）
                f.printf( "%-40s /* %s #_CCP2B_# */\n", "0,", p.get_name )
              end
            end
          end
        end

      }
      end
  end

  #=== 受け口の初期化コードの生成
  def gen_cell_cb_entry_port( cell, indent, f, name_array )
    jl = cell.get_join_list

    port = get_port_list
    if @n_entry_port != 0 then
      print_indent( f, indent + 1 )
      f.print "/* entry port #_EP_# */ \n"
      @port.each{ |p|
        # next if p.is_omit?  # 受け口配列の個数は省略しない
        if p.get_port_type == :ENTRY && p.get_array_size == "[]"
          print_indent( f, indent + 1 )
          f.printf( "%-40s /*  #_EEP_# */\n", "#{cell.get_entry_port_max_subscript( p )+1}," )
        end
      }
    end
  end

  #=== セルの attribute の初期値を出力
  #
  #f_get_str:: true の場合、文字列を返す、false の場合、ファイル f に出力する．
  # 文字列を返すとき、末尾に ',' は含まれない．
  # ファイルへ出力するとき、末尾に ',' が出力される．構造体要素、配列要素の初期値を出力すると ',' が二重に出力される．
  # ただし現状では、ファイルへ出力することはない
  #
  def gen_cell_cb_init( f, cell, name_array, type, init, identifier, indent, f_get_str = false )

    cell_CB_name = name_array[2]
    cell_CB_INIT = name_array[3]

    while type.kind_of?( DefinedType )
      type = type.get_type
    end

    if ( init == nil ) then

      if f_get_str then
        # 初期値未指定
        if type.kind_of?( BoolType ) then
          str = "false"   # formerly tecs_false
        elsif type.kind_of?( IntType ) then
          str = "0"
        elsif type.kind_of?( FloatType ) then
          str = "0.0"
        elsif type.kind_of?( EnumType ) then
          str = "0"
        elsif type.kind_of?( ArrayType ) then
          str = "{}"
        elsif type.kind_of?( StructType ) then
          str = "{}"
        elsif type.kind_of?( PtrType ) then
          if type.get_size then
            str = "#{cell_CB_INIT}_#{identifier}_INIT"
          else
            str = "0"
          end
        else
          raise "UnknownType"
        end
        return str
      else
        # 初期値未指定
        if type.kind_of?( BoolType ) then
          f.print "    " * indent
          f.printf( "%-40s /* %s */\n", "false,", identifier )   # formerly tecs_false
        elsif type.kind_of?( IntType ) then
          f.print "    " * indent
          f.printf( "%-40s /* %s */\n", "0,", identifier )
        elsif type.kind_of?( FloatType ) then
          f.print "    " * indent
          f.printf( "%-40s /* %s */\n", "0.0,", identifier )
        elsif type.kind_of?( EnumType ) then
          f.print "    " * indent
          f.printf( "%-40s /* %s */\n", "0,", identifier )
        elsif type.kind_of?( ArrayType ) then
          f.print "    " * indent
          f.printf( "%-40s /* %s */\n", "{},", identifier )
        elsif type.kind_of?( StructType ) then
          f.print "    " * indent
          f.printf( "%-40s /* %s */\n", "{},", identifier )
        elsif type.kind_of?( PtrType ) then
          if type.get_size then
            f.print "    " * indent
            f.printf( "%-40s /* %s */\n", "#{cell_CB_INIT}_#{identifier}_INIT,", identifier )
          else
            f.print "    " * indent
            f.printf( "%-40s /* %s */\n", "0,", identifier )
          end
        else
          raise "UnknownType"
        end
      end
      return
    end    

    # このメソッドは Celltype のものである必要は無い（上に続くのでここに置く）
    # 初期値指定あり
    if type.kind_of?( BoolType ) then
      if init.instance_of?( C_EXP ) then
        init_str = subst_name( init.get_c_exp_string, name_array )
      else
        init_str = init.eval_const2(cell.get_join_list,@name_list)
      end

      if f_get_str then
        return "#{init_str}"
      else
        f.print "    " * indent
        f.printf( "%-40s /* %s */\n", "#{init_str},", identifier )
      end
#      if f_get_str then
#        return "#{init.eval_const2(nil)}"
#      else
#        f.print "    " * indent
#        f.printf( "%-40s /* %s */\n", "#{init.eval_const2(nil)},", identifier )
#      end
    elsif type.kind_of?( IntType ) then
      if init.instance_of?( C_EXP ) then
        init_str = subst_name( init.get_c_exp_string, name_array )
      else
        init_str = init.eval_const2(cell.get_join_list,@name_list)
      end

      if f_get_str then
        return "#{init_str}"
      else
        f.print "    " * indent
        f.printf( "%-40s /* %s */\n", "#{init_str},", identifier )
      end
    elsif type.kind_of?( FloatType ) then
      # mikan C_EXP for FloatType
      if f_get_str then
        return "#{init.eval_const2(cell.get_join_list,@name_list)}"
      else
        f.print "    " * indent
        f.printf( "%-40s /* %s */\n", "#{init.eval_const2(cell.get_join_list,@name_list)},", identifier )
      end
    elsif type.kind_of?( EnumType ) then
      # mikan C_EXP for EnumType
      if f_get_str then
        return "#{init.eval_const2(cell.get_join_list,@name_list)}"
      else
        f.print "    " * indent
        f.printf( "%-40s /* %s */\n", "#{init.eval_const2(cell.get_join_list,@name_list)},", identifier )
      end
    elsif type.kind_of?( ArrayType ) then
      if type.get_subscript
        len = type.get_subscript.eval_const(cell.get_join_list,@name_list)
      else
        len = init.length
      end

      at = type.get_type
      i = 0
      if f_get_str then
        str = "{ " 
      else
        f.print "    " * indent
        f.print( "{\n" )
      end

      len.times {
        next if ! init[i]        # mikan この処置は適切？
        if f_get_str then
          str += gen_cell_cb_init( f, cell, name_array, at, init[i], "#{identifier}[#{i}]", indent + 1, f_get_str )
          str += ', '
        else
          gen_cell_cb_init( f, cell, name_array, at, init[i], "#{identifier}[#{i}]", indent + 1, f_get_str )
        end
        i += 1
      }

      if f_get_str then
         str += "}"
      else
        f.print "    " * indent
        f.print( "},\n" )
      end

    elsif type.kind_of?( StructType ) then
      i = 0
      decls = type.get_members_decl.get_items
      if f_get_str then
        str = "{ "
      else
        f.print "    " * indent
        f.print( "{                                        /* #{identifier} */\n" )
      end

      decls.each{ |d|
        # p "#{d.get_identifier}: #{init}"
        next if ! init[i]

        if f_get_str then
          str += gen_cell_cb_init( f, cell, name_array, d.get_type, init[i], d.get_identifier, indent + 1, f_get_str )
          str += ', '
        else
          gen_cell_cb_init( f, cell, name_array, d.get_type, init[i], d.get_identifier, indent + 1, f_get_str )
        end
        i += 1
      }
      if f_get_str then
        str += "}"
      else
        f.print "    " * indent
        f.print( "},\n" )
      end

    elsif type.kind_of?( PtrType ) then

      if init.instance_of?( Array ) then
        if f_get_str then
          return "#{cell_CB_INIT}_#{identifier}_INIT"
        else
          f.print "    " * indent
          f.printf( "%-40s /* %s */\n", "#{cell_CB_INIT}_#{identifier}_INIT,", identifier )
        end
      elsif init.instance_of?( C_EXP ) then
        init_str = subst_name( init.get_c_exp_string, name_array )

        if f_get_str then
          return "#{init_str}"
        else
          f.print "    " * indent
          f.printf( "%-40s /* %s */\n", "#{init_str},", identifier )
        end

      else
        if f_get_str then
          return "#{init.eval_const2(cell.get_join_list,@name_list)}"
        else
          f.print "    " * indent
# p init.eval_const2(cell.get_join_list,@name_list).class
# p init.eval_const2(cell.get_join_list,@name_list)
# p identifier
          f.printf( "%-40s /* %s */\n", "#{init.eval_const2(cell.get_join_list,@name_list)},", identifier )

        end
      end
    else
      raise "UnknownType"
    end
  end

  #== 関数テーブルの外部参照
  def gen_cell_extern_mt fs
    fs.each{ |r, f|
      if ! r.is_root? then
        @port.each{ |p|
          next if p.is_omit?
          if p.get_port_type == :ENTRY && ! p.is_VMT_useless? then
            f.print "extern const struct tag_#{p.get_signature.get_global_name}_VMT"
            f.print " #{@global_name}_#{p.get_name}_MT_;\n"
          end
        }
      end
    }
  end

  #=== 受け口ディスクリプタの定義を生成
  def gen_cell_ep_des fs
    if @n_cell_gen >0 then
      fs.each{ |r, f|        f.printf( TECSMsg.get( :EPD_comment ),  "#_EPD_#" ) }
    end

    index = 0
    @ordered_cell_list.each{ |c|

      next if ! c.is_generate?

      f = fs[ c.get_region.get_domain_root ]

      ct = c.get_celltype
      jl = c.get_join_list

      port = ct.get_port_list
      if port.length != 0 then
        port.each{ |p|
          next if p.get_port_type != :ENTRY
          next if p.is_omit?
          if p.is_skelton_useless?       # 受け口最適化n ep_opt
            f.print( "/* #{p.get_name} : omitted by entry port optimize */\n" )
            next
          end

          len = p.get_array_size
          if len == "[]" then
            len = c.get_entry_port_max_subscript(p) + 1
          end

          if len != nil then
            # 受け口配列の場合
            i = 0
            while i < len
              f.print "extern const struct tag_#{@global_name}_#{p.get_name}_DES"
              f.print " #{c.get_global_name}_#{p.get_name}_des#{i};\n"
              f.print "const struct tag_#{@global_name}_#{p.get_name}_DES"
              # f.print " #{c.get_name}_#{p.get_name}_des#{i} = {\n"
              f.print " #{c.get_global_name}_#{p.get_name}_des#{i} = {\n"
              if p.is_VMT_useless? then
                f.print "    0,\n"
              else
                f.print "    &#{@global_name}_#{p.get_name}_MT_,\n"
              end
              if( @idx_is_id_act )then
                f.print "    #{c.get_id},           /* ID */\n"
              else
                if has_CB? then
                  if @singleton then
                    f.print "    &#{@global_name}_SINGLE_CELL_CB,        /* CB */\n"
                  else
                    # f.print "    &#{@global_name}_#{c.get_name}_CB,\n"
                    f.print "    &#{@global_name}_CB_tab[#{index}],      /* CB */\n"
                  end
                elsif has_INIB? then
                  if @singleton then
                    f.print "    &#{@global_name}_SINGLE_CELL_INIB,      /* INIB */\n"
                  else
                    f.print "    &#{@global_name}_INIB_tab[#{index}],    /* INIB */\n"
                  end
                else
                  f.print "    0,\n"
                end
              end
              f.print "    #{i}\n"
              f.print "};\n"
              i += 1
            end
          else
            f.print "extern const struct tag_#{@global_name}_#{p.get_name}_DES"
            f.print " #{c.get_global_name}_#{p.get_name}_des;\n"
            f.print "const struct tag_#{@global_name}_#{p.get_name}_DES"
            # f.print " #{c.get_name}_#{p.get_name}_des = {\n"
            f.print " #{c.get_global_name}_#{p.get_name}_des = {\n"
            if p.is_VMT_useless? then
              f.print "    0,\n"
            else
              f.print "    &#{@global_name}_#{p.get_name}_MT_,\n"
            end
            if @idx_is_id_act then
              f.print "    #{c.get_id},     /* ID */\n"
            else
              if has_CB? then
                if @singleton then
                  f.print "    &#{@global_name}_SINGLE_CELL_CB,       /* CB */\n"
                else
                  f.print "    &#{@global_name}_CB_tab[#{index}],     /* CB */\n"
                  # f.print "    &#{@global_name}_#{c.get_name}_CB,\n"
                end
              elsif has_INIB? then
                if @singleton then
                  f.print "    &#{@global_name}_SINGLE_CELL_INIB,     /* INIB */\n"
                else
                  f.print "    &#{@global_name}_INIB_tab[#{index}],   /* INIB */\n"
                end
              else
                f.print "    0,\n"
              end
            end
            f.print "};\n"
          end
        }
      end
      index += 1
    }
  end

  def generate_template_code

    return if is_all_entry_inline?
    return if @b_reuse && ! $generate_all_template
    if ! ( @plugin && @plugin.gen_ep_func? ) then
      return if $generate_no_template     # $generate_all_template より優先される

      # 参考として出力するテンプレートファイルであることを示すために "_templ" を付加する
      fname = "#{$gen}/#{@global_name}_templ.#{$c_suffix}"
    else
      # Plugin により生成されたセルタイプについては、原則的にテンプレートではなく、
      # 修正不要なセルタイプの実装コードを生成する．
      # このため、ファイル名に _temp を付加しない
      fname = "#{$gen}/#{@global_name}.#{$c_suffix}"
    end

    f = AppFile.open(fname)

    unless ( @plugin && @plugin.gen_ep_func? ) then
      f.printf( TECSMsg.get( :template_note ), @name, @name )
    else
      print_note( f, true )
    end

    f.print TECSMsg.get( :preamble_note )
    gen_template_attr_access f
    gen_template_cp_fun f
    # gen_template_types f     # 0805503 追加してみたが、やっぱりやめる
    f.print( " *\n * #[</PREAMBLE>]# */\n\n" )
    f.printf TECSMsg.get( :PAC_comment ), "#_PAC_#"

    gen_template_private_header f
    if ( @plugin ) then
      # このメソッドの引数は plugin.rb の説明を見よ
      @plugin.gen_preamble( f, @singleton, @name, @global_name )
    end

    gen_template_ep_fun f

    f.print TECSMsg.get( :postamble_note )

    if ( @plugin ) then
      # このメソッドの引数は plugin.rb の説明を見よ
      @plugin.gen_postamble( f, @singleton, @name, @global_name )
    end

    f.close
  end

#####  celltype template

  def gen_template_private_header f
    f.print "#include \"#{@global_name}_tecsgen.#{$h_suffix}\"\n\n"
    f.print <<EOT
#ifndef E_OK
#define	E_OK	0		/* success */
#define	E_ID	(-18)	/* illegal ID */
#endif

EOT
  end

  def gen_template_attr_access f

    if @n_attribute_rw > 0 || @n_attribute_ro > 0 || @n_var > 0 then
      f.printf( TECSMsg.get( :CAAM_comment ), "#_CAAM_#" )
    end

    @attribute.each { |a|

      next if a.is_omit?

      f.printf( " * %-16s %-16s %-16s\n", a.get_name, "#{a.get_type.get_type_str} #{a.get_type.get_type_str_post}", "ATTR_#{a.get_name}" )

    }

    @var.each { |v|

      next if v.is_omit?

      f.printf( " * %-16s %-16s %-16s\n", v.get_name, "#{v.get_type.get_type_str} #{v.get_type.get_type_str_post}", "VAR_#{v.get_name}" )
    }

  end

  def gen_template_types f
    f.printf( TECSMsg.get( :TYP_comment ), "#_TYP_#", "#{@global_name}_CB", "#{@name}_IDX" )
  end

  def gen_template_cp_fun f
    if @n_call_port >0 then
      f.print " *\n"
      f.printf( TECSMsg.get( :TCPF_comment ), "#_TCPF_#" )
    end

    @port.each { |p|
      next if p.get_port_type != :CALL
      # next if p.is_omit?

      sig_name = p.get_signature.get_global_name
      con_tmp = p.get_signature.get_context
      if con_tmp then
        context = " context:#{con_tmp}"
      else
        context = ""
      end

      if p.is_optional? then
        optional = " optional:true"
        if p.get_array_size
          is_join = " *   bool_t     is_#{p.get_name}_joined(int subscript)        check if joined\n"
        else
          is_join = " *   bool_t     is_#{p.get_name}_joined()                     check if joined\n"
        end
      else
        optional = ""
        is_join = ""
      end

      if p.is_omit? then
        omit = " omit:true"
      else
        omit = ""
      end

      if p.is_allocator_port? then
        f.print " * allocator port for #{p.get_port_type.to_s.downcase} port:#{p.get_allocator_port.get_name} func:#{p.get_allocator_func_decl.get_name} param: #{p.get_allocator_param_decl.get_name}\n"
      elsif ! p.is_require? then
        f.print " * call port: #{p.get_name} signature: #{sig_name}#{context}#{optional}#{omit}\n#{is_join}"
      else
        f.print " * require port: signature:#{sig_name}#{context}\n"
      end

      p.get_signature.get_function_head_array.each{ |fun|
        
        ft = fun.get_declarator.get_type

        f.printf( " *   %-14s ", ft.get_type.get_type_str )
        if ! p.is_require? || p.has_name? then
          f.print( "#{p.get_name}_#{fun.get_name}(" )
        else
          f.print( "#{fun.get_name}(" )
        end
        delim = ""

#        if ! @singleton then
#          f.print( "#{delim} p_that" )
#          delim = ","
#        end

        if p.get_array_size then
          f.print( "#{delim} subscript" )
          delim = ","
        end

        ft.get_paramlist.get_items.each{ |param|
          # p "type_str: #{param.get_type.get_type_str}"
          f.print( "#{delim} #{param.get_type.get_type_str}" )
          f.print( " #{param.get_name}#{param.get_type.get_type_str_post}" )
          delim = ","
	}
        f.print( " );\n" )

#        subsc = ""
#        subsc = "[subscript]" if p.get_array_size
#
#        if @singleton then
#          f.print( " *        #{@global_name}_#{p.get_name}" )
#        else
#          f.print( " *        (p_that)->#{p.get_name}" )
#        end
#        f.print( "#{subsc}->VMT->#{fun.get_name}(" )
#        f.print( " (p_that)->#{p.get_name}#{subsc}" )
#        ft.get_paramlist.get_items.each{ |param|
#          f.print( ", (#{param.get_name})" )
#        }
#        f.print( " )\n" )

      }

      if p.get_array_size then
        f.print " *       subscript:  0...(NCP_#{p.get_name}-1)\n"
      end

      if p.is_ref_desc? then
        subsc = p.get_array_size ? ' int_t subscript ' : ''
        f.print " *   [ref_desc]\n"
        f.printf( " *      %-14s %s;\n",
                  "Descriptor( #{p.get_signature.get_global_name} )", 
                  "#{p.get_name}_refer_to_descriptor(#{subsc})" )
        f.printf( " *      %-14s %s;\n",
                  "Descriptor( #{p.get_signature.get_global_name} )", 
                  "#{p.get_name}_ref_desc(#{subsc})      (same as above; abbreviated version)" )
      end
      if p.is_dynamic? then
        subsc = p.get_array_size ? 'int_t subscript, ' : ''
        subsc2 = p.get_array_size ? ' int_t subscript' : ''
        if p.is_optional? then
          f.print " *   [dynamic, optional]\n"
        else
          f.print " *   [dynamic]\n"
        end
        f.printf( " *      %-14s %s;\n",
                  "void",
                  "#{p.get_name}_set_descriptor( #{subsc}Descriptor( #{p.get_signature.get_global_name} ) desc )" )
        if p.is_optional? then
          f.printf( " *      %-14s %s;\n",
                    "void",
                    "#{p.get_name}_unjoin( #{subsc2} )" )
        end
      end
    }

  end

  def gen_template_ep_fun( f, b_inline = false )

    if @n_entry_port >0 then
      f.printf( TECSMsg.get( :TEPF_comment ), "#_TEPF_#" )
    end

    @port.each { |p|
      next if p.get_port_type != :ENTRY
      next if p.is_omit?
      next if b_inline && ! p.is_inline?  # inline ポート
      next if ! b_inline && p.is_inline?  # 非 inline ポート

      inline_prefix = ""
      nCELLIDX = "CELLIDX"
      nCELLCB = "CELLCB"
      nVALID_IDX = "VALID_IDX"
      nGET_CELLCB = "GET_CELLCB"

      f.print <<EOT
/* #[<ENTRY_PORT>]# #{p.get_name}
 * entry port: #{p.get_name}
 * signature:  #{p.get_signature.get_global_name}
 * context:    #{p.get_signature.get_context}
EOT

      if p.get_array_size != nil then
        f.print <<EOT
 * entry port array size:  NEP_#{p.get_name}
EOT
      end

      f.print <<EOT
 * #[</ENTRY_PORT>]# */

EOT

      p.get_signature.get_function_head_array.each{ |fun|
        f.print <<EOT
/* #[<ENTRY_FUNC>]# #{p.get_name}_#{fun.get_name}
 * name:         #{p.get_name}_#{fun.get_name}
 * global_name:  #{@global_name}_#{p.get_name}_#{fun.get_name}
 * oneway:       #{fun.is_oneway?}
 * #[</ENTRY_FUNC>]# */
EOT

        if b_inline then
          f.print "Inline "
        end
        functype = fun.get_declarator.get_type
        f.printf "%s\n", functype.get_type_str
        f.print "#{inline_prefix}#{p.get_name}_#{fun.get_name}("
        if @singleton then
          delim = ""
        else
          f.print "#{nCELLIDX} idx"
          delim = ", "
        end

        if p.get_array_size then
          f.print "#{delim}int_t subscript"
          delim = ", "
        end

        if functype.get_paramlist then
          items = functype.get_paramlist.get_items
          len = items.length
        else
          # ここで nil になるのは、引数なしの時に void がなかった場合
          items = []
          len = 0
        end

        i = 0
        items.each{ |param|
          f.print "#{delim}"
          delim = ", "
          f.print param.get_type.get_type_str
          # p "type_str2: #{param.get_type.get_type_str}"
          f.print " "
          f.print param.get_name
          f.print param.get_type.get_type_str_post
          i += 1
        }
        f.print ")\n"

        f.print "{\n"

        if ( @plugin && @plugin.gen_ep_func? ) then
          # このメソッドの引数は plugin.rb の説明を見よ
          @plugin.gen_ep_func_body( f, @singleton, @name, @global_name, p.get_signature.get_global_name, p.get_name, fun.get_name, "#{@global_name}_#{p.get_name}_#{fun.get_name}", functype, items )

        else
          if ! @singleton then
            if functype.get_type.kind_of?( DefinedType ) && ( functype.get_type.get_type_str == "ER" || functype.get_type.get_type_str == "ER_UINT" ) then
              if ! fun.is_oneway? then
                f.print "	ER\t\tercd = E_OK;\n"
                er_cd = "return(E_ID);"
                ret_cd = "return(ercd);"
              else
                er_cd = "#{TECSMsg.get(:oneway_ercd_note)}\n		return(E_OK);"
                ret_cd = "#{TECSMsg.get(:oneway_ercd_note)}\n	return(E_OK);"
              end
            else
              er_cd = "#{TECSMsg.get(:ercd_note)}"
              ret_cd = nil
            end
            f.print <<EOT
	#{nCELLCB}	*p_cellcb;
	if (#{nVALID_IDX}(idx)) {
		p_cellcb = #{nGET_CELLCB}(idx);
	}
	else {
		#{er_cd}
	} /* end if #{nVALID_IDX}(idx) */

EOT
            f.printf( TECSMsg.get( :TEFB_comment ), "#_TEFB_#" )
            f.printf( "\n" )

            if ret_cd then
              f.print "	#{ret_cd}\n"
            end
          else
          end # ! @singleton
        end # @plugin

        f.print "}\n\n"
      }
    }
  end

  def generate_inline_template_code
    return if @n_entry_port_inline == 0
    if ! ( @plugin && @plugin.gen_ep_func? ) then
      return if @b_reuse && ! $generate_all_template
      return if $generate_no_template     # $generate_all_template より優先される

      # 参考として出力するテンプレートファイルであることを示すために "_templ" を付加する
      fname = "#{$gen}/#{@global_name}_inline_templ.#{$h_suffix}"
    else
      # Plugin により生成されたセルタイプについては、原則的にテンプレートではなく、
      # 修正不要なセルタイプの実装コードを生成する．
      # このため、ファイル名に _temp を付加しない
      fname = "#{$gen}/#{@global_name}_inline.#{$h_suffix}"
    end
    f = AppFile.open(fname)

    gen_ph_guard f, "_INLINE"

    unless ( @plugin && @plugin.gen_ep_func? ) then
      f.printf( TECSMsg.get( :inline_template_note ), @name, @name )
    else
      print_note( f, true )
    end

    f.print TECSMsg.get( :preamble_note )
    gen_template_attr_access f
    gen_template_cp_fun f
    f.print( " *\n * #[</PREAMBLE>]# */\n\n" )

    gen_template_ep_fun( f, true )

    f.print TECSMsg.get( :postamble_note )

    if ( @plugin ) then
      # このメソッドの引数は plugin.rb の説明を見よ
      @plugin.gen_postamble( f, @singleton, @name, @global_name )
    end

    f.print "\n"
    gen_ph_endif f, "INLINE"
    f.close
  end


##### generate tecsgen.cfg

  def generate_celltype_factory_code

    @ct_factory_list.each { |fa|
      if fa.get_name == :write then

        # 前後の " を取り除く
        # file_name = fa.get_file_name.sub( /^\"(.*)\"$/, "\\1" )
        file_name = CDLString.remove_dquote fa.get_file_name
        format = CDLString.remove_dquote fa.get_format
        # format    = fa.get_format.sub( /^\"(.*)/, "\\1" )        # 前の " を取り除く
        # format    = format.sub( /(.*)\"\z/, "\\1" )              # 後の " を取り除く
        format    = format.gsub( /\\\n/, "\n" )                  # \\\n => \n


        # mikan 以下は subst_name で置換するように変更すべき
        file_name = file_name.gsub( /(^|[^\$])\$ct\$/, "\\1#{@name}" )   # $ct$ をセルタイプ名に置換
        file_name = file_name.gsub( /(^|[^\$])\$ct_global\$/, "\\1#{@global_name}" )   # $ct$ をセルタイプ名に置換
        format    = format.gsub( /(^|[^\$])\$ct\$/, "\\1#{@name}" )   # $ct$ をセルタイプ名に置換
        format    = format.gsub( /(^|[^\$])\$ct_global\$/, "\\1#{@global_name}" )   # $ct$ をセルタイプ名に置換
        format    = format.gsub( /\$\$/, "\$" )                # $$ を $ に置換

        if file_name[0] != ?/ then
          file_name = "#{$gen}/#{file_name}"
        end

        begin
          cfg_file = AppFile.open( file_name )
          if $debug then
            print "'#{@name}' : celltype factory format: "
            puts( format )
          end
          # format 中の \n, \r, \t, \f と \" などを置換
          fmt = CDLString.escape format
          cfg_file.print( fmt )
          cfg_file.puts( "\n" )
          cfg_file.close()
        rescue => evar
          cdl_error( "H1004 \'$1\' : write error while writing factory (specify -t to get more info)" , file_name )
          print_exception( evar )
        end
      end
    }
  end

  def generate_cell_factory_code

    @ordered_cell_list.each{ |c|

      # cell のプロトタイプ宣言なら無視
      next if ! c.is_generate?

      name_array = get_name_array( c )

      @factory_list.each { |fa|

        if fa.get_name == :write then

          # 前後の " を取り除く
          # file_name = fa.get_file_name.sub( /^\"(.*)\"$/, "\\1" )
          file_name = CDLString.remove_dquote fa.get_file_name
          file_name = subst_name( file_name, name_array )
          # format    = fa.get_format.sub( /^\"(.*)\"$/, "\\1" )        # 前後の "" を取り除く
          format    = CDLString.remove_dquote fa.get_format
          # format    = fa.get_format.sub( /^\"(.*)/, "\\1" )        # 前の " を取り除く
          # format    = format.sub( /(.*)\"\z/, "\\1" )              # 後の " を取り除く
          format    = format.gsub( /\\\n/, "\n" )                  # \\\n => \n

          format    = subst_name( format, name_array )

          arg_list  = fa.get_arg_list

          if file_name[0] != ?/ then
            file_name = "#{$gen}/#{file_name}"
          end

          na = []     # シンボルを attribute の値に置き換えた後の引数
          if arg_list then
            arg_list.each { |a|
              case a[0]
              when :STRING_LITERAL   # 文字列定数
                # s = a[1].sub( /^\"(.*)\"$/, "\\1" )            # 前後の "" を取り除く
                s = CDLString.remove_dquote a[1]
                s = subst_name( s, name_array )
                # s = subst_name( a[1], name_array )

                na << s
              when :IDENTIFIER
                param_name = a[1]    # 識別子（属性の名前）
                attr = self.find( param_name )
                init = attr.get_initializer      # celltype で指定された初期値

                # cell の join のリストから名前を探す
                j = c.get_join_list.get_item( param_name )
                if j then    # param_name の cell のジョインがあるか
                  init = j.get_rhs                    # cell で指定された初期値を優先
                end

                str = gen_cell_cb_init( nil, c,    name_array, attr.get_type, init, attr.get_identifier, 0,   true )
                                      # file,cell, name_array, type,          init, identifier,       indent, f_get_str

                # str = str.sub( /^\"(.*)\"$/, "\\1" )            # 前後の "" を取り除く mikan ここで置換でよい？
                str = CDLString.remove_dquote str
                na << str
              end
            }
          end

          begin
            cfg_file = AppFile.open( file_name )

            if $debug then
              print "'#{c.get_name}' : factory format: "
              print( format )
              print( " arg: " )
              na.each { |n| print "'#{n}' " }
              print( "\n" )
            end

            # format 中の \n, \r, \t, \f と \" などを置換
            fmt = CDLString.escape format
            cfg_file.printf( fmt, *na )
            cfg_file.puts( "\n" )
            cfg_file.close()
          rescue => evar
            cdl_error( "H1005 \'$1\' : write error while writing factory (specify -t to get more info)" , file_name )
            print_exception( evar )
          end
        end
      }
    }

  end

  def generate_makefile
    generate_makefile_template
    generate_makefile_depend
  end

  def generate_makefile_template

    return if $generate_no_template

    # Makefile.templ の生成（追記）

    f = AppFile.open( "#{$gen}/Makefile.templ" )
    f.print <<EOT
$(_TECS_OBJ_DIR)#{@global_name}.o : #{@global_name}.#{$c_suffix}
	$(CC) -c $(CFLAGS) -o $@ $<
 
EOT
# この生成規則は2点で意味がない
#  ・$(GEN_DIR) に .o を生成するルールがない
#  ・テンプレートコードをそのままビルドするのは紛らわしい
# # Celltype: #{@name}
# $(GEN_DIR)/#{@global_name}_tecsgen.o : $(GEN_DIR)/#{@global_name}_tecsgen.#{$c_suffix}
# 	$(CC) -c $(CFLAGS) -o $@ $<
# 
# $(GEN_DIR)/#{@global_name}_templ.o : $(GEN_DIR)/#{@global_name}_templ.#{$c_suffix}
# 	$(CC) -c $(CFLAGS) -o $@ $<
# 

    f.close

  end

  def generate_makefile_depend

    headers = [ "$(GEN_DIR)/#{@global_name}_tecsgen.#{$h_suffix}", "$(GEN_DIR)/#{@global_name}_factory.#{$h_suffix}", "$(GEN_DIR)/global_tecsgen.#{$h_suffix}" ]

    # inline 受け口を持つか？
    if @n_entry_port_inline > 0 then
      headers << "#{@global_name}_inline.#{$h_suffix}"
    end

    # 呼び口または受け口のシグニチャのヘッダ
    @port.each { |p|
      next if p.is_omit?
      headers << "$(GEN_DIR)/#{p.get_signature.get_global_name}_tecsgen.#{$h_suffix}"
    }

    headers += get_depend_header_list
    headers.sort!
    headers.uniq!
    headers = headers.join " "

    f = AppFile.open( "#{$gen}/Makefile.depend" )

#    print_Makefile_note f

    f.print <<EOT
# Celltype: #{@name}  #_MDEP_#
$(_TECS_OBJ_DIR)#{@global_name}_tecsgen.o : #{@global_name}_tecsgen.#{$c_suffix} #{headers}
$(_TECS_OBJ_DIR)#{@global_name}_templ.o : #{@global_name}_templ.#{$c_suffix} #{headers}
$(_TECS_OBJ_DIR)#{@global_name}.o : #{@global_name}.#{$c_suffix} #{headers}

EOT
    f.close
  end

  #=== decl 用の dealloc コードを生成
  #b_reset:: Bool:  リセット用の dealloc コードの生成 (NULL ポインタの場合 dealloc しない)
  # mikan string 修飾されたポインタの先にポインタが来ないと仮定。ポインタ型を持つ構造体の可能性を排除していない
  # このメソッドでは、行を出力する直前に " \\\n" を出力し、行末で改行文字を出力しない
  def gen_dealloc_code_for_type( f, type, dealloc_func_name, pre, name, post, level, b_reset, count_str = nil )
    type = type.get_original_type
    indent = "	" + "  " * (level+1)
    if ! type.has_pointer?
      return
    elsif type.kind_of?( ArrayType ) then
      if type.get_type.has_pointer?
        loop_str = "i#{level}__"
        count_str = "#{type.get_subscript.eval_const(nil)}"
        f.print " \\\n"
        f.print "#{indent}{ int_t  #{loop_str};"
        f.print " \\\n"
        f.print "#{indent}  for( #{loop_str} = 0; #{loop_str} < #{count_str}; #{loop_str}++ ){ "
        
        gen_dealloc_code_for_type( f, type.get_type, dealloc_func_name, pre, name, "#{post}[#{loop_str}]", level+2, b_reset )

        f.print " \\\n"
        f.print "#{indent}  }"
        f.print " \\\n"
        f.print "#{indent}}"
      end
    elsif type.kind_of?( StructType ) then
      members_decl = type.get_members_decl
      members_decl.get_items.each { |md|
        pre2 = pre + name.to_s + post + "."
        name2 = md.get_name
        post2 = ""
        type2 = md.get_type.get_original_type
        if type2.kind_of? PtrType then   # mikan typedef された型
          if type2.get_count then
            count_str = type2.get_count.to_str( members_decl, pre2, post2 )
          elsif type2.get_size then
            count_str = type2.get_size.to_str( members_decl, pre2, post2 )
          else
            count_str = nil
          end
        else
            count_str = nil
        end
        gen_dealloc_code_for_type( f, md.get_type, dealloc_func_name, pre2, name2, post2, level, b_reset, count_str )
      }

    elsif type.kind_of?( PtrType ) then

      if b_reset || type.is_nullable? then
        nullable = ""
        if( !b_reset && type.is_nullable? )then
          nullable = "\t/* nullable */"
        end
        level2 = level + 1
        indent2 = indent + "  "
        f.print " \\\n"
        f.print "#{indent}if( #{pre}#{name}#{post} ){#{nullable}"
      else
        level2 = level
        indent2 = indent
      end

      if type.get_type.has_pointer?
        if count_str then
          loop_str = "i#{level}__"
          f.print " \\\n"
          f.print "#{indent2}{ int_t  #{loop_str};"
          f.print " \\\n"
          f.print "#{indent2}  for( #{loop_str} = 0; #{loop_str} < #{count_str}; #{loop_str}++ ){ "

          gen_dealloc_code_for_type( f, type.get_type, dealloc_func_name, pre, name, "#{post}[#{loop_str}]", level2+2, b_reset )

          f.print " \\\n"
          f.print "#{indent2}  }"
          f.print " \\\n"
          f.print "#{indent2}}"
        else
          gen_dealloc_code_for_type( f, type.get_type, dealloc_func_name, "(*#{pre}", name, "#{post})", level2, b_reset )
        end
      end
      f.print " \\\n"
      f.print "#{indent2}#{dealloc_func_name}( #{pre}#{name}#{post} ); "

      if b_reset || type.is_nullable? then
        f.print " \\\n"
        f.print "#{indent}}"
      end
    else
      raise "UnknownType"
    end
  end

  def get_depend_header_list
    get_depend_header_list_( [] )
  end

  def get_depend_header_list_( celltype_list )
    headers = []

    if celltype_list.include? self then
      return headers
    else
      celltype_list << self
    end

    # 呼び口の結合先のセルタイプのヘッダ（最適化の場合のみ）
    # 結合先の受け口が inline の場合、inline ヘッダも
    @port.each { |p|
      next if p.get_port_type != :CALL
      next if p.is_omit?

      if p.is_skelton_useless? || p.is_cell_unique? || p.is_VMT_useless? then
        # 最適化コード (optimize) # スケルトン不要など
        p2 = p.get_real_callee_port
        if p2 then
          ct = p2.get_celltype
          headers << " $(GEN_DIR)/#{ct.get_global_name}_tecsgen.#{$h_suffix}"
          if p2.is_inline? then
            headers << " #{ct.get_global_name}_inline.#{$h_suffix}"
          end
          headers += ct.get_depend_header_list_( celltype_list )
        #else
        #  optional で未結合  
        end
      end
    }
    return headers
  end


  #=== $id$, $ct$, $cb$, $idx$ 置換
  #
  #  str に以下の置換を行う
  #-   $ct$ ⇒ セルタイプ名(ct)
  #-   $cell$ ⇒ セル名(cell)   cell が nil ならば3つの置換は行われない
  #-   $cb$ ⇒ CB の C 言語名(cb)
  #-   $cbp$ ⇒ CB へのポインタ(cbp)
  #-   $cb_proto$ ⇒ CB の C 言語名プロトタイプ宣言用(cb_proto)
  #-   $id$ ⇒ ct_cell
  #-   $idx$ ⇒ idx
  #-   $ID$ ⇒ id (整数の番号)
  #-   $ct_global$ ⇒ セルタイプ名(ct)
  #-   $cell_global$ ⇒ セル名(cell)   cell が nil ならば3つの置換は行われない
  #-   $$   ⇒ $
  def subst_name( str, name_array )
    ct   = name_array[0]    # celltype name
    cell = name_array[1]    # cell name
    cb   = name_array[2]    # cell CB name
    cb_init = name_array[3] # cell CB INIT, これは置換に使われない
    cb_proto = name_array[4] # cell CB name for prototype
    id   = name_array[6]    # cell ID
    idx  = name_array[7]    # cell CB name for prototype
    cbp  = name_array[8]    # cell CB pointer
    ct_global = name_array[9]    # cell CB pointer
    cell_global  = name_array[10]    # cell CB pointer

    str = str.gsub( /(^|[^\$])\$ct\$/, "\\1#{ct}" )
    if cell then
      str = str.gsub( /(^|[^\$])\$cell\$/, "\\1#{cell}" )
      str = str.gsub( /(^|[^\$])\$cb\$/, "\\1#{cb}" )
      str = str.gsub( /(^|[^\$])\$id\$/, "\\1#{ct}_#{cell}" )
      str = str.gsub( /(^|[^\$])\$cb_proto\$/, "\\1#{cb_proto}" )
      str = str.gsub( /(^|[^\$])\$ID\$/, "\\1#{id}" )
      str = str.gsub( /(^|[^\$])\$idx\$/, "\\1#{idx}" )
      str = str.gsub( /(^|[^\$])\$cbp\$/, "\\1#{cbp}" )
      str = str.gsub( /(^|[^\$])\$ct_global\$/, "\\1#{ct_global}" )
      str = str.gsub( /(^|[^\$])\$cell_global\$/, "\\1#{cell_global}" )
    end
    str = str.gsub( /\$\$/, "\$" )                       # $$ を $ に置換

    return str
  end

end

# Appendable File（追記可能ファイル）
class AppFile
  # 開いたファイルのリスト
  @@file_name_list = {}

  def self.open( name )
    if $force_overwrite
      real_name = name
    else
      real_name = name+".tmp"
    end

#2.0
    if $b_no_kcode then 
      mode = ":" + $Ruby19_File_Encode
    else
      mode = ""
    end

    # 既に開いているか？
    if @@file_name_list[ name ] then
#2.0
      mode = "a" + mode
      # 追記モードで開く
      file = File.open( real_name, mode )
    else
#2.0
      mode = "w" + mode
      # 新規モードで開く（既にあれば、サイズを０にする）
      file = File.open( real_name, mode )
      @@file_name_list[ name ] = true
    end
    # File クラスのオブジェクトを返す
    return file
  end

  def self.update
    if $force_overwrite
      return
    end

    @@file_name_list.each{ |name,boo|
      b_identical = false
      if File.readable? name
        old_lines = File.readlines name
        new_lines = File.readlines name + ".tmp"
        if old_lines.length == new_lines.length then
          i = 0
          len = old_lines.length
          while i < len
            if old_lines[i] != new_lines[i]
              break
            end
            i += 1
          end
          if i == len
            b_identical = true
          end
        end
      end
      if b_identical == false then
        if $verbose then
          print "#{name} updated\n"
          print "renaming '#{name}.tmp' => '#{name}'\n"
        end
        File.rename name+".tmp", name
      else
        if $verbose then
          print "#{name} not updated\n"
        end
        File.delete name+".tmp"
      end
    }
  end
end

class MemFile
  def initialize
    @string = ""
  end
  def print str
    @string += str
  end
  def get_string
    @string
  end
end

class Region

  def gen_region_str_pre f
    nest = 1
    while nest < @family_line.length
      f.print "  " * ( nest-1 )
      f.print "region #{@family_line[ nest ].get_name}{\n"
      nest += 1
    end
    return nest - 1
  end

  def gen_region_str_post f
    nest = @family_line.length - 1
    while nest >= 1
      f.print "  " * ( nest-1 )
      f.print "};\n"
      nest -= 1
    end
    return nest - 1
  end
end
