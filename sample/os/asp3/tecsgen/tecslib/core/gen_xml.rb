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
#   $Id: gen_xml.rb 2061 2014-05-31 22:15:33Z okuma-top $
#++

XML_INDENT = "    "

class Namespace
  def self.gen_XML root_namespace
    begin
      file_name = $gen_base + "/" + $target + ".xml"
      dbgPrint "generating XML file:#{file_name}\n"
      file = AppFile.open( file_name )

      file.print <<EOT
<?xml version="1.0" encoding="UTF-8"?>
<GUI_Tool xmlns="http://www.toppers.jp/tecs.html">
EOT
      root_namespace.gen_XML file, 1

      file.print <<EOT
</GUI_Tool>
EOT

      file.close
    rescue => evar
      Generator.error( "T2001 fail to create XML file $1", file_name)
      print_exception( evar )
    end
  end

  def gen_XML( file, nest )
    # signature のコードを生成
    @signature_list.each { |s|
      s.gen_XML file, nest
    }

    # celltype のコードを生成
    @celltype_list.each { |t|
      t.gen_XML( file, nest )
    }

    # composite のコードを生成
    @compositecelltype_list.each { |t|
      t.gen_XML( file, nest )
    }

    # cell のコードを生成
    @cell_list.each { |t|
      t.gen_XML( file, nest )
    }

    # サブネームスペースのコードを生成
    @namespace_list.each { |n|
      kind = n.instance_of?( Namespace ) ? "namespace" : "region"
      file.print <<EOT
#{XML_INDENT * nest}<#{kind}>
#{XML_INDENT * (nest+1)}<name> #{n.get_name} </name>
EOT
      n.gen_XML( file, nest + 1 )
      file.print <<EOT
#{XML_INDENT * nest}</#{kind}>
EOT
    }
  end
end

class Signature
  def gen_XML file, nest
    indent = XML_INDENT * nest
    if is_imported? then
      file.print <<EOT
#{indent}<import path='#{@import.get_cdl_name}'>
EOT
      nest += 1
      indent = XML_INDENT * nest
    end

    file.print <<EOT
#{indent}<signature>
#{indent}#{XML_INDENT}<name> #{@name} </name>
EOT
    @function_head_list.get_items.each{ |fh|
      file.print <<EOT
#{indent}#{XML_INDENT}<func>
#{indent}#{XML_INDENT}#{XML_INDENT}<name> #{fh.get_name} </name>
#{indent}#{XML_INDENT}#{XML_INDENT}<rettype> #{fh.get_return_type.get_type_str}#{fh.get_return_type.get_type_str_post} </rettype>
EOT
      fh.get_paramlist.get_items.each{ |pd|
        file.print <<EOT
#{indent}#{XML_INDENT}#{XML_INDENT}<param>
#{indent}#{XML_INDENT}#{XML_INDENT}#{XML_INDENT}<name> #{pd.get_name} </name>
#{indent}#{XML_INDENT}#{XML_INDENT}#{XML_INDENT}<type> #{pd.get_type.get_type_str}#{pd.get_type.get_type_str_post} </type>
#{indent}#{XML_INDENT}#{XML_INDENT}</param>
EOT
      }
      file.print <<EOT
#{indent}#{XML_INDENT}</func>
EOT
    }
    file.print <<EOT
#{indent}</signature>
EOT
    if is_imported? then
      nest -= 1
      indent = XML_INDENT * nest
      file.print <<EOT
#{indent}</import>
EOT
    end
  end
end

class Celltype
  def gen_XML file, nest
    indent = XML_INDENT * nest
    if is_imported? then
      file.print <<EOT
#{indent}<import path='#{@import.get_cdl_name}'>
EOT
      nest += 1
      indent = XML_INDENT * nest
    end

    file.print <<EOT
#{indent}<celltype>
#{indent}#{XML_INDENT}<name> #{@name} </name>
EOT
    if @active then
      file.print <<EOT
#{indent}#{XML_INDENT}<active />
EOT
    end
    if @singleton then
      file.print <<EOT
#{indent}#{XML_INDENT}<singleton />
EOT
    end

    @attribute.each{ |attr|
      file.print <<EOT
#{indent}#{XML_INDENT}<attr>
#{indent}#{XML_INDENT}#{XML_INDENT}<name> #{attr.get_name} </name>
#{indent}#{XML_INDENT}#{XML_INDENT}<type> #{attr.get_type.get_type_str}#{attr.get_type.get_type_str_post} </type>
EOT
      if attr.get_initializer then
        file.print <<EOT
#{indent}#{XML_INDENT}#{XML_INDENT}<initializer> #{attr.get_initializer.to_CDL_str} </initializer>
EOT
      end

      if attr.get_choice_list then
        file.print <<EOT
#{indent}#{XML_INDENT}#{XML_INDENT}<choice>
EOT

        attr.get_choice_list.each { |choiceElement|
          file.print <<EOT
#{indent}#{XML_INDENT}#{XML_INDENT}#{XML_INDENT}<choiceElement>#{choiceElement}</choiceElement>
EOT
        }

        file.print <<EOT
#{indent}#{XML_INDENT}#{XML_INDENT}</choice>
EOT
      end

      file.print <<EOT
#{indent}#{XML_INDENT}</attr>
EOT
    }
    @port.each{ |port|
      port_type = port.get_port_type == :CALL ? "call" : "entry"
      file.print <<EOT
#{indent}#{XML_INDENT}<#{port_type}>
#{indent}#{XML_INDENT}#{XML_INDENT}<name> #{port.get_name} </name>
#{indent}#{XML_INDENT}#{XML_INDENT}<signame> #{port.get_signature.get_name} </signame>
EOT
      if port.get_array_size then
        file.print <<EOT
#{indent}#{XML_INDENT}#{XML_INDENT}<subscript> #{port.get_array_size} </subscript>
EOT
      end
      file.print <<EOT
#{indent}#{XML_INDENT}</#{port_type}>
EOT
    }
    file.print <<EOT
#{indent}</celltype>
EOT
    if is_imported? then
      nest -= 1
      indent = XML_INDENT * nest
      file.print <<EOT
#{indent}</import>
EOT
    end
  end

end

#
# Celltype と共用可能なはずだが、以下の点の変更が必要
#  @active ⇒ @b_active
#  @singleton ⇒ @b_singleton
#  @attribute ⇒ @name_list (Decl)
#  @port ⇒ @name_list (Port)
class CompositeCelltype
  def gen_XML file, nest
    indent = XML_INDENT * nest
    if is_imported? then
      file.print <<EOT
#{indent}<import path='#{@import.get_cdl_name}'>
EOT
      nest += 1
      indent = XML_INDENT * nest
    end

    file.print <<EOT
#{indent}<celltype>
#{indent}#{XML_INDENT}<name> #{@name} </name>
EOT
    file.print <<EOT
#{indent}#{XML_INDENT}<composite />
EOT

    if @b_active then
      file.print <<EOT
#{indent}#{XML_INDENT}<active />
EOT
    end
    if @b_singleton then
      file.print <<EOT
#{indent}#{XML_INDENT}<singleton />
EOT
    end

    @name_list.get_items.each{ |attr|
      if ! attr.instance_of? Decl then
        next
      end
      file.print <<EOT
#{indent}#{XML_INDENT}<attr>
#{indent}#{XML_INDENT}#{XML_INDENT}<name> #{attr.get_name} </name>
#{indent}#{XML_INDENT}#{XML_INDENT}<type> #{attr.get_type.get_type_str}#{attr.get_type.get_type_str_post} </type>
EOT
      if attr.get_initializer then
        file.print <<EOT
#{indent}#{XML_INDENT}#{XML_INDENT}<initializer> #{attr.get_initializer.to_CDL_str} </initializer>
EOT
      end
      file.print <<EOT
#{indent}#{XML_INDENT}</attr>
EOT
    }
    @name_list.get_items.each{ |port|
      if ! port.instance_of? Port then
        next
      end
      port_type = port.get_port_type == :CALL ? "call" : "entry"
      file.print <<EOT
#{indent}#{XML_INDENT}<#{port_type}>
#{indent}#{XML_INDENT}#{XML_INDENT}<name> #{port.get_name} </name>
#{indent}#{XML_INDENT}#{XML_INDENT}<signame> #{port.get_signature.get_name} </signame>
EOT
      if port.get_array_size then
        file.print <<EOT
#{indent}#{XML_INDENT}#{XML_INDENT}<subscript> #{port.get_array_size} </subscript>
EOT
      end
      file.print <<EOT
#{indent}#{XML_INDENT}</#{port_type}>
EOT
    }
    file.print <<EOT
#{indent}</celltype>
EOT
    if is_imported? then
      nest -= 1
      indent = XML_INDENT * nest
      file.print <<EOT
#{indent}</import>
EOT
    end
  end

end

class Cell
  def gen_XML file, nest
    indent = XML_INDENT * nest

    if is_imported? then
      file.print <<EOT
#{indent}<import path='#{@import.get_cdl_name}'>
EOT
      nest += 1
      indent = XML_INDENT * nest
    end

    file.print <<EOT
#{indent}<cell>
#{indent}#{XML_INDENT}<name> #{@name} </name>
EOT
    @join_list.get_items.each{ |join|
      if join.get_definition.kind_of? Port then
        kind = "call_join"
      elsif join.get_definition.kind_of? Decl then
        kind = "attr_join"
      else
        raise "Unknown"
      end
      if join.get_array_member2 then
        join.get_array_member2.each { |j2|
          file.print <<EOT
#{indent}#{XML_INDENT}<#{kind}>
#{indent}#{XML_INDENT}#{XML_INDENT}<name> #{join.get_name} </name>
#{indent}#{XML_INDENT}#{XML_INDENT}<subscript> #{join.get_subscript} </subscript>
#{indent}#{XML_INDENT}#{XML_INDENT}<rhs> #{join.get_rhs.to_CDL_str} </rhs>
#{indent}#{XML_INDENT}</#{kind}>
EOT
        }
      else
        file.print <<EOT
#{indent}#{XML_INDENT}<#{kind}>
#{indent}#{XML_INDENT}#{XML_INDENT}<name> #{join.get_name} </name>
#{indent}#{XML_INDENT}#{XML_INDENT}<rhs> #{join.get_rhs.to_CDL_str} </rhs>
#{indent}#{XML_INDENT}</#{kind}>
EOT
      end
    }
    file.print <<EOT
#{indent}</cell>
EOT
    if is_imported? then
      nest -= 1
      indent = XML_INDENT * nest
      file.print <<EOT
#{indent}</import>
EOT
    end
  end
end
