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
#   $Id: tool_info.rb 2640 2017-06-03 11:27:12Z okuma-top $
#++

#= TOOL_INFO class
# The syntax of the contents of __tool_info__ is same as JSON.
# Home made schema is used to validate the contents of __tool_info__.
# the schema is not defiened well. see saveload.rb for example of the schema.
# 
class TOOL_INFO

  # tool_info 
  @@tool_info = { }

  # tool_info schema for tecsgen
  @@TECSGEN_schema = {
    :tecsgen        => {   # require
      :base_dir       => [ :string ],             # dir where the cde created initially
      :direct_import  => [ :string ],             # .cdl (sometimes .cde) specified in argments
      :import_path    => [ :string ],             # -I of tecsgen
      :define_macro   => [ :string ],             # -D of tecsgen
      :tecscde_version=> :string,                 # TECSCDE version
      :cde_format_version=> :string,              # CDE format version
      :save_date      => :string,                 # last save date & time
    },
    :__tecsgen      => {   # optioanl
      :cpp            => :string                  # -c or TECS_CPP environment variable
    }
  }

  def initialize( name, val )
    @@tool_info[name] = val

    # __tool_info__( "tecsgen" ): validate & reflect immediately
    p "tool_info: tecsgen #{name}"
    if name == :tecsgen then
      set_tecsgen_tool_info
    end
  end

  def self.get_tool_info name
    @@tool_info[ name ]
  end

  #=== TOOL_INFO#set_tecsgen_tool_info
  # __tool_info__( "tecsgen" )
  def set_tecsgen_tool_info
    validator = TOOL_INFO::VALIDATOR.new( :tecsgen, @@TECSGEN_schema )
    if validator.validate || $b_force_apply_tool_info
      info = TOOL_INFO.get_tool_info( :tecsgen )
      (info[ :base_dir ]).each{ |bd|
        if ! $base_dir.include? bd
          $base_dir[ bd ] = true
        end
      }

      info[ :import_path ].each{ |path|
        if ! $import_path.include?( path )
          $import_path << path
        end
      }

      info[ :define_macro ].each{ |define|
        if ! $define.include?( define )
          $define << define
        end
      }
      if info[ :cpp ]
        if ! $b_cpp_specified
          $cpp = info[ :cpp ]
          $b_cpp_specified = true
        end
      end

      info[ :direct_import ] && info[ :direct_import ].each{ |import|
        Import.new( import, false, false )
      }
    end
  end

  #== simple JSON validator
  # this validator use simple schema.
  # array length cannot be checked.
  # you have to check array length in your code, if necessary.
  class VALIDATOR

    #@b_ok::Bool

    def error( msg )
      STDERR.print( "__tool_info__: " + msg )
      @b_ok = false
    end

    def initialize name, schema
      @name = name
      @schema = schema
      @b_ok = true
    end

    #=== VALIDATOR#validate
    def validate
      info = TOOL_INFO.get_tool_info @name
      if info == nil
        error( "\"#{@name}\" not found\n" )
        return @b_ok
      end
      validate_object info, @name, @name
      return @b_ok
    end

    def validate_object( object, require_type, path )
      obj_type = @schema[ require_type ]
      dbgPrint "validate_object: #{path}  object=#{obj_type[ :type ]}  required=#{object[:type]}\n"

      obj_type.each{ |name, val_type|
        val = object[name]
        path2 = path.to_s + "." + name.to_s
        if val == nil
          error( "#{path}: required property not found '#{name}'\n" )
          next
        end
        if val_type.kind_of? Array
          validate_array_member val, val_type, path2
        else
          validate_types val, val_type, path2
        end
      }

      optional = @schema[ ("__" + require_type.to_s).to_sym ]
      if optional
        dbgPrint "#{require_type.to_s} has optional\n"

        optional.each{ |name, val_type|
          val = object[name]
          path2 = path.to_s + "." + name.to_s
          if val == nil
            # no need to occur error
            # error( "#{path}: required property not found '#{name}'\n" )
            next
          end
          if val_type.kind_of? Array
            validate_array_member val, val_type, path2
          else
            validate_types val, val_type, path2
          end
        }
      end

    end

    def validate_array_member array, val_types, path
      if ! array.kind_of? Array
        error( "#{path2}: array required as value\n" )
        return
      end
      index=0
      array.each{ |member|
        type = get_object_type member
        i = val_types.find_index type
        if i == nil
          if type == :integer
            i = val_types.find_index :number
          end
        end
        if i == nil
          error( "#{path}: array member type mismatch, #{type} for #{val_types}\n" )
          next
        end
        val_type = val_types[ i ]
        validate_types member, val_type, (path.to_s + '[' + index.to_s + '].')
        index += 1
      }
    end

    #=== TOOL_INFO::VALIDATOR#validate_types
    #obj::Object (Integer, Floating, String, Hash)
    #val_type::Symbol : required object type
    def validate_types obj, val_type, path
      type = get_object_type obj
      case val_type
      when :integer
        if type == :integer
          return
        end
      when :number
        if type == :integer || type == :number
          return
        end
      when :string
        if type == :string
          return
        end
        # when :nil   # mikan
        # when :bool  # mikan
      else # object or fixed string
        if val_type.kind_of? String
          # val is specified by String
          if type == :string
            if obj == val_type
              return
            end
          end
        else
          if type.to_sym == val_type
            validate_object( obj, val_type, path + val_type.to_s )
            return
          end
        end
      end
      error( "#{path}: type mismatch, #{type} for #{val_type}\n" )
    end

    def get_object_type obj
# p "#{obj.class} #{obj.to_s}"
      if obj.kind_of? Integer
        return :integer
      elsif obj.kind_of? Float
        return :number
      elsif obj.kind_of? String
        return :string
      elsif obj.kind_of? Hash
        return obj[ :type ].to_sym
      end
      return nil
    end
  end

end


#---------- obsolete -------#

#--- TOOL_INFO replaced location information ---#

class TECSGEN

  #------ manupulate location information --------#
  def self.new_cell_location cell_location
    @@current_tecsgen.new_cell_location cell_location
  end
  def new_cell_location cell_location
    @cell_location_list << cell_location
  end
  def get_cell_location_list
    @cell_location_list
  end

  def self.new_join_location join_location
    @@current_tecsgen.new_join_location join_location
  end
  def new_join_location join_location
    @join_location_list << join_location
  end
  def get_join_location_list
    @join_location_list
  end

#==  Cell_location
  # tecscde の位置情報
  class  Cell_location

    #=== Join_location#initialize
    #cell_nspath::NamespacePath
    #x,y,w,h::Expression
    #port_location_list::[ [Symbol(ep_or_cp_name), Symbol(edge_name), Expression(offset)], ... ]
    #ep_name::Symbol
    def initialize( cell_nspath, x, y, w, h, port_location_list )
      # p "Cell_location: #{cell_nspath}, #{x}, #{y}, #{w}, #{h}, #{port_location_list}"
      @cell_nspath = cell_nspath
      @x = x.eval_const nil
      @y = y.eval_const nil
      @w = w.eval_const nil
      @h = h.eval_const nil
      @port_location_list = port_location_list

      TECSGEN.new_cell_location self
    end

    def get_location
      [ @cell_nspath, @x, @y, @w, @h, @port_location_list ]
    end

  end # Cell_location

  #==  Join_location
  # tecscde の位置情報
  class  Join_location
    @@join_location_list = []

    #=== Join_location#initialize
    #cp_cell_nspath::NamespacePath
    #cp_name::Symbol
    #ep_cell_nspath::NamespacePath
    #ep_name::Symbol
    #bar_list::[[Symbol (VBar or HBar), Expression(position mm)], ....]
    def initialize( cp_cell_nspath, cp_name, ep_cell_path, ep_name, bar_list )
      # p "Join_location  #{cp_cell_nspath}, #{cp_name}, #{ep_cell_path}, #{ep_name} #{bar_list}"
      @cp_cell_nspath = cp_cell_nspath
      @cp_name = cp_name
      @ep_cell_path = ep_cell_path
      @ep_name = ep_name
      @bar_list = bar_list

      TECSGEN.new_join_location self
    end

    def get_location
      [@cp_cell_nspath, @cp_name, @ep_cell_path, @ep_name, @bar_list]
    end

  end # Join_location

end # TECSGEN
