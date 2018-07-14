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
#   $Id: location.rb 2640 2017-06-03 11:27:12Z okuma-top $
#++

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
