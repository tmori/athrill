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
#   $Id: DomainPlugin.rb 2640 2017-06-03 11:27:12Z okuma-top $
#++

#== ドメインプラグインの親クラス
class DomainPlugin < Plugin

  #== domain 指定されたリージョンが定義された
  # region で domain 指定があった
  #domain_type_name::Symbol : domain 指定子の第一引数
  #option::String : domain 指定子の第二引数
  def initialize( region, domain_type_name, option )
  end

  #== 結合 join にプラグインを挿入する
  #join::Join : 結合に関する情報
  #return::[ plugin_name::String, option::String ]: 挿入するプラグイン。 挿入するものがなければ nil を返す
  #return::(1) [ plugin_name, option ] or (2) [[ plugin_name, option ], ...], or [] or nil:  (2): not supported now
  #   nil: region 間の結合禁止, []: region 間の結合可(プラグイン挿入無し)
  #   [ plugin_name, option ]: *_through( plugin_name, option ) 指定したのと同等
  #
  # region 間の through が指定されている場合、out_through, to_through
  # では、それらによって指定されたプラグインの後ろに、in_through の場合
  # in_through プラグインの前に挿入される
  #
  # このメソッドが呼出された時点では Join#get_rhs_cell など意味解析後
  # にしか呼び出せないメソッドを呼出しても、有効な値は得られない
  #
  def add_through_plugin( join, current_region, next_region, through_type )
    # join.get_owner:Cell  左辺のセル
    # join.get_definition:Port 呼び口
    # join.get_subscript:Integer or nil 呼び口配列の添数 (Join::@subscript の説明参照)
    # join.get_cell:Cell 右辺のセル
    # join.get_port_name:Symbol 受け口
    # get_rhs_subscript:Integer or nil 受け口配列の添数 (Join::@rhs_subscript の説明参照)
    # return []
    return nil
  end

  #== require が結合可能か？
  def joinable?(current_region, next_region, through_type )
    return false
  end
end
