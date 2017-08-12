# -*- coding: utf-8 -*-
#
#  mruby => TECS bridge
#  
#   Copyright (C) 2008-2014 by TOPPERS Project
#
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
#   $Id: Mruby2CBridgePlugin.rb 2061 2014-05-31 22:15:33Z okuma-top $
#

class Mruby2CBridgePlugin < SignaturePlugin
  @@signature_list = { }

  def initialize( signature, option )
    super

    @celltype_name = :"t#{@signature.get_global_name}"
    @class_name = :"T#{@signature.get_global_name}"

    parse_plugin_arg
  end

  def gen_cdl_file( file )
    if( @@signature_list.length == 0 )then
      print_msg "  Mruby2CBridgePlugin: [initialize function] 'void initializeBridge( mrb_state *mrb )' must be called from VM.\n"
      c2tecs = "generate( C2TECSBridgePlugin, nMruby::sInitializeBridge, \"silent=true\" );\n"
    end

    if @@signature_list[ @signature.get_global_name ] then
      @@signature_list[ @signature.get_global_name ] << self
      cdl_warning( "MRCW001 signature '$1' duplicate. ignored current one", @signature.get_namespace_path )
      return
    end

    @@signature_list[ @signature.get_global_name ] = [self]
    print_msg "  Mruby2CBridgePlugin: [object creattion]    object = TECS::#{@class_name}.new( 'C#{@signature.get_global_name}' )\n"
    print_msg "  Mruby2CBridgePlugin: [function call]       result = object.function( params )  # substitute 'function' and params \n"

    cf = CFile.open( "#{$gen}/Mruby2C_tsInitializerBridge.cdl", "w")
    cf.print <<EOT
cell nC2TECS::tnMruby_sInitializeBridge C2TECS_tsInitializeBridge{
    cCall = VM_TECSInitializer.eInitialize;
};
EOT
    cf.close

    file.print <<EOT
generate( MrubyBridgePlugin, #{@signature.get_namespace_path}, "silent=true" );
generate( TECS2CBridgePlugin, #{@signature.get_namespace_path}, "silent=true" );
#{c2tecs}
cell nMruby::t#{@signature.get_name} C#{@signature.get_name}{
    cTECS = TECS2C_#{@signature.get_name}.eEnt;
};
cell nTECS2C::t#{@signature.get_name} TECS2C_#{@signature.get_name}{};

import( "Mruby2C_tsInitializerBridge.cdl" );
EOT

  end
end
