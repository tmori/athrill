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
#   $Id: ThroughPlugin.rb 2061 2014-05-31 22:15:33Z okuma-top $
#++

# mikan through plugin: namespace が考慮されていない

#==  スループラグインの共通の親クラス　かつ （何もせず）スルーするセルを挿入するスループラグイン
#    スループラグインは ThroughPlugin の子クラスとして定義する
class ThroughPlugin < Plugin
#@cell_name::      Symbol             生成するセル名（複数セルを生成する場合、受け口側のセル）
#@plugin_arg_str:: string             through で指定された引数
#@next_cell:: Cell                    呼び口を結合するセル
#@next_cell_port_name:: Symbol       呼び口を結合する受口の名前
#@signature::      Signature          シグニチャ
#@celltype::       Celltype           呼び先のセルのセルタイプ. through が連接する場合、最終的な呼び先のセルのセルタイプ
#@entry_port_name::Symbol             生成するセルの受け口名  "eThroughEntry"
#@call_port_name:: Symbol             生成するセルの呼び口名  "cCall"
#@ct_name::        Symbol             生成するセルのセルタイプ名   "t#{self.class.name}_#{@signature.get_global_name}"
#@plugin_arg_list:: Hash              プラグイン引数をパースした結果のハッシュ変数
#@caller_cell::    Cell               呼び元のセル．through プラグインが連接する場合では、最も呼び元のセル．($source$)
#                                     through プラグインが合流するケースでは、1つ目の呼び元セルのみ引数として与えられる
#                                     従って TracePlugin の呼び元の判別に利用する場合は、異なる呼び元から呼ばれる可能性があることに注意しなくてはならない
#@callee_cell:: Cell                  呼び先のセル($destination$)
#@plugin_arg_check_proc_tab:: [string => Proc]  プラグイン引数名⇒チェック関数
# 以下の変数は、initialize ではなく、後から設定される
#@start_@region::  Region             始まりのリージョン： caller_cell のリージョンとは異なる可能性がある ($start_region$)
#@end_region::  Region                終わりのリージョン： next_cell のリージョンとは異なる可能性がある ($end_region$)
#@region:: Region                     @start_region と @end_region のいずれかで、cell を置くのが好ましいリージョン ($preferred_region$)
#@through_type:: Symbol              :THROUGH, :TO_THROUGH, :IN_THROUGH, :OUT_THROUGH のいずれか

  # この Plugin が生成したセルタイプのリスト
  @@generated_celltype = {}

  #=== ThroughPlugin の初期化
  #     through が指定された時点で生成が行われる
  #         初期化では、指定された引数を記録するに留める
  #cell_name::      Symbol             生成すべきセル名（受口側）
  #plugin_arg::     string             through で指定された引数
  #next_cell::      Cell               呼び口を接続するセル
  #next_cell_port_name:: Symbol        呼び口を接続する受口の名前
  #signature::      Signature          シグニチャ
  #celltype::       Celltype           セルタイプ (呼び先のセルのセルタイプ)
  #caller_cell::    Cell               呼び元のセル．@caller_cell の項を参照
  def initialize( cell_name, plugin_arg, next_cell, next_cell_port_name, signature, celltype, caller_cell )
    super()
    @cell_name = cell_name                      # 生成すべきセル名（受け口側のセル名）
                                                # この呼び先に別セルを生成する場合、この名前を接頭辞とすべき
    @next_cell = next_cell                      # 呼び先のセル
    @next_cell_port_name = next_cell_port_name
    @signature = signature
    @entry_port_name = :"eThroughEntry"
    @call_port_name = :"cCall"
    @ct_name = :"t#{self.class.name}_#{@signature.get_global_name}"
    @celltype = celltype
    @plugin_arg_str = plugin_arg
    @plugin_arg_list = {}                       # プラグイン引数をパースした結果のハッシュ変数
    @caller_cell     = caller_cell
    Join.set_through_info self                  # 引数で渡らない(後から追加された)ものは set_through_info で設定される
    print( "#{self.class.name}.new( '#{cell_name}', '#{plugin_arg}', '#{next_cell.get_name}', '#{next_cell_port_name}', #{celltype.get_name} )\n" )
  end

  #=== 情報を設定する
  # 共有チャンネルの場合 caller_cell, next_cell のいずれの region でもないケースがある
  # 後から追加したので initialize の引数ではなく、別メソッドで設定
  # このメソッドは、オーバーライドしないでください
  # Join と ThrougPlugin の間の連絡用で、今後とも引数が追加される可能性があるため
  # このメソッドは V1.C.0.34 で位置が移動され、ThroughPlugin#initialize で呼び出される
  def set_through_info( start_region, end_region, through_type, join, callee_cell, count )
    @start_region = start_region
    @end_region = end_region
    @through_type = through_type
    @join = join
    @callee_cell = callee_cell
    @count = count

    # preferred_region の設定
    case through_type
    when :IN_THROUGH, :THROUGH
      @region = end_region
    when :OUT_THROUGH, :TO_THROUGH
      @region = start_region
    else
      raise "Unknown through_type #{through_type}"
    end
  end

  #===  セルの名前を得る
  def get_cell_name
    @cell_name
  end

  #=== NamespacePath を得る
  # 生成するセルの namespace path を生成する
  def get_cell_namespace_path
#    nsp = @region.get_namespace.get_namespace_path
    nsp = @region.get_namespace_path
    return nsp.append( @cell_name )
  end

  #===  生成されたセルの受け口の名前を得る
  def get_through_entry_port_name
    @entry_port_name
  end

  #===  宣言コードの生成
  #      typedef, signature, celltype など（cell 以外）のコードを生成
  #          重複して生成してはならない（すでに生成されている場合は出力しないこと）
  #file::        FILE       生成するファイル
  def gen_plugin_decl_code( file )

    # このセルタイプ（同じシグニチャ）は既に生成されているか？
    if @@generated_celltype[ @ct_name ] == nil then
      @@generated_celltype[ @ct_name ] = [ self ]
    else
      @@generated_celltype[ @ct_name ] << self
      return
    end

    file2 = CFile.open( "#{$gen}/#{@ct_name}.cdl", "w" )

    send_receive = []
    if @signature != nil then
      @signature.each_param{ |fd,param|
        dir =param.get_direction
        case dir
        when :SEND, :RECEIVE
          send_receive << [ dir, fd, param ]
        end
      }
    end

    file2.print <<EOT
celltype #{@ct_name} {
EOT

    if send_receive.length > 0 then
      file2.print "  [ allocator(\n"
      delim = ""
      send_receive.each { |a|
        file2.print "#{delim}\t#{a[1].get_name}.#{a[2].get_name}<=#{@call_port_name}.#{a[1].get_name}.#{a[2].get_name}"
        delim = ",\n"
      }
      file2.print "\n  )]\n"
    end

    file2.print <<EOT
  entry #{@signature.get_namespace_path} #{@entry_port_name};
  call  #{@signature.get_namespace_path} #{@call_port_name};
};
EOT
    file2.close

    file.print "import( \"#{$gen}/#{@ct_name}.cdl\" );\n"
  end

  #=== CDL ファイルの生成
  #file::     FILE    生成するファイル
  def gen_cdl_file( file )
    gen_plugin_decl_code( file )
    gen_through_cell_code( file )
  end

  #===  セルコードの生成
  #     through 指定により生じるセルコード(CDL)を生成する
  #file::        FILE       生成するファイル
  def gen_through_cell_code( file )

    nest = @region.gen_region_str_pre file
    nest_str = "  " * nest

    file.print <<EOT
#{nest_str}cell #{@ct_name} #{@cell_name} {
#{nest_str}  #{@call_port_name} = #{@next_cell.get_namespace_path.get_path_str}.#{@next_cell_port_name};
#{nest_str}};
EOT
    @region.gen_region_str_post file

  end

  #=== 後ろのコードを生成
  #プラグインの後ろのコード (CDL) を生成
  #file:: File: 
  def self.gen_post_code( file )
    # 複数のプラグインの post_code が一つのファイルに含まれるため、以下のような見出しをつけること
    # file.print "/* '#{self.class.name}' post code */\n"
  end

  #===  受け口関数の本体(C言語)を生成する
  #     通常であれば、ジェネレータは受け口関数のテンプレートを生成する
  #     プラグインの場合、変更する必要のないセルタイプコードを生成する
  #file::           FILE        出力先ファイル
  #b_singleton::    bool        true if singleton
  #ct_name::        Symbol
  #global_ct_name:: string
  #sig_name::       string
  #ep_name::        string
  #func_name::      string
  #func_global_name:: string
  #func_type::      class derived from Type
  def gen_ep_func_body( file, b_singleton, ct_name, global_ct_name, sig_name, ep_name, func_name, func_global_name, func_type, params )

    ret_type = func_type.get_type
    b_ret_void = ret_type.is_void?

    if ! b_ret_void then
      file.print( "  #{ret_type.get_type_str}  retval;\n" )
    end

    if ! b_singleton then

      file.print <<EOT
  #{ct_name}_CB    *p_cellcb;
  if( VALID_IDX( idx ) ){
    p_cellcb = #{global_ct_name}_GET_CELLCB(idx);
  }else{
     /* エラー処理コードをここに記述 */
  }

EOT
    end

    # p "celltype_name, sig_name, func_name, func_global_name"
    # p "#{ct_name}, #{sig_name}, #{func_name}, #{func_global_name}"

    delim = ""
    if ! b_ret_void then
      file.print( "  retval = " )
    end

    file.print( "#{@call_port_name}_#{func_name}(" )

#    if ( ! b_singleton ) then
#      file.print( " tecs_this" )
#      delim = ","
#    end

    params.each{ |param|
      file.printf( "#{delim} #{param.get_name}" )
      delim = ","
    }

    file.print( " );\n" )

    if ! b_ret_void then
      file.print( "  return retval;\n" )
    end
  end

  #=== Through プラグインの引数の名前を置換する
  def check_plugin_arg( ident, rhs )
    rhs = subst_name rhs
    super( ident, rhs )
  end

  #=== ThroughPlugin#名前の置換
  # プラグインオプション引数内の文字列を置換する
  #   $source$       … 呼び元のセル名
  #   $destination$  … 呼び先のセル名
  #   $SOURCE$       … 呼び元のセル名 (リージョン名を '_' で連結した global_name)
  #   $DESTINATION$  … 呼び先のセル名 (リージョン名を '_' で連結した global_name)
  #   $next$         … 次のセル名
  #                     複数の through がつながっている場合、すぐ後ろに来るもの
  #   $NEXT$         … 次のセル名 (リージョン名を '_' で連結した global_name)
  #                     複数の through がつながっている場合、すぐ後ろに来るもの
  #   $start_region$ … $source$ のセルの存在する region (global_name)
  #   $end_region$   … $destination$ のセルの存在する region (global_name)
  #   $preferred_region$  … 適切な region (global_name), start_region または end_region
  #   $count$        … region 間の through の適用数
  #   $$             … $ に置換
  def subst_name( str )
    # セル名の置換
    str = str.gsub( /(^|[^\$])\$source\$/, "\\1#{@caller_cell.get_name}" )
    str = str.gsub( /(^|[^\$])\$destination\$/, "\\1#{@callee_cell.get_name}" )
    str = str.gsub( /(^|[^\$])\$SOURCE\$/, "\\1#{@caller_cell.get_global_name}" )
    str = str.gsub( /(^|[^\$])\$DESTINATION\$/, "\\1#{@callee_cell.get_global_name}" )
    str = str.gsub( /(^|[^\$])\$next\$/, "\\1#{@next_cell.get_name}" )
    str = str.gsub( /(^|[^\$])\$NEXT\$/, "\\1#{@next_cell.get_global_name}" )
    # region 名の置換
    str = str.gsub( /(^|[^\$])\$start_region\$/, "\\1#{@start_region.get_global_name}" )
    str = str.gsub( /(^|[^\$])\$end_region\$/, "\\1#{@end_region.get_global_name}" )
    str = str.gsub( /(^|[^\$])\$preferred_region\$/, "\\1#{@region.get_global_name}" )
    str = str.gsub( /(^|[^\$])\$count\$/, "\\1#{@count}" )

    str = str.gsub( /\$\$/, "\$" )                       # $$ を $ に置換

    return str
  end

  def show_tree( indent )
    indent.times { print "  " }
    puts "Plugin: celltype: #{@ct_name} cell: #{@cell_name}"
    (indent+1).times { print "  " }
    puts "next: signature: #{@signature.get_namespace_path} call = #{@next_cell.get_name}.#{@next_cell_port_name}"
  end
end

