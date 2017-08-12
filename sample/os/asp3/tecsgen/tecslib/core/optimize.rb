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
#   $Id: optimize.rb 2633 2017-04-02 06:02:05Z okuma-top $
#++

=begin
This file includes the processes between semantics analysis and code generation.
Optimize is one of them.
Other processes are setting ID for each cell and setting domain information

このファイルには、意味解析からコード生成の間で行うべき処理が含まれる．
最適化もその一つである．
その他に、セル毎の ID 付け、ドメインわけを行う．
コード生成対象となるセルを対象に処理を行うものが含まれる．
=end

class Namespace

  #===  各セルに ID （整数値）を割付ける
  def set_cell_id_and_domain
    # celltype の各セルに ID を割付ける
    @celltype_list.each { |t|
      t.set_cell_id_and_domain
    }

    # サブネームスペースの各セルに ID を割付ける
    @namespace_list.each { |n|
      n.set_cell_id_and_domain
    }
  end

  def optimize
    # celltype の最適化
    @celltype_list.each { |t|
      t.optimize
    }

    # サブネームスペースの最適化
    @namespace_list.each { |n|
      n.optimize
    }
  end

  def reset_optimize
    # celltype の最適化
    @celltype_list.each { |t|
      t.reset_optimize
    }

    # サブネームスペースの最適化
    @namespace_list.each { |n|
      n.reset_optimize
    }
  end
end

class Celltype

  ID_BASE = 1               # reset_optimize でリセットする
  @@ID_BASE = ID_BASE

  def set_cell_id_and_domain
    set_cell_id
    set_domain
  end

  #=== 各セルに ID （整数値）を割付ける
  def set_cell_id

    if $verbose then
      print( "=== id for the cells of celltype #{get_namespace_path.to_s} ===\n" ) 
    end

    if $unique_id then
      @id_base = @@ID_BASE   # id をシステム全体で連番にする
    else
      @id_base = 1           # base を常に 1 から始める
    end

    id_specified_cells = []
    no_id_specified_cells = []

    # プロトタイプを除いた数を求める
    @cell_list.each{ |c|
      if c.is_generate? then
        # c.set_id( @id_base + @n_cell_gen )
        id = c.get_specified_id
        if id then
          id_specified_cells << c
        else
          no_id_specified_cells << c
        end
        
        # p "#{c.get_name} #{@id_base+@n_cell_gen}"
        @@ID_BASE     += 1
        @n_cell_gen   += 1
      end
    }

    @ordered_cell_list = []   # id = 1 が添数 0 に格納される
    # ID 指定されているセルに id 番号を与える
    id_specified_cells.each{ |c|
      id = c.get_specified_id
      if id > 0 then
        if id >= @n_cell_gen then
          cdl_error( "S3001 $1: id too large $2 (max=$3)", c.get_name, id, @n_cell_gen )
          next
        end
      else
        if - id >= @n_cell_gen then
          cdl_error( "S3002 $1: id too large $2 (max=$3)", c.get_name, id, @n_cell_gen )
          next
        end
        id = @n_cell_gen + id + 1
      end

      if @ordered_cell_list[ id - 1 ] then
        cdl_error( "S3003 $1: id number '$2' conflict with $3", c.get_name, id, @ordered_cell_list[ id - 1 ].get_name )
      end
      @ordered_cell_list[ id - 1 ] = c
      # 通し番号とする場合のため @id_base を加える
      c.set_id( @id_base - 1 + id )
      if $verbose then
        print( "#{c.get_name}: id=#{c.get_id}  specified id=#{c.get_specified_id}\n" )
      end
    }

    # ID 指定されていないセルに id 番号を与える
    i = 0
    no_id_specified_cells.each{ |c|
      while( @ordered_cell_list[i] != nil )
        i += 1
      end
      @ordered_cell_list[ i ] = c
      c.set_id( @id_base + i )
      if $verbose then
        print( "#{c.get_name}: id=#{c.get_id}\n" )
      end
    }
    if @n_cell_gen >0 && i >= @n_cell_gen then
      raise "id over id=#{i} N=#{@n_cell_gen}"
    end
  end

  def set_domain
    @cell_list.each{ |c|
      if c.is_generate? then
        dr = c.get_region.get_domain_root
        if dr.get_domain_type then
          dn = dr.get_domain_type.get_name
        else
          dn = nil
        end
        if @domain_roots[ dn ] then
          @domain_roots[ dn ] << dr
        else
          @domain_roots[ dn ] = [ dr ]
        end
      end
    }

    @domain_roots.each{ |dn, drs|
      drs.uniq!
      if ! $debug then
        dbgPrint "domains celltype:#{@name} domain=#{dn} "
        drs.each{ |r|
          dbgPrint " region=#{r.get_name}"
        }
        dbgPrint "\n"
      end
    }
    if @domain_roots.length > 1 then
      p @domain_roots
      raise "ambigous DomainType"
    end

    @domain_roots.each{ |dn, regions|
      # domain_type は一つのノードに一つしかないので、一つの要素を無条件で取り出す
      if regions.length > 1 then
        cdl_info( "I9999 celltype:#{@name} has cells in multi domain.\n" )
        if @idx_is_id == false then
          cdl_info( "I9999 celltype:#{@name} forcely set idx_is_id\n" )
        end
        @idx_is_id_act = true
      end
    }
  end

  def optimize

    # port の参照するセルタイプの数、セルの数を求める
    if $verbose then
      print "=== optimizing celltype #{get_namespace_path.to_s} ===\n"
    end

    @port.each{ |port|
      next if port.get_port_type != :CALL
      if port.is_omit? then
        # 呼び口最適化実施
        @b_cp_optimized = true
        @n_call_port_omitted_in_CB += 1               # CB で省略する呼び口
        port.set_skelton_useless                      # スケルトン関数不要最適化
        port.set_VMT_useless                          # VMT 不要最適化 (直接受け口関数を呼出す)
        if $verbose then
          print "optimized by omit: port: #{port.get_name} : o\n"
        end
        next
      elsif port.is_dynamic? then
        if $verbose then
          print "unoptimized by dynamic: port: #{port.get_name}\n"
        end
        next
      elsif port.is_ref_desc? then
        if $verbose then
          print "unoptimized by ref_desc: port: #{port.get_name}\n"
        end
        next
      end

      if $verbose then
        print "optimizing port : #{port.get_name}\n"
      end

      port_cells = []    # 呼び先セル
      port_ports = []    # 呼び先のポート

      # セルの参照するセルを集める（ポートも一緒に集める）
      @cell_list.each{ |cell|

        if ! cell.is_generate? then
          next
        end

        jl = cell.get_join_list
        j = jl.get_item( port.get_name )

        if j then
          if j.get_array_member2 then
            # 呼び口配列の場合、全部の結合先を集める
            j.get_array_member2.each { |j2|
              if j2 then
                port_cells << j2.get_rhs_cell
                port_ports << j2.get_rhs_port   # 右辺のポート
              else
                # optional で、ある添数のみ初期化されていない（すべて初期化されない場合は、下）
                port_cells << nil
                port_ports << nil
              end
            }
          else
            # 全ての結合先を集める
            port_cells << j.get_rhs_cell
            port_ports << j.get_rhs_port   # 右辺のポート
          end
        else
          # optional で初期化されていない（nil を要素に加えておく）
          port_cells << nil
          port_ports << nil   # 右辺のポート
        end
      }

      # 重複要素を取り除く
      port_cells.uniq!
      port_ports.uniq!

      # 呼び口の呼び先が一つのポートだけか？
      if port_ports.length == 1 then

        # 呼び口配列が可変長の場合、最適化しない
             # mikan 呼び口配列要素数マクロ不具合暫定対策
             # より望ましい修正は、受け口へのポインタは省略するが、配列個数は出力する(#_CP_#, #_TCP_#)
             # さらに配列個数が定数化できるのであれば、定数マクロを出力 (#_NCPA_#)
        next if port.get_array_size == "[]"

        # 呼び口最適化実施
        @b_cp_optimized = true

        # 呼び先が一つのセルだけか？
        if port_cells.length == 1 then

          # 呼び口は optional で初期化されていない、または受け口は配列ではないか？
          if port_ports[0] == nil || port_ports[0].get_array_size == nil then

            @n_call_port_omitted_in_CB += 1               # CB で省略する呼び口
            port.set_cell_unique                          # セル一つだけ最適化
            port.set_skelton_useless                      # スケルトン関数不要最適化
            port.set_VMT_useless                          # VMT 不要最適化 (直接受け口関数を呼出す)

            if $verbose then
              print "cell_unique, VMT_useless & skelton_useless optimize\n"
            end
          else
            port.set_VMT_useless                          # VMT 不要最適化 (スケルトン関数を呼出す)

            if $verbose then
              print "VMT_useless optimize\n"
            end
          end

        else  # 呼び先が複数のセル（単一のポート）

          # 呼び口は optional で初期化されていない、または受け口は配列ではないか？
          if port_ports[0] == nil || port_ports[0].get_array_size == nil then
            if ! @singleton then
              port.set_skelton_useless                    # スケルトン関数不要最適化
              port.set_VMT_useless                        # VMT 不要最適化 (スケルトン関数 or 受け口関数を呼出す)

              if $verbose then
                print "VMT_useless & skelton useless optimize\n"
              end
            else
              port.set_VMT_useless                           # VMT 不要最適化 (スケルトン関数 or 受け口関数を呼出す)

              if $verbose then
                print "VMT_useless optimize\n"
              end
            end
          end
        end

        port.set_only_callee( port_ports[0], port_cells[0] )
           # set_cell_unique でない場合 cell は意味がない

      end

      # debug
      dbgPrint "#{port.get_name} : # of cells : #{port_cells.length}  # of ports : #{port_ports.length}\n"
    }

    # 受け口最適化の設定
    @port.each{ |port|
      next if port.get_port_type != :CALL

      # 呼び口側の最適化状態
      b_VMT_useless     = port.is_VMT_useless?
      b_skelton_useless = port.is_skelton_useless?

      # セルの参照するセルを集める（ポートも一緒に集める）
      @cell_list.each{ |cell|

        if ! cell.is_generate? then
          next
        end

        jl = cell.get_join_list
        j = jl.get_item( port.get_name )

        if j then    # optional で結合されていない場合 nil
          if j.get_array_member2 then
            # 呼び口配列
            j.get_array_member2.each { |j2|
              if j2 then
                port2 = j2.get_rhs_port   # 右辺のポート
                # 受け口側の最適化可能性を設定
                port2.set_entry_VMT_skelton_useless( b_VMT_useless, b_skelton_useless )
              #else
              #  optional で呼び口配列要素が初期化されていない
              end
            }
          else
            port2 = j.get_rhs_port      # 右辺のポート
            # 受け口側の最適化可能性を設定
            port2.set_entry_VMT_skelton_useless( b_VMT_useless, b_skelton_useless )
          end
        end
      }
    }
  end

  #Celltype# リセットする
  def reset_optimize
    @@ID_BASE = ID_BASE      # 本当は一回だけでよい
    @id_base = 1             # set_cell_id でリセットされるので不要

    @b_cp_optimized = false  # 呼び口最適化
    @n_call_port_omitted_in_CB = 0 # 呼び口最適化により不生成となったポートの数
    @n_cell_gen = 0          # 生成セル個数
    @port.each{ |p|
      p.reset_optimize
    }
    @included_header = {}
    @domain_roots = {}
  end

  #Celltype# ヘッダは include されているか
  #hname::Symbol : ヘッダ名
  #RETURN:: bool_t: false インクルードされていない、true インクルードされている
  # #_ISH_#, #_ICT_# でヘッダが取り込まれているかチェックする
  # false が返った場合、hname は登録されて、次回の呼び出しでは true が返る
  def header_included?( hname )
    if @included_header[ hname ] == nil then
      @included_header[ hname ] = true
      return false
    else
      return true
    end
  end
end


