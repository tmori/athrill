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
#   $Id: GenTransparentMarshaler.rb 2418 2016-01-04 12:17:36Z okuma-top $
#++

#プラグインオプション用変数
#@task_priority:: Integer
#@channelCelltype:: String
#@channelCellName:: String
#@PPAllocatorSize:: Integer
module GenTransparentMarshaler

  # プラグイン引数名と Proc
  RPCPluginArgProc = {
    "taskPriority"    => Proc.new { |obj,rhs| obj.set_taskPriority rhs },
    "channelCelltype" => Proc.new { |obj,rhs| obj.set_channelCelltype rhs },
    "TDRCelltype"     => Proc.new { |obj,rhs| obj.set_TDRCelltype rhs },
    "channelCell"     => Proc.new { |obj,rhs| obj.set_channelCellName rhs },
    "PPAllocatorSize" => Proc.new { |obj,rhs| obj.set_PPAllocatorSize rhs },
  }

  #=== プラグイン引数 taskPriority のチェック
  def set_taskPriority( rhs )
    @task_priority = rhs
  end

  #=== プラグイン引数 channelCelltype のチェック
  def set_channelCelltype( rhs )
    @channelCelltype = rhs.to_sym
    # path = [ "::", @channelCelltype ]
    # obj = Namespace.find( path )
    nsp = NamespacePath.analyze( @channelCelltype.to_s )
    obj = Namespace.find( nsp )
    if ! obj.instance_of?( Celltype ) && ! obj.instance_of?( CompositeCelltype ) then
      cdl_error( "RPCPlugin: channeclCelltype '#{rhs}' not celltype or not found" )
    end
  end

  #=== プラグイン引数 TDRCelltype のチェック
  def set_TDRCelltype( rhs )
    @TDRCelltype = rhs.to_sym
    # path = [ "::", @TDRCelltype ]
    # obj = Namespace.find( path )
    nsp = NamespacePath.analyze( @TDRCelltype.to_s )
    obj = Namespace.find( nsp )
    if ! obj.instance_of?( Celltype ) && ! obj.instance_of?( CompositeCelltype ) then
      cdl_error( "RPCPlugin: TDRCelltype '#{rhs}' not celltype or not found" )
    end
  end

  #=== プラグイン引数 channelCellName のチェック
  def set_channelCellName( rhs )
    @channelCellName = rhs
    if @channelCellName =~ /\A[a-zA-Z_]\w*/ then
      # OK
    else
      cdl_error( "RPCPlugin: channeclCellName '#{rhs}' unsuitable for identifier" )
    end
  end

  #=== プラグイン引数 PPAllocatorSize のチェック
  def set_PPAllocatorSize( rhs )
    @PPAllocatorSize = rhs
  end

  #=== marshaler のセルタイプ名を設定する
  def initialize_transparent_marshaler cell_name
    @task_priority = 8
    @channelCelltype = "tDataqueueOWChannel"
    @TDRCelltype     = "tTDR"
    @channelCellName = "#{cell_name}_Channel"
    @PPAllocatorSize = nil

    @marshaler_celltype_name = "tMarshaler_#{@signature.get_global_name}"
    @unmarshaler_celltype_name = "tUnmarshaler_#{@signature.get_global_name}"
    @marshaler_celltype_file_name = "#{$gen}/#{@marshaler_celltype_name}.cdl"
  end

  def gen_marshaler_celltype 

    if @PPAllocatorSize then
      alloc_call_port = "  call sPPAllocator cPPAllocator;\n"
    else
      alloc_call_port = ""
    end

    f = CFile.open( @marshaler_celltype_file_name, "w" )
    # 同じ内容を二度書く可能性あり (AppFile は不可)

    f.print <<EOT

celltype #{@marshaler_celltype_name} {
  entry #{@signature.get_namespace_path} eClientEntry;
  call sTDR        cTDR;
  call sEventflag  cEventflag;
  [optional]
    call sSemaphore cLockChannel;  // this port is eliminated by optimize
};
celltype #{@unmarshaler_celltype_name} {
  call #{@signature.get_namespace_path} cServerCall;
  call sTDR        cTDR;
  call sEventflag  cEventflag;
  entry sUnmarshalerMain  eUnmarshalAndCallFunction;
#{alloc_call_port}};
EOT
    f.close
  end

  #===  受け口関数の本体コードを生成（頭部と末尾は別途出力）
  #ct_name:: Symbol    (through プラグインで生成された) セルタイプ名 ．Symbol として送られてくる（らしい）
  def gen_ep_func_body( file, b_singleton, ct_name, global_ct_name, sig_name, ep_name, func_name, func_global_name, func_type, params )

    # unmarshaler クラスか?
    if ct_name == @unmarshaler_celltype_name.to_sym then
      gen_ep_func_body_unmarshal( file, b_singleton, ct_name, global_ct_name, sig_name, ep_name, func_name, func_global_name, func_type, params )
    else
      gen_ep_func_body_marshal( file, b_singleton, ct_name, global_ct_name, sig_name, ep_name, func_name, func_global_name, func_type, params )
    end
  end

  #===  marshal コードの生成
  def gen_ep_func_body_marshal( file, b_singleton, ct_name, global_ct_name, sig_name, ep_name, func_name, func_global_name, func_type, params )

    b_void = false
    b_ret_er = false

    # 関数の戻り値の元の型を得る(typedef されている場合)
    type = func_type.get_type.get_original_type

    # 戻り値記憶用の変数を出力（void 型の関数では出力しない）
    if ! type.is_void? then
      if func_type.get_type.kind_of?( DefinedType ) && ( func_type.get_type.get_type_str == "ER" || func_type.get_type.get_type_str == "ER_INT" ) then
        file.print( "    #{func_type.get_type.get_type_str}  retval_ = E_OK;\n" )
        b_ret_er = true
      else
        file.print( "    #{func_type.get_type.get_type_str}  retval_;\n" )
      end
    else
      b_void = true
    end

    file.print( "    ER      ercd_;\n" )
    file.print( "    FLGPTN  flgptn;\n" )

    # 呼び先の signature を取り出す
    signature = @signature

    # 関数 ID （整数値）
    func_id = signature.get_id_from_func_name( func_name )
    file.print( "    int16_t  func_id_ = #{func_id};    /* id of #{func_name}: #{func_id} */\n" )

    # シングルトンでないか？
    if ! b_singleton then

      # singleton でなければ p_cellcb 取得コードを出力
      file.print <<EOT
    #{ct_name}_CB *p_cellcb;

    if( VALID_IDX( idx ) ){
        p_cellcb = GET_CELLCB(idx);
EOT

      # エラーを返すか？
      if b_ret_er then
        file.print <<EOT
    }else{
         return ERCD( E_RPC, E_ID );
    }
EOT
      else
        file.print <<EOT
    }else{
        /* エラー処理コードをここに記述 */
    }
EOT
      end
    end

    # channel lock コード
    file.print <<EOT
    /* Channel Lock */
    if( is_cLockChannel_joined() )
      cLockChannel_wait();

EOT

    # SOP を送信
    file.print "    /* SOPの送出 */\n"
    file.print "    if( ( ercd_ = cTDR_sendSOP( true ) ) != E_OK )\n"
    file.print "      goto error_reset;\n"

    # func_id を送信
    file.print "    /* 関数 id の送出 */\n"
    file.print "    if( ( ercd_ = cTDR_putInt16( func_id_ ) ) != E_OK )\n"
    file.print "        goto error_reset;\n"

    # p "celltype_name, sig_name, func_name, func_global_name"
    # p "#{ct_name}, #{sig_name}, #{func_name}, #{func_global_name}"

    b_get = false    # marshal なら put
    b_marshal = true  # marshal

    # in 方向の入出力を出力
    file.print "    /* 入力引数送出 */\n"
    print_params( params, file, 1, b_marshal, b_get, true, func_type.is_oneway? )
    print_params( params, file, 1, b_marshal, b_get, false, func_type.is_oneway? )
    if ! b_void && ! func_type.is_oneway? then
      ret_ptr_type = PtrType.new( func_type.get_type )
      print_param_nc( "retval_", ret_ptr_type, file, 1, :RETURN, "&", nil, b_get )
    end

    file.print "    /* EOPの送出（パケットの掃きだし） */\n"
    if ! func_type.is_oneway? then
      b_continue = "true"
    else
      b_continue = "false"
    end
    file.print "    if( (ercd_=cTDR_sendEOP(#{b_continue})) != E_OK )\n"
    file.print "        goto error_reset;\n\n"

    if ! func_type.is_oneway? then
      file.print <<EOT
    if( (ercd_=cEventflag_wait( 0x01, TWF_ANDW, &flgptn )) != E_OK ){
      ercd_ = ERCD(E_RPC,ercd_);
      goto error_reset;
    }
    if( (ercd_=cEventflag_clear( 0x00 ) ) != E_OK ){
      ercd_ = ERCD(E_RPC,ercd_);
      goto error_reset;
    }
EOT
    end # ! func_type.is_oneway?

    file.print <<EOT
    /* Channel Lock */
    if( is_cLockChannel_joined() )
      cLockChannel_signal();
EOT

    if( b_void == false )then
      # 呼び元に戻り値をリターン
      file.print( "    return retval_;\n" )
    else
      file.print( "    return;\n" )
    end

    file.print <<EOT

error_reset:
    if( ercd_ != ERCD( E_RPC, E_RESET ) )
        (void)cTDR_reset();
EOT

    # channel lock コード
    file.print <<EOT
    /* Channel Lock */
    if( is_cLockChannel_joined() )
      cLockChannel_signal();

EOT

    if( b_ret_er != false )then
      # 呼び元に戻り値をリターン
      file.print( "    return ercd_;\n" )
    else
      file.print( "    return;\n" )
    end

  end

  #===  unmarshal コードの生成
  def gen_ep_func_body_unmarshal( file, b_singleton, ct_name, global_ct_name, sig_name, ep_name, func_name, func_global_name, func_type, params )

#    b_ret_er = true
    b_ret_er = false

    # func_id を得るコードを生成
    file.print <<EOT

    int16_t   func_id_;
    ER        ercd_;

    #{ct_name}_CB *p_cellcb;

    if( VALID_IDX( idx ) ){
        p_cellcb = GET_CELLCB(idx);
EOT

    if b_ret_er then
        file.print <<EOT
    }else{
        return ERCD( E_RPC, E_ID );
    }
EOT
    else
        file.print <<EOT
    }else{
        /* エラー処理コードをここに記述 */
    }
EOT
    end

    file.print <<EOT

    /* SOPのチェック */
    if( (ercd_=cTDR_receiveSOP( false )) != E_OK )
        goto error_reset;
    /* func_id の取得 */
    if( (ercd_=cTDR_getInt16( &func_id_ )) != E_OK )
        goto error_reset;

#ifdef RPC_DEBUG
    syslog(LOG_INFO, "unmarshaler task: func_id: %d", func_id_ );
#endif
    switch( func_id_ ){
EOT

    # 呼び先の signature を取り出す
    # port = @celltype.find( @next_cell_port_name )
    # signature = port.get_signature
    signature = @signature

    # through の signature に含まれる すべての関数について
    signature.get_function_head_array.each { |f|
      f_name = f.get_name
      f_type = f.get_declarator.get_type
      id = signature.get_id_from_func_name( f_name )

      # 関数は返り値を持つか?
      if f_type.get_type.is_void? then
        b_void = true
      else
        b_void = false
      end

      # パケットの終わりをチェック（未受け取りのデータが残っていないかチェック）
      file.print "    case #{id}:       /*** #{f_name} ***/ \n"
      file.print "        if( tTransparentUnmarshaler_#{@signature.get_global_name}_#{f_name}() != E_OK )\n"
      file.print "            goto error_reset;\n"
      file.print "        break;\n"

    } # 

    if @PPAllocatorSize then
      ppallocator_dealloc_str = "    /* PPAllocator のすべてを解放 */\n    cPPAllocator_dealloc_all();"
    else
      ppallocator_dealloc_str = ""
    end


    file.print <<EOT
    default:
        syslog(LOG_INFO, "unmarshaler task: ERROR: unknown func_id: %d", func_id_ );
    };
#{ppallocator_dealloc_str}
    return;

error_reset:
    if( ercd_ != ERCD( E_RPC, E_RESET ) )
        (void)cTDR_reset();
#{ppallocator_dealloc_str}
EOT

  end

  # IN b_marshal, b_get
  #  b_marshal = true  && b_get == false   :  マーシャラで入力引数送出
  #  b_marshal = true  && b_get == true    :  マーシャラで出力引数受取
  #  b_marshal = false && b_get == true    :  アンマーシャラで入力引数受取
  #  b_marshal = false && b_get == get     :  アンマーシャラで出力引数送出
  def print_params( params, file, nest, b_marshal, b_get, b_referenced, b_oneway = false )
    params.each{ |param|
# p "#{param.get_name}:  b_marshal: #{b_marshal} b_get: #{b_get}"
      if ! ( b_referenced == param.is_referenced? ) then
        next
      end

      dir = param.get_direction
      type = param.get_type
      if b_oneway && dir == :IN && type.get_original_type.kind_of?( PtrType ) || type.get_original_type.kind_of?( ArrayType ) then
        # oneway, in, PtrType の場合コピー
        alloc_cp = "cPPAllocator_alloc"
        alloc_cp_extra = nil
        print_param( param.get_name, type, file, nest, dir, nil, nil, b_marshal, b_get, alloc_cp, alloc_cp_extra )
      else
        if( b_get == false && b_marshal == true || b_get == true && b_marshal == false  )then
          case dir
#          when :IN, :INOUT, :SEND
          when :IN, :INOUT, :OUT, :SEND, :RECEIVE
            print_param_nc( param.get_name, type, file, nest, b_marshal, nil, nil, b_get )
          end
        else
#         case dir
#         when :OUT, :INOUT, :RECEIVE
#         when :RECEIVE
#           print_param_nc( param.get_name, type, file, nest, b_marshal, nil, nil, b_get )
#         end
        end
      end
    }
  end

  #=== コピーしない引数渡しコードの出力
  def print_param_nc( name, type, file, nest, b_marshal, outer, outer2, b_get )
    indent = "    " * ( nest + 1 )

    case type
    when DefinedType
      print_param_nc( name, type.get_type, file, nest, b_marshal, outer, outer2, b_get )
    when BoolType, IntType, FloatType, PtrType, ArrayType
      case type
      when BoolType
        type_str = "Int8"
        cast_str = "int8_t"
      when IntType
        bit_size = type.get_bit_size
        case type.get_sign
        when :UNSIGNED
          signC = "U"
          sign  = "u"
        when :SIGNED
          if bit_size == -1 || bit_size == -11 then
            # signed char の場合、signed を指定する
            signC = "S"
            sign  = "s"
          else
            signC = ""
            sign  = ""
          end
        else
          signC = ""
          sign  = ""
        end

        # p "pn:: #{name} #{bit_size} #{type.get_type_str}"
        case bit_size
        when  -1, -11  # -1: char_t, -11: char
          type_str = "#{signC}Char"
          cast_str = "#{sign}char_t"
        when -2
          type_str = "#{signC}Short"
          cast_str = "#{sign}short_t"
        when -3
          type_str = "#{signC}Int"
          cast_str = "#{sign}int_t"
        when -4
          type_str = "#{signC}Long"
          cast_str = "#{sign}long_t"
        when -5
          type_str = "Intptr"
          cast_str = "intptr_t"
        when 8, 16, 32, 64, 128
          type_str = "#{signC}Int#{bit_size}"
          cast_str = "#{sign}int#{bit_size}_t"
        else
          raise "unknown bit_size '#{bit_size}' for int type "
        end

      when FloatType
        bit_size = type.get_bit_size
        if bit_size == 32 then
          type_str = "Float32"
          cast_str = "float32_t"
        else
          type_str = "Double64"
          cast_str = "double64_t"
        end

      when PtrType
        type_str = "Intptr"
        cast_str = "intptr_t"
      when ArrayType
        type_str = "Intptr"
        cast_str = "intptr_t"
      end

      if type.get_type_str == cast_str then
        cast_str = ""
      else
        cast_str = "(" + cast_str + ")"
      end

      if( b_get )then
        cast_str.gsub!( /\)$/, "*)" )
        file.print "    " * nest
        file.print "if( ( ercd_ = cTDR_get#{type_str}( #{cast_str}&(#{outer}#{name}#{outer2}) ) ) != E_OK )\n"
        file.print "    " * nest
        file.print "    goto error_reset;\n"
      else
        file.print "    " * nest
        file.print "if( ( ercd_ = cTDR_put#{type_str}( #{cast_str}#{outer}#{name}#{outer2} ) ) != E_OK )\n"
        file.print "    " * nest
        file.print "    goto error_reset;\n"
      end

    when StructType
      members_decl =type.get_members_decl
      members_decl.get_items.each { |m|
        if m.is_referenced? then
          print_param_nc( m.get_name, m.get_type, file, nest, b_marshal, "#{outer}#{name}#{outer2}.", nil, b_get )
        end
      }
      members_decl.get_items.each { |m|
        if ! m.is_referenced? then
          print_param_nc( m.get_name, m.get_type, file, nest, b_marshal, "#{outer}#{name}#{outer2}.", nil, b_get )
        end
      }

    when VoidType
    when EnumType  # mikan EnumType
    when FuncType  # mikan FuncType
    end
  end


  #=== PREAMBLE 部のコード生成
  # アンマーシャラセルタイプの場合、アンマーシャラ関数のプロトタイプ宣言を生成
  def gen_preamble file, b_singleton, ct_name, global_name
    if ct_name != @unmarshaler_celltype_name.to_sym then
      return
    end

    file.print "/* アンマーシャラ関数のプロトタイプ宣言 */\n"
    # signature に含まれる すべての関数について
    @signature.get_function_head_array.each { |f|
      f_name = f.get_name
      f_type = f.get_declarator.get_type
      id = @signature.get_id_from_func_name( f_name )
      file.print "static ER  tTransparentUnmarshaler_#{@signature.get_global_name}_#{f_name}();\t/* func_id: #{id} */\n"
    }
    file.print "\n"
  end

  #=== POSTAMBLE 部のコード生成
  # アンマーシャラセルタイプの場合、アンマーシャラ関数の生成
  def gen_postamble file, b_singleton, ct_name, global_name
    if ct_name != @unmarshaler_celltype_name.to_sym then
      return
    end

    file.print "\n/*** アンマーシャラ関数 ***/\n\n"
    @signature.get_function_head_array.each { |f|
      f_name = f.get_name
      f_type = f.get_declarator.get_type
      id = @signature.get_id_from_func_name( f_name )

      # 関数は返り値を持つか?
      if f_type.get_type.is_void? then
        b_void = true
      else
        b_void = false
      end

      file.print <<EOT
/*
 * name:    #{f_name}
 * func_id: #{id} 
 */
EOT
      file.print "static ER\n"
      file.print "tTransparentUnmarshaler_#{@signature.get_global_name}_#{f_name}()\n"
      file.print "{\n"
      file.print "	ER  ercd_;\n"
      file.print "	CELLCB  *p_cellcb;\n"

      # 引数を受取る変数の定義
      param_list = f.get_declarator.get_type.get_paramlist.get_items
           # FuncHead->  Decl->    FuncType->ParamList
      param_list.each{ |par|
        name = par.get_name
        type = par.get_type
        if type.kind_of? ArrayType then
          type = type.get_type
          aster = "(*"
          aster2 = ")"
        else
          aster = ""
          aster2 = ""
        end

        type_str = type.get_type_str.gsub( /\bconst\b */, "" ) # "const" を外す

        file.printf( "    %-12s %s%s%s%s;\n", type_str, aster, name, aster2, type.get_type_str_post )
      }

      # 戻り値を受け取る変数の定義
      if ! b_void then
        if f.is_oneway? then
          retval_ptr = ""   # oneway の場合、受け取るが捨てられる
        else
          retval_ptr = "*"
        end
        file.printf( "    %-12s #{retval_ptr}retval_%s;\n", f_type.get_type.get_type_str, f_type.get_type.get_type_str_post )
      end

      # in 方向の入出力を入力
      file.print "\n        /* 入力引数受取 */\n"
      b_get = true    # unmarshal では get
      b_marshal  = false
      print_params( param_list, file, 1, b_marshal, b_get, true, f.is_oneway? )
      print_params( param_list, file, 1, b_marshal, b_get, false, f.is_oneway? )
      if ! b_void && ! f.is_oneway? then
        ret_ptr_type = PtrType.new( f_type.get_type )
        print_param_nc( "retval_", ret_ptr_type, file, 2, :RETURN, nil, nil, b_get )
      end

      # パケットの受信完了
      # mikan 本当は、対象関数を呼出す後に実施したい．呼出しパケットの使用終わりを宣言する目的として
      file.print "        /* パケット終わりをチェック */\n"
      if ! f.is_oneway? then
        b_continue = "true"
      else
        b_continue = "false"
      end
      file.print "    if( (ercd_=cTDR_receiveEOP(#{b_continue})) != E_OK )\n"
      file.print "        goto error_reset;\n\n"

      # 対象関数を呼出す
      file.print "    /* 対象関数の呼出し */\n"
      if b_void then
        file.print( "    cServerCall_#{f_name}(" )
      else
        file.print( "    #{retval_ptr}retval_ = cServerCall_#{f_name}(" )
      end

      delim = " "
      param_list.each{ |par|
        file.print delim
        delim = ", "
        file.print "#{par.get_name}"
      }
      file.print( " );\n" )

      # 戻り値、出力引数の受取コードの生成

      # oneway の場合出力、戻り値が無く、受取を待たない（非同期な呼出し）
      if ! f.is_oneway? then
        file.print <<EOT
    /* 関数処理の終了を通知 */
    if( ( ercd_ = cEventflag_set( 0x01 ) ) != E_OK ){
      goto error_reset;
    }
EOT
      end  # ! f.is_oneway?
      file.print <<EOT
    return E_OK;
error_reset:
    return ercd_;
}

EOT

    }

  end

end
