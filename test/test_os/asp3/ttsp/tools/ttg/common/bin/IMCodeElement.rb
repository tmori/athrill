#!ruby -Ke
#
#  TTG
#      TOPPERS Test Generator
#
#  Copyright (C) 2009-2012 by Center for Embedded Computing Systems
#              Graduate School of Information Science, Nagoya Univ., JAPAN
#  Copyright (C) 2010-2011 by Graduate School of Information Science,
#                             Aichi Prefectural Univ., JAPAN
#
#  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
#  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
#  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
#  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
#      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
#      スコード中に含まれていること．
#  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
#      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
#      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
#      の無保証規定を掲載すること．
#  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
#      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
#      と．
#    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
#        作権表示，この利用条件および下記の無保証規定を掲載すること．
#    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
#        報告すること．
#  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
#      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
#      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
#      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
#      免責すること．
#
#  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
#  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
#  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
#  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
#  の責任を負わない．
#
#  $Id: IMCodeElement.rb 6 2012-09-03 11:06:01Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "common/bin/Config.rb"

module CommonModule

  #==================================================================
  # クラス名: IMCodeElement
  # 概　  要: IntermediateCodeが解釈できる形式で中間コードの要素を格納
  #           するクラス
  #==================================================================
  class IMCodeElement
    include CommonModule

    #================================================================
    # 概　要: コンストラクタ．
    #         指定したタイプのエレメントを生成する
    #================================================================
    def initialize(lMode = :block)
      check_class(Symbol, lMode) # 生成したコードをBlock化するか共通部に挿入するか（デフォルト :block）

      @aElement = []           # 要素を保持
      @cConf    = Config.new() # コンフィグを取得

      # モードの設定
      if (lMode == :block || :common)
        @lMode = lMode
      else
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

    end
    attr_reader :aElement, :lMode


    #================================================================
    # 概　要: API呼び出しと戻り値のチェックを行うコードの要素を保持する
    #================================================================
    def set_syscall(hProcUnitInfo, sSyscall, snbReturn = "E_OK", sVarType = TYP_ER)
      check_class(Hash, hProcUnitInfo)                       # コードを追加する処理単位情報
      check_class(String, sSyscall)                          # 追加するAPI
      check_class([String, Integer, *Bool], snbReturn, true) # 想定される戻り値（デフォルト E_OK）
      check_class(String, sVarType)                          # 想定される戻り値の型（デフォルト TYP_ER）

      # 変数名を取得
      sVarName = GRP_VAR_TYPE[sVarType]

      # 戻り値のチェックを行う場合
      if (!snbReturn.nil?())
        @aElement.push({:atr => :api, :proc_unit => hProcUnitInfo, :syscall => sSyscall, :var => sVarName})
        set_local_var(hProcUnitInfo[:id], sVarName, sVarType)

        # 戻り値の型がTYP_ERかTYP_ER_UINTの場合はcheck_ercd
        if (sVarType != TYP_BOOL_T)
          @aElement.push({:atr => :ercd, :proc_unit => hProcUnitInfo, :ret => snbReturn, :var => sVarName})
        # 戻り値の型がTYP_BOOL_Tの場合はassert
        else
          set_assert(hProcUnitInfo, sVarName, snbReturn)
        end

      # 戻り値のチェックを行わない場合
      else
        # 呼び出し側で個別にチェックしているところがあるため
        set_local_var(hProcUnitInfo[:id], sVarName, sVarType)
        @aElement.push({:atr => :api, :proc_unit => hProcUnitInfo, :syscall => sSyscall, :var => sVarName})
      end
    end


    #================================================================
    # 概　要: assertチェックを行うコードの要素を保持する
    #================================================================
    def set_assert(hProcUnitInfo, sVar, snbValue)
      check_class(Hash, hProcUnitInfo)                # コードを追加する処理単位情報
      check_class(String, sVar)                       # assertによって確認する変数
      check_class([String, Integer, *Bool], snbValue) # assertによって確認する値

      @aElement.push({:atr => :assert, :proc_unit => hProcUnitInfo, :var => sVar, :value => snbValue})
    end


    #================================================================
    # 概　要: 引数で与えた文字列をそのまま出力する要素を保持する．
    #         関数呼び出し，時間関連，割込み発生，CPU例外発生，syslog
    #         などのコードを追加したい場合での利用を想定
    #================================================================
    def set_code(hProcUnitInfo, sEvalCode, bExpression = true)
      check_class(Hash, hProcUnitInfo) # コードを追加する処理単位情報
      check_class(String, sEvalCode)   # 出力したいコード
      check_class(Bool, bExpression)   # コードが式であるかどうか（デフォルト：true）

      if (bExpression == true)
        # 式の場合は出力文字列にセミコロンを付与する
        sEvalCodeSemicolon = sEvalCode + ";"
        @aElement.push({:atr => :expression, :proc_unit => hProcUnitInfo, :code => sEvalCodeSemicolon})
      else
        # 文の場合はそのまま格納する
        @aElement.push({:atr => :statement,  :proc_unit => hProcUnitInfo, :code => sEvalCode})
      end
    end


    #================================================================
    # 概　要: チェックポイントを挿入するコードの要素を保持する
    #================================================================
    def set_checkpoint(hProcUnitInfo)
      check_class(Hash, hProcUnitInfo) # チェックポイントを追加する処理単位情報

      if (@cConf.is_asp?())
        @aElement.push({:atr => :check,    :proc_unit => hProcUnitInfo})
      elsif (@cConf.is_fmp?())
        @aElement.push({:atr => :check_mp, :proc_unit => hProcUnitInfo})
      else
        abort(ERR_MSG % [__FILE__, __LINE__])
      end
    end


    #================================================================
    # 概　要: ゼロチェックポイントを挿入するコードの要素を保持する
    #================================================================
    def set_checkpoint_zero(hProcUnitInfo)
      check_class(Hash, hProcUnitInfo) # チェックポイントを追加する処理単位情報

      if (@cConf.is_asp?())
        @aElement.push({:atr => :check_zero,    :proc_unit => hProcUnitInfo})
      else
        abort(ERR_MSG % [__FILE__, __LINE__])
      end
    end


    #================================================================
    # 概　要: チェックポイントを待つコードの要素を保持する
    #================================================================
    def wait_checkpoint(hProcUnitInfo)
      check_class(Hash, hProcUnitInfo) # チェックポイントを待つ処理単位情報

      unless (@cConf.is_asp?())
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      @aElement.push({:atr => :wait, :proc_unit => hProcUnitInfo})
    end


    #================================================================
    # 概　要: 最終チェックポイントを挿入するコードの要素を保持する
    #================================================================
    def set_checkfinish(hProcUnitInfo)
      check_class(Hash, hProcUnitInfo) # チェックポイントを追加する処理単位情報

      if (@cConf.is_asp?())
        @aElement.push({:atr => :check_finish,    :proc_unit => hProcUnitInfo})
      elsif (@cConf.is_fmp?())
        @aElement.push({:atr => :check_finish_mp, :proc_unit => hProcUnitInfo})
      else
        abort(ERR_MSG % [__FILE__, __LINE__])
      end
    end


    #================================================================
    # 概　要: WaitCheckSyncを行うコードの要素を保持する
    #================================================================
    def set_wait_check_sync(hProcUnitInfo, nPrcid)
      check_class(Hash, hProcUnitInfo) # waitCPを追加する処理単位情報
      check_class(Integer, nPrcid)     # 待ち対象のプロセッサID

      unless (@cConf.is_fmp?())
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      @aElement.push({:atr => :waitcp, :proc_unit => hProcUnitInfo, :prcid => nPrcid})
    end


    #================================================================
    # 概　要: StateSyncを行うコードの要素を保持する
    #================================================================
    def set_state_sync(hProcUnitInfo, sTargetTask, sTargetState)
      check_class(Hash, hProcUnitInfo)    # waitCPを追加する処理単位情報
      check_class(String, sTargetTask)    # 待ち対象タスクID
      check_class(String, sTargetState)   # 待ち対象状態

      unless (@cConf.is_fmp?())
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      set_code(hProcUnitInfo, "#{FNC_STATE_SYNC}(\"#{hProcUnitInfo[:id]}\", \"#{sTargetTask}\", #{sTargetTask}, \"#{sTargetState}\", #{sTargetState})")
    end


    #================================================================
    # 概　要: WaitFinishSyncを行うコードの要素を保持する
    #================================================================
    def set_wait_finish_sync(hProcUnitInfo)
      check_class(Hash, hProcUnitInfo)  # WaitFinishSyncを追加する処理単位情報

      unless (@cConf.is_fmp?())
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      set_code(hProcUnitInfo, "#{FNC_WAIT_FINISH_SYNC}(\"#{hProcUnitInfo[:id]}\")")
    end


    #================================================================
    # 概　要: メインタスクの優先度がTTG_MAIN_PRIでない場合だけ，
    #         chg_priをするコードをcElementに設定する
    #================================================================
    def set_chg_pri_main_task(hProcUnitInfo)
      check_class(Hash, hProcUnitInfo)  # chg_pri_main_task処理を追加する処理単位情報の配列

      @aElement.push({:atr => :chg_pri_main_task})
      set_code(hProcUnitInfo, "#{FNC_CHG_PRI_MAIN_TASK}()")
    end


    #================================================================
    # 概　要: BarrierSyncを行うコードの要素を保持する
    #================================================================
    def set_barrier_sync(aProcUnitInfo)
      check_class(Array, aProcUnitInfo) # バリア同期を追加する処理単位情報の配列

      unless (@cConf.is_fmp?())
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      aProcUnitInfo.each{ |hProcUnitInfo|
        @aElement.push({:atr => :barrier, :proc_unit => hProcUnitInfo, :num => aProcUnitInfo.size})
      }
    end


    #================================================================
    # 概　要: dormantのタスクに対するマイグレート処理を行うコードの要
    #         素を保持する
    #================================================================
    def set_migrate_task(hMainTaskInfo, sTargetTask, nTargetPrcid)
      check_class(Hash, hMainTaskInfo)    # メインタスクの処理単位情報
      check_class(String, sTargetTask)    # 待ち対象タスクID
      check_class(Integer, nTargetPrcid)  # マイグレートさせたいプロセッサID

      unless (@cConf.is_fmp?())
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      @aElement.push({:atr => :migrate_func})
      set_code(hMainTaskInfo, "#{FNC_MIGRATE_TASK}(#{sTargetTask}, #{nTargetPrcid})")
    end


    #================================================================
    # 概　要: 遅延処理を行うコードの要素を保持する
    #================================================================
    def set_delay_loop(hProcUnitInfo)
      check_class(Hash, hProcUnitInfo)  # 遅延処理を行う処理単位情報

      set_comment(hProcUnitInfo, CMT_WAIT_SPIN_LOOP)
      set_local_var(hProcUnitInfo[:id], "i", VAR_VOLATILE_ULONG_T)
      set_code(hProcUnitInfo, "for (i = 0; i < #{@cConf.get_wait_spin_loop()}; i++)")
    end


    #================================================================
    # 概　要: 処理単位名と最終的な起動回数をを保持する
    #================================================================
    def set_proc_unit(sProcUnitID, nFinalBootCnt = 0)
      check_class(String, sProcUnitID)    # 追加する処理単位ID
      check_class(Integer, nFinalBootCnt) # 最終的な起動回数

      @aElement.push({:atr => :proc_unit, :proc_unit_id => sProcUnitID, :fbootcnt => nFinalBootCnt})
    end


    #================================================================
    # 概　要: ローカル変数宣言コードの要素を保持する
    #================================================================
    def set_local_var(sProcUnitID, sVarName, sVarType, snVarValue = nil)
      check_class(String, sProcUnitID)                 # コメントを追加する処理単位ID
      check_class(String, sVarName)                    # 変数名
      check_class(String, sVarType)                    # 変数の型
      check_class([String, Integer], snVarValue, true) # 変数の初期値

      @aElement.push({:atr => :localvar, :proc_unit_id => sProcUnitID, :name => sVarName, :type => sVarType, :value => snVarValue})
    end


    #================================================================
    # 概　要: グローバル変数宣言コードの要素を保持する
    #================================================================
    def set_global_var(sVarName, sVarType, snVarValue = nil)
      check_class(String, sVarName)                           # 変数名
      check_class(String, sVarType)                           # 変数の型
      check_class([String, Integer, *Bool], snVarValue, true) # 変数の初期値

      @aElement.push({:atr => :globalvar, :name => sVarName, :type => sVarType, :value => snVarValue})
    end


    #================================================================
    # 概　要: プロトタイプ宣言コードの要素を保持する
    #================================================================
    def set_header(sProcUnitID, sObjectType, aFuncArgs = nil)
      check_class(String, sProcUnitID)    # プロトタイプ宣言に追加する処理単位
      check_class(String, sObjectType)    # 処理単位のオブジェクトタイプ（関数の場合はIMC_FUNC_TYPE）
      check_class(Array, aFuncArgs, true) # 引数の配列（関数の場合のみ）

      @aElement.push({:atr => :header, :proc_unit_id => sProcUnitID, :type => sObjectType, :args => aFuncArgs})
    end


    #================================================================
    # 概　要: 静的APIの要素を保持する
    #================================================================
    def set_config(sCode, sClass = nil)
      check_class(String, sCode)        # 追加する静的API
      check_class(String, sClass, true) # 追加する対象クラス(デフォルト nil)

      @aElement.push({:atr => :config, :code => sCode, :class => sClass})
    end


    #================================================================
    # 概　要: コメントコードの要素を保持する
    #================================================================
    def set_comment(hProcUnitInfo, sComment, lAtr = :comment)
      check_class(Hash, hProcUnitInfo) # コメントを追加する処理単位情報
      check_class(String, sComment)    # 追加するコメント
      check_class(Symbol, lAtr)        # 付与する属性（デフォルト :comment）

      @aElement.push({:atr => lAtr, :proc_unit => hProcUnitInfo, :code => sComment})
    end


    #================================================================
    # 概　要: ブロックの境界を挿入する命令を保持する
    #================================================================
    def set_block_delimiter()
      if (lMode == :block)
        @aElement.push({:atr => :delimiter_cmd})
      end
    end


    #================================================================
    # 概　要: 以降の出力コードをインデントする命令を保持する
    #================================================================
    def set_indent(nTabs = 1)
      check_class(Integer, nTabs) # インデントの深さ（デフォルト 1）

      @aElement.push({:atr => :indent_cmd, :num => nTabs})
    end


    #================================================================
    # 概　要: インデント命令を解除する命令を保持する
    #================================================================
    def unset_indent()
      @aElement.push({:atr => :unindent_cmd})
    end


    #================================================================
    # 概　要: 関数定義の直後に出力するプレコードの要素を保持する
    #================================================================
    def set_pre_code(sProcUnitID, sPreCode, bExpression = true)
      check_class(String, sProcUnitID)  # コードを追加する処理単位ID
      check_class(String, sPreCode)     # 出力したいプレコード
      check_class(Bool, bExpression)    # コードが式であるかどうか（デフォルト：true）

      if (bExpression == true)
        # 式の場合は出力文字列にセミコロンを付与する
        sPreCodeSemicolon = sPreCode + ";"
        @aElement.push({:atr => :precode, :proc_unit => sProcUnitID, :code => sPreCodeSemicolon})
      else
        # 文の場合はそのまま格納する
        @aElement.push({:atr => :precode, :proc_unit => sProcUnitID, :code => sPreCode})
      end
    end


    #================================================================
    # 概　要: 関数定義の最後に出力するポストコードの要素を保持する
    #================================================================
    def set_post_code(sProcUnitID, sPostCode, bExpression = true)
      check_class(String, sProcUnitID)  # コードを追加する処理単位ID
      check_class(String, sPostCode)    # 出力したいポストコード
      check_class(Bool, bExpression)    # コードが式であるかどうか（デフォルト：true）

      if (bExpression == true)
        # 式の場合は出力文字列にセミコロンを付与する
        sPostCodeSemicolon = sPostCode + ";"
        @aElement.push({:atr => :postcode, :proc_unit => sProcUnitID, :code => sPostCodeSemicolon})
      else
        # 文の場合はそのまま格納する
        @aElement.push({:atr => :postcode, :proc_unit => sProcUnitID, :code => sPostCode})
      end
    end


    #================================================================
    # 概　要: IMCodeElementをppより見やすい形で整形して出力する
    #================================================================
    def p_code(lOption = :all)
      # [atr]         [proc]   [other parameters]
      # api           proc,    syscall,          var
      # ercd          proc,    ret,              var
      # assert        proc,                      var,     value
      # code          proc,    code
      # check(mp)     proc
      # zero          proc
      # finish(mp)    proc
      # waitcp        proc,    prcid
      # barrier       proc,    num
      # proc_unit     proc_id, fbootcnt
      # localvar      proc_id, name,    type,             value
      # globalvar              name,    type,             value
      # header        proc_id,          type,    var
      # config                 code,    class
      # comment       proc,    code
      # delimiter_cmd
      # precode       proc_id, code
      # postcode      proc_id, code
      puts "[atr]".center(15) + "|" + "[proc_unit]".center(14) + "|" + "[other_parameters]"
      puts "-" * 15 + "+" + "-" * 14 + "+" + "-" * 20
      @aElement.each{ |hElement|
        if (hElement[:atr] == :delimiter_cmd)
          puts ""; next
        end
        print "#{hElement[:atr]}".ljust(15) + "|"

        print "#{hElement[:proc_unit][:id].sub(/.*_(\w+\z)/, "\\1")}".ljust(8) + "|" if (hElement[:proc_unit])
        print "#{hElement[:proc_unit_id].sub(/.*_(\w+\z)/, "\\1")}".ljust(8)   + "|" if (hElement[:proc_unit_id])
        print "".ljust(8) + "|" unless (hElement[:proc_unit] || hElement[:proc_unit_id])
        print "#{hElement[:proc_unit][:bootcnt]}".ljust(2) + "|" if (hElement[:proc_unit])
        print "#{hElement[:proc_unit][:prcid]}".ljust(2)   + "|" if (hElement[:proc_unit])
        print "".ljust(2) + "|" + "".ljust(2) + "|" unless (hElement[:proc_unit])
        print "#{hElement[:syscall]}, "   if (hElement[:syscall])
        print "#{hElement[:ret]}, "       if (hElement[:ret])
        print "#{hElement[:code]}, "      if (hElement[:code])
        print "#{hElement[:prcid]}, "     if (hElement[:prcid])
        print "#{hElement[:num]}, "       if (hElement[:num])
        print "#{hElement[:fbootcnt]}, "  if (hElement[:fbootcnt])
        print "#{hElement[:name]}, "      if (hElement[:name])
        print "#{hElement[:type]}, "      if (hElement[:type])
        print "#{hElement[:class]}, "     if (hElement[:class])
        print "#{hElement[:var]}, "       if (hElement[:var])
        print "#{hElement[:value]}, "     if (hElement[:value])
        print "#{hElement[:code]}, "      if (hElement[:precode])
        print "#{hElement[:code]}, "      if (hElement[:postcode])
        puts ""
      }
    end

  end
end
