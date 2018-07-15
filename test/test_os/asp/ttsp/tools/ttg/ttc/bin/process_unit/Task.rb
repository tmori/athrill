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
#  $Id: Task.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "ttc/bin/process_unit/ProcessUnit.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: Task
  # 概    要: タスクの情報を処理するクラス
  #===================================================================
  class Task < ProcessUnit
    #=================================================================
    # 概  要: オブジェクトチェック
    #=================================================================
    def object_check(bIsPre)
      check_class(Bool, bIsPre)  # pre_conditionか

      aErrors = []
      begin
        super(bIsPre)
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      # 待ち状態の場合
      if (GRP_TASK_WAITING.include?(@hState[TSR_PRM_STATE]))
        ### T3_TSK001: 状態が待ち状態であるのに待ち対象が指定されていない
        if (@hState[TSR_PRM_WOBJID].nil?())
          aErrors.push(YamlError.new("T3_TSK001: " + ERR_NO_WOBJID_WAITING, @aPath))
        ### T3_TSK002: 状態が待ち状態で待ち対象がSLEEPの場合に起床要求キューイング数が指定されている
        elsif (@hState[TSR_PRM_WOBJID] == TSR_STT_SLEEP && !@hState[TSR_PRM_WUPCNT].nil?() && @hState[TSR_PRM_WUPCNT] > 0)
          aErrors.push(YamlError.new("T3_TSK002: " + ERR_SLEEPING_CANNOT_WUP, @aPath))
        ### T3_TSK003: pre_conditionでDELAY待ちの時に相対時間が指定されていない
        elsif (bIsPre == true && @hState[TSR_PRM_WOBJID] == TSR_STT_DELAY && @hState[TSR_PRM_LEFTTMO].nil?())
          aErrors.push(YamlError.new("T3_TSK003: " + ERR_NO_LEFTTMO_DELAY, @aPath))
        end
      ### T3_TSK004: 待ち状態ではないのに待ち対象が指定されている
      elsif (!@hState[TSR_PRM_WOBJID].nil?())
        aErrors.push(YamlError.new("T3_TSK004: " + ERR_SET_WOBJID_NO_WAITING, @aPath))
      end

      ### T3_TSK005: actcntが0であるのにactprcが設定されている
      if (@hState[TSR_PRM_ACTCNT] == 0 && !@hState[TSR_PRM_ACTPRC].nil?() && @hState[TSR_PRM_ACTPRC] != KER_TPRC_NONE)
        aErrors.push(YamlError.new("T3_TSK005: " + ERR_SET_ACTPRC_NO_ACTCNT, @aPath))
      end
      ### T3_TSK006: 状態がスピンロック取得待ちの時にspinidが指定されていない
      if (@hState[TSR_PRM_STATE] == TSR_STT_R_WAITSPN && @hState[TSR_PRM_SPINID].nil?())
        aErrors.push(YamlError.new("T3_TSK006: " + ERR_NO_SPINID_WAITING, @aPath))
      end

      # dormantのタスク
      if (is_dormant?())
        @aSpecifiedAttributes.each{|sAtr|
          ### T3_TSK007: 休止状態の場合に指定できないパラメータが指定されている
          if (sAtr != TSR_PRM_TYPE && !GRP_ACTIVATE_PRM_ON_DORMANT[TSR_OBJ_TASK].include?(sAtr))
            sErr = sprintf("T3_TSK007: " + ERR_ATR_DORMANT_TASK, sAtr)
            aErrors.push(YamlError.new(sErr, @aPath))
          end
        }
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概  要: 初期値を補完する
    #=================================================================
    def complement_init_object_info()
      super()
      hMacro  = @cConf.get_macro()

      # tskpri
      unless (is_specified?(TSR_PRM_TSKPRI))
        @hState[TSR_PRM_TSKPRI]  = hMacro["TSK_PRI_MID"]
      end
      # itskpri
      unless (is_specified?(TSR_PRM_ITSKPRI))
        @hState[TSR_PRM_ITSKPRI] = hMacro["TSK_PRI_MID"]
      end
      # exinf
      if (!is_specified?(TSR_PRM_EXINF) && @sObjectType != TSR_OBJ_TASK_EXC)
        @hState[TSR_PRM_EXINF] = hMacro["EXINF_A"]
      end
      # bootcnt
      unless (is_specified?(TSR_PRM_BOOTCNT))
        @hState[TSR_PRM_BOOTCNT] = 0
      end
    end

    #=================================================================
    # 概　要: 補完を実行する
    #=================================================================
    def complement(cPrevObjectInfo)
      check_class(ProcessUnit, cPrevObjectInfo)  # 直前のオブジェクト情報

      super(cPrevObjectInfo)

      # 補完例外(1)
      if (!is_specified?(TSR_PRM_WOBJID) && (cPrevObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_WAI ||
          cPrevObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_WAS) && 
          (@hState[TSR_PRM_STATE] != KER_TTS_WAI && @hState[TSR_PRM_STATE] != KER_TTS_WAS))
        @hState[TSR_PRM_WOBJID] = nil
        @hState[TSR_PRM_TSKWAIT] = nil
      end
      # 補完例外(2)
      if (!is_specified?(TSR_PRM_LEFTTMO) && (cPrevObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_WAI ||
          cPrevObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_WAS) && 
          !cPrevObjectInfo.hState[TSR_PRM_WOBJID].nil?() && !cPrevObjectInfo.hState[TSR_PRM_LEFTTMO].nil?() &&
          ((@hState[TSR_PRM_STATE] != KER_TTS_WAI && @hState[TSR_PRM_STATE] != KER_TTS_WAS) || 
           cPrevObjectInfo.hState[TSR_PRM_WOBJID] != @hState[TSR_PRM_WOBJID]))
        @hState[TSR_PRM_LEFTTMO] = nil
      end
      # 補完例外(4)
      if (!is_specified?(TSR_PRM_PORDER) && (cPrevObjectInfo.hState[TSR_PRM_STATE] != @hState[TSR_PRM_STATE] ||
          cPrevObjectInfo.hState[TSR_PRM_TSKPRI] != @hState[TSR_PRM_TSKPRI]))
        @hState[TSR_PRM_PORDER] = nil
      end
    end

    #=================================================================
    # 概　要: 全補完終了後に実行する処理
    #=================================================================
    def complement_after()
      # dormantの場合，一部のパラメータを無効とする
      if (@hState[TSR_PRM_STATE] == KER_TTS_DMT)
        @hState.each_key{|sAtr|
          if (!is_specified?(sAtr) && !GRP_COMPLEMENT_PRM_ON_DORMANT.include?(sAtr))
            @hState[sAtr] = nil
          end
        }
      end
    end

    #=================================================================
    # 概　要: 待ち対象が指定されているかを返す
    #=================================================================
    def has_wait_target?()
      return !(@hState[TSR_PRM_WOBJID].nil?())
    end

    #=================================================================
    # 概　要: 待ちオブジェクトが指定されているかを返す
    #=================================================================
    def has_wait_object?()
      return (has_wait_target?() && !GRP_WAIT_NON_OBJECT.include?(@hState[TSR_PRM_WOBJID]))  # [Bool]待ちオブジェクトが指定されているか
    end

    #=================================================================
    # 概　要: 実行可能状態かを返す
    #=================================================================
    def is_ready?()
      return (@hState[TSR_PRM_STATE] == KER_TTS_RDY)  # [Bool]実行可能状態か
    end

    #=================================================================
    # 概　要: オブジェクト待ち状態かを返す
    #=================================================================
    def is_object_waiting?()
      return ([KER_TTS_WAI, KER_TTS_WAS].include?(@hState[TSR_PRM_STATE]))  # [Bool]オブジェクト待ち状態か
    end

    #=================================================================
    # 概　要: 休止状態かを返す
    #=================================================================
    def is_dormant?()
      return (@hState[TSR_PRM_STATE] == KER_TTS_DMT)  # [Bool]休止状態か
    end

    #=================================================================
    # 概　要: 過渡状態かを返す
    #=================================================================
    def is_running_suspended?()
      return (@hState[TSR_PRM_STATE] == KER_TTS_RUS)  # [Bool]過渡状態か
    end
  end
end
