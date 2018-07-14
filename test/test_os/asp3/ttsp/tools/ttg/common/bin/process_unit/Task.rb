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
require "common/bin/process_unit/ProcessUnit.rb"
require "ttc/bin/process_unit/Task.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: Task
  # 概    要: タスクの情報を処理するクラス
  #===================================================================
  class Task < ProcessUnit
    attr_reader :cTex
    attr_accessor :nStackNum

    #=================================================================
    # 概  要: タスクの初期化
    #=================================================================
    def initialize(sObjectID, hObjectInfo, aPath, bIsPre)
      check_class(String, sObjectID)  # オブジェクトID
      check_class(Hash, hObjectInfo)  # オブジェクト情報
      check_class(Array, aPath)       # ルートからのパス
      check_class(Bool, bIsPre)       # pre_condition内か

      super(sObjectID, hObjectInfo, TSR_OBJ_TASK, aPath, bIsPre)

      # ref_tsk
      @sRefAPI     = FNC_REF_TSK
      @sRefStrType = TYP_T_TTSP_RTSK
      @sRefStrVar  = GRP_VAR_TYPE[TYP_T_TTSP_RTSK]
      @sRefState   = STR_TSKSTAT
      @sRefExInf   = STR_EXINF
      @sRefPrcID   = STR_PRCID
      @sRefActPrc  = STR_ACTPRC

      @cTex      = nil
      @nStackNum = nil
    end

    #=================================================================
    # 概  要: 待ち要因の設定
    #=================================================================
    def set_task_wait(cObjectInfo)
      check_class(Object, cObjectInfo)  # 待ち要因オブジェクト

      case cObjectInfo.sObjectType
      when TSR_OBJ_SEMAPHORE
        @hState[TSR_PRM_TSKWAIT] = KER_TTW_SEM
      when TSR_OBJ_EVENTFLAG
        @hState[TSR_PRM_TSKWAIT] = KER_TTW_FLG
      when TSR_OBJ_MAILBOX
        @hState[TSR_PRM_TSKWAIT] = KER_TTW_MBX
      when TSR_OBJ_MEMORYPOOL
        @hState[TSR_PRM_TSKWAIT] = KER_TTW_MPF
      when TSR_OBJ_DATAQUEUE
        if (!cObjectInfo.hState[TSR_PRM_STSKLIST].nil?() && !cObjectInfo.hState[TSR_PRM_STSKLIST].empty?())
          @hState[TSR_PRM_TSKWAIT] = KER_TTW_SDTQ
        elsif (!cObjectInfo.hState[TSR_PRM_RTSKLIST].nil?() && !cObjectInfo.hState[TSR_PRM_RTSKLIST].empty?())
          @hState[TSR_PRM_TSKWAIT] = KER_TTW_RDTQ
        end
      when TSR_OBJ_P_DATAQUEUE
        if (!cObjectInfo.hState[TSR_PRM_STSKLIST].nil?() && !cObjectInfo.hState[TSR_PRM_STSKLIST].empty?())
          @hState[TSR_PRM_TSKWAIT] = KER_TTW_SPDQ
        elsif (!cObjectInfo.hState[TSR_PRM_RTSKLIST].nil?() && !cObjectInfo.hState[TSR_PRM_RTSKLIST].empty?())
          @hState[TSR_PRM_TSKWAIT] = KER_TTW_RPDQ
        end
      else
        abort(ERR_MSG % [__FILE__, __LINE__])
      end
    end


    #=================================================================
    # 概  要: コンフィグファイルに出力するコード作成
    #=================================================================
    def gc_config(cElement)
      check_class(IMCodeElement, cElement) # エレメント

      cElement.set_config("#{API_CRE_TSK}(#{@sObjectID}, {#{KER_TA_NULL}, #{@hState[TSR_PRM_EXINF]}, #{@sObjectID.downcase}, #{@hState[TSR_PRM_ITSKPRI]}, TTSP_TASK_STACK_SIZE, #{@nStackNum ? "ttg_stack_#{@nStackNum}" : "NULL"}});", @hState[TSR_PRM_CLASS])

      # 順序に依存するためタスク例外ハンドラの分もここで作成
      if (@cTex)
        cElement.set_config("#{API_DEF_TEX}(#{@sObjectID}, {#{KER_TA_NULL}, #{@cTex.sObjectID.downcase}});", @hState[TSR_PRM_CLASS])
      end
    end

    #=================================================================
    # 概  要: タスク例外の関連付け設定
    #=================================================================
    def set_tex(cTex)
      check_class(TaskExcept, cTex)  # 設定するタスク例外クラス

      @cTex = cTex
    end

  end
end
