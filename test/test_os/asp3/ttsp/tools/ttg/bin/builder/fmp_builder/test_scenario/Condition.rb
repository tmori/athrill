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
#  $Id: Condition.rb 11 2012-10-25 09:29:59Z nces-shigihara $
#

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: Condition
  # 概    要: pre_condition, post_conditionの情報を処理するクラス
  #===================================================================
  class Condition

    #=================================================================
    # 概  要: 実行状態のタスクがある場合はtrueを，
    #         ない場合はfalseを返す
    #=================================================================
    def exist_task_running_fmp()
      @aRunning.each{ |aRunning|
        if (!aRunning.nil?())
          return true  # [Bool]実行状態のタスクがある場合
        end
      }

      return false  # [Bool]実行状態のタスクがない場合
    end

    #=================================================================
    # 概  要: 起動中の非タスクがある場合はtrueを，
    #         ない場合はfalseを返す
    #=================================================================
    def exist_nontask_activate_fmp()
      @aActivate.each{ |cActivate|
        if (!cActivate.nil?())
          return true  # [Bool]起動中の非タスクがある場合
        end
      }

      return false  # [Bool]起動中の非タスクがない場合
    end

    #=================================================================
    # 概  要: 実行中のタスクを返す
    #         (ACTIVATEなタスク例外が関連付いている場合はタスク例外を返す)
    #=================================================================
    def get_running_fmp()
      aRunning = []
      @aTask.each{|nPrcid, sObjectID, cObjectInfo|
        if ((cObjectInfo.sObjectType == TSR_OBJ_TASK) && GRP_ACTIVATE.include?(cObjectInfo.hState[TSR_PRM_STATE]))
          if (!cObjectInfo.cTex.nil?() && GRP_ACTIVATE.include?(cObjectInfo.cTex.hState[TSR_PRM_HDLSTAT]))
            aRunning[nPrcid] = cObjectInfo.cTex
          else
            aRunning[nPrcid] = cObjectInfo
          end
        end
      }

      return aRunning  # [Array]実行中のタスク
    end

    #=================================================================
    # 概  要: 実行中の非タスクを返す
    #=================================================================
    def get_activate_fmp()
      aActivate = []

      @hAllObject.each{|sObjectID, cObjectInfo|
        if (GRP_NON_CONTEXT.include?(cObjectInfo.sObjectType) && GRP_ACTIVATE.include?(cObjectInfo.hState[TSR_PRM_HDLSTAT]))
          aActivate[cObjectInfo.hState[TSR_PRM_PRCID]] = cObjectInfo
        end
      }

      return aActivate  # [Array]実行中の非タスク
    end

    #=================================================================
    # 概  要: CPU状態を返す
    #=================================================================
    def get_cpu_state_fmp()
      aCpuState = []
      @hAllObject.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.sObjectType[TSR_OBJ_CPU_STATE])
           aCpuState[cObjectInfo.hState[TSR_PRM_PRCID]] = cObjectInfo
        end
      }
      return aCpuState  # [Array]CPU状態
    end

    #=================================================================
    # 概  要: 実行中の処理単位を返す
    #=================================================================
    def get_actvate_running_fmp()
      aProcUnitInfo = []

      @aPrcid.each{ |nPrcid|
        if (!@aActivate[nPrcid].nil?())
          aProcUnitInfo.push(@aActivate[nPrcid])
        elsif (!@aRunning[nPrcid].nil?())
          aProcUnitInfo.push(@aRunning[nPrcid])
        end
      }

      return aProcUnitInfo  # [Array]実行中の処理単位
    end

    #=================================================================
    # 概  要: 全オブジェクトのref処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_obj_ref_fmp()
      cElement      = IMCodeElement.new()

      # refはCPU状態，変数以外はすべてメインプロセッサから行う
      hProcUnitInfo = get_proc_unit_info(@cActivate, @cRunning)

      # コメント出力
      sComment = String.new()
      if (@nSeqNum.nil?() && @nTimeTick.nil?())
        sComment = "#{TSR_LBL_PRE}"
      else
        sComment = "#{TSR_UNS_POST}#{@nSeqNum}_#{@nTimeTick}"
      end
      cElement.set_comment(hProcUnitInfo, sComment)

      @aPrcid.each{|nPrcid|
        # タスク例外がACTIAVTEだった場合，texptn/exinfが引数と一致していることを確認するコードを入れる
        if (@aActivate[nPrcid].nil?() && !@aRunning[nPrcid].nil?() && @aRunning[nPrcid].sObjectType == TSR_OBJ_TASK_EXC)
          @aRunning[nPrcid].gc_assert_texptn_exinf(cElement, get_proc_unit_info(@aRunning[nPrcid]))
        end

        # 割込みサービスルーチンがACTIAVTEだった場合，exinfが引数と一致していることを確認するコードを入れる
        if (!@aActivate[nPrcid].nil?() && @aActivate[nPrcid].sObjectType == TSR_OBJ_ISR)
          @aActivate[nPrcid].gc_assert_exinf(cElement, get_proc_unit_info(@aActivate[nPrcid]))
        end

        # 実行中の処理単位に値が指定された変数があれば，一致していることを確認するコードを入れる
        if (!@aActivate[nPrcid].nil?() && !@aActivate[nPrcid].hState[TSR_PRM_VAR].nil?())
          @aActivate[nPrcid].gc_assert_value(cElement, get_proc_unit_info(@aActivate[nPrcid]))
        elsif (!@aRunning[nPrcid].nil?() && !@aRunning[nPrcid].hState[TSR_PRM_VAR].nil?())
          @aRunning[nPrcid].gc_assert_value(cElement, get_proc_unit_info(@aRunning[nPrcid]))
        end
      }

      # メインプロセッサのCPU_STATEがcpu_lockを持っているかを参照
      bCpuLock = false
      if (!@cCpuState.nil?() && (@cCpuState.hState[TSR_PRM_LOCCPU] == true))
        bCpuLock = true
      end

      # 全オブジェクトのref_codeをまとめる
      @hAllObject.each{|sObjectID, cObjectInfo|
        case cObjectInfo.sObjectType
        when TSR_OBJ_CPU_STATE
          # 他プロセッサのCPU状態は他プロセッサの実行状態の処理単位から参照する
          nPrcid = cObjectInfo.hState[TSR_PRM_PRCID]

          if (!@aActivate[nPrcid].nil?())
            hTempInfo = get_proc_unit_info(@aActivate[nPrcid])
            cElement.set_comment(hTempInfo, "#{sComment} (CPU state)")
            cObjectInfo.gc_obj_ref(cElement, hTempInfo)
          # メインプロセッサの場合は，実行中のタスクがいなくてもメインタスクが参照する
          elsif (!@aRunning[nPrcid].nil?() || (nPrcid == @sMainPrcid))
            hTempInfo = get_proc_unit_info(@aRunning[nPrcid])
            cElement.set_comment(hTempInfo, "#{sComment} (CPU state)")
            cObjectInfo.gc_obj_ref(cElement, hTempInfo)
          end

        when TSR_OBJ_TASK_EXC
          # タスク例外は関連付けされたタスクが休止状態の場合，参照できない
          cTask = get_object_info(cObjectInfo.hState[TSR_PRM_TASK])
          if (cTask.hState[TSR_PRM_STATE] != KER_TTS_DMT)
            cObjectInfo.gc_obj_ref(cElement, hProcUnitInfo, check_activate_context(), bCpuLock)
          end

        # 状態を参照できない処理単位の場合，何もしない
        when TSR_OBJ_INTHDR, TSR_OBJ_ISR, TSR_OBJ_EXCEPTION, TSR_OBJ_INIRTN, TSR_OBJ_TERRTN

        else
          cObjectInfo.gc_obj_ref(cElement, hProcUnitInfo, check_activate_context(), bCpuLock)

        end
      }

      # メインプロセッサのチェックポイント
      cElement.set_checkpoint(hProcUnitInfo)

      return cElement  # [IMCodeElement]全オブジェクトのref処理
    end


    #=================================================================
    # 概  要: BarrierSyncを行うコードを返す
    #=================================================================
    def gc_barrier_sync()
      cElement      = IMCodeElement.new()
      aProcUnitInfo = [] 

      @aPrcid.each{ |nPrcid|
        if (nPrcid == @sMainPrcid)
          aProcUnitInfo.push(get_proc_unit_info(@cActivate, @cRunning))
        elsif (!@aActivate[nPrcid].nil?())
          aProcUnitInfo.push(get_proc_unit_info(@aActivate[nPrcid]))
        elsif (!@aRunning[nPrcid].nil?())
          aProcUnitInfo.push(get_proc_unit_info(@aRunning[nPrcid]))
        end
      }

      if (aProcUnitInfo.size > 1)
        cElement.set_barrier_sync(aProcUnitInfo)
      end

      return cElement  # [IMCodeElement]BarrierSyncを行うコード
    end

    #=================================================================
    # 概  要: DoStartSyncを行うコードを返す
    #=================================================================
    def gc_do_start_sync(cPrevCondition)
      check_class(Condition, cPrevCondition)  # 前のコンディション

      cElement = IMCodeElement.new()

      # doでAPIを発行するプロセッサ以外において，前のコンディションでは実行状態で，
      # post_conditonで実行状態ではなくなるタスクがいる場合，
      # doのAPI発行まで実行状態のままとする
      cPrevCondition.aRunning.each{|cObjectInfo|
        if (!cObjectInfo.nil?() && !@hDo.empty?())
          # doでAPIを発行するオブジェクトの前状態における情報取得
          cPreDoObjectInfo = cPrevCondition.get_object_info(@hDo[TSR_PRM_ID])

          # doでAPIを発行するタスクの場合は何もしない
          if (cObjectInfo.hState[TSR_PRM_PRCID] == cPreDoObjectInfo.hState[TSR_PRM_PRCID])
            next
          end

          # 現在のオブジェクト情報取得
          cNowObjectInfo = get_object_info(cObjectInfo.sObjectID)

          # API発行後にチェックポイントを設定し，実行状態のタスクからはそのチェックポイントを待機する
          if (!GRP_ACTIVATE.include?(cNowObjectInfo.hState[TSR_PRM_STATE]))
            # doでAPIを発行するオブジェクトの後状態における情報取得
            cPostDoObjectInfo = get_object_info(@hDo[TSR_PRM_ID])

            # 後状態でAPIを発行した処理単位が実行状態でなくなる場合は，
            # メインプロセッサの実行状態の処理単位にチェックポイントを入れる
            if (!GRP_ACTIVATE.include?(cPostDoObjectInfo.hState[TSR_PRM_STATE]))
              cElement.set_checkpoint(get_proc_unit_info(@cActivate, @cRunning))
            else
              cElement.set_checkpoint(get_proc_unit_info(cPostDoObjectInfo))
            end
            cElement.set_wait_check_sync(get_proc_unit_info(cObjectInfo), cPostDoObjectInfo.hState[TSR_PRM_PRCID])
          end
        end
      }

      # doでAPIを発行するプロセッサ以外において，前のコンディションでは実行状態で，
      # post_conditonで実行状態ではなくなる非タスクがいる場合，
      # doのAPI発行まで実行状態のままとする
      cPrevCondition.aActivate.each{|cObjectInfo|
        if (!cObjectInfo.nil?() && !@hDo.empty?())
          # doでAPIを発行するオブジェクトの前状態における情報取得
          cPreDoObjectInfo = cPrevCondition.get_object_info(@hDo[TSR_PRM_ID])

          # doでAPIを発行する非タスクの場合は何もしない
          if (cObjectInfo.hState[TSR_PRM_PRCID] == cPreDoObjectInfo.hState[TSR_PRM_PRCID])
            next
          end

          # 現在のオブジェクト情報取得
          cNowObjectInfo = get_object_info(cObjectInfo.sObjectID)

          # API発行後にチェックポイントを設定し，実行状態の非タスクからはそのチェックポイントを待機する
          if (!GRP_ACTIVATE.include?(cNowObjectInfo.hState[TSR_PRM_HDLSTAT]))
            # doでAPIを発行するオブジェクトの後状態における情報取得
            cPostDoObjectInfo = get_object_info(@hDo[TSR_PRM_ID])

            cElement.set_checkpoint(get_proc_unit_info(cPostDoObjectInfo))
            cElement.set_wait_check_sync(get_proc_unit_info(cObjectInfo), cPostDoObjectInfo.hState[TSR_PRM_PRCID])
          end
        end
      }

      return cElement  # [IMCodeElement]DoStartSyncを行うコード
    end

    #=================================================================
    # 概  要: DoFinishSyncを行うコードを返す
    #=================================================================
    def gc_do_finish_sync(cPrevCondition)
      check_class(Condition, cPrevCondition)  # 前のコンディション

      cElement = IMCodeElement.new()

      # 他プロセッサにおいて，前のコンディションでは実行状態だったが，
      # ディスパッチにより実行状態ではなくなったタスクがいる場合，
      # そのタスクが現在の状態になるまで待機する(先にrefするのを防ぐ)
      cPrevCondition.aRunning.each_with_index{|cObjectInfo, nPrcid|
        if (!cObjectInfo.nil?() && (nPrcid != @sMainPrcid))
          # 現在のオブジェクト情報取得
          cNowObjectInfo = get_object_info(cObjectInfo.sObjectID)

          # タスク例外の場合は，関連タスクの情報を取り出す
          if (cNowObjectInfo.sObjectType == TSR_OBJ_TASK_EXC)
            cNowObjectInfo = get_object_info(cNowObjectInfo.hState[TSR_PRM_TASK])
          end

          # 対象タスクが現在の状態になるまでStateSyncする
          if (!GRP_ACTIVATE.include?(cNowObjectInfo.hState[TSR_PRM_STATE]))
             cElement.set_state_sync(get_proc_unit_info(@cActivate, @cRunning), cNowObjectInfo.sObjectID, cNowObjectInfo.hState[TSR_PRM_STATE])
          end
        end
      }

      return cElement  # [IMCodeElement]DoFinishSyncを行うコード
    end

    #=================================================================
    # 概  要: 時間を進める処理をcElementに格納する
    #=================================================================
    def gc_tick_gain_fmp(cElement, aNextActivate, cNextRunning, nGainTick)
      check_class(IMCodeElement, cElement)         # コードを追加するエレメント
      check_class(Array, aNextActivate)            # 次のコンディションでACTIVATEである非タスク
      check_class(ProcessUnit, cNextRunning, true) # 次のコンディションでRunningであるタスク
      check_class(Integer, nGainTick)              # 進める時間

      hNowProcUnitInfo = get_proc_unit_info(@cRunning)
      hNextProcUnitInfo = get_proc_unit_info(cNextRunning)

      # 他プロセッサに次のコンディションで起動するACTIVATEな非タスクが存在する場合，
      # メインプロセッサで起動を待つ
      aPrcidProcUnitInfo = []
      aWaitPrcid = []
      @aPrcid.each{|nPrcid|
        if (!aNextActivate[nPrcid].nil?() && (nPrcid != @sMainPrcid))
          cNowObjectInfo = get_object_info(aNextActivate[nPrcid].sObjectID)
          # 起動確認用チェックポイント
          if (cNowObjectInfo.hState[TSR_PRM_HDLSTAT] == TSR_STT_STP)
            cElement.set_checkpoint(get_proc_unit_info(cNowObjectInfo))
            aPrcidProcUnitInfo.push([hNowProcUnitInfo, nPrcid])
            aWaitPrcid.push(nPrcid)
          else
            cElement.set_checkpoint(get_proc_unit_info(aNextActivate[nPrcid]))
            aPrcidProcUnitInfo.push([hNextProcUnitInfo, nPrcid])
            aWaitPrcid.push(nPrcid)
          end
        end
      }

      # ローカルタイマ方式の場合，他プロセッサの時間が進んでからメインプロセッサを進める
      if (@cConf.is_timer_local?())
        if (!aPrcidProcUnitInfo.empty?())
          (1..nGainTick).each{|nCnt|
            if (nCnt == nGainTick)
              # 他プロセッサの時間を進めて起動する
              gc_tick_gain_local_other_fmp(cElement, aWaitPrcid)
              # メインプロセッサで実行中のタスクで起動したことを待つ
              aPrcidProcUnitInfo.each{|aPrcidProcUnitInfo|
                cElement.set_wait_check_sync(aPrcidProcUnitInfo[0], aPrcidProcUnitInfo[1])
              }
              # メインプロセッサの時間を進めて起動する
              gc_tick_gain_local_main_fmp(cElement)
            else
              # 途中の時間は，一括して進める
              gc_tick_gain_local_all_fmp(cElement)
            end
          }
        else
          # メインタスクのみの場合は，一括して時間を進める
          nGainTick.times{
            gc_tick_gain_local_all_fmp(cElement)
          }
        end
      # グローバルタイマ方式は時間管理プロセッサのタイムティックの供給のみ行う
      else
        nGainTick.times{
          gc_tick_gain_global_fmp(cElement, aWaitPrcid)
        }

        # 他プロセッサのACTIVATEな非タスクをメインプロセッサで実行中のタスクで起動したことを待つ
        if (!aPrcidProcUnitInfo.empty?())
          aPrcidProcUnitInfo.each{|aPrcidProcUnitInfo|
            cElement.set_wait_check_sync(aPrcidProcUnitInfo[0], aPrcidProcUnitInfo[1])
          }
        end
      end
    end

    #=================================================================
    # 概  要: 実行順序を守るための同期用コードを生成して返す
    #=================================================================
    def gc_exec_sequence_sync(cPrevCondition = nil)
      check_class(Condition, cPrevCondition, true)  # 前のコンディション

      cElement = IMCodeElement.new()

      @aPrcid.each{|nPrcid|
        # ACTIVATEなタスク例外とACTIVATEな非タスクが存在する場合
        if (!@aActivate[nPrcid].nil?() && !@aRunning[nPrcid].nil?() && (@aRunning[nPrcid].sObjectType == TSR_OBJ_TASK_EXC))
          cElement.set_checkpoint(get_proc_unit_info(@aActivate[nPrcid]))
          cElement.set_wait_check_sync(get_proc_unit_info(@aRunning[nPrcid]), nPrcid)
          hRunningTask  = get_proc_unit_info(get_object_info(@aRunning[nPrcid].hState[TSR_PRM_TASK]))
          cElement.set_wait_check_sync(hRunningTask, nPrcid)

          # ACTIVATEな非タスクが変わった場合は，元の非タスクから次の非タスクの起動を待つ
          if (!cPrevCondition.nil?() && !cPrevCondition.aActivate[nPrcid].nil?() &&
              (cPrevCondition.aActivate[nPrcid].sObjectID != @aActivate[nPrcid].sObjectID))
            cElement.set_wait_check_sync(get_proc_unit_info(cPrevCondition.aActivate[nPrcid]), nPrcid)
          end

        # ACTIVATEな非タスクが存在する かつ そのプロセッサに実行中のタスクがいる場合，
        # 先に処理が進まないよう非タスクの起動を待つ
        elsif (!@aActivate[nPrcid].nil?() && !@aRunning[nPrcid].nil?())
          cElement.set_checkpoint(get_proc_unit_info(@aActivate[nPrcid]))
          cElement.set_wait_check_sync(get_proc_unit_info(@aRunning[nPrcid]), nPrcid)

          # ACTIVATEな非タスクが変わった場合は，元の非タスクから次の非タスクの起動を待つ
          if (!cPrevCondition.nil?() && !cPrevCondition.aActivate[nPrcid].nil?() &&
              (cPrevCondition.aActivate[nPrcid].sObjectID != @aActivate[nPrcid].sObjectID))
            cElement.set_wait_check_sync(get_proc_unit_info(cPrevCondition.aActivate[nPrcid]), nPrcid)
          end

        # ACTIVATEなタスク例外が存在する場合，タスク側の処理が先に進まないようタスク例外の起動を待つ
        elsif (!@aRunning[nPrcid].nil?() && (@aRunning[nPrcid].sObjectType == TSR_OBJ_TASK_EXC))
          cElement.set_checkpoint(get_proc_unit_info(@aRunning[nPrcid]))
          hRunningTask  = get_proc_unit_info(get_object_info(@aRunning[nPrcid].hState[TSR_PRM_TASK]))
          cElement.set_wait_check_sync(hRunningTask, nPrcid)
        end
      }

      return cElement  # [IMCodeElement]実行順序を守るための同期用コード
    end

    #=================================================================
    # 概  要: メインプロセッサの時間を進める処理をcElementに格納する
    #         (ローカルタイマ方式用)
    #=================================================================
    def gc_tick_gain_local_main_fmp(cElement)
      check_class(IMCodeElement, cElement)   # コードを追加するエレメント

      hProcUnitInfo = get_proc_unit_info(@cRunning)

      # メインプロセッサはタイマ割込みハンドラの終了を待つ
      cElement.set_code(hProcUnitInfo, "#{FNC_GAIN_TICK_PE}(#{@sMainPrcid}, true)")
    end

    #=================================================================
    # 概  要: メインプロセッサ以外の時間を進める処理をcElementに格納する
    #         (ローカルタイマ方式用)
    #=================================================================
    def gc_tick_gain_local_other_fmp(cElement, aWaitPrcid)
      check_class(IMCodeElement, cElement)  # コードを追加するエレメント
      check_class(Array, aWaitPrcid)        # 非タスクの起動を待つプロセッサID

      hProcUnitInfo = get_proc_unit_info(@cRunning)

      # 時間を進める前にチェックポイント
      cElement.set_checkpoint(hProcUnitInfo)

      @aPrcid.each{ |nPrcid|
        if (nPrcid != @sMainPrcid)
          # 非タスクが起動する場合はタイマ割込みハンドラの終了を待たない
          if (aWaitPrcid.include?(nPrcid))
            cElement.set_code(hProcUnitInfo, "#{FNC_GAIN_TICK_PE}(#{nPrcid}, false)")
          else
            cElement.set_code(hProcUnitInfo, "#{FNC_GAIN_TICK_PE}(#{nPrcid}, true)")
          end
        end
      }
    end

    #=================================================================
    # 概  要: 全プロセッサの時間を進める処理をcElementに格納する
    #         (グローバルタイマ方式用)
    #=================================================================
    def gc_tick_gain_local_all_fmp(cElement)
      check_class(IMCodeElement, cElement)  # コードを追加するエレメント

      hProcUnitInfo = get_proc_unit_info(@cRunning)

      cElement.set_code(hProcUnitInfo, "#{FNC_GAIN_TICK}()")
    end

    #=================================================================
    # 概  要: メインプロセッサの時間を進める処理をcElementに格納する
    #         (グローバルタイマ方式用)
    #=================================================================
    def gc_tick_gain_global_fmp(cElement, aWaitPrcid)
      check_class(IMCodeElement, cElement)  # コードを追加するエレメント
      check_class(Array, aWaitPrcid)        # 非タスクの起動を待つプロセッサID

      hProcUnitInfo = get_proc_unit_info(@cRunning)

      nTimePrcid = @cConf.get_time_manage_prcid()

      # 時間管理プロセッサで非タスクが起動する場合はタイマ割込みハンドラの終了を待たない
      if (aWaitPrcid.include?(nTimePrcid))
        cElement.set_code(hProcUnitInfo, "#{FNC_GAIN_TICK_PE}(#{nTimePrcid}, false)")
      else
        cElement.set_code(hProcUnitInfo, "#{FNC_GAIN_TICK_PE}(#{nTimePrcid}, true)")
      end
    end
  end
end
