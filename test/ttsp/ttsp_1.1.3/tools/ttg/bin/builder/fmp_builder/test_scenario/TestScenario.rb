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
#  $Id: TestScenario.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: TestScenario
  # 概    要: PreCondition，Do，PostConditionのデータを保持
  #===================================================================
  class TestScenario

    #=================================================================
    # 概  要: 過渡状態のタスクの有無確認
    #=================================================================
    def exist_pre_task_running_suspended_fmp()
      return @cPreCondition.exist_pre_task_running_suspended_fmp()  # [Bool]過渡状態のタスクの有無
    end

    #=================================================================
    # 概  要: 現在優先度と初期優先度が異なるタスクの有無確認
    #=================================================================
    def exist_pre_task_pri_chg_fmp()
      return @cPreCondition.exist_pre_task_pri_chg_fmp()  # [Bool]現在優先度と初期優先度が異なるタスクの有無
    end

    #=================================================================
    # 概  要: 実行状態のタスクの有無確認
    #=================================================================
    def exist_task_running_fmp(nIndex = nil)
      check_class(Integer, nIndex, true)  # do/postのシーケンス番号

      if (nIndex.nil?())
        return @cPreCondition.exist_task_running_fmp()  # [Bool]実行状態のタスクの有無
      else
        return @aDo_PostCondition[nIndex].exist_task_running_fmp()  # [Bool]実行状態のタスクの有無
      end
    end

    #=================================================================
    # 概  要: 起動中の非タスクの有無確認
    #=================================================================
    def exist_nontask_activate_fmp(nIndex = nil)
      check_class(Integer, nIndex, true)  # do/postのシーケンス番号

      if (nIndex.nil?())
        return @cPreCondition.exist_nontask_activate_fmp()  # [Bool]起動中の非タスクの有無
      else
        return @aDo_PostCondition[nIndex].exist_nontask_activate_fmp()  # [Bool]起動中の非タスクの有無
      end
    end

    #=================================================================
    # 概  要: スピンロック取得待ち状態の処理単位の有無確認
    #=================================================================
    def exist_wait_spinlock_fmp(nIndex)
      check_class(Integer, nIndex)  # do/postのシーケンス番号

      return @aDo_PostCondition[nIndex].exist_wait_spinlock_fmp()  # [Bool]スピンロック取得待ち状態の処理単位の有無
    end

    #=================================================================
    # 概  要: 休止状態のタスク設定のためのコードを返す
    #=================================================================
    def gc_pre_task_dormant_fmp()
      return @cPreCondition.gc_pre_task_dormant_fmp()  # [IMCodeElement]休止状態のタスク設定のためのコード
    end

    #=================================================================
    # 概  要: オブジェクト待ちタスク設定のためのコードを返す
    #=================================================================
    def gc_pre_task_scobj_waiting_fmp()
      return @cPreCondition.gc_pre_task_scobj_waiting_fmp()  # [IMCodeElement]オブジェクト待ちタスク設定のためのコード
    end

    #=================================================================
    # 概  要: 待ち状態(スリープ，ディレイ)のタスクと，二重待ち状態の
    #         待ち状態のタスクに設定するコードを返す
    #         （二重待ち状態の待ち状態(スリープ，ディレイ)も含む）
    #=================================================================
    def gc_pre_task_waiting_fmp()
      return @cPreCondition.gc_pre_task_waiting_fmp()  # [IMCodeElement]待ち状態のタスクに設定するコード
    end

    #=================================================================
    # 概  要: 強制待ち・二重待ち状態に設定するコードを返す
    #         （二重待ち状態の強制待ちも含む）
    #=================================================================
    def gc_pre_task_suspended_fmp()
      return @cPreCondition.gc_pre_task_suspended_fmp()  # [IMCodeElement]強制待ち・二重待ち状態に設定するコード
    end

    #=================================================================
    # 概  要: 実行可能状態のタスクに設定するコードを返す
    #         実際は起こした後に，一律に待ち状態にしている
    #         (タスクはこの時点で，タスク例外処理が許可されている)
    #=================================================================
    def gc_pre_task_ready_fmp()
      return @cPreCondition.gc_pre_task_ready_fmp()  # [IMCodeElement]実行可能状態のタスクに設定するコード
    end

    #=================================================================
    # 概  要: 実行状態のタスク，タスク例外の設定をするコードを返す
    #=================================================================
    def gc_pre_task_running_texhdr_activate_fmp()
      return @cPreCondition.gc_pre_task_running_texhdr_activate_fmp()  # [IMCodeElement]実行状態のタスク，タスク例外の設定をするコード
    end

    #=================================================================
    # 概  要: 起動中の非タスクの設定コードを返す
    #=================================================================
    def gc_pre_nontask_activate_fmp()
      return @cPreCondition.gc_pre_nontask_activate_fmp()  # [IMCodeElement]起動中の非タスクの設定コード
    end

    #=================================================================
    # 概  要: 一時スリープさせておいたタスクを実行可能状態にさせるコードを返す
    #=================================================================
    def gc_pre_task_ready_porder_fmp()
      return @cPreCondition.gc_pre_task_ready_porder_fmp()  # [IMCodeElement]一時スリープさせておいたタスクを実行可能状態にさせるコード
    end

    #=================================================================
    # 概  要: 起動・起床要求キューイングの設定をするコードを返す
    #=================================================================
    def gc_pre_task_queueing_fmp()
      return @cPreCondition.gc_pre_task_queueing_fmp()  # [IMCodeElement]起動・起床要求キューイングの設定をするコード
    end

    #=================================================================
    # 概  要: 停止中のタイムイベントハンドラを設定するコードを返す
    #=================================================================
    def gc_pre_time_event_stp_other_fmp()
      return @cPreCondition.gc_pre_time_event_stp_other_fmp()  # [IMCodeElement]停止中のタイムイベントハンドラを設定するコード
    end

    #=================================================================
    # 概  要: 動作状態（Txxx_STA）のタイムイベントハンドラを設定する
    #         コードを返す
    #=================================================================
    def gc_pre_time_event_sta_fmp()
      return @cPreCondition.gc_pre_time_event_sta_fmp()  # [IMCodeElement]動作状態（Txxx_STA）のタイムイベントハンドラを設定するコード
    end

    #=================================================================
    # 概  要: ディスパッチ禁止にさせるコードを返す
    #=================================================================
    def gc_pre_dis_dsp_fmp()
      return @cPreCondition.gc_pre_dis_dsp_fmp()  # [IMCodeElement]ディスパッチ禁止にさせるコード
    end

    #=================================================================
    # 概  要: 割込み優先度マスクが0以外にさせるコードを返す
    #=================================================================
    def gc_pre_set_ipm_fmp()
      return @cPreCondition.gc_pre_set_ipm_fmp()  # [IMCodeElement]割込み優先度マスクが0以外にさせるコード
    end

    #=================================================================
    # 概  要: スピンロック状態を設定するコードを返す
    #=================================================================
    def gc_pre_spin_loc_fmp()
      return @cPreCondition.gc_pre_spin_loc_fmp()  # [IMCodeElement]スピンロック状態を設定するコード
    end

    #=================================================================
    # 概  要: CPUロック状態を設定するコードを返す
    #=================================================================
    def gc_pre_cpu_loc_fmp()
      return @cPreCondition.gc_pre_cpu_loc_fmp()  # [IMCodeElement]CPUロック状態を設定するコード
    end

    #=================================================================
    # 概  要: 過渡状態に設定するコードを返す
    #=================================================================
    def gc_pre_task_running_suspended_fmp()
      return @cPreCondition.gc_pre_task_running_suspended_fmp()  # [IMCodeElement]過渡状態に設定するコード
    end

    #=================================================================
    # 概  要: オブジェクト毎に生成したref_codeを返す
    #=================================================================
    def gc_obj_ref_fmp(nIndex = nil)
      check_class(Integer, nIndex, true)  # do/postのシーケンス番号

      if (nIndex.nil?())
        return @cPreCondition.gc_obj_ref_fmp()  # [IMCodeElement]オブジェクト毎に生成したref_code
      else
        return @aDo_PostCondition[nIndex].gc_obj_ref_fmp()  # [IMCodeElement]オブジェクト毎に生成したref_code
      end
    end

    #=================================================================
    # 概  要: doのコードを返す
    #=================================================================
    def gc_do_fmp(nIndex)
      check_class(Integer, nIndex)  # do/postのシーケンス番号

      if (nIndex == 0)
        aPrevObjectInfo = @cPreCondition.get_actvate_running_fmp()
      else
        aPrevObjectInfo = @aDo_PostCondition[nIndex - 1].get_actvate_running_fmp()
      end

      return @aDo_PostCondition[nIndex].gc_do_fmp(aPrevObjectInfo)  # [IMCodeElement]doのコード
    end

    #=================================================================
    # 概  要: ディスパッチ可能にさせるコードを返す
    #=================================================================
    def gc_lastpost_ena_dsp_fmp()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_ena_dsp_fmp()  # [IMCodeElement]ディスパッチ可能にさせるコード
    end

    #=================================================================
    # 概  要: 割込み優先度マスクを初期化させるコードを返す
    #=================================================================
    def gc_lastpost_set_ini_ipm_fmp()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_set_ini_ipm_fmp()  # [IMCodeElement]割込み優先度マスクを初期化させるコード
    end

    #=================================================================
    # 概  要: スピンロック解放のコードを返す
    #=================================================================
    def gc_lastpost_spin_unl_fmp()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_spin_unl_fmp()  # [IMCodeElement]スピンロック解放のコード
    end

    #=================================================================
    # 概  要: CPUロック状態の解除のコードを返す
    #=================================================================
    def gc_lastpost_cpu_unl_fmp()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_cpu_unl_fmp()  # [IMCodeElement]CPUロック状態の解除のコード
    end

    #=================================================================
    # 概  要: 他タスクを終了させるコードを返す
    #=================================================================
    def gc_lastpost_all_task_ter_fmp()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_all_task_ter_fmp()  # [IMCodeElement]他タスクを終了させるコード
    end

    #=================================================================
    # 概  要: BarrierSyncを行うコードを返す
    #=================================================================
    def gc_barrier_sync(nIndex = nil)
      check_class(Integer, nIndex, true)  # do/postのシーケンス番号

      if (nIndex.nil?())
        return @cPreCondition.gc_barrier_sync()  # [IMCodeElement]BarrierSyncを行うコード
      else
        return @aDo_PostCondition[nIndex].gc_barrier_sync()  # [IMCodeElement]BarrierSyncを行うコード
      end
    end

    #=================================================================
    # 概  要: DoStartSyncを行うコードを返す
    #=================================================================
    def gc_do_start_sync(nIndex)
      check_class(Integer, nIndex)  # do/postのシーケンス番号

      if (nIndex == 0)
        cPrev = @cPreCondition
      else
        cPrev = @aDo_PostCondition[nIndex - 1]
      end

      return @aDo_PostCondition[nIndex].gc_do_start_sync(cPrev)  # [IMCodeElement]DoStartSyncを行うコード
    end

    #=================================================================
    # 概  要: DoFinishSyncを行うコードを返す
    #=================================================================
    def gc_do_finish_sync(nIndex)
      check_class(Integer, nIndex)  # do/postのシーケンス番号

      if (nIndex == 0)
        cPrev = @cPreCondition
      else
        cPrev = @aDo_PostCondition[nIndex - 1]
      end

      return @aDo_PostCondition[nIndex].gc_do_finish_sync(cPrev)  # [IMCodeElement]DoFinishSyncを行うコード
    end

    #=================================================================
    # 概  要: 最後のrefが完了するまで他プロセッサの後処理を止めておく
    #         同期を行うコードを返す
    #=================================================================
    def gc_last_running_sync()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_last_running_sync()  # [IMCodeElement]最後のrefが完了するまで他プロセッサの後処理を止めておく同期を行うコード
    end

    #=================================================================
    # 概  要: readyのタスクを全てslp_tskするコードを返す
    #=================================================================
    def gc_lastpost_ready_sleep_fmp()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_ready_sleep_fmp()  # [IMCodeElement]readyのタスクを全てslp_tskするコード
    end

    #=================================================================
    # 概  要: 時間進めるコードを返す
    #=================================================================
    def gc_tick_gain_fmp(nIndex)
      check_class(Integer, nIndex)  # do/postのシーケンス番号

      cElement = IMCodeElement.new()
      nGainTick = get_post_time(nIndex)

      @aDo_PostCondition[nIndex].gc_tick_gain_fmp(cElement, @aDo_PostCondition[nIndex + 1].aActivate, @aDo_PostCondition[nIndex + 1].cRunning, nGainTick)

      return cElement  # [IMCodeElement]時間進めるコード
    end

    #=================================================================
    # 概  要: スピンロック取得待ちのために遅延処理するコードを返す
    #=================================================================
    def gc_delay_wait_spin_fmp(nIndex)
      check_class(Integer, nIndex)  # do/postのシーケンス番号

      # メインタスクの状態はそのままとするため，情報を渡す
      if (nIndex == 0)
        cPrevCondition = @cPreCondition
      else
        cPrevCondition = @aDo_PostCondition[nIndex - 1]
      end

      return @aDo_PostCondition[nIndex].gc_delay_wait_spin_fmp(cPrevCondition.lMainTaskState)  # [IMCodeElement]スピンロック取得待ちのために遅延処理するコード
    end

    #=================================================================
    # 概  要: 許可状態の割込みを設定するコードを返す
    #=================================================================
    def gc_pre_interrupt_ena_fmp()
      return @cPreCondition.gc_pre_interrupt_ena_fmp()  # [IMCodeElement]許可状態の割込みを設定するコード
    end

    #=================================================================
    # 概  要: 同期処理後のgcov_pause用コードを生成して返す
    #=================================================================
    def gc_gcov_pause_after_sync(nIndex)
      check_class(Integer, nIndex)  # do/postのシーケンス番号

      return @aDo_PostCondition[nIndex].gc_gcov_pause_after_sync()  # [IMCodeElement]同期処理後のgcov_pause用コード
    end

    #=================================================================
    # 概  要: 実行順序を守るための同期用コードを生成して返す
    #=================================================================
    def gc_exec_sequence_sync(nIndex = nil)
      check_class(Integer, nIndex, true)  # do/postのシーケンス番号

      if (nIndex.nil?())
        # pre_condition時はpost_conditionにおける実行順序だけを考慮すればよい
        return @cPreCondition.gc_exec_sequence_sync()  # [IMCodeElement]実行順序を守るための同期用コード
      end

      # post_conditionは前の状態のACTIVATEな非タスクの有無で待つ処理単位を変える
      if (nIndex == 0)
        cPrev = @cPreCondition
      else
        cPrev = @aDo_PostCondition[nIndex - 1]
      end

      return @aDo_PostCondition[nIndex].gc_exec_sequence_sync(cPrev)  # [IMCodeElement]実行順序を守るための同期用コード
    end

    #=================================================================
    # 概  要: 禁止状態の割込みを設定するコードを返す
    #=================================================================
    def gc_lastpost_interrupt_dis_fmp()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_interrupt_dis_fmp()  # [IMCodeElement]禁止状態の割込みを設定するコード
    end

  end
end
