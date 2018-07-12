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
#  $Id: Do_PostCondition.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/test_scenario/Condition.rb"
require "ttc/bin/test_scenario/Do_PostCondition.rb"
require "ttj/bin/test_scenario/Do_PostCondition.rb"
require "bin/builder/fmp_builder/test_scenario/Do_PostCondition.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: Do_PostCondition
  # 概    要: do，post_conditionの情報を処理するクラス
  #===================================================================
  class Do_PostCondition < Condition
    include CommonModule

    attr_accessor :hDo
    attr_reader :nSeqNum, :nTimeTick

    #=================================================================
    # 概  要: do，post_conditionの初期化
    #=================================================================
    def initialize(sTestID, hScenarioDo, hScenarioPost, nSeqNum, nTimeTick, cPreCondition)
      check_class(String, sTestID)              # テストID
      check_class(Hash, hScenarioDo, true)      # do
      check_class(Hash, hScenarioPost, true)    # post_condition
      check_class(Integer, nSeqNum)             # シーケンス番号
      check_class(Integer, nTimeTick)           # タイムティック
      check_class(PreCondition, cPreCondition)  # PreCondition

      @nSeqNum                = nSeqNum
      @nTimeTick              = nTimeTick
      @cCallerObject          = nil        # API発行オブジェクト
      @hDo                    = {}
      @aSpecifiedDoAttributes = []

      super(sTestID)

      # 構造チェック
      aErrors = []
      begin
        basic_post_check(hScenarioPost)
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end
      check_error(aErrors)

      # 補完
      if (hScenarioDo.nil?())
        hScenarioDo = {}
      end
      if (hScenarioPost.nil?())
        hScenarioPost = {}
      end
      pre_attribute_check(hScenarioDo)

      # エラーがなければdo，post_conditionの情報格納
      store_do_info(hScenarioDo, cPreCondition)
      store_condition_info(hScenarioPost, cPreCondition.hObjectType)
    end

    #=================================================================
    # 概  要: doの情報を格納
    #=================================================================
    def store_do_info(hScenarioDo, cPreCondition)
      check_class(Hash, hScenarioDo)            # do
      check_class(PreCondition, cPreCondition)  # PreCondition

      # 格納
      @aSpecifiedDoAttributes = hScenarioDo.keys()
      hScenarioDo.each{|atr, val|
        case atr
        when TSR_PRM_ID
          @hDo[atr] = val
          @cCallerObject = cPreCondition.hAllObject[val]

        else
          @hDo[atr] = val
        end
      }
    end


    #=================================================================
    # 概  要: 処理単位情報をまとめてエレメントとして返す
    #=================================================================
    def get_proc_units(cElement)
      @hAllObject.each{|sObjectID, cObjectInfo|
        if (GRP_PROCESS_UNIT.include?(cObjectInfo.sObjectType))
          cElement.set_proc_unit(sObjectID, cObjectInfo.hState[TSR_PRM_BOOTCNT])
        end
      }
    end

    #=================================================================
    # 概  要: Doの処理をIMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_do(cPrevObjectInfo)
      check_class(ProcessUnit, cPrevObjectInfo) # 前のコンディション

      cElement = IMCodeElement.new()

      hProcUnitInfo = get_proc_unit_info(cPrevObjectInfo)

      # idに記載したIDと一致していることを確認する
      if (hProcUnitInfo[:id] != @hDo[TSR_PRM_ID])
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      # コメント出力
      cElement.set_comment(hProcUnitInfo, "#{TSR_UNS_DO}#{@nSeqNum}_#{@nTimeTick}")
      # GCOV取得の開始もしくは再開
      gc_gcov_resume(cElement, hProcUnitInfo)

      # syscall部
      bIsReturn = true
      if (@hDo.has_key?(TSR_PRM_SYSCALL))
        if (@hDo.has_key?(TSR_PRM_ERCD))
          cElement.set_syscall(hProcUnitInfo, @hDo[TSR_PRM_SYSCALL], @hDo[TSR_PRM_ERCD])
        elsif (@hDo.has_key?(TSR_PRM_ERUINT))
          cElement.set_syscall(hProcUnitInfo, @hDo[TSR_PRM_SYSCALL], @hDo[TSR_PRM_ERUINT], TYP_ER_UINT)
        elsif (@hDo.has_key?(TSR_PRM_BOOL))
          cElement.set_syscall(hProcUnitInfo, @hDo[TSR_PRM_SYSCALL], @hDo[TSR_PRM_BOOL], TYP_BOOL_T)
        else
          cElement.set_syscall(hProcUnitInfo, @hDo[TSR_PRM_SYSCALL], nil)
          bIsReturn = false
        end
      else
        # "#"から始まるコードはセミコロンを付与しない
        bSemicolon = true
        if (@hDo[TSR_PRM_CODE] =~ /^\#.*/)
          bSemicolon = false
        end
        cElement.set_code(hProcUnitInfo, @hDo[TSR_PRM_CODE], bSemicolon)
        bIsReturn = false
      end

      # GCOV取得の中断
      if (bIsReturn == true)
        # 戻り値がある場合，いずれ実行状態になるためAPIを発行した処理単位で行う
        gc_gcov_pause(cElement, hProcUnitInfo)
      else
        # 戻り値が無い場合，doの後で実行中の処理単位から行う
        gc_gcov_pause(cElement, get_proc_unit_info(@cActivate, @cRunning))
      end

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: gcov取得開始の処理をIMCodeElementクラスに追加して
    #         返す
    #=================================================================
    def gc_gcov_resume(cElement, hProcUnitInfo)
      check_class(IMCodeElement, cElement) # IMCodeElementクラスのインスタンス
      check_class(Hash, hProcUnitInfo)     # 処理を実行する処理単位情報

      if (@cConf.enable_gcov?() && (@hDo[TSR_PRM_GCOV] == true) && (@bGcovAll == false))
        cElement.set_code(hProcUnitInfo, FNC_GCOV_TTG_C_RESUME)
      end
    end

    #=================================================================
    # 概  要: gcov取得中断の処理をIMCodeElementクラスに追加して返す
    #=================================================================
    def gc_gcov_pause(cElement, hProcUnitInfo)
      check_class(IMCodeElement, cElement) # IMCodeElementクラスのインスタンス
      check_class(Hash, hProcUnitInfo)     # 処理を実行する処理単位情報

      if (@cConf.enable_gcov?() && (@hDo[TSR_PRM_GCOV] == true) && (@bGcovAll == false))
        cElement.set_code(hProcUnitInfo, FNC_GCOV_TTG_C_PAUSE)
      end
    end

    #=================================================================
    # 概  要: CPUロック状態の解除の処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_lastpost_cpu_unl()
      cElement = IMCodeElement.new()
      hProcUnitInfo = get_proc_unit_info(@cActivate, @cRunning)

      if (!@cActivate.nil?())
        cElement.set_syscall(hProcUnitInfo, "#{API_IUNL_CPU}()")
      else
        cElement.set_syscall(hProcUnitInfo, "#{API_UNL_CPU}()")
      end

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: 最後の後状態でメインタスクを起こすコードを返す
    #=================================================================
    def gc_lastpost_maintask_wup(lPrevMainTaskState)
      cElement = IMCodeElement.new()

      # 前状態でメインタスクが実行状態の場合
      if (lPrevMainTaskState == :running)
        # メインタスクが実行中であるため，何もしない

      # 前状態でメインタスクが実行状態の場合
      elsif (lPrevMainTaskState == :ready)
        if (!@cRunning.nil?())
          cElement.set_syscall(get_proc_unit_info(@cRunning), "#{API_CHG_PRI}(#{TTG_MAIN_TASK}, #{TTG_MAIN_PRI})")
          if (check_running_pri(TTG_MAIN_PRI) == true)
            cElement.set_syscall(get_proc_unit_info(@cRunning), "#{API_ROT_RDQ}(#{TTG_MAIN_PRI})")
          end

          # メインプロセッサに優先度がTTG_MAIN_PRIと同じでreadyのタスクがいる場合，rot_rdqを発行してディスパッチさせる
          @hTask.each{|sObjectID, cObjectInfo|
            if ((cObjectInfo.hState[TSR_PRM_PRCID] == @sMainPrcid) && (cObjectInfo.hState[TSR_PRM_TSKPRI] == TTG_MAIN_PRI) && (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RDY))
              cElement.set_syscall(get_proc_unit_info(cObjectInfo), "#{API_ROT_RDQ}(#{TTG_MAIN_PRI})")
            end
          }
        end

      # 前状態でメインタスクが起床待ち状態の場合
      elsif (lPrevMainTaskState == :sleep)
        if (!@cRunning.nil?())
          cElement.set_syscall(get_proc_unit_info(@cRunning), "#{API_WUP_TSK}(#{TTG_MAIN_TASK})")
          cElement.set_syscall(get_proc_unit_info(@cRunning), "#{API_CHG_PRI}(#{TTG_MAIN_TASK}, #{TTG_MAIN_PRI})")
          if (check_running_pri(TTG_MAIN_PRI) == true)
            cElement.set_syscall(get_proc_unit_info(@cRunning), "#{API_ROT_RDQ}(#{TTG_MAIN_PRI})")
          end

          # メインプロセッサに優先度がTTG_MAIN_PRIと同じでreadyのタスクがいる場合，rot_rdqを発行してディスパッチさせる
          @hTask.each{|sObjectID, cObjectInfo|
            if ((cObjectInfo.hState[TSR_PRM_PRCID] == @sMainPrcid) && (cObjectInfo.hState[TSR_PRM_TSKPRI] == TTG_MAIN_PRI) && (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RDY))
              cElement.set_syscall(get_proc_unit_info(cObjectInfo), "#{API_ROT_RDQ}(#{TTG_MAIN_PRI})")
            end
          }

        elsif (!@cActivate.nil?())
          cElement.set_syscall(get_proc_unit_info(@cActivate), "#{API_IWUP_TSK}(#{TTG_MAIN_TASK})")

        # メインタスクを起床できる処理単位がいない
        else
          abort(ERR_MSG % [__FILE__, __LINE__])
        end

      else
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: ディスパッチ禁止状態を解除する処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_lastpost_ena_dsp()
      cElement      = IMCodeElement.new()
      hProcUnitInfo = get_proc_unit_info(@cRunning)

      cElement.set_syscall(hProcUnitInfo, "#{API_ENA_DSP}()")

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: 割込み優先度マスクを初期化させる処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_lastpost_set_ini_ipm()
      cElement      = IMCodeElement.new()
      hProcUnitInfo = get_proc_unit_info(@cRunning)

      cElement.set_syscall(hProcUnitInfo, "#{API_CHG_IPM}(#{KER_TIPM_ENAALL})")

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: 動作状態のタイムイベントハンドラの停止処理のコードを
    #         IMCodeElementクラスにまとめて返す
    #         (一律メインタスクが実行する)
    #=================================================================
    def gc_lastpost_time_event_stp()
      cElement      = IMCodeElement.new()
      hProcUnitInfo = get_proc_unit_info()

      @hAllObject.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.sObjectType == TSR_OBJ_ALARM)
          cElement.set_syscall(hProcUnitInfo, "#{API_STP_ALM}(#{sObjectID})")
        elsif (cObjectInfo.sObjectType == TSR_OBJ_CYCLE)
          cElement.set_syscall(hProcUnitInfo, "#{API_STP_CYC}(#{sObjectID})")
        end
      }

      cElement.set_checkpoint(hProcUnitInfo)

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: 禁止状態の割込みを設定するコードを返す
    #=================================================================
    def gc_lastpost_interrupt_dis()
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      # dis_intする必要のある割込み番号を抽出
      aDisIntNo = []
      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((GRP_INTERRUPT.include?(cObjectInfo.sObjectType) == true) && (cObjectInfo.hState[TSR_PRM_STATE] == KER_TA_ENAINT))
          aDisIntNo.push(cObjectInfo.hState[TSR_PRM_INTNO])
        end
      }

      # 重複を削除してすべてdis_intを実行する
      aDisIntNo.uniq!()
      aDisIntNo.each{|snIntNo|
        cElement.set_syscall(hMainTaskInfo, "#{API_DIS_INT}(#{snIntNo})")
      }

      cElement.set_checkpoint(hMainTaskInfo)

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: 次の状態を処理する前に行うべきのメインの設定処理を
    #         IMCodeElementクラスクラスにまとめて返す
    #=================================================================
    def gc_post_maintask_set(cNextActivate, cNextRunning, bNextCpuLock, bNextRunSus, lPrevMainTaskState, bGainTime)
      check_class(ProcessUnit, cNextActivate, true) # 次のコンディションで起動している非タスク
      check_class(ProcessUnit, cNextRunning, true)  # 次のコンディションで実行中のタスク
      check_class(Bool, bNextCpuLock)               # 次のコンディションでCPUロックかどうか
      check_class(Bool, bNextRunSus)                # 次のコンディションで過渡状態かどうか
      check_class(Symbol, lPrevMainTaskState)       # 前のコンディションのメインタスクの状態
      check_class(Bool, bGainTime)                  # 時間を進める必要があるかどうか

      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      # 現在のCPU状態を取得
      bNowCpuLock = check_cpu_loc()

      # 現在と後状態の過渡状態を取得
      bNowRunSus = check_run_sus()

      # CPUロック状態の場合(何も発行できない)
      if (bNowCpuLock == true)
        @lMainTaskState = lPrevMainTaskState
        return nil
      end

      # 前状態でメインタスクが実行状態の場合
      if (lPrevMainTaskState == :running)
        # 次の条件をすべて満たす場合，次のコンディションで
        # メインタスクを起床できるため，slp_tskする
        # 1)次のコンディションに実行状態のタスクが存在する
        # 2)次のコンディションの実行状態のタスクが過渡状態でない
        #   (後処理で過渡状態を解除した時点で強制待ちとなり，メインタスクを起床できない)
        # 3)次のコンディションがCPUロックでない
        #   (複数のdo/postが続く場合に途中でchg_pri/wup_tskが発行できない可能性がある)
        if ((!cNextActivate.nil?() || !cNextRunning.nil?()) && (bNextRunSus == false) && (bNextCpuLock == false))
          cElement.set_syscall(hMainTaskInfo, "#{API_SLP_TSK}()")
          @lMainTaskState = :sleep

        # 上記以外は何もしない
        else
          @lMainTaskState = lPrevMainTaskState
        end

      # 前状態でメインタスクが起床待ち状態の場合
      elsif (lPrevMainTaskState == :sleep)
        # 現在実行状態のタスクが存在しない，かつ時間を進める必要がある場合は
        # メインタスクを一旦起動して時間を進めさせる
        if (@cRunning.nil?() && (bGainTime == true))
          hProcUnitInfo = get_proc_unit_info(@cActivate)
          cElement.set_syscall(hProcUnitInfo, "#{API_IWUP_TSK}(#{TTG_MAIN_TASK})")

          @lMainTaskState = :running

        # 次のいずれかの条件を満たす場合，次のコンディションで
        # 実行状態の処理単位がいなくなるため，メインタスクを起床する
        # 1)次のコンディションに実行状態の処理単位が存在しない
        # 2)次のコンディションがCPUロック状態
        # (do/postが続く場合に途中でchg_pri/wup_tskが発行できない可能性がある)
        # 3)次のコンディションの実行状態の処理単位が過渡状態
        # (後処理で過渡状態を解除した時点で強制待ちとなり，メインタスクを起床できない)
        elsif ((cNextActivate.nil?() && cNextRunning.nil?()) || (bNextCpuLock == true) || (bNextRunSus == true))
          hProcUnitInfo = get_proc_unit_info(@cActivate, @cRunning)
          if (@cActivate.nil?())
            cElement.set_syscall(hProcUnitInfo, "#{API_CHG_PRI}(#{TTG_MAIN_TASK}, #{TTG_WAIT_PRI})")
            cElement.set_syscall(hProcUnitInfo, "#{API_WUP_TSK}(#{TTG_MAIN_TASK})")

          else
            cElement.set_syscall(hProcUnitInfo, "#{API_IWUP_TSK}(#{TTG_MAIN_TASK})")

            # 実行状態のタスクがいる場合，メインタスクが実行状態とならないようchg_priしておく
            if (!@cRunning.nil?())
              cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{TTG_MAIN_TASK}, #{TTG_WAIT_PRI})")
              if (check_running_pri(TTG_WAIT_PRI) == true)
                cElement.set_syscall(hMainTaskInfo, "#{API_ROT_RDQ}(#{TTG_WAIT_PRI})")
              end
            end
          end

          @lMainTaskState = :ready
        # 上記以外は何もしない
        else
          @lMainTaskState = lPrevMainTaskState
        end

      elsif (lPrevMainTaskState == :ready)
        # 次の条件をすべて満たす場合，次のコンディションで
        # メインタスクを起床できるため，優先度を戻してslp_tskする
        # 1)次のコンディションに実行状態の処理単位が存在する
        # 2)次のコンディションの実行状態のタスクが過渡状態でない
        #   (後処理で過渡状態を解除した時点で強制待ちとなり，メインタスクを起床できない)
        # 3)次のコンディションがCPUロックでない
        #   (複数のdo/postが続く場合に途中でchg_pri/wup_tskが発行できない可能性がある)
        if ((!cNextActivate.nil?() || !cNextRunning.nil?()) && (bNextRunSus == false) && (bNextCpuLock == false))
          # メインタスクが既にTTG_MAIN_PRIの場合はchg_priを発行しない
          cElement.set_chg_pri_main_task(hMainTaskInfo)

          cElement.set_syscall(hMainTaskInfo, "#{API_SLP_TSK}()")
          @lMainTaskState = :sleep

        # 次のコンディションに実行状態のタスクが存在しない場合
        # メインタスクを実行状態とする
        elsif (cNextRunning.nil?())
          hProcUnitInfo = get_proc_unit_info(@cRunning)
          cElement.set_syscall(get_proc_unit_info(), "#{API_CHG_PRI}(#{TTG_TSK_SELF}, #{TTG_MAIN_PRI})")

          @lMainTaskState = :running
        # 上記以外は何もしない
        else
          @lMainTaskState = lPrevMainTaskState
        end

      else
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: 起動・起床要求キューイングの設定をする処理を
    #         IMCodeElementクラスにまとめて返す
    #         (一律メインタスクが実行する)
    #=================================================================
    def gc_lastpost_task_can_queueing()
      cElement = IMCodeElement.new()

      hProcUnitInfo = get_proc_unit_info()

      @hTask.each{|sObjectID, cObjectInfo|
        if (!cObjectInfo.hState[TSR_PRM_ACTCNT].nil?() && cObjectInfo.hState[TSR_PRM_ACTCNT] > 0)
          cElement.set_local_var(hProcUnitInfo[:id], TSR_PRM_ERUINT, "ER_UINT")
          cObjectInfo.hState[TSR_PRM_ACTCNT].times{
            cElement.set_syscall(hProcUnitInfo, "#{API_CAN_ACT}(#{sObjectID})", nil, TYP_ER_UINT)
            cElement.set_assert(hProcUnitInfo, TSR_PRM_ERUINT, cObjectInfo.hState[TSR_PRM_ACTCNT])
          }
        end

      }

      cElement.set_checkpoint(hProcUnitInfo)

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: readyのタスクを全てslp_tskする処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_lastpost_ready_sleep()
      cElement = IMCodeElement.new()

      @hTask.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RDY)
          hProcUnitInfo = get_proc_unit_info(cObjectInfo)
          cElement.set_syscall(hProcUnitInfo, "#{API_SLP_TSK}()")
        end
      }

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: 全タスクを起動終了させるコードを
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_lastpost_all_task_ter()
      # 他タスクが存在する場合，すべてter_tsk()させる
      cElement = IMCodeElement.new()

      hProcUnitInfo = get_proc_unit_info()

      @hTask.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] != KER_TTS_DMT)
          cElement.set_syscall(hProcUnitInfo, "#{API_TER_TSK}(#{sObjectID})")
        end
      }

      cElement.set_checkpoint(hProcUnitInfo)

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: ゼロチェックポイントを出力する処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_checkpoint_zero()
      cElement = IMCodeElement.new()

      @hTask.each{ |sObjectID, cObjectInfo|
        hProcUnitInfo = get_proc_unit_info(cObjectInfo)
        cElement.set_checkpoint_zero(hProcUnitInfo)
      }

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: 次のコンディションで起動するACTIVATEな非タスクが存在する
    #         場合，起動を待つコードを返す
    #=================================================================
    def gc_wait_non_task_activate(cPrevActivate, bGainTime)
      check_class(ProcessUnit, cPrevActivate, true)  # 前のコンディションでACTIVATEである非タスク
      check_class(Bool, bGainTime)                   # 時間を進めるか

      cElement = IMCodeElement.new()

      # 非タスクが存在する場合のみ
      if (!@cActivate.nil?())
        hActivateInfo = get_proc_unit_info(@cActivate)
        hRunningInfo = get_proc_unit_info(@cRunning)
        # 時間を止めないテストケースの場合，起動を待たない
        if (GRP_TIME_EVENT_HDL.include?(@cActivate.sObjectType) && (bGainTime == true))
        # 前のコンディションに非タスクがいない場合，タスクから起動を待つ
        elsif (cPrevActivate.nil?())
          cElement.set_checkpoint(hActivateInfo)
          cElement.wait_checkpoint(hRunningInfo)
        # 異なる非タスクがACTIVATEになった場合は，前の非タスクから起動を待つ
        elsif (cPrevActivate.sObjectID != @cActivate.sObjectID)
          cElement.set_checkpoint(hActivateInfo)
          cElement.wait_checkpoint(get_proc_unit_info(cPrevActivate))
        # 同じ非タスクでbootcntが変わっている場合は，一度タスクに戻るためタスクから待つ
        elsif (cPrevActivate.hState[TSR_PRM_BOOTCNT] != @cActivate.hState[TSR_PRM_BOOTCNT])
          cElement.set_checkpoint(hActivateInfo)
          cElement.wait_checkpoint(hRunningInfo)
        end
      end

      return cElement # [IMCodeElement]エレメント
    end
  end
end
