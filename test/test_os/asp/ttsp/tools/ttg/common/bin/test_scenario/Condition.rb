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
#  $Id: Condition.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "common/bin/Config.rb"
require "common/bin/process_unit/Task.rb"
require "common/bin/process_unit/Alarm.rb"
require "common/bin/process_unit/Cyclic.rb"
require "common/bin/process_unit/TaskExcept.rb"
require "common/bin/process_unit/IntHdr.rb"
require "common/bin/process_unit/ISR.rb"
require "common/bin/process_unit/Execption.rb"
require "common/bin/process_unit/IniRtn.rb"
require "common/bin/process_unit/TerRtn.rb"
require "common/bin/sc_object/Semaphore.rb"
require "common/bin/sc_object/Eventflag.rb"
require "common/bin/sc_object/Dataqueue.rb"
require "common/bin/sc_object/PriDataQ.rb"
require "common/bin/sc_object/Mailbox.rb"
require "common/bin/sc_object/MemPFix.rb"
require "common/bin/sc_object/Spinlock.rb"
require "common/bin/sys_state/CPUState.rb"
require "common/bin/IMCodeElement.rb"
require "ttc/bin/test_scenario/Condition.rb"
require "ttj/bin/test_scenario/Condition.rb"
require "bin/builder/fmp_builder/test_scenario/Condition.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: Condition
  # 概    要: pre_condition, post_conditionの情報を処理するクラス
  #===================================================================
  class Condition
    include CommonModule

    attr_reader :hAllObject, :sTestMain, :hTask, :cActivate, :aActivate, :cRunning, :aRunning
    attr_reader :bIsIniRtn, :bIsTerRtn, :bIsIntHdr, :bIsISR, :bIsException, :lMainTaskState

    #=================================================================
    # 概  要: コンストラクタ
    #=================================================================
    def initialize(sTestID)
      check_class(String, sTestID)        # テストID

      @hAllObject = {}                 # 生成された全オブジェクトの情報を保持するためのハッシュ
      @cConf      = Config.new()       # コンフィグの取得
      @sTestID    = sTestID
      @sTestMain  = sTestID + "_main"  # メイン関数名の生成

      @hMainTaskInfo = {               # メインタスク情報
        :id      => @sTestMain,
        :prcid   => @cConf.get_main_prcid(),
        :bootcnt => TTG_MAIN_BOOTCNT
      }

      @bIsIniRtn    = false    # 初期化ルーチンの有無
      @bIsTerRtn    = false    # 終了ルーチンの有無
      @bIsIntHdr    = false    # 割込みハンドラの有無
      @bIsISR       = false    # 割込みサービスルーチンの有無
      @bIsException = false    # CPU例外ハンドラの有無

      @bGcovAll = false  # GCOV全取得フラグ

      # FMP用
      @sMainPrcid  = @cConf.get_main_prcid() # メインプロセッサID
      nPrcNum      = @cConf.get_prc_num()    # プロセッサ数
      @aPrcid      = (1..nPrcNum).to_a()     # プロセッサループのための配列
      @bPassed     = false
      @aSpinProcID = []                      # スピンロックを取得している処理単位ID

    end

    #=================================================================
    # 概  要: 各オブジェクトを生成する
    #=================================================================
    def store_condition_info(hCondition, hObjectType)
      check_class(Hash, hCondition)  # コンディション情報
      check_class(Hash, hObjectType) # オブジェクトタイプ

      # オブジェクト生成
      aErrors = []
      aPath = get_condition_path()
      bIsPre  = (self.class() == PreCondition)
      hCondition.each{|sObjectID, hObjectInfo|
        ### T5_001: post_conditionにpre_conditionで定義されていないオブジェクトIDが存在する
        unless (hObjectType.has_key?(sObjectID))
          sErr = sprintf("T5_001: " + ERR_OBJECT_NOT_DEFINED_IN_PRE, sObjectID)
          aErrors.push(YamlError.new(sErr, aPath))
        else
          begin
            sObjectType = hObjectType[sObjectID]
            case sObjectType
            when TSR_OBJ_TASK
              cNewObject = Task.new(sObjectID, hObjectInfo, aPath, bIsPre)
            when TSR_OBJ_ALARM
              cNewObject = Alarm.new(sObjectID, hObjectInfo, aPath, bIsPre)
            when TSR_OBJ_CYCLE
              cNewObject = Cyclic.new(sObjectID, hObjectInfo, aPath, bIsPre)
            when TSR_OBJ_TASK_EXC
              cNewObject = TaskExcept.new(sObjectID, hObjectInfo, aPath, bIsPre)
            when TSR_OBJ_SEMAPHORE
              cNewObject = Semaphore.new(sObjectID, hObjectInfo, aPath, bIsPre)
            when TSR_OBJ_EVENTFLAG
              cNewObject = Eventflag.new(sObjectID, hObjectInfo, aPath, bIsPre)
            when TSR_OBJ_DATAQUEUE
              cNewObject = Dataqueue.new(sObjectID, hObjectInfo, aPath, bIsPre)
            when TSR_OBJ_P_DATAQUEUE
              cNewObject = PriDataQ.new(sObjectID, hObjectInfo, aPath, bIsPre)
            when TSR_OBJ_MAILBOX
              cNewObject = Mailbox.new(sObjectID, hObjectInfo, aPath, bIsPre)
            when TSR_OBJ_MEMORYPOOL
              cNewObject = MemPFix.new(sObjectID, hObjectInfo, aPath, bIsPre)
            when TSR_OBJ_CPU_STATE
              cNewObject = CPUState.new(sObjectID, hObjectInfo, aPath, bIsPre)
            when TSR_OBJ_SPINLOCK
              cNewObject = Spinlock.new(sObjectID, hObjectInfo, aPath, bIsPre)
            when TSR_OBJ_INTHDR
              cNewObject = IntHdr.new(sObjectID, hObjectInfo, aPath, bIsPre)
              @bIsIntHdr = true
            when TSR_OBJ_ISR
              cNewObject = ISR.new(sObjectID, hObjectInfo, aPath, bIsPre)
              @bIsISR = true
            when TSR_OBJ_EXCEPTION
              cNewObject = CPUException.new(sObjectID, hObjectInfo, aPath, bIsPre)
              @bIsException = true
            when TSR_OBJ_INIRTN
              cNewObject = IniRtn.new(sObjectID, hObjectInfo, aPath, bIsPre)
              @bIsIniRtn = true
            when TSR_OBJ_TERRTN
              cNewObject = TerRtn.new(sObjectID, hObjectInfo, aPath, bIsPre)
              @bIsTerRtn = true
            else
              abort(ERR_MSG % [__FILE__, __LINE__])
            end

            # 生成されたオブジェクトを@hAllObjectに保持
            @hAllObject[sObjectID] = cNewObject
          # オブジェクト生成時に発生したエラーをまとめる
          rescue TTCMultiError
            aErrors.concat($!.aErrors)
          end
        end
      }

      check_error(aErrors)
    end

    #=================================================================
    # 概　要: インスタンス変数初期化
    #=================================================================
    def init_instance_variables()
      @hTask       = get_objects_by_type(TSR_OBJ_TASK)   # タスクのリスト作成
      @hWaitObject = get_objects_by_type(GRP_SC_OBJECT)  # 待ちオブジェクトのリスト作成

      # タスクとタスク例外の関連付け
      @hAllObject.each{|key, val|
        if (val.sObjectType == TSR_OBJ_TASK_EXC)
          sObjectID = val.hState[TSR_PRM_TASK]
          unless (@hAllObject[sObjectID].nil?())
            @hAllObject[sObjectID].set_tex(val)
            # 関連タスクの拡張情報をタスク例外クラスにも登録する(assert処理用)
            val.hState[TSR_PRM_EXINF] = @hAllObject[sObjectID].hState[TSR_PRM_EXINF]
            # 関連タスクのprcidをタスク例外クラスにも登録する(FMP)
            val.hState[TSR_PRM_PRCID] = @hAllObject[sObjectID].hState[TSR_PRM_PRCID]
          end
        end
      }

      # タスクの待ち要因属性設定
      @hTask.each{|sObjectID, cObjectInfo|
        # 待ち対象がオブジェクトの場合のみ設定
        if (cObjectInfo.has_wait_object?())
          cObjectInfo.set_task_wait(@hAllObject[cObjectInfo.hState[TSR_PRM_WOBJID]])
        end
      }

      # asp
      if (@cConf.is_asp?())
        @cCpuState = get_cpu_state()
        @cActivate = get_activate()
        @cRunning  = get_running()
      # fmp
      elsif (@cConf.is_fmp?())
        @aTask = []
        @hTask.each{ |key, val|
          @aTask.push([val.hState[TSR_PRM_PRCID], key, val])
        }

        @aCpuState = get_cpu_state_fmp()
        @aActivate = get_activate_fmp()
        @aRunning  = get_running_fmp()
        # c系はメインプロセッサIDのものを指し示すように設定
        @cCpuState = @aCpuState[@sMainPrcid]
        @cRunning  = @aRunning[@sMainPrcid]
        @cActivate = @aActivate[@sMainPrcid]
      end
    end

    #=================================================================
    # 概　要: 指定したタイプに一致するオブジェクトを取得
    #=================================================================
    def get_objects_by_type(saType)
      check_class([String, Array], saType)  # オブジェクトタイプ

      # 文字列の場合配列にする
      if (saType.is_a?(String))
        saType = saType.each_line().to_a()
      end
      # 全オブジェクト情報から該当するタイプのオブジェクトを抜き出す
      hResult = @hAllObject.reject{|sObjectID, cObjectInfo|
        !(saType.include?(cObjectInfo.sObjectType))
      }

      return hResult # [Hash]オブジェクトのハッシュ
    end

    #=================================================================
    # 概  要: オブジェクトIDで該当のオブジェクトの情報を探して返す
    #=================================================================
    def get_object_info(sObjectID)
      check_class(String, sObjectID)

      return @hAllObject[sObjectID] # [Object]オブジェクトクラス
    end

    #=================================================================
    # 概  要: 実行中の非タスクのある場合はfalseを，
    #         実行中の非タスクのない場合はtrueを返す
    #=================================================================
    def check_activate_context()
      if (@cActivate.nil?())
        return true # [Bool]非タスクがある場合
      else
        return false # [Bool]非タスクがない場合
      end
    end

    #=================================================================
    # 概  要: 実行中のタスクを返す
    #         (ACTIVATEなタスク例外が関連付いている場合はタスク例外を返す)
    #=================================================================
    def get_running()
      @hTask.each{|sObjectID, cObjectInfo|
        if ((cObjectInfo.sObjectType == TSR_OBJ_TASK) && GRP_ACTIVATE.include?(cObjectInfo.hState[TSR_PRM_STATE]))
          if (!cObjectInfo.cTex.nil?() && GRP_ACTIVATE.include?(cObjectInfo.cTex.hState[TSR_PRM_HDLSTAT]))
            return cObjectInfo.cTex # [TaskExcept]実行中のタスク例外
          else
            return cObjectInfo # [Task]実行中のタスク
          end
        end
      }

      return nil
    end

    #=================================================================
    # 概  要: 実行中の処理単位を返す
    #         アラームハンドラ > 周期ハンドラ > タスク
    #=================================================================
    def get_activate()
      @hAllObject.each{|sObjectID, cObjectInfo|
        if (GRP_NON_CONTEXT.include?(cObjectInfo.sObjectType) && GRP_ACTIVATE.include?(cObjectInfo.hState[TSR_PRM_HDLSTAT]))
          return cObjectInfo # [Object] 実行中の処理単位
        end
      }

      return nil
    end

    #=================================================================
    # 概  要: CPU_STATEを返す
    #=================================================================
    def get_cpu_state()
      @hAllObject.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.sObjectType == TSR_OBJ_CPU_STATE)
          return cObjectInfo
        end
      }

      return nil
    end

    #=================================================================
    # 概  要: 実行中の処理単位を返す
    #=================================================================
    def get_prev_actvate_running()
      if (!@cActivate.nil?())
        return @cActivate # [Object] 起動中の処理単位
      else
        return @cRunning # [Object] 実行中の処理単位
      end
    end

    #=================================================================
    # 概  要: コード生成時に必要なオブジェクト情報をまとめて返す
    #=================================================================
    def get_proc_unit_info(cFristObject = nil, cSecondObject = nil)
      check_class(ProcessUnit, cFristObject, true)
      check_class(ProcessUnit, cSecondObject, true)

      hProcUnitInfo = {}

      if (!cFristObject.nil?())
        cObjectTemp = cFristObject
      elsif (!cSecondObject.nil?())
        cObjectTemp = cSecondObject
      else
        return @hMainTaskInfo # [Hash] コード生成時に必要な処理単位情報
      end

      hProcUnitInfo.store(:id, cObjectTemp.sObjectID)
      hProcUnitInfo.store(:prcid, cObjectTemp.hState[TSR_PRM_PRCID])
      hProcUnitInfo.store(:bootcnt, cObjectTemp.hState[TSR_PRM_BOOTCNT])

      return hProcUnitInfo # [Hash] コード生成時に必要な処理単位情報
    end

    #=================================================================
    # 概  要: ディスパッチ禁止状態の場合はtrueを，
    #         そうでない場合はfalseを返す
    #=================================================================
    def check_dis_dsp()
      if (!@cCpuState.nil?() && !@cCpuState.hState[TSR_PRM_DISDSP].nil?() && (@cCpuState.hState[TSR_PRM_DISDSP] == true))
        return true # [Bool]ディスパッチ禁止の場合
      end

      return false # [Bool]ディスパッチ禁止でない場合
    end

    #=================================================================
    # 概  要: 割込み優先度マスクが全解除ではない場合はtrueを，
    #         そうでない場合はfalseを返す
    #=================================================================
    def check_chg_ipm()
      if (!@cCpuState.nil?() && !@cCpuState.hState[TSR_PRM_CHGIPM].nil?() && 
          (@cCpuState.hState[TSR_PRM_CHGIPM] != 0) && (@cCpuState.hState[TSR_PRM_CHGIPM] != KER_TIPM_ENAALL))
        return true # [Bool]割込み優先度マスクが全解除ではない場合
      end

      return false # [Bool]割込み優先度マスクが全解除の場合
    end

    #=================================================================
    # 概  要: CPUロック状態の場合はtrueを，そうでない場合はfalseを返す
    #=================================================================
    def check_cpu_loc()
      unless (@cCpuState.nil?() || @cCpuState.hState[TSR_PRM_LOCCPU].nil?() || (@cCpuState.hState[TSR_PRM_LOCCPU] != true))
        return true # [Bool] CPUロック状態の場合
      end

      return false # [Bool] CPUロック状態ではない場合
    end

    #=================================================================
    # 概  要: 現在実行状態のタスクが過渡状態の場合はtrueを，
    #         そうでない場合はfalseを返す
    #=================================================================
    def check_run_sus()
      if (!@cRunning.nil?())
        if (((@cRunning.sObjectType == TSR_OBJ_TASK) && (@cRunning.hState[TSR_PRM_STATE] == KER_TTS_RUS)) ||
            ((@cRunning.sObjectType == TSR_OBJ_TASK_EXC) && (get_object_info(@cRunning.hState[TSR_PRM_TASK]).hState[TSR_PRM_STATE] == KER_TTS_RUS)))
          return true # [Bool] 現在実行状態のタスクが過渡状態の場合
        end
      end

      return false # [Bool] 現在実行状態のタスクが過渡状態ではない場合
    end

    #=================================================================
    # 概  要: 現在実行状態のタスクが指定した優先度の場合はtrueを，
    #         そうでない場合はfalseを返す
    #=================================================================
    def check_running_pri(nTaskPri)
      check_class(Integer, nTaskPri)  # チェックする優先度

      if (!@cRunning.nil?())
        # タスクの場合
        if ((@cRunning.sObjectType == TSR_OBJ_TASK) && (@cRunning.hState[TSR_PRM_TSKPRI] == nTaskPri))
          return true # [Bool] 現在実行状態のタスクが指定した優先度の場合

        # タスク例外の場合
        elsif (@cRunning.sObjectType == TSR_OBJ_TASK_EXC)
          cObjectInfo = get_object_info(@cRunning.hState[TSR_PRM_TASK])
          if (cObjectInfo.hState[TSR_PRM_TSKPRI] == nTaskPri)
            return true # [Bool] 現在実行状態のタスクが指定した優先度の場合
          end
        end
      end

      return false # [Bool] 現在実行状態のタスクが指定した優先度ではない場合
    end

    #=================================================================
    # 概  要: 動作状態（Txxx_STA）のタイムイベントハンドラがいる場合は
    #         trueを，そうでない場合はfalseを返す
    #=================================================================
    def exist_time_event_sta()
      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((GRP_TIME_EVENT_HDL.include?(cObjectInfo.sObjectType) == true) &&
            (GRP_TIME_EVENT_STA.include?(cObjectInfo.hState[TSR_PRM_STATE]) == true))
          return true # [Bool] 動作状態（Txxx_STA）のタイムイベントハンドラがいる場合
        end
      }

      return false # [Bool] 動作状態（Txxx_STA）のタイムイベントハンドラがいない場合
    end

    #=================================================================
    # 概  要: 許可状態の割込みハンドラ，割込みサービスルーチンがいる
    #         場合はtrueを，そうでない場合はfalseを返す
    #=================================================================
    def exist_interrupt_ena()
      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((GRP_INTERRUPT.include?(cObjectInfo.sObjectType) == true) && (cObjectInfo.hState[TSR_PRM_STATE] == KER_TA_ENAINT))
          return true # [Bool] 許可状態の割込みハンドラ，割込みサービスルーチンがいる場合
        end
      }

      return false # [Bool] 許可状態の割込みハンドラ，割込みサービスルーチンがいない場合
    end

    #=================================================================
    # 概  要: 実行状態のタスクを検査してある場合はtrueを，
    #         ない場合はfalseを返す
    #=================================================================
    def exist_task_running()
      @hTask.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RUN)
          return true # [Bool] 実行状態のタスクを検査してある場合
        end
      }

      return false # [Bool] 実行状態のタスクを検査してない場合
    end

    #=================================================================
    # 概  要: 起動中の非タスクのある場合はtrueを，
    #         ない場合はfalseを返す
    #=================================================================
    def exist_nontask_activate()
      if (!@cActivate.nil?())
        return true # [Bool]起動中の非タスクのある場合
      else
        return false # [Bool]起動中の非タスクのない場合
      end
    end

    #=================================================================
    # 概  要: 起動・起床要求キューイングの設定をしている場合はtrueを，
    #         そうでない場合はfalseを返す
    #=================================================================
    def exist_task_queueing()
      @hTask.each{|sObjectID, cObjectInfo|
        if (!cObjectInfo.hState[TSR_PRM_ACTCNT].nil?() && cObjectInfo.hState[TSR_PRM_ACTCNT] >= 1 || !cObjectInfo.hState[TSR_PRM_WUPCNT].nil?() && cObjectInfo.hState[TSR_PRM_WUPCNT] >= 1)
          return true # [Bool] 起動・起床要求キューイングの設定をしている場合
        end
      }

      return false # [Bool] 起動・起床要求キューイングの設定をしていない場合

    end

    #=================================================================
    # 概  要: 時間を進める処理をcElementに格納する
    #=================================================================
    def gc_tick_gain(cElement)
      check_class(IMCodeElement, cElement) # エレメント

      hProcUnitInfo = get_proc_unit_info(@cRunning)

      # 時間を進める前にチェックポイント
      cElement.set_checkpoint(hProcUnitInfo)
      # ASPではシステム時刻更新確認関数を使用する
      cElement.set_code(hProcUnitInfo, "#{FNC_TARGET_GAIN_TICK}()")
    end

    #=================================================================
    # 概  要: 全オブジェクトのref処理をIMCodeElementクラスにまとめて返
    #         す
    #=================================================================
    def gc_obj_ref()
      cElement      = IMCodeElement.new()
      hProcUnitInfo = get_proc_unit_info(@cActivate, @cRunning)

      # コメント出力
      if (@nSeqNum.nil?() && @nTimeTick.nil?())
        cElement.set_comment(hProcUnitInfo, "#{TSR_LBL_PRE}")
      else
        cElement.set_comment(hProcUnitInfo, "#{TSR_UNS_POST}#{@nSeqNum}_#{@nTimeTick}")
      end

      # タスク例外がACTIAVTEだった場合，texptn/exinfが引数と一致していることを確認するコードを入れる
      if (@cActivate.nil?() && !@cRunning.nil?() && @cRunning.sObjectType == TSR_OBJ_TASK_EXC)
        @cRunning.gc_assert_texptn_exinf(cElement, hProcUnitInfo)
      end

      # 割込みサービスルーチンがACTIAVTEだった場合，exinfが引数と一致していることを確認するコードを入れる
      if (!@cActivate.nil?() && @cActivate.sObjectType == TSR_OBJ_ISR)
        @cActivate.gc_assert_exinf(cElement, hProcUnitInfo)
      end

      # 実行中の処理単位に値が指定された変数があれば，一致していることを確認するコードを入れる
      if (!@cActivate.nil?() && !@cActivate.hState[TSR_PRM_VAR].nil?())
        @cActivate.gc_assert_value(cElement, hProcUnitInfo)
      elsif (!@cRunning.nil?() && !@cRunning.hState[TSR_PRM_VAR].nil?())
        @cRunning.gc_assert_value(cElement, hProcUnitInfo)
      end

      # CPU_STATEがcpu_lockを持っているかを参照
      bCpuLock = false

      if (!@cCpuState.nil?() && (@cCpuState.hState[TSR_PRM_LOCCPU] == true))
        bCpuLock = true
      end

      # 全オブジェクトのref_codeをまとめる
      @hAllObject.each{|sObjectID, cObjectInfo|
        case cObjectInfo.sObjectType
        when TSR_OBJ_CPU_STATE
          cObjectInfo.gc_obj_ref(cElement, hProcUnitInfo)

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

      cElement.set_checkpoint(hProcUnitInfo)

      return cElement # [IMCodeElement]エレメント
    end

    #=================================================================
    # 概  要: 指定したタイプの全オブジェクトの情報を返す
    #=================================================================
    def get_all_object_info(sObjectType)
      check_class(String, sObjectType) # 対象とするオブジェクトタイプ

      hObjectInfo = Hash.new()
      # 該当するオブジェクトの情報を格納する
      @hAllObject.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.sObjectType == sObjectType)
          hObjectInfo[sObjectID] = cObjectInfo
        end
      }

      return hObjectInfo # [Hash]オブジェクト情報ハッシュ
    end

    #=================================================================
    # 概  要: ACTIVATEである指定した処理単位のIDを返す
    #=================================================================
    def get_activate_proc_unit_id(sObjectType)
      check_class(String, sObjectType) # 対象とするオブジェクトタイプ

      aProcUnitID = Array.new()

      # ACTIVATEである処理単位の情報を格納する
      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((cObjectInfo.sObjectType == sObjectType) &&
            (cObjectInfo.hState[TSR_PRM_HDLSTAT] == TSR_STT_ACTIVATE))
          aProcUnitID.push(sObjectID)
        end
      }

      return aProcUnitID # [Array]ACTIVATEである処理単位ID配列
    end

    #=================================================================
    # 概  要: GCOV全取得フラグを設定する
    #=================================================================
    def set_gcov_all_flg(bFlg)
      check_class(Bool, bFlg) # 設定するGCOV全取得フラグ

      @bGcovAll = bFlg
    end

  end
end
