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
#  $Id: PreCondition.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/test_scenario/Condition.rb"
require "ttc/bin/test_scenario/PreCondition.rb"
require "ttj/bin/test_scenario/PreCondition.rb"
require "bin/builder/fmp_builder/test_scenario/PreCondition.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: PreCondition
  # 概    要: pre_conditionの情報を処理するクラス
  #===================================================================
  class PreCondition < Condition
    include CommonModule

    attr_reader :hObjectType

    #=================================================================
    # 概  要: pre_conditionの初期化
    #=================================================================
    def initialize(sTestID, hScenarioPre)
      check_class(String, sTestID)           # テストID
      check_class(Hash, hScenarioPre, true)  # pre_condition

      super(sTestID)
      @hObjectType = {}  # オブジェクトIDとタイプの対応付け

      # 構造チェック
      basic_check(hScenarioPre)

      # オブジェクトIDとオブジェクトタイプの対応表作成
      hScenarioPre.each{|sObjectID, hObjectInfo|
        @hObjectType[sObjectID] = hObjectInfo[TSR_PRM_TYPE]
      }
      # オブジェクト情報格納
      store_condition_info(hScenarioPre, @hObjectType)

      # スタック番号の格納
      if (@cConf.is_stack_share_mode?())
        nStackNum = 1
        hObjects = get_objects_by_type(TSR_OBJ_TASK)
        hObjects.each{ |sObjectID, cObjectInfo|
          cObjectInfo.nStackNum = nStackNum
          nStackNum += 1
        }
      end
    end

    #=================================================================
    # 概  要: pre_conditionで進む時間の設定
    #=================================================================
    def set_pre_gain_tick()
      @nPreGainTick = 0

      # pre_conditionでACTIVATEなタイムイベントハンドラを進めるために必要な時間を取得
      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((GRP_TIME_EVENT_HDL.include?(cObjectInfo.sObjectType) == true) && (cObjectInfo.hState[TSR_PRM_HDLSTAT] == TSR_STT_ACTIVATE))
          if (cObjectInfo.sObjectType == TSR_OBJ_ALARM)
            if (@nPreGainTick == 0)
              @nPreGainTick = 1
            end
          elsif (cObjectInfo.sObjectType == TSR_OBJ_CYCLE)
            if ((cObjectInfo.hState[STR_CYCPHS].to_i + 1) > @nPreGainTick)
              @nPreGainTick = cObjectInfo.hState[STR_CYCPHS].to_i + 1
            end
          end
        end
      }
    end

    #=================================================================
    # 概  要: 対象タスクを起動して，メインタスクの優先度を下げる処理を
    #         cElementに格納する
    #=================================================================
    def set_act_task(cElement, cObjectInfo)
      check_class(IMCodeElement, cElement)  # エレメント
      check_class(ProcessUnit, cObjectInfo) # 対象タスククラス

      hMainTaskInfo = get_proc_unit_info()

      # 対象タスクを起動
      cElement.set_syscall(hMainTaskInfo, "#{API_ACT_TSK}(#{cObjectInfo.sObjectID})")

      # メインタスクの優先度を下げてディスパッチ
      cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{TTG_TSK_SELF}, #{TTG_WAIT_PRI})")

      # 対象タスクの優先度がTTG_WAIT_PRIと同じだった場合，rot_rdqを発行してディスパッチさせる
      if (cObjectInfo.hState[TSR_PRM_TSKPRI] == TTG_WAIT_PRI)
        cElement.set_syscall(hMainTaskInfo, "#{API_ROT_RDQ}(#{TTG_WAIT_PRI})")
      end
    end
    private :set_act_task

    #=================================================================
    # 概  要: メインタスクの優先度を上げる処理を
    #         cElementに格納する
    #=================================================================
    def set_chg_pri(cElement)
      check_class(IMCodeElement, cElement) # エレメント
      hMainTaskInfo = get_proc_unit_info()

      # メインタスクを元の優先度へ戻す
      cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{TTG_TSK_SELF}, #{TTG_MAIN_PRI})")
    end
    private :set_chg_pri

    #=================================================================
    # 概  要: タスク例外処理許可状態タスクがいる場合はtrueを，
    #         そうでない場合はfalseを返す
    #=================================================================
    def exist_pre_task_tex_ena()
      @hTask.each{|sObjectID, cObjectInfo|
        if (!cObjectInfo.cTex.nil?() && cObjectInfo.cTex.hState[TSR_PRM_STATE] == TSR_STT_TTEX_ENA)
          return true # [Bool]タスク例外処理許可状態タスクがいる場合
        end
      }

      return false # [Bool]タスク例外処理許可状態タスクがいない場合
    end

    #=================================================================
    # 概  要: ACTIVATEであるタスク例外ハンドラがいる場合はtrueを，
    #         そうでない場合はfalseを返す
    #=================================================================
    def exist_pre_task_texhdr_activate()
      @hTask.each{|sObjectID, cObjectInfo|
        if (!cObjectInfo.cTex.nil?() && cObjectInfo.cTex.hState[TSR_PRM_HDLSTAT] == TSR_STT_ACTIVATE)
          return true # [Bool]ACTIVATEであるタスク例外ハンドラがいる場合
        end
      }

      return false # [Bool]ACTIVATEであるタスク例外ハンドラがいない場合
    end

    #=================================================================
    # 概  要: 初期状態の設定処理の必要な同期・通信オブジェクトを
    #         検査してある場合はtrueを，ない場合はfalseを返す
    #=================================================================
    def exist_pre_scobj_data()
      @hWaitObject.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_ISEMCNT] != cObjectInfo.hState[TSR_PRM_SEMCNT]) ||
           (cObjectInfo.hState[TSR_PRM_IFLGPTN] != cObjectInfo.hState[TSR_PRM_FLGPTN]) ||
           !(cObjectInfo.hState[TSR_PRM_DATALIST].nil?() || cObjectInfo.hState[TSR_PRM_DATALIST].empty?()) ||
           !(cObjectInfo.hState[TSR_PRM_MSGLIST].nil?() || cObjectInfo.hState[TSR_PRM_MSGLIST].empty?()) ||
           (cObjectInfo.hState[TSR_PRM_BLKCNT] != cObjectInfo.hState[TSR_PRM_FBLKCNT])
          return true # [Bool]検査してある場合
        end
      }

      return false # [Bool]検査してない場合
    end

    #=================================================================
    # 概  要: 同期・通信オブジェクトによる待ち状態のタスクがあれば
    #         trueを，なければfalseを返す
    #=================================================================
    def exist_pre_task_scobj_waiting()
      @hTask.each{|sObjectID, cObjectInfo|
        if (!cObjectInfo.hState[TSR_PRM_WOBJID].nil?() && !GRP_WAIT_NON_OBJECT.include?(cObjectInfo.hState[TSR_PRM_WOBJID]))
          return true # [Bool]同期・通信オブジェクトによる待ち状態のタスクがある場合
        end
      }

      return false # [Bool]同期・通信オブジェクトによる待ち状態のタスクがない場合
    end

    #=================================================================
    # 概  要: 待ち状態(スリープ，ディレイ)のタスクを検査してある場合は
    #         trueを，ない場合はfalseを返す
    #=================================================================
    def exist_pre_task_waiting()
      @hTask.each{|sObjectID, cObjectInfo|
        if (!cObjectInfo.hState[TSR_PRM_WOBJID].nil?() && GRP_WAIT_NON_OBJECT.include?(cObjectInfo.hState[TSR_PRM_WOBJID]))
          return true # [Bool]検査してある場合
        end
      }

      return false # [Bool]検査してない場合
    end

    #=================================================================
    # 概  要: 強制待ち状態のタスクを検査してある場合はtrueを，
    #         ない場合はfalseを返す
    #=================================================================
    def exist_pre_task_suspended()
      @hTask.each{|sObjectID, cObjectInfo|
        if (GRP_SUSPENDED.include?(cObjectInfo.hState[TSR_PRM_STATE]))
          return true # [Bool]検査してある場合
        end
      }

      return false # [Bool]検査してない場合
    end

    #=================================================================
    # 概  要: レディー状態のタスクを検査してある場合はtrueを，
    #         ない場合はfalseを返す
    #=================================================================
    def exist_pre_task_ready()
      @hTask.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RDY)
          return true # [Bool]検査してある場合
        end
      }

      return false # [Bool]検査してない場合
    end

    #=================================================================
    # 概  要: 現在優先度と初期優先度が異なるタスクを検査してある場合は
    #         trueを，ない場合はfalseを返す
    #=================================================================
    def exist_pre_task_pri_chg()
      @hTask.each{|sObjectID, cObjectInfo|
        if ((cObjectInfo.hState[TSR_PRM_STATE] != KER_TTS_RUN) &&
            (cObjectInfo.hState[TSR_PRM_STATE] != KER_TTS_DMT) &&
            (cObjectInfo.hState[TSR_PRM_TSKPRI] != cObjectInfo.hState[TSR_PRM_ITSKPRI]) &&
            !(!cObjectInfo.hState[TSR_PRM_WOBJID].nil?() && !GRP_WAIT_NON_OBJECT.include?(cObjectInfo.hState[TSR_PRM_WOBJID])))
          return true # [Bool]検査してある場合
        end
      }

      return false # [Bool]検査してない場合
    end

    #=================================================================
    # 概  要: タスク保留例外要因の設定をしている場合はtrueを，
    #         そうでない場合はfalseを返す
    #=================================================================
    def exist_pre_task_tex_pndptn()
      @hTask.each{|sObjectID, cObjectInfo|
        if (!cObjectInfo.cTex.nil?() && !cObjectInfo.cTex.hState[TSR_PRM_PNDPTN].nil?() && cObjectInfo.cTex.hState[TSR_PRM_PNDPTN] != 0)
          return true # [Bool]タスク保留例外要因の設定をしている場合
        end
      }

      return false # [Bool]タスク保留例外要因の設定をしてない場合
    end

    #=================================================================
    # 概  要: グローバル/ローカル変数を宣言するためのコードを
    #         cElementに格納する
    #=================================================================
    def gc_global_local_var(cElement)
      # 変数が存在する場合，定義する
      @hAllObject.each{|sObjectID, cObjectInfo|
        if (!cObjectInfo.hState[TSR_PRM_VAR].nil?())
          cObjectInfo.hState[TSR_PRM_VAR].each{|sValName, cVariable|
            cVariable.gc_global_local_var(cElement, sObjectID)
          }
        end
      }
    end

    #=================================================================
    # 概  要: ヘッダーファイルに出力するコード作成
    #=================================================================
    def gc_header(cElement)
      @hAllObject.each{|sObjectID, cObjectInfo|
        if (GRP_PROCESS_UNIT.include?(cObjectInfo.sObjectType))
          cElement.set_header(sObjectID, cObjectInfo.sObjectType)
        end
      }
    end

    #=================================================================
    # 概  要: コンフィグファイルに出力するコード作成
    #=================================================================
    def gc_config(cElement)
      @hAllObject.each{|sObjectID, cObjectInfo|
        # CPU状態は必要なし
        # タスク例外はタスクとの順序依存があるため，タスク内でコードを生成する
        # 初期化ルーチン，終了ルーチンは他のコードとまとめて生成する
        # 割込みハンドラ，CPU例外は共有関数のため別途定義する
        if ((cObjectInfo.sObjectType != TSR_OBJ_CPU_STATE) &&
            (cObjectInfo.sObjectType != TSR_OBJ_TASK_EXC) &&
            (cObjectInfo.sObjectType != TSR_OBJ_INIRTN) &&
            (cObjectInfo.sObjectType != TSR_OBJ_TERRTN) &&
            (cObjectInfo.sObjectType != TSR_OBJ_INTHDR) &&
            (cObjectInfo.sObjectType != TSR_OBJ_EXCEPTION))
          cObjectInfo.gc_config(cElement)
        end
      }
    end

    #=================================================================
    # 概  要: タスク例外処理許可状態にするコードを返す
    #=================================================================
    def gc_pre_task_tex_ena()
      cElement = IMCodeElement.new()

      @hTask.each{|sObjectID, cObjectInfo|
        if (!cObjectInfo.cTex.nil?() && cObjectInfo.cTex.hState[TSR_PRM_STATE] == TSR_STT_TTEX_ENA)
          cElement.set_syscall(get_proc_unit_info(get_object_info(sObjectID)), "#{API_ENA_TEX}()")
        end
      }

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: タスク例外処理を起こすコードを返す
    #=================================================================
    def gc_pre_task_texhdr_activate()
      cElement     = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()
      hRunningTask  = get_proc_unit_info(get_object_info(@cRunning.hState[TSR_PRM_TASK]))
      hActivateTex  = get_proc_unit_info(@cRunning)

      # 実行状態のタスクを起動する
      cElement.set_syscall(hMainTaskInfo, "#{API_ACT_TSK}(#{@cRunning.hState[TSR_PRM_TASK]})")

      # 実行状態のタスクがタスク例外を起動する
      cElement.set_syscall(hRunningTask, "#{API_RAS_TEX}(#{TTG_TSK_SELF}, #{@cRunning.hState[TSR_PRM_TEXPTN]})")

      # タスク例外許可状態がTTEX_DISの場合，ena_texしてACTIVATEにする
      if (@cRunning.hState[TSR_PRM_STATE] == TSR_STT_TTEX_DIS)
        cElement.set_syscall(hRunningTask, "#{API_ENA_TEX}()")
      # タスク例外許可状態がTTEX_ENAの場合，既にena_texしてあるが，タスク例外起動後に再度ena_texする
      else
        cElement.set_syscall(hActivateTex, "#{API_ENA_TEX}()")
      end

      # 保留例外要因パターンが0でない場合，自分でras_texする
      if (@cRunning.hState[TSR_PRM_PNDPTN] != 0)
        cElement.set_syscall(hActivateTex, "#{API_RAS_TEX}(#{TTG_TSK_SELF}, #{@cRunning.hState[TSR_PRM_PNDPTN]})")
      end

      cElement.set_checkpoint(hMainTaskInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 同期・通信オブジェクトの初期状態の設定処理を
    #         IMCodeElementクラスにまとめて返す(メイン関数の処理部分)
    #=================================================================
    def gc_pre_scobj_data()
      # メインタスクからの適切な処理により，
      # 待ちオブジェクトの初期状態を設定する
      cElement      = IMCodeElement.new()
      hProcUnitInfo = get_proc_unit_info()

      @hWaitObject.each{|sObjectID, cObjectInfo|
        # SEMAPHORE
        if (cObjectInfo.hState[TSR_PRM_ISEMCNT] != cObjectInfo.hState[TSR_PRM_SEMCNT])
          nTimes = cObjectInfo.hState[TSR_PRM_ISEMCNT] - cObjectInfo.hState[TSR_PRM_SEMCNT]

          if (nTimes > 0)
            nTimes.downto(1){
              cElement.set_syscall(hProcUnitInfo, "#{API_WAI_SEM}(#{cObjectInfo.sObjectID})")
            }
          else
            nTimes.upto(-1){
              cElement.set_syscall(hProcUnitInfo, "#{API_SIG_SEM}(#{cObjectInfo.sObjectID})")
            }
          end
        end

        # EVENTFLAG
        if (cObjectInfo.hState[TSR_PRM_IFLGPTN] != cObjectInfo.hState[TSR_PRM_FLGPTN])
          # 初期ビットパターンが0でない場合に備えて，初期ビットパターンのフラグをクリアしておく
          cElement.set_syscall(hProcUnitInfo, "#{API_CLR_FLG}(#{cObjectInfo.sObjectID}, ~#{cObjectInfo.hState[TSR_PRM_IFLGPTN]})")
          cElement.set_syscall(hProcUnitInfo, "#{API_SET_FLG}(#{cObjectInfo.sObjectID}, #{cObjectInfo.hState[TSR_PRM_FLGPTN]})")
        end

        # DATAQUEUE，P_DATAQUEUE
        if !(cObjectInfo.hState[TSR_PRM_DATALIST].nil?() || cObjectInfo.hState[TSR_PRM_DATALIST].empty?())
          cObjectInfo.hState[TSR_PRM_DATALIST].each{|hData|
            case cObjectInfo.sObjectType
            when TSR_OBJ_DATAQUEUE
              cElement.set_syscall(hProcUnitInfo, "#{API_SND_DTQ}(#{cObjectInfo.sObjectID}, #{hData[TSR_VAR_DATA]})")

            when TSR_OBJ_P_DATAQUEUE
              cElement.set_syscall(hProcUnitInfo, "#{API_SND_PDQ}(#{cObjectInfo.sObjectID}, #{hData[TSR_VAR_DATA]}, #{hData[TSR_VAR_DATAPRI]})")

            end
          }
        end

        # MAILBOX
        if !(cObjectInfo.hState[TSR_PRM_MSGLIST].nil?() || cObjectInfo.hState[TSR_PRM_MSGLIST].empty?())
          cObjectInfo.hState[TSR_PRM_MSGLIST].each{|hData|
            if (hData.has_key?(TSR_VAR_MSGPRI))
              cElement.set_global_var(hData[TSR_VAR_MSG], TYP_T_MSG_PRI)
              cElement.set_code(hProcUnitInfo, "#{hData[TSR_VAR_MSG]}.#{STR_MSGPRI} = #{hData[TSR_VAR_MSGPRI]}")
              cElement.set_syscall(hProcUnitInfo, "#{API_SND_MBX}(#{cObjectInfo.sObjectID}, #{CST_MSG1}&#{hData[TSR_VAR_MSG]})")
            else
              cElement.set_global_var(hData[TSR_VAR_MSG], TYP_T_MSG)
              cElement.set_syscall(hProcUnitInfo, "#{API_SND_MBX}(#{cObjectInfo.sObjectID}, &#{hData[TSR_VAR_MSG]})")
            end
          }
        end

        # MEMORYPOOL
        if (cObjectInfo.hState[TSR_PRM_BLKCNT] != cObjectInfo.hState[TSR_PRM_FBLKCNT])
          nTimes = cObjectInfo.hState[TSR_PRM_BLKCNT] - cObjectInfo.hState[TSR_PRM_FBLKCNT]

          (1..nTimes).each{|nCnt|
            cElement.set_local_var(hProcUnitInfo[:id], "#{VAR_BLK}_#{nCnt}", TYP_VOID_P)
            cElement.set_syscall(hProcUnitInfo, "#{API_GET_MPF}(#{cObjectInfo.sObjectID}, &#{VAR_BLK}_#{nCnt})")
          }
        end
      }

      cElement.set_checkpoint(hProcUnitInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 同期・通信オブジェクトによる待ち状態タスクの初期状態の
    #         設定処理をIMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_scobj_waiting()
      # タスクをオブジェクト待ちにするコードブロックを返す
      # 例）ASP_dataqueue_rcv_dtq_f_2_1_1
      #   メインタスクはTASK2を起こし，タスク例外ハンドラが存在する場合は
      #   該当の設定をし，TASK2を自分でsnd_dtqさせ，
      #   送信待ちにさせる(これを送信待ちになっているタスクの数だけ)
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      @hWaitObject.each{|sObjectID, cObjectInfo|
        # 待ちタスクのリスト
        if (!cObjectInfo.hState[TSR_PRM_WTSKLIST].nil?() && !cObjectInfo.hState[TSR_PRM_WTSKLIST].empty?())
          cObjectInfo.hState[TSR_PRM_WTSKLIST].each{|aWtskList|
            aWtskList.each_key{|sTaskID|
              cTaskInfo = @hTask[sTaskID]

              # 対象タスクへディスパッチするためのコード
              set_act_task(cElement, cTaskInfo)

              # 対象タスクの情報作成，チェックポイント設定
              hProcUnitInfo = get_proc_unit_info(cTaskInfo)
              cElement.set_checkpoint(hProcUnitInfo)

              case cObjectInfo.sObjectType
              when TSR_OBJ_SEMAPHORE
                # 時間指定があればタイムアウト付きAPIを使用する
                if (cTaskInfo.hState[TSR_PRM_LEFTTMO].nil?())
                  cElement.set_syscall(hProcUnitInfo, "#{API_WAI_SEM}(#{sObjectID})", nil)
                else
                  cElement.set_syscall(hProcUnitInfo, "#{API_TWAI_SEM}(#{sObjectID}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                end

              when TSR_OBJ_EVENTFLAG
                # 時間指定があればタイムアウト付きAPIを使用する
                if (cTaskInfo.hState[TSR_PRM_LEFTTMO].nil?())
                  # 変数が指定されていれば使用する
                  if (aWtskList[sTaskID][TSR_PRM_VAR] != nil)
                    cElement.set_syscall(hProcUnitInfo, "#{API_WAI_FLG}(#{sObjectID}, #{aWtskList[sTaskID][TSR_VAR_WAIPTN]}, #{aWtskList[sTaskID][TSR_VAR_WFMODE]}, &#{aWtskList[sTaskID][TSR_PRM_VAR]})", nil)
                  else
                    cElement.set_local_var(sTaskID, VAR_FLGPTN, TYP_FLGPTN)
                    cElement.set_syscall(hProcUnitInfo, "#{API_WAI_FLG}(#{sObjectID}, #{aWtskList[sTaskID][TSR_VAR_WAIPTN]}, #{aWtskList[sTaskID][TSR_VAR_WFMODE]}, &#{VAR_FLGPTN})", nil)
                  end
                else
                  # 変数が指定されていれば使用する
                  if (aWtskList[sTaskID][TSR_PRM_VAR] != nil)
                    cElement.set_syscall(hProcUnitInfo, "#{API_TWAI_FLG}(#{sObjectID}, #{aWtskList[sTaskID][TSR_VAR_WAIPTN]}, #{aWtskList[sTaskID][TSR_VAR_WFMODE]}, &#{aWtskList[sTaskID][TSR_PRM_VAR]}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                  else
                    cElement.set_local_var(sTaskID, VAR_FLGPTN, TYP_FLGPTN)
                    cElement.set_syscall(hProcUnitInfo, "#{API_TWAI_FLG}(#{sObjectID}, #{aWtskList[sTaskID][TSR_VAR_WAIPTN]}, #{aWtskList[sTaskID][TSR_VAR_WFMODE]}, &#{VAR_FLGPTN}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                  end
                end

              when TSR_OBJ_MAILBOX
                # 時間指定があればタイムアウト付きAPIを使用する
                if (cTaskInfo.hState[TSR_PRM_LEFTTMO].nil?())
                  # 変数が指定されていれば使用する
                  if (!aWtskList[sTaskID].nil?())
                    cElement.set_syscall(hProcUnitInfo, "#{API_RCV_MBX}(#{sObjectID}, &#{aWtskList[sTaskID][TSR_PRM_VAR]})", nil)
                  else
                    cElement.set_local_var(sTaskID, VAR_P_MSG, TYP_T_P_MSG)
                    cElement.set_syscall(hProcUnitInfo, "#{API_RCV_MBX}(#{sObjectID}, &#{VAR_P_MSG})", nil)
                  end
                else
                  # 変数が指定されていれば使用する
                  if (!aWtskList[sTaskID].nil?())
                    cElement.set_syscall(hProcUnitInfo, "#{API_TRCV_MBX}(#{sObjectID}, &#{aWtskList[sTaskID][TSR_PRM_VAR]}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                  else
                    cElement.set_local_var(sTaskID, VAR_P_MSG, TYP_T_P_MSG)
                    cElement.set_syscall(hProcUnitInfo, "#{API_TRCV_MBX}(#{sObjectID}, &#{VAR_P_MSG}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                  end
                end

              when TSR_OBJ_MEMORYPOOL
                # 時間指定があればタイムアウト付きAPIを使用する
                if (cTaskInfo.hState[TSR_PRM_LEFTTMO].nil?())
                  # 変数が指定されていれば使用する
                  if (!aWtskList[sTaskID].nil?())
                    cElement.set_syscall(hProcUnitInfo, "#{API_GET_MPF}(#{sObjectID}, &#{aWtskList[sTaskID][TSR_PRM_VAR]})", nil)
                  else
                    cElement.set_local_var(sTaskID, VAR_BLK, TYP_VOID_P)
                    cElement.set_syscall(hProcUnitInfo, "#{API_GET_MPF}(#{sObjectID}, &#{VAR_BLK})", nil)
                  end
                else
                  # 変数が指定されていれば使用する
                  if (!aWtskList[sTaskID].nil?())
                    cElement.set_syscall(hProcUnitInfo, "#{API_TGET_MPF}(#{sObjectID}, &#{aWtskList[sTaskID][TSR_PRM_VAR]}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                  else
                    cElement.set_local_var(sTaskID, VAR_BLK, TYP_VOID_P)
                    cElement.set_syscall(hProcUnitInfo, "#{API_TGET_MPF}(#{sObjectID}, &#{VAR_BLK}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                  end
                end

              end

              # メインタスクの優先度を戻すためのコード
              set_chg_pri(cElement)

              # 対象タスクの現在優先度と初期優先度が異なる場合ここでchg_priする
              # (同期通信オブジェクトの属性が優先度順だった場合に，待ち順でないと順序が変わる可能性がある)
              if (cTaskInfo.hState[TSR_PRM_TSKPRI] != cTaskInfo.hState[TSR_PRM_ITSKPRI])
                cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{cTaskInfo.sObjectID}, #{cTaskInfo.hState[TSR_PRM_TSKPRI]})")
              end
            }
          }
        end

        # 送信待ちタスクのリスト
        if (!cObjectInfo.hState[TSR_PRM_STSKLIST].nil?() && !cObjectInfo.hState[TSR_PRM_STSKLIST].empty?())
          cObjectInfo.hState[TSR_PRM_STSKLIST].each{|aStskList|
            aStskList.each{|sTaskID, hData|
              cTaskInfo = @hTask[sTaskID]

              # 対象タスクへディスパッチするためのコード
              set_act_task(cElement, cTaskInfo)

              # 対象タスクの情報作成，チェックポイント設定
              hProcUnitInfo = get_proc_unit_info(cTaskInfo)
              cElement.set_checkpoint(hProcUnitInfo)

              case cObjectInfo.sObjectType
              when TSR_OBJ_DATAQUEUE
                # 時間指定があればタイムアウト付きAPIを使用する
                if (cTaskInfo.hState[TSR_PRM_LEFTTMO].nil?())
                  cElement.set_syscall(hProcUnitInfo, "#{API_SND_DTQ}(#{sObjectID}, #{hData[hData.keys[0]]})", nil)
                else
                  cElement.set_syscall(hProcUnitInfo, "#{API_TSND_DTQ}(#{sObjectID}, #{hData[hData.keys[0]]}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                end

              when TSR_OBJ_P_DATAQUEUE
                # 時間指定があればタイムアウト付きAPIを使用する
                if (cTaskInfo.hState[TSR_PRM_LEFTTMO].nil?())
                  cElement.set_syscall(hProcUnitInfo, "#{API_SND_PDQ}(#{sObjectID}, #{hData[TSR_VAR_DATA]}, #{hData[TSR_VAR_DATAPRI]})", nil)
                else
                  cElement.set_syscall(hProcUnitInfo, "#{API_TSND_PDQ}(#{sObjectID}, #{hData[TSR_VAR_DATA]}, #{hData[TSR_VAR_DATAPRI]}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                end

              end

              # メインタスクの優先度を戻すためのコード
              set_chg_pri(cElement)

              # 対象タスクの現在優先度と初期優先度が異なる場合ここでchg_priする
              # (同期通信オブジェクトの属性が優先度順だった場合に，待ち順でないと順序が変わる可能性がある)
              if (cTaskInfo.hState[TSR_PRM_TSKPRI] != cTaskInfo.hState[TSR_PRM_ITSKPRI])
                cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{cTaskInfo.sObjectID}, #{cTaskInfo.hState[TSR_PRM_TSKPRI]})")
              end
            }
          }
        end

        # 受信待ちタスクのリスト
        if (!cObjectInfo.hState[TSR_PRM_RTSKLIST].nil?() && !cObjectInfo.hState[TSR_PRM_RTSKLIST].empty?())
          cObjectInfo.hState[TSR_PRM_RTSKLIST].each{|aRtskList|
            aRtskList.each_key{|sTaskID|
              cTaskInfo = @hTask[sTaskID]

              # 対象タスクへディスパッチするためのコード
              set_act_task(cElement, cTaskInfo)

              # 対象タスクの情報作成，チェックポイント設定
              hProcUnitInfo = get_proc_unit_info(cTaskInfo)
              cElement.set_checkpoint(hProcUnitInfo)

              case cObjectInfo.sObjectType
              when TSR_OBJ_DATAQUEUE
                # 時間指定があればタイムアウト付きAPIを使用する
                if (cTaskInfo.hState[TSR_PRM_LEFTTMO].nil?())
                  # 変数が指定されていれば使用する
                  if (!aRtskList[sTaskID].nil?())
                    cElement.set_syscall(hProcUnitInfo, "#{API_RCV_DTQ}(#{sObjectID}, &#{aRtskList[sTaskID][TSR_PRM_VAR]})", nil)
                  else
                    cElement.set_local_var(sTaskID, VAR_DATA, TYP_INTPTR_T)
                    cElement.set_syscall(hProcUnitInfo, "#{API_RCV_DTQ}(#{sObjectID}, &#{VAR_DATA})", nil)
                  end
                else
                  if (!aRtskList[sTaskID].nil?())
                    cElement.set_syscall(hProcUnitInfo, "#{API_TRCV_DTQ}(#{sObjectID}, &#{aRtskList[sTaskID][TSR_PRM_VAR]}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                  else
                    # 変数が指定されていれば使用する
                    cElement.set_local_var(sTaskID, VAR_DATA, TYP_INTPTR_T)
                    cElement.set_syscall(hProcUnitInfo, "#{API_TRCV_DTQ}(#{sObjectID}, &#{VAR_DATA}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                  end
                end

              when TSR_OBJ_P_DATAQUEUE
                # 時間指定があればタイムアウト付きAPIを使用する
                if (cTaskInfo.hState[TSR_PRM_LEFTTMO].nil?())
                  # 変数が指定されていれば使用する
                  if (!aRtskList[sTaskID].nil?())
                    cElement.set_syscall(hProcUnitInfo, "#{API_RCV_PDQ}(#{sObjectID}, &#{aRtskList[sTaskID][TSR_VAR_VARDATA]}, &#{aRtskList[sTaskID][TSR_VAR_VARPRI]})", nil)
                  else
                    cElement.set_local_var(sTaskID, VAR_DATA, TYP_INTPTR_T)
                    cElement.set_local_var(sTaskID, VAR_DATAPRI, TYP_PRI)
                    cElement.set_syscall(hProcUnitInfo, "#{API_RCV_PDQ}(#{sObjectID}, &#{VAR_DATA}, &#{VAR_DATAPRI})", nil)
                  end
                else
                  # 変数が指定されていれば使用する
                  if (!aRtskList[sTaskID].nil?())
                    cElement.set_syscall(hProcUnitInfo, "#{API_TRCV_PDQ}(#{sObjectID}, &#{aRtskList[sTaskID][TSR_VAR_VARDATA]}, &#{aRtskList[sTaskID][TSR_VAR_VARPRI]}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                  else
                    cElement.set_local_var(sTaskID, VAR_DATA, TYP_INTPTR_T)
                    cElement.set_local_var(sTaskID, VAR_DATAPRI, TYP_PRI)
                    cElement.set_syscall(hProcUnitInfo, "#{API_TRCV_PDQ}(#{sObjectID}, &#{VAR_DATA}, &#{VAR_DATAPRI}, #{cTaskInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
                  end
                end

              end

              # メインタスクの優先度を戻すためのコード
              set_chg_pri(cElement)

              # 対象タスクの現在優先度と初期優先度が異なる場合ここでchg_priする
              # (同期通信オブジェクトの属性が優先度順だった場合に，待ち順でないと順序が変わる可能性がある)
              if (cTaskInfo.hState[TSR_PRM_TSKPRI] != cTaskInfo.hState[TSR_PRM_ITSKPRI])
                cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{cTaskInfo.sObjectID}, #{cTaskInfo.hState[TSR_PRM_TSKPRI]})")
              end
            }
          }
        end
      }

      # 待ち状態作成後，メインタスクに戻った時点でチェックポイント
      cElement.set_checkpoint(get_proc_unit_info())

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 待ち状態であるタスクの初期状態の設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_waiting()
      cElement = IMCodeElement.new()

      @hTask.each{|sObjectID, cObjectInfo|
        if (GRP_WAIT_NON_OBJECT.include?(cObjectInfo.hState[TSR_PRM_WOBJID]))
          # 対象タスクへディスパッチするためのコード
          set_act_task(cElement, cObjectInfo)

          # 対象タスクの情報作成，チェックポイント設定
          hProcUnitInfo = get_proc_unit_info(cObjectInfo)
          cElement.set_checkpoint(hProcUnitInfo)

          if (cObjectInfo.hState[TSR_PRM_WOBJID] == TSR_STT_SLEEP)
            if (cObjectInfo.hState[TSR_PRM_LEFTTMO].nil?())
              cElement.set_syscall(hProcUnitInfo, "#{API_SLP_TSK}()", nil)
            else
              cElement.set_syscall(hProcUnitInfo, "#{API_TSLP_TSK}(#{cObjectInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
            end
          elsif (cObjectInfo.hState[TSR_PRM_WOBJID] == TSR_STT_DELAY)
            cElement.set_syscall(hProcUnitInfo, "#{API_DLY_TSK}(#{cObjectInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})", nil)
          end

          # メインタスクの優先度を戻すためのコード
          set_chg_pri(cElement)
        end
      }

      # 待ち状態作成後，メインタスクに戻った時点でチェックポイント
      cElement.set_checkpoint(get_proc_unit_info())

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 強制状態であるタスクの初期状態の設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_suspended()
      # メインタスクでsus_tsk(対象タスク)処理
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      @hTask.each{|sObjectID, cObjectInfo|
        if (GRP_SUSPENDED.include?(cObjectInfo.hState[TSR_PRM_STATE]))  # TTS_SUS，TTS_WAS
          if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_SUS)
            # 対象タスクへディスパッチするためのコード
            set_act_task(cElement, cObjectInfo)

            # 対象タスクの情報作成，チェックポイント設定
            hProcUnitInfo = get_proc_unit_info(cObjectInfo)
            cElement.set_checkpoint(hProcUnitInfo)

            cElement.set_syscall(hProcUnitInfo, "#{API_SLP_TSK}()")

            # メインタスクの優先度を戻すためのコード
            set_chg_pri(cElement)
          end

          cElement.set_syscall(hMainTaskInfo, "#{API_SUS_TSK}(#{sObjectID})")

          # 強制待ちの場合，起床して完成
          if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_SUS)
            cElement.set_syscall(hMainTaskInfo, "#{API_WUP_TSK}(#{sObjectID})")
          end
        end
      }

      cElement.set_checkpoint(hMainTaskInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: レディー状態であるタスクの初期状態の設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_ready()
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      # メインタスクから起こして、自タスクでslp_tskしておく
      @hTask.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RDY)
          # 対象タスクへディスパッチするためのコード
          set_act_task(cElement, cObjectInfo)

          # 対象タスクの情報作成，チェックポイント設定
          hProcUnitInfo = get_proc_unit_info(cObjectInfo)
          cElement.set_checkpoint(hProcUnitInfo)

          cElement.set_syscall(hProcUnitInfo, "#{API_SLP_TSK}()")

          # メインタスクの優先度を戻すためのコード
          set_chg_pri(cElement)
        end
      }

      cElement.set_checkpoint(hMainTaskInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: タスク保留例外要因に設定するコードを返す
    #=================================================================
    def gc_pre_task_tex_pndptn()
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      @hTask.each{|sObjectID, cObjectInfo|
        # タスクが実行中もしくはタスク例外ハンドラがACTIVATEの場合は，自分自身でras_texする
        if (!GRP_ACTIVATE.include?(cObjectInfo.hState[TSR_PRM_STATE]) && (!cObjectInfo.cTex.nil?()) &&
            !cObjectInfo.cTex.hState[TSR_PRM_PNDPTN].nil?() && (cObjectInfo.cTex.hState[TSR_PRM_PNDPTN] != 0))
          cElement.set_syscall(hMainTaskInfo, "#{API_RAS_TEX}(#{sObjectID}, #{cObjectInfo.cTex.hState[TSR_PRM_PNDPTN]})")
        end
      }

      cElement.set_checkpoint(hMainTaskInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 動作状態(Txxx_STA)のタイムイベントハンドラを設定する
    #         コードを返す
    #=================================================================
    def gc_pre_time_event_sta()
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((GRP_TIME_EVENT_HDL.include?(cObjectInfo.sObjectType) == true) &&
            (GRP_TIME_EVENT_STA.include?(cObjectInfo.hState[TSR_PRM_STATE]) == true) &&
             (cObjectInfo.hState[TSR_PRM_HDLSTAT] != TSR_STT_ACTIVATE))
          if (cObjectInfo.sObjectType == TSR_OBJ_ALARM)
            cElement.set_syscall(hMainTaskInfo, "#{API_STA_ALM}(#{sObjectID}, #{cObjectInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})")
          elsif (cObjectInfo.sObjectType == TSR_OBJ_CYCLE)
            cElement.set_syscall(hMainTaskInfo, "#{API_STA_CYC}(#{sObjectID})")
          end
        end
      }

      cElement.set_checkpoint(hMainTaskInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 許可状態の割込みを設定するコードを返す
    #=================================================================
    def gc_pre_interrupt_ena()
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      # ena_intする必要のある割込み番号を抽出
      aEnaIntNo = []
      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((GRP_INTERRUPT.include?(cObjectInfo.sObjectType) == true) &&
            (cObjectInfo.hState[TSR_PRM_STATE] == KER_TA_ENAINT))
          aEnaIntNo.push(cObjectInfo.hState[TSR_PRM_INTNO])
        end
      }

      # 重複を削除してすべてena_intを実行する
      aEnaIntNo.uniq!()
      aEnaIntNo.each{|snIntNo|
        cElement.set_syscall(hMainTaskInfo, "#{API_ENA_INT}(#{snIntNo})")
      }

      cElement.set_checkpoint(hMainTaskInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 現在優先度と初期優先度が異なるタスクの設定処理を
    #         IMCodeElementクラスにまとめて返す
    #         (running,dormant,running-suspend,オブジェクト待ち以外のタスク)
    #=================================================================
    def gc_pre_task_pri_chg()
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      @hTask.each{|sObjectID, cObjectInfo|
        if ((cObjectInfo.hState[TSR_PRM_STATE] != KER_TTS_RUN) &&
            (cObjectInfo.hState[TSR_PRM_STATE] != KER_TTS_RUS) &&
            (cObjectInfo.hState[TSR_PRM_STATE] != KER_TTS_DMT) &&
            (cObjectInfo.hState[TSR_PRM_TSKPRI] != cObjectInfo.hState[TSR_PRM_ITSKPRI]) &&
            !(!cObjectInfo.hState[TSR_PRM_WOBJID].nil?() && !GRP_WAIT_NON_OBJECT.include?(cObjectInfo.hState[TSR_PRM_WOBJID])))
          cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{sObjectID}, #{cObjectInfo.hState[TSR_PRM_TSKPRI]})")
        end
      }

      cElement.set_checkpoint(hMainTaskInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 実行状態であるタスクの初期状態の設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_running()
      # 実行状態になるべきのタスクをメインタスクから起こす
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      cElement.set_syscall(hMainTaskInfo, "#{API_ACT_TSK}(#{@cRunning.sObjectID})")

      # タスク保留例外要因があれば設定する
      if (!@cRunning.cTex.nil?() && 
          !@cRunning.cTex.hState[TSR_PRM_PNDPTN].nil?() && 
          @cRunning.cTex.hState[TSR_PRM_PNDPTN] != 0 &&
          @cRunning.cTex.hState[TSR_PRM_HDLSTAT] != TSR_STT_ACTIVATE)
        cElement.set_syscall(hMainTaskInfo, "#{API_RAS_TEX}(#{@cRunning.sObjectID}, #{@cRunning.cTex.hState[TSR_PRM_PNDPTN]})")
      end

      # 現在優先度と初期優先度が異なれば設定する
      if (@cRunning.hState[TSR_PRM_TSKPRI] != @cRunning.hState[TSR_PRM_ITSKPRI])
        cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{@cRunning.sObjectID}, #{@cRunning.hState[TSR_PRM_TSKPRI]})")
      end

      cElement.set_checkpoint(hMainTaskInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: ディスパッチ禁止にさせる処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_dis_dsp()
      cElement      = IMCodeElement.new()
      hProcUnitInfo = get_proc_unit_info(@cRunning)

      cElement.set_syscall(hProcUnitInfo, "#{API_DIS_DSP}()")

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 割込み優先度マスクが0以外にさせる処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_set_ipm()
      cElement      = IMCodeElement.new()
      hProcUnitInfo = get_proc_unit_info(@cRunning)

      cElement.set_syscall(hProcUnitInfo, "#{API_CHG_IPM}(#{@cCpuState.hState[TSR_PRM_CHGIPM]})")

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 起動中の非タスクの設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_nontask_activate()
      cElement      = IMCodeElement.new()
      hProcUnitInfo = get_proc_unit_info(@cRunning)

      case @cActivate.sObjectType
      # アラームハンドラ
      when TSR_OBJ_ALARM
        # 0秒に設定して1ミリ秒進めて起動する
        cElement.set_syscall(hProcUnitInfo, "#{API_STA_ALM}(#{@cActivate.sObjectID}, 0)")
        # STA属性があれば自身からista_almを実行する
        if (@cActivate.hState[TSR_PRM_STATE] == TSR_STT_TALM_STA)
          cElement.set_syscall(get_proc_unit_info(@cActivate), "#{API_ISTA_ALM}(#{@cActivate.sObjectID}, #{@cActivate.hState[TSR_PRM_LEFTTMO]})")
        end
        gc_tick_gain(cElement)

      # 周期ハンドラ
      when TSR_OBJ_CYCLE
        cElement.set_syscall(hProcUnitInfo, "#{API_STA_CYC}(#{@cActivate.sObjectID})")
        # 位相の分だけ時間を進めて起動する
        (@cActivate.hState[TSR_PRM_CYCPHS].to_i + 1).times{
          gc_tick_gain(cElement)
        }

      # 割込みハンドラ，割込みサービスルーチン
      when TSR_OBJ_INTHDR, TSR_OBJ_ISR
        # 割込み発生関数を実行して起動する
        cElement.set_code(hProcUnitInfo, "#{FNC_INT_RAISE}(#{@cActivate.hState[TSR_PRM_INTNO]})")

      # CPU例外ハンドラ
      when TSR_OBJ_EXCEPTION
        # CPU例外発生関数を実行して起動する
        cElement.set_code(hProcUnitInfo, "#{FNC_CPUEXC_RAISE}(#{@cActivate.hState[TSR_PRM_EXCNO]})")

      else
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      # システム時刻更新/割込み/CPU例外発生が遅延した場合に備え，
      # 非タスク側でチェックポイントを発行し，発行元でチェックポイントを待つ
      cElement.set_checkpoint(get_proc_unit_info(@cActivate))
      cElement.wait_checkpoint(hProcUnitInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 一時的にSLEEPまたはDELAYさせたタスクを実行順番に起こす
    #         処理をIMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_ready_porder()
      # 一時スリープさせておいたタスクのporderを確認し，
      # 実行可能状態にさせる

      cElement  = IMCodeElement.new()  # elementクラスを用意する

      aPriorityTemp = []  # 優先順位のないタスクを保持するための配列
      aPriOrderTemp = []  # 優先順位のあるタスクを優先順で保持するための配列

      @hTask.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RDY)
          nPOrder = cObjectInfo.hState[TSR_PRM_PORDER]

          if (nPOrder.nil?())
            aPriorityTemp.push(sObjectID)
          else
            aPriOrderTemp[nPOrder] = sObjectID  # 優先順位の順で挿入
          end
        end
      }

      aPriOrderTemp.delete(nil)  # 必要ではないnilは削除
      aPriOrderTemp = aPriOrderTemp.concat(aPriorityTemp)
      hProcUnitInfo = get_proc_unit_info(@cActivate, @cRunning)

      aPriOrderTemp.each{|sObjectID|
        # 実行状態の非タスクがいる場合
        if (!@cActivate.nil?())
          cElement.set_syscall(hProcUnitInfo, "#{API_IWUP_TSK}(#{sObjectID})")
        # 実行状態のタスクがいる場合
        elsif (!@cRunning.nil?())
          cElement.set_syscall(hProcUnitInfo, "#{API_WUP_TSK}(#{sObjectID})")
        else
          abort(ERR_MSG % [__FILE__, __LINE__])
        end
      }

      cElement.set_checkpoint(hProcUnitInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 起動・起床要求キューイングの設定をする処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_queueing()
      cElement      = IMCodeElement.new()  # elementクラスを用意する
      hProcUnitInfo = get_proc_unit_info(@cActivate, @cRunning)

      @hTask.each{|sObjectID, cObjectInfo|
        if (!cObjectInfo.hState[TSR_PRM_ACTCNT].nil?() && cObjectInfo.hState[TSR_PRM_ACTCNT] >= 1)
          cObjectInfo.hState[TSR_PRM_ACTCNT].times{
            if (!@cActivate.nil?())
              cElement.set_syscall(hProcUnitInfo, "#{API_IACT_TSK}(#{sObjectID})")
            elsif (!@cRunning.nil?())
              cElement.set_syscall(hProcUnitInfo, "#{API_ACT_TSK}(#{sObjectID})")
            else
              abort(ERR_MSG % [__FILE__, __LINE__])
            end
          }
        end

        if(!cObjectInfo.hState[TSR_PRM_WUPCNT].nil?() && cObjectInfo.hState[TSR_PRM_WUPCNT] >= 1)
          cObjectInfo.hState[TSR_PRM_WUPCNT].times{
            if (!@cActivate.nil?())
              cElement.set_syscall(hProcUnitInfo, "#{API_IWUP_TSK}(#{sObjectID})")
            elsif (!@cRunning.nil?())
              cElement.set_syscall(hProcUnitInfo, "#{API_WUP_TSK}(#{sObjectID})")
            else
              abort(ERR_MSG % [__FILE__, __LINE__])
            end
          }
        end
      }

      cElement.set_checkpoint(hProcUnitInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: CPUロック状態の設定(実行中の処理単位から設定)処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_cpu_loc()
      # 起動状態の非タスクがいる場合，アラーム・周期ハンドラ両方
      # いる場合はアラームにさせる
      # また，起動状態の非タスクがいない，かつ実行状態のタスクが
      # いる場合は，タスクにさせる

      cElement      = IMCodeElement.new()
      hProcUnitInfo = get_proc_unit_info(@cActivate, @cRunning)

      if (!@cActivate.nil?())
        cElement.set_syscall(hProcUnitInfo, "#{API_ILOC_CPU}()")
      else
        cElement.set_syscall(hProcUnitInfo, "#{API_LOC_CPU}()")
      end

      cElement.set_checkpoint(hProcUnitInfo)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 次の状態を処理する前に行うべきのメインの設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_maintask_set(cNextActivate, cNextRunning, bNextCpuLock, bNextRunSus)
      check_class(ProcessUnit, cNextActivate, true) # 次のコンディションで起動中の処理単位
      check_class(ProcessUnit, cNextRunning, true)  # 次のコンディションで実行中のタスク
      check_class(Bool, bNextCpuLock)               # 次のコンディションでCPUロックか
      check_class(Bool, bNextRunSus)                # 次のコンディションで過渡状態かどうか

      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      # 次のコンディションに実行状態の処理単位が存在せず，
      # pre_conditionで非タスクがACTIVATEだった場合は，
      # メインタスクをslp_tskして，非タスクからiwup_tskで起こす
      # (非タスク起動前にメインタスクの処理が進むのを避ける)
      if (!@cActivate.nil?() && @cRunning.nil?() && cNextActivate.nil?() && cNextRunning.nil?())
        cElement.set_syscall(hMainTaskInfo, "#{API_SLP_TSK}()")
        cElement.set_syscall(get_proc_unit_info(@cActivate), "#{API_IWUP_TSK}(#{TTG_MAIN_TASK})")
        @lMainTaskState = :running

      # 次の条件をすべて満たす場合，次のコンディションで
      # メインタスクを起床できるため，slp_tskする
      # 1)次のコンディションに実行状態の処理単位が存在する
      # 2)次のコンディションの実行状態のタスクが過渡状態でない
      #   (後処理で過渡状態を解除した時点で強制待ちとなり，メインタスクを起床できない)
      # 3)次のコンディションがCPUロックでない
      #   (複数のdo/postが続く場合に途中でchg_pri/wup_tskが発行できない可能性がある)
      elsif ((!cNextActivate.nil?() || !cNextRunning.nil?()) && (bNextRunSus == false) && (bNextCpuLock == false))
        cElement.set_syscall(hMainTaskInfo, "#{API_SLP_TSK}()")
        @lMainTaskState = :sleep

      # 上記以外ではメインプロセッサを起床できない可能性があるため，
      # 実行中のタスクが存在する場合，メインタスクの優先度を下げておく
      elsif (!@cRunning.nil?())
        cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{TTG_TSK_SELF}, #{TTG_WAIT_PRI})")
        # pre_conditionでrunningのタスク優先度がTTG_WAIT_PRIだった場合
        if (check_running_pri(TTG_WAIT_PRI) == true)
          cElement.set_syscall(hMainTaskInfo, "#{API_ROT_RDQ}(#{TTG_WAIT_PRI})")
        end
        # メインタスクが既にTTG_MAIN_PRIの場合はchg_priを発行しない
        cElement.set_chg_pri_main_task(hMainTaskInfo)

        @lMainTaskState = :ready

      # 上記以外は何もしない
      else
        @lMainTaskState = :running

      end

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 割込み優先度マスク設定の必要性がある場合はtrueを，
    #         ない場合はfalseを返す
    #=================================================================
    def check_pre_need_chg_ipm()
      # 割込み優先度マスクが指定されていない場合はfalseを返す
      if (check_chg_ipm() == false)
        return false # [Bool] 割込み優先度マスク設定の必要性有無
      end

      # 以下のいずれかを満たす場合のみ割込み優先度マスクを変更する
      # 1)ACTIVATEな非タスクが存在しない
      # 2)ACTIVATEな非タスクがCPU例外ハンドラである
      if (@cActivate.nil?() || (@cActivate.sObjectType == TSR_OBJ_EXCEPTION))
        return true # [Bool] 割込み優先度マスク設定の必要性有無
      else
        return false # [Bool] 割込み優先度マスク設定の必要性有無
      end
    end

    #=================================================================
    # 概  要: 初期化/終了ルーチンクラスの配列をID順で生成して返す
    #=================================================================
    def get_ini_ter_object_info_sort_by_id(sObjectType)
      check_class(String, sObjectType)  # 対象とするオブジェクトタイプ[初期化or終了ルーチン]

      aIniTerInfo = []
      @hAllObject.sort.each{|aObjectInfo|
        if (aObjectInfo[1].sObjectType == sObjectType)
          aIniTerInfo.push(aObjectInfo[1])
        end
      }

      return aIniTerInfo  # [Array]初期化/終了ルーチンクラスの配列
    end

    #=================================================================
    # 概  要: TA_STA属性の周期ハンドラの有無を返す
    #=================================================================
    def exist_cyclic_sta()
      @hAllObject.each{|sObjID, cObjectInfo|
        if ((cObjectInfo.sObjectType == TSR_OBJ_CYCLE) && (cObjectInfo.hState[TSR_PRM_ATR] == KER_TA_STA))
          return true    # [Bool]TA_STA属性の周期ハンドラが存在する
        end
      }

      return false  # [Bool]TA_STA属性の周期ハンドラが存在しない
    end

    #=================================================================
    # 概  要: TA_ENAINT属性の割込みサービスルーチンの有無を返す
    #=================================================================
    def exist_isr_enaint()
      @hAllObject.each{|sObjID, cObjectInfo|
        if ((cObjectInfo.sObjectType == TSR_OBJ_ISR) && (cObjectInfo.hState[TSR_PRM_ATR] == KER_TA_ENAINT))
          return true    # [Bool]TA_ENAINT属性の割込みサービスルーチンが存在する
        end
      }

      return false  # [Bool]TA_ENAINT属性の割込みサービスルーチンが存在しない
    end
  end
end
