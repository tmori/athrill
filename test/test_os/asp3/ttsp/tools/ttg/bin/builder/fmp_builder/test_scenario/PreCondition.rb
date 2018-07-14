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

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: PreCondition
  # 概    要: pre_conditionの情報を処理するクラス
  #===================================================================
  class PreCondition < Condition

    #=================================================================
    # 概  要: 現在優先度と初期優先度が異なるタスクを検査してある場合は
    #         trueを，ない場合はfalseを返す
    #=================================================================
    def exist_pre_task_pri_chg_fmp()
      @hTask.each{|sObjectID, cObjectInfo|
        if ((cObjectInfo.hState[TSR_PRM_STATE] != KER_TTS_RUN) &&
            (cObjectInfo.hState[TSR_PRM_STATE] != KER_TTS_DMT) &&
            (cObjectInfo.hState[TSR_PRM_STATE] != KER_TTS_RUS) &&
            (cObjectInfo.hState[TSR_PRM_TSKPRI] != cObjectInfo.hState[TSR_PRM_ITSKPRI]) &&
            !(!cObjectInfo.hState[TSR_PRM_WOBJID].nil?() && !GRP_WAIT_NON_OBJECT.include?(cObjectInfo.hState[TSR_PRM_WOBJID])))
          return true # [Bool]現在優先度と初期優先度が異なるタスクがある場合
        end
      }

      return false # [Bool]現在優先度と初期優先度が異なるタスクがない場合
    end

    #=================================================================
    # 概  要: 過渡状態のタスクを検査してある場合はtrueを，
    #         ない場合はfalseを返す
    #=================================================================
    def exist_pre_task_running_suspended_fmp()
      @hTask.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RUS)
          return true # [Bool]過渡状態のタスクがある場合
        end
      }

      return false # [Bool]過渡状態のタスクがない場合
    end

    #=================================================================
    # 概  要: 対象タスクを起動して，メインタスクの優先度を下げる処理を
    #         cElementに格納する
    #=================================================================
    def set_act_task_fmp(cElement, cObjectInfo)
      check_class(ProcessUnit, cObjectInfo) # 対象タスククラス

      hMainTaskInfo = get_proc_unit_info()

      # 対象タスクを起動
      cElement.set_syscall(hMainTaskInfo, "#{API_MACT_TSK}(#{cObjectInfo.sObjectID}, #{cObjectInfo.hState[TSR_PRM_PRCID]})")

      # メインプロセッサならばメインタスクの優先度を下げてディスパッチ
      if (cObjectInfo.hState[TSR_PRM_PRCID] == @sMainPrcid)
        cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{TTG_TSK_SELF}, #{TTG_WAIT_PRI})")

        # 対象タスクの優先度がTTG_WAIT_PRIと同じだった場合，rot_rdqを発行してディスパッチさせる
        if (cObjectInfo.hState[TSR_PRM_TSKPRI] == TTG_WAIT_PRI)
          cElement.set_syscall(hMainTaskInfo, "#{API_ROT_RDQ}(#{TTG_WAIT_PRI})")
        end
      end

    end
    private :set_act_task_fmp


    #=================================================================
    # 概  要: 同期・通信オブジェクトによる待ち状態タスクの初期状態の
    #         設定処理をIMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_scobj_waiting_fmp()
      # タスクをオブジェクト待ちにするコードブロックを返す
      # 例）ASP_dataqueue_rcv_dtq_f_2_1_1
      #   メインタスクはTASK2を起こし，タスク例外ハンドラが存在する場合は
      #   該当の設定をし，TASK2を自分でsnd_dtqさせ，
      #   送信待ちにさせる(これを送信待ちになっているタスクの数だけ)
      #   （※プロセッサの順序が保たれていない）
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      @hWaitObject.each{|sObjectID, cObjectInfo|
        # 待ちタスクのリスト
        if (!cObjectInfo.hState[TSR_PRM_WTSKLIST].nil?() && !cObjectInfo.hState[TSR_PRM_WTSKLIST].empty?())
          cObjectInfo.hState[TSR_PRM_WTSKLIST].each{|aWtskList|
            aWtskList.each_key{|sTaskID|
              cTaskInfo = @hTask[sTaskID]

              # 対象タスクへディスパッチするためのコード
              set_act_task_fmp(cElement, cTaskInfo)

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

              if (cTaskInfo.hState[TSR_PRM_PRCID] == @sMainPrcid)
                # メインタスクの優先度を戻すためのコード
                set_chg_pri(cElement)
              else
                cElement.set_state_sync(get_proc_unit_info(), hProcUnitInfo[:id], KER_TTS_WAI)
              end

              # 対象タスクの現在優先度と初期優先度が異なる場合ここでchg_priする
              # (同期通信オブジェクトの属性が優先度順だった場合に，待ち順でないと順序が変わる可能性がある)
              if (cTaskInfo.hState[TSR_PRM_TSKPRI] != cTaskInfo.hState[TSR_PRM_ITSKPRI])
                cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{cTaskInfo.sObjectID}, #{cTaskInfo.hState[TSR_PRM_TSKPRI]})")
              end

              cElement.set_block_delimiter()
            }
          }
        end

        # 送信待ちタスクのリスト
        if (!cObjectInfo.hState[TSR_PRM_STSKLIST].nil?() && !cObjectInfo.hState[TSR_PRM_STSKLIST].empty?())
          cObjectInfo.hState[TSR_PRM_STSKLIST].each{|aStskList|
            aStskList.each{|sTaskID, hData|
              cTaskInfo = @hTask[sTaskID]

              # 対象タスクへディスパッチするためのコード
              set_act_task_fmp(cElement, cTaskInfo)

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

              if (cTaskInfo.hState[TSR_PRM_PRCID] == @sMainPrcid)
                # メインタスクの優先度を戻すためのコード
                set_chg_pri(cElement)
              else
                cElement.set_state_sync(get_proc_unit_info(), hProcUnitInfo[:id], KER_TTS_WAI)
              end

              # 対象タスクの現在優先度と初期優先度が異なる場合ここでchg_priする
              # (同期通信オブジェクトの属性が優先度順だった場合に，待ち順でないと順序が変わる可能性がある)
              if (cTaskInfo.hState[TSR_PRM_TSKPRI] != cTaskInfo.hState[TSR_PRM_ITSKPRI])
                cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{cTaskInfo.sObjectID}, #{cTaskInfo.hState[TSR_PRM_TSKPRI]})")
              end

              cElement.set_block_delimiter()
            }
          }
        end

        # 受信待ちタスクのリスト
        if (!cObjectInfo.hState[TSR_PRM_RTSKLIST].nil?() && !cObjectInfo.hState[TSR_PRM_RTSKLIST].empty?())
          cObjectInfo.hState[TSR_PRM_RTSKLIST].each{|aRtskList|
            aRtskList.each_key{|sTaskID|
              cTaskInfo = @hTask[sTaskID]

              # 対象タスクへディスパッチするためのコード
              set_act_task_fmp(cElement, cTaskInfo)

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

              if (cTaskInfo.hState[TSR_PRM_PRCID] == @sMainPrcid)
                # メインタスクの優先度を戻すためのコード
                set_chg_pri(cElement)
              else
                cElement.set_state_sync(get_proc_unit_info(), hProcUnitInfo[:id], KER_TTS_WAI)
              end

              # 対象タスクの現在優先度と初期優先度が異なる場合ここでchg_priする
              # (同期通信オブジェクトの属性が優先度順だった場合に，待ち順でないと順序が変わる可能性がある)
              if (cTaskInfo.hState[TSR_PRM_TSKPRI] != cTaskInfo.hState[TSR_PRM_ITSKPRI])
                cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{cTaskInfo.sObjectID}, #{cTaskInfo.hState[TSR_PRM_TSKPRI]})")
              end

              cElement.set_block_delimiter()
            }
          }
        end
      }

      # 待ち状態作成後，メインタスクに戻った時点でチェックポイント
      cElement.set_checkpoint(get_proc_unit_info())

      return cElement # [IMCodeElement]同期・通信オブジェクトによる待ち状態タスクの初期状態の設定コード
    end


    #=================================================================
    # 概  要: 待ち状態であるタスクの初期状態の設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_waiting_fmp()
      cElement = IMCodeElement.new()

      @aTask.each{ |nPrcid, sObjectID, cObjectInfo|
        if (GRP_WAIT_NON_OBJECT.include?(cObjectInfo.hState[TSR_PRM_WOBJID]))
          # 対象タスクへディスパッチするためのコード
          set_act_task_fmp(cElement, cObjectInfo)

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

          if (nPrcid == @sMainPrcid)
            # メインタスクの優先度を戻すためのコード
            set_chg_pri(cElement)
          else
            cElement.set_state_sync(get_proc_unit_info(), hProcUnitInfo[:id], KER_TTS_WAI)
          end
          # 待ち状態作成後，メインタスクに戻った時点でチェックポイントとブロック切り分け
          cElement.set_checkpoint(get_proc_unit_info())
          cElement.set_block_delimiter()
        end
      }

      return cElement # [IMCodeElement]エレメント
    end


    #=================================================================
    # 概  要: 休止状態であるタスクの初期状態の設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_dormant_fmp()
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      @aTask.each{|nPrcid, sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_DMT)
          cElement.set_migrate_task(hMainTaskInfo, sObjectID, nPrcid)
        end
      }

      return cElement # [IMCodeElement]休止状態であるタスクの初期状態の設定コード
    end


    #=================================================================
    # 概  要: 強制状態であるタスクの初期状態の設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_suspended_fmp()
      # メインタスクでsus_tsk(対象タスク)処理
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      @aTask.each{|nPrcid, sObjectID, cObjectInfo|
        if (GRP_SUSPENDED.include?(cObjectInfo.hState[TSR_PRM_STATE]))  # TTS_SUS，TTS_WAS
          hProcUnitInfo = get_proc_unit_info(cObjectInfo)
          if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_SUS)
            # 対象タスクへディスパッチするためのコード
            set_act_task_fmp(cElement, cObjectInfo)

            # 対象タスクのチェックポイント設定
            cElement.set_checkpoint(hProcUnitInfo)

            cElement.set_syscall(hProcUnitInfo, "#{API_SLP_TSK}()")

            if (nPrcid == @sMainPrcid)
              # メインタスクの優先度を戻すためのコード
              set_chg_pri(cElement)
            else
              cElement.set_state_sync(hMainTaskInfo, sObjectID, KER_TTS_WAI)
            end
          end

          cElement.set_syscall(hMainTaskInfo, "#{API_SUS_TSK}(#{sObjectID})")

          # 強制待ちの場合，起床して完成
          if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_SUS)
            cElement.set_syscall(hMainTaskInfo, "#{API_WUP_TSK}(#{sObjectID})")
            # 起床する前にrefしないよう強制待ちとなったことを確認する
            cElement.set_state_sync(hMainTaskInfo, sObjectID, KER_TTS_SUS)
          end
          # 待ち状態作成後，メインタスクに戻った時点でチェックポイントとブロック切り分け
          cElement.set_checkpoint(hMainTaskInfo)
          cElement.set_block_delimiter()
        end
      }

      return cElement # [IMCodeElement]強制状態であるタスクの初期状態の設定コード
    end

    #=================================================================
    # 概  要: レディー状態であるタスクの初期状態の設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_ready_fmp()
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      # メインタスクから起こして、自タスクでslp_tskしておく
      @aTask.each{|nPrcid, sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RDY)
          # 対象タスクへディスパッチするためのコード
          set_act_task_fmp(cElement, cObjectInfo)

          # 対象タスクの情報作成，チェックポイント設定
          hProcUnitInfo = get_proc_unit_info(cObjectInfo)
          cElement.set_checkpoint(hProcUnitInfo)

          cElement.set_syscall(hProcUnitInfo, "#{API_SLP_TSK}()")

          if (nPrcid == @sMainPrcid)
            # メインタスクの優先度を戻すためのコード
            set_chg_pri(cElement)
          else
            cElement.set_state_sync(get_proc_unit_info(), hProcUnitInfo[:id], KER_TTS_WAI)
          end
          # メインタスクに戻った時点でチェックポイントとブロック切り分け
          cElement.set_checkpoint(get_proc_unit_info())
          cElement.set_block_delimiter()
        end
      }

      return cElement # [IMCodeElement]レディー状態であるタスクの初期状態の設定コード
    end

    #=================================================================
    # 概  要: 実行状態であるタスク，タスク例外の初期状態の設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_running_texhdr_activate_fmp()
      # 実行状態になるべきのタスクをメインタスクから起こす
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      # メインプロセッサを最後に設定する
      # (他プロセッサが実行状態となる前にメインからrefするのを防ぐ)
      aPrcid = []
      @aPrcid.each{ |nPrcid|
        if (nPrcid != @sMainPrcid)
          aPrcid.push(nPrcid)
        end
      }
      aPrcid.push(@sMainPrcid)

      aPrcid.each{|nPrcid|
        if (!@aRunning[nPrcid].nil?())
          # タスクの場合
          if (@aRunning[nPrcid].sObjectType == TSR_OBJ_TASK)
            hProcUnitInfo = get_proc_unit_info(@aRunning[nPrcid])
            cElement.set_syscall(hMainTaskInfo, "#{API_MACT_TSK}(#{@aRunning[nPrcid].sObjectID}, #{@aRunning[nPrcid].hState[TSR_PRM_PRCID]})")
            # タスク保留例外要因があれば設定する
            if (!@aRunning[nPrcid].cTex.nil?() && 
                !@aRunning[nPrcid].cTex.hState[TSR_PRM_PNDPTN].nil?() && 
                @aRunning[nPrcid].cTex.hState[TSR_PRM_PNDPTN] != 0 &&
                @aRunning[nPrcid].cTex.hState[TSR_PRM_HDLSTAT] != TSR_STT_ACTIVATE)
              cElement.set_syscall(hMainTaskInfo, "#{API_RAS_TEX}(#{@aRunning[nPrcid].sObjectID}, #{@aRunning[nPrcid].cTex.hState[TSR_PRM_PNDPTN]})")
            end

            # 現在優先度と初期優先度が異なれば設定する
            if (@aRunning[nPrcid].hState[TSR_PRM_TSKPRI] != @aRunning[nPrcid].hState[TSR_PRM_ITSKPRI])
              cElement.set_syscall(hMainTaskInfo, "#{API_CHG_PRI}(#{@aRunning[nPrcid].sObjectID}, #{@aRunning[nPrcid].hState[TSR_PRM_TSKPRI]})")
            end

            # active_sync
            if (@aRunning[nPrcid].hState[TSR_PRM_PRCID] != @sMainPrcid)
              cElement.set_checkpoint(hProcUnitInfo)
              cElement.set_wait_check_sync(hMainTaskInfo, @aRunning[nPrcid].hState[TSR_PRM_PRCID])
            end

            cElement.set_block_delimiter()

          # タスク例外の場合
          else
            hRunningTask  = get_proc_unit_info(get_object_info(@aRunning[nPrcid].hState[TSR_PRM_TASK]))
            hActivateTex  = get_proc_unit_info(@aRunning[nPrcid])

            # 実行状態のタスクを起動する
            cElement.set_syscall(hMainTaskInfo, "#{API_MACT_TSK}(#{@aRunning[nPrcid].hState[TSR_PRM_TASK]}, #{@aRunning[nPrcid].hState[TSR_PRM_PRCID]})")

            # 実行状態のタスクがタスク例外を起動する
            cElement.set_syscall(hRunningTask, "#{API_RAS_TEX}(#{TTG_TSK_SELF}, #{@aRunning[nPrcid].hState[TSR_PRM_TEXPTN]})")

            # タスク例外許可状態がTTEX_DISの場合，ena_texしてACTIVATEにする
            if (@aRunning[nPrcid].hState[TSR_PRM_STATE] == TSR_STT_TTEX_DIS)
              cElement.set_syscall(hRunningTask, "#{API_ENA_TEX}()")
            # タスク例外許可状態がTTEX_ENAの場合，既にena_texしてあるが，タスク例外起動後に再度ena_texする
            else
              cElement.set_syscall(hActivateTex, "#{API_ENA_TEX}()")
            end

            # 保留例外要因パターンが0でない場合，自分でras_texする
            if (@aRunning[nPrcid].hState[TSR_PRM_PNDPTN] != 0)
              cElement.set_syscall(hActivateTex, "#{API_RAS_TEX}(#{TTG_TSK_SELF}, #{@aRunning[nPrcid].hState[TSR_PRM_PNDPTN]})")
            end

            # active_sync
            if (@aRunning[nPrcid].hState[TSR_PRM_PRCID] != @sMainPrcid)
              cElement.set_checkpoint(hActivateTex)
              cElement.set_wait_check_sync(hMainTaskInfo, @aRunning[nPrcid].hState[TSR_PRM_PRCID])
            end

            cElement.set_block_delimiter()
          end
        end
      }

      return cElement # [IMCodeElement]実行状態であるタスク，タスク例外の初期状態の設定コード
    end

    #=================================================================
    # 概  要: 過渡状態であるタスクの初期状態の設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_running_suspended_fmp()
      cElement = IMCodeElement.new()

      @aTask.each{|nPrcid, sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RUS)
          # 過渡状態であるタスク自身から強制待ちにさせる
          hProcUnitInfo = get_proc_unit_info(@aActivate[nPrcid], @aRunning[nPrcid])
          cElement.set_syscall(hProcUnitInfo, "#{FNC_SUS_TSK}(#{sObjectID})")

          if (nPrcid == @sMainPrcid)
            cElement.set_block_delimiter()
          else
            # 他プロセッサの場合，メインプロセッサの実行状態の処理単位から確認する
            cElement.set_state_sync(get_proc_unit_info(@cActivate, @cRunning), sObjectID, KER_TTS_RUS)
            cElement.set_block_delimiter()
          end
        end
      }

      return cElement # [IMCodeElement]過渡状態であるタスクの初期状態の設定コード
    end

    #=================================================================
    # 概  要: 起動中の非タスクの設定処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_nontask_activate_fmp()
      cElement      = IMCodeElement.new()
      hProcUnitInfo = get_proc_unit_info(@cRunning)

      # 進めるべき時間を保持する
      nGainTick = 0

      # メインプロセッサを最後に設定しないと非タスクの起動によって処理を
      # 継続できなくなるため，メインプロセッサを最後に移動させる
      aPrcid = []
      @aPrcid.each{|nPrcid|
        if (nPrcid != @sMainPrcid)
          aPrcid.push(nPrcid)
        end
      }
      aPrcid.push(@sMainPrcid)

      aWaitPrcid = []
      aPrcid.each{|nPrcid|
        if (!@aActivate[nPrcid].nil?())
          case @aActivate[nPrcid].sObjectType
          # アラームハンドラ
          when TSR_OBJ_ALARM
            # 0秒に設定して1ミリ秒進める
            if (@cConf.is_timer_local?())
              cElement.set_syscall(hProcUnitInfo, "#{API_MSTA_ALM}(#{@aActivate[nPrcid].sObjectID}, 0, #{nPrcid})")
            else
              cElement.set_syscall(hProcUnitInfo, "#{API_STA_ALM}(#{@aActivate[nPrcid].sObjectID}, 0)")
            end
            # STA属性があれば自身からista_almを実行する
            if (@aActivate[nPrcid].hState[TSR_PRM_STATE] == TSR_STT_TALM_STA)
              cElement.set_syscall(get_proc_unit_info(@aActivate[nPrcid]), "#{API_ISTA_ALM}(#{@aActivate[nPrcid].sObjectID}, #{@aActivate[nPrcid].hState[TSR_PRM_LEFTTMO]})")
            end
            if (nGainTick < 1)
              nGainTick = 1
            end

          # 周期ハンドラ
          when TSR_OBJ_CYCLE
            if (@cConf.is_timer_local?())
              cElement.set_syscall(hProcUnitInfo, "#{API_MSTA_CYC}(#{@aActivate[nPrcid].sObjectID}, #{nPrcid})")
            else
              cElement.set_syscall(hProcUnitInfo, "#{API_STA_CYC}(#{@aActivate[nPrcid].sObjectID})")
            end
            # 位相の分だけ時間を進める
            if (nGainTick < @aActivate[nPrcid].hState[TSR_PRM_CYCPHS].to_i + 1)
              nGainTick = @aActivate[nPrcid].hState[TSR_PRM_CYCPHS].to_i + 1
            end

          # 割込みハンドラ，割込みサービスルーチン
          when TSR_OBJ_INTHDR, TSR_OBJ_ISR
            # 割込みハンドラ，割込みサービスルーチンが起動しているプロセッサで実行中の処理単位を取得
            cObjectInfo = aRunning[@aActivate[nPrcid].hState[TSR_PRM_PRCID]]
            hObjectInfo = get_proc_unit_info(cObjectInfo)
            # 割込み発生関数を実行して起動する
            cElement.set_code(hObjectInfo, "#{FNC_INT_RAISE}(#{@aActivate[nPrcid].hState[TSR_PRM_INTNO]})")

          # CPU例外ハンドラ
          when TSR_OBJ_EXCEPTION
            # CPU例外が起動しているプロセッサで実行中の処理単位を取得
            cObjectInfo = aRunning[@aActivate[nPrcid].hState[TSR_PRM_PRCID]]
            hObjectInfo = get_proc_unit_info(cObjectInfo)
            # CPU例外発生関数を実行して起動する
            cElement.set_code(hObjectInfo, "#{FNC_CPUEXC_RAISE}(#{@aActivate[nPrcid].hState[TSR_PRM_EXCNO]})")

          else
            abort(ERR_MSG % [__FILE__, __LINE__])
          end

          # 起動確認用チェックポイント
          if (nPrcid != @sMainPrcid)
            cElement.set_checkpoint(get_proc_unit_info(@aActivate[nPrcid]))
            aWaitPrcid.push(nPrcid)
          end
        end
      }

      # ローカルタイマ方式の場合，他プロセッサの時間が進んでからメインプロセッサを進める
      if (@cConf.is_timer_local?())
        # 他プロセッサにACTIVATEな非タスクが存在する場合
        if (!aWaitPrcid.empty?())
          (1..nGainTick).each{|nCnt|
            if (nCnt == nGainTick)
              # 他プロセッサの時間を進めて起動する
              gc_tick_gain_local_other_fmp(cElement, aWaitPrcid)
              # メインプロセッサで実行中のタスクで起動したことを待つ
              aWaitPrcid.each{|nPrcid|
                cElement.set_wait_check_sync(hProcUnitInfo, nPrcid)
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
      # グローバルタイマ方式はタイムティックの供給のみ行う
      else
        nGainTick.times{
          gc_tick_gain_global_fmp(cElement, aWaitPrcid)
        }

        # 他プロセッサのACTIVATEな非タスクをメインプロセッサで実行中のタスクで起動したことを待つ
        if (!aWaitPrcid.empty?())
          aWaitPrcid.each{|nPrcid|
            cElement.set_wait_check_sync(hProcUnitInfo, nPrcid)
          }
        end
      end

      return cElement # [IMCodeElement]起動中の非タスクの設定コード
    end


    #=================================================================
    # 概  要: ディスパッチ禁止にさせる処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_dis_dsp_fmp()
      cElement = IMCodeElement.new()

      @aPrcid.each{ |nPrcid|
        if (@aRunning[nPrcid] != nil)
          if (!@aCpuState[nPrcid].nil?() && !@aCpuState[nPrcid].hState[TSR_PRM_DISDSP].nil?() && (@aCpuState[nPrcid].hState[TSR_PRM_DISDSP] == true))
            hProcUnitInfo = get_proc_unit_info(@aRunning[nPrcid])
            cElement.set_syscall(hProcUnitInfo, "#{API_DIS_DSP}()")
          end
        end
      }

      return cElement # [IMCodeElement]ディスパッチ禁止にさせる処理コード
    end


    #=================================================================
    # 概  要: 割込み優先度マスクを0以外にさせる処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_set_ipm_fmp()
      cElement = IMCodeElement.new()

      @aPrcid.each{ |nPrcid|
        # 割込み優先度マスクが指定されている場合のみ
        if (!@aCpuState[nPrcid].nil?() &&
            !@aCpuState[nPrcid].hState[TSR_PRM_CHGIPM].nil?() &&
            (@aCpuState[nPrcid].hState[TSR_PRM_CHGIPM] != 0) && (@aCpuState[nPrcid].hState[TSR_PRM_CHGIPM] != KER_TIPM_ENAALL))

          # 以下のいずれかを満たす場合のみ割込み優先度マスクを変更する
          # 1)ACTIVATEな非タスクが存在しない
          # 2)ACTIVATEな非タスクがCPU例外ハンドラである
          if (@aActivate[nPrcid].nil?() || (@aActivate[nPrcid].sObjectType == TSR_OBJ_EXCEPTION))
            hProcUnitInfo = get_proc_unit_info(@aRunning[nPrcid])
            cElement.set_syscall(hProcUnitInfo, "#{API_CHG_IPM}(#{@aCpuState[nPrcid].hState[TSR_PRM_CHGIPM]})")
          end
        end
      }

      return cElement # [IMCodeElement]割込み優先度マスクを0以外にさせる処理コード
    end


    #=================================================================
    # 概  要: 一時的にSLEEPまたはDELAYさせたタスクを実行順番に起こす
    #         処理をIMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_ready_porder_fmp()
      # 一時スリープさせておいたタスクのporderを確認し，
      # 実行可能状態にさせる

      # 実行状態のタスクがいる場合  ：実行状態のタスクから設定
      # 実行状態のタスクがいない場合：非タスクから設定
      cElement  = IMCodeElement.new()  # elementクラスを用意する

      aPriorityTemp = []  # 優先順位のないタスクを保持するための配列
      aPriOrderTemp = []  # 優先順位のあるタスクを順で保持するための配列

      @aTask.each{|nPrcid, sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RDY)
          nPOrder = cObjectInfo.hState[TSR_PRM_PORDER]

          if (aPriorityTemp[nPrcid].nil?())
            aPriorityTemp[nPrcid] = []
          end
          if (aPriOrderTemp[nPrcid].nil?())
            aPriOrderTemp[nPrcid] = []
          end

          if (nPOrder.nil?())
            aPriorityTemp[nPrcid].push(sObjectID)
          else
            aPriOrderTemp[nPrcid][nPOrder] = sObjectID  # 優先順位の順で挿入
          end
        end
      }

      aPriOrderTemp.each{ |aPriPrc|
        if (!aPriPrc.nil?())
          aPriPrc.delete(nil)  # 必要ではないnilは削除
        end
      }

      @aPrcid.each{ |nPrcid|
        if (!aPriorityTemp[nPrcid].nil?())
          aPriOrderTemp[nPrcid].concat(aPriorityTemp[nPrcid])
        end
      }

      hProcUnitInfo = get_proc_unit_info(@cActivate, @cRunning)
      aPriOrderTemp.each{|aPriPrc|
        if (!aPriPrc.nil?())
          aPriPrc.each{|sObjectID|
            if (!@cActivate.nil?())
              cElement.set_syscall(hProcUnitInfo, "#{API_IWUP_TSK}(#{sObjectID})")
            else
              cElement.set_syscall(hProcUnitInfo, "#{API_WUP_TSK}(#{sObjectID})")
            end
          }
        end
      }

      cElement.set_checkpoint(hProcUnitInfo)

      return cElement # [IMCodeElement]一時的にSLEEPまたはDELAYさせたタスクを実行順番に起こす処理コード
    end


    #=================================================================
    # 概  要: 起動・起床要求キューイングの設定をする処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_task_queueing_fmp()
      cElement      = IMCodeElement.new()  # elementクラスを用意する
      hProcUnitInfo = get_proc_unit_info(@cActivate, @cRunning)

      @aTask.each{|nPrcid, sObjectID, cObjectInfo|
        # 起動要求キューイングはFMPでは1固定
        if (!cObjectInfo.hState[TSR_PRM_ACTCNT].nil?() && cObjectInfo.hState[TSR_PRM_ACTCNT] == 1)
          # 次回割付プロセッサIDがあればmact_tskで設定する
          if (!cObjectInfo.hState[TSR_PRM_ACTPRC].nil?())
            if (!@cActivate.nil?())
              cElement.set_syscall(hProcUnitInfo, "#{API_IMACT_TSK}(#{sObjectID}, #{cObjectInfo.hState[TSR_PRM_ACTPRC]})")
            else
              cElement.set_syscall(hProcUnitInfo, "#{API_MACT_TSK}(#{sObjectID}, #{cObjectInfo.hState[TSR_PRM_ACTPRC]})")
            end
          else
            if (!@cActivate.nil?())
              cElement.set_syscall(hProcUnitInfo, "#{API_IACT_TSK}(#{sObjectID})")
            else
              cElement.set_syscall(hProcUnitInfo, "#{API_ACT_TSK}(#{sObjectID})")
            end
          end
        end

        if(!cObjectInfo.hState[TSR_PRM_WUPCNT].nil?() && cObjectInfo.hState[TSR_PRM_WUPCNT] >= 1)
          cObjectInfo.hState[TSR_PRM_WUPCNT].times{
            if (!@cActivate.nil?())
              cElement.set_syscall(hProcUnitInfo, "#{API_IWUP_TSK}(#{sObjectID})")
            else
              cElement.set_syscall(hProcUnitInfo, "#{API_WUP_TSK}(#{sObjectID})")
            end
          }
        end
      }

      cElement.set_checkpoint(hProcUnitInfo)

      return cElement # [IMCodeElement]起動・起床要求キューイングの設定をする処理コード
    end


    #=================================================================
    # 概  要: 停止中のタイムイベントハンドラを設定するコードを返す
    #=================================================================
    def gc_pre_time_event_stp_other_fmp()
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()
      bFlg = false

      # 初期割付プロセッサがメインプロセッサでない場合に備え，一律マイグレートする
      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((GRP_TIME_EVENT_HDL.include?(cObjectInfo.sObjectType) == true) &&
            (GRP_TIME_EVENT_STP.include?(cObjectInfo.hState[TSR_PRM_STATE]) == true) &&
            (cObjectInfo.hState[TSR_PRM_HDLSTAT] == TSR_STT_STP))
          if (cObjectInfo.sObjectType == TSR_OBJ_ALARM)
            cElement.set_syscall(hMainTaskInfo, "#{API_MSTA_ALM}(#{sObjectID}, #{TTG_ENOUGH_MIG_TIME}, #{cObjectInfo.hState[TSR_PRM_PRCID]})")
            cElement.set_syscall(hMainTaskInfo, "#{API_STP_ALM}(#{sObjectID})")
          elsif (cObjectInfo.sObjectType == TSR_OBJ_CYCLE)
            cElement.set_syscall(hMainTaskInfo, "#{API_MSTA_CYC}(#{sObjectID}, #{cObjectInfo.hState[TSR_PRM_PRCID]})")
            cElement.set_syscall(hMainTaskInfo, "#{API_STP_CYC}(#{sObjectID})")
          end

          bFlg = true
        end
      }

      cElement.set_checkpoint(hMainTaskInfo)

      if (bFlg == false)
        return nil # [nil]追加の必要なし
      else
        return cElement # [IMCodeElement]停止中のタイムイベントハンドラを設定するコード
      end
    end


    #=================================================================
    # 概  要: 動作状態(Txxx_STA)のタイムイベントハンドラを設定する
    #         コードを返す
    #=================================================================
    def gc_pre_time_event_sta_fmp()
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((GRP_TIME_EVENT_HDL.include?(cObjectInfo.sObjectType) == true) &&
            (GRP_TIME_EVENT_STA.include?(cObjectInfo.hState[TSR_PRM_STATE]) == true) &&
            (cObjectInfo.hState[TSR_PRM_HDLSTAT] != TSR_STT_ACTIVATE))
          if (cObjectInfo.sObjectType == TSR_OBJ_ALARM)
            if (@cConf.is_timer_local?())
              cElement.set_syscall(hMainTaskInfo, "#{API_MSTA_ALM}(#{sObjectID}, #{cObjectInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick}, #{cObjectInfo.hState[TSR_PRM_PRCID]})")
            else
              cElement.set_syscall(hMainTaskInfo, "#{API_STA_ALM}(#{sObjectID}, #{cObjectInfo.hState[TSR_PRM_LEFTTMO].to_i + @nPreGainTick})")
            end
          elsif (cObjectInfo.sObjectType == TSR_OBJ_CYCLE)
            if (@cConf.is_timer_local?())
              cElement.set_syscall(hMainTaskInfo, "#{API_MSTA_CYC}(#{sObjectID}, #{cObjectInfo.hState[TSR_PRM_PRCID]})")
            else
              cElement.set_syscall(hMainTaskInfo, "#{API_STA_CYC}(#{sObjectID})")
            end
          end
        end
      }

      cElement.set_checkpoint(hMainTaskInfo)

      return cElement # [IMCodeElement]動作状態(Txxx_STA)のタイムイベントハンドラを設定するコード
    end


    #=================================================================
    # 概  要: スピンロック状態の設定(指定された処理単位から設定)処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_spin_loc_fmp()
      cElement = IMCodeElement.new()

      # メインプロセッサ準備完了確認用チェックポイント
      cElement.set_checkpoint(get_proc_unit_info(@cActivate, @cRunning))
      bFlg = false

      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((cObjectInfo.sObjectType == TSR_OBJ_SPINLOCK) && (cObjectInfo.hState[TSR_PRM_SPNSTAT] == TSR_STT_TSPN_LOC))
          cObject = get_object_info(cObjectInfo.hState[TSR_PRM_PROCID])
          hProcUnitInfo = get_proc_unit_info(cObject)

          # メインプロセッサが他の処理を完了したことを待ってからロックする
          # (CPUロックとなると時間が進められない)
          cElement.set_wait_check_sync(hProcUnitInfo, @sMainPrcid)

          if (GRP_NON_CONTEXT.include?(cObject.sObjectType))
            cElement.set_syscall(hProcUnitInfo, "#{API_ILOC_SPN}(#{sObjectID})")
          else
            cElement.set_syscall(hProcUnitInfo, "#{API_LOC_SPN}(#{sObjectID})")
          end
          cElement.set_checkpoint(hProcUnitInfo)

          # 他プロセッサの場合，スピンロックを取得したことをメインプロセッサでも待つ
          # (先にrefされるのを防ぐ)
          if (cObject.hState[TSR_PRM_PRCID] != @sMainPrcid)
            cElement.set_wait_check_sync(get_proc_unit_info(@cActivate, @cRunning), cObject.hState[TSR_PRM_PRCID])
          end

          # CPUロックを重複して実行しないため，保持しておく
          @aSpinProcID.push(cObject.sObjectID)

          bFlg = true
        end
      }

      if (bFlg == false)
        return nil # [nil]追加の必要なし
      else
        return cElement # [IMCodeElement]スピンロック状態の設定(指定された処理単位から設定)処理コード
      end
    end


    #=================================================================
    # 概  要: CPUロック状態の設定(実行中の処理単位から設定)処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_pre_cpu_loc_fmp()
      # 起動状態の非タスクがいる場合，アラーム・周期ハンドラ両方
      # いる場合はアラームにさせる
      # また，起動状態の非タスクがいない，かつ実行状態のタスクが
      # いる場合は，タスクにさせる

      cElement = IMCodeElement.new()

      # メインプロセッサ準備完了確認用チェックポイント
      cElement.set_checkpoint(get_proc_unit_info(@cActivate, @cRunning))
      bFlg = false

      @aPrcid.each{ |nPrcid|
        # check処理も兼ねて行う
        if (!@aCpuState[nPrcid].nil?() && !@aCpuState[nPrcid].hState[TSR_PRM_LOCCPU].nil?() && (@aCpuState[nPrcid].hState[TSR_PRM_LOCCPU] == true))
          hProcUnitInfo = get_proc_unit_info(@aActivate[nPrcid], @aRunning[nPrcid])

          # スピンロックによるCPUロックは除外
          if (!@aSpinProcID.include?(hProcUnitInfo[:id]))
            # メインプロセッサが他の処理を完了したことを待ってからロックする
            # (CPUロックとなると時間が進められない)
            cElement.set_wait_check_sync(hProcUnitInfo, @sMainPrcid)
            bFlg = true

            if (!@aActivate[nPrcid].nil?())
              cElement.set_syscall(hProcUnitInfo, "#{API_ILOC_CPU}()")
            else
              cElement.set_syscall(hProcUnitInfo, "#{API_LOC_CPU}()")
            end
            cElement.set_checkpoint(hProcUnitInfo)
          end
        end
      }

      if (bFlg == false)
        return nil # [nil]追加の必要なし
      else
        return cElement # [IMCodeElement]CPUロック状態の設定(実行中の処理単位から設定)処理
      end
    end

    #=================================================================
    # 概  要: 許可状態の割込みを設定するコードを返す
    #=================================================================
    def gc_pre_interrupt_ena_fmp()
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      # ena_intする必要のある割込み番号とプロセッサIDを抽出
      aEnaIntNoID = []
      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((GRP_INTERRUPT.include?(cObjectInfo.sObjectType) == true) &&
            (cObjectInfo.hState[TSR_PRM_STATE] == KER_TA_ENAINT))
          aEnaIntNoID.push([cObjectInfo.hState[TSR_PRM_INTNO], cObjectInfo.hState[TSR_PRM_PRCID]])
        end
      }

      # 重複を削除してすべてena_intを実行する
      nMainPrcid = @sMainPrcid
      aEnaIntNoID.uniq!()
      aEnaIntNoID.each{|snIntNoID|
        # メインタスクと対象IntHdr/ISRのプロセッサが異なる場合はメインタスクを移動
        if (nMainPrcid != snIntNoID[1])
          cElement.set_syscall(hMainTaskInfo, "#{API_MIG_TSK}(#{TTG_TSK_SELF}, #{snIntNoID[1]})")
          nMainPrcid = snIntNoID[1]
        end

        cElement.set_syscall(hMainTaskInfo, "#{API_ENA_INT}(#{snIntNoID[0]})")
      }

      # 最後は必ずメインプロセッサに戻る
      if (nMainPrcid != @sMainPrcid)
        cElement.set_syscall(hMainTaskInfo, "#{API_MIG_TSK}(#{TTG_TSK_SELF}, #{@sMainPrcid})")
      end

      cElement.set_checkpoint(hMainTaskInfo)

      return cElement # [IMCodeElement]許可状態の割込みを設定するコード
    end

  end
end
