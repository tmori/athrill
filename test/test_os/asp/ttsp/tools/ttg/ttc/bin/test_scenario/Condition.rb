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
require "ttc/bin/class/TTCCommon.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: Condition
  # 概    要: pre_condition, post_conditionの情報を処理するクラス
  #===================================================================
  class Condition
    include TTCModule

    #=================================================================
    # 概　要: オブジェクトの共通基礎チェック
    #=================================================================
    def common_basic_check(sObjectID, hObjectInfo, aPath)
      check_class(Object, sObjectID, true)     # オブジェクトID
      check_class(Object, hObjectInfo, true)   # コンディション
      check_class(Array, aPath)                # ルートからのパス

      aErrors = []
      ### T1_015: オブジェクトIDの文字列が不正
      unless (sObjectID =~ TSR_REX_OBJECT_ID)
        sErr = sprintf("T1_015: " + ERR_INVALID_OBJECTID, sObjectID)
        aErrors.push(YamlError.new(sErr, aPath))
      end
      ### T1_016: オブジェクトがHashではない
      unless (hObjectInfo.is_a?(Hash))
        sErr = sprintf("T1_016: " + ERR_INVALID_TYPE, sObjectID, Hash, hObjectInfo.class())
        aErrors.push(YamlError.new(sErr, aPath + [sObjectID]))
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概  要: 属性チェック
    #=================================================================
    def attribute_check()
      aErrors = []
      @hAllObject.each{|sObjectID, cObjectInfo|
        begin
          cObjectInfo.attribute_check()
        rescue TTCMultiError
          aErrors.concat($!.aErrors)
        end
      }
      check_error(aErrors)
    end

    #=================================================================
    # 概  要: オブジェクトチェック
    #=================================================================
    def object_check(bIsPre = false)
      check_class(Bool, bIsPre)  # pre_conditionか

      aErrors = []
      @hAllObject.each{|sObjectID, cObjectInfo|
        begin
          cObjectInfo.object_check(bIsPre)
        rescue TTCMultiError
          aErrors.concat($!.aErrors)
        end
      }
      check_error(aErrors)
    end

    #=================================================================
    # 概  要: コンディションチェック
    #=================================================================
    def condition_check()
      aErrors = []
      aPath = get_condition_path()

      # CPU_STATEのチェック
      aCpuStateCount = []
      aCpuState      = []
      hObjects       = get_objects_by_type(TSR_OBJ_CPU_STATE)
      hObjects.each_value{|cObjectInfo|
        nPrcid = cObjectInfo.get_process_id()
        if (aCpuStateCount[nPrcid].nil?())
          aCpuStateCount[nPrcid] = 1
        else
          aCpuStateCount[nPrcid] += 1
        end
        aCpuState[nPrcid] = cObjectInfo

=begin
        # loc_id
        unless (cObjectInfo.hState[TSR_PRM_LOC_ID].nil?())
          sLocID = cObjectInfo.hState[TSR_PRM_LOC_ID]
          if (@hAllObject.has_key?(sLocID))
            cProcUnit = @hAllObject[sLocID]
            if (GRP_PROCESS_UNIT_ALL.include?(cProcUnit.sObjectType))
              ### T4_043: loc_idで指定された処理単位が実行状態でない
              unless (cProcUnit.is_activate?())
                sErr = sprintf(ERR_TARGET_NOT_ACTIVATE, sLocID)
                aErrors.push(YamlError.new("T4_043: " + sErr, aPath))
              end
            ### T4_042: loc_idで指定されたオブジェクトが処理単位でない
            else
              sErr = sprintf(ERR_TARGET_NOT_PROCESS_UNIT, sLocID)
              aErrors.push(YamlError.new("T4_042: " + sErr, aPath))
            end
          ### T4_041: loc_idで指定されたオブジェクトが存在しない
          else
            sErr = sprintf(ERR_TARGET_NOT_DEFINED, sLocID)
            aErrors.push(YamlError.new("T4_041: " + sErr, aPath))
          end
        end
=end
      }
      # チェック
      aCpuStateCount.each_with_index{|nCount, nPrcid|
        ### T4_006: CPU_STATEが1つのプロセッサに2つ以上存在している
        if (!nCount.nil?() && nCount > 1)
          sErr = sprintf("T4_006: " + ERR_PLURAL_CPU_STATE, nPrcid)
          aErrors.push(YamlError.new(sErr, aPath))
        end
      }

      # 処理単位のチェック
      hActivate = Hash.new{|hash, key|
        hash[key] = {
          :nTask         => 0,  # 実行中のタスクの数
          :nNonContext   => 0,  # 実行中の非タスクコンテキストの数
          :nTaskExc      => 0,  # 実行中のタスク例外の数
          :nException    => 0   # 実行中のCPU例外の数
        }
      }
      (1..@cConf.get_prc_num()).each{|nPrcid|
        hActivate[nPrcid]
      }

      hAllVariable = {}
      aActivate = get_activate_by_prcid()
      hObjects = get_objects_by_type(GRP_PROCESS_UNIT_ALL)
      hObjects.each{|sObjectID, cObjectInfo|
        # プロセッサ番号取得
        nPrcid = get_process_prcid(cObjectInfo)
        if (nPrcid.nil?())
          next
        end
        # 起動中の処理単位
        if (cObjectInfo.is_activate?())
          # 起動中の処理単位の判別
          if (cObjectInfo.sObjectType == TSR_OBJ_TASK)
            hActivate[nPrcid][:nTask] += 1
          elsif (cObjectInfo.sObjectType == TSR_OBJ_TASK_EXC)
            hActivate[nPrcid][:nTaskExc] += 1
          elsif (cObjectInfo.sObjectType == TSR_OBJ_EXCEPTION)
            hActivate[nPrcid][:nException] += 1
          end
          if (GRP_NON_CONTEXT.include?(cObjectInfo.sObjectType))
            hActivate[nPrcid][:nNonContext] += 1
          end
        end
        # 変数の定義
        unless (cObjectInfo.hState[TSR_PRM_VAR].nil?())
          hAllVariable[sObjectID] = cObjectInfo.hState[TSR_PRM_VAR]
          # 値がセットされた変数のチェック
          if (cObjectInfo.is_value_set_variable?())
            ### T4_007: 実行状態でない処理単位に変数のvalueが定義されている
            if (cObjectInfo.is_activate?())
              if (!aActivate[nPrcid].nil?() && aActivate[nPrcid] != cObjectInfo)
                aErrors.push(YamlError.new("T4_007: " + ERR_SET_VALUE_NON_ACTIVATE, aPath + [cObjectInfo.sObjectID]))
              end
            else
              aErrors.push(YamlError.new("T4_007: " + ERR_SET_VALUE_NON_ACTIVATE, aPath + [cObjectInfo.sObjectID]))
            end
          end
        end
      }

      # 起動中の処理単位チェック
      hActivate.each{|nPrcid, hCheck|
        ### T4_008: 同一コンディション内で実行状態のタスクが複数存在する
        if (hActivate[nPrcid][:nTask] > 1)
          sErr = sprintf("T4_008: " + ERR_PLURAL_RUNNING_TASK, nPrcid)
          aErrors.push(YamlError.new(sErr, aPath))
        end
        ### T4_009: 同一コンディション内で起動中の非タスクコンテキストが複数存在する
        if (hActivate[nPrcid][:nNonContext] > 1)
          sErr = sprintf("T4_009: " + ERR_PLURAL_ACTIVATE_NON_CONTEXT, nPrcid)
          aErrors.push(YamlError.new(sErr, aPath))
        end
        # CPU_STATEが関連するチェック
        unless (aCpuState[nPrcid].nil?())
          ### T4_010: 実行状態の処理単位が存在しない状態でCPU状態(CPUロック状態 | ディスパッチ禁止状態 | 割込み禁止マスク != 0)を指定している
          aValues = hActivate[nPrcid].values()
          if (aValues.max() == 0 && aCpuState[nPrcid].is_state_changed?())
            sErr = sprintf("T4_010: " + ERR_CANNNOT_REF_CPU_STATE, nPrcid)
            aErrors.push(YamlError.new(sErr, aPath))
          end
=begin
          ### T4_011: アラームハンドラしかいないのにCPUロック以外のCPU状態が設定されている
          if (hActivate[nPrcid][:nAlarm] > 0 && hActivate[nPrcid][:nNonAlarm] == 0 &&
              (aCpuState[nPrcid].is_disable_dispatch?() || !aCpuState[nPrcid].is_enable_interrupt?()))
            sErr = sprintf("T4_011: " + ERR_ONLY_ALARM_SET_CPU_STATE, nPrcid)
            aErrors.push(YamlError.new(sErr, aPath))
          end

          ### T4_012: タスクかアラームが動作していないのにCPUロック状態が設定されている
          if (hActivate[nPrcid][:nTask] == 0 && hActivate[nPrcid][:nAlarm] == 0 && aCpuState[nPrcid].is_cpu_lock?())
            sErr = sprintf("T4_012: " + ERR_CPULOCK_NOT_TASK_ALARM_RUN, nPrcid)
            aErrors.push(YamlError.new(sErr, aPath))
          end

          # loc_id
          unless (aCpuState[nPrcid].hState[TSR_PRM_LOC_ID].nil?())
            ### T4_044: loc_idが指定されたCPU_STATEと同じプロセッサにACTIVATEなCPU例外ハンドラが存在しない
            if (hActivate[nPrcid][:nException] == 0)
              aErrors.push(YamlError.new("T4_044: " + ERR_NO_EXCEPTION_SET_LOC_ID, aPath))
            end
            ### T4_045: loc_idが指定されたCPU_STATEと同じプロセッサにCPU例外ハンドラ以外の実行中の処理単位が存在しない
            if (hActivate[nPrcid][:nNonException] == 0)
              aErrors.push(YamlError.new("T4_045: " + ERR_NO_PROCESS_UNIT_SET_LOC_ID, aPath))
            end
          end
=end
        end
      }


      # タスク
      aPOrder = []
      hObjects = get_objects_by_type(TSR_OBJ_TASK)
      hObjects.each{|sObjectID, cObjectInfo|
        ### T4_013: 待ち状態における待ち対象オブジェクトが存在しない
        if (cObjectInfo.has_wait_object?())
          sWaitObjID = cObjectInfo.hState[TSR_PRM_WOBJID]
          unless (@hAllObject.has_key?(sWaitObjID))
            sErr = sprintf("T4_013: " + ERR_NO_WAITING_OBJECT, sWaitObjID)
            aErrors.push(YamlError.new(sErr, aPath + [sObjectID, TSR_PRM_WOBJID]))
          end
        end
        # porderチェック準備
        if (cObjectInfo.is_activate?() || cObjectInfo.is_ready?())
          nPrcid = get_process_prcid(cObjectInfo)
          nPri   = cObjectInfo.hState[TSR_PRM_TSKPRI]
          unless (cObjectInfo.hState[TSR_PRM_PORDER].nil?())
            nPOrder = cObjectInfo.hState[TSR_PRM_PORDER]
            if (aPOrder[nPrcid].nil?())
              aPOrder[nPrcid] = []
            end
            if (aPOrder[nPrcid][nPri].nil?())
              aPOrder[nPrcid][nPri] = []
            end
            if (aPOrder[nPrcid][nPri][nPOrder].nil?())
              aPOrder[nPrcid][nPri][nPOrder] = []
            end
            aPOrder[nPrcid][nPri][nPOrder].push(sObjectID)
          end
        end
      }
      # porderチェック
      aPOrder.each_with_index{|aPriByPrcid, nPrcid|
        unless (aPriByPrcid.nil?())
          aPriByPrcid.each_with_index{|aPri, nPri|
            unless (aPri.nil?())
              bCheckFlag = false
              aPri.each_with_index{|aTasks, nPorder|
                if (nPorder != 0)
                  ### T4_014: 同一優先度内でporderが連番でない
                  if (aTasks.nil?())
                    if (bCheckFlag == true)
                      sErr = sprintf("T4_014: " + ERR_PORDER_NOT_SEQUENCE, nPri, nPrcid)
                      aErrors.push(YamlError.new(sErr, aPath))
                    end
                  ### T4_015: 同一優先度内でporderがユニークでない
                  else
                    bCheckFlag = true
                    if (aTasks.size() > 1)
                      sErr = sprintf("T4_015: " + ERR_PORDER_NOT_UNIQUE, aTasks.join(", "))
                      aErrors.push(YamlError.new(sErr, aPath))
                    end
                  end
                end
              }
            end
          }
        end
      }


      # CPU例外ハンドラ
      hException = Hash.new{|hash, snExcno|
        hash[snExcno] = []
      }
      hObjects = get_objects_by_type(TSR_OBJ_EXCEPTION)
      hObjects.each{|sObjectID, cObjectInfo|
        # activateな場合
        if (cObjectInfo.is_activate?())
          nPrcid = cObjectInfo.get_process_id()
          ### T4_016: ACTIVATEであるCPU例外ハンドラが存在するプロセッサに，実行中のタスクかタスク例外が存在しない
          if (hActivate[nPrcid][:nTask] == 0 && hActivate[nPrcid][:nTaskExc] == 0)
            sErr = sprintf("T4_016: " + ERR_NO_TASK_CONTEXT_ON_OBJ, TSR_OBJ_EXCEPTION)
            aErrors.push(YamlError.new(sErr, aPath))
          end
        end
        # CPU例外ハンドラ番号
        snExcno = cObjectInfo.hState[TSR_PRM_EXCNO]
        hException[snExcno].push(sObjectID)
      }
      # チェック
      hException.each{|snExcno, aObjectID|
        ### T4_017: 同一CPU例外ハンドラ番号に対するCPU例外ハンドラが複数定義されている
        if (aObjectID.size() > 1)
          sErr = sprintf("T4_017: " + ERR_DUPLICATE_EXCNO, snExcno, aObjectID.join(", "))
          aErrors.push(YamlError.new(sErr, aPath))
        end
      }


      # タスク例外
      hObjects = get_objects_by_type(TSR_OBJ_TASK_EXC)
      hObjects.each{|sObjectID, cObjectInfo|
        # 関連タスク
        cTask = @hAllObject[cObjectInfo.hState[TSR_PRM_TASK]]
        if (!cTask.nil?() && cTask.sObjectType == TSR_OBJ_TASK)
          if (cTask.is_dormant?())
            ### T4_018: 関連タスクの状態がdormantであるのにタスク例外が有効
            if (cObjectInfo.is_activate?())
              sErr = sprintf("T4_018: " + ERR_TASK_EXC_CANNOT_ACTIVATE, cTask.sObjectID)
              aErrors.push(YamlError.new(sErr, aPath + [sObjectID]))
            end
            # パラメータ指定チェック
            cObjectInfo.hState.each_key{|sAtr|
              ### T4_019: 関連タスクがdormantである場合に，指定不可なパラメータが指定されている
              sRealAtr = cObjectInfo.get_real_attribute_name(sAtr)
              if (cObjectInfo.is_specified?(sRealAtr) && !GRP_ACTIVATE_PRM_ON_DORMANT[TSR_OBJ_TASK_EXC].include?(sRealAtr))
                sErr = sprintf("T4_019: " + ERR_ATR_DORMANT_TASK_EXC, sRealAtr)
                aErrors.push(YamlError.new(sErr, aPath + [sObjectID]))
              end
            }
          ### T4_020: 関連タスクがdormantでない場合にtexstatが指定されていない
          elsif (cObjectInfo.hState[TSR_PRM_STATE].nil?())
            sErr = sprintf("T4_020: " + ERR_NO_TEXSTAT_STT_NOT_DORMANT, cTask.sObjectID)
            aErrors.push(YamlError.new(sErr, aPath + [sObjectID]))
          end
        end
      }


      # 割込みハンドラ，割込みサービスルーチン
      hIntno = Hash.new{|hash, snIntno|
        hash[snIntno] = {
          TSR_OBJ_INTHDR => [],
          TSR_OBJ_ISR    => []
        }
      }
      hObjects = get_objects_by_type([TSR_OBJ_INTHDR, TSR_OBJ_ISR])
      hObjects.each{|sObjectID, cObjectInfo|
        # 割込み番号ごとにまとめる
        snIntno = cObjectInfo.hState[TSR_PRM_INTNO]
        hIntno[snIntno][cObjectInfo.sObjectType].push(cObjectInfo)
        ### T4_038: ACTIVATEである割込みハンドラが存在するプロセッサに，実行中のタスクかタスク例外が存在しない
        ### T4_039: ACTIVATEである割込みサービスルーチンが存在するプロセッサに，実行中のタスクかタスク例外が存在しない
        if (cObjectInfo.is_activate?())
          nPrcid = get_process_prcid(cObjectInfo)
          if (hActivate[nPrcid][:nTask] == 0 && hActivate[nPrcid][:nTaskExc] == 0)
            sErr = sprintf(ERR_NO_TASK_CONTEXT_ON_OBJ, cObjectInfo.sObjectType)
            if (cObjectInfo.sObjectType == TSR_OBJ_INTHDR)
              sErr = "T4_038: " + sErr
            else
              sErr = "T4_039: " + sErr
            end
            aErrors.push(YamlError.new(sErr, aPath))
          end
        end
      }
      # 割込み番号に着目したチェック
      hIntno.each{|snIntno, hItem|
        ### T4_023: intnoが同じであるINTHDRが複数定義されている
        if (hItem[TSR_OBJ_INTHDR].size() > 1)
          aObjectID = []
          hItem[TSR_OBJ_INTHDR].each{|cObjectInfo|
            aObjectID.push(cObjectInfo.sObjectID)
          }
          sErr = sprintf("T4_023: " + ERR_INTHDR_DUPLICATE, snIntno, aObjectID.join(", "))
          aErrors.push(YamlError.new(sErr, aPath))
        end
        ### T4_025: intnoが同じであるINTHDRとISRが定義されている
        if (hItem[TSR_OBJ_INTHDR].size() > 0 && hItem[TSR_OBJ_ISR].size() > 0)
          sErr = sprintf("T4_025: " + ERR_INTNO_DUPLICATE, snIntno)
          aErrors.push(YamlError.new(sErr, aPath))
        end
        # intstatのチェック
        if (hItem[TSR_OBJ_ISR].size() > 1)
          sIntStat = nil
          hItem[TSR_OBJ_ISR].each{|cObjectInfo|
            if (sIntStat.nil?())
              sIntStat = cObjectInfo.hState[TSR_PRM_STATE]
            ### T4_024: intnoが同じであるISRが複数定義されている場合にintstatが一致していない
            elsif (sIntStat != cObjectInfo.hState[TSR_PRM_STATE])
              sErr = sprintf("T4_024: " + ERR_ISR_PRM_MISMATCH, snIntno)
              aErrors.push(YamlError.new(sErr, aPath))
            end
          }
        end
      }


      # 同期通信オブジェクト
      aTaskListAtr = [TSR_PRM_WTSKLIST, TSR_PRM_STSKLIST, TSR_PRM_RTSKLIST]
      hObjects = get_objects_by_type(GRP_SC_OBJECT)
      hObjects.each{|sObjectID, cObjectInfo|
        # タスクリストのチェック
        aTaskListAtr.each{|sAtr|
          unless (cObjectInfo.hState[sAtr].nil?())
            cObjectInfo.hState[sAtr].each{|hTask|
              hTask.each{|sTask, hVar|
                # 登録されたタスクのチェック
                if (@hAllObject.has_key?(sTask))
                  cTask = @hAllObject[sTask]
                  if (cTask.sObjectType == TSR_OBJ_TASK)
                    # タスクの状態チェック
                    if (cTask.is_object_waiting?())
                      ### T4_026: 待ちタスクリストに登録されたタスクの待ち対象がこの待ちタスクリストを
                      ###       : 持つオブジェクトと異なる
                      if (cTask.hState[TSR_PRM_WOBJID] != sObjectID)
                        sErr = sprintf("T4_026: " + ERR_TSKLIST_TASK_TARGET_MISMATCH, sTask, sObjectID)
                        aErrors.push(YamlError.new(sErr, aPath + [sObjectID, sAtr]))
                      end
                    ### T4_027: 待ちタスクリストに登録されたタスクが待ち状態か二重待ち状態でない
                    else
                      sErr = sprintf("T4_027: " + ERR_TSKLIST_TASK_NOT_WAITING, sTask)
                      aErrors.push(YamlError.new(sErr, aPath + [sObjectID, sAtr]))
                    end
                  ### T4_029: 待ちタスクリストに登録されたオブジェクトがタスクでない
                  else
                    sErr = sprintf("T4_029: " + ERR_TARGET_NOT_TASK, sTask)
                    aErrors.push(YamlError.new(sErr, aPath + [sObjectID, sAtr]))
                  end
                ### T4_028: 待ちタスクリストに登録されたオブジェクトが存在しない
                else
                  sErr = sprintf("T4_028: " + ERR_TARGET_NOT_DEFINED, sTask)
                  aErrors.push(YamlError.new(sErr, aPath + [sObjectID, sAtr]))
                end
              }
            }
          end

          # 変数のチェック
          # wtsklist，rtsklist
          aTaskListVars = []
          aTaskListVars.push(cObjectInfo.get_rtsklist_variable())
          aTaskListVars.push(cObjectInfo.get_wtsklist_variable())
          aTaskListVars.each{|hVars|
            hVars.each{|sTask, hVar|
              if (@hAllObject.has_key?(sTask) && @hAllObject[sTask].sObjectType == TSR_OBJ_TASK)
                hVar.each{|sVarName, aType|
                  hVarAtr = @hAllObject[sTask].hState[TSR_PRM_VAR]
                  if (!hVarAtr.nil?() && hVarAtr.has_key?(sVarName))
                    ### T4_030: 待ちタスクリストに登録された待ち条件用変数が適切な型でない
                    unless (aType.include?(hVarAtr[sVarName].sType))
                      sErr = sprintf("T4_030: " + ERR_VARIABLE_TYPE_MISMATCH, sVarName, sTask, aType.join(", "))
                      aErrors.push(YamlError.new(sErr, aPath + [sObjectID, TSR_PRM_RTSKLIST, sTask]))
                    end
                  ### T4_031: 待ちタスクリストに登録された待ち条件用変数がタスクで宣言されていない
                  else
                    sErr = sprintf("T4_031: " + ERR_VARIABLE_NOT_DEFINED, sVarName, sTask)
                    aErrors.push(YamlError.new(sErr, aPath + [sObjectID, TSR_PRM_RTSKLIST, sTask]))
                  end
                }
              end
            }
          }
          # msglist
          hVar = cObjectInfo.get_msglist_variable()
          hVar.each{|sVarName, aType|
            hAllVariable.each{|sProcObjectID, hProcVar|
              if (hProcVar.has_key?(sVarName))
                cVar = hProcVar[sVarName]
                ### T4_035: msglistのメッセージ受信用変数が適切な型で宣言されていない
                unless (aType.include?(cVar.sType))
                  sErr = sprintf("T4_035: " + ERR_VARIABLE_TYPE_MISMATCH, sVarName, sProcObjectID, aType.join(", "))
                  aErrors.push(YamlError.new(sErr, aPath + [sObjectID, TSR_PRM_MSGLIST]))
                end
              end
            }
          }
        }

        # メモリプール
        if (cObjectInfo.sObjectType == TSR_OBJ_MEMORYPOOL && !cObjectInfo.hState[TSR_PRM_MPF].nil?())
          hAllVariable.each{|sProcObjectID, hVar|
            hVar.each{|sVarName, cVar|
              if (sVarName == cObjectInfo.hState[TSR_PRM_MPF])
                ### T4_032: メモリプールの先頭番地を代入する変数を持つ処理単位が実行状態でない時にその変数を確認しようとしている
                unless (@hAllObject[sProcObjectID].is_activate?())
                  sErr = sprintf("T4_032: " + ERR_MPF_NOT_ACTIVATE, sProcObjectID, cObjectInfo.hState[TSR_PRM_MPF])
                  aErrors.push(YamlError.new(sErr, aPath + [sObjectID, TSR_PRM_MPF]))
                end
                ### T4_033: メモリプールの先頭番地を代入する変数が適切な型でない
                if (cVar.sType != TYP_VOID_P)
                  sErr = sprintf("T4_033: " + ERR_VARIABLE_TYPE_MISMATCH, sVarName, sProcObjectID, TYP_VOID_P)
                  aErrors.push(YamlError.new(sErr, aPath + [sObjectID, TSR_PRM_MPF]))
                end
             end
            }
          }
        end
      }


      # FMP限定
      if (@cConf.is_fmp?())
        # スピンロック
        aSpinlock = []
        hObjects = get_objects_by_type(TSR_OBJ_SPINLOCK)
        hObjects.each{|sObjectID, cObjectInfo|
          if (cObjectInfo.is_lock?())
            sProcID = cObjectInfo.hState[TSR_PRM_PROCID]
            if (@hAllObject.has_key?(sProcID))
              cProcUnit = @hAllObject[sProcID]
              if (GRP_PROCESS_UNIT_ALL.include?(cProcUnit.sObjectType))
                nPrcid  = get_process_prcid(cProcUnit)
                unless (nPrcid.nil?())
                  aSpinlock[nPrcid] = true
                  # CPU_STATEが定義されているか
                  unless (aCpuState[nPrcid].nil?())
                    ### T4_F002: スピンロック中にCPUロックになっていない
                    unless (aCpuState[nPrcid].is_cpu_lock?())
                      sErr = sprintf("T4_F002: " + ERR_NOT_CPU_LOCK_IN_SPINLOCK, nPrcid)
                      aErrors.push(YamlError.new(sErr, aPath))
                    end
                  ### T4_F001: スピンロック中にCPU_STATEが定義されていない
                  else
                    sErr = sprintf("T4_F001: " + ERR_NO_CPU_STATE_IN_SPINLOCK, nPrcid)
                    aErrors.push(YamlError.new(sErr, aPath))
                  end
                end
              ### T4_F004 スピンロック取得元オブジェクトが処理単位ではない
              else
                sErr = sprintf("T4_F004: " + ERR_TARGET_NOT_PROCESS_UNIT, sProcID)
                aErrors.push(YamlError.new(sErr, aPath + [sObjectID, TSR_PRM_PROCID]))
              end
            ### T4_F003: スピンロック取得元オブジェクトが存在しない
            else
              sErr = sprintf("T4_F003: " + ERR_TARGET_NOT_DEFINED, sProcID)
              aErrors.push(YamlError.new(sErr, aPath + [sObjectID, TSR_PRM_PROCID]))
            end
          end
        }


        # 処理単位
        hObjects = get_objects_by_type(GRP_PROCESS_UNIT_ALL)
        hObjects.each{|sObjectID, cObjectInfo|
          # running-suspendedなタスク
          if (cObjectInfo.sObjectType == TSR_OBJ_TASK && cObjectInfo.is_running_suspended?())
            nPrcid    = cObjectInfo.get_process_id()
            cCpuState = aCpuState[nPrcid]
            ### T4_F005: タスク状態がrunning-suspendedの時に以下の条件のいずれも満たしていない
            ###         * CPU状態(ipm != 0 | loc_cpu = true | dis_dsp = true)
            ###         * スピンロック中
            ###         * ACTIVATEな非タスクが存在
            if (hActivate[nPrcid][:nNonContext] == 0 && aSpinlock[nPrcid] != true && (cCpuState.nil?() ||
                (cCpuState.is_enable_interrupt?() && !cCpuState.is_cpu_lock?() && !cCpuState.is_disable_dispatch?())))
              sErr = sprintf("T4_F005: " + ERR_CANNOT_RUNNING_SUSPENDED, nPrcid)
              aErrors.push(YamlError.new(sErr, aPath + [sObjectID]))
            end
          end
          # スピンロック待ち
          if (cObjectInfo.is_spinlock_waiting?())
            sSpinID = cObjectInfo.hState[TSR_PRM_SPINID]
            if (@hAllObject.has_key?(sSpinID))
              cSpinlock = @hAllObject[sSpinID]
              ### T4_F007: スピンロックIDのオブジェクトがスピンロックではない
              unless (cSpinlock.sObjectType == TSR_OBJ_SPINLOCK)
                sErr = sprintf("T4_F007: " + ERR_TARGET_NOT_SPINLOCK, sSpinID)
                aErrors.push(YamlError.new(sErr, aPath + [sObjectID, TSR_PRM_SPINID]))
              end
            ### T4_F006: スピンロックIDのオブジェクトが存在しない
            else
              sErr = sprintf("T4_F006: " + ERR_TARGET_NOT_DEFINED, sSpinID)
              aErrors.push(YamlError.new(sErr, aPath + [sObjectID, TSR_PRM_SPINID]))
            end
          end
        }
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概　要: 処理単位のプロセッサ番号を返す
    #=================================================================
    def get_process_prcid(cObjectInfo)
      check_class(Object, cObjectInfo, true)  # ProcessUnit

      nPrcid = nil
      unless (cObjectInfo.nil?())
        if (@cConf.is_asp?())
          nPrcid = 1
        elsif (@cConf.is_fmp?())
          # タスク例外の場合関連タスクのprcid
          if (cObjectInfo.sObjectType == TSR_OBJ_TASK_EXC)
            cTask = @hAllObject[cObjectInfo.hState[TSR_PRM_TASK]]
            unless (cTask.nil?())
              nPrcid = cTask.hState[TSR_PRM_PRCID]
            end
          else
            nPrcid = cObjectInfo.hState[TSR_PRM_PRCID]
          end
        end
      end

      return nPrcid  # [Integer,NilClass]プロセッサ番号
    end

    #=================================================================
    # 概　要: コンディション内の全てのマクロを置換する
    #=================================================================
    def convert_macro()
      @hAllObject.each_value{|cObjectInfo|
        cObjectInfo.convert_macro()
      }
    end

    #=================================================================
    # 概　要: エイリアスを実行する
    #=================================================================
    def alias(hAlias)
      check_class(Hash, hAlias)   # エイリアス変換テーブル

      hTmp        = @hAllObject.dup()
      @hAllObject = {}
      hTmp.each{|sObjectID, cObjectInfo|
        if (hAlias.has_key?(sObjectID))
          cObjectInfo.alias(hAlias)
          @hAllObject[hAlias[sObjectID]] = cObjectInfo
        else
          abort(ERR_MSG % [__FILE__, __LINE__])
        end
      }
    end

    #=================================================================
    # 概　要: 全補完終了後に実行する処理
    #=================================================================
    def complement_after()
      @hAllObject.each_value{|cObjectInfo|
        cObjectInfo.complement_after()
      }
    end

    #=================================================================
    # 概  要: コンディションの内容をYAMLオブジェクトに変換して返す
    #=================================================================
    def to_yaml(bIsPre = false)
      check_class(Bool, bIsPre)  # pre_conditionか

      hYaml = {}
      @hAllObject.each{|sObjectID, cObjectInfo|
        hYaml[sObjectID] = cObjectInfo.to_yaml(bIsPre)
      }

      return hYaml  # [Hash]YAMLオブジェクト
    end

    #=================================================================
    # 概  要: コンディション内に存在する変数名一覧を返す
    #=================================================================
    def get_variable_names()
      aVarNames = []
      @hAllObject.each_value{|cObjectInfo|
        aVarNames.concat(cObjectInfo.get_variable_names())
      }
      aVarNames = aVarNames.uniq()

      return aVarNames  # [Array]コンディション内に存在する変数名一覧
    end

    #=================================================================
    # 概  要: コンディション内に存在するプロセッサ番号一覧を返す
    #=================================================================
    def get_all_prcid()
      aPrcID = []
      @hAllObject.each_value{|cObjectInfo|
        unless (cObjectInfo.hState[TSR_PRM_PRCID].nil?())
          aPrcID.push(cObjectInfo.hState[TSR_PRM_PRCID])
        end
        unless (cObjectInfo.hState[TSR_PRM_ACTPRC].nil?())
          aPrcID.push(cObjectInfo.hState[TSR_PRM_ACTPRC])
        end
      }

      return aPrcID.compact().uniq()  # [Array]コンディション内に存在するプロセッサ番号一覧
    end

    #=================================================================
    # 概  要: プロセッサ番号ごとに起動中の処理単位を返す
    #=================================================================
    def get_activate_process_unit_by_prcid()
      aDoProcess = get_activate_by_prcid()
      hObjects   = get_objects_by_type(TSR_OBJ_TASK)
      hObjects.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.is_activate?())
          nPrcid = cObjectInfo.get_process_id()
          if (aDoProcess[nPrcid].nil?())
            cTex = get_task_exc_by_task(sObjectID)
            if (!cTex.nil?() && cTex.is_activate?())
              aDoProcess[nPrcid] = cTex
            else
              aDoProcess[nPrcid] = cObjectInfo
            end
          end
        end
      }

      return aDoProcess  # [Array]プロセッサ番号ごとに起動中の処理単位
    end

    #=================================================================
    # 概  要: プロセッサ番号ごとにactivateな非タスクを返す
    #=================================================================
    def get_activate_by_prcid()
      aActivate = []
      if (@cConf.is_asp?())
        aActivate[1] = get_activate()
      elsif (@cConf.is_fmp?())
        aActivate = get_activate_fmp()
      else
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      return aActivate  # [Array]プロセッサ番号ごとにactivateな非タスクを返す
    end

    #=================================================================
    # 概  要: タスクIDから該当タスクを関連付けているタスク例外を返す
    #=================================================================
    def get_task_exc_by_task(sObjectID)
      check_class(String, sObjectID)  # タスクID

      cResult = nil
      hObjects = get_objects_by_type(TSR_OBJ_TASK_EXC)
      hObjects.each_value{|cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_TASK] == sObjectID)
          cResult = cObjectInfo
          break
        end
      }

      return cResult  # [TaskExcept]タスク例外
    end

    #=================================================================
    # 概  要: いずれかのプロセッサでCPUロック状態かを返す
    #=================================================================
    def exist_cpulock?()
      bResult = false

      hObjects = get_objects_by_type(TSR_OBJ_CPU_STATE)
      hObjects.each_value{|cObjectInfo|
        if (cObjectInfo.is_cpu_lock?())
          bResult = true
          break
        end
      }

      return bResult  # [Bool]いずれかのプロセッサでCPUロック状態か
    end

    #=================================================================
    # 概  要: 時間制御条件を満たすかを返す
    #=================================================================
    def is_time_control_situation?(hVariation)
      check_class(Hash, hVariation)  # バリエーション情報

      bResult = false
      # タイムイベントハンドラが登場する(lefttimが指定されている) 
      hObjects = get_objects_by_type(GRP_TIME_EVENT_HDL)
      hObjects.each_value{|cObjectInfo|
        if (cObjectInfo.has_lefttmo?())
          bResult = true
          break
        end
      }

      # ローカルタイマー方式指定時にprcidが指定されている周期ハンドラが登場する
      if (hVariation[TSR_PRM_TIMER_ARCH] == TSR_PRM_TIMER_LOCAL)
        hObjects = get_objects_by_type(TSR_OBJ_CYCLE)
        hObjects.each_value{|cObjectInfo|
          if (cObjectInfo.is_specified?(TSR_PRM_PRCID))
            bResult = true
          end
        }
      end

      # タイムアウト有りのオブジェクト待ちタスクが存在する
      # タイムアウト有りの起床待ちタスクが存在する
      # 時間経過待ちタスクが存在する(lefttmoが指定されている) 
      # => lefttmoとwobjidが設定されている
      hObjects = get_objects_by_type(TSR_OBJ_TASK)
      hObjects.each_value{|cObjectInfo|
        if (cObjectInfo.has_wait_target?() && cObjectInfo.has_lefttmo?())
          bResult = true
        end
      }

      return bResult  # [Bool]時間制御条件を満たすか
    end

    #=================================================================
    # 概  要: グローバル対応のためパラメータの変換を実行する
    #=================================================================
    def convert_global(hTable, hClassTable)
      check_class(Hash, hTable)       # 変換テーブル
      check_class(Hash, hClassTable)  # クラス置換対応表

      @hAllObject.each_value{|cObjectInfo|
        cObjectInfo.convert_global(hTable, hClassTable)
      }
    end

    #=================================================================
    # 概  要: 割込みハンドラで割込み番号ごとにチェックする属性を返す
    #=================================================================
    def get_inthdr_attributes_by_intno()
      return get_object_attributes_by_attributes(TSR_OBJ_INTHDR, [TSR_PRM_ATR, TSR_PRM_INHNO, TSR_PRM_INTPRI, TSR_PRM_CLASS], [TSR_PRM_INTNO])  # [Hash]割込み番号ごとの該当属性
    end

    #=================================================================
    # 概  要: 割込みサービスルーチンで割込み番号ごとにチェックする属性
    #       : を返す
    #=================================================================
    def get_isr_attributes_by_intno()
      return get_object_attributes_by_attributes(TSR_OBJ_ISR, [TSR_PRM_ATR, TSR_PRM_INTPRI], [TSR_PRM_INTNO])  # [Hash]割込み番号ごとの該当属性
    end

    #=================================================================
    # 概  要: 割込みサービスルーチンで割込み番号と割込み優先度の組み合
    #       : わせごとにチェックする属性を返す
    #=================================================================
    def get_isr_attributes_by_intno_and_isrpri()
      return get_object_attributes_by_attributes(TSR_OBJ_ISR, [TSR_PRM_EXINF, TSR_PRM_CLASS], [TSR_PRM_INTNO, TSR_PRM_ISRPRI])  # [Hash]割込み番号と割込み優先度の組み合わせごとの該当属性
    end

    #=================================================================
    # 概  要: CPU例外ハンドラでCPU例外ハンドラ番号ごとにチェックする属
    #       : 性を返す
    #=================================================================
    def get_exception_attributes_by_excno()
      return get_object_attributes_by_attributes(TSR_OBJ_EXCEPTION, [TSR_PRM_CLASS], [TSR_PRM_EXCNO])  # [Hash]CPU例外ハンドラ番号ごとの該当属性
    end

    #=================================================================
    # 概  要: 指定したオブジェクトの属性の値ごとに指定した属性の組み合
    #       : わせをグループ分けして返す
    #=================================================================
    def get_object_attributes_by_attributes(sObjectType, aTargetAtrs, aAtrsByGroup)
      check_class(String, sObjectType)  # オブジェクトタイプ
      check_class(Array, aTargetAtrs)   # 組み合わせとする属性名
      check_class(Array, aAtrsByGroup)  # グループ分けに使う属性名

      hResult = Hash.new{|hash, key|
        hash[key] = []
      }
      hObjects = get_objects_by_type(sObjectType)
      hObjects.each{|sObjectID, cObjectInfo|
        # key
        hKey = {}
        aAtrsByGroup.each{|sAtr|
          sRealAtr       = cObjectInfo.get_real_attribute_name(sAtr)
          hKey[sRealAtr] = cObjectInfo.hState[sAtr]
        }
        # value
        hVal  = {}
        aTargetAtrs.each{|sAtr|
          sRealAtr       = cObjectInfo.get_real_attribute_name(sAtr)
          hVal[sRealAtr] = cObjectInfo.hState[sAtr]
        }
        hResult[hKey].push(hVal)
      }

      # ユニーク化
      hResult.each{|hAtrs, aVals|
        hResult[hAtrs] = aVals.uniq()
      }

      return hResult  # [Hash]属性の組み合わせ(ハッシュ)の配列を値にもつハッシュ
    end
    private :get_object_attributes_by_attributes

    #=================================================================
    # 概　要: グローバル置換対象属性が全てマクロで定義されているか
    #=================================================================
    def is_global_attribute_all_macro?()
      return @hAllObject.all?(){|sObjectID, cObjectInfo|
        cObjectInfo.is_global_attribute_all_macro?()
      }  # [Bool]グローバル置換対象属性が全てマクロで定義されているか
    end

    #=================================================================
    # 概　要: 全オブジェクトにおいて指定されていない属性をセット
    #=================================================================
    def set_nil_attribute()
      @hAllObject.each_value{|cObjectInfo|
        cObjectInfo.set_nil_attribute()
      }
    end

=begin
    #=================================================================
    # 概  要: いずれかのプロセッサで割り込み優先度マスクが設定されてい
    #       : るかを返す
    #=================================================================
    def exist_disable_interrupt?()
      bResult = false

      hObjects = get_objects_by_type(TSR_OBJ_CPU_STATE)
      hObjects.each_value{|cObjectInfo|
        unless (cObjectInfo.is_enable_interrupt?())
          bResult = true
          break
        end
      }

      return bResult  # [Bool]いずれかのプロセッサで割り込み優先度マスクが設定されているか
    end
=end
  end
end
