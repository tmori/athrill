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
    # 概　要: pre_conditionの構造チェック
    #=================================================================
    def basic_check(hScenarioPre)
      check_class(Hash, hScenarioPre)  # pre_condition

      aErrors = []
      aPath = get_condition_path()
      # オブジェクトの構造チェック
      hScenarioPre.each{|sObjectID, hObjectInfo|
        begin
          common_basic_check(sObjectID, hObjectInfo, aPath)

          # オブジェクトの構造に問題がなければタイプ属性のチェック
          ### T1_018: pre_condition内のオブジェクトにtype属性が定義されていない
          unless (hObjectInfo.has_key?(TSR_PRM_TYPE))
            sErr = sprintf("T1_018: " + ERR_REQUIRED_KEY, TSR_PRM_TYPE)
            aErrors.push(YamlError.new(sErr, aPath + [sObjectID]))
          else
            # 定義できるオブジェクトタイプ
            if (@cConf.is_asp?())
              aObjectType = GRP_DEF_OBJECT_ASP.keys()
            elsif (@cConf.is_fmp?())
              aObjectType = GRP_DEF_OBJECT_FMP.keys()
            else
              abort(ERR_MSG % [__FILE__, __LINE__])
            end
            ### T1_017: 指定されたオブジェクトタイプが定義されていない
            unless (aObjectType.include?(hObjectInfo[TSR_PRM_TYPE]))
              sErr = sprintf("T1_017: " + ERR_OBJECT_UNDEFINED, hObjectInfo[TSR_PRM_TYPE])
              aErrors.push(YamlError.new(sErr, aPath + [sObjectID]))
            end
          end
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

      begin
        super()
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      # CPU_STATEの準備
      aCpuLock = []
      hObjects = get_objects_by_type(TSR_OBJ_CPU_STATE)
      hObjects.each_value{|cObjectInfo|
        nPrcid = cObjectInfo.get_process_id()
        aCpuLock[nPrcid] = cObjectInfo
      }


      # 処理単位のチェック
      hExistFlags = {:bProcessUnit => false, :bTask => false}
      hPOrder     = Hash.new{|hash, key|
        hash[key] = {:cRunning => nil, :aOthers => []}
      }
      hObjects = get_objects_by_type(GRP_PROCESS_UNIT_ALL)
      hObjects.each{|sObjectID, cObjectInfo|
        # タスク
        if (cObjectInfo.sObjectType == TSR_OBJ_TASK)
          # 存在フラグtrue
          hExistFlags[:bTask] = true
          # 優先度逆転チェックのためのデータ収集
          nPrcid = cObjectInfo.get_process_id()
          if (cObjectInfo.is_activate?())
            hPOrder[nPrcid][:cRunning] = cObjectInfo
          else
            hPOrder[nPrcid][:aOthers].push(cObjectInfo) 
          end
        end
        # 実行中の処理単位の存在
        if (cObjectInfo.is_activate?())
          # 存在フラグtrue
          hExistFlags[:bProcessUnit] = true
        end
      }
      ### T4_001: pre_conditionで実行状態の処理単位が存在しない
      if (hExistFlags[:bProcessUnit] == false)
        aErrors.push(YamlError.new("T4_001: " + ERR_NO_RUNNING_PROCESS_UNIT, aPath))
      end
      ### T4_002: pre_conditionでタスクが存在しない
      if (hExistFlags[:bTask] == false)
        aErrors.push(YamlError.new("T4_002: " + ERR_NO_TASK, aPath))
      end

      # 優先度逆転のチェック
      hPOrder.each{|nPrcid, hTasks|
        unless (hTasks[:cRunning].nil?())
          hTasks[:aOthers].each{|cObjectInfo|
            ### T4_003: pre_conditionでCPUロック状態の時に，タスクの優先度に対して実行状態が逆転している状態になっている
            if (!aCpuLock[nPrcid].nil?() && aCpuLock[nPrcid].is_cpu_lock?() &&
                !cObjectInfo.hState[TSR_PRM_TSKPRI].nil?() && cObjectInfo.is_ready?() &&
                hTasks[:cRunning].hState[TSR_PRM_TSKPRI] > cObjectInfo.hState[TSR_PRM_TSKPRI])
              sErr = sprintf("T4_003: " + ERR_INVERTED_STATE_IN_PRE, nPrcid)
              aErrors.push(YamlError.new(sErr, aPath))
              break
            end
            ### T4_004: pre_conditionでporderが逆転している状態になっている
            if (hTasks[:cRunning].hState[TSR_PRM_TSKPRI] == cObjectInfo.hState[TSR_PRM_TSKPRI] &&
                !hTasks[:cRunning].hState[TSR_PRM_PORDER].nil?() && !cObjectInfo.hState[TSR_PRM_PORDER].nil?() &&
                hTasks[:cRunning].hState[TSR_PRM_PORDER] > cObjectInfo.hState[TSR_PRM_PORDER])
              sErr = sprintf("T4_004: " + ERR_INVERTED_PORDER_IN_PRE, nPrcid)
              aErrors.push(YamlError.new(sErr, aPath))
              break
            end
          }
        end
      }


      # タスク例外
      hObjects = get_objects_by_type(TSR_OBJ_TASK_EXC)
      hObjects.each{|sObjectID, cObjectInfo|
        # 関連タスク
        sTask = cObjectInfo.hState[TSR_PRM_TASK]
        if (@hAllObject.has_key?(sTask))
          cTask = @hAllObject[sTask]
          if (cTask.sObjectType == TSR_OBJ_TASK)
            ### T4_034: タスク例外処理ルーチンの起動条件を満たしているのにACTIVATEでない
            if (cTask.is_activate?() && cObjectInfo.hState[TSR_PRM_STATE] == TSR_STT_TTEX_ENA &&
                cObjectInfo.hState[TSR_PRM_HDLSTAT] == TSR_STT_STP && cObjectInfo.hState[TSR_PRM_PNDPTN] != 0)
              aErrors.push(YamlError.new("T4_034: " + ERR_NOT_ACTIVATE_TASK_EXC, aPath + [sObjectID]))
            end
          ### T4_022: 関連タスクIDのオブジェクトがタスクでない
          else
            sErr = sprintf("T4_022: " + ERR_TARGET_NOT_TASK, sTask)
            aErrors.push(YamlError.new(sErr, aPath + [sObjectID, TSR_PRM_TASK]))
          end
        ### T4_021: 関連タスクIDのオブジェクトが存在しない
        else
          sErr = sprintf("T4_021: " + ERR_TARGET_NOT_DEFINED, sTask)
          aErrors.push(YamlError.new(sErr, aPath + [sObjectID, TSR_PRM_TASK]))
        end
      }

      check_error(aErrors)
    end

    #=================================================================
    # 概　要: 初期値を補完する
    #=================================================================
    def complement_init_object_info()
      @hAllObject.each_value{|cObjectInfo|
        cObjectInfo.complement_init_object_info()
      }
    end

    #=================================================================
    # 概　要: 全オブジェクト・変数のエイリアス変換テーブルを返す
    #=================================================================
    def get_alias()
      hResult       = {}
      hSpinCount    = Hash.new(1)  # スピンロックの名前統一のためクラス別登場回数をカウント
      aSortedObject = @hAllObject.sort()
      aSortedObject.each{|aObjectInfo|
        cObjectInfo = aObjectInfo[1]
        if (cObjectInfo.sObjectType == TSR_OBJ_SPINLOCK)
          sClass = cObjectInfo.hState[TSR_PRM_CLASS]
          hAlias = cObjectInfo.get_alias(@sTestID, hSpinCount[sClass])
          hSpinCount[sClass] += 1
        else
          hAlias = cObjectInfo.get_alias(@sTestID)
        end
        hResult = hResult.merge(hAlias)
      }

      return hResult  # [Hash]エイリアス変換テーブル
    end

    #=================================================================
    # 概  要: 時間制御条件を満たすかを返す
    #=================================================================
    def is_time_control_situation?(hVariation)
      check_class(Hash, hVariation)  # バリエーション情報

      bResult = super(hVariation)

      if (bResult == false)
        hObjects = get_objects_by_type(GRP_TIME_EVENT_HDL)
        hObjects.each_value{|cObjectInfo|
          # pre_conditionでACTIVATEなタイムイベントハンドラが存在する
          if (cObjectInfo.is_activate?())
            bResult = true
            break
          # pre_conditionにおいて，cycstatにTCYC_STAが指定されている周期ハンドラが存在する
          elsif (cObjectInfo.sObjectType == TSR_OBJ_CYCLE && cObjectInfo.is_cyc_sta?())
            bResult = true
            break
          end
        }
      end

      return bResult  # [Bool]時間制御条件を満たすか
    end

    #=================================================================
    # 概　要: pre_conditionのパス情報を返す
    #=================================================================
    def get_condition_path()
      return [@sTestID, TSR_LBL_PRE]  # [Array]pre_conditionのパス情報
    end

=begin
    #=================================================================
    # 概　要: 割込み番号一覧を取得する
    #=================================================================
    def get_all_intno()
      aResult  = []
      hObjects = get_objects_by_type([TSR_OBJ_INTHDR, TSR_OBJ_ISR])
      hObjects.each_value{|cObjectInfo|
        aResult.push(cObjectInfo.hState[TSR_PRM_INTNO])
      }

      return aResult.uniq()  # [Array]割込み番号一覧
    end
=end
  end
end
