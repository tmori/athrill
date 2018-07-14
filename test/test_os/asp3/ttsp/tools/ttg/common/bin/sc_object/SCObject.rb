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
#  $Id: SCObject.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "common/bin/Config.rb"
require "common/bin/IMCodeElement.rb"
require "ttc/bin/sc_object/SCObject.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: SCObject
  # 概    要: 同期・通信オブジェクトの情報を処理するクラス
  #===================================================================
  class SCObject
    include CommonModule

    attr_accessor :hState, :sObjectType, :sObjectID

    #=================================================================
    # 概  要: 同期・通信オブジェクトの初期化
    #=================================================================
    def initialize(sObjectID, hObjectInfo, sObjectType, aPath, bIsPre)
      check_class(String, sObjectID)    # オブジェクトID
      check_class(Hash, hObjectInfo)    # オブジェクト情報
      check_class(String, sObjectType)  # オブジェクトタイプ
      check_class(Array, aPath)         # ルートからのパス
      check_class(Bool, bIsPre)         # pre_condition内か

      @cConf   = Config.new()
      @hState = {}

      @sObjectID   = sObjectID
      @sObjectType = sObjectType  # オブジェクトのタイプ
      @aPath       = aPath + [@sObjectID]

      # 同期・通信オブジェクト共通部分
      @hState[TSR_PRM_DATACNT]  = nil  # 格納できるデータ数[データキュー/優先度データキュー]
      @hState[TSR_PRM_STSKLIST] = nil  # 送信待ちタスクのリスト[データキュー/優先度データキュー]
      @hState[TSR_PRM_RTSKLIST] = nil  # 受信待ちタスクのリスト[データキュー/優先度データキュー]
      @hState[TSR_PRM_DATALIST] = nil  # 管理領域のデータのリスト[データキュー/優先度データキュー]
      @hState[TSR_PRM_WTSKLIST] = nil  # 待ちタスクのリスト[セマフォ/イベントフラグ/メールボックス/固定長メモリプール]
      @hState[TSR_PRM_ATR]      = nil  # 属性[セマフォ/イベントフラグ/データキュー/優先度データキュー/メールボックス/固定長メモリプール]
      @hState[TSR_PRM_CLASS]    = nil  # クラス

      # SEMAPHORE
      @hState[TSR_PRM_MAXSEM]   = nil  # 最大資源数
      @hState[TSR_PRM_ISEMCNT]  = nil  # 初期資源数
      @hState[TSR_PRM_SEMCNT]   = nil  # 現在資源数

      # EVENTFLAG
      @hState[TSR_PRM_IFLGPTN]  = nil  # 初期ビットパターン
      @hState[TSR_PRM_FLGPTN]   = nil  # 現在ビットパターン

      # P_DATAQUEUE
      @hState[TSR_PRM_MAXDPRI]  = nil  # データ優先度の最大値

      # MAILBOX
      @hState[TSR_PRM_MAXMPRI]  = nil  # メッセージ優先度の最大値
      @hState[TSR_PRM_MSGLIST]  = nil  # 受信待ちメッセージのリスト

      # MEMORYPOOL
      @hState[TSR_PRM_BLKCNT]   = nil  # 獲得できるブロックの初期数
      @hState[TSR_PRM_FBLKCNT]  = nil  # 獲得できるブロックの現在数
      @hState[TSR_PRM_BLKSZ]    = nil  # ブロックのサイズ
      @hState[TSR_PRM_MPF]      = nil  # メモリプール先頭番地参照用変数

      # SPINLOCK
      @hState[TSR_PRM_SPNSTAT]  = nil  # スピンロックの状態
      @hState[TSR_PRM_PROCID]   = nil  # スピンロックを取得している処理単位ID

      pre_attribute_check(hObjectInfo, aPath + [@sObjectID], bIsPre)
      store_object_info(hObjectInfo)
    end

    #=================================================================
    # 概  要: テストシナリオの通り同期・通信オブジェクトデータを
    #         @hStateに代入
    #=================================================================
    def store_object_info(hObjectInfo)
      check_class(Hash, hObjectInfo)  # オブジェクト情報

      # 格納
      set_specified_attribute(hObjectInfo)
      hObjectInfo.each{|atr, val|
        case atr
        when TSR_PRM_TYPE
          # 何もしない

        when TSR_PRM_DTQCNT, TSR_PRM_PDQCNT
          @hState[TSR_PRM_DATACNT] = val

        when TSR_PRM_SEMATR, TSR_PRM_FLGATR, TSR_PRM_DTQATR, TSR_PRM_PDQATR, TSR_PRM_MBXATR, TSR_PRM_MPFATR
          @hState[TSR_PRM_ATR] = val

        else
          @hState[atr] = val
        end
      }

      # リスト系パラメータでTESRYで属性だけ記述されている場合，
      # 空配列をセットする
      [TSR_PRM_WTSKLIST, TSR_PRM_STSKLIST, TSR_PRM_RTSKLIST, TSR_PRM_DATALIST, TSR_PRM_MSGLIST].each{|sAtr|
        if (hObjectInfo[sAtr].nil?() && hObjectInfo.has_key?(sAtr))
          @hState[sAtr] = []
        end
      }
    end

    #=================================================================
    # 概  要: 同期・通信オブジェクトのpre_condition処理をcElementに格
    #         納する
    #=================================================================
    def gc_obj_ref(cElement, hProcUnitInfo, bContext, bCouLock)
      check_class(IMCodeElement, cElement) # エレメント
      check_class(Hash, hProcUnitInfo)     # 処理単位情報
      check_class(Bool, bContext)          # タスクコンテキストか
      check_class(Bool, bCouLock)          # CPUロックか


      # CPU_LOCK
      if (check_cpu_lock() && (bCouLock != true))
        cElement.set_syscall(hProcUnitInfo, get_cpu_lock(bContext))
      end

      cElement.set_local_var(hProcUnitInfo[:id], @sRefStrVar, @sRefStrType)
      cElement.set_syscall(hProcUnitInfo, "#{@sRefAPI}(#{@sObjectID}, &#{@sRefStrVar})")

      # 属性
      if (!@hState[TSR_PRM_ATR].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{@sRefAtr}", @hState[TSR_PRM_ATR])
      end

      # 共通部分
      # 待ちタスクのリスト[SEMAPHORE, EVENTFLAG, MAILBOX, MEMORYPOOL]
      if (!@hState[TSR_PRM_WTSKLIST].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{@sRefWaitCnt}", @hState[TSR_PRM_WTSKLIST].size())

        if (!@hState[TSR_PRM_WTSKLIST].empty?())
          cElement.set_local_var(hProcUnitInfo[:id], VAR_TSKID, TYP_ID)

          @hState[TSR_PRM_WTSKLIST].each_with_index{|hTaskInfo, nIndex|
            case @sObjectType
            when TSR_OBJ_SEMAPHORE, TSR_OBJ_MAILBOX, TSR_OBJ_MEMORYPOOL
              cElement.set_syscall(hProcUnitInfo, "#{@sRefRWaitAPI}(#{@sObjectID}, #{nIndex + 1}, &#{VAR_TSKID})")
              cElement.set_assert(hProcUnitInfo, VAR_TSKID, hTaskInfo.keys[0])

            when TSR_OBJ_EVENTFLAG
              cElement.set_local_var(hProcUnitInfo[:id], VAR_WAIPTN, TYP_FLGPTN)
              cElement.set_local_var(hProcUnitInfo[:id], VAR_WFMODE, TYP_MODE)
              cElement.set_syscall(hProcUnitInfo, "#{@sRefRWaitAPI}(#{@sObjectID}, #{nIndex + 1}, &#{VAR_TSKID}, &#{VAR_WAIPTN}, &#{VAR_WFMODE})")
              cElement.set_assert(hProcUnitInfo, "#{VAR_TSKID}", hTaskInfo.keys[0])
              cElement.set_assert(hProcUnitInfo, VAR_WAIPTN, hTaskInfo[hTaskInfo.keys[0]][TSR_VAR_WAIPTN])
              cElement.set_assert(hProcUnitInfo, VAR_WFMODE, hTaskInfo[hTaskInfo.keys[0]][TSR_VAR_WFMODE])

            else
              abort(ERR_MSG % [__FILE__, __LINE__])
            end
          }
        end
      end

      # DATAQUEUE，P_DATAQUEUE
      # データ管理領域リスト
      if (!@hState[TSR_PRM_DATACNT].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{@sRefDataCnt}", @hState[TSR_PRM_DATACNT])

        if (!@hState[TSR_PRM_DATALIST].nil?())
          cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{@sRefDataList}", @hState[TSR_PRM_DATALIST].size())

          if (!@hState[TSR_PRM_DATALIST].empty?())
            case @sObjectType
            when TSR_OBJ_DATAQUEUE
              cElement.set_local_var(hProcUnitInfo[:id], VAR_DATA, TYP_INTPTR_T)
              @hState[TSR_PRM_DATALIST].each_with_index{|hData, nIndex|
                cElement.set_syscall(hProcUnitInfo, "#{FNC_REF_DATA}(#{sObjectID}, #{nIndex + 1}, &#{VAR_DATA})")
                cElement.set_assert(hProcUnitInfo, VAR_DATA, hData[TSR_VAR_DATA])
              }

            when TSR_OBJ_P_DATAQUEUE
              cElement.set_local_var(hProcUnitInfo[:id], VAR_DATA, TYP_INTPTR_T)
              cElement.set_local_var(hProcUnitInfo[:id], VAR_DATAPRI, TYP_PRI)

              @hState[TSR_PRM_DATALIST].each_with_index{|hData, nIndex|
                cElement.set_syscall(hProcUnitInfo, "#{FNC_REF_PRI_DATA}(#{sObjectID}, #{nIndex + 1}, &#{VAR_DATA}, &#{VAR_DATAPRI})")
                cElement.set_assert(hProcUnitInfo, VAR_DATA, hData[TSR_VAR_DATA])
                cElement.set_assert(hProcUnitInfo, VAR_DATAPRI, hData[TSR_VAR_DATAPRI])
              }

            else
              abort(ERR_MSG % [__FILE__, __LINE__])
            end
          end
        end
      end

      # 送信待ちタスクリスト
      if (!@hState[TSR_PRM_STSKLIST].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{@sRefSWaitList}", @hState[TSR_PRM_STSKLIST].size())

        if (!@hState[TSR_PRM_STSKLIST].empty?())
          cElement.set_local_var(hProcUnitInfo[:id], VAR_TSKID, TYP_ID)
          cElement.set_local_var(hProcUnitInfo[:id], VAR_DATA, TYP_INTPTR_T)

          case @sObjectType
          when TSR_OBJ_DATAQUEUE
            @hState[TSR_PRM_STSKLIST].each_with_index{|hTaskInfo, nIndex|
              cElement.set_syscall(hProcUnitInfo, "#{FNC_REF_SWAIT_DTQ}(#{@sObjectID}, #{nIndex + 1}, &#{VAR_TSKID}, &#{VAR_DATA})")
              cElement.set_assert(hProcUnitInfo, VAR_TSKID, hTaskInfo.keys[0])
              cElement.set_assert(hProcUnitInfo, VAR_DATA, hTaskInfo[hTaskInfo.keys[0]][TSR_VAR_DATA])
            }

          when TSR_OBJ_P_DATAQUEUE
            cElement.set_local_var(hProcUnitInfo[:id], VAR_DATAPRI, TYP_PRI)
            @hState[TSR_PRM_STSKLIST].each_with_index{|hTaskInfo, nIndex|
              cElement.set_syscall(hProcUnitInfo, "#{FNC_REF_SWAIT_PDQ}(#{@sObjectID}, #{nIndex + 1}, &#{VAR_TSKID}, &#{VAR_DATA}, &#{VAR_DATAPRI})")
              cElement.set_assert(hProcUnitInfo, VAR_TSKID, hTaskInfo.keys[0])
              cElement.set_assert(hProcUnitInfo, VAR_DATA, hTaskInfo[hTaskInfo.keys[0]][TSR_VAR_DATA])
              cElement.set_assert(hProcUnitInfo, VAR_DATAPRI, hTaskInfo[hTaskInfo.keys[0]][TSR_VAR_DATAPRI])
            }

          else
            abort(ERR_MSG % [__FILE__, __LINE__])
          end
        end
      end

      # 受信待ちタスクリスト
      if (!@hState[TSR_PRM_RTSKLIST].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{@sRefRWaitList}", @hState[TSR_PRM_RTSKLIST].size())

        if (!@hState[TSR_PRM_RTSKLIST].empty?())
          cElement.set_local_var(hProcUnitInfo[:id], VAR_TSKID, TYP_ID)

          @hState[TSR_PRM_RTSKLIST].each_with_index{|hTaskInfo, nIndex|
            cElement.set_syscall(hProcUnitInfo, "#{@sRefRWaitAPI}(#{@sObjectID}, #{nIndex + 1}, &#{VAR_TSKID})")
            cElement.set_assert(hProcUnitInfo, VAR_TSKID, hTaskInfo.keys[0])
          }
        end
      end

      # MAILBOX
      # メッセージリスト
      if (!@hState[TSR_PRM_MSGLIST].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{@sRefMsgCnt}", @hState[TSR_PRM_MSGLIST].size())

        if (!@hState[TSR_PRM_MSGLIST].empty?())
          @hState[TSR_PRM_MSGLIST].each_with_index{|hData, nIndex|
            if (hData.has_key?(TSR_VAR_MSGPRI))
              cElement.set_local_var(hProcUnitInfo[:id], VAR_P_MSG_PRI, TYP_T_P_MSG_PRI)
              cElement.set_syscall(hProcUnitInfo, "#{FNC_REF_MSG}(#{@sObjectID}, #{nIndex + 1}, #{CST_MSG2}&#{VAR_P_MSG_PRI})")
              cElement.set_assert(hProcUnitInfo, VAR_P_MSG_PRI, "&#{hData[TSR_VAR_MSG]}")
              cElement.set_assert(hProcUnitInfo, "#{VAR_P_MSG_PRI}->#{STR_MSGPRI}", hData[TSR_VAR_MSGPRI])
            else
              cElement.set_local_var(hProcUnitInfo[:id], VAR_P_MSG, TYP_T_P_MSG)
              cElement.set_syscall(hProcUnitInfo, "#{FNC_REF_MSG}(#{@sObjectID}, #{nIndex + 1}, #{CST_MSG2}&#{VAR_P_MSG})")
              cElement.set_assert(hProcUnitInfo, VAR_P_MSG, "&#{hData[TSR_VAR_MSG]}")
            end
          }
        end
      end

      # SEMAPHORE
      if (!@hState[TSR_PRM_SEMCNT].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_SEMCNT}", @hState[TSR_PRM_SEMCNT])
      end

      if (!@hState[TSR_PRM_ISEMCNT].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_ISEMCNT}", @hState[TSR_PRM_ISEMCNT])
      end

      if (!@hState[TSR_PRM_MAXSEM].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_MAXSEM}", @hState[TSR_PRM_MAXSEM])
      end

      # EVENTFLAG
      if (!@hState[TSR_PRM_FLGPTN].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_FLGPTN}", @hState[TSR_PRM_FLGPTN])
      end

      if (!@hState[TSR_PRM_IFLGPTN].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_IFLGPTN}", @hState[TSR_PRM_IFLGPTN])
      end

      # MAILBOX
      if (!@hState[TSR_PRM_MAXMPRI].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_MAXMPRI}", @hState[TSR_PRM_MAXMPRI])
      end

      # P_DATAQUEUE
      if (!@hState[TSR_PRM_MAXDPRI].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_MAXDPRI}", @hState[TSR_PRM_MAXDPRI])
      end

      # MEMORYPOOL
      if (!@hState[TSR_PRM_BLKCNT].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_BLKCNT}", @hState[TSR_PRM_BLKCNT])
      end

      if (!@hState[TSR_PRM_FBLKCNT].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_FBLKCNT}", @hState[TSR_PRM_FBLKCNT])
      end

      if (!@hState[TSR_PRM_BLKSZ].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_BLKSZ}", @hState[TSR_PRM_BLKSZ])
      end

      if (!@hState[TSR_PRM_MPF].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_MPF}", @hState[TSR_PRM_MPF])
      end

      # SPINLOCK
      if (!@hState[TSR_PRM_SPNSTAT].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_SPNSTAT}", @hState[TSR_PRM_SPNSTAT])
      end

      # CPU_UNLOCK
      if (check_cpu_lock() && (bCouLock != true))
        cElement.set_syscall(hProcUnitInfo, get_cpu_unlock(bContext))
      end
    end

    #=================================================================
    # 概  要: cpu_lockが必要かどうかを判断して必要であればtrueを，
    #         必要でなければfalseを返す
    #=================================================================
    def check_cpu_lock()
      if !(@hState[TSR_PRM_STSKLIST].nil?() || @hState[TSR_PRM_STSKLIST].empty?()) ||
         !(@hState[TSR_PRM_RTSKLIST].nil?() || @hState[TSR_PRM_RTSKLIST].empty?()) ||
         !(@hState[TSR_PRM_DATALIST].nil?() || @hState[TSR_PRM_DATALIST].empty?()) ||
         !(@hState[TSR_PRM_WTSKLIST].nil?() || @hState[TSR_PRM_WTSKLIST].empty?()) ||
         !(@hState[TSR_PRM_MSGLIST].nil?() || @hState[TSR_PRM_MSGLIST].empty?())
        return true # [Bool]CPUロックが必要
      end

      return false # [Bool]CPUロックが不要
    end

    #=================================================================
    # 概  要: タスクコンテキストの場合はcpu_lockを，
    #         非タスクコンテキストの場合はicpu_lockを返す
    #=================================================================
    def get_cpu_lock(bCpuLock)
      check_class(Bool, bCpuLock)

      if (bCpuLock == true)
        sCpuLock = "#{API_LOC_CPU}()"
      else
        sCpuLock = "#{API_ILOC_CPU}()"
      end

      return sCpuLock # [String]cpu_lockかicpu_lock
    end

    #=================================================================
    # 概  要: タスクコンテキストの場合はunl_lockを，
    #         非タスクコンテキストの場合はiunl_lockを返す
    #=================================================================
    def get_cpu_unlock(bCpuLock)
      check_class(Bool, bCpuLock)

      if (bCpuLock == true)
        sCpuUnlock = "#{API_UNL_CPU}()"
      else
        sCpuUnlock = "#{API_IUNL_CPU}()"
      end

      return sCpuUnlock # [String]unl_lockかiunl_cpu
    end
  end
end
