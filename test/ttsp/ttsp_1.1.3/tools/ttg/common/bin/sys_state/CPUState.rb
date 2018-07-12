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
#  $Id: CPUState.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "common/bin/Config.rb"
require "common/bin/IMCodeElement.rb"
require "ttc/bin/sys_state/CPUState.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: CPUState
  # 概    要: CPU状態の情報を処理するクラス
  #===================================================================
  class CPUState
    include CommonModule

    attr_accessor :hState, :sObjectID, :sObjectType

    #=================================================================
    # 概  要: CPU状態の初期化
    #=================================================================
    def initialize(sObjectID, hObjectInfo, aPath, bIsPre)
      check_class(String, sObjectID)  # オブジェクトID
      check_class(Hash, hObjectInfo)  # オブジェクト情報
      check_class(Array, aPath)       # ルートからのパス
      check_class(Bool, bIsPre)       # pre_condition内か

      @cConf  = Config.new()
      @hState = {}

      @sObjectID   = sObjectID
      @sObjectType = TSR_OBJ_CPU_STATE
      @aPath       = aPath + [@sObjectID]

      @hState[TSR_PRM_LOCCPU] = nil  # CPUロック状態
      @hState[TSR_PRM_DISDSP] = nil  # ディスパッチ禁止状態
      @hState[TSR_PRM_CHGIPM] = nil  # 割り込み優先度マスクの値
      @hState[TSR_PRM_PRCID]  = nil  # プロセッサID

      pre_attribute_check(hObjectInfo, aPath + [@sObjectID], bIsPre)
      store_object_info(hObjectInfo)
    end

    #=================================================================
    # 概  要: テストシナリオのCPU状態データを@hStateに代入
    #=================================================================
    def store_object_info(hObjectInfo)
      check_class(Hash, hObjectInfo)  # オブジェクト情報

      # 格納
      set_specified_attribute(hObjectInfo)
      hObjectInfo.each{|atr, val|
        if (atr != TSR_PRM_TYPE)
          @hState[atr] = val
        end
      }
    end

    #=================================================================
    # 概  要: CPU状態を参照するコードをcElementに格納する
    #=================================================================
    def gc_obj_ref(cElement, hProcUnitInfo)
      check_class(IMCodeElement, cElement) # エレメント
      check_class(Hash, hProcUnitInfo)     # 処理単位情報

      # CPUロック状態
      cElement.set_syscall(hProcUnitInfo, "#{API_SNS_LOC}()", @hState[TSR_PRM_LOCCPU], TYP_BOOL_T)

      # ディスパッチ禁止状態
      cElement.set_syscall(hProcUnitInfo, "#{API_SNS_DSP}()", @hState[TSR_PRM_DISDSP], TYP_BOOL_T)

      # 割込み優先度マスク
      cElement.set_local_var(hProcUnitInfo[:id], VAR_INTPRI, TYP_PRI)
      cElement.set_syscall(hProcUnitInfo, "#{FNC_GET_IPM}(&#{VAR_INTPRI})")
      cElement.set_assert(hProcUnitInfo, VAR_INTPRI, @hState[TSR_PRM_CHGIPM])
    end

    #=================================================================
    # 概  要: プロセッサIDを返す
    #=================================================================
    def get_process_id()
      return @hState[TSR_PRM_PRCID] == nil ? 1 : @hState[TSR_PRM_PRCID] # [Integer]プロセッサID
    end
  end
end
