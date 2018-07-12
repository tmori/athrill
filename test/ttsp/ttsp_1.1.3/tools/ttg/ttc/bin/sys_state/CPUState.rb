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
require "ttc/bin/class/TTCCommon.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: CPUState
  # 概    要: CPU状態の情報を処理するクラス
  #===================================================================
  class CPUState
    include TTCModule
    include TTCModule::ObjectCommon

    #=================================================================
    # 概  要: 属性チェック
    #=================================================================
    def attribute_check()
      aErrors = []
      @hState.each{|sAtr, val|
        begin
          if (is_specified?(sAtr))
            case sAtr
            # 0以下の整数か文字列
            when TSR_PRM_CHGIPM
              unless (val.is_a?(String))
                check_attribute_le(sAtr, val, 0, @aPath)
              end

            # 真偽値
            when TSR_PRM_LOCCPU, TSR_PRM_DISDSP
              check_attribute_type(sAtr, val, Bool, false, @aPath)

            # 0より大きい整数
            when TSR_PRM_PRCID
              check_attribute_gt(sAtr, val, 0, @aPath)

            else
              abort(ERR_MSG % [__FILE__, __LINE__])
            end
          end
        rescue TTCError
          aErrors.push($!)
        end
      }

      check_error(aErrors)
    end

    #=================================================================
    # 概  要: オブジェクトチェック
    # 戻り値: なし
    #=================================================================
    def object_check(bIsPre = false)
      check_class(Bool, bIsPre)  # pre_conditionか

=begin
      aErrors = []
      ### T3_CPU001: loc_idが指定されている時にCPUロック状態でない
      if (!@hState[TSR_PRM_LOC_ID].nil?() && @hState[TSR_PRM_LOCCPU] == false)
        aErrors.push(YamlError.new("T3_CPU001: " + ERR_SET_LOC_ID_NOT_CPU_LOCK, @aPath))
      end

      check_error(aErrors)
=end
    end

    #=================================================================
    # 概  要: 初期値を補完する
    #=================================================================
    def complement_init_object_info()
      # loccpu
      unless (is_specified?(TSR_PRM_LOCCPU))
        @hState[TSR_PRM_LOCCPU] = false
      end
      # disdsp
      unless (is_specified?(TSR_PRM_DISDSP))
        @hState[TSR_PRM_DISDSP] = false
      end
      # chgipm
      unless (is_specified?(TSR_PRM_CHGIPM))
        @hState[TSR_PRM_CHGIPM] = KER_TIPM_ENAALL
      end

      # fmp限定
      if (@cConf.is_fmp?())
        # prcid
        unless (is_specified?(TSR_PRM_PRCID))
          @hState[TSR_PRM_PRCID] = CFG_MCR_PRC_SELF
        end
      end
    end

    #=================================================================
    # 概　要: 補完を実行する
    #=================================================================
    def complement(cPrevObj)
      check_class(CPUState, cPrevObj)  # 直前の状態のオブジェクト

      super(cPrevObj)
    end

    #=================================================================
    # 概　要: オブジェクトのエイリアス変換テーブルを返す
    #=================================================================
    def get_alias(sTestID, nNum = nil)
      check_class(String, sTestID)      # テストID
      check_class(Integer, nNum, true)  # 識別番号

      hResult = {}
      hResult[@sObjectID] = alias_str(@sObjectID, sTestID)

      return hResult  # [Hash]エイリアス変換テーブル
    end

    #=================================================================
    # 概　要: オブジェクトを複製して返す
    #=================================================================
    def dup()
      cObjectInfo = super()

      # オブジェクトID複製
      cObjectInfo.sObjectID = safe_dup(@sObjectID)
      # オブジェクトタイプ複製
      cObjectInfo.sObjectType = safe_dup(@sObjectType)
      # パラメータ複製
      cObjectInfo.hState = safe_dup(@hState)

      return cObjectInfo  # [Object]複製したオブジェクト
    end

    #=================================================================
    # 概  要: CPUロック中かを返す
    #=================================================================
    def is_cpu_lock?()
      return (@hState[TSR_PRM_LOCCPU] == true)  # [Bool]CPUロック中か
    end

    #=================================================================
    # 概  要: ディスパッチ禁止状態かを返す
    #=================================================================
    def is_disable_dispatch?()
      return (@hState[TSR_PRM_DISDSP] == true)  # [Bool]ディスパッチ禁止状態か
    end

    #=================================================================
    # 概  要: 割込み優先度マスクが全解除状態かを返す
    #=================================================================
    def is_enable_interrupt?()
      return (@hState[TSR_PRM_CHGIPM] == KER_TIPM_ENAALL || @hState[TSR_PRM_CHGIPM] == 0)  # [Bool]割込み優先度マスクが全解除状態か
    end

    #=================================================================
    # 概  要: 状態変化が指定されているかを返す
    #=================================================================
    def is_state_changed?()
      return (is_cpu_lock?() || is_disable_dispatch?() || !is_enable_interrupt?())  # [Bool]状態変化が指定されているか
    end
  end
end
