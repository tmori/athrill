#!ruby -Ke
#
#  TTG
#      TOPPERS Test Generator
#
#  Copyright (C) 2009-2012 by Center for Embedded Computing Systems
#              Graduate School of Information Science, Nagoya Univ., JAPAN
#  Copyright (C) 2010-2011 by Graduate School of Information Science,
#                             Aichi Prefectural Univ., JAPAN
#  Copyright (C) 2012 by FUJISOFT INCORPORATED
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
#  $Id: Variable.rb 6 2012-09-03 11:06:01Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "ttc/bin/class/TTCCommon.rb"
require "common/bin/IMCodeElement.rb"
require "ttc/bin/process_unit/Variable.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: Variable
  # 概    要: variableの情報を処理するクラス
  #===================================================================
  class Variable
    include CommonModule
    include TTCModule
    include TTCModule::ObjectCommon

    attr_accessor :sVarName, :sType, :snValue, :hMember, :aPath

    #=================================================================
    # 概  要: variableの初期化
    #=================================================================
    def initialize(sVarName, hVarInfo, aPath)
      check_class(Object, sVarName)        # 変数名(T2_001でチェックを行う)
      check_class(Object, hVarInfo, true)  # 変数の内容
      check_class(Array, aPath)            # ルートからのパス

      @sVarName = sVarName    # 変数名
      @sType    = nil         # 型
      @snValue  = nil         # 値
      @hMember  = nil         # 構造体のメンバ
      @aPath    = aPath + [@sVarName]

      @cConf = Config.new()

      store_var_info(hVarInfo)
    end

    #=================================================================
    # 概  要: テストシナリオ通りvariableにデータを代入
    #=================================================================
    def store_var_info(hVarInfo)
      check_class(Object, hVarInfo, true)  # 変数の内容

      set_specified_attribute(hVarInfo)
      if (hVarInfo.is_a?(Hash))
        # 構造体のメンバがあれば空ハッシュで初期化
        hVarInfo.each_key{|sKey|
          unless ((sKey == TSR_PRM_VAR_TYPE) || (sKey == TSR_PRM_VAR_VALUE))
            @hMember = {}
          end
        }

        hVarInfo.each{|atr, val|
          case atr
          when TSR_PRM_VAR_TYPE
            @sType = val
          when TSR_PRM_VAR_VALUE
            @snValue = val
          else
            @hMember[atr] = val
          end
        }
      end
    end

    #=================================================================
    # 概　要: グローバル/ローカル変数宣言用コード生成
    #=================================================================
    def gc_global_local_var(cElement, sObjectID)
      check_class(IMCodeElement, cElement) # エレメント
      check_class(String, sObjectID)       # オブジェクトID

      # グローバル変数
      # (メッセージは異なるタスクで送受信する可能性があるためグローバル変数とする)
      if (GRP_GLOBAL_TYPE.include?(@sType))
        # 構造体でない場合
        if (@hMember.nil?())
          cElement.set_global_var(@sVarName, @sType, @snValue)
        # 構造体の場合(全メンバ変数初期化)
        else
          sInitStr = make_initial_str()
          cElement.set_global_var(@sVarName, @sType, sInitStr)
        end

      # ローカル変数
      else
        # 構造体でない場合
        if (@hMember.nil?())
          cElement.set_local_var(sObjectID, @sVarName, @sType, @snValue)
        # 構造体の場合(全メンバ変数初期化)
        else
          sInitStr = make_initial_str()
          cElement.set_local_var(sObjectID, @sVarName, @sType, sInitStr)
        end
      end
    end

    #=================================================================
    # 概　要: 構造体初期化用文字列を返す
    #=================================================================
    def make_initial_str()
      # 定義されていない構造体の型の場合エラー
      if (!STR_INITIAL_INFO.has_key?(@sType))
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      sInitStr = "{"
      STR_INITIAL_INFO[@sType].each{|aStructInfo|
        # ASPの場合，特定のメンバ変数を除外する
        if (@cConf.is_asp?())
          if ((aStructInfo[0] == STR_PRCID) || (aStructInfo[0] == STR_ACTPRC))
            next
          end
        end

        # 初期値が指定されている場合
        if (@hMember.has_key?(aStructInfo[0]))
          sInitStr.concat(" #{@hMember[aStructInfo[0]]},")
        # 初期値が指定されていない場合
        else
          sInitStr.concat(" #{aStructInfo[2]},")
        end
      }
      sInitStr.chop!()
      sInitStr.concat(" }")

      return sInitStr # [String]構造体初期化用文字列
    end

    #=================================================================
    # 概　要: ローカル変数比較用コード生成
    #=================================================================
    def gc_assert_value(cElement, hProcUnitInfo)
      check_class(IMCodeElement, cElement) # エレメント
      check_class(Hash, hProcUnitInfo)     # 処理単位情報

      # 変数の値が設定されている場合
      if (@hMember.nil?() && !@snValue.nil?())
        cElement.set_assert(hProcUnitInfo, @sVarName, @snValue)
      # 構造体が設定されている場合
      elsif (!@hMember.nil?())
        @hMember.each{|name, val|
          cElement.set_assert(hProcUnitInfo, "#{@sVarName}.#{name}", val)
        }
      end
    end

  end
end
