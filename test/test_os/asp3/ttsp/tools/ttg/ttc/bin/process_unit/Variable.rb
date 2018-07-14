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
require "ttc/bin/class/TTCCommon.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: Variable
  # 概    要: variableの情報を処理するクラス
  #===================================================================
  class Variable
    include TTCModule
    include TTCModule::ObjectCommon

    #=================================================================
    # 概  要: 設定値のチェック
    #=================================================================
    def check_attribute_value()
      check_attribute_type(TSR_PRM_VAR_TYPE, @sType, String, false, @aPath)
      check_attribute_type(TSR_PRM_VAR_VALUE, @snValue, [String, Integer], true, @aPath)
      if (!@hMember.nil?())
        @hMember.each_value{|val|
          check_attribute_type("member", val, [String, Integer], false, @aPath)
        }
      end
    end

    #=================================================================
    # 概  要: オブジェクトチェック
    #=================================================================
    def object_check(bIsPre)
      check_class(Bool, bIsPre)  # pre_conditionか

      aErrors = []
      # 構造体の場合
      if (STR_INITIAL_INFO.has_key?(@sType))
        ### T3_VAR001: 変数が構造体の場合にその構造体のメンバ以外を使用している
        # メンバ名一覧作成
        aMemberNames = []
        STR_INITIAL_INFO[@sType].each{|aMember|
          aMemberNames.push(aMember[0])
        }
        # 宣言されているメンバが使用できるものか
        unless (@hMember.nil?())
          @hMember.each_key{|sMember|
            unless (aMemberNames.include?(sMember))
              sErr = sprintf("T3_VAR001: " + ERR_VAR_UNDEFINED_MEMBER, sMember, @sType)
              aErrors.push(YamlError.new(sErr, @aPath))
            end
          }
        end
        ### T3_VAR003: 変数が構造体の場合にvalue属性を定義している
        unless (@snValue.nil?())
          aErrors.push(YamlError.new("T3_VAR003: " + ERR_VAR_INVALID_DEFINE_VALUE, @aPath))
        end
      ### T3_VAR002: 変数がデータ型の場合にメンバを定義している
      elsif (!@hMember.nil?())
        aErrors.push(YamlError.new("T3_VAR002: " + ERR_VAR_INVALID_DEFINE_MEMBER, @aPath))
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概  要: valueを定義しているかを返す
    #=================================================================
    def is_value_set?()
      return !(@snValue.nil?() && @hMember.nil?())  # [Bool]valueを定義しているか
    end

    #=================================================================
    # 概　要: 変数内のマクロを置換する
    #=================================================================
    def convert_macro()
      hMacro = @cConf.get_macro()
      # 値
      @snValue = calc_all_expr(@snValue, hMacro)
      # メンバ
      unless (@hMember.nil?())
        @hMember.each{|sAtr, val|
          @hMember[sAtr] = calc_all_expr(@hMember[sAtr], hMacro)
        }
      end
    end

    #=================================================================
    # 概  要: グローバル対応のためパラメータの変換を実行する
    #=================================================================
    def convert_global(hTable)
      check_class(Hash, hTable)  # 対応表

      # 値
      if (@snValue.is_a?(String))
        @snValue = convert_global_params(@snValue, hTable)
      end
      # メンバ
      unless (@hMember.nil?())
        @hMember.each{|sAtr, val|
          if (@hMember[sAtr].is_a?(String))
            @hMember[sAtr] = convert_global_params(@hMember[sAtr], hTable)
          end
        }
      end
    end

    #=================================================================
    # 概　要: 補完を実行する
    #=================================================================
    def complement(cPrevVar)
      check_class(Variable, cPrevVar)  # 直前の状態の変数

      # タイプ
      if (@sType.nil?())
        @sType = safe_dup(cPrevVar.sType)
      end
      # 値
      unless (is_specified?(TSR_PRM_VAR_VALUE))
        @snValue = safe_dup(cPrevVar.snValue)
      end
      # 構造体
      if (!cPrevVar.hMember.nil?() && !@hMember.nil?())
        cPrevVar.hMember.each{|sAtr, val|
          unless (is_specified?(sAtr))
            @hMember[sAtr] = safe_dup(val)
          end
        }
      end
    end

    #=================================================================
    # 概　要: エイリアスを実行する
    #=================================================================
    def alias(hAlias)
      check_class(Hash, hAlias)   # エイリアス変換テーブル

      # 変数名
      @sVarName = hAlias[@sVarName]
      # 値
      if (@snValue.is_a?(String))
        @snValue = alias_replace(@snValue, hAlias)
      end
      # メンバ
      unless (@hMember.nil?())
        @hMember.each{|sAtr, val|
          @hMember[sAtr] = alias_replace(val, hAlias)
        }
      end
    end

    #=================================================================
    # 概　要: オブジェクトを複製して返す
    #=================================================================
    def dup()
      cVar = super()

      # 変数名複製
      cVar.sVarName = safe_dup(@sVarName)
      # タイプ複製
      cVar.sType = safe_dup(@sType)
      # 値複製
      cVar.snValue = safe_dup(@snValue)
      # 構造体複製
      unless (@hMember.nil?())
        hNewMember = {}
        @hMember.each{|atr, val|
          hNewMember[atr] = safe_dup(val)
        }
        cVar.hMember = hNewMember
      end

      return cVar  # [Object]複製したオブジェクト
    end

    #=================================================================
    # 概  要: オブジェクトの内容をyamlオブジェクトにして出力
    #=================================================================
    def to_yaml()
      hYaml = {}

      # インスタンス変数
      unless (@sVarName.nil?())
        hYaml["@sVarName"] = @sVarName
      end
      # type
      hYaml[TSR_PRM_VAR_TYPE] = @sType
      # value
      if (!@snValue.nil?() || is_specified?(TSR_PRM_VAR_VALUE))
        hYaml[TSR_PRM_VAR_VALUE] = @snValue
      end
      # 構造体のメンバ
      unless (@hMember.nil?())
        @hMember.each{|atr, val|
          hYaml[atr] = val
        }
      end

      return hYaml  # [Hash]YAMLオブジェクト
    end
  end
end
