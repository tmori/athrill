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
#  $Id: ObjectCommon.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "ttc/bin/class/TTCCommon.rb"

#=====================================================================
# TTCModule
#=====================================================================
module TTCModule
  #===================================================================
  # 概　要: オブジェクトクラス共通メソッド
  #===================================================================
  module ObjectCommon
    include CommonModule
    attr_accessor :aNilAttributes, :aSpecifiedAttributes

    #=================================================================
    # 概　要: オブジェクト内のマクロを置換する
    #=================================================================
    def convert_macro()
      hMacro = @cConf.get_macro()
      @hState.each{|sAtr, val|
        @hState[sAtr] = calc_all_expr(@hState[sAtr], hMacro)
      }
    end

    #=================================================================
    # 概  要: グローバル対応のためパラメータの変換を実行する
    #=================================================================
    def convert_global(hTable, hClassTable)
      check_class(Hash, hTable)       # 変換表
      check_class(Hash, hClassTable)  # クラス置換対応表

      # prcid，class
      if (@hState[TSR_PRM_PRCID].is_a?(String))
        sMacro = @hState[TSR_PRM_PRCID]
        # prcid
        @hState[TSR_PRM_PRCID] = convert_global_prcid(sMacro, hTable)
        # class
        if (!@hState[TSR_PRM_CLASS].nil?() && sMacro != @hState[TSR_PRM_PRCID])
          sClass = @hState[TSR_PRM_CLASS]
          @hState[TSR_PRM_CLASS] = hClassTable[sMacro][sClass][@hState[TSR_PRM_PRCID]]
        end
      end

      # actprc
      if (@hState[TSR_PRM_ACTPRC].is_a?(String))
        @hState[TSR_PRM_ACTPRC] = convert_global_prcid(@hState[TSR_PRM_ACTPRC], hTable)
      end

      # intno
      if (@hState[TSR_PRM_INTNO].is_a?(String))
        @hState[TSR_PRM_INTNO] = convert_global_intno(@hState[TSR_PRM_INTNO], hTable)
      end

      # inhno
      if (@hState[TSR_PRM_INHNO].is_a?(String))
        @hState[TSR_PRM_INHNO] = convert_global_inhno(@hState[TSR_PRM_INHNO], hTable)
      end

      # excno
      if (@hState[TSR_PRM_EXCNO].is_a?(String))
        @hState[TSR_PRM_EXCNO] = convert_global_excno(@hState[TSR_PRM_EXCNO], hTable)
      end
    end

    #=================================================================
    # 概　要: グローバル置換対象属性が全てマクロで定義されているか
    #=================================================================
    def is_global_attribute_all_macro?()
      # チェックする属性と，マクロの正規表現
      hCheckAtr = {
        TSR_PRM_PRCID  => CFG_MCR_REX_PRCID,
        TSR_PRM_CLASS  => CFG_MCR_REX_CLASS,
        TSR_PRM_INTNO  => CFG_MCR_REX_INTNO,
        TSR_PRM_INHNO  => CFG_MCR_REX_INHNO,
        TSR_PRM_EXCNO  => CFG_MCR_REX_EXCNO
      }

      return hCheckAtr.all?(){|sAtr, cReg|
        (@hState[sAtr].nil?() || @hState[sAtr] =~ cReg)
      }  # [Bool]グローバル置換対象属性が全てマクロで定義されているか
    end

    #=================================================================
    # 概  要: YAML階層のパスを設定する
    #=================================================================
    def set_path(aPath)
      check_class(Array, aPath)  # ルートからのパス

      @aPath = aPath
    end

    #=================================================================
    # 概  要: 初期値を補完する
    #=================================================================
    def complement_init_object_info()
      # 通常は何もしない
      # 必要なオブジェクトはオーバーライド
    end

    #=================================================================
    # 概　要: 補完を実行する
    #=================================================================
    def complement(cPrevObj)
      # 型チェックは各オブジェクトのスーパークラスで実施すること
      check_class(Object, cPrevObj)  # 直前の状態のオブジェクト

      # 指定されていない場合補完
      cPrevObj.hState.each{|sAtr, val|
        sRealAtr = get_real_attribute_name(sAtr)
        unless (is_specified?(sRealAtr))
          @hState[sAtr] = safe_dup(val)
        end
      }
    end

    #=================================================================
    # 概　要: 全補完終了後に実行する処理
    #=================================================================
    def complement_after()
      # 通常は何もしない
      # 必要なオブジェクトはオーバーライド
    end

    #=================================================================
    # 概　要: エイリアスを実行する
    #=================================================================
    def alias(hAlias)
      check_class(Hash, hAlias)   # エイリアス変換テーブル

      # オブジェクトID
      @sObjectID = hAlias[@sObjectID]

      # 属性
      @hState.each{|sAtr, val|
        if (val.is_a?(String))
          @hState[sAtr] = alias_replace(val, hAlias)
        end
      }
    end

    #=================================================================
    # 概　要: オブジェクトのエイリアス変換テーブルを返す
    #=================================================================
    def get_alias(sTestID, nNum = nil)
      abort(ERR_MSG % [__FILE__, __LINE__])
    end

    #=================================================================
    # 概  要: オブジェクト内に存在する変数名一覧を返す
    #=================================================================
    def get_variable_names()
      return []  # [Array]オブジェクト内に存在する変数名一覧
    end

    #=================================================================
    # 概  要: 内部保持用属性名からTESRYの属性名を取得する
    #=================================================================
    def get_real_attribute_name(sAtr)
      check_class(String, sAtr)  # 内部保持用属性名

      return sAtr  # [String]TESRYの属性名
    end

    #=================================================================
    # 概  要: Yamlに記述されているパラメータリストを記録する
    #=================================================================
    def set_specified_attribute(hObjectInfo)
      check_class(Hash, hObjectInfo, true)  # オブジェクト情報

      if (hObjectInfo.nil?())
        @aSpecifiedAttributes = []
      else
        @aSpecifiedAttributes = hObjectInfo.keys()
        @aSpecifiedAttributes.delete(TSR_PRM_TYPE)
      end
    end

    #=================================================================
    # 概  要: Nilのパラメータリストを記録する
    #=================================================================
    def set_nil_attribute()
      @aNilAttributes = []
      @hState.each{|sAtr, val|
        if (val.nil?())
          @aNilAttributes.push(sAtr)
        end
      }
    end

    #=================================================================
    # 概　要: 属性チェック(何の属性を指定したかをチェック)
    #=================================================================
    def pre_attribute_check(hObjectInfo, aPath, bIsPre)
      check_class(Hash, hObjectInfo)  # オブジェクト情報
      check_class(Array, aPath)       # ルートからのパス
      check_class(Bool, bIsPre)       # pre_condition内か

      aErrors = []
      # オブジェクトタイプに対応する属性定義を取得
      hAtrDefList = get_attribute_list()

      ### 定義可能な属性か
      aAtrs = hObjectInfo.keys()
      begin
        aAtrs.delete(TSR_PRM_TYPE)
        check_defined_attribute(aAtrs, hAtrDefList.keys(), aPath)
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      ### T1_023: pre_conditionで指定必須な属性が指定されていない
      if (bIsPre == true)
        hAtrDefList.each{|sAtr, aDef|
          if (aDef[0] == true)
            unless (hObjectInfo.has_key?(sAtr))
              sErr = sprintf("T1_023: " + ERR_REQUIRED_KEY, sAtr)
              aErrors.push(YamlError.new(sErr, aPath))
            end
          end
        }
      ### T1_024: pre_conditionのみで指定可能な属性が指定されている
      else
        hAtrDefList.each{|sAtr, aDef|
          if (aDef[1] == true)
            if (hObjectInfo.has_key?(sAtr))
              sErr = sprintf("T1_024: " + ERR_ONLY_PRE_ATR, sAtr)
              aErrors.push(YamlError.new(sErr, aPath))
            end
          end
        }
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概  要: オブジェクトの属性定義一覧を取得
    #=================================================================
    def get_attribute_list()
      # プロファイルに対応する属性定義一覧を取得
      cConf    = Config.new()
      hAtrList = {}
      if (cConf.is_asp?())
        hAtrList = GRP_DEF_OBJECT_ASP
      elsif (cConf.is_fmp?())
        hAtrList = GRP_DEF_OBJECT_FMP
      end

      # オブジェクトタイプに対応する属性定義を取得
      if (hAtrList.has_key?(@sObjectType))
        return hAtrList[@sObjectType]  # [Hash]オブジェクトの属性定義一覧
      else
        abort(ERR_MSG % [__FILE__, __LINE__])
      end
    end

    #=================================================================
    # 概  要: オブジェクトの内容をYAMLオブジェクトに変換して返す
    #=================================================================
    def to_yaml(bIsPre = false)
      check_class(Bool, bIsPre)  # pre_conditionか

      hYaml = {}
      # preの場合type属性を付与
      if (bIsPre == true)
        hYaml[TSR_PRM_TYPE] = @sObjectType
      end
      # 保持している情報を取りだす
      @hState.each{|sAtr, val|
        sRealAtr = get_real_attribute_name(sAtr)
        if (!val.nil?() || is_specified?(sRealAtr))
          hYaml[sRealAtr] = safe_dup(val)
        end
      }

      return hYaml  # [Hash]YAMLオブジェクト
    end

    #=================================================================
    # 概  要: 指定された属性がYAML上で記述されているかを返す
    #=================================================================
    def is_specified?(sAtr)
      check_class(String, sAtr, true)  # 属性名

      return @aSpecifiedAttributes.include?(sAtr)  # [Bool]指定された属性がYAML上で記述されているか
    end

    #=================================================================
    # 概　要: 属性チェック
    #=================================================================
    def attribute_check()
      abort(ERR_MSG % [__FILE__, __LINE__])
    end

    #=================================================================
    # 概  要: オブジェクトチェック
    #=================================================================
    def object_check(aPath, bIsPre = false)
      abort(ERR_MSG % [__FILE__, __LINE__])
    end
  end
end
