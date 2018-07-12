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
#  $Id: TTCCommon.rb 6 2012-09-03 11:06:01Z nces-shigihara $
#
require "kconv"
require "yaml"
require "common/bin/CommonModule.rb"
require "ttc/bin/class/TTCError.rb"
require "ttc/bin/class/ObjectCommon.rb"
require "ttc/bin/kwalify.rb"
require "common/bin/Config.rb"

#=====================================================================
# TTCModule
#=====================================================================
module TTCModule
  include CommonModule

  #===================================================================
  # 概　要: YAMLファイルをロードする
  #===================================================================
  def load_yaml_file(sFileName)
    check_class(String, sFileName)  # ファイル名

    cConf = Config.new()
    begin
      if (cConf.use_yaml_library?())
        yaml = YAML.load(File.read(sFileName).toutf8)
      else
        yaml = Kwalify::Yaml.load_file(sFileName)
       end
    rescue SystemCallError
      raise(TTCError.new(sprintf(ERR_CANNOT_OPEN_FILE, sFileName) + "#{TTG_NL}(#{$!.message()})"))
    rescue ArgumentError
      raise(TTCError.new(sprintf(ERR_YAML_SYNTAX_INVALID, sFileName) + "#{TTG_NL}(#{$!.message()})"))
    rescue Kwalify::SyntaxError
      raise(TTCError.new(sprintf(ERR_YAML_SYNTAX_INVALID, sFileName) + "#{TTG_NL}(#{$!.message()})"))
    end

    return yaml  # [Object]YAMLオブジェクト
  end
  protected :load_yaml_file

  #===================================================================
  # 概　要: 値を演算する
  #===================================================================
  def parse_value(val, hMacro = {})
    check_class(Object, val, true)  # 値
    check_class(Hash, hMacro)       # マクロ

    # 文字列の場合は様々なケースに対応
    if (val.is_a?(String))
      result = val.dup()
      # ユーザ定義マクロを置換
      unless (hMacro.empty?())
        aMatches = val.scan(/\w+/)
        aMatches.each{|sMacro|
          if (hMacro.has_key?(sMacro))
            result = result.gsub(/(^|\W)#{sMacro}(?=$|\W)/, "\\1#{hMacro[sMacro]}")
          end
        }
      end

      # 数値（16進数も）と一部の記号と空白のみで構成された文字列
      if (result =~ /^(\d+|0x[a-fA-F\d]+|[\s\+\-\*\/\|\&\(\)])+$/)
        begin
          result = eval(result)
        rescue SyntaxError
          sErr = sprintf(ERR_EXPRESSION_INVALID, result)
          raise(TTCError.new(sErr))
        end
      end
    # 文字列以外の場合はそのまま
    else
      result = safe_dup(val)
    end

    return result  # [Object]演算結果
  end

  #===================================================================
  # 概　要: マクロ置換・計算式の演算を行う
  #===================================================================
  def calc_all_expr(data, hMacro)
    check_class(Object, data, true)  # 値
    check_class(Hash, hMacro)        # マクロ

    if (data.is_a?(Hash))
      hTmp = data
      data = {}
      hTmp.each{|atr, val|
        atr       = parse_value(atr, hMacro)
        val       = calc_all_expr(val, hMacro)
        data[atr] = val
      }
    elsif (data.is_a?(Array))
      data = data.dup()
      data.each_with_index{|val, index|
        data[index] = calc_all_expr(val, hMacro)
      }
    elsif (data.is_a?(String))
      data = parse_value(data, hMacro)
    end

    return data  # [Object]マクロ置換・計算式の演算の結果
  end

  #===================================================================
  # 概　要: 文字列を検索し，テストIDを付与できるもの全てをエイリアス
  #       : 文字列以外の場合はそのまま返す
  #===================================================================
  def alias_replace(sStr, hAlias)
    check_class(Object, sStr)  # 対象文字列
    check_class(Hash, hAlias)  # エイリアス変換テーブル

    if (sStr.is_a?(String))
      sResult = sStr.dup()
      aMatches = sResult.scan(/\w+/)
      aMatches.each{|sMatch|
        # 置き換え対象IDに含まれる場合はエイリアス
        if (hAlias.has_key?(sMatch))
          sAlias  = hAlias[sMatch]
          sResult = sResult.gsub(/(^|\W)#{sMatch}(?=$|\W)/, "\\1#{sAlias}")
        end
      }
      return sResult  # [Object]テストIDが付与された文字列
    else
      return safe_dup(sStr)  # 元のオブジェクト
    end
  end

  #===================================================================
  # 概　要: 文字列にテストIDを付与して返す
  #===================================================================
  def alias_str(sStr, sTestID)
    check_class(String, sStr)     # 対象文字列
    check_class(String, sTestID)  # テストID

    return "#{sTestID}_#{sStr}"  # [String]テストIDが付与された文字列
  end

  #===================================================================
  # 概  要: グローバル対応のためパラメータの変換を実行する
  #===================================================================
  def convert_global_params(sStr, hTable)
    check_class(String, sStr)  # 対象文字列
    check_class(Hash, hTable)  # 変換表

    sResult = ""  # 変換後の文字列
    aList = sStr.split(/(\w+)/)
    aList.each{|val|
      case val
      when CFG_MCR_REX_PRCID
        val = convert_global_prcid(val, hTable)
      when CFG_MCR_REX_INTNO
        val = convert_global_intno(val, hTable)
      when CFG_MCR_REX_INHNO
        val = convert_global_inhno(val, hTable)
      when CFG_MCR_REX_EXCNO
        val = convert_global_excno(val, hTable)
      end
      sResult.concat(val)
    }

    return sResult  # [String]変換後の文字列
  end

  #===================================================================
  # 概  要: グローバル対応のためパラメータの変換（prcid）
  #===================================================================
  def convert_global_prcid(sMacro, hTable)
    check_class(String, sMacro)  # マクロ
    check_class(Hash, hTable)    # 変換表

    lPrevKey = TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_PRCID][:local].index(sMacro)
    lConvKey = hTable[lPrevKey]
    unless (lConvKey.nil?())
      sGlobal =  TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_PRCID][:global][lConvKey]
    else
      sGlobal = sMacro.dup()
    end

    return sGlobal  # [String]変換後のパラメータ
  end

  #===================================================================
  # 概  要: グローバル対応のためパラメータの変換（intno）
  #===================================================================
  def convert_global_intno(sMacro, hTable)
    check_class(String, sMacro)  # マクロ
    check_class(Hash, hTable)    # 変換表

    sGlobal = sMacro.dup()
    if (sMacro =~ CFG_MCR_REX_INTNO)
      case $3
      when "INVALID"
        hTempHash = TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_INTNO_INVALID][:local].invert()
        lPrevKey = hTempHash[$1]
        lConvKey = hTable[lPrevKey]
        unless (lConvKey.nil?())
          sGlobal = TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_INTNO_INVALID][:global][lConvKey]
        end
      when "NOT_SET"
        hTempHash = TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_INTNO_NOT_SET][:local].invert()
        lPrevKey = hTempHash[$1]
        lConvKey = hTable[lPrevKey]
        unless (lConvKey.nil?())
          sGlobal = TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_INTNO_NOT_SET][:global][lConvKey]
        end
      else
        hTempHash = TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_INTNO_PREFIX][:local].invert()
        lPrevKey = hTempHash[$1]
        lConvKey = hTable[lPrevKey]
        unless (lConvKey.nil?())
          sGlobal = TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_INTNO_PREFIX][:global][lConvKey] + $6
        end
      end
    end

    return sGlobal  # [String]変換後のパラメータ
  end

  #===================================================================
  # 概  要: グローバル対応のためパラメータの変換（inhno）
  #===================================================================
  def convert_global_inhno(sMacro, hTable)
    check_class(String, sMacro)  # マクロ
    check_class(Hash, hTable)    # 変換表

    sGlobal = sMacro.dup()
    if (sMacro =~ CFG_MCR_REX_INHNO)
      hTempHash = TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_INTNO_PREFIX][:local].invert()
      lPrevKey = hTempHash[$1]
      lConvKey = hTable[lPrevKey]
      unless (lConvKey.nil?())
        sGlobal = TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_INHNO_PREFIX][:global][lConvKey] + $4
      end
    end

    return sGlobal  # [String]変換後のパラメータ
  end

  #===================================================================
  # 概  要: グローバル対応のためパラメータの変換（excno）
  #===================================================================
  def convert_global_excno(sMacro, hTable)
    check_class(String, sMacro)  # マクロ
    check_class(Hash, hTable)    # 変換表

    sGlobal = sMacro.dup()

    hTempHash = TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_EXCNO][:local].invert()
    lPrevKey = hTempHash[sMacro]
    lConvKey = hTable[lPrevKey]
    unless (lConvKey.nil?())
      sGlobal = TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_EXCNO][:global][lConvKey]
    end

    return sGlobal  # [String]変換後のパラメータ
  end

  #===================================================================
  # 概　要: コンディションを分析しタイムティックを持つか判定して返す
  #===================================================================
  def has_timetick?(hCondition)
    check_class(Object, hCondition, true)  # do，post_condition

    bResult = false
    if (hCondition.is_a?(Hash))
      cConf   = Config.new()
      hMacro  = cConf.get_macro()
      bResult = hCondition.all?(){|key, val|
        key = parse_value(key, hMacro)
        key.is_a?(Integer)
      }
    end

    return bResult  # [Bool]タイムティックを持つ場合true，そうでない場合false
  end

  #===================================================================
  # 概　要: エラーがあるかチェックし，ある場合は例外を投げる
  #===================================================================
  def check_error(aErrors)
    check_class(Array, aErrors)  # エラー格納配列

    # エラーがあった場合
    unless (aErrors.empty?())
      raise(TTCMultiError.new(aErrors))
    end
  end

  #===================================================================
  # 概　要: 属性が定義されているものかチェック
  #===================================================================
  def check_defined_attribute(aAtrs, aAtrList, aPath)
    check_class(Array, aAtrs)     # 記述された属性
    check_class(Array, aAtrList)  # 定義できる属性
    check_class(Array, aPath)     # ルートからのパス

    aErrors = []
    ### T1_022: オブジェクトで定義可能な属性以外が指定されている
    aAtrs.each{|sAtr|
      unless (aAtrList.include?(sAtr))
        sErr = sprintf("T1_022: " + ERR_UNDEFINED_KEY, sAtr)
        aErrors.push(YamlError.new(sErr, aPath))
      end
    }

    check_error(aErrors)
  end

  #===================================================================
  # 概　要: 属性のクラスタイプチェック
  #===================================================================
  def check_attribute_type(sAtr, val, acClass, bAllowNil, aPath)
    check_class(String, sAtr)             # 属性名
    check_class(Object, val, true)        # チェックする値
    check_class([Class, Array], acClass)  # 正しいクラス
    check_class(Bool, bAllowNil)          # nilを許可するか
    check_class(Array, aPath)             # ルートからのパス

    bMatch = false
    # 指定されたクラスかチェック
    unless (acClass.is_a?(Array))
      acClass = [acClass]
    end
    # nil許可
    if (bAllowNil == true)
      acClass.push(NilClass)
    end
    acClass.each{|cClass|
      if (val.is_a?(cClass))
        bMatch = true
        break
      end
    }

    ### T2_001: パラメータの型が想定されている型と異なる
    if (bMatch == false)
      sErr = sprintf("T2_001: " + ERR_INVALID_TYPE, sAtr, acClass.join(" or "), val.class())
      raise(YamlError.new(sErr, aPath + [sAtr]))
    end
  end

  #===================================================================
  # 概　要: 属性の値が0以上の整数かチェック
  #===================================================================
  def check_attribute_unsigned(sAtr, val, aPath)
    check_class(String, sAtr)       # 属性名
    check_class(Object, val, true)  # チェックする値
    check_class(Array, aPath)       # ルートからのパス

    check_attribute_type(sAtr, val, Integer, false, aPath)
    # 比較
    if (val < 0)
      sErr = sprintf("T2_002: " + ERR_BE_INTEGER_GE, sAtr, 0, val)
      raise(YamlError.new(sErr, aPath + [sAtr]))
    end
  end

  #===================================================================
  # 概　要: 属性の値が指定した数値より大きい整数かチェック
  #===================================================================
  def check_attribute_gt(sAtr, val, nComp, aPath)
    check_class(String, sAtr)       # 属性名
    check_class(Object, val, true)  # チェックする値
    check_class(Integer, nComp)     # 比較する数値
    check_class(Array, aPath)       # ルートからのパス

    check_attribute_type(sAtr, val, Integer, false, aPath)
    # 比較
    if (val <= nComp)
      sErr = sprintf("T2_002: " + ERR_BE_INTEGER_GT, sAtr, nComp, val)
      raise(YamlError.new(sErr, aPath + [sAtr]))
    end
  end

  #===================================================================
  # 概　要: 属性の値が指定した数値以下の整数かチェック
  #===================================================================
  def check_attribute_le(sAtr, val, nComp, aPath)
    check_class(String, sAtr)       # 属性名
    check_class(Object, val, true)  # チェックする値
    check_class(Integer, nComp)     # 比較する数値
    check_class(Array, aPath)       # ルートからのパス

    check_attribute_type(sAtr, val, Integer, false, aPath)
    # 比較
    if (val > nComp)
      sErr = sprintf("T2_002: " + ERR_BE_INTEGER_LE, sAtr, nComp, val)
      raise(YamlError.new(sErr, aPath + [sAtr]))
    end
  end

  #===================================================================
  # 概　要: 属性の値が指定した数値より小さい整数かチェック
  #===================================================================
  def check_attribute_lt(sAtr, val, nComp, aPath)
    check_class(String, sAtr)       # 属性名
    check_class(Object, val, true)  # チェックする値
    check_class(Integer, nComp)     # 比較する数値
    check_class(Array, aPath)       # ルートからのパス

    check_attribute_type(sAtr, val, Integer, false, aPath)
    # 比較
    if (val >= nComp)
      sErr = sprintf("T2_002: " + ERR_BE_INTEGER_LT, sAtr, nComp, val)
      raise(YamlError.new(sErr, aPath + [sAtr]))
    end
  end

  #===================================================================
  # 概　要: 属性の値が範囲内の整数かチェック
  #===================================================================
  def check_attribute_range(sAtr, val, nMin, nMax, aPath)
    check_class(String, sAtr)       # 属性名
    check_class(Object, val, true)  # チェックする値
    check_class(Integer, nMin)      # 最小値
    check_class(Integer, nMax)      # 最大値
    check_class(Array, aPath)       # ルートからのパス

    check_attribute_type(sAtr, val, Integer, false, aPath)
    # 比較
    unless (val >= nMin && val <= nMax)
      sErr = sprintf("T2_002: " + ERR_BE_INTEGER_RANGE, sAtr, nMin, nMax, val)
      raise(YamlError.new(sErr, aPath + [sAtr]))
    end
  end

  #===================================================================
  # 概　要: 属性の値が予期された値かチェック
  #===================================================================
  def check_attribute_enum(sAtr, val, aEnum, aPath)
    check_class(String, sAtr)       # 属性名
    check_class(Object, val, true)  # チェックする値
    check_class(Array, aEnum)       # 予期された値
    check_class(Array, aPath)       # ルートからのパス

    unless (aEnum.include?(val))
      sErr = sprintf("T2_002: " + ERR_BE_INCLUDE, sAtr, aEnum.join(" | "), val)
      raise(YamlError.new(sErr, aPath + [sAtr]))
    end
  end

  #===================================================================
  # 概　要: 属性の値が予期された値，またはその組み合わせかチェック
  #===================================================================
  def check_attribute_multi(sAtr, val, aEnum, aPath)
    check_class(String, sAtr)       # 属性名
    check_class(Object, val, true)  # チェックする値
    check_class(Array, aEnum)       # 予期された値
    check_class(Array, aPath)       # ルートからのパス

    val = val.gsub(/\s+/, "")
    val = val.gsub(/^\((.*)\)$/, "\\1")
    aMacro = val.split("|")
    aMacro.each{|sMacro|
      begin
        check_attribute_enum(sAtr, sMacro, aEnum, aPath)
      rescue YamlError
        sErr = sprintf("T2_002: " + ERR_BE_INCLUDE, sAtr, aEnum.join(" | "), val)
        raise(YamlError.new(sErr, aPath + [sAtr]))
      end
    }
  end

  #===================================================================
  # 概　要: 属性の値が変数名かチェック
  #===================================================================
  def check_attribute_variable(sAtr, val, aPath)
    check_class(String, sAtr)       # 属性名
    check_class(Object, val, true)  # チェックする値(T2_001でチェックを行う)
    check_class(Array, aPath)       # ルートからのパス

    check_attribute_type(sAtr, val, String, false, aPath)
    ### T2_003: 変数名が命名規則に反している
    unless (val =~ TSR_REX_VARIABLE)
      sErr = sprintf("T2_003: " + ERR_INVALID_VAR_NAME, val)
      raise(YamlError.new(sErr, aPath + [sAtr]))
    end
  end

  #===================================================================
  # 概　要: doの属性チェック
  #===================================================================
  def check_do_attribute(hDo, aSpecified, aPath)
    check_class(Hash, hDo)          # チェックする値
    check_class(Array, aSpecified)  # 指定された属性
    check_class(Array, aPath)       # ルートからのパス

    aErrors = []
    ### T2: doの属性指定チェック
    hDefs = get_do_attribute_list()
    hDefs.each{|sAtr, aClass|
      if (aSpecified.include?(sAtr))
        cClass = hDo[sAtr].class()
        unless (aClass.include?(cClass))
          sErr = sprintf("T2_001: " + ERR_INVALID_TYPE, sAtr, aClass.join(" or "), cClass)
          aErrors.push(YamlError.new(sErr, aPath + [sAtr]))
        end
      end
    }

    check_error(aErrors)
  end

  #===================================================================
  # 概　要: doの属性定義一覧を取得
  #===================================================================
  def get_do_attribute_list()
    # プロファイルに対応する属性定義一覧を取得
    cConf = Config.new()
    if (cConf.is_asp?())
      aResult = GRP_DEF_DO_ASP
    elsif (cConf.is_fmp?())
      aResult = GRP_DEF_DO_FMP
    else
      abort(ERR_MSG % [__FILE__, __LINE__])
    end

    return aResult.dup()  # [Array]doの属性定義一覧
  end
end
