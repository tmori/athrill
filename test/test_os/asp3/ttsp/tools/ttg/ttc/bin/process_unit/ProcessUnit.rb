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
#  $Id: ProcessUnit.rb 6 2012-09-03 11:06:01Z nces-shigihara $
#
require "ttc/bin/class/TTCCommon.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: ProcessUnit
  # 概    要: 処理単位オブジェクトの情報を処理するクラス
  #===================================================================
  class ProcessUnit
    include TTCModule
    include TTCModule::ObjectCommon

    #=================================================================
    # 概　要: 属性チェック(何の属性を指定したかをチェック)
    #=================================================================
    def pre_attribute_check(hObjectInfo, aPath, bIsPre)
      check_class(Hash, hObjectInfo)  # オブジェクト情報
      check_class(Array, aPath)       # ルートからのパス
      check_class(Bool, bIsPre)       # pre_condition内か

      aErrors = []
      begin
        super(hObjectInfo, aPath, bIsPre)
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      # do
      if (hObjectInfo[TSR_PRM_DO].is_a?(Hash))
        aAtrDefList = get_do_attribute_list().keys()
        aAtrDefList.delete(TSR_PRM_ID)  # id属性は定義できないので削除
        begin
          check_defined_attribute(hObjectInfo[TSR_PRM_DO].keys(), aAtrDefList, aPath + [TSR_PRM_DO])
        rescue TTCMultiError
          aErrors.concat($!.aErrors)
        end
      end

      # var
      if (hObjectInfo[TSR_PRM_VAR].is_a?(Hash))
        hObjectInfo[TSR_PRM_VAR].each{|sVarName, hVar|
          if (hVar.is_a?(Hash))
            ### T1_025: pre_conditionで変数のtypeが指定されていない
            if (bIsPre == true && hVar[TSR_PRM_VAR_TYPE].nil?())
              sErr = sprintf("T1_025: " + ERR_REQUIRED_KEY, TSR_PRM_VAR_TYPE)
              aErrors.push(YamlError.new(sErr, aPath + [TSR_PRM_VAR, sVarName]))
            ### T1_026: post_conditionで変数のtypeが指定されている
            elsif (bIsPre == false && hVar.has_key?(TSR_PRM_VAR_TYPE))
              sErr = sprintf("T1_026: " + ERR_ONLY_PRE_ATR, TSR_PRM_VAR_TYPE)
              aErrors.push(YamlError.new(sErr, aPath + [TSR_PRM_VAR, sVarName]))
            end
          else
            # Variableクラスのインスタンス変数に代入する前にT2_001チェックを実施
            sErr = sprintf("T2_001: " + ERR_INVALID_TYPE, sVarName, "Hash", hVar.class())
            aErrors.push(YamlError.new(sErr, aPath + [TSR_PRM_VAR, sVarName]))
          end
        }
      elsif (hObjectInfo[TSR_PRM_VAR].nil?())
        # 何もしない
      else
        # Variableクラスのインスタンス変数に代入する前にT2_001チェックを実施
        sErr = sprintf("T2_001: " + ERR_INVALID_TYPE, TSR_PRM_VAR, "Hash", hObjectInfo[TSR_PRM_VAR].class())
        aErrors.push(YamlError.new(sErr, aPath))
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概  要: 属性チェック
    #=================================================================
    def attribute_check()
      aErrors = []
      @hState.each{|sAtr, val|
        begin
          sAtr = get_real_attribute_name(sAtr)
          if (is_specified?(sAtr))
            case sAtr
            # 0以上の整数
            when TSR_PRM_LEFTTMO, TSR_PRM_LEFTTIM, TSR_PRM_EXINF, TSR_PRM_BOOTCNT, TSR_PRM_PORDER,
                 TSR_PRM_CYCPHS, TSR_PRM_CYCTIM, TSR_PRM_PNDPTN, TSR_PRM_TEXPTN
              check_attribute_unsigned(sAtr, val, @aPath)

            # 0より大きい整数
            when TSR_PRM_PRCID
              check_attribute_gt(sAtr, val, 0, @aPath)

            # 文字列
            when TSR_PRM_CLASS, TSR_PRM_TASK, TSR_PRM_SPINID
              check_attribute_type(sAtr, val, String, false, @aPath)

            # 文字列かNil
            when TSR_PRM_WOBJID
              check_attribute_type(sAtr, val, String, true, @aPath)

            # 0以上の整数か文字列
            when TSR_PRM_ACTPRC, TSR_PRM_EXCNO, TSR_PRM_INHNO, TSR_PRM_INTNO
              unless (val.is_a?(String))
                check_attribute_unsigned(sAtr, val, @aPath)
              end

            # 0より小さい整数か文字列
            when TSR_PRM_INTPRI
              unless (val.is_a?(String))
                check_attribute_lt(sAtr, val, 0, @aPath)
              end

            # 真偽値
            when TSR_PRM_GLOBAL
              check_attribute_type(sAtr, val, Bool, false, @aPath)

            # 優先度
            when TSR_PRM_ITSKPRI, TSR_PRM_TSKPRI, TSR_PRM_ISRPRI
              check_attribute_range(sAtr, val, TTC_MAX_PRI, TTC_MIN_PRI, @aPath)

            # 起動キューイング
            when TSR_PRM_ACTCNT, TSR_PRM_WUPCNT
              check_attribute_range(sAtr, val, 0, TTC_MAX_QUEUING, @aPath)

            # do
            when TSR_PRM_DO
              check_attribute_type(sAtr, val, Hash, false, @aPath)
              check_do_attribute(val, @aSpecifiedDoAttributes, @aPath + [sAtr])

            # var
            when TSR_PRM_VAR
              check_attribute_type(sAtr, val, Hash, false, @aPath)
              # 変数名のチェック
              val.each{|sVarName, cVar|
                check_attribute_variable(sAtr, sVarName, @aPath)
                cVar.check_attribute_value()
              }

            # hdlstat
            when TSR_PRM_HDLSTAT
              if (@cConf.is_asp?())
                aList = GRP_ENUM_HDLSTAT_ASP
              elsif (@cConf.is_fmp?())
                aList = GRP_ENUM_HDLSTAT_FMP
              else
                abort(ERR_MSG % [__FILE__, __LINE__])
              end
              check_attribute_type(sAtr, val, String, false, @aPath)
              check_attribute_enum(sAtr, val, aList, @aPath)

            # state
            when TSR_PRM_TSKSTAT, TSR_PRM_ALMSTAT, TSR_PRM_CYCSTAT, TSR_PRM_TEXSTAT, TSR_PRM_INTSTAT
              # TESRYでの値に戻す
              val = get_real_state_value()
              # 指定可能な値
              if (@cConf.is_asp?())
                aList = GRP_OBJECT_STATE_AVAILABLE_ASP
              elsif (@cConf.is_fmp?())
                aList = GRP_OBJECT_STATE_AVAILABLE_FMP
              else
                abort(ERR_MSG % [__FILE__, __LINE__])
              end
              check_attribute_type(sAtr, val, String, false, @aPath)
              check_attribute_enum(sAtr, val, aList[sAtr], @aPath)

            # 属性
            when TSR_PRM_CYCATR, TSR_PRM_INTATR
              check_attribute_type(sAtr, val, String, false, @aPath)
              check_attribute_multi(sAtr, val, GRP_AVAILABLE_OBJATR[@sObjectType], @aPath)

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
    #=================================================================
    def object_check(bIsPre = false)
      check_class(Bool, bIsPre)  # pre_conditionか

      aErrors = []
      # 変数のチェック
      unless (@hState[TSR_PRM_VAR].nil?())
        @hState[TSR_PRM_VAR].each{|sVarName, cVar|
          begin
            cVar.object_check(bIsPre)
          rescue TTCMultiError
            aErrors.concat($!.aErrors)
          end
        }
      end

      # グローバル指定時のチェック
      if (bIsPre == true && (@sObjectType == TSR_OBJ_INIRTN || @sObjectType == TSR_OBJ_TERRTN))
        if (@hState[TSR_PRM_GLOBAL] == true)
          ### T3_INI001: グローバル初期化ルーチンで定義されたとき，classが指定されている
          ### T3_TER001: グローバル終了ルーチンで定義されたとき，classが指定されている
          if (is_specified?(TSR_PRM_CLASS))
            sErr = sprintf(ERR_ON_GLOBAL_SET_CLASS, @sObjectID)
            if (@sObjectType == TSR_OBJ_INIRTN)
              sErr = "T3_INI001: " + sErr
            else
              sErr = "T3_TER001: " + sErr
            end
            aErrors.push(YamlError.new(sErr, @aPath))
          end
          ### T3_INI002: グローバル初期化ルーチンで定義されたとき，prcidが指定されている
          ### T3_TER002: グローバル終了ルーチンで定義されたとき，prcidが指定されている
          if (is_specified?(TSR_PRM_PRCID))
            sErr = sprintf(ERR_ON_GLOBAL_SET_PRCID, @sObjectID)
            if (@sObjectType == TSR_OBJ_INIRTN)
              sErr = "T3_INI002: " + sErr
            else
              sErr = "T3_TER002: " + sErr
            end
            aErrors.push(YamlError.new(sErr, @aPath))
          end
        end
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概　要: オブジェクト内のマクロを置換する
    #=================================================================
    def convert_macro()
      super()

      # 変数
      if (@hState[TSR_PRM_VAR].is_a?(Hash))
        @hState[TSR_PRM_VAR].each_value{|cVar|
          cVar.convert_macro()
        }
      end
    end

    #=================================================================
    # 概  要: グローバル対応のためパラメータの変換を実行する
    #=================================================================
    def convert_global(hTable, hClassTable)
      check_class(Hash, hTable)       # 変換表
      check_class(Hash, hClassTable)  # クラス置換対応表

      super(hTable, hClassTable)

      # 変数
      unless (@hState[TSR_PRM_VAR].nil?())
        @hState[TSR_PRM_VAR].each_value{|cVar|
          cVar.convert_global(hTable)
        }
      end
    end

    #=================================================================
    # 概  要: 初期値を補完する
    #=================================================================
    def complement_init_object_info()
      # 初期化・終了ルーチン
      if (@sObjectType == TSR_OBJ_INIRTN || @sObjectType == TSR_OBJ_TERRTN)
        hMacro = @cConf.get_macro()

        # global
        unless (is_specified?(TSR_PRM_GLOBAL))
          @hState[TSR_PRM_GLOBAL] = false
        end
        # exinf
        unless (is_specified?(TSR_PRM_EXINF))
          @hState[TSR_PRM_EXINF] = hMacro["EXINF_A"]
        end
        # gcov
        if (!@aSpecifiedDoAttributes.include?(TSR_PRM_GCOV) &&
            (@aSpecifiedDoAttributes.include?(TSR_PRM_SYSCALL) || @aSpecifiedDoAttributes.include?(TSR_PRM_CODE)))
          @hState[TSR_PRM_DO][TSR_PRM_GCOV] = true
        end
      end

      # fmp限定
      if (@cConf.is_fmp?())
        # globalの初期化ルーチン・終了ルーチン
        if ((@sObjectType == TSR_OBJ_INIRTN || @sObjectType == TSR_OBJ_TERRTN) && @hState[TSR_PRM_GLOBAL] == true)
          # class
          unless (is_specified?(TSR_PRM_CLASS))
            @hState[TSR_PRM_CLASS] = IMC_NO_CLASS
          end
          # prcid
          unless (is_specified?(TSR_PRM_PRCID))
            @hState[TSR_PRM_PRCID] = "MAIN_PRCID"
          end
        else
          # class
          unless (is_specified?(TSR_PRM_CLASS))
            if (@sObjectType == TSR_OBJ_ISR)
              @hState[TSR_PRM_CLASS] = CFG_MCR_CLS_SELF_ONLY_SELF
            else
              @hState[TSR_PRM_CLASS] = CFG_MCR_CLS_SELF_ALL
            end
          end
          # prcid
          unless (is_specified?(TSR_PRM_PRCID))
            @hState[TSR_PRM_PRCID] = CFG_MCR_PRC_SELF
          end
        end
      end
    end

    #=================================================================
    # 概　要: 補完を実行する
    #=================================================================
    def complement(cPrevObj)
      check_class(ProcessUnit, cPrevObj)  # 直前の状態のオブジェクト

      # 指定されていない場合補完
      cPrevObj.hState.each{|sAtr, val|
        sRealAtr = get_real_attribute_name(sAtr)
        if (!is_specified?(sRealAtr))
          # 変数の補完
          if (sRealAtr == TSR_PRM_VAR && val.is_a?(Hash))
            complement_variable(val)
          else
            @hState[sAtr] = safe_dup(val)
          end
        # 変数の省略された属性の補完
        elsif (sRealAtr == TSR_PRM_VAR && val.is_a?(Hash))
          complement_variable(val)
        end
      }
    end

    #=================================================================
    # 概　要: 変数の補完を実行する
    #=================================================================
    def complement_variable(hPrevVar)
      check_class(Hash, hPrevVar)  # 直前の状態の変数

      if (@hState[TSR_PRM_VAR].nil?())
        @hState[TSR_PRM_VAR] = {}
      end
      hPrevVar.each{|sVarName, cVar|
        # 変数の情報が省略されている場合は複製
        if (@hState[TSR_PRM_VAR][sVarName].nil?())
          @hState[TSR_PRM_VAR][sVarName] = cVar.dup()
          @hState[TSR_PRM_VAR][sVarName].aSpecifiedAttributes = []
        # 変数の情報がある場合は補完
        else
          @hState[TSR_PRM_VAR][sVarName].complement(cVar)
        end
      }
    end

    #=================================================================
    # 概　要: エイリアスを実行する
    #=================================================================
    def alias(hAlias)
      check_class(Hash, hAlias)   # エイリアス変換テーブル

      super(hAlias)

      # 変数
      unless (@hState[TSR_PRM_VAR].nil?())
        hTmp = @hState[TSR_PRM_VAR].dup()
        @hState[TSR_PRM_VAR] = {}
        hTmp.each{|sVarName, cVar|
          sAlias = hAlias[sVarName]
          cVar.alias(hAlias)
          @hState[TSR_PRM_VAR][sAlias] = cVar
        }
      end

      # do
      unless (@hState[TSR_PRM_DO].nil?())
        @hState[TSR_PRM_DO].each{|sAtr, val|
          @hState[TSR_PRM_DO][sAtr] = alias_replace(val, hAlias)
        }
      end
    end

    #=================================================================
    # 概　要: オブジェクトのエイリアス変換テーブルを返す
    #=================================================================
    def get_alias(sTestID, nNum = nil)
      check_class(String, sTestID)      # テストID
      check_class(Integer, nNum, true)  # 識別番号

      hResult = {}

      # オブジェクトID
      case @sObjectType
      # 割込みハンドラの場合，ラベル+割込み番号へ置き換える
      when TSR_OBJ_INTHDR
        hResult[@sObjectID] = "#{TTG_LBL_INTHDR}_#{@hState[TSR_PRM_INTNO]}"
      # 割込みサービスルーチンの場合，ラベル+割込み番号+テストID+オブジェクトIDへ置き換える
      when TSR_OBJ_ISR
        hResult[@sObjectID] = "#{TTG_LBL_ISR}_#{@hState[TSR_PRM_INTNO]}_#{alias_str(@sObjectID, sTestID)}"
      # CPU例外ハンドラの場合，ラベル+CPU例外ハンドラ番号へ置き換える
      when TSR_OBJ_EXCEPTION
        hResult[@sObjectID] = "#{TTG_LBL_EXCEPTION}_#{@hState[TSR_PRM_EXCNO]}"
      else
        hResult[@sObjectID] = alias_str(@sObjectID, sTestID)
      end

      # 変数名
      unless (@hState[TSR_PRM_VAR].nil?())
        @hState[TSR_PRM_VAR].each_key{|sVarName|
          hResult[sVarName] = alias_str(sVarName, sTestID)
        }
      end

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
          case sAtr
          # 変数
          when TSR_PRM_VAR
            hYaml[TSR_PRM_VAR] = {}
            val.each{|sVarName, cVar|
              hYaml[TSR_PRM_VAR][sVarName] = cVar.to_yaml()
            }
          # 状態
          when TSR_PRM_STATE
            hYaml[sRealAtr] = get_real_state_value()
          # その他
          else
            hYaml[sRealAtr] = safe_dup(val)
          end
        end
      }

      return hYaml  # [Hash]YAMLオブジェクト
    end

    #=================================================================
    # 概  要: オブジェクト内に存在する変数名一覧を返す
    #=================================================================
    def get_variable_names()
      aResult = []
      unless (@hState[TSR_PRM_VAR].nil?())
        aResult = @hState[TSR_PRM_VAR].keys()
      end

      return aResult  # [Array]オブジェクト内に存在する変数名一覧
    end

    #=================================================================
    # 概  要: 内部保持用属性名からTESRYの属性名を取得する
    #=================================================================
    def get_real_attribute_name(sAtr)
      check_class(String, sAtr)  # 内部保持用属性名

      case sAtr
      when TSR_PRM_STATE
        sAtr = GRP_PRM_KEY_PROC_STAT[@sObjectType]
      when TSR_PRM_ATR
        sAtr = GRP_PRM_KEY_PROC_ATR[@sObjectType]
      when TSR_PRM_LEFTTMO
        if (@sObjectType != TSR_OBJ_TASK)
          sAtr = TSR_PRM_LEFTTIM
        end
      end

      return sAtr  # [String]TESRYの属性名
    end

    #=================================================================
    # 概  要: 内部保持用状態属性からTESRYの状態属性を取得する
    #=================================================================
    def get_real_state_value()
      if (@bConvertState == true)
        hTempHash = GRP_OBJECT_STATE.invert()
        val = hTempHash[@hState[TSR_PRM_STATE]]
      else
        val = @hState[TSR_PRM_STATE]
      end

      return val  # [String]TESRYの属性値
    end

    #=================================================================
    # 概  要: YAML階層のパスを設定する
    #=================================================================
    def set_path(aPath)
      check_class(Array, aPath)  # ルートからのパス

      super(aPath)
      if (@hState[TSR_PRM_VAR].is_a?(Hash))
        @hState[TSR_PRM_VAR].each{|sVarName, cVar|
          cVar.aPath = aPath + [sVarName]
        }
      end
    end

    #=================================================================
    # 概  要: Yamlに記述されているパラメータリストを記録する
    #=================================================================
    def set_specified_attribute(hObjectInfo)
      check_class(Hash, hObjectInfo)  # オブジェクト情報

      super(hObjectInfo)

      # 初期化・終了ルーチンのdo指定
      if (hObjectInfo[TSR_PRM_DO].is_a?(Hash))
        @aSpecifiedDoAttributes = hObjectInfo[TSR_PRM_DO].keys()
      end
    end

    #=================================================================
    # 概  要: valueを定義した変数があるかを返す
    #=================================================================
    def is_value_set_variable?()
      bResult = false

      if (is_specified?(TSR_PRM_VAR))
        @hState[TSR_PRM_VAR].each_value{|cVar|
          if (cVar.is_value_set?())
            bResult = true
            break
          end
        }
      end

      return bResult  # [Bool]valueを定義した変数があるか
    end

    #=================================================================
    # 概  要: activateな処理単位かを返す
    #=================================================================
    def is_activate?()
      case @sObjectType
      when TSR_OBJ_TASK
        bResult = GRP_ACTIVATE.include?(@hState[TSR_PRM_STATE])
      when TSR_OBJ_INIRTN, TSR_OBJ_TERRTN
        bResult = false
      else
        bResult = GRP_ACTIVATE.include?(@hState[TSR_PRM_HDLSTAT])
      end

      return bResult  # [Bool]activateな処理単位か
    end

    #=================================================================
    # 概  要: スピンロック待ちの処理単位か
    #=================================================================
    def is_spinlock_waiting?()
      case @sObjectType
      when TSR_OBJ_TASK
        bResult = (@hState[TSR_PRM_STATE] == TSR_STT_R_WAITSPN)
      when TSR_OBJ_INIRTN, TSR_OBJ_TERRTN
        bResult = false
      else
        bResult = (@hState[TSR_PRM_HDLSTAT] == TSR_STT_A_WAITSPN)
      end

      return bResult  # [Bool]スピンロック待ちの処理単位か
    end

    #=================================================================
    # 概  要: lefttmo・lefttimが指定されているか
    #=================================================================
    def has_lefttmo?()
      return !(@hState[TSR_PRM_LEFTTMO].nil?())  # [Bool]lefttmo・lefttimが指定されているか
    end

    #=================================================================
    # 概  要: 割込みハンドラと割込みサービスルーチンの状態が初期状態か
    #=================================================================
    def is_initial_status?()
      return ((@hState[TSR_PRM_ATR] == KER_TA_NULL && @hState[TSR_PRM_STATE] == KER_TA_DISINT) ||
              (@hState[TSR_PRM_ATR] == KER_TA_ENAINT && @hState[TSR_PRM_STATE] == KER_TA_ENAINT))
             # [Bool]割込みハンドラと割込みサービスルーチンの状態が初期状態か
    end
  end
end
