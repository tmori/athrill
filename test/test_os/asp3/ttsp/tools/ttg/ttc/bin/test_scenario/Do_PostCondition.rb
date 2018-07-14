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
#  $Id: Do_PostCondition.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: Do_PostCondition
  # 概    要: do，post_conditionの情報を処理するクラス
  #===================================================================
  class Do_PostCondition < Condition
    #=================================================================
    # 概　要: post_conditionの構造チェック
    #=================================================================
    def basic_post_check(hScenarioPost)
      check_class(Hash, hScenarioPost, true)  # post_condition

      # オブジェクトの構造チェック
      if (hScenarioPost.is_a?(Hash))
        aErrors = []
        aPath = get_condition_path()
        hScenarioPost.each{|sObjectID, hObjectInfo|
          begin
            ### T1_019: post_conditionで状態変化のないオブジェクトが記述されている
            if (hObjectInfo.nil?())
              sErr = sprintf("T1_019: " + ERR_OBJECT_NOCHANGE, sObjectID)
              aErrors.push(YamlError.new(sErr, aPath + [sObjectID]))
            else
              common_basic_check(sObjectID, hObjectInfo, aPath)
              ### T1_020: post_condition内のオブジェクトにtype属性が定義されている
              if (hObjectInfo.has_key?(TSR_PRM_TYPE))
                sErr = sprintf("T1_020: " + ERR_ONLY_PRE_ATR, TSR_PRM_TYPE)
                aErrors.push(YamlError.new(sErr, aPath + [sObjectID]))
              end
            end
          rescue TTCMultiError
            aErrors.concat($!.aErrors)
          end
        }

        check_error(aErrors)
      end
    end

    #=================================================================
    # 概　要: 属性チェック(何の属性を指定したかをチェック)
    #=================================================================
    def pre_attribute_check(hScenarioDo)
      check_class(Hash, hScenarioDo)  # do

      aPath       = get_do_path()
      aAtrDefList = get_do_attribute_list().keys()

      check_defined_attribute(hScenarioDo.keys(), aAtrDefList, aPath)
    end

    #=================================================================
    # 概  要: 属性チェック
    #=================================================================
    def attribute_check()
      aErrors = []

      aPath = get_do_path()
      # オブジェクトの属性チェック
      begin
        super()
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      ### T3_DO001: syscallかcodeが指定されている場合に発行元オブジェクトIDが記述されていない
      if ((!@hDo[TSR_PRM_SYSCALL].nil?() || !@hDo[TSR_PRM_CODE].nil?()) && @hDo[TSR_PRM_ID].nil?())
        sErr = sprintf("T3_DO001: " + ERR_REQUIRED_KEY, TSR_PRM_ID)
        aErrors.push(YamlError.new(sErr, aPath + [TSR_PRM_ID]))
      end

      ### T2: doの属性指定チェック
      check_do_attribute(@hDo, @aSpecifiedDoAttributes, aPath)

      check_error(aErrors)
    end

    #=================================================================
    # 概  要: コンディションチェック
    #=================================================================
    def condition_check()
      aErrors = []

      begin
        super()
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      # doのチェック
      aPath = get_do_path()
      ### T4_036: doでcodeが指定されている時に戻り値が指定されている
      if (!@hDo[TSR_PRM_CODE].nil?() && exist_error_code?())
        aErrors.push(YamlError.new("T4_036: " + ERR_SET_ERRCODE_SET_CODE, aPath))
      end
      ### T4_037: doでsyscallとcodeが同時に指定されている
      if (!@hDo[TSR_PRM_SYSCALL].nil?() && !@hDo[TSR_PRM_CODE].nil?())
        aErrors.push(YamlError.new("T4_037: " + ERR_SET_SYSCALL_AND_CODE, aPath))
      end

      # post_conditionのチェック
      aPath = get_condition_path()
      # 初期化ルーチン，終了ルーチンのチェック
      hObjects = get_objects_by_type([TSR_OBJ_INIRTN, TSR_OBJ_TERRTN])
      hObjects.each{|sObjectID, cObjectInfo|
        ### T4_005: 初期化ルーチン，終了ルーチンがpre_condition以外で指定されている
        sErr = sprintf("T4_005: " + ERR_CANNNOT_DEFINED_IN_POST, cObjectInfo.sObjectType)
        aErrors.push(YamlError.new(sErr, aPath + [sObjectID]))
      }

      check_error(aErrors)
    end

    #=================================================================
    # 概　要: コンディション内の全てのマクロを置換する
    #=================================================================
    def convert_macro()
      super()

      hMacro = @cConf.get_macro()
      @hDo.each{|sAtr, val|
        @hDo[sAtr] = calc_all_expr(@hDo[sAtr], hMacro)
      }
    end

    #=================================================================
    # 概　要: 補完を実行する
    #=================================================================
    def complement(cPrevCondition)
      check_class(Condition, cPrevCondition)  # 直前のコンディション

      # gcov（doがある場合のみ）
      if (!@hDo.empty?() && !@aSpecifiedDoAttributes.include?(TSR_PRM_GCOV))
        @hDo[TSR_PRM_GCOV] = true
      end

      cPrevCondition.hAllObject.each{|sObjectID, cObjectInfo|
        # 補完しないオブジェクト以外は補完実行
        unless (GRP_NOT_COMPLEMENT_TYPE.include?(cObjectInfo.sObjectType))
          # postに記述されている場合省略された属性を補完
          unless (@hAllObject[sObjectID].nil?())
            @hAllObject[sObjectID].complement(cObjectInfo)
          # post_conditionに記述されていない場合pre_conditionからコピー
          else
            cNewObjectInfo = cObjectInfo.dup()
            cNewObjectInfo.aSpecifiedAttributes = []
            cNewObjectInfo.set_path(get_condition_path() + [cNewObjectInfo.sObjectID])
            @hAllObject[sObjectID] = cNewObjectInfo
          end
        end
      }
    end

    #=================================================================
    # 概　要: エイリアスを実行する
    #=================================================================
    def alias(hAlias)
      check_class(Hash, hAlias)   # エイリアス変換テーブル

      # post_conditionのエイリアス
      super(hAlias)

      # doのエイリアス
      @hDo.each_key{|sAtr, val|
        @hDo[sAtr] = alias_replace(@hDo[sAtr], hAlias)
      }
    end

    #=================================================================
    # 概　要: doのパス情報を返す
    #=================================================================
    def get_do_path()
      return [@sTestID, "#{TSR_UNS_DO}#{@nSeqNum}", @nTimeTick]  # [Array]doのパス情報
    end

    #=================================================================
    # 概　要: post_conditionのパス情報を返す
    #=================================================================
    def get_condition_path()
      return [@sTestID, "#{TSR_UNS_POST}#{@nSeqNum}", @nTimeTick]  # [Array]post_conditionのパス情報
    end

    #=================================================================
    # 概  要: グローバル対応のためパラメータの変換を実行する
    #=================================================================
    def convert_global(hTable, hClassTable)
      check_class(Hash, hTable)       # 変換テーブル
      check_class(Hash, hClassTable)  # クラス置換対応表

      super(hTable, hClassTable)

      # syscall，code
      [TSR_PRM_SYSCALL, TSR_PRM_CODE].each{|sAtr|
        if (@hDo[sAtr].is_a?(String))
          @hDo[sAtr] = convert_global_params(@hDo[sAtr], hTable)
        end
      }
    end

    #=================================================================
    # 概  要: エラーコードが指定されているかを返す
    #=================================================================
    def exist_error_code?()
      return (!@hDo[TSR_PRM_ERCD].nil?() || !@hDo[TSR_PRM_ERUINT].nil?() || !@hDo[TSR_PRM_BOOL].nil?())  # [Bool]エラーコードが指定されているか
    end
  end
end
