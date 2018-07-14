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
#  $Id: TestScenario.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "ttc/bin/class/TTCCommon.rb"
require "common/bin/test_scenario/TestScenario.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: TestScenario
  # 概    要: PreCondition，Do，PostConditionのデータを保持
  #===================================================================
  class TestScenario
    include TTCModule

    #=================================================================
    # 概　要: 構造チェック
    #=================================================================
    def basic_check(sTestID, hScenario)
      check_class(Object, sTestID, true)    # テストID
      check_class(Object, hScenario, true)  # テストシナリオ

      aErrors = []
      ### T1_003: テストIDの文字列が不正
      unless (sTestID =~ TSR_REX_TEST_ID)
        sErr = sprintf("T1_003: " + ERR_INVALID_TESTID, sTestID)
        aErrors.push(YamlError.new(sErr))
      else
        ### T1_028: テストIDに予約語が使われている
        if (sTestID =~ /^#{TTG_LBL_RESERVED}/i)
          sErr = sprintf("T1_028: " + ERR_INVALID_TESTID_PREFIX, sTestID)
          aErrors.push(YamlError.new(sErr))
        end
      end

      ### T1_004: テストシナリオがHashではない
      unless (hScenario.is_a?(Hash))
        sErr = sprintf("T1_004: " + ERR_INVALID_TYPE_SCENARIO, Hash, hScenario.class())
        aErrors.push(YamlError.new(sErr))
      else
        aPath = [sTestID]

        # do，post_conditionのチェック
        bExistPre = false
        aSeqDo = []
        aSeqPost = []
        hScenario.each{|sCondition, shCondition|
          case sCondition
          # pre_condition
          when TSR_LBL_PRE
            bExistPre = true
            ### T1_007: pre_conditionがHashではない
            unless (shCondition.is_a?(Hash))
              sErr = sprintf("T1_007: " + ERR_INVALID_TYPE, TSR_LBL_PRE, Hash, shCondition.class())
              aErrors.push(YamlError.new(sErr, aPath + [sCondition]))
            end

          # do
          when TSR_REX_PRE_DO
            nSeqNum = $1.to_i()
            ### T1_009: doのシーケンス番号が二重に定義されている
            if (aSeqDo.include?(nSeqNum))
              sErr = sprintf("T1_009: " + ERR_CONDITION_MULTIPLE, TSR_LBL_DO, nSeqNum)
              aErrors.push(YamlError.new(sErr, aPath))
            end
            aSeqDo.push(nSeqNum)

            ### T1_010: doがHashかnilではない
            unless (shCondition.nil?() || shCondition.is_a?(Hash))
              sErr = sprintf("T1_010: " + ERR_INVALID_TYPE_NIL, TSR_LBL_DO, Hash, shCondition.class())
              aErrors.push(YamlError.new(sErr, aPath + [sCondition]))
            end

            # タイムティック
            begin
              basic_check_timetick(sCondition, shCondition, aPath)
            rescue TTCMultiError
              aErrors.concat($!.aErrors)
            end

          # post_condition
          when TSR_REX_PRE_POST
            nSeqNum = $1.to_i()
            ### T1_011: post_conditionのシーケンス番号が二重に定義されている
            if (aSeqPost.include?(nSeqNum))
              sErr = sprintf("T1_011: " + ERR_CONDITION_MULTIPLE, TSR_LBL_POST, nSeqNum)
              aErrors.push(YamlError.new(sErr, aPath))
            end
            aSeqPost.push(nSeqNum)

            ### T1_012: post_conditionがHashかnilではない
            unless (shCondition.nil?() || shCondition.is_a?(Hash))
              sErr = sprintf("T1_012: " + ERR_INVALID_TYPE_NIL, TSR_LBL_POST, Hash, shCondition.class())
              aErrors.push(YamlError.new(sErr, aPath + [sCondition]))
            end

            # タイムティック
            begin
              basic_check_timetick(sCondition, shCondition, aPath)
            rescue TTCMultiError
              aErrors.concat($!.aErrors)
            end

          # variation
          when TSR_LBL_VARIATION
            begin
              basic_check_variation(hScenario[TSR_LBL_VARIATION], aPath)
            rescue TTCMultiError
              aErrors.concat($!.aErrors)
            end

          # note
          when TSR_LBL_NOTE
            ### T1_027: noteが文字列ではない
            unless (shCondition.is_a?(String))
              sErr = sprintf("T1_027: " + ERR_INVALID_TYPE, TSR_LBL_NOTE, String, shCondition.class())
              aErrors.push(YamlError.new(sErr, aPath + [sCondition]))
            end

          ### T1_005: テストシナリオ内不正なキーが指定されている
          else
            sErr = sprintf("T1_005: " + ERR_CONDITION_UNDEFINED, sCondition)
            aErrors.push(YamlError.new(sErr, aPath))
          end
        }

        ### T1_006: pre_conditionが存在しない
        if (bExistPre == false)
          sErr = sprintf("T1_006: " + ERR_CONDITION_NOT_EXIST, TSR_LBL_PRE)
          aErrors.push(YamlError.new(sErr, aPath))
        end
        ### T1_008: doが存在しない
        if (aSeqDo.empty?())
          sErr = sprintf("T1_008: " + ERR_CONDITION_NOT_EXIST, TSR_LBL_DO)
          aErrors.push(YamlError.new(sErr, aPath))
        else
          ### T1_013: doとpost_conditionのシーケンス番号が対応していない
          if (aSeqDo.uniq.sort() != aSeqPost.uniq.sort())
            aErrors.push(YamlError.new("T1_013: " + ERR_CONDITION_NOT_MATCH, aPath))
          end
        end
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概　要: バリエーション設定の構造チェック
    #=================================================================
    def basic_check_variation(hVariation, aPath)
      check_class(Object, hVariation, true)  # バリエーション
      check_class(Array, aPath)            # ルートからのパス

      aErrors = []
      ### T1_021: variationがHashかnilではない
      unless (hVariation.nil?() || hVariation.is_a?(Hash))
        sErr = sprintf("T1_021: " + ERR_INVALID_TYPE_NIL, TSR_LBL_VARIATION, Hash, hVariation.class())
        aErrors.push(YamlError.new(sErr))
      end

      # パラメータ
      if (hVariation.is_a?(Hash))
        aNewPath = aPath + [TSR_LBL_VARIATION]
        hVariation.each{|sAtr, val|
          case sAtr
          # 真偽値
          when TSR_PRM_GAIN_TIME, TSR_PRM_ENA_EXC_LOCK, TSR_PRM_GCOV_ALL, TSR_PRN_ENA_CHGIPM,
               TSR_PRM_SUPPORT_GET_UTM, TSR_PRM_SUPPORT_ENA_INT, TSR_PRM_SUPPORT_DIS_INT
            unless (Bool.include?(val.class()))
              sErr = sprintf(ERR_VARIATION_INVALID_TYPE, sAtr, "Bool", val.class())
              aErrors.push(YamlError.new(sErr, aNewPath))
            end

          # タイマアーキテクチャ
          when TSR_PRM_TIMER_ARCH
            if (val != TSR_PRM_TIMER_GLOBAL && val != TSR_PRM_TIMER_LOCAL)
              sErr = sprintf(ERR_VARIATION_INVALID_VALUE, sAtr, val)
              aErrors.push(YamlError.new(sErr, aNewPath))
            end

          # IRCアーキテクチャ
          when TSR_PRM_IRC_ARCH
            if (val != TSR_PRM_IRC_GLOBAL && val != TSR_PRM_IRC_LOCAL && val != TSR_PRM_IRC_COMBINATION)
              sErr = sprintf(ERR_VARIATION_INVALID_VALUE, sAtr, val)
              aErrors.push(YamlError.new(sErr, aNewPath))
            end

          # 未定義のキー
          else
            sErr = sprintf(ERR_VARIATION_UNDEFINED_KEY, sAtr)
            aErrors.push(YamlError.new(sErr, aNewPath))
          end
        }
      end

      check_error(aErrors)
    end
    private :basic_check_variation

    #=================================================================
    # 概　要: タイムティックチェック
    #=================================================================
    def basic_check_timetick(sCondition, hCondition, aPath)
      check_class(String, sCondition)        # コンディション名
      check_class(Object, hCondition, true)  # コンディション
      check_class(Array, aPath)              # ルートからのパス

      aErrors = []
      hMacro = @cConf.get_macro()
      if (has_timetick?(hCondition))
        hCondition.each_key{|nTimeTick|
          ### T1_014: タイムティック指定が0より小さい
          if (parse_value(nTimeTick, hMacro) < 0)
            sErr = sprintf("T1_014: " + ERR_INVALID_TIMETICK, nTimeTick)
            aErrors.push(YamlError.new(sErr, aPath))
          end
        }
      end

      check_error(aErrors)
    end
    private :basic_check_timetick

    #=================================================================
    # 概  要: 属性チェック
    #=================================================================
    def attribute_check()
      aErrors = []
      aCondition = get_all_condition()
      aCondition.each{|cCondition|
        begin
          cCondition.attribute_check()
        rescue TTCMultiError
          aErrors.concat($!.aErrors)
        end
      }
      check_error(aErrors)
    end

    #=================================================================
    # 概  要: オブジェクトチェック
    #=================================================================
    def object_check()
      aErrors = []
      begin
        @cPreCondition.object_check(true)
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end
      @aDo_PostCondition.each{|cCondition|
        begin
          cCondition.object_check()
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
      aCondition = get_all_condition()
      aCondition.each{|cCondition|
        begin
          cCondition.condition_check()
        rescue TTCMultiError
          aErrors.concat($!.aErrors)
        end
      }
      check_error(aErrors)
    end

    #=================================================================
    # 概  要: シナリオチェック
    #=================================================================
    def scenario_check()
      aErrors = []

      aPreVarNames   = @cPreCondition.get_variable_names()
      cPrevCondition = @cPreCondition
      @aDo_PostCondition.each_with_index{|cCondition, nIndex|
        # doのチェック
        aPath = cCondition.get_do_path()
        # id
        sCallerObject = cCondition.hDo[TSR_PRM_ID]
        unless (sCallerObject.nil?())
          if (cPrevCondition.hAllObject.has_key?(sCallerObject))
            cPrevCaller      = cPrevCondition.hAllObject[sCallerObject]
            nPrcid           = cPrevCondition.get_process_prcid(cPrevCaller)
            aActivateProcess = cPrevCondition.get_activate_process_unit_by_prcid()
            if (GRP_PROCESS_UNIT_ALL.include?(cPrevCaller.sObjectType))
              # APIを発行できる処理単位かチェック
              if (cPrevCaller != aActivateProcess[nPrcid])
                ### T5_F001: タスク状態がrunning-suspendedの時，非タスクが実行状態ならばdoでapiを発行しているのはその非タスクか
                if (cPrevCaller.sObjectType == TSR_OBJ_TASK && cPrevCaller.is_running_suspended?())
                  sErr = sprintf("T5_F001: " + ERR_DO_ID_MISMATCH_RUS_TASK, aActivateProcess[nPrcid].sObjectID)
                ### T5_005: doでAPIを発行するオブジェクトが同一タイムティック内の直前の状態で実行状態でない
                else
                  sErr = sprintf("T5_005: " + ERR_OBJECT_NOT_ACTIVATE_PREV, sCallerObject)
                end
                aErrors.push(YamlError.new(sErr, aPath + [TSR_PRM_ID]))
              end

            ### T5_004: doでAPIを発行するオブジェクトが処理単位でない
            else
              sErr = sprintf("T5_004: " + ERR_TARGET_NOT_PROCESS_UNIT, sCallerObject)
              aErrors.push(YamlError.new(sErr, aPath + [TSR_PRM_ID]))
            end
          ### T5_003: doでAPIを発行するオブジェクトが存在しない
          else
            sErr = sprintf("T5_003: " + ERR_OBJECT_NOT_DEFINED_IN_PRE, sCallerObject)
            aErrors.push(YamlError.new(sErr, aPath + [TSR_PRM_ID]))
          end
        end

        # post_conditionのチェック
        # 変数定義のチェック
        aNotDefined = cCondition.get_variable_names() - aPreVarNames
        ### T5_002: post_conditionにpre_conditionで定義されていない変数が存在する
        unless (aNotDefined.empty?())
          sErr  = sprintf("T5_002: " + ERR_VAR_NOT_DEFINED_IN_PRE, aNotDefined.join(", "))
          aPath = cCondition.get_condition_path()
          aErrors.push(YamlError.new(sErr, aPath))
        end

        # 直前のコンディションとの関係チェック
        if (cPrevCondition != @cPreCondition)
          aPath = [@sTestID]
          # 時間指定のチェック
          if (cPrevCondition.nTimeTick > cCondition.nTimeTick)
            ### T5_006: タイムティック指定で過去に戻る
            sErr = sprintf("T5_006: " + ERR_TIME_RETURN, cPrevCondition.nTimeTick, cCondition.nTimeTick)
            aErrors.push(YamlError.new(sErr, aPath))
          elsif (cPrevCondition.nTimeTick != cCondition.nTimeTick)
            ### T5_008: CPUロック状態で時間が進む
            if (cPrevCondition.exist_cpulock?())
              sErr = sprintf("T5_008: " + ERR_TIME_PROGRESS_IN_CPU_LOCK, cPrevCondition.nTimeTick, cCondition.nTimeTick)
              aErrors.push(YamlError.new(sErr, aPath))
            end
=begin
            ### T5_009: CPU状態がchg_ipm!=0 の状態で時間が進む
            if (cPrevCondition.exist_disable_interrupt?())
              sErr = sprintf("T5_009: " + ERR_TIME_PROGRESS_CHG_IPM, cPrevCondition.nTimeTick, cCondition.nTimeTick)
              aErrors.push(YamlError.new(sErr, aPath))
            end
=end
            ### T5_010: 非タスク実行中に時間が経過する
            if (exist_keep_activate_non_context?(cPrevCondition, cCondition))
              sErr = sprintf("T5_010: " + ERR_TIME_PROGRESS_NON_CONTEXT, cPrevCondition.nTimeTick, cCondition.nTimeTick)
              aErrors.push(YamlError.new(sErr, aPath))
            end
          end
        end

        # エラーコード指定
        begin
          unless (cCondition.exist_error_code?())
            scenario_check_require_ercd(nIndex)
          else
            scenario_check_cannot_check_ercd(nIndex)
          end
        rescue YamlError
          aErrors.push($!)
        end

        # 直前のコンディション
        cPrevCondition = cCondition
      }


      # FMP限定
      if (@cConf.is_fmp?())
        # running-waitspin
        aCondition = [@cPreCondition, @aDo_PostCondition.last()]
        aCondition.each{|cCondition|
          hObjects = cCondition.get_objects_by_type(TSR_OBJ_TASK)
          hObjects.each{|sObjectID, cObjectInfo|
            ### T5_F003: pre_conditionおよび最後のpost_conditionに存在するタスクの状態がrunning-waitspinになっている
            if (cObjectInfo.is_spinlock_waiting?())
              sErr = sprintf("T5_F003: " + ERR_CANNOT_BE_RUNNING_WAITSPIN, sObjectID)
              aPath = cCondition.get_condition_path() + [sObjectID, TSR_PRM_TSKSTAT]
              aErrors.push(YamlError.new(sErr, aPath))
            end
          }
        }
        # グローバルタイマ
        if (!@cConf.is_timer_local?() && @hVariation[TSR_PRM_TIMER_ARCH] != TSR_PRM_TIMER_LOCAL)
          aPrcid = []
          aCondition = get_all_condition()
          aCondition.each{|cCondition|
            hObjects = cCondition.get_objects_by_type(GRP_TIME_EVENT_HDL)
            hObjects.each_value{|cObjectInfo|
              aPrcid.push(cObjectInfo.hState[TSR_PRM_PRCID])
            }
          }
          ### T5_F004: グローバルタイマ指定時，タイムイベントハンドラが複数のプロセッサで定義されている
          if (aPrcid.uniq().size() > 1)
            aErrors.push(YamlError.new("T5_F004: " + ERR_TIMEEVENT_PRCID_GLOBAL, [@sTestID]))
          end
        end
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概　要: エラーコード指定漏れチェック
    #=================================================================
    def scenario_check_require_ercd(nIndex)
      check_class(Integer, nIndex)  # 何番目のdo，postか

      cCondition      = @aDo_PostCondition[nIndex]
      aCondition      = get_all_condition()
      cPrevCondition  = aCondition[nIndex]
      aAfterCondition = aCondition.slice(nIndex + 1, aCondition.size())
      sCallerObject   = cCondition.hDo[TSR_PRM_ID]
      aPath           = cCondition.get_do_path()
      ### T5_012: エラーコードが指定されていない
      unless (cCondition.hDo[TSR_PRM_SYSCALL].nil?())
        # 後続のコンディションチェック
        aAfterCondition.each{|cTargetCondition|
          cObjectInfo = cTargetCondition.hAllObject[sCallerObject]
          aActivate   = cTargetCondition.get_activate_by_prcid()
          nPrcid      = cTargetCondition.get_process_prcid(cObjectInfo)
          # 起動しているか
          if (!nPrcid.nil?() && !cObjectInfo.nil?() && GRP_PROCESS_UNIT_ALL.include?(cObjectInfo.sObjectType) &&
              ((aActivate[nPrcid].nil?() && cObjectInfo.is_activate?()) || aActivate[nPrcid] == cObjectInfo)
             )
            # タスクの場合
            if (cObjectInfo.sObjectType == TSR_OBJ_TASK)
              cPrevObject = cPrevCondition.hAllObject[sCallerObject]
              cTex        = cTargetCondition.get_task_exc_by_task(sCallerObject)
              unless ((!cTex.nil?() && cTex.is_activate?()) || (cPrevObject.hState[TSR_PRM_BOOTCNT] < cObjectInfo.hState[TSR_PRM_BOOTCNT]))
                sErr = sprintf("T5_012: " + ERR_NOT_DEFINED_ERROR_CODE, sCallerObject)
                raise(YamlError.new(sErr, aPath))
              end
            else
              sErr = sprintf("T5_012: " + ERR_NOT_DEFINED_ERROR_CODE, sCallerObject)
              raise(YamlError.new(sErr, aPath))
            end
          end
          cPrevCondition = cTargetCondition
        }
      end
    end

    #=================================================================
    # 概　要: エラーコード確認不能チェック
    #=================================================================
    def scenario_check_cannot_check_ercd(nIndex)
      check_class(Integer, nIndex)  # 何番目のdo，postか

      cCondition      = @aDo_PostCondition[nIndex]
      aCondition      = get_all_condition()
      cPrevCondition  = aCondition[nIndex]
      aAfterCondition = aCondition.slice(nIndex + 1, aCondition.size())
      sCallerObject   = cCondition.hDo[TSR_PRM_ID]
      # 後続のコンディションチェック
      bActivate = false
      aAfterCondition.each{|cTargetCondition|
        cObjectInfo = cTargetCondition.hAllObject[sCallerObject]
        unless (cObjectInfo.nil?())
          aActivate   = cTargetCondition.get_activate_by_prcid()
          nPrcid      = cTargetCondition.get_process_prcid(cObjectInfo)
          # 起動しているか
          if ((aActivate[nPrcid].nil?() && cObjectInfo.is_activate?()) || aActivate[nPrcid] == cObjectInfo)
            # タスクの場合
            if (cObjectInfo.sObjectType == TSR_OBJ_TASK)
              cPrevObject = cPrevCondition.hAllObject[sCallerObject]
              cTex        = cTargetCondition.get_task_exc_by_task(sCallerObject)
              unless ((!cTex.nil?() && cTex.is_activate?()) || (cPrevObject.hState[TSR_PRM_BOOTCNT] < cObjectInfo.hState[TSR_PRM_BOOTCNT]))
                bActivate = true
                break
              end
            else
              bActivate = true
              break
            end
          end
        end
        cPrevCondition = cTargetCondition
      }
      ### T5_013: エラーコードが確認できない
      if (bActivate == false)
        sErr = sprintf("T5_013: " + ERR_CANNNOT_CHECK_ERROR_CODE, sCallerObject)
        raise(YamlError.new(sErr, cCondition.get_do_path()))
      end
    end

    #=================================================================
    # 概  要: シナリオチェック(マクロ部分のみ)
    #=================================================================
    def scenario_check_macro()
      aMacro = get_all_prcid()
      ### T5_F002: prcidに指定するマクロがPRC_OTHERとPRC_OTHER_1・PRC_OTHER_2のいずれかと同時に指定されている
      if (aMacro.include?(CFG_MCR_PRC_OTHER) && (aMacro.include?(CFG_MCR_PRC_OTHER_1) || aMacro.include?(CFG_MCR_PRC_OTHER_2)))
        aMacro.delete(CFG_MCR_PRC_SELF)
        aMacro = aMacro.reject(){|val|
          !(val =~ /^PRC_/)
        }
        aMacro = aMacro.uniq().sort()
        sErr = sprintf("T5_F002: " + ERR_PRCID_MACRO_COMBINATION, aMacro.join(", "))
        raise(YamlError.new(sErr, [@sTestID]))
      end
    end

    #=================================================================
    # 概　要: 全てのマクロを置換する
    #=================================================================
    def convert_macro()
      aCondition = get_all_condition()
      aCondition.each{|cCondition|
        cCondition.convert_macro()
      }
    end

    #=================================================================
    # 概　要: 全てのpost_conditionの補完を実行する
    #=================================================================
    def complement()
      # デフォルト値補完
      @cPreCondition.complement_init_object_info()
      # 直前の状態から補完
      cPrevCondition = @cPreCondition
      @aDo_PostCondition.each{|cCondition|
        cCondition.complement(cPrevCondition)
        cPrevCondition = cCondition
      }
      # 後処理
      @cPreCondition.complement_after()
      cPrevCondition = @cPreCondition
      @aDo_PostCondition.each{|cCondition|
        cCondition.complement_after()
      }
    end

    #=================================================================
    # 概　要: エイリアスを実行する
    #=================================================================
    def alias()
      hAlias = @cPreCondition.get_alias()
      aCondition = get_all_condition()
      aCondition.each{|cCondition|
        cCondition.alias(hAlias)
      }
    end

    #=================================================================
    # 概  要: テストシナリオの内容をYAMLオブジェクトに変換して返す
    #=================================================================
    def to_yaml()
      hYaml = {}
      hYaml[@sTestID] = Hash.new{|hash, key|
        hash[key] = {}
      }
      # pre_condition
      hYaml[@sTestID][TSR_LBL_PRE] = @cPreCondition.to_yaml(true)
      # do，post_condition
      @aDo_PostCondition.each{|cCondition|
        nTimeTick = cCondition.nTimeTick
        sDoSeq    = "#{TSR_UNS_DO}#{cCondition.nSeqNum}"
        sPostSeq  = "#{TSR_UNS_POST}#{cCondition.nSeqNum}"
        hYaml[@sTestID][sDoSeq][nTimeTick]   = cCondition.hDo
        hYaml[@sTestID][sPostSeq][nTimeTick] = cCondition.to_yaml()
      }
      return safe_dup(hYaml)  # [Hash]YAMLオブジェクト
    end

    #=================================================================
    # 概　要: バリエーションチェック
    #=================================================================
    def variation_check()
      aErrors = []

      # 時間制御
      begin
        @bGainTime = is_gain_time_mode?()
      rescue TTCExcludeError
        aErrors.push($!)
      end

      ### T6_002: 割込み発生関数が必要なのに使用不可
      if (@cConf.get_func_interrupt() == false && exist_interrupt_handler?())
        sErr = sprintf("T6_002: " + ERR_VARIATION_NO_FUNC, CFG_FUNC_INTERRUPT)
        aErrors.push(TTCExcludeError.new(sErr, :T6_002))
      end
      ### T6_003: CPU例外発生関数が必要なのに使用不可
      if (@cConf.get_func_exception() == false && exist_cpu_exception?())
        sErr = sprintf("T6_003: " + ERR_VARIATION_NO_FUNC, CFG_FUNC_EXCEPTION)
        aErrors.push(TTCExcludeError.new(sErr, :T6_003))
      end

      ### T6_004: CPUロック中にCPU例外発生のサポートがされていない
      if (@cConf.get_enable_exc_in_cpulock() == false && @hVariation[TSR_PRM_ENA_EXC_LOCK] == true)
        aErrors.push(TTCExcludeError.new("T6_004: " + ERR_VARIATION_EXCEPT_IN_CPULOCK, :T6_004))
      end

      ### T6_005: 非タスクコンテキストからの割込み優先度マスク変更サポートがされていない
      if (@cConf.get_enable_chg_ipm_in_non_task() == false && @hVariation[TSR_PRN_ENA_CHGIPM] == true)
        aErrors.push(TTCExcludeError.new("T6_005: " + ERR_VARIATION_CHGIPM_IN_NONTASK, :T6_005))
      end

      ### T6_006: APIのget_utmがサポートされていない
      if (@cConf.get_api_support_get_utm() == false && @hVariation[TSR_PRM_SUPPORT_GET_UTM] == true)
        aErrors.push(TTCExcludeError.new("T6_006: " + ERR_VARIATION_NOT_SUPPORT_GET_UTM, :T6_006))
      end

      ### T6_007: APIのena_intがサポートされていない
      if (@cConf.get_api_support_ena_int() == false && @hVariation[TSR_PRM_SUPPORT_ENA_INT] == true)
        aErrors.push(TTCExcludeError.new("T6_007: " + ERR_VARIATION_NOT_SUPPORT_ENA_INT, :T6_007))
      end

      ### T6_008: APIのdis_intがサポートされていない
      if (@cConf.get_api_support_dis_int() == false && @hVariation[TSR_PRM_SUPPORT_DIS_INT] == true)
        aErrors.push(TTCExcludeError.new("T6_008: " + ERR_VARIATION_NOT_SUPPORT_DIS_INT, :T6_008))
      end

=begin
      ### T6_009: 設定した割込み番号の数よりも多く割込み番号が使用されている
      aAllIntno = get_all_intno()
      if (aAllIntno.size() > @cConf.get_intno_num())
        sErr = sprintf("T6_009: " + ERR_VARIATION_OVER_INTNO_NUM, @cConf.get_intno_num(), aAllIntno.size())
        raise(TTCError.new(sErr))
      end
=end

      ### T6_010: ena_int，dis_intのいずれかがサポートされていない時に割込みハンドラ・割込みサービスルーチンの状態が変化する
      if (@cConf.get_api_support_ena_int() == false || @cConf.get_api_support_dis_int() == false)
        bOnError = false
        aCondition = get_all_condition()
        aCondition.each{|cCondition|
          hObjects = cCondition.get_objects_by_type([TSR_OBJ_INTHDR, TSR_OBJ_ISR])
          hObjects.each{|sObjectID, cObjectInfo|
            unless (cObjectInfo.is_initial_status?())
              aErrors.push(TTCExcludeError.new("T6_010: " + ERR_VARIATION_STATUS_INVALID, :T6_010))
              bOnError = true
              break
            end
          }
          # 既にエラーが確定しているなら抜ける
          if (bOnError == true)
            break
          end
        }
      end

      # fmp
      if (@cConf.is_fmp?())
        # プロセッサ数のチェック
        nMax = @cConf.get_prc_num()
        nProc = get_max_prcid()
        ### T6_F001: 設定したプロセッサ数より多いプロセッサが使用されている
        if (nMax != 0 && nProc > nMax)
          sErr = sprintf("T6_F001: " + ERR_VARIATION_OVER_PRC_NUM, nMax, nProc)
          aErrors.push(TTCExcludeError.new(sErr, :T6_F001))
        end

        # スピンロック数のチェック
        nMax = @cConf.get_spinlock_num()
        nSpinlock = get_spinlock_count()
        ### T6_F002: 設定したスピンロック数より多いスピンロックが使用されている
        if (!nMax.nil?() && nMax != 0 && nSpinlock > nMax)
          sErr = sprintf("T6_F002: " + ERR_VARIATION_OVER_SPINLOCK_NUM, nMax, nSpinlock)
          raise(TTCError.new(sErr))
        end

        ### T6_F003: 自プロセッサ割込みが発生させられないのにpre_conditionで過渡状態のタスクが登場する
        if (@cConf.get_own_ipi_raise() == false && exist_rus_task?())
          aErrors.push(TTCExcludeError.new("T6_F003: " + ERR_VARIATION_CANNOT_OWN_IPI, :T6_F003))
        end

        ### T6_F004: IRCアーキテクチャがconfigureで指定された方式と異なる
        if (!@hVariation[TSR_PRM_IRC_ARCH].nil?() && @hVariation[TSR_PRM_IRC_ARCH] != @cConf.get_irc_arch())
          sErr = sprintf("T6_F004: " + ERR_VARIATION_NOT_MATCH_IRC_ARCH, @cConf.get_irc_arch(), @hVariation[TSR_PRM_IRC_ARCH])
          aErrors.push(TTCExcludeError.new(sErr, "T6_F004_#{@hVariation[TSR_PRM_IRC_ARCH]}".to_sym()))
        end

        ### T6_F005: タイマーアーキテクチャがconfigureで指定された方式と異なる
        if (!@hVariation[TSR_PRM_TIMER_ARCH].nil?() && @hVariation[TSR_PRM_TIMER_ARCH] != @cConf.get_timer_arch())
          sErr = sprintf("T6_F005: " + ERR_VARIATION_NOT_MATCH_TIMER_ARCH, @cConf.get_timer_arch(), @hVariation[TSR_PRM_TIMER_ARCH])
          aErrors.push(TTCExcludeError.new(sErr, :T6_F005))
        end
      end

      # エラーがあった場合
      unless (aErrors.empty?())
        raise(TTCMultiExcludeError.new(aErrors))
      end
    end

    #=================================================================
    # 概  要: コンディション内に存在するプロセッサ番号の最大値を返す
    #=================================================================
    def get_max_prcid()
      nMaxPrcid = 0
      aPrcid = get_all_prcid()
      aPrcid = aPrcid.reject(){|val|
        !val.is_a?(Integer)
      }
      unless (aPrcid.empty?())
        nMaxPrcid = aPrcid.max()
      end

      return nMaxPrcid  # [Integer]コンディション内に存在するプロセッサ番号の最大値
    end

    #=================================================================
    # 概  要: テストシナリオ内に存在する全てのプロセッサ番号を返す
    #=================================================================
    def get_all_prcid()
      aPrcid = []
      # 全てのコンディションの処理単位のprcidを収集する
      aCondition = get_all_condition()
      aCondition.each{|cCondition|
        aPrcid.concat(cCondition.get_all_prcid())
      }

      return aPrcid  # [Array]テストシナリオ内に存在する全てのプロセッサ番号
    end

    #=================================================================
    # 概  要: テストシナリオ内に存在するスピンロック数を返す
    #=================================================================
    def get_spinlock_count()
      aSpinID = get_spinlock_names()

      return aSpinID.size()  # [Integer]テストシナリオ内に存在するスピンロック数
    end

    #=================================================================
    # 概  要: テストシナリオ内に存在するスピンロックIDを返す
    #=================================================================
    def get_spinlock_names()
      hSpinlock = @cPreCondition.get_objects_by_type(TSR_OBJ_SPINLOCK)

      return hSpinlock.keys()  # [Array]テストシナリオ内に存在するスピンロックID
    end

    #=================================================================
    # 概  要: 割込みハンドラで割込み番号ごとにチェックする属性を返す
    #=================================================================
    def get_inthdr_attributes_by_intno()
      return @cPreCondition.get_inthdr_attributes_by_intno()  # [Hash]割込み番号ごとの該当属性
    end

    #=================================================================
    # 概  要: 割込みサービスルーチンで割込み番号ごとにチェックする属性
    #       : を返す
    #=================================================================
    def get_isr_attributes_by_intno()
      return @cPreCondition.get_isr_attributes_by_intno()  # [Hash]割込み番号ごとの該当属性
    end

    #=================================================================
    # 概  要: 割込みサービスルーチンで割込み番号と割込み優先度の組み合
    #       : わせごとにチェックする属性を返す
    #=================================================================
    def get_isr_attributes_by_intno_and_isrpri()
      return @cPreCondition.get_isr_attributes_by_intno_and_isrpri()  # [Hash]割込み番号と割込み優先度の組み合わせごとの該当属性
    end

    #=================================================================
    # 概  要: CPU例外ハンドラでCPU例外ハンドラ番号ごとにチェックする属
    #       : 性を返す
    #=================================================================
    def get_exception_attributes_by_excno()
      return @cPreCondition.get_exception_attributes_by_excno()  # [Hash]CPU例外ハンドラ番号ごとの該当属性
    end

    #=================================================================
    # 概  要: 割込みハンドラが存在するかを返す
    #=================================================================
    def exist_interrupt_handler?()
      hIntHdr = @cPreCondition.get_objects_by_type([TSR_OBJ_INTHDR, TSR_OBJ_ISR])

      return !(hIntHdr.empty?())  # [Bool]割込みハンドラが存在するか
    end
    private :exist_interrupt_handler?

    #=================================================================
    # 概  要: CPU例外が存在するかを返す
    #=================================================================
    def exist_cpu_exception?()
      hCpuExc = @cPreCondition.get_objects_by_type(TSR_OBJ_EXCEPTION)

      return !(hCpuExc.empty?())  # [Bool]CPU例外が存在するか
    end
    private :exist_cpu_exception?

    #=================================================================
    # 概  要: 過渡状態のタスクがpre_conditionに存在するかを返す
    #=================================================================
    def exist_rus_task?()
      bResult = false
      @cPreCondition.hAllObject.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.sObjectType == TSR_OBJ_TASK && cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RUS)
          bResult = true
          break
        end
      }

      return bResult  # [Bool]過渡状態のタスクがpre_conditionに存在するか
    end
    private :exist_rus_task?

    #=================================================================
    # 概  要: コンディション間で実行状態のままの非タスクがあるかを返す
    #=================================================================
    def exist_keep_activate_non_context?(cPrevCondition, cCondition)
      check_class(Condition, cPrevCondition)  # 直前のコンディション
      check_class(Condition, cCondition)      # 基準となるコンディション

      bResult = false
      # 直前のコンディションの実行状態の非タスク取得
      hPrevActivate = {}
      hObjects = cPrevCondition.get_objects_by_type(GRP_NON_CONTEXT)
      hObjects.each{|sObjectID, cObjectInfo|
        if (cObjectInfo.is_activate?())
          hPrevActivate[sObjectID] = cObjectInfo
        end
      }
      # 基準となるコンディションで非タスクが動作し続けているか
      hObjects = cCondition.get_objects_by_type(GRP_NON_CONTEXT)
      hObjects.each{|sObjectID, cObjectInfo|
        # bootcntが変化していない
        if (hPrevActivate.has_key?(sObjectID) && cObjectInfo.is_activate?() &&
            hPrevActivate[sObjectID].hState[TSR_PRM_BOOTCNT] == cObjectInfo.hState[TSR_PRM_BOOTCNT])
          bResult = true
          break
        end
      }

      return bResult  # [Bool]コンディション間で実行状態のままの非タスクがあるか
    end

    #=================================================================
    # 概  要: 時間制御モードの判別結果を返す
    #=================================================================
    def is_gain_time_mode?()
      # all_gain_modeがoff (+ 時間操作関数がある)
      unless (@cConf.is_all_gain_time_mode?())
        # gain_timeがtrue
        if (@hVariation[TSR_PRM_GAIN_TIME] == true)
          # 時間制御条件を満たす
          if (is_time_control_situation?())
            raise(TTCExcludeError.new("T6_011: " + ERR_VARIATION_MUST_STOP_TIME))  # (パターン2)
          # 時間制御条件を満たさない
          else
            return true  # [Bool]時間制御モード(パターン3)
          end
        # gain_timeがfalse，省略
        else
          return false  # [Bool]時間制御モード(パターン4)
        end
      # all_gain_modeがon
      else
        # 時間操作関数がない
        if (@cConf.get_func_time() == false)
          # gain_timeがtrue，省略
          if (@hVariation[TSR_PRM_GAIN_TIME] != false)
            # 時間制御条件を満たす
            if (is_time_control_situation?())
              ### T6_001: 時間操作関数が必要なのに使用不可
              raise(TTCExcludeError.new("T6_001: " + ERR_VARIATION_CANNOT_STOP_TIME))  # (パターン5)
            # 時間制御条件を満たさない
            else
              return true  # [Bool]時間制御モード(パターン6)
            end
          # gain_timeがfalse
          else
            ### T6_001: 時間操作関数が必要なのに使用不可
            raise(TTCExcludeError.new("T6_001: " + ERR_VARIATION_CANNOT_CONTROL_TIME))  # (パターン7)
          end
        # 時間操作関数がある
        else
          case @hVariation[TSR_PRM_GAIN_TIME]
          when true
            # 時間制御条件を満たす
            if (is_time_control_situation?())
              raise(TTCExcludeError.new("T6_011: " + ERR_VARIATION_MUST_STOP_TIME))  # (パターン8)
            # 時間制御条件を満たさない
            else
              return true  # [Bool]時間制御モード(パターン9)
            end
          when false
            return false  # [Bool]時間制御モード(パターン10)
          else
            # 時間制御条件を満たす
            if (is_time_control_situation?())
              return false  # [Bool]時間制御モード(パターン11)
            # 時間制御条件を満たさない
            else
              return true  # [Bool]時間制御モード(パターン12)
            end
          end
        end
      end

      # いずれの条件にも一致しない
      abort(ERR_MSG % [__FILE__, __LINE__])
    end
    private :is_gain_time_mode?

    #=================================================================
    # 概  要: 時間制御条件を満たすかを返す
    #=================================================================
    def is_time_control_situation?()
      bResult = false
      # タイムティックがある場合は条件を満たす
      if (@bHasTimeTick == true)
        bResult = true
      # タイムティックがない場合はコンディションごとに条件をチェック
      else
        aCondition = get_all_condition()
        aCondition.each{|cCondition|
          if (cCondition.is_time_control_situation?(@hVariation))
            bResult = true
            break
          end
        }
      end

      return bResult  # [Bool]時間制御条件を満たすか
    end
    private :is_time_control_situation?

    #=================================================================
    # 概  要: 全コンディションの配列を返す
    #=================================================================
    def get_all_condition()
      aCondition = [@cPreCondition]
      aCondition.concat(@aDo_PostCondition)

      return aCondition  # [Array]全コンディション
    end
    private :get_all_condition

    #=================================================================
    # 概  要: グローバル対応のためパラメータの変換を実行する
    #=================================================================
    def convert_global()
      # タイムイベントハンドラのプロセッサIDが1種類かつ，
      # グローバル置換対象属性が定義されたマクロで指定されているならばグローバル置換
      aTimeEvtMacro = get_time_event_prcid()
      if (aTimeEvtMacro.size() == 1 && is_global_attribute_all_macro?())
        # 2コアか3コアか判別
        aMacro = get_all_prcid()
        if (aMacro.include?(CFG_MCR_PRC_OTHER_1) || aMacro.include?(CFG_MCR_PRC_OTHER_2))
          hClassTable = GRP_CONVERT_TABLE_CLASS_MULTI_OTHER
        else
          hClassTable = GRP_CONVERT_TABLE_CLASS_SINGLE_OTHER
        end

        hTable = make_table(aTimeEvtMacro[0])
        # 全コンディションで適用
        aCondition = get_all_condition()
        aCondition.each{|cCondition|
          cCondition.convert_global(hTable, hClassTable)
        }
      end
    end

    #=================================================================
    # 概  要: タイムイベントハンドラのプロセッサIDを返す
    #=================================================================
    def get_time_event_prcid()
      aMacro = []
      aCondition = get_all_condition()
      aCondition.each{|cCondition|
        hObjects = cCondition.get_objects_by_type(GRP_TIME_EVENT_HDL)
        hObjects.each_value{|cObjectInfo|
          # プロセッサIDを記録
          aMacro.push(cObjectInfo.hState[TSR_PRM_PRCID])
        }
      }

      return aMacro.uniq()  # [Array]タイムイベントハンドラのプロセッサID
    end

    #=================================================================
    # 概　要: 与えられたプロセッサIDの情報から変換テーブル作成し返す
    #=================================================================
    def make_table(sTimePrcid)
      check_class(String, sTimePrcid)  # タイマ管理プロセッサIDとなるマクロ

      # 元となる変換テーブル（キー: 変換前，値: 変換後）
      hTable = {
        TTC_GLOBAL_KEY_SELF    => TTC_GLOBAL_KEY_SELF,
        TTC_GLOBAL_KEY_OTHER   => TTC_GLOBAL_KEY_OTHER,
        TTC_GLOBAL_KEY_OTHER_1 => TTC_GLOBAL_KEY_OTHER_1,
        TTC_GLOBAL_KEY_OTHER_2 => TTC_GLOBAL_KEY_OTHER_2
      }
      hTempHash = TTC_GLOBAL_TABLE[TTC_GLOBAK_KEY_PRCID][:local].invert()
      lPrevKey = hTempHash[sTimePrcid]
      # 入れ替え
      hTable[lPrevKey]            = TTC_GLOBAL_KEY_SELF
      hTable[TTC_GLOBAL_KEY_SELF] = lPrevKey

      return hTable  # [Hash]変換テーブル
    end
    private :make_table

    #=================================================================
    # 概　要: グローバル置換対象属性が全てマクロで定義されているか
    #=================================================================
    def is_global_attribute_all_macro?()
      aCondition = get_all_condition()
      return aCondition.all?(){|cCondition|
        cCondition.is_global_attribute_all_macro?()
      }  # [Bool]グローバル置換対象属性が全てマクロで定義されているか
    end

    #=================================================================
    # 概　要: 全オブジェクトにおいて指定されていない属性をセット
    #=================================================================
    def set_nil_attribute()
      aCondition = get_all_condition()
      aCondition.each{|cCondition|
        cCondition.set_nil_attribute()
      }
    end

=begin
    #=================================================================
    # 概　要: 割込み番号一覧を取得する
    #=================================================================
    def get_all_intno()
      return @cPreCondition.get_all_intno()  # [Array]割込み番号一覧
    end
=end
  end
end
