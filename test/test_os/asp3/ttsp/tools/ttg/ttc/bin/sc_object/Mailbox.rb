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
#  $Id: Mailbox.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "ttc/bin/sc_object/SCObject.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: Mailbox
  # 概    要: メールボックスの情報を処理するクラス
  #===================================================================
  class Mailbox < SCObject
    #=================================================================
    # 概  要: 属性チェック
    #=================================================================
    def attribute_check()
      aErrors = []
      begin
        super()
      rescue TTCMultiError
        aErrors = $!.aErrors
      end

      begin
        # wtsklist
        sAtr = TSR_PRM_WTSKLIST
        if (is_specified?(sAtr))
          cProc = Proc.new(){|hData, aPath|
            if (hData.is_a?(Hash))
              aProcErrors = []
              hData.each{|atr, val|
                begin
                  if (atr == TSR_VAR_VAR)
                    check_attribute_variable(atr, val, aPath)
                  else
                    sErr = sprintf(ERR_UNDEFINED_KEY, atr)
                    raise(YamlError.new(sErr, aPath))
                  end
                rescue YamlError
                  aProcErrors.push($!)
                end
              }
              check_error(aProcErrors)
            elsif (!hData.nil?())
              sErr = sprintf(ERR_LIST_ITEM_INVALID_TYPE_NIL, sAtr, Hash, hData.class())
              raise(YamlError.new(sErr, aPath))
            end
          }
          attribute_check_task_list(sAtr, @hState[sAtr], cProc)
        end
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      begin
        # msglist
        sAtr = TSR_PRM_MSGLIST
        if (is_specified?(sAtr))
          check_attribute_type(sAtr, @hState[sAtr], Array, false, @aPath)
          aPath = @aPath + [sAtr]
          aTmpErrors = []
          @hState[sAtr].each_with_index{|hData, nIndex|
            # リストの要素がHashか
            unless (hData.is_a?(Hash))
              sErr = sprintf(ERR_LIST_INVALID_TYPE, sAtr, Hash, hData.class())
              raise(YamlError.new(sErr, aPath + [nIndex]))
            end
            # 要素の内容チェック
            hData.each{|atr, val|
              begin
                case atr
                when TSR_VAR_MSG
                  check_attribute_variable(atr, val, aPath + [nIndex])
                when TSR_VAR_MSGPRI
                  check_attribute_range(atr, val, TTC_MAX_PRI, TTC_MIN_PRI, aPath + [nIndex])
                else
                  sErr = sprintf(ERR_UNDEFINED_KEY, atr)
                  raise(YamlError.new(sErr, aPath + [nIndex]))
                end
              rescue YamlError
                aTmpErrors.push($!)
              end
            }
          }
          check_error(aTmpErrors)
        end
      rescue YamlError
        aErrors.push($!)
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概  要: オブジェクトチェック
    #=================================================================
    def object_check(bIsPre)
      check_class(Bool, bIsPre)  # pre_conditionか

      aErrors = []
      begin
        super(bIsPre)
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      ### T3_MBX001: メールボックスの属性がTA_MPRIの時に受信待ちメッセージリストのメッセージに
      ###          : メッセージヘッダ変数名と優先度の属性が記述されていない
      unless (@hState[TSR_PRM_MSGLIST].nil?())
        if (@hState[TSR_PRM_ATR] == KER_TA_MPRI)
          @hState[TSR_PRM_MSGLIST].each_with_index{|hData, index|
            if (hData[TSR_VAR_MSG].nil?() || hData[TSR_VAR_MSGPRI].nil?())
              aErrors.push(YamlError.new("T3_MBX001: " + ERR_MSGLIST_VARS_MUST_DEFINE, @aPath + [index]))
            end
          }
        ### T3_MBX002: メールボックスの属性がTA_MPRI以外の時に受信待ちメッセージリストのメッセージに
        ###          : 優先度の属性が記述されている
        else
          @hState[TSR_PRM_MSGLIST].each_with_index{|hData, index|
            if (hData.has_key?(TSR_VAR_MSGPRI))
              aErrors.push(YamlError.new("T3_MBX002: " + ERR_CANNOT_BE_DEFINED_MSGPRI, @aPath + [index]))
            end
          }
        end
        ### T3_MBX003: msglistのリストに同一変数が二つ以上存在する
        aVarName = []
        @hState[TSR_PRM_MSGLIST].each{|hData|
          if (hData.has_key?(TSR_VAR_MSG))
            if (aVarName.include?(hData[TSR_VAR_MSG]))
              sErr = sprintf("T3_MBX003: " + ERR_MSGLIST_VAR_DUPLICATE, hData[TSR_VAR_MSG])
              aErrors.push(YamlError.new(sErr, @aPath))
            end
            aVarName.push(hData[TSR_VAR_MSG])
          end
        }
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概  要: 初期値を補完する
    #=================================================================
    def complement_init_object_info()
      super()

      # mbxatr
      unless (is_specified?(TSR_PRM_MBXATR))
        @hState[TSR_PRM_ATR] = "ANY_ATT_MBX"
      end
      # maxmpri
      unless (is_specified?(TSR_PRM_MAXMPRI))
        @hState[TSR_PRM_MAXMPRI] = "MSG_PRI_MAX"
      end
    end

    #=================================================================
    # 概  要: 待ちタスクリスト内の変数と型の組み合わせ一覧を返す
    #=================================================================
    def get_wtsklist_variable()
      hVars = {}
      unless (@hState[TSR_PRM_WTSKLIST].nil?())
        @hState[TSR_PRM_WTSKLIST].each{|hTask|
          hTask.each{|sTask, hData|
            unless (hData.nil?())
              hVars[sTask] = {}
              hData.each{|sAtr, sVarName|
                if (sAtr == TSR_VAR_VAR)
                  hVars[sTask][sVarName] = [TYP_T_P_MSG, TYP_T_P_MSG_PRI]
                end
              }
            end
          }
        }
      end

      return hVars  # [Hash]待ちタスクリスト内の変数と型の組み合わせ一覧
    end

    #=================================================================
    # 概  要: メッセージリスト内の変数と型の組み合わせ一覧を返す
    #=================================================================
    def get_msglist_variable()
      hVars = {}
      unless (@hState[TSR_PRM_MSGLIST].nil?())
        @hState[TSR_PRM_MSGLIST].each{|hData|
          unless (hData.nil?())
            hData.each{|sAtr, sVarName|
              if (sAtr == TSR_VAR_MSG)
                if (@hState[TSR_PRM_ATR] == KER_TA_MPRI)
                  hVars[sVarName] = [TYP_T_MSG_PRI]
                else
                  hVars[sVarName] = [TYP_T_MSG]
                end
              end
            }
          end
        }
      end

      return hVars  # [Hash]メッセージリスト内の変数と型の組み合わせ一覧
    end
  end
end
