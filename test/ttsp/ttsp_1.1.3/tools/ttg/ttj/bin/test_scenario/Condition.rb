#!ruby -Ke
#
#  TTG
#      TOPPERS Test Generator
#
#  Copyright (C) 2009-2012 by Center for Embedded Computing Systems
#              Graduate School of Information Science, Nagoya Univ., JAPAN
#  Copyright (C) 2010-2011 by Digital Craft Inc.
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
#  $Id: Condition.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "ttj/bin/class/TTJModule.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: Condition
  # 概    要: pre_condition, post_conditionの情報を処理するクラス
  #===================================================================
  class Condition
    include TTJModule

    #=================================================================
    # 概　要: pre_conditionまたはpost_conditionの各オブジェクトの情報を
    #         日本語化した後，全てのオブジェクトの情報をつなげて返す
    #=================================================================
    def japanize_condition_info(sAllCond, sIndent, sCondition)
      check_class(String, sAllCond)    # pre_condition，post_conditionのラベル
      check_class(String, sIndent)     # pre_condition，post_conditionの基本インデント
      check_class(String, sCondition)  # pre_condition，post_conditionの情報

      @hExistState  = {}       # nil属性を除外したState
      @sBlankIndent = sIndent  # 属性の長さを計算する際に使うインデント
      bChangeFlag   = false    # 状態に変化のないconditionを示す

      # オブジェクトID順で出力するために全オブジェクトのタイプをソートして保持
      aAllObject = hAllObject.keys()
      aAllObject = aAllObject.sort()

      # オブジェクト順，さらにオブジェクトID順で日本語化処理を実施する
      GRP_TTJ_OBJECT.each{|hObject|
        aAllObject.each{|sObjectID|
          if (hObject.keys[0] == hAllObject[sObjectID].sObjectType)
            sObjectType = hAllObject[sObjectID].sObjectType  # オブジェクトのタイプ
            sObjectID   = hAllObject[sObjectID].sObjectID    # テストID

            # 値がnilでない属性を取出す
            @hExistState = hAllObject[sObjectID].hState.reject{|key, val|
              hAllObject[sObjectID].aNilAttributes.include?(key)
            }

            # 変化のないpost_conditionは日本語化処理から除外する
            next if ((sCondition == TSR_LBL_POST) && (@hExistState.empty?() == true))

            # 日本語化処理開始
            bChangeFlag = true
            sAllCond += japanize_condition(sObjectType, sObjectID)
          end
        }
      }

      # 状態に変化のないconditionの場合
      if (bChangeFlag == false)
        sAllCond += "#{sIndent}#{TTJ_POST_EMPTY}#{TTJ_NEW_LINE}#{TTJ_NEW_LINE}"
      end

      # 指定されていないオブジェクトのエラー出力
      aObjectErr= []

      GRP_TTJ_OBJECT.each{|hTTJObject|
        aObjectErr.push(hTTJObject.keys[0])  # オブジェクトのタイプを取得する
      }

      aAllObject.each{|sObjectID|
        # TESTYコードとTTJオブジェクトマクロを比較して合わない場合はエラーメッセージに反映する
        if (aObjectErr.include?(hAllObject[sObjectID].sObjectType) == false)
          $sAttrErr += "[#{sCondition}/#{sObjectID.gsub("#{@sTestID}_", "")} : #{hAllObject[sObjectID].sObjectType}] #{@sTestID}#{TTJ_NEW_LINE}"  # 指定されていないオブジェクトのエラー出力
        end
      }

      return sAllCond  # [String]pre_conditionまたはpost_conditionを日本語化した文字列
    end

    #=================================================================
    # 概　要: 各オブジェクトの情報を日本語化して返す
    #=================================================================
    def japanize_condition(sObjectType, sObjectID)
      check_class(String, sObjectType)  # オブジェクトのタイプ
      check_class(String, sObjectID)    # オブジェクトのID

      # IDとtype追加
      @hExistState.store(TTJ_STT_ID,   sObjectID)
      @hExistState.store(TSR_PRM_TYPE, sObjectType)

      # Blank処理
      @nBlankSize = 1
      @hExistState.each_key{|key|
        if (key == TSR_PRM_STATE)
          blank_size(GRP_TTJ_STATUS[key][sObjectType])
        else
          blank_size($hTTJAttribute[key])
        end
      }

      sReturnState = ""

      # 属性順にして日本語化処理を開始する
      GRP_TTJ_ATTRIBUTE.each{|hAttribute|
        @hExistState.each{|key, val|
          if (hAttribute.keys[0] == key)
            case key
            when TTJ_STT_ID
              val = val.gsub("#{@sTestID}_", "")     # Alias削除
              sReturnState += "#{blank(hAttribute[key])}#{val}"  # オブジェクトのID挿入

            when TSR_PRM_TYPE
              GRP_TTJ_OBJECT.each{|hObject|  # dictionaryのオブジェクト順で変換
                if (hObject.keys[0] == val)
                  sReturnState += "#{blank(hAttribute[key])}#{hObject.values[0]}"  # オブジェクトのID挿入
                end
              }

            # 各オブジェクトのstate
            when TSR_PRM_STATE
              sReturnState += "#{blank(GRP_TTJ_STATUS[key][sObjectType])}#{GRP_TTJ_STATUS[val]}"

            # 各オブジェクトの属性
            when TSR_PRM_ATR
              sAtr = ""

              if (val.include?(KER_TA_NULL))
                sAtr += "#{GRP_TTJ_STATUS[TSR_PRM_ATR][KER_TA_NULL]}"
              else
                if (val.include?(KER_TA_TPRI))
                  sAtr += "#{GRP_TTJ_STATUS[TSR_PRM_ATR][KER_TA_TPRI]}#{TTJ_PAUSE}"
                end

                if (val.include?(KER_TA_WMUL))
                  sAtr += "#{GRP_TTJ_STATUS[TSR_PRM_ATR][KER_TA_WMUL]}#{TTJ_PAUSE}"
                end

                if (val.include?(KER_TA_CLR))
                  sAtr += "#{GRP_TTJ_STATUS[TSR_PRM_ATR][KER_TA_CLR]}#{TTJ_PAUSE}"
                end

                if (val.include?(KER_TA_MPRI))
                  sAtr += "#{GRP_TTJ_STATUS[TSR_PRM_ATR][KER_TA_MPRI]}#{TTJ_PAUSE}"
                end

                if (val.include?(KER_TA_STA))
                  sAtr += "#{GRP_TTJ_STATUS[TSR_PRM_ATR][KER_TA_STA]}#{TTJ_PAUSE}"
                end

                if (val.include?(KER_TA_ENAINT))
                  sAtr += "#{GRP_TTJ_STATUS[TSR_PRM_ATR][KER_TA_ENAINT]}#{TTJ_PAUSE}"
                end

                if (val.include?(KER_TA_DISINT))
                  sAtr += "#{GRP_TTJ_STATUS[TSR_PRM_ATR][KER_TA_DISINT]}#{TTJ_PAUSE}"
                end
              end

              sAtr = sAtr.gsub(/, \z/, "")
              sReturnState += "#{blank(hAttribute[key])}#{sAtr}"

            # 待ち要因
            when TSR_PRM_WOBJID
              if (GRP_TTJ_STATUS[val].nil?() == true)
                val = val.gsub("#{@sTestID}_", "")  # Alias削除
                sReturnState += "#{blank(hAttribute[key])}#{val}"
              else
                sReturnState += "#{blank(hAttribute[key])}#{GRP_TTJ_STATUS[val]}"
              end

            # cpuロック，ディスパッチ
            when TSR_PRM_LOCCPU, TSR_PRM_DISDSP
              sReturnState += "#{blank(hAttribute[key])}#{GRP_TTJ_STATUS[val][key]}"

            # 割込み優先度マスク
            when TSR_PRM_CHGIPM
              if (GRP_TTJ_STATUS[key][val].nil?() == true)
                sReturnState += "#{blank(hAttribute[key])}#{val}"
              else
                sReturnState += "#{blank(hAttribute[key])}#{GRP_TTJ_STATUS[key][val]}"
              end

            # 送信待ちタスクリスト
            when TSR_PRM_STSKLIST
              if (val.empty?() == false)
                sSendTaskList = ""

                val.each{|hVal|
                  sSendTaskList += "#{TTJ_BLOCK_OPEN}#{hVal.keys[0]}#{TTJ_PAUSE}#{hVal[hVal.keys[0]][TSR_VAR_DATA]}#{TTJ_BLOCK_CLOSE}#{TTJ_PAUSE}"
                }

                sSendTaskList = sSendTaskList.gsub(/, \z/, "")
                sSendTaskList = sSendTaskList.gsub("#{@sTestID}_", "")  # Alias削除
                sReturnState += "#{blank(hAttribute[key])}#{sSendTaskList}"
              else
                sReturnState += "#{blank(hAttribute[key])}#{TTJ_ATTR_EMPTY}"
              end

            # 受信待ちタスクリスト
            when TSR_PRM_RTSKLIST
              if (val.empty?() == false)
                sRcvTaskList = ""

                val.each{|hVal|
                  sRcvTaskList += "#{TTJ_BLOCK_OPEN}#{hVal.keys[0]}"

                  if ((hVal[hVal.keys[0]].nil?() == false) && (sObjectType == TSR_OBJ_DATAQUEUE))  # データキューの場合
                    sRcvTaskList += "#{TTJ_PAUSE}#{hVal[hVal.keys[0]][TSR_VAR_VAR]}"
                  elsif ((hVal[hVal.keys[0]].nil?() == false) && (sObjectType == TSR_OBJ_P_DATAQUEUE))  # 優先度データキューの場合
                    sRcvTaskList += "#{TTJ_PAUSE}#{hVal[hVal.keys[0]][TSR_VAR_VARPRI]}#{TTJ_PAUSE}#{hVal[hVal.keys[0]][TSR_VAR_VARDATA]}"
                  end

                  sRcvTaskList += "#{TTJ_BLOCK_CLOSE}#{TTJ_PAUSE}"
                }

                sRcvTaskList = sRcvTaskList.gsub(/, \z/, "")
                sRcvTaskList = sRcvTaskList.gsub("#{@sTestID}_", "")  # Alias削除
                sReturnState += "#{blank(hAttribute[key])}#{sRcvTaskList}"
              else
                sReturnState += "#{blank(hAttribute[key])}#{TTJ_ATTR_EMPTY}"
              end

            # データ管理領域のデータリスト
            when TSR_PRM_DATALIST
              if (val.empty?() == false)
                sDataList = "#{TTJ_BLOCK_OPEN}"

                val.each{|hVal|
                  sDataList += "#{hVal[TSR_VAR_DATA]}"#{TTJ_PAUSE}"
                }

                sDataList += "#{TTJ_BLOCK_CLOSE}"
                sDataList = sDataList.gsub(/, \}/, TTJ_BLOCK_CLOSE)
                sReturnState += "#{blank(hAttribute[key])}#{sDataList}"
              else
                sReturnState += "#{blank(hAttribute[key])}#{TTJ_ATTR_EMPTY}"
              end

            # 待ちタスクリスト
            when TSR_PRM_WTSKLIST
              if (val.empty?() == false)
                sWaitTaskList = ""

                val.each{|hVal|
                  sWaitTaskList += "#{TTJ_BLOCK_OPEN}"

                  if (sObjectType == TSR_OBJ_SEMAPHORE)  # セマフォの場合
                    sWaitTaskList += "#{hVal.keys[0]}"
                  elsif (sObjectType == TSR_OBJ_EVENTFLAG)  # イベントフラグの場合
                    hVal.each{|wtlKey, wtlVal|
                      sWaitTaskList += "#{wtlKey}"
                      sWaitTaskList += "#{TTJ_PAUSE}#{wtlVal[TSR_VAR_WAIPTN]}"
                      sWaitTaskList += "#{TTJ_PAUSE}#{wtlVal[TSR_VAR_WFMODE]}"

                      if (wtlVal[TSR_VAR_VAR].nil?() == false)
                        sWaitTaskList += "#{TTJ_PAUSE}#{wtlVal[TSR_VAR_VAR]}"
                      end
                    }
                  elsif (sObjectType == TSR_OBJ_MAILBOX) ||  # メールボックスの場合
                        (sObjectType == TSR_OBJ_MEMORYPOOL)  #固定長メモリプールの場合
                    hVal.each{|wtlKey, wtlVal|
                      sWaitTaskList += "#{wtlKey}"

                      if (wtlVal.nil?() == false)
                        sWaitTaskList += "#{TTJ_PAUSE}#{wtlVal[TSR_VAR_VAR]}"
                      end
                    }
                  end

                  sWaitTaskList += "#{TTJ_BLOCK_CLOSE}#{TTJ_PAUSE}"
                }

                sWaitTaskList = sWaitTaskList.gsub(/, \z/, "")
                sWaitTaskList = sWaitTaskList.gsub("#{@sTestID}_", "")  # Alias削除
                sReturnState += "#{blank(hAttribute[key])}#{sWaitTaskList}"
              else
                sReturnState += "#{blank(hAttribute[key])}#{TTJ_ATTR_EMPTY}"
              end

            # メッセージリスト
            when TSR_PRM_MSGLIST
              if (val.empty?() == false)
                sMsgList = ""

                val.each{|hVal|
                  sMsgList += "#{TTJ_BLOCK_OPEN}#{hVal[TSR_VAR_MSG]}"

                  if (hVal[TSR_VAR_MSGPRI].nil?() == false)
                    sMsgList += "#{TTJ_PAUSE}#{hVal[TSR_VAR_MSGPRI]}"
                  end

                  sMsgList += "#{TTJ_BLOCK_CLOSE}#{TTJ_PAUSE}"
                }

                sMsgList = sMsgList.gsub(/, \z/, "")
                sMsgList = sMsgList.gsub("#{@sTestID}_", "")  # Alias削除
                sReturnState += "#{blank(hAttribute[key])}#{sMsgList}"
              else
                sReturnState += "#{blank(hAttribute[key])}#{TTJ_ATTR_EMPTY}"
              end

            # 変数
            when TSR_PRM_VAR
              sVar = ""

              # 変数の改行によるインデント計算
              sVarBlank     = blank(hAttribute[key])
              nVarTabCount  = sVarBlank.count("\t")
              sVarBlank     = sVarBlank.gsub(TTJ_TAB, "")
              nVarBlankSize = sVarBlank.size()
              sVarBlank     = "#{(TTJ_TAB * nVarTabCount)}#{(TTJ_BLINK_INDEX * nVarBlankSize)}"

              val.each_value{|cVarInfo|
                if (cVarInfo.hMember.nil?() == false)  # 構造体の場合(ref系)
                  sVar += "#{cVarInfo.sVarName}#{TTJ_PAUSE}#{GRP_TTJ_STATUS[cVarInfo.sType]}#{TTJ_NEW_LINE}"  # 変数名と変数のタイプ

                  # 構造体の属性のインデント設定
                  nBlankSizeTemp = 1

                  cVarInfo.hMember.each_key{|blankSizeKey|
                    if (nBlankSizeTemp < blankSizeKey.size())
                      nBlankSizeTemp = blankSizeKey.size()
                    end
                  }

                  # 構造体の日本語化
                  GRP_TTJ_STRUCT_SEQUENCE.each{|sStruct|
                    cVarInfo.hMember.each{|memKey, memVal|
                      if (sStruct == memKey)
                        sVar += "#{sVarBlank}#{TTJ_BLOCK_OPEN}#{memKey}#{(TTJ_BLINK_INDEX * (nBlankSizeTemp - memKey.size()))}#{TTJ_PARTITION}#{memVal}#{TTJ_BLOCK_CLOSE}#{TTJ_NEW_LINE}"  # 構造体の値
                      end

                      if (GRP_TTJ_STRUCT_SEQUENCE.include?(memKey) == false)
                        $sAttrErr += "Struct [#{sObjectType} : #{memKey}] #{@sTestID}#{TTJ_NEW_LINE}"  # 指定されていない属性のエラー出力
                      end
                    }
                  }

                  sVar[sVar.rindex(TTJ_NEW_LINE)] = ""  # 最後の改行を削除
                  sVar = sVar.gsub(/, \}/, TTJ_BLOCK_CLOSE)
                else
                  # valが複数の場合は括弧で囲んで区別する
                  if (val.size() > 1)
                    sVar += "#{TTJ_BLOCK_OPEN}"
                  end

                  sVar += "#{cVarInfo.sVarName}#{TTJ_PAUSE}#{GRP_TTJ_STATUS[cVarInfo.sType]}"  # 変数名と変数のタイプ

                  if (cVarInfo.snValue.nil?() == false)
                    sVar += "#{TTJ_PAUSE}#{cVarInfo.snValue}"  # 変数の値
                  end

                  # valが複数の場合は括弧で囲んで区別する
                  if (val.size() > 1)
                    sVar += "#{TTJ_BLOCK_CLOSE}"
                  end

                  sVar += "#{TTJ_NEW_LINE}#{sVarBlank}"
                end
              }

              sVar = sVar.gsub(/, \z/, "")
              sVar = sVar.gsub(/#{TTJ_NEW_LINE}#{sVarBlank}\z/, "")
              sVar = sVar.gsub("#{@sTestID}_", "")  # Alias削除
              sReturnState += "#{blank(hAttribute[key])}#{sVar}"

            # sns_kerのdo処理
            when TSR_PRM_DO
              sVar = ""

              sDoIndent = "#{@sBlankIndent}#{(TTJ_BLINK_INDEX * (blank(hAttribute[key]).size() - 1))}"
              sVar = japanize_do(blank(hAttribute[key]), sDoIndent, val)

              sReturnState += "#{sVar}"

            # sns_kerのglobal処理
            when TSR_PRM_GLOBAL
              sReturnState += "#{blank(hAttribute[key])}#{GRP_TTJ_STATUS[val][key][@hExistState[TSR_PRM_TYPE]]}"

            # その以外の属性の日本語変換(別途の処理が要らない属性)
            else
              # val代入 : prcid, wupcnt, tskpri, actprc, actcnt, bootcnt, pndptn, itskpri, cyctim,
              #           cycphs, exinf, texptn, procid, class, spinid, excno, intno, inhno, intpri
              #           maxsem, isemcnt, semcnt, maxmpri, flgptn, iflgptn, dtqcnt, maxdpri, pdqcnt,
              #           fblkcnt, blkcnt, lefttim, porder, lefttmo, blksz
              # GRP_TTJ_STATUS[val]代入：task
              if (GRP_TTJ_STATUS[val].nil?())
                val = val.to_s.gsub("#{@sTestID}_", "")  # Alias削除
                sReturnState += "#{blank(hAttribute[key])}#{val}"
              elsif ((val == TSR_STT_TRUE) || (val == TSR_STT_FALSE))
                $sAttrErr += "[#{sObjectType} : #{key}属性の#{val}] #{@sTestID}#{TTJ_NEW_LINE}"  # 指定されていない属性のエラー出力
              else
                sReturnState += "#{blank(hAttribute[key])}#{GRP_TTJ_STATUS[val]}"
              end
            end

            sReturnState += "#{TTJ_NEW_LINE}"
          end
        }
      }

      # 指定されていない属性のエラー出力
      aAttrErr= []

      GRP_TTJ_ATTRIBUTE.each{|hAttribute|
        aAttrErr.push(hAttribute.keys[0])
      }

      @hExistState.each{|key, val|
        if (aAttrErr.include?(key) == false)
          $sAttrErr += "[#{sObjectType} : #{key}] #{@sTestID}#{TTJ_NEW_LINE}"  # 指定されていない属性のエラー出力
        end
      }

      # 各オブジェクトの改行
      sReturnState += "#{TTJ_NEW_LINE}"

      return sReturnState  # [String]各オブジェクトを日本語化した文字列
    end

    #=================================================================
    # 概　要: doの情報を日本語化して返す
    #=================================================================
    def japanize_do(sAllDo, sIndent, hVal = nil)
      check_class(String, sAllDo)    # doの前処理を保持している
      check_class(String, sIndent)   # doの基本インデント
      check_class(Hash, hVal, true)  # 上位doから呼ばれずsns_kerから呼ばれた場合

      # doの処理日本語化処理開始
      if (hVal.nil?() == true)
        hDoInfo = @hDo
        sAllDo += "#{sIndent}#{hDoInfo[TSR_PRM_ID]}が"
      else
        hDoInfo = hVal  # sns_kerのオブジェクト内のdoの処理
      end

      if (hDoInfo[TSR_PRM_CODE].nil?() == false)  # code
        sAllDo += "#{hDoInfo[TSR_PRM_CODE]}を実行する．"
      elsif (hDoInfo[TSR_PRM_ERUINT].nil?() == false)  # eruint
        if (hDoInfo[TSR_PRM_ERUINT].is_a?(Integer) == false)  # eruintがエラーコードの場合
          sAllDo += "#{hDoInfo[TSR_PRM_SYSCALL]}を発行し，#{TTJ_NEW_LINE}"
          sAllDo += "#{sIndent}エラーコードとして#{hDoInfo[TSR_PRM_ERUINT]}が返る．"
        else
          sAllDo += "#{hDoInfo[TSR_PRM_SYSCALL]}を発行し，#{TTJ_NEW_LINE}"
          sAllDo += "#{sIndent}前キューイング数として#{hDoInfo[TSR_PRM_ERUINT]}が返る．"
        end
      elsif (hDoInfo[TSR_PRM_BOOL].nil?() == false)  # bool
        sAllDo += "#{hDoInfo[TSR_PRM_SYSCALL]}を発行し，#{TTJ_NEW_LINE}"
        sAllDo += "#{sIndent}参照結果として#{hDoInfo[TSR_PRM_BOOL]}が返る．"
      elsif (hDoInfo[TSR_PRM_ERCD].nil?() == false)  # ercd
        sAllDo += "#{hDoInfo[TSR_PRM_SYSCALL]}を発行し，#{TTJ_NEW_LINE}"
        sAllDo += "#{sIndent}エラーコードとして#{hDoInfo[TSR_PRM_ERCD]}が返る．"
      else
        sAllDo += "#{hDoInfo[TSR_PRM_SYSCALL]}を発行する．"
      end

      if (hDoInfo[TSR_PRM_GCOV].nil?() == false)  # gcov
        sAllDo += "#{TTJ_NEW_LINE}#{sIndent}#{TSR_PRM_GCOV}を#{GRP_TTJ_STATUS[hDoInfo[TSR_PRM_GCOV]][TSR_PRM_GCOV]}"
      end

      sAllDo = sAllDo.gsub("#{@sTestID}_", "")  # Alias削除

      return sAllDo  # [String]doを日本語化した文字列
    end

    #=================================================================
    # 概　要: Blankのサイズを計算する
    #=================================================================
    def blank_size(sAttr)
      check_class(String, sAttr)  # オブジェクトの該当属性

      if (@nBlankSize < sAttr.size())
        @nBlankSize = sAttr.size()
      end
    end

    #=================================================================
    # 概　要: 属性の長さを合わせるためにインデントを入れて返す
    #=================================================================
    def blank(sAttr)
      check_class(String, sAttr)  # オブジェクトの該当属性

      sRetAtt = "#{@sBlankIndent}#{sAttr}"

      (@nBlankSize - sAttr.size()).times{
        sRetAtt += "#{TTJ_BLINK_INDEX}"
      }

      sRetAtt += "#{TTJ_PARTITION}"

      return sRetAtt  # [String]属性の長さを合わせた文字列
    end

  end
end
