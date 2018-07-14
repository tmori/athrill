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
#  $Id: TestScenario.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/test_scenario/TestScenario.rb"
require "ttj/bin/class/TTJModule.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: TestScenario
  # 概    要: PreCondition，Do，PostConditionのデータを保持
  #===================================================================
  class TestScenario
    include CommonModule
    include TTJModule

    #=================================================================
    # 概　要: TestScenarioのTESRYコード情報を日本語化してテストIDと
    #         一緒に返す
    #=================================================================
    def japanize_tesry_info()
      sVariation = ""
      sTTJPre    = ""
      sTTJDoPost = ""

      # Variation処理
      if (hVariation.empty?() == false)
        sVariation = japanize_variation_info()
      end

      # pre_condition処理
      sTTJPre = cPreCondition.japanize_pre_info()

      # doとpost_conditionのシーケンス番号とタイムティックを取得
      # 2次元配列で行はシーケンス番号を、列はタイムティックを示す
      aSteps = []

      aDo_PostCondition.each{|cDo_PostCondition|
        if (aSteps[cDo_PostCondition.nSeqNum].nil?() == true)
          aSteps[cDo_PostCondition.nSeqNum] = []  # シーケンス番号の取得
        end

        aSteps[cDo_PostCondition.nSeqNum].push(cDo_PostCondition.nTimeTick)  # タイムティックの取得
      }

      # シーケンス番号とタイムティックを書くための処理
      # シーケンス番号とタイムティックによりdoとpost_conditionの情報を日本語化する
      bSeqNum   = false  # シーケンス番号が複数あるか単一あるかを示すフラグ
      bTickFlag = false  # タイムティック発行後からの情報には必ずタイムティックを書くことを示すフラグ
      bCondFlag = true   # conditionとdoを書くか書かないかを示すフラグ

      if (aSteps.size() > 1)
        bSeqNum = true  # シーケンス番号が複数ある場合はシーケンス番号が複数あることを示すフラグ
      end

      aSteps.each_index{|nSeq|
        if (aSteps[nSeq].size() > 1) || (bTickFlag == true)  # 該当するシーケンス番号に情報が複数の場合
          aSteps[nSeq].each{|nTick|  # doの処理
            bTimeTick = search_do_info(nSeq)  # doに複数の処理があるかチェック
            cDo_PostCondition = search_do_post_condition(nSeq, nTick)  # 該当するdoを取得
            sTTJDoPost += cDo_PostCondition.japanize_do_info(bSeqNum, (bTickFlag == true) ? bTickFlag : bTimeTick, bCondFlag)  # 日本語化
            bCondFlag = false  # 「処理」が何回も書かれないようにするため
          }

          bCondFlag = true

          aSteps[nSeq].each{|nTick|  # post_conditionの処理
            cDo_PostCondition = search_do_post_condition(nSeq, nTick)  # 該当するpost_conditionを取得
            sTTJDoPost += cDo_PostCondition.japanize_post_info(bSeqNum, true, bCondFlag)  # 日本語化
            bCondFlag = false  # 「後状態」が何回も書かれないようにするため
          }

          bCondFlag = true
          bTickFlag = true
        else  # 該当するシーケンス番号に情報が一つのみの場合
          cDo_PostCondition = search_do_post_condition(nSeq)
          sTTJDoPost += cDo_PostCondition.japanize_do_info(bSeqNum, false, bCondFlag)
          sTTJDoPost += cDo_PostCondition.japanize_post_info(bSeqNum, false, bCondFlag)
        end
      }

      # 日本語化したTESRYコードの情報をハッシュにまとめる
      hJapanizeInfo = {@sTestID => sVariation + sTTJPre + sTTJDoPost}

      return hJapanizeInfo  # [Hash]テストID(key)と日本語化情報(val)をハッシュにして返す
    end

    #=================================================================
    # 概　要: variationを日本語化する
    #=================================================================
    def japanize_variation_info()
      sIndent       = "#{TTJ_TAB}"
      sAllVariation = "#{GRP_TTJ_CONDITION[TSR_LBL_VARIATION]}#{TTJ_NEW_LINE}"

      # Blankのサイズ計算
      blank_size = 1

      @hVariation.each{|key, val|
        if (GRP_TTJ_VARIATION[key].nil?() == true)
          $sAttrErr += "[variation : #{key}] #{@sTestID}#{TTJ_NEW_LINE}"  # 指定されていない属性のエラー出力
        elsif (GRP_TTJ_STATUS[val].nil?() == true)
          $sAttrErr += "[variation/#{key} : #{val}] #{@sTestID}#{TTJ_NEW_LINE}"  # 指定されていない属性のエラー出力
        elsif (blank_size < GRP_TTJ_VARIATION[key].size())
          blank_size = GRP_TTJ_VARIATION[key].size()  # 一番長い属性のサイズを取得する
        end
      }

      # Variationの日本語化
      @hVariation.each{|key, val|
        blank = ""

        if (GRP_TTJ_VARIATION[key].nil?() == true)
          next
        else
          (blank_size - GRP_TTJ_VARIATION[key].size()).times {
            blank += TTJ_BLINK_INDEX  # Blank取得
          }
        end

        # 属性の値がtrue，falseの場合
        if ((key == TSR_PRM_GAIN_TIME)       || (key == TSR_PRM_ENA_EXC_LOCK)    || (key == TSR_PRM_GCOV_ALL) ||
            (key == TSR_PRM_SUPPORT_GET_UTM) || (key == TSR_PRM_SUPPORT_ENA_INT) || (key == TSR_PRM_SUPPORT_DIS_INT))
          sAllVariation += "#{sIndent}#{GRP_TTJ_VARIATION[key]}#{blank}#{TTJ_PARTITION}#{GRP_TTJ_STATUS[val][key]}#{TTJ_NEW_LINE}"
        else
          sAllVariation += "#{sIndent}#{GRP_TTJ_VARIATION[key]}#{blank}#{TTJ_PARTITION}#{GRP_TTJ_STATUS[val]}#{TTJ_NEW_LINE}"
        end
      }

      sAllVariation += "#{TTJ_NEW_LINE}"

      return sAllVariation  # [String]バリエーションを日本語化した文字列
    end

    #=================================================================
    # 概　要: nSeqNumとnTimeTickに該当するcDo_PostConditionを
    #         取得して消す
    #=================================================================
    def search_do_post_condition(nSeq, nTick = nil)
      check_class(Integer, nSeq)         # シーケンス番号
      check_class(Integer, nTick, true)  # タイムティック

      aDo_PostCondition.each{|cDo_PostCondition|
        if ((nTick.nil?() == true) && (cDo_PostCondition.nSeqNum == nSeq))
          return cDo_PostCondition  # [cDo_PostCondition]DoとPostCondition情報のクラス
        elsif ((cDo_PostCondition.nSeqNum == nSeq) && (cDo_PostCondition.nTimeTick == nTick))
          return cDo_PostCondition  # [cDo_PostCondition]DoとPostCondition情報のクラス
        end
      }
    end

    #=================================================================
    # 概　要: 該当するnSeqNumにdoの情報が複数あるかどうか検査して返す
    #=================================================================
    def search_do_info(nSeq)
      check_class(Integer, nSeq)  # シーケンス番号

      # doに複数の処理があるかチェックする
      # 複数の場合はtrueを、単一の場合はfalseを返す
      aDoInfo = []

      aDo_PostCondition.each{|cDo_PostCondition|
        if ((cDo_PostCondition.nSeqNum == nSeq) && (cDo_PostCondition.hDo.empty?() == false))
          aDoInfo.push(cDo_PostCondition.nTimeTick)
        end
      }

      bDoFlag = (aDoInfo.size() > 1) ? true : false

      return bDoFlag  # [Bool]Doの情報が複数の場合はtrueを，複数ではない場合はfalseを返す
    end

  end
end
