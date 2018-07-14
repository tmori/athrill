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
    # 概　要: do文字とインデントとタイムティックを日本語化した後，
    #         日本語化されたdoの情報をつなげて返す
    #=================================================================
    def japanize_do_info(bSeqNum, bTimeTick, bCondFlag)
      check_class(Bool, bSeqNum)    # シーケンス番号の有無
      check_class(Bool, bTimeTick)  # タイムティックの有無

      sIndent = TTJ_TAB  # doの基本インデント
      sAllDo  = ""       # 日本語化されたdoの情報を保持するために変数

      # do文字の日本語化
      if (bCondFlag == true)
        sAllDo += "#{GRP_TTJ_CONDITION[TSR_LBL_DO]}#{(bSeqNum == true ? nSeqNum : "")}#{TTJ_NEW_LINE}"

        if (hDo.empty?() == true)
          return sAllDo += "#{sIndent}#{TTJ_DO_EMPTY}#{TTJ_NEW_LINE}#{TTJ_NEW_LINE}"  # [String]doの情報がないことを示す文字列
        end
      end

      # doに情報がない場合
      if (hDo.empty?() == true)
        return ""  # [String]doの情報がないことを示す文字列
      end

      # doにタイムティック情報がある場合タイムティックを処理する
      # タイムティック記入およびインデントの挿入
      if (bTimeTick == true)
        sAllDo  += "#{sIndent}#{nTimeTick}#{TTJ_NEW_LINE}"
        sIndent += "#{TTJ_TAB}"
      end

      # doの情報を日本語化する
      sAllDo = japanize_do(sAllDo, sIndent)
      sAllDo += "#{TTJ_NEW_LINE}#{TTJ_NEW_LINE}"

      return sAllDo  # [String]日本語化されたdoの情報
    end

    #=================================================================
    # 概　要: post_condition文字とインデントとタイムティックを
    #         日本語化した後，日本語化されたpost_conditionの情報を
    #         つなげて返す
    #=================================================================
    def japanize_post_info(bSeqNum, bTimeTick, bCondFlag)
      check_class(Bool, bSeqNum)    # シーケンス番号の有無
      check_class(Bool, bTimeTick)  # タイムティックの有無

      sIndent  = TTJ_TAB  # post_conditionの基本インデント
      sAllPost = ""       # 日本語化されたpost_conditionの情報を保持するために変数

      # post_condition文字の日本語化
      if (bCondFlag == true)
        sAllPost += "#{GRP_TTJ_CONDITION[TSR_LBL_POST]}#{(bSeqNum == true ? nSeqNum : "")}#{TTJ_NEW_LINE}"
      end

      # post_conditionにタイムティック情報がある場合タイムティックを処理する
      # タイムティック記入およびインデントの挿入
      if (bTimeTick == true)
        sAllPost += "#{sIndent}#{nTimeTick}#{TTJ_NEW_LINE}"
        sIndent  += "#{TTJ_TAB}"
      end

      # doの情報を日本語化する
      sAllPost = japanize_condition_info(sAllPost, sIndent, TSR_LBL_POST)

      return sAllPost  # [String]日本語化されたpost_conditionの情報
    end

  end
end
