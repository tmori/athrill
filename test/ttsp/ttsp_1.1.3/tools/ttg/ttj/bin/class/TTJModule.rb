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
#  $Id: TTJModule.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "common/bin/Config.rb"
require "ttj/bin/dictionary.rb"

module TTJModule
  include TTCModule
  include CommonModule

  $aTestScenarios = []  # テストシナリオを保持するためのグローバル変数
  $sAttrErr       = ""  # エラーメッセージ

  #===================================================================
  # 概  要: TestScenarioの情報を日本語化して返す
  #===================================================================
  def japanize_test_scenario_info()
    cConf     = Config.new()  # プログレスバー
    @aTTJInfo = []            # 日本語化した情報を保持する配列変数

    # ハッシュ構造の属性を設ける
    $hTTJAttribute = {}

    GRP_TTJ_ATTRIBUTE.each{|hAttribute|
      hAttribute.each{|key, val|
        $hTTJAttribute.store(key, val)
      }
    }

    # プログレスバーの準備
    nCnt   = 0
    nTotal = $aTestScenarios.size()

    # 各テストシナリオの処理
    $aTestScenarios.each{|cTestScenario|
      nCnt += 1

      if (cConf.is_no_progress_bar_mode?() == true)
        $stderr.puts "[TTJ][#{"%5.1f" % (100 * nCnt.to_f / nTotal.to_f)}\% (#{"%4d" % nCnt}/#{"%4d" % nTotal})] #{cTestScenario.sTestID}"
      else
        print_progress("TTJ", cTestScenario.sTestID, nCnt, nTotal)
      end

      cTSDup = Marshal.load(Marshal.dump(cTestScenario))  # 深いコピー(cTestScenario内容の保護のため)
      @aTTJInfo.push(cTSDup.japanize_tesry_info())        # 日本語化処理
    }

    # プログレスバー用表示のリセット
    if (cConf.is_no_progress_bar_mode?() != true)
      finish_progress("TTJ", nTotal)
    end
  end

  #===================================================================
  # 概  要: TTCからTESRYコードを読込んでTestScenarioを生成する
  #===================================================================
  def japanize_TTC_info(aArgs)
    check_class(Array, aArgs)  # コマンドの引数

    cTTC = TTC.new()
    $aTestScenarios = cTTC.pre_process(aArgs)
    japanize_test_scenario_info()

    # バリエーションで除外されたファイル情報を表示
    cTTC.print_exclude()
  end

  #===================================================================
  # 概  要: 画面出力
  #===================================================================
  def puts_monitor()
    @aTTJInfo.each{|hTTJInfo|
      hTTJInfo.each{|sObjectID, sObjectInfo|
        $stderr.puts("-" * 70)
        $stderr.puts(sObjectID)
        $stderr.puts("-" * 70)
        $stderr.puts("#{TTJ_NEW_LINE}#{sObjectInfo}#{TTJ_NEW_LINE}")
      }
    }

    if ($sAttrErr.empty?() == false)
      $stderr.puts("== 指定されていない属性 ==#{TTJ_NEW_LINE}")
      $stderr.puts($sAttrErr)
    end
  end

  #===================================================================
  # 概  要: ファイル出力
  #===================================================================
  def create_file()
    # ファイル生成
    File.open(TTJ_PRINT_FILE, "w"){|cIO|
      if ($sAttrErr.empty?() == false)
        cIO.puts("== 指定されていない属性 ==#{TTJ_NEW_LINE}")
        cIO.puts($sAttrErr)
      end

      @aTTJInfo.each{|hTTJInfo|
        hTTJInfo.each{|sTestID, sTTJInfo|
          cIO.puts("-" * 70)
          cIO.puts(sTestID)
          cIO.puts("-" * 70)
          cIO.puts("#{TTJ_NEW_LINE}#{sTTJInfo}#{TTJ_NEW_LINE}")
        }
      }
    }
  end

end
