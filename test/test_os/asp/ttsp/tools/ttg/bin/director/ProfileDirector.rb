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
#  $Id: ProfileDirector.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/CommonModule.rb"

module TTG

  #==================================================================
  # クラス名: ProfileDirector
  # 概　  要: ProfileBuilderを利用して中間コードを生成するクラス
  #==================================================================
  class ProfileDirector
    include CommonModule

    #================================================================
    # 概　要: コンストラクタ
    #================================================================
    def initialize(cProfileBuilder)
      check_class(ProfileBuilder, cProfileBuilder) # ProfileBuilderクラスのインスタンス

      @cProfileBuilder = cProfileBuilder.dup()

      @cConf = Config.new() # Configを取得
    end


    #================================================================
    # 概　要: 指定されたProfileBuilderを用いてbuildを行う
    #================================================================
    def build(aTestScenarios)
      check_class(Array, aTestScenarios) # テストシナリオの配列

      # 初期化ルーチン/終了ルーチンクラスの配列を作成する
      bIsIniRtn = false
      bIsTerRtn = false
      aIniRtn = []
      aTerRtn = []
      aTestScenarios.each{|cTestScenario|
        if (@cProfileBuilder.exist_ini_rtn(cTestScenario) == true)
          aIniRtn.concat(cTestScenario.get_ini_ter_object_info_sort_by_id(TSR_OBJ_INIRTN))
          bIsIniRtn = true
        end
        if (@cProfileBuilder.exist_ter_rtn(cTestScenario) == true)
          aTerRtn.concat(cTestScenario.get_ini_ter_object_info_sort_by_id(TSR_OBJ_TERRTN))
          bIsTerRtn = true
        end
      }

      # 初期化ルーチンのコードを作成する
      if (bIsIniRtn == true)
        # FMPではグローバル初期化ルーチンでテストライブラリ用変数の初期化を行う
        if (@cConf.is_fmp?())
          @cProfileBuilder.build_initialize_test_lib_ini_rtn()
        end

        @cProfileBuilder.build_ini_rtn(aIniRtn)

        # 初期化ルーチンがあるときはメインタスクの先頭でcp_stateをチェックする
        @cProfileBuilder.build_check_cp_state()
      else
        # 初期化ルーチンが無い場合，メインタスクの先頭で各初期化処理を行う
        @cProfileBuilder.build_initialize()
      end


      # TA_STA属性の周期ハンドラの情報を取得
      hCyclicStaInfo = get_all_object_info(aTestScenarios, TSR_OBJ_CYCLE)
      # TA_STA属性の周期ハンドラがあるときはメインタスクの先頭で状態チェックとstp_cycする
      if (hCyclicStaInfo.empty?() == false)
        @cProfileBuilder.build_stp_cyc(hCyclicStaInfo)
      end

      # TA_ENAINT属性の割込みサービスルーチンの情報を取得
      hISRInfo = get_all_object_info(aTestScenarios, TSR_OBJ_ISR)
      # TA_ENAINT属性の割込みサービスルーチンがあるときは割込み確認とdis_intを発行する
      if (hISRInfo.empty?() == false)
        @cProfileBuilder.build_ena_int_isr(hISRInfo)
      end

      # 割込みハンドラの情報を取得
      hIntHdrInfo = get_all_object_info(aTestScenarios, TSR_OBJ_INTHDR)
      # 割込みハンドラがあるときは共有処理単位/関数を定義し，割込み確認とdis_intを発行する
      if (hIntHdrInfo.empty?() == false)
        @cProfileBuilder.build_shared_proc_unit(hIntHdrInfo, TSR_OBJ_INTHDR)
      end

      # CPU例外ハンドラの情報を取得
      hExceptionInfo = get_all_object_info(aTestScenarios, TSR_OBJ_EXCEPTION)
      # CPU例外ハンドラがあるときは共有処理単位/関数を定義する
      if (hExceptionInfo.empty?() == false)
        @cProfileBuilder.build_shared_proc_unit(hExceptionInfo, TSR_OBJ_EXCEPTION)
      end


      # GCOVを取得する かつ 初期化ルーチンが無い場合の処理(プロファイル毎に異なる)
      if (@cConf.enable_gcov?() && (bIsIniRtn == false))
        @cProfileBuilder.build_main_task_gcov_start()
      end

      # 中間コードにテストシナリオごとに必要な情報を追加する
      nCnt = 0
      nTotal = aTestScenarios.size()
      # 前のテストシナリオにおける時間動作フラグをconfigure.yamlのフラグで初期化
      bPrevGainTime = @cConf.is_all_gain_time_mode?()
      aTestScenarios.each{|cTestScenario|
        nCnt += 1
        if (@cConf.is_no_progress_bar_mode?() == true)
          $stderr.puts "[TTG][#{"%5.1f" % (100 * nCnt.to_f / nTotal.to_f)}\% (#{"%4d" % nCnt}/#{"%4d" % nTotal})] #{cTestScenario.sTestID}"
        else
          print_progress("TTG", cTestScenario.sTestID, nCnt, nTotal)
        end

        @cProfileBuilder.build(cTestScenario, bPrevGainTime)
        bPrevGainTime = cTestScenario.bGainTime
      }
      # プログレスバー用表示のリセット
      if (@cConf.is_no_progress_bar_mode?() != true)
        finish_progress("TTG", nTotal)
      end


      # 最後のチェックポイントを入れる
      # 終了ルーチンがある場合はcheck_pointしてext_ker，無い場合はcheck_finish
      @cProfileBuilder.build_finish_cp(bIsTerRtn)

      # 終了ルーチンのコードを作成する
      if (bIsTerRtn == true)
        @cProfileBuilder.build_ter_rtn(aTerRtn)
      end
    end

    #================================================================
    # 概  要: 指定したオブジェクトの全テストシナリオの情報を返す
    #         (割込み，CPU例外，周期ハンドラ(TA_STA)のみに使用する)
    #================================================================
    def get_all_object_info(aTestScenarios, sObjectType)
      check_class(Array, aTestScenarios) # 全テストシナリオデータ
      check_class(String, sObjectType)   # 対象とするオブジェクトタイプ

      hAllObjectInfo = Hash.new()

      aTestScenarios.each{|cTestScenario|
        if (((sObjectType == TSR_OBJ_INTHDR) && (cTestScenario.exist_int_hdr() == true)) ||
            ((sObjectType == TSR_OBJ_ISR) && (cTestScenario.exist_isr_enaint() == true)) ||
            ((sObjectType == TSR_OBJ_EXCEPTION) && (cTestScenario.exist_exception() == true)) ||
            ((sObjectType == TSR_OBJ_CYCLE) && (cTestScenario.exist_cyclic_sta() == true)))
          hAllObjectInfo.update(cTestScenario.get_all_object_info(sObjectType)){|key, self_val, other_val|
            # bootcntは最大値を選択
            if (self_val.hState[TSR_PRM_BOOTCNT] > other_val.hState[TSR_PRM_BOOTCNT])
              self_val
            else
              other_val
            end
          }
        end
      }

      return hAllObjectInfo  # [Hash]オブジェクト情報ハッシュ
    end

  end
end
