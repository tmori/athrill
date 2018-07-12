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
#  $Id: ASPBuilder.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "bin/builder/ProfileBuilder.rb"

module TTG

  #==================================================================
  # クラス名: ASPBuilder
  # 概    要: テストシナリオを用いて中間コードを生成するクラス
  #==================================================================
  class ASPBuilder < ProfileBuilder
    include CommonModule

    #================================================================
    # 概  要: コンストラクタ
    #================================================================
    def initialize()
      super()

      # ASPでのGCOV取得設定
      if (@cConf.enable_gcov?())
        # GCOV取得用関数定義
        @cIMCode.add_gcov_resume_asp()
        @cIMCode.add_gcov_pause_asp()
      end

      # 時間制御関数があれば，システム時刻更新確認関数を定義
      if (@cConf.get_func_time())
        @cIMCode.add_gain_tick_asp()
      end
    end

    #================================================================
    # 概  要: pre_conditionを生成するための中間コードを生成する
    #================================================================
    def build_pre_condition()
      # pre_conditionのハッシュを追加する
      @cIMCode.set_condition_level(TSR_LBL_PRE)

      #
      # pre_conditionで進む時間の設定
      #
      @cTS.set_pre_gain_tick()


      #
      # タスク例外処理許可状態の設定
      #
      if (@cTS.exist_pre_task_tex_ena() == true)
        @cIMCode.add_element(@cTS.gc_pre_task_tex_ena())
      end


      #
      # 待ちオブジェクトの設定
      #

      # 同期・通信オブジェクトの初期設定
      if (@cTS.exist_pre_scobj_data() == true)
        @cIMCode.add_element(@cTS.gc_pre_scobj_data())
      end

      # オブジェクト待ちタスクの設定
      if (@cTS.exist_pre_task_scobj_waiting() == true)
        @cIMCode.add_element(@cTS.gc_pre_task_scobj_waiting())
      end


      #
      # 実行状態ではない，タスクコンテキストの設定
      #

      # 待ち状態(スリープ，ディレイ)のタスクの設定
      # ＋二重待ち状態の待ち状態(スリープ，ディレイ)の設定
      if (@cTS.exist_pre_task_waiting() == true)
        @cIMCode.add_element(@cTS.gc_pre_task_waiting())
      end

      # 強制待ち・二重待ち状態のタスクの設定
      # ＋二重待ち状態の強制待ち状態の設定
      if (@cTS.exist_pre_task_suspended() == true)
        @cIMCode.add_element(@cTS.gc_pre_task_suspended())
      end

      # 実行可能状態のタスクの設定
      if (@cTS.exist_pre_task_ready() == true)
        @cIMCode.add_element(@cTS.gc_pre_task_ready())
      end


      #
      # タスク保留例外要因の設定(running,dormant,オブジェクト待ち以外のタスク)
      #
      if (@cTS.exist_pre_task_tex_pndptn() == true)
        @cIMCode.add_element(@cTS.gc_pre_task_tex_pndptn())
      end


      #
      # 現在優先度と初期優先度が異なるタスクの設定(running,dormant以外のタスク)
      #
      if (@cTS.exist_pre_task_pri_chg() == true)
        @cIMCode.add_element(@cTS.gc_pre_task_pri_chg())
      end


      #
      # 動作状態（Txxx_STA）のタイムイベントハンドラの設定
      #
      if (@cTS.exist_time_event_sta() == true)
        @cIMCode.add_element(@cTS.gc_pre_time_event_sta())
      end


      #
      # 許可状態の割込みの設定(ena_intをサポートしている場合のみ)
      #
      if ((@cTS.exist_interrupt_ena() == true) && (@cConf.get_api_support_ena_int() == true))
        @cIMCode.add_element(@cTS.gc_pre_interrupt_ena())
      end


      #
      # 実行状態のタスクコンテキストの設定
      # （タスク，タスク例外）
      #
      if (@cTS.exist_task_running() == true)
        # 起動状態（ACTIVATE）のタスク例外がいない場合
        if (@cTS.exist_pre_task_texhdr_activate() == false)
          @cIMCode.add_element(@cTS.gc_pre_task_running())
        # 起動状態（ACTIVATE）のタスク例外がいる場合
        else
          @cIMCode.add_element(@cTS.gc_pre_task_texhdr_activate())
        end
      end


      # ディスパッチ禁止の設定
      if (@cTS.check_dis_dsp() == true)
        @cIMCode.add_element(@cTS.gc_pre_dis_dsp())
      end


      # 割込み優先度マスクの設定
      # (非タスクが起動することによって変更された場合は変更しない)
      if (@cTS.check_chg_ipm() == true)
        @cIMCode.add_element(@cTS.gc_pre_set_ipm())
      end


      #
      # 起動状態（ACTIVATE）の非タスクコンテキストの設定
      #
      if (@cTS.exist_nontask_activate() == true)
        @cIMCode.add_element(@cTS.gc_pre_nontask_activate())
      end


      #
      # 待ち状態にしていた実行可能状態のタスクの起床
      #
      if (@cTS.exist_pre_task_ready() == true)
        @cIMCode.add_element(@cTS.gc_pre_task_ready_porder())
      end


      #
      # 起動・起床要求キューイングの設定
      #
      if (@cTS.exist_task_queueing() == true)
        @cIMCode.add_element(@cTS.gc_pre_task_queueing())
      end


      #
      # CPUロック状態の設定
      #
      if (@cTS.check_cpu_loc() == true)
        @cIMCode.add_element(@cTS.gc_pre_cpu_loc())
      end


      #
      # 前状態作成時点で，実行中の処理単位から
      # 全オブジェクトのrefコードを挿入
      #
      @cIMCode.add_element(@cTS.gc_obj_ref())


      #
      # 次の状態に行く前のメインタスクの設定
      #
      @cIMCode.add_element(@cTS.gc_pre_maintask_set())

    end

    #================================================================
    # 概  要: do_conditionを生成するための中間コードを生成する
    #================================================================
    def build_do(nIndex)
      check_class(Integer, nIndex)  # do/postのシーケンス番号

      # doのハッシュを追加する
      @cIMCode.set_condition_level(TSR_LBL_DO + "_#{@cTS.get_seqnum(nIndex)}_#{@cTS.get_timetick(nIndex)}")

      # doの処理
      if (@cTS.check_do_info(nIndex) == true)
        @cIMCode.add_element(@cTS.gc_do(nIndex))
      end

      # 次のコンディションで起動するACTIVATEな非タスクが存在する場合，起動を待つ
      @cIMCode.add_element(@cTS.gc_wait_non_task_activate(nIndex))
    end

    #================================================================
    # 概  要: post_conditionの終了処理を生成するための中間コードを生成
    #         する
    #================================================================
    def build_lastpost_condition(nIndex)
      check_class(Integer, nIndex)  # do/postのシーケンス番号

      # post_conditionのハッシュを追加する
      @cIMCode.set_condition_level(TSR_LBL_POST + "_#{@cTS.get_seqnum(nIndex)}_#{@cTS.get_timetick(nIndex)}")

      # 全オブジェクトのrefコードを返す
      @cIMCode.add_element(@cTS.gc_obj_ref(nIndex))

      # ディスパッチ保留状態で優先度が逆転した状態に備え，readyのタスクを全てslp_tskする
      # (ディスパッチで実行状態のタスクが変わってもslp_tskによって，cRunningが実行状態になる)
      if ((@cTS.exist_nontask_activate(nIndex) == true) ||
          (@cTS.check_cpu_loc(nIndex) == true) ||
          (@cTS.check_dis_dsp(nIndex) == true) ||
          (@cTS.check_chg_ipm(nIndex) == true))
        @cIMCode.add_element(@cTS.gc_lastpost_ready_sleep())
      end

      #
      # 終了処理
      # 

      # 実行状態の処理単位からCPUロックを解除
      if (@cTS.check_cpu_loc(nIndex) == true)
        @cIMCode.add_element(@cTS.gc_lastpost_cpu_unl())
      end

      # 実行状態のタスクか起動状態(ACTIVATE)の非タスクがいる場合，メインタスクを起こす
      if (@cTS.exist_task_running(nIndex) == true || @cTS.exist_nontask_activate(nIndex) == true)
        @cIMCode.add_element(@cTS.gc_lastpost_maintask_wup(nIndex))
      end

      # ディスパッチ禁止の解除
      if (@cTS.check_dis_dsp(nIndex) == true)
        @cIMCode.add_element(@cTS.gc_lastpost_ena_dsp())
      end

      # 割込み優先度マスクの初期化
      if (@cTS.check_chg_ipm(nIndex) == true)
        @cIMCode.add_element(@cTS.gc_lastpost_set_ini_ipm())
      end

      #
      # この時点でメインタスクが実行状態となっている
      #

      # 正常動作では通過することのないゼロチェックポイントを各タスクへ追加
      @cIMCode.add_element(@cTS.gc_checkpoint_zero())

      # 動作状態（Txxx_STA）のタイムイベントハンドラの停止処理
      if (@cTS.exist_time_event_sta(nIndex) == true)
        @cIMCode.add_element(@cTS.gc_lastpost_time_event_stp())
      end

      #
      # 許可状態の割込みの禁止処理(dis_intをサポートしている場合のみ)
      #
      if ((@cTS.exist_interrupt_ena(nIndex) == true) && (@cConf.get_api_support_dis_int() == true))
        @cIMCode.add_element(@cTS.gc_lastpost_interrupt_dis())
      end

      # 起動要求キューイングの初期化
      if (@cTS.exist_task_queueing(nIndex) == true)
        @cIMCode.add_element(@cTS.gc_lastpost_task_can_queueing())
      end

      # メインタスクですべてのタスクをter_tskさせる
      @cIMCode.add_element(@cTS.gc_lastpost_all_task_ter())

    end


    #================================================================
    # 概  要: post_conditionを生成するための中間コードを生成する
    #================================================================
    def build_post_condition(nIndex)
      check_class(Integer, nIndex)  # do/postのシーケンス番号

      # post_conditionのハッシュを追加する
      @cIMCode.set_condition_level(TSR_LBL_POST + "_#{@cTS.get_seqnum(nIndex)}_#{@cTS.get_timetick(nIndex)}")

      # 全オブジェクトのrefコードを返す
      @cIMCode.add_element(@cTS.gc_obj_ref(nIndex))

      # 次のコンディションへ時間の差だけ進める
      if (@cTS.exist_post_timetick(nIndex) == true)
        @cTS.get_post_time(nIndex).times{
          @cIMCode.add_element(@cTS.gc_tick_gain(nIndex))
        }
      end

      # 次の状態に行く前のメインタスクの設定のコードを返す
      @cIMCode.add_element(@cTS.gc_post_maintask_set(nIndex, @cTS.exist_post_timetick(nIndex)))

    end

    #================================================================
    # 概  要: メインタスクでGCOV取得を開始する
    #================================================================
    def build_main_task_gcov_start()
      cElement = IMCodeElement.new(:common)
      cElement.set_code(@cIMCode.hMainTask, "#{TTG_NL}#{TTG_TB}#{FNC_GCOV_C_PAUSE}")
      @cIMCode.add_element(cElement)
    end

    #================================================================
    # 概  要: メインタスクから割込みを発生させる
    #================================================================
    def build_int_raise(cElement, snIntNo, nPrcid)
      check_class(IMCodeElement, cElement)    # エレメント
      check_class([String, Integer], snIntNo) # 割込み番号
      check_class(Integer, nPrcid, true)      # プロセッサID(ASPでは不使用)

      cElement.set_code(@cIMCode.hMainTask, "#{FNC_INT_RAISE}(#{snIntNo})")
    end

    #================================================================
    # 概  要: メインタスクでdis_intを実行する
    #================================================================
    def build_dis_int(cElement, snIntNo, nPrcid)
      check_class(IMCodeElement, cElement)    # エレメント
      check_class([String, Integer], snIntNo) # 割込み番号
      check_class(Integer, nPrcid, true)      # プロセッサID(ASPでは不使用)

      cElement.set_syscall(@cIMCode.hMainTask, "#{API_DIS_INT}(#{snIntNo})")
    end

  end
end
