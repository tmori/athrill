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
require "common/bin/CommonModule.rb"
require "common/bin/Config.rb"
require "common/bin/test_scenario/PreCondition.rb"
require "common/bin/test_scenario/Do_PostCondition.rb"
require "ttc/bin/test_scenario/TestScenario.rb"
require "ttj/bin/test_scenario/TestScenario.rb"
require "bin/builder/fmp_builder/test_scenario/TestScenario.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: TestScenario
  # 概    要: PreCondition，Do，PostConditionのデータを生成
  #===================================================================
  class TestScenario
    include CommonModule

    attr_reader :sTestID, :hVariation, :cPreCondition, :aDo_PostCondition, :bGainTime, :bGcovAll

    #=================================================================
    # 概  要: コンストラクタ
    #=================================================================
    def initialize(sTestID, hScenario)
      check_class(Object, sTestID, true)    # テストID
      check_class(Object, hScenario, true)  # pre，do，postシナリオ

      @cConf        = Config.new()  # コンフィグの取得
      @hVariation   = {}            # バリエーション情報
      @bGainTime    = nil           # 時間処理
      @bHasTimeTick = false         # タイムティックを持つか判定用フラグ
      @sTestID      = sTestID       # テストID
      @sNote        = nil           # コメント

      # 構造チェック
      basic_check(sTestID, hScenario)

      # タイムティックを持つか判定
      hMacro = @cConf.get_macro()
      hScenario.each{|sCondition, shCondition|
        if (sCondition =~ TSR_REX_PRE_DO || sCondition =~ TSR_REX_PRE_POST)
          if (has_timetick?(shCondition))
            @bHasTimeTick = true
            break
          end
        end
      }

      # variation格納
      if (hScenario[TSR_LBL_VARIATION].nil?())
        @hVariation = {}
      else
        @hVariation = hScenario[TSR_LBL_VARIATION]
      end
      # gcov_all
      if (@hVariation[TSR_PRM_GCOV_ALL].nil?())
        @bGcovAll = false
      else
        @bGcovAll = @hVariation[TSR_PRM_GCOV_ALL]
      end

      # note格納
      @sNote = hScenario[TSR_LBL_NOTE]

      # pre_condition生成
      @cPreCondition = PreCondition.new(sTestID, hScenario[TSR_LBL_PRE])
      @cPreCondition.set_gcov_all_flg(@bGcovAll)

      # do，post_condition生成(doとpostは一緒に処理)
      @aDo_PostCondition = []

      # 構造を整形
      hScenario = fix_structure(hScenario)

      # シーケンス番号取得
      aSeqNumList = []
      hScenario.each_key{|sCondition|
        if (sCondition =~ TSR_REX_DO)
          aSeqNumList.push($1.to_i())
        end
      }
      aSeqNumList = aSeqNumList.sort()

      # 時系列順にdoとpostを作成
      aSeqNumList.each{|nSeqNum|
        sSeqDo   = "#{TSR_UNS_DO}#{nSeqNum}"
        sSeqPost = "#{TSR_UNS_POST}#{nSeqNum}"
        # 該当シーケンス番号での全タイムティック取得
        aTimeTickList = hScenario[sSeqDo].keys()
        aTimeTickList.concat(hScenario[sSeqPost].keys())
        aTimeTickList = aTimeTickList.uniq().sort()

        aTimeTickList.each{|nTimeTick|
          hScenarioDo   = hScenario["#{TSR_UNS_DO}#{nSeqNum}"][nTimeTick]
          hScenarioPost = hScenario["#{TSR_UNS_POST}#{nSeqNum}"][nTimeTick]

          cObj = Do_PostCondition.new(sTestID, hScenarioDo, hScenarioPost, nSeqNum, nTimeTick, @cPreCondition)
          cObj.set_gcov_all_flg(@bGcovAll)
          @aDo_PostCondition.push(cObj)
        }
      }
    end

    #=================================================================
    # 概  要: テストシナリオの構造を整える
    #=================================================================
    def fix_structure(hScenario)
      check_class(Hash, hScenario)  # テストシナリオ

      hResult = {}
      hScenario.each{|sCondition, shCondition|
        case sCondition
        when TSR_LBL_PRE, TSR_LBL_VARIATION, TSR_LBL_NOTE
          hResult[sCondition] = shCondition
        when TSR_REX_PRE_DO
          sSeq    = "#{TSR_UNS_DO}#{$1.to_i()}"
          hResult = fix_condition(hResult, shCondition, sSeq)
        when TSR_REX_PRE_POST
          sSeq    = "#{TSR_UNS_POST}#{$1.to_i()}"
          hResult = fix_condition(hResult, shCondition, sSeq)
        else
          abort(ERR_MSG % [__FILE__, __LINE__])
        end
      }

      return hResult  # [Hash]整形されたテストシナリオ
    end

    #=================================================================
    # 概  要: コンディションの構造を整える
    #=================================================================
    def fix_condition(hScenario, hCondition, sSeq)
      check_class(Hash, hScenario)         # シナリオ
      check_class(Hash, hCondition, true)  # コンディション
      check_class(String, sSeq)            # シーケンス番号

      hResult = hScenario.dup()
      hMacro = @cConf.get_macro()
      # タイムティックあり
      if (has_timetick?(hCondition))
        hResult[sSeq] = {}
        hCondition.each{|nTimeTick, hBlock|
          nTimeTick = parse_value(nTimeTick, hMacro)
          hResult[sSeq][nTimeTick] = hBlock
        }
      # タイムティック無し（0ティック補完）
      else
        hResult[sSeq] = {}
        hResult[sSeq][0] = hCondition
      end

      return hResult  # [Hash]整形されたコンディション
    end

    #=================================================================
    # 概  要: 各コンディションを初期化
    #=================================================================
    def init_conditions()
      aConditions = get_all_condition()
      aConditions.each{|cCondition|
        cCondition.init_instance_variables()
      }
    end

    #=================================================================
    # 概  要: シーケンス番号を返す
    #=================================================================
    def get_seqnum(nIndex)
      check_class(Integer, nIndex)  # 何番目のdo_postか  # 何番目のdo_postか

      return @aDo_PostCondition[nIndex].nSeqNum  # [Integer]シーケンス番号
    end

    #=================================================================
    # 概  要: タイムティック番号を返す
    #=================================================================
    def get_timetick(nIndex)
      check_class(Integer, nIndex)  # 何番目のdo_postか  # 何番目のdo_postか

      return @aDo_PostCondition[nIndex].nTimeTick  # [Integer]タイムティック番号
    end

    #=================================================================
    # 概  要: pre_conditionで進む時間の設定
    #=================================================================
    def set_pre_gain_tick()
      @cPreCondition.set_pre_gain_tick()
    end

    #=================================================================
    # 概  要: ディスパッチ禁止の確認(ディスパッチ禁止状態の場合はtrue)
    #=================================================================
    def check_dis_dsp(nIndex = nil)
      check_class(Integer, nIndex, true)  # 何番目のdo_postか  # 何番目のdo_postか

      if (nIndex.nil?())
        return @cPreCondition.check_dis_dsp()  # [Bool]ディスパッチ禁止状態
      else
        return @aDo_PostCondition[nIndex].check_dis_dsp()  # [Bool]ディスパッチ禁止状態
      end
    end

    #=================================================================
    # 概  要: 割込み優先度マスクの確認(全解除ではない場合はtrue)
    #=================================================================
    def check_chg_ipm(nIndex = nil)
      check_class(Integer, nIndex, true)  # 何番目のdo_postか  # 何番目のdo_postか

      if (nIndex.nil?())
        return @cPreCondition.check_pre_need_chg_ipm()  # [Bool]割込み優先度マスク
      else
        return @aDo_PostCondition[nIndex].check_chg_ipm()  # [Bool]割込み優先度マスク
      end
    end

    #=================================================================
    # 概  要: CPUロック状態の確認(CPUロック状態の場合はtrue)
    #=================================================================
    def check_cpu_loc(nIndex = nil)
      check_class(Integer, nIndex, true)  # 何番目のdo_postか  # 何番目のdo_postか

      if (nIndex.nil?())
        return @cPreCondition.check_cpu_loc()  # [Bool]CPUロック状態
      else
        return @aDo_PostCondition[nIndex].check_cpu_loc()  # [Bool]CPUロック状態
      end
    end

    #=================================================================
    # 概  要: 最後のpost_conditionか確認
    #=================================================================
    def check_lastpost_condition(nIndex)
      check_class(Integer, nIndex)  # 何番目のdo_postか

      if (@aDo_PostCondition[TTG_IDX_LAST] == @aDo_PostCondition[nIndex])
        return true  # [Bool]最後のpost_conditionか
      end

      return false  # [Bool]最後のpost_conditionか
    end

    #=================================================================
    # 概  要: メインタスクID，プロセッサID，起動回数を取得
    #=================================================================
    def get_main_task_proc()
      return @cPreCondition.get_proc_unit_info()  # [Hash]メインタスクの情報
    end

    #=================================================================
    # 概  要: タスクの数を返す
    #=================================================================
    def get_stack_num()
      return @cPreCondition.hTask.size()  # [Integer]タスクの数
    end

    #=================================================================
    # 概  要: 全ての処理単位名とそれぞれの最終的な起動回数を取得
    #=================================================================
    def get_proc_units(cElement)
      check_class(IMCodeElement, cElement)  # コードを追加するエレメント

      @aDo_PostCondition[TTG_IDX_LAST].get_proc_units(cElement)
    end

    #=================================================================
    # 概  要: 現在の状態の前状態との時間の差を取得
    #=================================================================
    def get_post_time(nIndex)
      check_class(Integer, nIndex)  # 何番目のdo_postか

      return @aDo_PostCondition[nIndex + 1].nTimeTick - @aDo_PostCondition[nIndex].nTimeTick  # [Integer]現在の状態の前状態との時間の差
    end

    #=================================================================
    # 概  要: タスク例外処理許可状態のタスク有無確認
    #=================================================================
    def exist_pre_task_tex_ena()
      return @cPreCondition.exist_pre_task_tex_ena()  # [Bool]タスク例外処理許可状態のタスクが存在するか
    end

    #=================================================================
    # 概  要: ACTIVATEであるタスク例外ハンドラの有無確認
    #=================================================================
    def exist_pre_task_texhdr_activate()
      return @cPreCondition.exist_pre_task_texhdr_activate()  # [Bool]ACTIVATEであるタスク例外ハンドラが存在するか
    end

    #=================================================================
    # 概  要: 待ちオブジェクトの有無確認
    #=================================================================
    def exist_pre_scobj_data()
      return @cPreCondition.exist_pre_scobj_data()  # [Bool]待ちオブジェクトが存在するか
    end

    #=================================================================
    # 概  要: オブジェクト待ちのタスクの有無確認
    #=================================================================
    def exist_pre_task_scobj_waiting()
      return @cPreCondition.exist_pre_task_scobj_waiting()  # [Bool]オブジェクト待ちのタスクが存在するか
    end

    #=================================================================
    # 概  要: 待ち状態(スリープ，ディレイ)のタスクの有無確認
    #         （二重待ち状態の待ち状態(スリープ，ディレイ)も含む）
    #=================================================================
    def exist_pre_task_waiting()
      return @cPreCondition.exist_pre_task_waiting()  # [Bool]待ち状態のタスクが存在するか
    end

    #=================================================================
    # 概  要: 強制待ち・二重待ち状態のタスクの有無確認
    #         （二重待ち状態の強制待ちも含む）
    #=================================================================
    def exist_pre_task_suspended()
      return @cPreCondition.exist_pre_task_suspended()  # [Bool]強制待ち・二重待ち状態のタスクが存在するか
    end

    #=================================================================
    # 概  要: 実行可能状態のタスクの有無確認
    #=================================================================
    def exist_pre_task_ready()
      return @cPreCondition.exist_pre_task_ready()  # [Bool]実行可能状態のタスクが存在するか
    end

    #=================================================================
    # 概  要: 現在優先度と初期優先度が異なるタスクの有無確認
    #=================================================================
    def exist_pre_task_pri_chg()
      return @cPreCondition.exist_pre_task_pri_chg()  # [Bool]現在優先度と初期優先度が異なるタスクが存在するか
    end

    #=================================================================
    # 概  要: 起動・起床要求キューイングの設定をしているタスクの有無確
    #         認
    #=================================================================
    def exist_task_queueing(nIndex = nil)
      check_class(Integer, nIndex, true)  # 何番目のdo_postか

      if (nIndex.nil?())
        return @cPreCondition.exist_task_queueing()  # [Bool]起動・起床要求キューイングの設定をしているタスクが存在するか
      else
        return @aDo_PostCondition[TTG_IDX_LAST].exist_task_queueing()  # [Bool]起動・起床要求キューイングの設定をしているタスクが存在するか
      end
    end

    #=================================================================
    # 概  要: タスク保留例外要因が設定されているタスク例外の有無確認
    #=================================================================
    def exist_pre_task_tex_pndptn()
      return @cPreCondition.exist_pre_task_tex_pndptn()  # [Bool]タスク保留例外要因が設定されているタスク例外が存在するか
    end

    #=================================================================
    # 概  要: 動作状態（Txxx_STA）のタイムイベントハンドラの有無確認
    #=================================================================
    def exist_time_event_sta(nIndex = nil)
      check_class(Integer, nIndex, true)  # 何番目のdo_postか

      if (nIndex.nil?())
        return @cPreCondition.exist_time_event_sta()  # [Bool]動作状態のタイムイベントハンドラが存在するか
      else
        return @aDo_PostCondition[TTG_IDX_LAST].exist_time_event_sta()  # [Bool]動作状態のタイムイベントハンドラが存在するか
      end
    end

    #=================================================================
    # 概  要: 許可状態の割込みハンドラ，割込みサービスルーチンの有無確
    #         認
    #=================================================================
    def exist_interrupt_ena(nIndex = nil)
      check_class(Integer, nIndex, true)  # 何番目のdo_postか

      if (nIndex.nil?())
        return @cPreCondition.exist_interrupt_ena()  # [Bool]許可状態の割込みハンドラ，割込みサービスルーチンが存在するか
      else
        return @aDo_PostCondition[TTG_IDX_LAST].exist_interrupt_ena()  # [Bool]許可状態の割込みハンドラ，割込みサービスルーチンが存在するか
      end
    end

    #=================================================================
    # 概  要: 実行状態のタスクの有無確認
    #=================================================================
    def exist_task_running(nIndex = nil)
      check_class(Integer, nIndex, true)  # 何番目のdo_postか

      if (nIndex.nil?())
        return @cPreCondition.exist_task_running()  # [Bool]実行状態のタスクが存在するか
      else
        return @aDo_PostCondition[TTG_IDX_LAST].exist_task_running()  # [Bool]実行状態のタスクが存在するか
      end
    end

    #=================================================================
    # 概  要: 起動中の非タスクの有無確認
    #=================================================================
    def exist_nontask_activate(nIndex = nil)
      check_class(Integer, nIndex, true)  # 何番目のdo_postか

      if (nIndex.nil?())
        return @cPreCondition.exist_nontask_activate()  # [Bool]起動中の非タスクが存在するか
      else
        return @aDo_PostCondition[TTG_IDX_LAST].exist_nontask_activate()  # [Bool]起動中の非タスクが存在するか
      end
    end

    #=================================================================
    # 概  要: 時間を進める必要があるか確認
    #=================================================================
    def exist_post_timetick(nIndex)
      check_class(Integer, nIndex)  # 何番目のdo_postか

      if (@aDo_PostCondition[nIndex].nTimeTick != @aDo_PostCondition[nIndex + 1].nTimeTick)
        return true  # [Bool]時間を進める必要があるか
      else
        return false  # [Bool]時間を進める必要があるか
      end
    end

    #=================================================================
    # 概  要: doが情報を持っていればtrue，そうではなければfalseを返す
    #=================================================================
    def check_do_info(nIndex)
      check_class(Integer, nIndex)  # 何番目のdo_postか

      if (@aDo_PostCondition[nIndex].hDo.empty?())
        return false  # [Bool]doに設定がされているか
      else
        return true  # [Bool]doに設定がされているか
      end
    end

    #=================================================================
    # 概  要: グローバル/ローカル変数を宣言するためのコードを設定する
    #=================================================================
    def gc_global_local_var(cElement)
      check_class(IMCodeElement, cElement)  # コードを追加するエレメント

      @cPreCondition.gc_global_local_var(cElement)
    end

    #=================================================================
    # 概  要: ヘッダーファイルに出力するコードを設定する
    #=================================================================
    def gc_header(cElement)
      check_class(IMCodeElement, cElement)  # コードを追加するエレメント

      @cPreCondition.gc_header(cElement)
    end

    #=================================================================
    # 概  要: コンフィグファイルに出力するコードを設定する
    #=================================================================
    def gc_config(cElement)
      check_class(IMCodeElement, cElement)  # コードを追加するエレメント

      @cPreCondition.gc_config(cElement)
    end

    #=================================================================
    # 概  要: タスク例外処理許可状態にするコードを返す
    #=================================================================
    def gc_pre_task_tex_ena()
      return @cPreCondition.gc_pre_task_tex_ena()  # [IMCodeElement]タスク例外処理許可状態にするコード
    end

    #=================================================================
    # 概  要: タスク例外処理を起こすコードを返す
    #=================================================================
    def gc_pre_task_texhdr_activate()
      return @cPreCondition.gc_pre_task_texhdr_activate()  # [IMCodeElement]タスク例外処理を起こすコード
    end

    #=================================================================
    # 概  要: 待ちオブジェクトの初期設定のためのコードを返す
    #=================================================================
    def gc_pre_scobj_data()
      return @cPreCondition.gc_pre_scobj_data()  # [IMCodeElement]待ちオブジェクトの初期設定のためのコード
    end

    #=================================================================
    # 概  要: オブジェクト待ちタスク設定のためのコードを返す
    #=================================================================
    def gc_pre_task_scobj_waiting()
      return @cPreCondition.gc_pre_task_scobj_waiting()  # [IMCodeElement]オブジェクト待ちタスク設定のためのコード
    end

    #=================================================================
    # 概  要: 待ち状態(スリープ，ディレイ)のタスクと，二重待ち状態の
    #         待ち状態のタスクに設定するコードを返す
    #         （二重待ち状態の待ち状態(スリープ，ディレイ)も含む）
    #=================================================================
    def gc_pre_task_waiting()
      return @cPreCondition.gc_pre_task_waiting()  # [IMCodeElement]待ち状態・二重待ち状態のタスクに設定するコード
    end

    #=================================================================
    # 概  要: 強制待ち・二重待ち状態に設定するコードを返す
    #         （二重待ち状態の強制待ちも含む）
    #=================================================================
    def gc_pre_task_suspended()
      return @cPreCondition.gc_pre_task_suspended()  # [IMCodeElement]強制待ち・二重待ち状態に設定するコード
    end

    #=================================================================
    # 概  要: 実行可能状態のタスクに設定するコードを返す
    #         実際は起こした後に，一律に待ち状態にしている
    #         （この時点で，タスク例外処理が許可されているタスクは）
    #=================================================================
    def gc_pre_task_ready()
      return @cPreCondition.gc_pre_task_ready()  # [IMCodeElement]実行可能状態のタスクに設定するコード
    end

    #=================================================================
    # 概  要: タスク保留例外要因に設定するコードを返す
    #=================================================================
    def gc_pre_task_tex_pndptn()
      return @cPreCondition.gc_pre_task_tex_pndptn()  # [IMCodeElement]タスク保留例外要因に設定するコード
    end

    #=================================================================
    # 概  要: 動作状態（Txxx_STA）のタイムイベントハンドラに設定する
    #         コードを返す
    #=================================================================
    def gc_pre_time_event_sta()
      return @cPreCondition.gc_pre_time_event_sta()  # [IMCodeElement]動作状態のタイムイベントハンドラに設定するコード
    end

    #=================================================================
    # 概  要: 許可状態の割込みを設定するコードを返す
    #=================================================================
    def gc_pre_interrupt_ena()
      return @cPreCondition.gc_pre_interrupt_ena()  # [IMCodeElement]許可状態の割込みを設定するコード
    end

    #=================================================================
    # 概  要: 現在優先度と初期優先度が異なるタスクの設定処理を
    #         IMCodeElementクラスにまとめて返す(dormant以外のタスク)
    #=================================================================
    def gc_pre_task_pri_chg()
      return @cPreCondition.gc_pre_task_pri_chg()  # [IMCodeElement]現在優先度と初期優先度が異なるタスクを設定するコード
    end

    #=================================================================
    # 概  要: 実行状態のタスクの設定をするコードを返す
    #=================================================================
    def gc_pre_task_running()
      return @cPreCondition.gc_pre_task_running()  # [IMCodeElement]実行状態のタスクの設定をするコード
    end

    #=================================================================
    # 概  要: ディスパッチ禁止にさせるコードを返す
    #=================================================================
    def gc_pre_dis_dsp()
      return @cPreCondition.gc_pre_dis_dsp()  # [IMCodeElement]ディスパッチ禁止にさせるコード
    end

    #=================================================================
    # 概  要: 割込み優先度マスクが0以外にさせるコードを返す
    #=================================================================
    def gc_pre_set_ipm()
      return @cPreCondition.gc_pre_set_ipm()  # [IMCodeElement]割込み優先度マスクが0以外にさせるコード
    end

    #=================================================================
    # 概  要: 起動中の非タスクの設定コードを返す
    #=================================================================
    def gc_pre_nontask_activate()
      return @cPreCondition.gc_pre_nontask_activate()  # [IMCodeElement]起動中の非タスクの設定コード
    end

    #=================================================================
    # 概  要: 時間進めるコードを返す
    #=================================================================
    def gc_tick_gain(nIndex)
      check_class(Integer, nIndex)  # 何番目のdo_postか

      cElement = IMCodeElement.new()

      @aDo_PostCondition[nIndex].gc_tick_gain(cElement)

      return cElement  # [IMCodeElement]時間進めるコード
    end

    #=================================================================
    # 概  要: 一時スリープさせておいたタスクを実行可能状態にさせるコードを返す
    #=================================================================
    def gc_pre_task_ready_porder()
      return @cPreCondition.gc_pre_task_ready_porder()  # [IMCodeElement]一時スリープさせておいたタスクを実行可能状態にさせるコード
    end

    #=================================================================
    # 概  要: 起動・起床要求キューイングの設定をするコードを返す
    #=================================================================
    def gc_pre_task_queueing()
      return @cPreCondition.gc_pre_task_queueing()  # [IMCodeElement]起動・起床要求キューイングの設定をするコード
    end

    #=================================================================
    # 概  要: CPUロック状態の設定（実行中の処理単位から設定）
    #=================================================================
    def gc_pre_cpu_loc()
      return @cPreCondition.gc_pre_cpu_loc()  # [IMCodeElement]CPUロック状態の設定をするコード
    end

    #=================================================================
    # 概  要: コンディション内のオブジェクトのrefコードを返す
    #=================================================================
    def gc_obj_ref(nIndex = nil)
      check_class(Integer, nIndex, true)  # 何番目のdo_postか

      if (nIndex.nil?())
        return @cPreCondition.gc_obj_ref()  # [IMCodeElement]コンディション内のオブジェクトのrefコード
      else
        return @aDo_PostCondition[nIndex].gc_obj_ref()  # [IMCodeElement]コンディション内のオブジェクトのrefコード
      end
    end

    #=================================================================
    # 概  要: Doの処理のコードを返す
    #=================================================================
    def gc_do(nIndex)
      check_class(Integer, nIndex)  # 何番目のdo_postか

      if (nIndex == 0)
        cPrevObjectInfo = @cPreCondition.get_prev_actvate_running()
      else
        cPrevObjectInfo = @aDo_PostCondition[nIndex - 1].get_prev_actvate_running()
      end

      return @aDo_PostCondition[nIndex].gc_do(cPrevObjectInfo)  # [IMCodeElement]Doの処理のコード
    end

    #=================================================================
    # 概  要: CPUロック状態の解除のコードを返す
    #=================================================================
    def gc_lastpost_cpu_unl()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_cpu_unl()  # [IMCodeElement]CPUロック状態の解除のコード
    end

    #=================================================================
    # 概  要: 最後の後状態でメインタスクを起こすコードを返す
    #=================================================================
    def gc_lastpost_maintask_wup(nIndex)
      check_class(Integer, nIndex)  # 何番目のdo_postか

      if (nIndex == 0)
        cPrevCondition = @cPreCondition
      else
        cPrevCondition = @aDo_PostCondition[TTG_IDX_LAST - 1]
      end

      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_maintask_wup(cPrevCondition.lMainTaskState)  # [IMCodeElement]最後の後状態でメインタスクを起こすコード
    end

    #=================================================================
    # 概  要: ディスパッチ可能にさせるコードを返す
    #=================================================================
    def gc_lastpost_ena_dsp()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_ena_dsp()  # [IMCodeElement]ディスパッチ可能にさせるコード
    end

    #=================================================================
    # 概  要: 割込み優先度マスクを初期化させるコードを返す
    #=================================================================
    def gc_lastpost_set_ini_ipm()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_set_ini_ipm()  # [IMCodeElement]割込み優先度マスクを初期化させるコード
    end

    #=================================================================
    # 概  要: 動作状態のタイムイベントハンドラの停止処理のコードを返す
    #=================================================================
    def gc_lastpost_time_event_stp()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_time_event_stp()  # [IMCodeElement]動作状態のタイムイベントハンドラの停止処理のコード
    end

    #=================================================================
    # 概  要: 禁止状態の割込みを設定するコードを返す
    #=================================================================
    def gc_lastpost_interrupt_dis()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_interrupt_dis()  # [IMCodeElement]禁止状態の割込みを設定するコード
    end

    #=================================================================
    # 概  要: 他タスクを終了させるコードを返す
    #=================================================================
    def gc_lastpost_all_task_ter()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_all_task_ter()  # [IMCodeElement]他タスクを終了させるコード
    end

    #=================================================================
    # 概  要: 起動要求キューイングを初期化するコードを返す
    #=================================================================
    def gc_lastpost_task_can_queueing()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_task_can_queueing()  # [IMCodeElement]起動要求キューイングを初期化するコード
    end

    #=================================================================
    # 概  要: readyのタスクを全てslp_tskするコードを返す
    #=================================================================
    def gc_lastpost_ready_sleep()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_lastpost_ready_sleep()  # [IMCodeElement]readyのタスクを全てslp_tskするコード
    end

    #=================================================================
    # 概  要: 前状態で，次の状態に行く前のメインタスクを設定するコード
    #         を返す
    #=================================================================
    def gc_pre_maintask_set()
      # [IMCodeElement]前状態で，次の状態に行く前のメインタスクを設定するコード
      return @cPreCondition.gc_pre_maintask_set(@aDo_PostCondition[0].cActivate, @aDo_PostCondition[0].cRunning, @aDo_PostCondition[0].check_cpu_loc(), @aDo_PostCondition[0].check_run_sus())
    end

    #=================================================================
    # 概  要: 後状態で，次の状態に行く前のメインタスクを設定するコード
    #         を返す
    #=================================================================
    def gc_post_maintask_set(nIndex, bGainTime)
      check_class(Integer, nIndex)  # 何番目のdo_postか
      check_class(Bool, bGainTime)  # 時間を進めるか

      if (nIndex == 0)
        cPrevCondition = @cPreCondition
      else
        cPrevCondition = @aDo_PostCondition[nIndex - 1]
      end
      cNowCondition  = @aDo_PostCondition[nIndex]
      cNextCondition = @aDo_PostCondition[nIndex + 1]

      # [IMCodeElement]後状態で，次の状態に行く前のメインタスクを設定するコード
      return cNowCondition.gc_post_maintask_set(cNextCondition.cActivate, cNextCondition.cRunning, cNextCondition.check_cpu_loc(), cNextCondition.check_run_sus(), cPrevCondition.lMainTaskState, bGainTime)
    end

    #=================================================================
    # 概  要: ゼロチェックポイントを出力するコードを返す
    #=================================================================
    def gc_checkpoint_zero()
      return @aDo_PostCondition[TTG_IDX_LAST].gc_checkpoint_zero()  # [IMCodeElement]ゼロチェックポイントを出力するコード
    end

    #=================================================================
    # 概  要: @aDo_PostConditionのIndex一覧を返す
    #=================================================================
    def get_do_post_index()
      aIndex = []

      @aDo_PostCondition.each_index{|nIndex|
        aIndex.push(nIndex)
      }

      return aIndex  # [Array]Index一覧
    end

    #=================================================================
    # 概  要: 初期化ルーチンの有無を返す
    #=================================================================
    def exist_ini_rtn()
      return @cPreCondition.bIsIniRtn  # [Bool]初期化ルーチンが存在するか
    end

    #=================================================================
    # 概  要: 終了ルーチンの有無を返す
    #=================================================================
    def exist_ter_rtn()
      return @cPreCondition.bIsTerRtn  # [Bool]終了ルーチンが存在するか
    end

    #=================================================================
    # 概  要: 割込みハンドラの有無を返す
    #=================================================================
    def exist_int_hdr()
      return @cPreCondition.bIsIntHdr  # [Bool]割込みハンドラが存在するか
    end

    #=================================================================
    # 概  要: 割込みサービスルーチンの有無を返す
    #=================================================================
    def exist_isr()
      return @cPreCondition.bIsISR  # [Bool]割込みサービスルーチンが存在するか
    end

    #=================================================================
    # 概  要: CPU例外ハンドラの有無を返す
    #=================================================================
    def exist_exception()
      return @cPreCondition.bIsException  # [Bool]CPU例外ハンドラが存在するか
    end

    #=================================================================
    # 概  要: 指定したオブジェクトの全情報を返す
    #=================================================================
    def get_all_object_info(sObjectType)
      check_class(String, sObjectType)  # 対象とするオブジェクトタイプ

      return @aDo_PostCondition[TTG_IDX_LAST].get_all_object_info(sObjectType)  # [Hash]オブジェクト情報
    end

    #=================================================================
    # 概  要: 指定したオブジェクトタイプの処理単位でACTIVATEであるオブ
    #         ジェクトのIDを返す
    #=================================================================
    def get_activate_proc_unit_id(sObjectType)
      check_class(String, sObjectType)  # 対象とするオブジェクトタイプ

      aProcUnitID = @cPreCondition.get_activate_proc_unit_id(sObjectType)

      @aDo_PostCondition.each{|cPostCondition|
        aProcUnitID.concat(cPostCondition.get_activate_proc_unit_id(sObjectType))
      }

      return aProcUnitID.uniq()  # [Array]ACTIVATEであるオブジェクトのID
    end

    #=================================================================
    # 概  要: TA_STA属性の周期ハンドラの有無を返す
    #=================================================================
    def exist_cyclic_sta()
      return @cPreCondition.exist_cyclic_sta()  # [Bool]TA_STA属性の周期ハンドラが存在するか
    end

    #=================================================================
    # 概  要: TA_ENAINT属性の割込みサービスルーチンの有無を返す
    #=================================================================
    def exist_isr_enaint()
      return @cPreCondition.exist_isr_enaint()  # [Bool]TA_ENAINT属性の割込みサービスルーチンが存在するか
    end

    #=================================================================
    # 概  要: 初期化/終了ルーチンの全情報をオブジェクトIDでソートした
    #         配列で返す
    #=================================================================
    def get_ini_ter_object_info_sort_by_id(sObjectType)
      check_class(String, sObjectType)  # 対象とするオブジェクトタイプ[初期化or終了ルーチン]

      return @cPreCondition.get_ini_ter_object_info_sort_by_id(sObjectType)  # [Hash]オブジェクト情報
    end

    #=================================================================
    # 概  要: 次のコンディションで起動するACTIVATEな非タスクが存在する
    #         場合，起動を待つコードを返す
    #=================================================================
    def gc_wait_non_task_activate(nIndex)
      check_class(Integer, nIndex)  # 何番目のdo_postか

      if (nIndex == 0)
        cPrevCondition = @cPreCondition
      else
        cPrevCondition = @aDo_PostCondition[nIndex - 1]
      end

      # [IMCodeElement]後状態で，次の状態に行く前のメインタスクを設定するコード
      return @aDo_PostCondition[nIndex].gc_wait_non_task_activate(cPrevCondition.cActivate, @bGainTime)
    end
  end
end
