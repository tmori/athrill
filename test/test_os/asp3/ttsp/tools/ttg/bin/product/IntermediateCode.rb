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
#  $Id: IntermediateCode.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "common/bin/Config.rb"
require "common/bin/IMCodeElement.rb"

module TTG

  #==================================================================
  # クラス名: IntermediateCode
  # 概　  要: 中間コードを保持するクラス
  #==================================================================
  class IntermediateCode
    include CommonModule

    #================================================================
    # 概　要: コンストラクタ
    #================================================================
    def initialize()
      @aCode = []            # コード情報
      @hProcUnitInfo = {}    # 処理単位ごとに持つ情報（ローカル変数、最終起動回数）
      @aGlobalVar = []       # グローバル変数宣言
      @aHeader = []          # ヘッダーファイルに出力する情報
      @hConfig = {}          # コンフィグファイルに出力する情報
      @nStack = 0            # スタック共有時に必要なスタック数

      @cConf           = Config.new()                             # Configを取得
      @nBlockSeqCnt    = 0                                        # ブロックのシーケンス番号
      @nBarrierSeqCnt  = 0                                        # BarrierSyncのシーケンス番号
      @aCheckSeqCnt    = Array.new(@cConf.get_prc_num() + 1, 0)   # チェックポイントのシーケンス番号（プロセッサごとに保持）

      @nIndent             = 0 # 出力コードに入れるタブの個数
      @bStateFunc          = false # state_sync関数が追加されているか
      @bWaitFinishFunc     = false # wait_finish_sync関数が追加されているか
      @bMigrateFunc        = false # migrate_task関数が追加されているか
      @bChgPriMainTaskFunc = false # chg_pri_main_task関数が追加されているか

      # 全テストシナリオ共通の情報を追加する
      add_common_info()

    end
    attr_reader :aCode, :hProcUnitInfo, :aGlobalVar, :aHeader, :hConfig, :nStack, :hMainTask, :hCheckMain


    #================================================================
    # 概　要: 全テストシナリオ共通の情報を追加する
    #================================================================
    def add_common_info()
      # 基本的なデータ構造の生成
      set_testid_level(IMC_COMMON)
      set_condition_level(IMC_TTG_MAIN)

      # メインタスクと状態参照チェック関数の処理単位情報を生成
      @hMainTask  = {:id => TTG_MAIN_TASK,  :prcid => @cConf.get_main_prcid(), :bootcnt => TTG_MAIN_BOOTCNT}
      @hCheckMain = {:id => FNC_CHECK_MAIN, :prcid => @cConf.get_main_prcid(), :bootcnt => TTG_MAIN_BOOTCNT}

      # コード情報を共通部分に格納するエレメントを用意
      cElement = IMCodeElement.new(:common)

      # メインタスクと状態参照チェック関数を追加
      cElement.set_proc_unit(@hMainTask[:id], @hMainTask[:bootcnt])
      cElement.set_proc_unit(@hCheckMain[:id], @hCheckMain[:bootcnt])

      # プロトタイプ宣言の追加
      cElement.set_header(@hMainTask[:id], TSR_OBJ_TASK)
      cElement.set_header(@hCheckMain[:id], IMC_FUNC_TYPE)

      # 静的API情報の追加
      cElement.set_config("#{API_CRE_TSK}(#{TTG_MAIN_TASK}, {#{KER_TA_ACT}, 1, #{TTG_MAIN_TASK.downcase}, #{TTG_MAIN_PRI}, TTSP_TASK_STACK_SIZE, NULL});", @cConf.get_default_class())

      # ローカル変数の追加
      cElement.set_local_var(@hCheckMain[:id], GRP_VAR_TYPE[TYP_T_TTSP_RTSK], TYP_T_TTSP_RTSK)

      # 一律で時間をとめる場合は，最初に時間停止
      if (!@cConf.is_all_gain_time_mode?())
        cElement.set_code(@hMainTask, "#{FNC_STOP_TICK}()")
      end
      cElement.set_syscall(@hCheckMain, "#{FNC_REF_TSK}(#{TTG_MAIN_TASK}, &#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]})")
      cElement.set_assert(@hCheckMain, "#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]}.#{STR_TSKSTAT}", KER_TTS_RUN)
      cElement.set_assert(@hCheckMain, "#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]}.#{STR_TSKPRI}",  TTG_MAIN_PRI)
      cElement.set_assert(@hCheckMain, "#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]}.#{STR_ITSKPRI}", TTG_MAIN_PRI)
      cElement.set_assert(@hCheckMain, "#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]}.#{STR_ACTCNT}",  0)
      cElement.set_assert(@hCheckMain, "#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]}.#{STR_WUPCNT}",  0)

      # FMPの場合のみ追加する情報
      if (@cConf.is_fmp?())
        cElement.set_assert(@hCheckMain, "#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]}.#{STR_PRCID}", @cConf.get_main_prcid())
      end

      # エレメントを追加する
      add_element(cElement)
    end


    #================================================================
    # 概　要: IMCodeElementを中間コードに変換して保持する
    #================================================================
    def add_element(cElement)
      check_class(IMCodeElement, cElement, true) # IMCodeElementクラスのインスタンス

      if (cElement.nil?())
        return
      end

      aCode      = []    # 生成したコード情報を一時的に保持
      aCodeBlock = []    # ブロック単位でコード情報を保持
      bPassed    = false # バリア同期のシーケンス番号管理フラグ

      cElement.aElement.each{|hElement|
        # 属性によって異なる方法で中間コードを生成して格納する
        # コード情報は格納時の操作が複雑なので生成のみ行う
        case hElement[:atr]
        when :api
          sCode = "#{hElement[:var]} = #{hElement[:syscall]};"
        when :ercd
          sCode = "#{FNC_CHECK_ERCD}(#{hElement[:var]}, #{hElement[:ret]});"
        when :assert
          sCode = "#{FNC_CHECK_ASSERT}(#{hElement[:var]} == #{hElement[:value]});"
        when :check
          sMainPrcid = @cConf.get_main_prcid()
          @aCheckSeqCnt[sMainPrcid] += 1
          sCode = "#{FNC_CHECK_POINT}(#{@aCheckSeqCnt[sMainPrcid]});"
        when :wait
          sMainPrcid = @cConf.get_main_prcid()
          sCode = "#{FNC_WAIT_CHECK_POINT}(#{@aCheckSeqCnt[sMainPrcid]});"
        when :check_mp
          @aCheckSeqCnt[hElement[:proc_unit][:prcid]] += 1
          sCode = "#{FNC_MP_CHECK_POINT}(#{hElement[:proc_unit][:prcid]}, #{@aCheckSeqCnt[hElement[:proc_unit][:prcid]]});"
        when :check_zero
          sCode = "#{FNC_CHECK_POINT}(0);"
        when :check_finish
          sMainPrcid = @cConf.get_main_prcid()
          @aCheckSeqCnt[sMainPrcid] += 1
          sCode = "#{FNC_CHECK_FINISH}(#{@aCheckSeqCnt[sMainPrcid]});"
        when :check_finish_mp
          @aCheckSeqCnt[hElement[:proc_unit][:prcid]] += 1
          sCode = "#{FNC_MP_CHECK_FINISH}(#{hElement[:proc_unit][:prcid]}, #{@aCheckSeqCnt[hElement[:proc_unit][:prcid]]});"
        when :waitcp
          sCode = "#{FNC_MP_WAIT_CHECK_POINT}(#{hElement[:prcid]}, #{@aCheckSeqCnt[hElement[:prcid]]});"
        when :barrier
          # 一つのエレメントで一回だけインクリメント
          if (bPassed != true)
            @nBarrierSeqCnt += 1
            bPassed = true
          end
          sCode = "#{FNC_BARRIER_SYNC}(#{@nBarrierSeqCnt}, #{hElement[:num]});"
        when :comment
          sCode = "/* #{hElement[:code]} */"
        when :expression, :statement
          sCode = "#{hElement[:code]}"
        when :localvar
          add_local_var(hElement)
          next
        when :globalvar
          add_global_var(hElement)
          next
        when :header
          add_header(hElement)
          next
        when :config
          add_config(hElement)
          next
        when :proc_unit
          add_proc_unit(hElement)
          next
        when :delimiter_cmd
          # ブロックを分ける
          aCodeBlock.push(aCode)
          aCode = []
          next
        when :indent_cmd
          @nIndent = hElement[:num]
          next
        when :unindent_cmd
          @nIndent = 0
          next
        when :migrate_func
          add_migrate_task()
          next
        when :chg_pri_main_task
          add_chg_pri_main_task()
          next
        when :precode
          add_pre_code(hElement)
          next
        when :postcode
          add_post_code(hElement)
          next
        else
          abort(ERR_MSG % [__FILE__, __LINE__])
        end

        # インデント命令がオンになっている場合，インデントを行う
        sCode = TTG_TB * @nIndent + sCode

        # コード情報を生成して一時的に保持する
        aCode.push([hElement[:proc_unit][:id], sCode, hElement[:proc_unit][:bootcnt], hElement[:proc_unit][:prcid], hElement[:atr]])
      }

      # ブロックを生成する
      aCodeBlock.push(aCode)

      # 空のブロックを取り除く
      aCodeBlock.delete_if{ |aBlock|
        aBlock.empty?()
      }
      # 一つも残らなければリターン
      if (aCodeBlock.empty?())
        return
      end

      if (cElement.lMode == :block)
        sTestID = get_current_level()
        sCondition = get_current_level(sTestID)

        aCodeBlock.each{ |aBlock|
          @nBlockSeqCnt += 1
          @aCode[TTG_IDX_LAST][sTestID][TTG_IDX_LAST][sCondition].push({IMC_BLOCK + "_#{@nBlockSeqCnt}" => aBlock})
        }
      else
        # commonモードの場合は共通部分にコード情報を格納する
        @aCode[0][IMC_COMMON][0][IMC_TTG_MAIN].concat(aCodeBlock[0])
      end
    end


    #================================================================
    # 概　要: aCodeにテストIDレベルを追加する
    #================================================================
    def set_testid_level(sTestID)
      check_class(String, sTestID) # テストID

      # テストIDレベルを追加する
      @aCode.push({sTestID => []})
    end


    #================================================================
    # 概　要: aCodeにコンディションレベルを追加する
    #================================================================
    def set_condition_level(sCondition)
      check_class(String, sCondition) # Condition or do

      # 現在のテストIDレベルのキーを取得する
      sTestID = get_current_level()
      #コンディションレベルを追加する
      @aCode[TTG_IDX_LAST][sTestID].push({sCondition => []})

    end


    #================================================================
    # 概　要: aCodeの指定したレベルのキー名を取得する
    #================================================================
    def get_current_level(sTestID = nil)
      check_class(String, sTestID, true)    # 現在のテストIDレベル

      if (sTestID.nil?())
        # 現在のテストIDレベルのキー名を返す
        return @aCode[TTG_IDX_LAST].keys[0] # [String]レベルのキー名
      else
        # 現在のコンディションレベルのキー名を返す
        return @aCode[TTG_IDX_LAST][sTestID][TTG_IDX_LAST].keys[0] # [String]レベルのキー名
      end
    end
    private :get_current_level


    #================================================================
    # 概　要: 処理単位情報の追加を行う
    #================================================================
    def add_proc_unit(hElement)
      check_class(Hash, hElement) # 処理単位情報の要素

      @hProcUnitInfo[hElement[:proc_unit_id]] = {:localvar => {}, :fbootcnt => hElement[:fbootcnt], :precode => [], :postcode => []}
    end
    private :add_proc_unit


    #================================================================
    # 概　要: ローカル変数情報を中間表現のデータ構造に変換して保持する
    #================================================================
    def add_local_var(hElement)
      check_class(Hash, hElement) # ローカル変数情報の要素

      # 処理単位が登録されていなければエラー
      unless (@hProcUnitInfo.has_key?(hElement[:proc_unit_id]))
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      @hProcUnitInfo[hElement[:proc_unit_id]][:localvar].update({hElement[:name] => {:type => hElement[:type], :value => hElement[:value]}})
    end
    private :add_local_var


    #================================================================
    # 概　要: グローバル変数情報を中間表現のデータ構造に変換して保持す
    #         る
    #================================================================
    def add_global_var(hElement)
      check_class(Hash, hElement) # グローバル変数情報の要素

      @aGlobalVar.push([hElement[:name], hElement[:type], hElement[:value]])
      @aGlobalVar.uniq!()
    end
    private :add_global_var


    #================================================================
    # 概　要: 処理単位のプロトタイプ宣言を中間表現のデータ構造に変換し
    #         て保持する
    #================================================================
    def add_header(hElement)
      check_class(Hash, hElement) # プロトタイプ宣言情報の要素

      sFuncID = hElement[:proc_unit_id].downcase

      case hElement[:type]
      when TSR_OBJ_TASK, TSR_OBJ_ALARM, TSR_OBJ_CYCLE, TSR_OBJ_ISR, TSR_OBJ_INIRTN, TSR_OBJ_TERRTN
        aArgs = ["#{TYP_INTPTR_T} #{VAR_EXINF}"]
      when TSR_OBJ_TASK_EXC
        aArgs = ["#{TYP_TEXPTN} #{VAR_TEXPTN}", "#{TYP_INTPTR_T} #{VAR_EXINF}"]
      when TSR_OBJ_INTHDR
        aArgs = [TYP_VOID]
      when TSR_OBJ_EXCEPTION
        aArgs = ["#{TYP_VOID_P} #{@cConf.get_exception_arg_name()}"]
      when IMC_FUNC_TYPE # TEST_MAIN，CHECK_MAIN，同期関数
        aArgs = hElement[:args] || []
      else
        # 該当無しエラー
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      @aHeader.push([sFuncID, aArgs])
    end
    private :add_header


    #================================================================
    # 概　要: 静的APIの情報を中間表現のデータ構造に変換して保持する
    #================================================================
    def add_config(hElement)
      check_class(Hash, hElement) # 静的API情報の要素

      if (@cConf.is_asp?())
        sClass = IMC_NO_CLASS
      else
        sClass = hElement[:class]
      end

      if (@hConfig[sClass].nil?())
        @hConfig[sClass] = []
      end

      @hConfig[sClass].push(hElement[:code])
      @hConfig[sClass].uniq!()
    end
    private :add_config


    #================================================================
    # 概　要: 共有するスタックの必要最大数を更新する
    #================================================================
    def update_stack_size(nStack)
      check_class(Integer, nStack) # 必要なスタック数

      if (@nStack < nStack)
        @nStack = nStack
      end
    end


    #================================================================
    # 概　要: migrate_task関数生成のコードをエレメントに追加する
    #================================================================
    def add_migrate_task()
      # 既に中間コード化されていた場合は何もしない
      if (@bMigrateFunc == true)
        return
      end
      @bMigrateFunc = true

      # コード情報を共通部分に格納するエレメントを用意
      cElement = IMCodeElement.new(:common)

      # 処理単位ではないためプロセッサIDとbootcntはダミー
      nMainPrcid = @cConf.get_main_prcid()
      hFuncInfo = {:id => FNC_MIGRATE_TASK, :prcid => nMainPrcid, :bootcnt => TTG_MAIN_BOOTCNT}

      cElement.set_proc_unit(hFuncInfo[:id], hFuncInfo[:bootcnt])
      cElement.set_header(hFuncInfo[:id], IMC_FUNC_TYPE, ["#{TYP_ID} #{VAR_TARGET_TSKID}, #{TYP_ID} #{VAR_PRCID}"])

      # 現在の割付プロセッサが指定したプロセッサでないことを判別するコードを生成する
      cElement.set_local_var(hFuncInfo[:id], GRP_VAR_TYPE[TYP_T_TTSP_RTSK], TYP_T_TTSP_RTSK)
      cElement.set_syscall(hFuncInfo, "#{FNC_REF_TSK}(#{VAR_TARGET_TSKID}, &#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]})")
      cElement.set_code(hFuncInfo, "if (#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]}.#{STR_PRCID} != #{VAR_PRCID}) {", false)
      cElement.set_indent(1)

      # さらに現在の割付プロセッサがメインプロセッサと異なる場合は，メインタスクをmig_tskする
      cElement.set_code(hFuncInfo, "if (#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]}.#{STR_PRCID} != #{nMainPrcid}) {", false)
      cElement.set_indent(2)
      cElement.set_syscall(hFuncInfo, "#{API_MIG_TSK}(#{TTG_TSK_SELF}, #{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]}.#{STR_PRCID})")
      cElement.set_indent(1)
      cElement.set_code(hFuncInfo, "}", false)

      cElement.set_syscall(hFuncInfo, "#{API_MIG_TSK}(#{VAR_TARGET_TSKID}, #{VAR_PRCID})")

      cElement.set_code(hFuncInfo, "if (#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]}.#{STR_PRCID} != #{nMainPrcid}) {", false)
      cElement.set_indent(2)
      cElement.set_syscall(hFuncInfo, "#{API_MIG_TSK}(#{TTG_TSK_SELF}, #{nMainPrcid})")
      cElement.set_indent(1)
      cElement.set_code(hFuncInfo, "}", false)

      cElement.unset_indent()
      cElement.set_code(hFuncInfo, "}", false)

      # エレメントを追加する
      add_element(cElement)
    end
    private :add_migrate_task

    #================================================================
    # 概　要: chg_pri_main_task関数生成のコードをエレメントに追加する
    #================================================================
    def add_chg_pri_main_task()
      # 既に中間コード化されていた場合は何もしない
      if (@bChgPriMainTaskFunc == true)
        return
      end
      @bChgPriMainTaskFunc = true

      # コード情報を共通部分に格納するエレメントを用意
      cElement = IMCodeElement.new(:common)

      # 処理単位ではないためプロセッサIDとbootcntはダミー
      nMainPrcid = @cConf.get_main_prcid()
      hFuncInfo = {:id => FNC_CHG_PRI_MAIN_TASK, :prcid => nMainPrcid, :bootcnt => TTG_MAIN_BOOTCNT}

      cElement.set_proc_unit(hFuncInfo[:id], hFuncInfo[:bootcnt])
      cElement.set_header(hFuncInfo[:id], IMC_FUNC_TYPE)

      # メインタスクの状態を参照するコードを生成する
      cElement.set_local_var(hFuncInfo[:id], GRP_VAR_TYPE[TYP_T_TTSP_RTSK], TYP_T_TTSP_RTSK)
      cElement.set_syscall(hFuncInfo, "#{FNC_REF_TSK}(#{TTG_MAIN_TASK}, &#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]})")

      # メインタスクの優先度が1でない場合は，1へ変更する
      cElement.set_code(hFuncInfo, "if (#{GRP_VAR_TYPE[TYP_T_TTSP_RTSK]}.#{STR_TSKPRI} != #{TTG_MAIN_PRI}) {", false)
      cElement.set_indent(1)
      cElement.set_syscall(hFuncInfo, "#{API_CHG_PRI}(#{TTG_MAIN_TASK}, #{TTG_MAIN_PRI})")
      cElement.unset_indent()
      cElement.set_code(hFuncInfo, "}", false)

      # エレメントを追加する
      add_element(cElement)
    end
    private :add_chg_pri_main_task

    #================================================================
    # 概　要: ASP用gcov_resume関数生成のコードをエレメントに追加する
    #         (FMPと関数名を合わせるためのラッパー)
    #================================================================
    def add_gcov_resume_asp()
      cElement = IMCodeElement.new(:common)
      nMainPrcid = @cConf.get_main_prcid()
      hFuncInfo = {:id => FNC_GCOV_TTG_RESUME, :prcid => nMainPrcid, :bootcnt => TTG_MAIN_BOOTCNT}
      cElement.set_proc_unit(hFuncInfo[:id], hFuncInfo[:bootcnt])
      cElement.set_header(hFuncInfo[:id], IMC_FUNC_TYPE)
      cElement.set_code(hFuncInfo, FNC_GCOV_C_RESUME)
      add_element(cElement)
    end

    #================================================================
    # 概　要: ASP用gcov_pause関数生成のコードをエレメントに追加する
    #================================================================
    def add_gcov_pause_asp()
      cElement = IMCodeElement.new(:common)
      nMainPrcid = @cConf.get_main_prcid()
      hFuncInfo = {:id => FNC_GCOV_TTG_PAUSE, :prcid => nMainPrcid, :bootcnt => TTG_MAIN_BOOTCNT}
      cElement.set_proc_unit(hFuncInfo[:id], hFuncInfo[:bootcnt])
      cElement.set_header(hFuncInfo[:id], IMC_FUNC_TYPE)
      cElement.set_code(hFuncInfo, FNC_GCOV_C_PAUSE)
      add_element(cElement)
    end

    #================================================================
    # 概　要: FMP用gcov_resume関数生成のコードをエレメントに追加する
    #================================================================
    def add_gcov_resume_fmp()
      # コード情報を共通部分に格納するエレメントを用意
      cElement = IMCodeElement.new(:common)

      # 処理単位ではないためプロセッサIDとbootcntはダミー
      nMainPrcid = @cConf.get_main_prcid()
      hFuncInfo = {:id => FNC_GCOV_TTG_RESUME, :prcid => nMainPrcid, :bootcnt => TTG_MAIN_BOOTCNT}

      cElement.set_proc_unit(hFuncInfo[:id], hFuncInfo[:bootcnt])
      cElement.set_header(hFuncInfo[:id], IMC_FUNC_TYPE)

      # グローバル変数定義
      cElement.set_global_var(VAR_GCOV_LOCK_FLG, TYP_BOOL_T, false)

      # ローカル変数定義
      cElement.set_local_var(hFuncInfo[:id], VAR_TIMEOUT, TYP_ULONG_T, 0)

      cElement.set_code(hFuncInfo, SIL_PRE_LOC)

      cElement.set_code(hFuncInfo, "while (1) {", false)
      cElement.set_indent(1)

      cElement.set_code(hFuncInfo, SIL_LOC_SPN)
      cElement.set_code(hFuncInfo, "if (#{VAR_GCOV_LOCK_FLG} == false) {", false)
      cElement.set_indent(2)
      cElement.set_code(hFuncInfo, "#{VAR_GCOV_LOCK_FLG} = true")
      cElement.set_code(hFuncInfo, SIL_UNL_SPN)
      cElement.set_code(hFuncInfo, FNC_GCOV_C_RESUME)
      cElement.set_code(hFuncInfo, "return")
      cElement.set_indent(1)
      cElement.set_code(hFuncInfo, "}", false)
      cElement.set_code(hFuncInfo, SIL_UNL_SPN)

      cElement.set_code(hFuncInfo, "#{VAR_TIMEOUT}++")

      cElement.set_code(hFuncInfo, "if (#{VAR_TIMEOUT} > #{TTG_LOOP_COUNT}) {", false)
      cElement.set_indent(2)
      cElement.set_code(hFuncInfo, "syslog_0(LOG_ERROR, \"## #{FNC_GCOV_TTG_C_RESUME} caused a timeout.\")")
      cElement.set_code(hFuncInfo, "if (#{API_SNS_KER}() == false) {", false)
      cElement.set_indent(3)
      cElement.set_code(hFuncInfo, "#{API_EXT_KER}()")
      cElement.set_indent(2)
      cElement.set_code(hFuncInfo, "}", false)
      cElement.set_code(hFuncInfo, "else {", false)
      cElement.set_indent(3)
      cElement.set_code(hFuncInfo, "return")
      cElement.set_indent(2)
      cElement.set_code(hFuncInfo, "}", false)
      cElement.set_indent(1)
      cElement.set_code(hFuncInfo, "}", false)

      cElement.set_code(hFuncInfo, "sil_dly_nse(#{TTG_SIL_DLY_NSE_TIME})")

      cElement.unset_indent()
      cElement.set_code(hFuncInfo, "}", false)

      # エレメントを追加する
      add_element(cElement)
    end

    #================================================================
    # 概　要: FMP用gcov_pause関数生成のコードをエレメントに追加する
    #================================================================
    def add_gcov_pause_fmp()
      # コード情報を共通部分に格納するエレメントを用意
      cElement = IMCodeElement.new(:common)

      # 処理単位ではないためプロセッサIDとbootcntはダミー
      nMainPrcid = @cConf.get_main_prcid()
      hFuncInfo = {:id => FNC_GCOV_TTG_PAUSE, :prcid => nMainPrcid, :bootcnt => TTG_MAIN_BOOTCNT}

      cElement.set_proc_unit(hFuncInfo[:id], hFuncInfo[:bootcnt])
      cElement.set_header(hFuncInfo[:id], IMC_FUNC_TYPE)

      cElement.set_code(hFuncInfo, "if (#{VAR_GCOV_LOCK_FLG} == false) {", false)
      cElement.set_indent(1)
      cElement.set_code(hFuncInfo, "syslog_0(LOG_ERROR, \"## #{VAR_GCOV_LOCK_FLG} don't be true. [#{FNC_GCOV_TTG_PAUSE}]\")")
      cElement.set_code(hFuncInfo, "if (#{API_SNS_KER}() == false) {", false)
      cElement.set_indent(2)
      cElement.set_code(hFuncInfo, "#{API_EXT_KER}()")
      cElement.set_indent(1)
      cElement.set_code(hFuncInfo, "}", false)
      cElement.set_code(hFuncInfo, "else {", false)
      cElement.set_indent(2)
      cElement.set_code(hFuncInfo, "return")
      cElement.set_indent(1)
      cElement.set_code(hFuncInfo, "}", false)
      cElement.unset_indent()
      cElement.set_code(hFuncInfo, "}", false)

      cElement.set_code(hFuncInfo, FNC_GCOV_C_PAUSE)

      cElement.set_code(hFuncInfo, "#{VAR_GCOV_LOCK_FLG} = false")

      # エレメントを追加する
      add_element(cElement)
    end


    #================================================================
    # 概　要: プレコード情報を中間表現のデータ構造に変換して保持する
    #================================================================
    def add_pre_code(hElement)
      check_class(Hash, hElement) # プレコード情報の要素

      # 処理単位が登録されていなければエラー
      unless (@hProcUnitInfo.has_key?(hElement[:proc_unit]))
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      @hProcUnitInfo[hElement[:proc_unit]][:precode].push(hElement[:code])
    end
    private :add_pre_code


    #================================================================
    # 概　要: ポストコード情報を中間表現のデータ構造に変換して保持する
    #================================================================
    def add_post_code(hElement)
      check_class(Hash, hElement) # ポストコード情報の要素

      # 処理単位が登録されていなければエラー
      unless (@hProcUnitInfo.has_key?(hElement[:proc_unit]))
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      @hProcUnitInfo[hElement[:proc_unit]][:postcode].push(hElement[:code])
    end
    private :add_post_code


    #================================================================
    # 概　要: ASP用時間制御関数生成のコードをエレメントに追加する
    #================================================================
    def add_gain_tick_asp()
      # コード情報を共通部分に格納するエレメントを用意
      cElement = IMCodeElement.new(:common)

      # 処理単位ではないためプロセッサIDとbootcntはダミー
      nMainPrcid = @cConf.get_main_prcid()
      hFuncInfo = {:id => FNC_TARGET_GAIN_TICK, :prcid => nMainPrcid, :bootcnt => TTG_MAIN_BOOTCNT}

      cElement.set_proc_unit(hFuncInfo[:id], hFuncInfo[:bootcnt])
      cElement.set_header(hFuncInfo[:id], IMC_FUNC_TYPE)

      # ローカル変数定義
      cElement.set_local_var(hFuncInfo[:id], VAR_ERCD, TYP_ER)
      cElement.set_local_var(hFuncInfo[:id], VAR_SYSTIM1, TYP_SYSTIM)
      cElement.set_local_var(hFuncInfo[:id], VAR_SYSTIM2, TYP_SYSTIM)
      cElement.set_local_var(hFuncInfo[:id], VAR_TIMEOUT, TYP_ULONG_T, 0)

      # ターゲット依存の時間制御関数によってシステム時刻が進んだことを確認する
      cElement.set_syscall(hFuncInfo, "#{API_GET_TIM}(&#{VAR_SYSTIM1})")
      cElement.set_code(hFuncInfo, "#{FNC_GAIN_TICK}()")
      cElement.set_code(hFuncInfo, "while (1) {", false)
      cElement.set_indent(1)

      cElement.set_syscall(hFuncInfo, "#{API_GET_TIM}(&#{VAR_SYSTIM2})")
      cElement.set_code(hFuncInfo, "if (#{VAR_SYSTIM1} != #{VAR_SYSTIM2}) {", false)
      cElement.set_indent(2)
      cElement.set_code(hFuncInfo, "return")
      cElement.set_indent(1)
      cElement.set_code(hFuncInfo, "}", false)

      cElement.set_code(hFuncInfo, "#{VAR_TIMEOUT}++")

      cElement.set_code(hFuncInfo, "if (#{VAR_TIMEOUT} > #{TTG_LOOP_COUNT}) {", false)
      cElement.set_indent(2)
      cElement.set_code(hFuncInfo, "syslog_0(LOG_ERROR, \"## #{FNC_TARGET_GAIN_TICK}() caused a timeout.\")")
      cElement.set_code(hFuncInfo, "#{API_EXT_KER}()")
      cElement.set_indent(1)
      cElement.set_code(hFuncInfo, "}", false)

      cElement.set_code(hFuncInfo, "sil_dly_nse(#{TTG_SIL_DLY_NSE_TIME})")

      cElement.unset_indent()
      cElement.set_code(hFuncInfo, "}", false)

      # エレメントを追加する
      add_element(cElement)
    end


    #================================================================
    # 概　要: IntermediateCodeをppより見やすい形で整形して出力する
    #================================================================
    def p_code(lOption = nil)
      check_class(Symbol, lOption, true) # 指定オプション

      # コード情報の出力
      puts "--- added CodeInfo ---"
      @aCode.each{ |hScenarios|
        hScenarios.each{ |sTestID, aTestIDLevel|
          puts "#{sTestID} => "
          aTestIDLevel.each{ |hScenario|
            hScenario.each{ |sCondition, aConditionLevel|
              puts "  #{sCondition} => "
              if (sCondition == IMC_TTG_MAIN)
                aConditionLevel.each{ |aCodeInfo|
                  print "      "
                  p aCodeInfo
                }
              else
                aConditionLevel.each{ |hBlocks|
                  hBlocks.each{ |sBlock, aBlockLevel|
                    puts "    #{sBlock} => "
                    aBlockLevel.each{ |aCodeInfo|
                      print "      "
                      p aCodeInfo
                    }
                  }
                }
              end
            }
          }
        }
      }

      if (lOption == :all)
        require "pp"
        puts ""
        print "@hProcUnitInfo = ".ljust(17); pp @hProcUnitInfo
        print "@aGlobalVar = ".ljust(17);    pp @aGlobalVar
        print "@aHeader = ".ljust(17);       pp @aHeader
        print "@hConfig = ".ljust(17);       pp @hConfig
        print "@nStack = ".ljust(17);        pp @nStack
      end
    end

  end
end
