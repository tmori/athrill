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
#  $Id: ProfileBuilder.rb 6 2012-09-03 11:06:01Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "common/bin/Config.rb"
require "common/bin/IMCodeElement.rb"
require "common/bin/test_scenario/TestScenario.rb"
require "bin/product/IntermediateCode.rb"

module TTG

  #==================================================================
  # クラス名: ProfileBuilder
  # 概　  要: ASP / FMP Builderのスーパークラス
  #==================================================================
  class ProfileBuilder
    include CommonModule

    #================================================================
    # 概　要: コンストラクタ
    #================================================================
    def initialize()
      # 中間コードの初期化処理で全テストシナリオ共通の情報を追加する
      @cIMCode = IntermediateCode.new()

      @cConf = Config.new() # コンフィグを取得
    end

    #================================================================
    # 概  要: テストシナリオから初期化ルーチンのコードを生成する
    #================================================================
    def build_ini_rtn(aIniRtn)
      check_class(Array, aIniRtn) # 初期化ルーチンクラス配列

      # グローバル初期化ルーチンを先頭に持ってくる
      aSortIniRtn = []
      aIniRtn.each{|cIniRtn|
        if (cIniRtn.hState[TSR_PRM_GLOBAL] == true)
          aSortIniRtn.push(cIniRtn)
        end
      }

      # グローバル初期化ルーチン以外を後に入れる
      aIniRtn.each{|cIniRtn|
        if (cIniRtn.hState[TSR_PRM_GLOBAL] != true)
          aSortIniRtn.push(cIniRtn)
        end
      }

      bFirstFlg = true
      aSortIniRtn.each{|cIniRtn|
        # 初期化ルーチン単体のテストIDを定義
        @cIMCode.set_testid_level("#{IMC_TTG_INIRTN}_#{cIniRtn.sObjectID}")

        # 初期化ルーチンのコンディションレベル作成
        @cIMCode.set_condition_level(TTG_LBL_INIRTN)

        # 初期化ルーチンに必要なすべてのコード生成
        @cIMCode.add_element(cIniRtn.gc_ini_rtn(bFirstFlg))

        # 初回のみtrueとする
        bFirstFlg = false
      }
    end

    #================================================================
    # 概  要: テストシナリオから終了ルーチンのコードを生成する
    #================================================================
    def build_ter_rtn(aTerRtn)
      check_class(Array, aTerRtn) # 終了ルーチンクラス配列

      # グローバル終了ルーチンを先頭に持ってくる
      aSortTerRtn = []
      aTerRtn.each{|cTerRtn|
        if (cTerRtn.hState[TSR_PRM_GLOBAL] == true)
          aSortTerRtn.push(cTerRtn)
        end
      }

      # グローバル終了ルーチン以外を後に入れる
      aTerRtn.each{|cTerRtn|
        if (cTerRtn.hState[TSR_PRM_GLOBAL] != true)
          aSortTerRtn.push(cTerRtn)
        end
      }

      aSortTerRtn.each{|cTerRtn|
        # 終了ルーチン単体のテストIDを定義
        @cIMCode.set_testid_level("#{IMC_TTG_TERRTN}_#{cTerRtn.sObjectID}")

        # 終了ルーチンのコンディションレベル作成
        @cIMCode.set_condition_level(TTG_LBL_TERRTN)

        # 終了ルーチンに必要なチェックポイント以外のコード生成
        @cIMCode.add_element(cTerRtn.gc_ter_rtn_info())
      }

      # チェックポイントを逆順で出力する
      aSortTerRtn.reverse.each{|cTerRtn|
        @cIMCode.add_element(cTerRtn.gc_ter_rtn_checkpoint())
      }

      # 最初の終了ルーチンにテスト終了メッセージ出力(逆順で実行されるため)
      @cIMCode.add_element(aSortTerRtn[0].gc_finish_message())
    end

    #================================================================
    # 概  要: テストシナリオの初期化ルーチンの有無を返す
    #================================================================
    def exist_ini_rtn(cTestScenario)
      check_class(TestScenario, cTestScenario) # テストシナリオクラス

      return cTestScenario.exist_ini_rtn()  # [Bool]初期化ルーチンの有無
    end

    #================================================================
    # 概  要: テストシナリオの終了ルーチンの有無を返す
    #================================================================
    def exist_ter_rtn(cTestScenario)
      check_class(TestScenario, cTestScenario) # テストシナリオクラス

      return cTestScenario.exist_ter_rtn()  # [Bool]終了ルーチンの有無
    end

    #================================================================
    # 概  要: テストシナリオから中間コードを生成する
    #================================================================
    def build(cTestScenario, bPrevGainTime)
      check_class(TestScenario, cTestScenario) # テストシナリオクラス
      check_class(Bool, bPrevGainTime)         # 前のテストシナリオにおける時間動作フラグ

      @cTS = cTestScenario.dup()
      @hTestMain = @cTS.get_main_task_proc() # メインタスクID取得

      # テストシナリオ開始処理
      start_test_scenario(bPrevGainTime)

      # pre_conditionの設定
      build_pre_condition()

      # do，post_conditionの設定
      @cTS.get_do_post_index().each{|nIndex|
        # doの処理
        build_do(nIndex)

        # post_conditionの処理
        if (@cTS.check_lastpost_condition(nIndex) == true)
          build_lastpost_condition(nIndex)
        else
          build_post_condition(nIndex)
        end
      }
    end

    #================================================================
    # 概　要: テストシナリオ開始処理
    #================================================================
    def start_test_scenario(bPrevGainTime)
      check_class(Bool, bPrevGainTime)  # 前のテストシナリオにおける時間動作フラグ

      hMainTask  = @cIMCode.hMainTask
      hCheckMain = @cIMCode.hCheckMain

      # このテストシナリオに登場する処理単位の追加
      # 最終的な起動回数の設定も同時に行う
      cElement = IMCodeElement.new(:common)
      cElement.set_proc_unit(@hTestMain[:id], @hTestMain[:bootcnt])
      @cTS.get_proc_units(cElement)


      # グローバル/ローカル変数宣言の追加(処理単位にTSR_PRM_VARが定義されているもの)
      @cTS.gc_global_local_var(cElement)


      # プロトタイプ宣言の追加
      cElement.set_header(@hTestMain[:id], IMC_FUNC_TYPE)
      @cTS.gc_header(cElement)


      # 静的API情報の追加
      @cTS.gc_config(cElement)


      # テストID開始のsyslog出力
      cElement.set_code(hMainTask, "#{TTG_NL}#{TTG_TB}syslog_0(LOG_NOTICE, \"#{@cTS.sTestID}: Start\")")


      # 時間動作に関して変化が無い場合，何もしない
      if (@cTS.bGainTime == bPrevGainTime)
      # 今回時間を動かす場合(=前のテストシナリオでは時間停止)，時間を動かす
      elsif (@cTS.bGainTime == true)
        cElement.set_code(hMainTask, "#{FNC_START_TICK}()")
      # 今回時間を動かさない場合(=前のテストシナリオでは時間動作)，時間を止める
      else
        cElement.set_code(hMainTask, "#{FNC_STOP_TICK}()")
      end


      # ACTIVATEである割込みハンドラがある場合，グローバル変数を定義，およびインクリメント
      if (@cTS.exist_int_hdr() == true)
        aIntHdrID = @cTS.get_activate_proc_unit_id(TSR_OBJ_INTHDR)
        aIntHdrID.each{|sObjectID|
          cElement.set_global_var("#{sObjectID}_#{VAR_BOOTCNT}", "enum ENUM_#{sObjectID}", "#{sObjectID}_#{TTG_ENUM_INVALID}")
          cElement.set_code(hMainTask, "#{sObjectID}_#{VAR_BOOTCNT}++")
        }
      end

      # ACTIVATEである割込みサービスルーチンがある場合，グローバル変数を定義，およびインクリメント
      # (割込みサービスルーチンの場合，1つのグローバルIPだけでよい)
      if (@cTS.exist_isr() == true)
        aIsrID = @cTS.get_activate_proc_unit_id(TSR_OBJ_ISR)
        if (!aIsrID.empty?())
          cElement.set_global_var("#{TTG_LBL_ISR}_#{VAR_BOOTCNT}", "enum ENUM_#{TTG_LBL_ISR}", "#{TTG_LBL_ISR}_#{TTG_ENUM_INVALID}")
          cElement.set_code(hMainTask, "#{TTG_LBL_ISR}_#{VAR_BOOTCNT}++")
        end
      end

      # ACTIVATEであるCPU例外ハンドラがある場合，グローバル変数を定義，およびインクリメント
      if (@cTS.exist_exception() == true)
        aExcID = @cTS.get_activate_proc_unit_id(TSR_OBJ_EXCEPTION)
        aExcID.each{|sObjectID|
          cElement.set_global_var("#{sObjectID}_#{VAR_BOOTCNT}", "enum ENUM_#{sObjectID}", "#{sObjectID}_#{TTG_ENUM_INVALID}")
          cElement.set_code(hMainTask, "#{sObjectID}_#{VAR_BOOTCNT}++")
        }
      end


      # TA_STA属性の周期ハンドラがある場合，グローバル変数を定義，フラグをセット、
      if (@cTS.exist_cyclic_sta() == true)
        hCycle = @cTS.get_all_object_info(TSR_OBJ_CYCLE)
        hCycle.each{|sObjectID, hObjectInfo|
          if (hObjectInfo.hState[TSR_PRM_ATR] == KER_TA_STA)
            cElement.set_global_var("#{sObjectID}_flg", TYP_BOOL_T, false)
            cElement.set_code(hMainTask, "#{sObjectID}_flg = true")

            # 対象周期ハンドラの先頭で周期ハンドラが起動しないようreturn処理を行う
            cElement.set_pre_code(sObjectID, "if (#{sObjectID}_flg == false) {", false)
            cElement.set_pre_code(sObjectID, "#{TTG_TB}return")
            cElement.set_pre_code(sObjectID, "}", false)
          end
        }
      end


      # GCOV全取得の場合，ここでresumeする
      if (@cConf.enable_gcov?() && (@cTS.bGcovAll == true))
        cElement.set_code(hMainTask, FNC_GCOV_TTG_C_RESUME)
      end

      cElement.set_code(hMainTask, "#{@hTestMain[:id].downcase()}()")

      # GCOV全取得の場合，ここでpauseする
      if (@cConf.enable_gcov?() && (@cTS.bGcovAll == true))
        cElement.set_code(hMainTask, FNC_GCOV_TTG_C_PAUSE)
      end


      # メインタスクの状態チェック処理
      cElement.set_code(hMainTask, "#{hCheckMain[:id]}()")

      # テストID完了のsyslog出力
      cElement.set_code(hMainTask, "syslog_0(LOG_NOTICE, \"#{@cTS.sTestID}: OK\")")
      @cIMCode.add_element(cElement)

      # テストIDレベルを追加する
      @cIMCode.set_testid_level(@cTS.sTestID)

      # 必要なタスクスタックの個数をアップデート
      @cIMCode.update_stack_size(@cTS.get_stack_num())
    end

    #================================================================
    # 概　要: checkfinishのコードを生成する
    #================================================================
    def build_finish_cp(bIsTerRtn)
      check_class(Bool, bIsTerRtn) # 終了ルーチン有無フラグ

      cElement = IMCodeElement.new(:common)

      # 終了ルーチンが無ければ，check_finish
      if (bIsTerRtn == false)
        # GCOVの出力
        if (@cConf.enable_gcov?() && @cConf.is_asp?())
          cElement.set_code(@cIMCode.hMainTask, "#{TTG_NL}#{TTG_TB}#{FNC_GCOV_C_RESUME}")
          cElement.set_code(@cIMCode.hMainTask, FNC_GCOV_C_DUMP)
        end
        cElement.set_checkfinish(@cIMCode.hMainTask)
      # 終了ルーチンがあれば，check_point
      else
        cElement.set_checkpoint(@cIMCode.hMainTask)
        cElement.set_code(@cIMCode.hMainTask, "#{API_EXT_KER}()")
      end

      @cIMCode.add_element(cElement)
    end

    #================================================================
    # 概  要: 関数を共有する処理単位の初期処理を行うコードを生成する
    #================================================================
    def build_shared_proc_unit(hSharedProcUnitInfo, sObjectType)
      check_class(Hash, hSharedProcUnitInfo) # 関数を共有する処理単位の情報
      check_class(String, sObjectType)       # 対象とする処理単位タイプ

      cElement = IMCodeElement.new()

      aEnaIntIntHdr = []

      # 対象とする処理単位に必要な情報を定義
      hSharedProcUnitInfo.sort.each{|aObjectInfo|
        # 処理単位情報追加
        cElement.set_proc_unit(aObjectInfo[0], aObjectInfo[1].hState[TSR_PRM_BOOTCNT])

        # プロトタイプ宣言の追加
        cElement.set_header(aObjectInfo[0], sObjectType)

        # 静的API情報の追加
        aObjectInfo[1].gc_config(cElement)

        # 割込みハンドラは一律先頭で割込み要求フラグクリア関数(開始時処理)を実行する
        if (sObjectType == TSR_OBJ_INTHDR)
          cElement.set_pre_code(aObjectInfo[0], "#{FNC_I_BEGIN_INT}(#{aObjectInfo[1].hState[TSR_PRM_INTNO]})")
          # 明示的な割込み要求クリア処理
          # (個々のデバイスに対する割込み要求のクリア処理/ターゲット依存)
          cElement.set_pre_code(aObjectInfo[0], "#{FNC_CLEAR_INT_REQ}(#{aObjectInfo[1].hState[TSR_PRM_INTNO]})")

          # 割込みハンドラは一律先頭で割込み要求フラグクリア関数(終了時処理)を実行する
          cElement.set_post_code(aObjectInfo[0], "#{FNC_I_END_INT}(#{aObjectInfo[1].hState[TSR_PRM_INTNO]})")

          # TA_ENAINT属性の割込みハンドラの情報を格納しておく
          if (aObjectInfo[1].hState[TSR_PRM_ATR] == KER_TA_ENAINT)
            hProcUnitInfo = {:id => aObjectInfo[0], :prcid => aObjectInfo[1].hState[TSR_PRM_PRCID], :bootcnt => TTG_MAIN_BOOTCNT}
            aEnaIntIntHdr.push([hProcUnitInfo, aObjectInfo[0], aObjectInfo[1].hState[TSR_PRM_INTNO], aObjectInfo[1].hState[TSR_PRM_PRCID]])
          end
        # CPU例外ハンドラは一律先頭でフック処理関数を実行する
        elsif (sObjectType == TSR_OBJ_EXCEPTION)
          cElement.set_pre_code(aObjectInfo[0], "#{FNC_CPUEXC_HOOK}(#{aObjectInfo[1].hState[TSR_PRM_EXCNO]}, #{@cConf.get_exception_arg_name()})")
        end
      }

      @cIMCode.add_element(cElement)

      # TA_ENAINT属性の割込みハンドラがある場合，先頭で割込みハンドラの起動確認を行い，dis_intする
      aEnaIntIntHdr.each{|aIntHdr|
        cElementCommon = IMCodeElement.new(:common)
        cElementCommon.set_global_var("#{aIntHdr[1]}_#{VAR_BOOTCNT}", "enum ENUM_#{aIntHdr[1]}", "#{aIntHdr[1]}_#{TTG_ENUM_INVALID}")
        cElementCommon.set_code(@cIMCode.hMainTask, "#{TTG_NL}#{TTG_TB}#{aIntHdr[1]}_#{VAR_BOOTCNT}++")
        build_int_raise(cElementCommon, aIntHdr[2], aIntHdr[3])
        cElementCommon.set_syscall(@cIMCode.hMainTask, "#{API_SLP_TSK}()")
        if (@cConf.get_api_support_dis_int() == true)
          build_dis_int(cElementCommon, aIntHdr[2], aIntHdr[3])
        end
        cElementCommon.set_checkpoint(@cIMCode.hMainTask)
        @cIMCode.add_element(cElementCommon)

        @cIMCode.set_testid_level("#{TTG_LBL_CHK_ENAINT}")
        @cIMCode.set_condition_level("#{TTG_LBL_CHK_ENAINT}")
        cElementIntHdr = IMCodeElement.new()
        cElementIntHdr.set_syscall(aIntHdr[0], "#{API_IWUP_TSK}(#{TTG_MAIN_TASK})")
        @cIMCode.add_element(cElementIntHdr)
      }
    end

    #================================================================
    # 概  要: メインタスクでGCOV取得を開始する
    #================================================================
    def build_main_task_gcov_start()
      # サブクラスでオーバーライドして利用する
      abort(ERR_MSG % [__FILE__, __LINE__])
    end

    #================================================================
    # 概  要: メインタスクから割込みを発生させる
    #================================================================
    def build_int_raise()
      # サブクラスでオーバーライドして利用する
      abort(ERR_MSG % [__FILE__, __LINE__])
    end

    #================================================================
    # 概  要: メインタスクでdis_intを実行する
    #================================================================
    def build_dis_int()
      # サブクラスでオーバーライドして利用する
      abort(ERR_MSG % [__FILE__, __LINE__])
    end

    #================================================================
    # 概　要: TA_STA属性の周期ハンドラへstp_cycを発行するコードを生成する
    #================================================================
    def build_stp_cyc(hCyclicStaInfo)
      check_class(Hash, hCyclicStaInfo) # TA_STA属性の周期ハンドラ情報ハッシュ

      cElement = IMCodeElement.new(:common)

      hCyclicStaInfo.each{|sObjectID, cObjectInfo|
        # メインタスクの先頭で属性確認とハンドラ停止処理を行う
        cObjectInfo.gc_obj_ref_only_cyc(cElement, @cIMCode.hMainTask);
        cElement.set_syscall(@cIMCode.hMainTask, "#{API_STP_CYC}(#{sObjectID})")
        cElement.set_checkpoint(@cIMCode.hMainTask)
      }

      @cIMCode.add_element(cElement)
    end

    #================================================================
    # 概　要: TA_ENAINT属性の割込みサービスルーチンが割りつけられてい
    #         る割込み番号に対して起動確認用の割込みサービスルーチンを
    #         起動し，dis_intする
    #         (割込みハンドラはbuild_shared_proc_unit内で実施)
    #================================================================
    def build_ena_int_isr(hEnaIntIsrInfo)
      check_class(Hash, hEnaIntIsrInfo) # TA_ENAINT属性の割込みサービスルーチンの情報

      aEnaIntIsr = []
      aChkIsr = []

      # 対象とする処理単位に必要な情報を定義
      hEnaIntIsrInfo.each{|sObjectID, cObjectInfo|
        # 同じ割込み番号は除外
        if (!aChkIsr.include?(cObjectInfo.hState[TSR_PRM_INTNO]))
          hTempObject = cObjectInfo.dup
          hTempObject.sObjectID = "#{TTG_LBL_ISR}_#{cObjectInfo.hState[TSR_PRM_INTNO]}_#{TTG_LBL_CHK_ENAINT}";
          hTempObject.hState[TSR_PRM_ISRPRI] = 1;
          aEnaIntIsr.push(hTempObject)
        end

        aChkIsr.push(cObjectInfo.hState[TSR_PRM_INTNO])
      }

      # 起動確認用の割込みサービスルーチンを登録・起動し，dis_intする
      aEnaIntIsr.each{|cEnaIntIsr|
        cElementCommon = IMCodeElement.new(:common)
        cElementCommon.set_global_var("#{TTG_LBL_ISR}_#{VAR_BOOTCNT}", "enum ENUM_#{TTG_LBL_ISR}", "#{TTG_LBL_ISR}_#{TTG_ENUM_INVALID}")
        cElementCommon.set_code(@cIMCode.hMainTask, "#{TTG_NL}#{TTG_TB}#{TTG_LBL_ISR}_#{VAR_BOOTCNT}++")
        build_int_raise(cElementCommon, cEnaIntIsr.hState[TSR_PRM_INTNO], cEnaIntIsr.hState[TSR_PRM_PRCID])
        cElementCommon.set_syscall(@cIMCode.hMainTask, "#{API_SLP_TSK}()")
        if (@cConf.get_api_support_dis_int() == true)
          build_dis_int(cElementCommon, cEnaIntIsr.hState[TSR_PRM_INTNO], cEnaIntIsr.hState[TSR_PRM_PRCID])
        end
        cElementCommon.set_checkpoint(@cIMCode.hMainTask)
        @cIMCode.add_element(cElementCommon)

        @cIMCode.set_testid_level("#{cEnaIntIsr.hState[TSR_PRM_INTNO]}_#{TTG_LBL_CHK_ENAINT}")
        @cIMCode.set_condition_level("#{cEnaIntIsr.hState[TSR_PRM_INTNO]}_#{TTG_LBL_CHK_ENAINT}")
        cElementIntIsr = IMCodeElement.new()
        cElementIntIsr.set_proc_unit(cEnaIntIsr.sObjectID, cEnaIntIsr.hState[TSR_PRM_BOOTCNT])
        cElementIntIsr.set_header(cEnaIntIsr.sObjectID, TSR_OBJ_ISR)
        cEnaIntIsr.gc_config(cElementIntIsr)
        hProcUnitInfo = {:id => cEnaIntIsr.sObjectID, :prcid => cEnaIntIsr.hState[TSR_PRM_PRCID], :bootcnt => TTG_MAIN_BOOTCNT}
        cElementIntIsr.set_syscall(hProcUnitInfo, "#{API_IWUP_TSK}(#{TTG_MAIN_TASK})")
        @cIMCode.add_element(cElementIntIsr)
      }

    end

    #================================================================
    # 概　要: メインタスクの先頭でcp_stateをチェックするコードを生成する
    #================================================================
    def build_check_cp_state()
      cElement = IMCodeElement.new(:common)

      cElement.set_local_var(@cIMCode.hMainTask[:id], VAR_STATE, TYP_BOOL_T)
      cElement.set_code(@cIMCode.hMainTask, "#{TTG_NL}#{TTG_TB}#{VAR_STATE} = #{FNC_GET_CP_STATE}()")
      cElement.set_code(@cIMCode.hMainTask, "if (#{VAR_STATE} == false) {", false)
      cElement.set_indent(1)
      cElement.set_code(@cIMCode.hMainTask, "#{API_EXT_KER}()")
      cElement.unset_indent()
      cElement.set_code(@cIMCode.hMainTask, "}")

      @cIMCode.add_element(cElement)
    end

    #================================================================
    # 概　要: メインタスクの先頭で各初期化処理を行うコードを生成する
    #================================================================
    def build_initialize()
      cElement = IMCodeElement.new(:common)

      # テストライブラリ用変数初期化
      cElement.set_code(@cIMCode.hMainTask, FNC_INITIALIZE_TEST_LIB)

      @cIMCode.add_element(cElement)
    end

    #================================================================
    # 概　要: 生成した中間コードを返す
    #================================================================
    def get_result()
      return @cIMCode  # [IntermediateCode]中間コード
    end

  end
end
