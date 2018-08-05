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
#  上記著作権者は，以下の(1)~(4)の条件を満たす場合に限り，本ソフトウェ
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
#  $Id: ProcessUnit.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "common/bin/Config.rb"
require "common/bin/process_unit/Variable.rb"
require "common/bin/IMCodeElement.rb"
require "ttc/bin/process_unit/ProcessUnit.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: ProcessUnit
  # 概    要: 処理単位オブジェクトの情報を処理するクラス
  #===================================================================
  class ProcessUnit
    include CommonModule

    attr_accessor :hState, :sObjectID, :sObjectType

    #=================================================================
    # 概  要: 処理単位オブジェクトの初期化
    #=================================================================
    def initialize(sObjectID, hObjectInfo, sObjectType, aPath, bIsPre)
      check_class(String, sObjectID)    # オブジェクトID
      check_class(Hash, hObjectInfo)    # オブジェクト情報
      check_class(String, sObjectType)  # オブジェクトタイプ
      check_class(Array, aPath)         # ルートからのパス
      check_class(Bool, bIsPre)         # pre_condition内か

      @cConf                  = Config.new()
      @hState                 = {}
      @aSpecifiedDoAttributes = []

      @sObjectID   = sObjectID
      @sObjectType = sObjectType
      @aPath       = aPath + [@sObjectID]

      @bConvertState = false  # state属性を内部保持用の値に置き換えたか(TESRYでTTS_RUNなど指定された場合にエラー検出できない)

      # 処理単位クラス共通部分
      @hState[TSR_PRM_STATE]   = nil  # 状態[タスク/アラームハンドラ/周期ハンドラ/タスク例外/割込みハンドラ/割込みサービスルーチン]
      @hState[TSR_PRM_LEFTTMO] = nil  # 時間指定[タスク/アラームハンドラ/周期ハンドラ]
      @hState[TSR_PRM_VAR]     = nil  # 変数情報[全処理単位]
      @hState[TSR_PRM_EXINF]   = nil  # 拡張情報[タスク/アラームハンドラ/周期ハンドラ/割込みサービスルーチン/初期化ルーチン/終了ルーチン]
      @hState[TSR_PRM_HDLSTAT] = nil  # アラームハンドラ状態[アラームハンドラ/周期ハンドラ/タスク例外/割込みハンドラ/割込みサービスルーチン/CPU例外ハンドラ]
      @hState[TSR_PRM_ACTPRC]  = nil  # 次回起動時割付けプロセッサID[タスク/アラームハンドラ/周期ハンドラ]
      @hState[TSR_PRM_BOOTCNT] = nil  # 処理単位の起動回数[タスク/アラームハンドラ/周期ハンドラ/タスク例外/割込みハンドラ/割込みサービスルーチン/CPU例外ハンドラ]
      @hState[TSR_PRM_PRCID]   = nil  # プロセッサID[タスク/アラームハンドラ/周期ハンドラ]
      @hState[TSR_PRM_CLASS]   = nil  # クラス[全処理単位]
      @hState[TSR_PRM_INTNO]   = nil  # 割込み番号[割込みハンドラ/割込みサービスルーチン]
      @hState[TSR_PRM_INTPRI]  = nil  # 割込み優先度[割込みハンドラ/割込みサービスルーチン]
      @hState[TSR_PRM_SPINID]  = nil  # スピンロックID[タスク/アラームハンドラ/周期ハンドラ/タスク例外/CPU例外ハンドラ/割込みハンドラ/割込みサービスルーチン]
      @hState[TSR_PRM_ATR]     = nil  # 属性[周期ハンドラ/割込みハンドラ/割込みサービスルーチン]

      # TASK
      @hState[TSR_PRM_ITSKPRI] = nil  # 起動時優先度
      @hState[TSR_PRM_TSKPRI]  = nil  # 現在優先度
      @hState[TSR_PRM_TSKWAIT] = nil  # 待ち要因
      @hState[TSR_PRM_WOBJID]  = nil  # 待ち状態における待ち対象
      @hState[TSR_PRM_ACTCNT]  = nil  # 起動要求キューイング数
      @hState[TSR_PRM_WUPCNT]  = nil  # 起床要求キューイング数
      @hState[TSR_PRM_PORDER]  = nil  # 実行(可能)状態時のレディキュー内での順位

      # CYCLE
      @hState[TSR_PRM_CYCPHS]  = nil  # 位相
      @hState[TSR_PRM_CYCTIM]  = nil  # 周期

      # TASK_EXC
      @hState[TSR_PRM_PNDPTN]  = nil  # タスク例外の保留例外要因パターン
      @hState[TSR_PRM_TEXPTN]  = nil  # タスク例外の例外要因パターン
      @hState[TSR_PRM_TASK]    = nil  # タスク例外関連タスク

      # INTHDR
      @hState[TSR_PRM_INHNO]   = nil  # 割込みハンドラ番号

      # ISR
      @hState[TSR_PRM_ISRPRI]  = nil  # 割込みサービスルーチン優先度

      # EXCEPTION
      @hState[TSR_PRM_EXCNO]   = nil  # CPU例外ハンドラ番号

      # INIRTN，TERRTN
      @hState[TSR_PRM_DO]      = nil  # 初期化/終了ルーチン内で実行する処理
      @hState[TSR_PRM_GLOBAL]  = nil  # グローバル初期化/終了ルーチン内かどうか

      pre_attribute_check(hObjectInfo, aPath + [@sObjectID], bIsPre)
      store_object_info(hObjectInfo)
    end

    #=================================================================
    # 概  要: テストシナリオの通り処理単位オブジェクトデータを
    #         @hStateに代入
    #=================================================================
    def store_object_info(hObjectInfo)
      check_class(Hash, hObjectInfo)  # オブジェクト情報

      # 格納
      set_specified_attribute(hObjectInfo)
      hObjectInfo.each{|atr, val|
        case atr
        when TSR_PRM_TYPE
          # 何もしない

        when TSR_PRM_TSKSTAT, TSR_PRM_ALMSTAT, TSR_PRM_CYCSTAT, TSR_PRM_TEXSTAT, TSR_PRM_INTSTAT
        if (GRP_OBJECT_STATE.has_key?(val))
            @hState[TSR_PRM_STATE] = GRP_OBJECT_STATE[val]
            @bConvertState = true
          else
            @hState[TSR_PRM_STATE] = val
          end

        when TSR_PRM_LEFTTIM, TSR_PRM_LEFTTMO
          @hState[TSR_PRM_LEFTTMO] = val

        when TSR_PRM_VAR
          @hState[atr] = {}
          if (val.is_a?(Hash))
            val.each{|sVarName, var|
              @hState[atr][sVarName] = Variable.new(sVarName, var, @aPath + [TSR_PRM_VAR])
            }
          else
            @hState[atr] = val
          end

        when TSR_PRM_CYCATR, TSR_PRM_INTATR
          @hState[TSR_PRM_ATR] = val

        else
          @hState[atr] = val

        end
      }
    end

    #=================================================================
    # 概  要: ref処理をcElementに追加する
    #=================================================================
    def gc_obj_ref(cElement, hProcUnitInfo, bContext, bCpuLock)
      check_class(IMCodeElement, cElement) # エレメント
      check_class(Hash, hProcUnitInfo)     # 処理単位情報
      check_class(Bool, bContext)          # 未使用(SCObject側との対称性のため)
      check_class(Bool, bCpuLock)          # 未使用(SCObject側との対称性のため)


      cElement.set_local_var(hProcUnitInfo[:id], @sRefStrVar, @sRefStrType)

      # タスク例外の場合，指定するIDは関連するタスクIDとする
      if (@sObjectType == TSR_OBJ_TASK_EXC)
        cElement.set_syscall(hProcUnitInfo, "#{@sRefAPI}(#{@hState[TSR_PRM_TASK]}, &#{@sRefStrVar})")
      else
        cElement.set_syscall(hProcUnitInfo, "#{@sRefAPI}(#{@sObjectID}, &#{@sRefStrVar})")
      end

      # 属性
      if (!@hState[TSR_PRM_ATR].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{@sRefAtr}", @hState[TSR_PRM_ATR])  # CYCLEのみ
      end

      # 状態
      if (!@hState[TSR_PRM_STATE].nil?())
        # スピンロック取得待ちの場合はrefされることがない
        if (@hState[TSR_PRM_STATE] == TSR_STT_R_WAITSPN)
          abort(ERR_MSG % [__FILE__, __LINE__])
        else
          cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{@sRefState}", @hState[TSR_PRM_STATE])
        end
      end

      # 現在優先度(タスクが休止状態の場合は参照しない)
      if (!@hState[TSR_PRM_TSKPRI].nil?() && (@hState[TSR_PRM_STATE] != KER_TTS_DMT))
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_TSKPRI}", @hState[TSR_PRM_TSKPRI])
      end

      # 起動時優先度
      if (!@hState[TSR_PRM_ITSKPRI].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_ITSKPRI}", @hState[TSR_PRM_ITSKPRI])
      end

      # 時間指定
      if (@sObjectType == TSR_OBJ_TASK)
        # タスクが休止状態の場合は参照しない
        if (!@hState[TSR_PRM_LEFTTMO].nil?() && (@hState[TSR_PRM_STATE] != KER_TTS_DMT))
          cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_LEFTTMO}", "#{CST_TMO}#{@hState[TSR_PRM_LEFTTMO]}U")
        end
      else
        # アラームハンドラ，周期ハンドラ
        if (!@hState[TSR_PRM_LEFTTMO].nil?())
          cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_LEFTTIM}", @hState[TSR_PRM_LEFTTMO])
        end
      end

      # レディキュー内での順位(タスクが休止状態の場合は参照しない)
      if (!@hState[TSR_PRM_PORDER].nil?() && (@hState[TSR_PRM_STATE] != KER_TTS_DMT))
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_PORDER}", @hState[TSR_PRM_PORDER])
      end

      # 起動要求キューイング数
      if (!@hState[TSR_PRM_ACTCNT].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_ACTCNT}", @hState[TSR_PRM_ACTCNT])
      end

      # 起床要求キューイング数(タスクが休止状態の場合は参照しない)
      if (!@hState[TSR_PRM_WUPCNT].nil?() && (@hState[TSR_PRM_STATE] != KER_TTS_DMT))
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_WUPCNT}", @hState[TSR_PRM_WUPCNT])
      end

      # 拡張情報(タスク例外の場合は参照できない)
      if (!@hState[TSR_PRM_EXINF].nil?() && (@sObjectType != TSR_OBJ_TASK_EXC))
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{@sRefExInf}", @hState[TSR_PRM_EXINF])
      end

      # プロセッサID(タスク例外ではrefしない)
      if (!@hState[TSR_PRM_PRCID].nil?() && (sObjectType != TSR_OBJ_TASK_EXC))
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{@sRefPrcID}", @hState[TSR_PRM_PRCID])
      end

      # 次回起動時割付けプロセッサID
      if (!@hState[TSR_PRM_ACTPRC].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{@sRefActPrc}", @hState[TSR_PRM_ACTPRC])
      end

      # 待ち対象(タスクが休止状態の場合は参照しない)
      if (!@hState[TSR_PRM_WOBJID].nil?() && (@hState[TSR_PRM_STATE] != KER_TTS_DMT))
        if (@hState[TSR_PRM_WOBJID] == TSR_STT_SLEEP)
          cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_TSKWAIT}", KER_TTW_SLP)
        elsif (@hState[TSR_PRM_WOBJID] == TSR_STT_DELAY)
          cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_TSKWAIT}", KER_TTW_DLY)
        else
          cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_WOBJID}", @hState[TSR_PRM_WOBJID])
        end
      end

      # 位相
      if (!@hState[TSR_PRM_CYCPHS].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_CYCPHS}", @hState[TSR_PRM_CYCPHS])
      end

      # 周期
      if (!@hState[TSR_PRM_CYCTIM].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_CYCTIM}", @hState[TSR_PRM_CYCTIM])
      end

      # 保留例外要因パターン
      if (!@hState[TSR_PRM_PNDPTN].nil?())
        cElement.set_assert(hProcUnitInfo, "#{@sRefStrVar}.#{STR_PNDPTN}", @hState[TSR_PRM_PNDPTN])
      end
    end

    #=================================================================
    # 概  要: 処理単位が持つ変数の値を比較するコードをcElementに格納す
    #         る
    #=================================================================
    def gc_assert_value(cElement, hProcUnitInfo)
      check_class(IMCodeElement, cElement) # エレメント
      check_class(Hash, hProcUnitInfo)     # 処理単位情報

      hState[TSR_PRM_VAR].each{|sValName, cVariable|
        cVariable.gc_assert_value(cElement, hProcUnitInfo)
      }
    end

    #=================================================================
    # 概  要: プロセッサIDを返す
    #=================================================================
    def get_process_id()
      return @hState[TSR_PRM_PRCID] == nil ? 1 : @hState[TSR_PRM_PRCID] # [Integer]プロセッサID
    end

    #=================================================================
    # 概  要: 初期化ルーチンのコードを生成して返す
    #=================================================================
    def gc_ini_rtn(bFirstFlg)
      check_class(Bool, bFirstFlg)  # 最初の初期化ルーチンかどうか

      cElement = IMCodeElement.new()

      # 処理単位定義(bootcntは0固定)
      cElement.set_proc_unit(@sObjectID, TTG_MAIN_BOOTCNT)

      # 自身の処理単位情報作成
      hIniRtnInfo = {:id => @sObjectID, :prcid => @hState[TSR_PRM_PRCID], :bootcnt => TTG_MAIN_BOOTCNT}

      # ASPで先頭の初期化ルーチンの場合，初期化処理を行う
      if ((bFirstFlg == true) && @cConf.is_asp?())
        # テストライブラリ用変数初期化
        cElement.set_code(hIniRtnInfo, FNC_INITIALIZE_TEST_LIB)
      end

      # テストID開始のsyslog出力
      cElement.set_code(hIniRtnInfo, "syslog_0(LOG_NOTICE, \"#{@sObjectID}: Start\")")

      # 初期化ルーチンに必要なすべてのコード作成
      gc_ini_ter_rtn_info(cElement, hIniRtnInfo, bFirstFlg)

      # 初期化ルーチン内にチェックポイントを配置
      cElement.set_checkpoint(hIniRtnInfo)

      # テストID完了のsyslog出力
      cElement.set_code(hIniRtnInfo, "syslog_0(LOG_NOTICE, \"#{@sObjectID}: OK\")")

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 終了ルーチンのチェックポイント以外のコードを生成して返す
    #=================================================================
    def gc_ter_rtn_info()
      cElement = IMCodeElement.new()

      # 処理単位定義(bootcntは0固定)
      cElement.set_proc_unit(@sObjectID, TTG_MAIN_BOOTCNT)

      # 自身の処理単位情報作成
      hTerRtnInfo = {:id => @sObjectID, :prcid => @hState[TSR_PRM_PRCID], :bootcnt => TTG_MAIN_BOOTCNT}

      # エラーが発生している場合は終了ルーチンを実行せずにreturnする
      cElement.set_local_var(hTerRtnInfo[:id], VAR_STATE, TYP_BOOL_T)
      cElement.set_code(hTerRtnInfo, "#{VAR_STATE} = #{FNC_GET_CP_STATE}()")
      cElement.set_code(hTerRtnInfo, "if (#{VAR_STATE} == false) {", false)
      cElement.set_indent(1)
      cElement.set_code(hTerRtnInfo, "return")
      cElement.unset_indent()
      cElement.set_code(hTerRtnInfo, "}")
      cElement.set_block_delimiter()

      # テストID開始のsyslog出力
      cElement.set_code(hTerRtnInfo, "syslog_0(LOG_NOTICE, \"#{@sObjectID}: Start\")")

      # 初期化ルーチンに必要なすべてのコード作成
      gc_ini_ter_rtn_info(cElement, hTerRtnInfo, false)

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 終了ルーチンのチェックポイントのコードを生成して返す
    #=================================================================
    def gc_ter_rtn_checkpoint()
      cElement = IMCodeElement.new()

      # 自身の処理単位情報作成
      hTerRtnInfo = {:id => @sObjectID, :prcid => @hState[TSR_PRM_PRCID], :bootcnt => TTG_MAIN_BOOTCNT}

      cElement.set_checkpoint(hTerRtnInfo)

      # テストID完了のsyslog出力
      cElement.set_code(hTerRtnInfo, "syslog_0(LOG_NOTICE, \"#{@sObjectID}: OK\")")

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: テスト終了メッセージ出力のコードを生成して返す
    #=================================================================
    def gc_finish_message()
      cElement = IMCodeElement.new()

      # 自身の処理単位情報作成
      hTerRtnInfo = {:id => @sObjectID, :prcid => @hState[TSR_PRM_PRCID], :bootcnt => TTG_MAIN_BOOTCNT}

      cElement.set_local_var(hTerRtnInfo[:id], VAR_STATE, TYP_BOOL_T)
      cElement.set_code(hTerRtnInfo, "#{VAR_STATE} = #{FNC_GET_CP_STATE}()")
      cElement.set_code(hTerRtnInfo, "if (#{VAR_STATE} == true) {", false)
      cElement.set_indent(1)

      # GCOVの出力
      # FMPの場合はGCOV専用のグローバル終了ルーチンで出力するため，ASPのみ
      if (@cConf.enable_gcov?() && @cConf.is_asp?())
        cElement.set_code(hTerRtnInfo, FNC_GCOV_C_RESUME)
        cElement.set_code(hTerRtnInfo, FNC_GCOV_C_DUMP)
      end

      if (@cConf.is_asp?())
        cElement.set_code(hTerRtnInfo, "syslog_0(LOG_NOTICE, \"#{TTG_FINISH_MESSAGE}\")")
      else
        cElement.set_code(hTerRtnInfo, "syslog_0(LOG_NOTICE, \"PE #{hTerRtnInfo[:prcid]} : #{TTG_FINISH_MESSAGE}\")")
      end
      cElement.unset_indent()
      cElement.set_code(hTerRtnInfo, "}")

      return cElement # [IMCodeElement] エレメント
    end

    #=================================================================
    # 概  要: 初期化/終了ルーチンに必要なすべてのコード作成
    #=================================================================
    def gc_ini_ter_rtn_info(cElement, hRtnInfo, bFirstFlg)
      check_class(IMCodeElement, cElement) # エレメント
      check_class(Hash, hRtnInfo)          # 処理単位情報
      check_class(Bool, bFirstFlg)         # 初回でない初期化ルーチンかどうか

      # ヘッダ情報
      cElement.set_header(@sObjectID, @sObjectType)

      # コンフィグファイル情報
      cElement.set_config("#{@sCfgAPI}({#{KER_TA_NULL}, #{@hState[TSR_PRM_EXINF]}, #{@sObjectID.downcase}});", @hState[TSR_PRM_CLASS])

      # 拡張情報の参照
      cElement.set_assert(hRtnInfo, VAR_EXINF, @hState[TSR_PRM_EXINF])

      # GCOV取得の開始
      if (@cConf.enable_gcov?())
        # ASPで最初の初期化ルーチンの場合，中断処理を入れる
        if (bFirstFlg == true && @cConf.is_asp?())
          cElement.set_code(hRtnInfo, FNC_GCOV_TTG_C_PAUSE)
        end
        if (@hState[TSR_PRM_DO][TSR_PRM_GCOV] == true)
          cElement.set_code(hRtnInfo, FNC_GCOV_TTG_C_RESUME)
        end
      end

      # 実行するコード
      if (@hState[TSR_PRM_DO].has_key?(TSR_PRM_SYSCALL))
        if (@hState[TSR_PRM_DO].has_key?(TSR_PRM_ERCD))
          cElement.set_syscall(hRtnInfo, @hState[TSR_PRM_DO][TSR_PRM_SYSCALL], @hState[TSR_PRM_DO][TSR_PRM_ERCD])
        elsif (@hState[TSR_PRM_DO].has_key?(TSR_PRM_ERUINT))
          cElement.set_syscall(hRtnInfo, @hState[TSR_PRM_DO][TSR_PRM_SYSCALL], @hState[TSR_PRM_DO][TSR_PRM_ERUINT], TYP_ER_UINT)
        elsif (@hState[TSR_PRM_DO].has_key?(TSR_PRM_BOOL))
          cElement.set_syscall(hRtnInfo, @hState[TSR_PRM_DO][TSR_PRM_SYSCALL], @hState[TSR_PRM_DO][TSR_PRM_BOOL], TYP_BOOL_T)
        else
          cElement.set_syscall(hRtnInfo, @hState[TSR_PRM_DO][TSR_PRM_SYSCALL], nil)
        end
      else
        cElement.set_code(hRtnInfo, @hState[TSR_PRM_DO][TSR_PRM_CODE])
      end

      # GCOV取得の中断
      if (@cConf.enable_gcov?() && @hState[TSR_PRM_DO][TSR_PRM_GCOV] == true)
        cElement.set_code(hRtnInfo, FNC_GCOV_TTG_C_PAUSE)
      end

    end

  end
end
