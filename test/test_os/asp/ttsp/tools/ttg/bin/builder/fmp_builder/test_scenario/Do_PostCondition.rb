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
#  $Id: Do_PostCondition.rb 14 2012-11-05 09:28:16Z nces-shigihara $
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
    # 概  要: Doの処理をIMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_do_fmp(aPrevObjectInfo)
      check_class(Array, aPrevObjectInfo)  # 前のコンディションのオブジェクト情報

      cElement        = IMCodeElement.new()
      cCallObjectInfo = nil
      # idに記載したIDと一致している処理単位を取得する
      aPrevObjectInfo.each{ |cObjectInfo|
        if (!cObjectInfo.nil?())
          if (cObjectInfo.sObjectID == @hDo[TSR_PRM_ID])
            cCallObjectInfo = cObjectInfo
          end
        end
      }
      if (cCallObjectInfo.nil?())
        abort(ERR_MSG % [__FILE__, __LINE__])
      end

      hProcUnitInfo = get_proc_unit_info(cCallObjectInfo)

      # コメント出力
      cElement.set_comment(hProcUnitInfo, "#{TSR_UNS_DO}#{@nSeqNum}_#{@nTimeTick}")
      # GCOV取得の開始もしくは再開
      gc_gcov_resume(cElement, hProcUnitInfo)
      # 対象の処理単位が非タスクで同期処理がない場合に備え，
      # resume/pauseの順序同期のためのチェックポイントを発行しておく
      if (@cConf.enable_gcov?() && @hDo[TSR_PRM_GCOV] == true)
        cElement.set_checkpoint(hProcUnitInfo)
      end

      # syscall部
      bIsReturn = true
      if (@hDo.has_key?(TSR_PRM_SYSCALL))
        if (@hDo.has_key?(TSR_PRM_ERCD))
          cElement.set_syscall(hProcUnitInfo, @hDo[TSR_PRM_SYSCALL], @hDo[TSR_PRM_ERCD])
        elsif (@hDo.has_key?(TSR_PRM_ERUINT))
          cElement.set_syscall(hProcUnitInfo, @hDo[TSR_PRM_SYSCALL], @hDo[TSR_PRM_ERUINT], TYP_ER_UINT)
        elsif (@hDo.has_key?(TSR_PRM_BOOL))
          cElement.set_syscall(hProcUnitInfo, @hDo[TSR_PRM_SYSCALL], @hDo[TSR_PRM_BOOL], TYP_BOOL_T)
        else
          cElement.set_syscall(hProcUnitInfo, @hDo[TSR_PRM_SYSCALL], nil)
          bIsReturn = false
        end
      else
        # "#"から始まるコードはセミコロンを付与しない
        bSemicolon = true
        if (@hDo[TSR_PRM_CODE] =~ /^\#.*/)
          bSemicolon = false
        end
        cElement.set_code(hProcUnitInfo, @hDo[TSR_PRM_CODE], bSemicolon)
        bIsReturn = false
      end

      # GCOV取得の中断
      # 戻り値がある場合，いずれ実行状態になるためAPIを発行した処理単位で行う
      @nGcovAfterSyncFlg = false
      if (bIsReturn == true)
        gc_gcov_pause(cElement, hProcUnitInfo)
      # resumeを発行したプロセッサに次のコンディションで実行中の処理単位がいれば
      # その処理単位でGCOVを中断する
      elsif (!@aActivate[hProcUnitInfo[:prcid]].nil?() || !@aRunning[hProcUnitInfo[:prcid]].nil?())
        gc_gcov_pause(cElement, get_proc_unit_info(@aActivate[hProcUnitInfo[:prcid]], @aRunning[hProcUnitInfo[:prcid]]))
      else
        # 戻り値が無く実行状態の処理単位もいない場合，同期処理の後にメインプロセッサで実行中の処理単位から行う
        # ただし，対象の処理単位が非タスクで同期処理がない場合に備え，
        # resume/pauseの順序同期のためのチェックポイント待ちとしておく
        if (@cConf.enable_gcov?() && @hDo[TSR_PRM_GCOV] == true)
          cElement.set_wait_check_sync(get_proc_unit_info(@cActivate, @cRunning), hProcUnitInfo[:prcid])
          # 同期後にpauseする必要があるかを保持しておく
          @nGcovAfterSyncFlg = true
        end
      end

      return cElement  # [IMCodeElement]Doの処理
    end

    #=================================================================
    # 概  要: 同期処理後のgcov_pause用コードを生成して返す
    #=================================================================
    def gc_gcov_pause_after_sync()
      cElement = IMCodeElement.new()

      # pauseを実行していない場合，同期処理の後で実行中の処理単位から行う
      if (@nGcovAfterSyncFlg == true)
        gc_gcov_pause(cElement, get_proc_unit_info(@cActivate, @cRunning))
      end

      return cElement  # [IMCodeElement]同期処理後のgcov_pause用コード
    end

    #=================================================================
    # 概  要: ディスパッチ禁止状態を解除する処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_lastpost_ena_dsp_fmp()
      cElement      = IMCodeElement.new()
      @aPrcid.each{ |nPrcid|
        # check処理も兼ねて行う
        if (!@aCpuState[nPrcid].nil?() && !@aCpuState[nPrcid].hState[TSR_PRM_DISDSP].nil?() && (@aCpuState[nPrcid].hState[TSR_PRM_DISDSP] == true))
          hProcUnitInfo = get_proc_unit_info(@aRunning[nPrcid])
          cElement.set_syscall(hProcUnitInfo, "#{API_ENA_DSP}()")
        end
      }

      return cElement  # [IMCodeElement]ディスパッチ禁止状態を解除する処理
    end

    #=================================================================
    # 概  要: 割込み優先度マスクを初期化させる処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_lastpost_set_ini_ipm_fmp()
      cElement      = IMCodeElement.new()
      @aPrcid.each{ |nPrcid|
        # check処理も兼ねて行う
        if (!@aCpuState[nPrcid].nil?() && !@aCpuState[nPrcid].hState[TSR_PRM_CHGIPM].nil?() && 
            (@aCpuState[nPrcid].hState[TSR_PRM_CHGIPM] != 0) && (@aCpuState[nPrcid].hState[TSR_PRM_CHGIPM] != KER_TIPM_ENAALL))
          hProcUnitInfo = get_proc_unit_info(@aRunning[nPrcid])
          cElement.set_syscall(hProcUnitInfo, "#{API_CHG_IPM}(#{KER_TIPM_ENAALL})")
        end
      }

      return cElement  # [IMCodeElement]割込み優先度マスクを初期化させる処理
    end

    #=================================================================
    # 概  要: スピンロック解放の処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_lastpost_spin_unl_fmp()
      cElement = IMCodeElement.new()

      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((cObjectInfo.sObjectType == TSR_OBJ_SPINLOCK) && (cObjectInfo.hState[TSR_PRM_SPNSTAT] == TSR_STT_TSPN_LOC))
          cObject = get_object_info(cObjectInfo.hState[TSR_PRM_PROCID])
          hProcUnitInfo = get_proc_unit_info(cObject)
          if (GRP_NON_CONTEXT.include?(cObject.sObjectType))
            cElement.set_syscall(hProcUnitInfo, "#{API_IUNL_SPN}(#{sObjectID})")
          else
            cElement.set_syscall(hProcUnitInfo, "#{API_UNL_SPN}(#{sObjectID})")
          end

          # CPUロック解除を重複して実行しないため，保持しておく
          @aSpinProcID.push(cObject.sObjectID)
        end
      }

      return cElement  # [IMCodeElement]スピンロック解放の処理
    end

    #=================================================================
    # 概  要: CPUロック状態の解除の処理を
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_lastpost_cpu_unl_fmp()
      cElement = IMCodeElement.new()
      @aPrcid.each{ |nPrcid|
        # check処理も兼ねて行う
        unless (@aCpuState[nPrcid].nil?() || @aCpuState[nPrcid].hState[TSR_PRM_LOCCPU].nil?() || (@aCpuState[nPrcid].hState[TSR_PRM_LOCCPU] != true))
          hProcUnitInfo = get_proc_unit_info(@aActivate[nPrcid], @aRunning[nPrcid])
          # スピンロックによるCPUロックは除外
          if (!@aSpinProcID.include?(hProcUnitInfo[:id]))
            if (!@aActivate[nPrcid].nil?())
              cElement.set_syscall(hProcUnitInfo, "#{API_IUNL_CPU}()")
            else
              cElement.set_syscall(hProcUnitInfo, "#{API_UNL_CPU}()")
            end
          end
        end
      }

      return cElement  # [IMCodeElement]CPUロック状態の解除の処理
    end

    #=================================================================
    # 概  要: 全タスクを起動終了させるコードを
    #         IMCodeElementクラスにまとめて返す
    #=================================================================
    def gc_lastpost_all_task_ter_fmp()
      # 他タスクが存在する場合，すべてter_tsk()させる
      cElement = IMCodeElement.new()

      hProcUnitInfo = get_proc_unit_info()
      nMainPrcid = @sMainPrcid

      # メインタスクがプリエンプトされることを防ぐために
      # 実行可能状態のタスクを強制待ち状態にしておく
      @aTask.each{|nPrcid, sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RDY)
          cElement.set_syscall(hProcUnitInfo, "#{API_SUS_TSK}(#{sObjectID})")
        end
      }
      # 実行状態のタスクを強制待ち状態にしておく
      @aTask.each{|nPrcid, sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RUN)
          cElement.set_syscall(hProcUnitInfo, "#{API_SUS_TSK}(#{sObjectID})")
        end
      }

      # 強制待ちにされるまで待つ
      # 実行状態よりも優先度が高い場合を考慮して，実行可能状態のタスクも待たせる
      cElement.set_checkpoint(get_proc_unit_info())
      @aTask.each{|nPrcid, sObjectID, cObjectInfo|
        if (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RDY || KER_TTS_RUN || KER_TTS_RUS)
          cElement.set_wait_finish_sync(get_proc_unit_info(cObjectInfo))
        end
      }

      @aTask.each{|nPrcid, sObjectID, cObjectInfo|
        # メインタスクと対象タスクのプロセッサが異なる場合はメインタスクを移動
        if (nMainPrcid != nPrcid)
          cElement.set_syscall(hProcUnitInfo, "#{API_MIG_TSK}(#{TTG_TSK_SELF}, #{nPrcid})")
          nMainPrcid = nPrcid
        end

        if (cObjectInfo.hState[TSR_PRM_STATE] != KER_TTS_DMT)
          cElement.set_syscall(hProcUnitInfo, "#{API_TER_TSK}(#{sObjectID})")
        end
      }
      # 最後は必ずメインプロセッサに戻る
      if (nMainPrcid != @sMainPrcid)
        cElement.set_syscall(hProcUnitInfo, "#{API_MIG_TSK}(#{TTG_TSK_SELF}, #{@sMainPrcid})")
      end
      cElement.set_checkpoint(hProcUnitInfo)

      return cElement  # [IMCodeElement]全タスクを起動終了させるコード
    end

    #=================================================================
    # 概  要: 最後のrefが完了するまで他プロセッサの後処理を止めておく
    #         同期
    #=================================================================
    def gc_last_running_sync()
      cElement = IMCodeElement.new()

      # 実行状態の処理単位取得
      aActivateRunningProc = get_actvate_running_fmp()
      bFlg = false

      # 実行状態の処理単位側でrefする処理がある場合に備え，
      # 他プロセッサの処理単位のチェックポイントを待つ
      aActivateRunningProc.each{|cObjectInfo|
        if (!cObjectInfo.nil?() && (cObjectInfo.hState[TSR_PRM_PRCID] != @sMainPrcid))
          cElement.set_checkpoint(get_proc_unit_info(cObjectInfo))
          cElement.set_wait_check_sync(get_proc_unit_info(@cActivate, @cRunning), cObjectInfo.hState[TSR_PRM_PRCID])
          bFlg = true
        end
      }

      # 他プロセッサの処理単位を待たせるチェックポイントを配置
      cElement.set_checkpoint(get_proc_unit_info(@cActivate, @cRunning))

      aActivateRunningProc.each{|cObjectInfo|
        if (!cObjectInfo.nil?() && (cObjectInfo.hState[TSR_PRM_PRCID] != @sMainPrcid))
          cElement.set_comment(get_proc_unit_info(cObjectInfo), "LastRunningSync")
          cElement.set_wait_check_sync(get_proc_unit_info(cObjectInfo), @sMainPrcid)
          bFlg = true
        end
      }

      if (bFlg == false)
        return nil       # [nil]追加の必要なし
      else
        return cElement  # [IMCodeElement]LastRunningSyncコード
      end
    end

    #=================================================================
    # 概  要: 意図しないディスパッチを防ぐため，実行可能状態のタスクを
    #         起床待ちと遷移させるコードを返す
    #=================================================================
    def gc_lastpost_ready_sleep_fmp()
      cElement = IMCodeElement.new()

      @aPrcid.each{|nPrcid|
        if ((@aActivate[nPrcid] != nil) || (!@aCpuState[nPrcid].nil?() &&
            ((@aCpuState[nPrcid].hState[TSR_PRM_LOCCPU] == true) || (@aCpuState[nPrcid].hState[TSR_PRM_DISDSP] == true) ||
            ((@aCpuState[nPrcid].hState[TSR_PRM_CHGIPM] != 0) && (@aCpuState[nPrcid].hState[TSR_PRM_CHGIPM] != KER_TIPM_ENAALL)))))
           @hTask.each{|sObjectID, cObjectInfo|
             if ((cObjectInfo.hState[TSR_PRM_PRCID] == nPrcid) && (cObjectInfo.hState[TSR_PRM_STATE] == KER_TTS_RDY))
               hProcUnitInfo = get_proc_unit_info(cObjectInfo)
               cElement.set_syscall(hProcUnitInfo, "#{API_SLP_TSK}()")
             end
           }
        end
      }

      return cElement  # [IMCodeElement]起床待ち遷移コード
    end

    #=================================================================
    # 概  要: スピンロック取得待ちのために遅延処理するコードを返す
    #=================================================================
    def gc_delay_wait_spin_fmp(lPrevMainTaskState)
      check_class(Symbol, lPrevMainTaskState)   # 前状態のメインタスクの状態

      cElement = IMCodeElement.new()

      # メインタスクの状態はそのままとする
      @lMainTaskState = lPrevMainTaskState

      # スピンロック取得待ちの処理単位情報取得
      # (複数存在する場合はどれかで遅延させればよい)
      cWaitProcUnit = nil
      @aActivate.each{|cActivate|
        if (!cActivate.nil?() && (cActivate.hState[TSR_PRM_HDLSTAT] == TSR_STT_A_WAITSPN))
          cWaitProcUnit = cActivate
          break
        end
      }
      @aRunning.each{|cRunning|
        if (!cRunning.nil?() &&
          ((cRunning.hState[TSR_PRM_STATE] == TSR_STT_R_WAITSPN) || (cRunning.hState[TSR_PRM_HDLSTAT] == TSR_STT_A_WAITSPN)))
          cWaitProcUnit = cRunning
          break
        end
      }

      # スピンロック取得待ち対象のスピンロック情報を取得
      cSpinlockInfo = get_object_info(cWaitProcUnit.hState[TSR_PRM_SPINID])

      # 対象のスピンロックを取得している処理単位情報を取得
      hSpinlockProcUnit = get_proc_unit_info(get_object_info(cSpinlockInfo.hState[TSR_PRM_PROCID]))

      # スピンロックを取得している処理単位において遅延処理を実行
      cElement.set_delay_loop(hSpinlockProcUnit)

      return cElement  # [IMCodeElement]スピンロック取得待ちのために遅延処理するコード
    end

    #=================================================================
    # 概  要: 禁止状態の割込みを設定するコードを返す
    #=================================================================
    def gc_lastpost_interrupt_dis_fmp()
      cElement      = IMCodeElement.new()
      hMainTaskInfo = get_proc_unit_info()

      # dis_intする必要のある割込み番号を抽出
      aDisIntNoID = []
      @hAllObject.each{|sObjectID, cObjectInfo|
        if ((GRP_INTERRUPT.include?(cObjectInfo.sObjectType) == true) && (cObjectInfo.hState[TSR_PRM_STATE] == KER_TA_ENAINT))
          aDisIntNoID.push([cObjectInfo.hState[TSR_PRM_INTNO], cObjectInfo.hState[TSR_PRM_PRCID]])
        end
      }

      # 重複を削除してすべてdis_intを実行する
      nMainPrcid = @sMainPrcid
      aDisIntNoID.uniq!()
      aDisIntNoID.each{|snIntNoID|
        # メインタスクと対象IntHdr/ISRのプロセッサが異なる場合はメインタスクを移動
        if (nMainPrcid != snIntNoID[1])
          cElement.set_syscall(hMainTaskInfo, "#{API_MIG_TSK}(#{TTG_TSK_SELF}, #{snIntNoID[1]})")
          nMainPrcid = snIntNoID[1]
        end

        cElement.set_syscall(hMainTaskInfo, "#{API_DIS_INT}(#{snIntNoID[0]})")
      }

      # 最後は必ずメインプロセッサに戻る
      if (nMainPrcid != @sMainPrcid)
        cElement.set_syscall(hMainTaskInfo, "#{API_MIG_TSK}(#{TTG_TSK_SELF}, #{@sMainPrcid})")
      end

      cElement.set_checkpoint(hMainTaskInfo)

      return cElement  # [IMCodeElement]禁止状態の割込みを設定するコード
    end

    #=================================================================
    # 概  要: スピンロック取得待ち状態の処理単位の有無確認を返す
    #=================================================================
    def exist_wait_spinlock_fmp()
      @aActivate.each{|cActivate|
        if (!cActivate.nil?() && (cActivate.hState[TSR_PRM_HDLSTAT] == TSR_STT_A_WAITSPN))
          return true  # [Bool]スピンロック取得待ち状態の処理単位がある
        end
      }

      @aRunning.each{|cRunning|
        if (!cRunning.nil?() &&
          ((cRunning.hState[TSR_PRM_STATE] == TSR_STT_R_WAITSPN) || (cRunning.hState[TSR_PRM_HDLSTAT] == TSR_STT_A_WAITSPN)))
          return true  # [Bool]スピンロック取得待ち状態の処理単位がある
        end
      }

      return false  # [Bool]スピンロック取得待ち状態の処理単位がない
    end
  end
end
