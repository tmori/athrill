#!ruby -Ke
#
#  TTG
#      TOPPERS Test Generator
#
#  Copyright (C) 2009-2012 by Center for Embedded Computing Systems
#              Graduate School of Information Science, Nagoya Univ., JAPAN
#  Copyright (C) 2010-2011 by Graduate School of Information Science,
#                             Aichi Prefectural Univ., JAPAN
#  Copyright (C) 2012 by FUJISOFT INCORPORATED
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
#  $Id: CommonModule.rb 11 2012-10-25 09:29:59Z nces-shigihara $
#
require "curses"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule

  #===================================================================
  # 変数のマクロ定義(Variable)
  #===================================================================
  # 変数名の定義
  VAR_ERCD             = "ercd"
  VAR_ERUINT           = "eruint"
  VAR_STATE            = "state"
  VAR_BOOTCNT          = "bootcnt"
  VAR_SYSTIM1          = "systim1"
  VAR_SYSTIM2          = "systim2"
  VAR_EXINF            = "exinf"
  VAR_TEXPTN           = "texptn"
  VAR_FLGPTN           = "flgptn"
  VAR_INTPRI           = "intpri"
  VAR_TSKID            = "tskid"
  VAR_PRCID            = "prcid"
  VAR_WAIPTN           = "waiptn"
  VAR_WFMODE           = "wfmode"
  VAR_DATA             = "data"
  VAR_DATAPRI          = "datapri"
  VAR_BLK              = "blk"
  VAR_RTSK             = "rtsk"
  VAR_RCYC             = "rcyc"
  VAR_RTEX             = "rtex"
  VAR_RDTQ             = "rdtq"
  VAR_RPDQ             = "rpdq"
  VAR_RSEM             = "rsem"
  VAR_RALM             = "ralm"
  VAR_RMPF             = "rmpf"
  VAR_RMBX             = "rmbx"
  VAR_RSPN             = "rspn"
  VAR_RFLG             = "rflg"
  VAR_P_MSG            = "p_msg"
  VAR_P_MSG_PRI        = "p_msgpri"
  VAR_TIMEOUT          = "timeout"
  VAR_TARGET_TSKID     = "target_tskid"
  VAR_VOLATILE_ULONG_T = "volatile ulong_t"
  VAR_GCOV_LOCK_FLG    = "coverage_progress"

  # 変数の型の定義
  TYP_ER          = "ER"
  TYP_ER_UINT     = "ER_UINT"
  TYP_BOOL_T      = "bool_t"
  TYP_INTPTR_T    = "intptr_t"
  TYP_PRI         = "PRI"
  TYP_ID          = "ID"
  TYP_MODE        = "MODE"
  TYP_STAT        = "STAT"
  TYP_TMO         = "TMO"
  TYP_RELTIM      = "RELTIM"
  TYP_VOID        = "void"
  TYP_VOID_P      = "void*"
  TYP_UINT_T      = "uint_t"
  TYP_ULONG_T     = "ulong_t"
  TYP_FLGPTN      = "FLGPTN"
  TYP_TEXPTN      = "TEXPTN"
  TYP_T_MSG       = "T_MSG"
  TYP_T_P_MSG     = "T_MSG*"
  TYP_T_MSG_PRI   = "T_MSG_PRI"
  TYP_T_P_MSG_PRI = "T_MSG_PRI*"
  TYP_T_RTSK      = "T_RTSK"
  TYP_T_RTEX      = "T_RTEX"
  TYP_T_RSEM      = "T_RSEM"
  TYP_T_RFLG      = "T_RFLG"
  TYP_T_RDTQ      = "T_RDTQ"
  TYP_T_RPDQ      = "T_RPDQ"
  TYP_T_RMBX      = "T_RMBX"
  TYP_T_RMPF      = "T_RMPF"
  TYP_T_RCYC      = "T_RCYC"
  TYP_T_RALM      = "T_RALM"
  TYP_T_RSPN      = "T_RSPN"
  TYP_T_TTSP_RTSK = "T_TTSP_RTSK"
  TYP_T_TTSP_RTEX = "T_TTSP_RTEX"
  TYP_T_TTSP_RSEM = "T_TTSP_RSEM"
  TYP_T_TTSP_RFLG = "T_TTSP_RFLG"
  TYP_T_TTSP_RDTQ = "T_TTSP_RDTQ"
  TYP_T_TTSP_RPDQ = "T_TTSP_RPDQ"
  TYP_T_TTSP_RMBX = "T_TTSP_RMBX"
  TYP_T_TTSP_RMPF = "T_TTSP_RMPF"
  TYP_T_TTSP_RCYC = "T_TTSP_RCYC"
  TYP_T_TTSP_RALM = "T_TTSP_RALM"
  TYP_T_TTSP_RSPN = "T_TTSP_RSPN"
  TYP_SYSTIM      = "SYSTIM"
  TYP_SYSUTM      = "SYSUTM"


  # キャストの定義
  CST_TMO  = "(TMO)"
  CST_MSG1 = "(#{TYP_T_MSG}*)"
  CST_MSG2 = "(#{TYP_T_MSG}**)"

  # 変数の型と名前の対応
  GRP_VAR_TYPE = {
    TYP_ER          => VAR_ERCD,
    TYP_ER_UINT     => VAR_ERUINT,
    TYP_BOOL_T      => VAR_STATE,
    TYP_T_TTSP_RTSK => VAR_RTSK,
    TYP_T_TTSP_RTEX => VAR_RTEX,
    TYP_T_TTSP_RSEM => VAR_RSEM,
    TYP_T_TTSP_RFLG => VAR_RFLG,
    TYP_T_TTSP_RDTQ => VAR_RDTQ,
    TYP_T_TTSP_RPDQ => VAR_RPDQ,
    TYP_T_TTSP_RMBX => VAR_RMBX,
    TYP_T_TTSP_RMPF => VAR_RMPF,
    TYP_T_TTSP_RCYC => VAR_RCYC,
    TYP_T_TTSP_RALM => VAR_RALM,
    TYP_T_TTSP_RSPN => VAR_RSPN
  }

  # グローバル変数定義する型
  GRP_GLOBAL_TYPE = [TYP_T_MSG, TYP_T_P_MSG, TYP_T_MSG_PRI, TYP_T_P_MSG_PRI, TYP_VOID_P]

  #===================================================================
  # 関数のマクロ定義(Function)
  #===================================================================
  # 関数名の定義
  FNC_CHECK_MAIN          = "ttg_check_main_task"
  FNC_CHG_PRI_MAIN_TASK   = "ttg_chg_pri_main_task"
  FNC_MIGRATE_TASK        = "ttg_migrate_task"
  FNC_TARGET_GAIN_TICK    = "ttg_target_gain_tick"
  FNC_CHECK_ERCD          = "check_ercd"
  FNC_CHECK_ASSERT        = "check_assert"
  FNC_CHECK_POINT         = "ttsp_check_point"
  FNC_WAIT_CHECK_POINT    = "ttsp_wait_check_point"
  FNC_CHECK_FINISH        = "ttsp_check_finish"
  FNC_MP_CHECK_POINT      = "ttsp_mp_check_point"
  FNC_MP_WAIT_CHECK_POINT = "ttsp_mp_wait_check_point"
  FNC_MP_CHECK_FINISH     = "ttsp_mp_check_finish"
  FNC_BARRIER_SYNC        = "ttsp_barrier_sync"
  FNC_GET_CP_STATE        = "ttsp_get_cp_state"
  FNC_WAIT_FINISH_SYNC    = "ttsp_wait_finish_sync"
  FNC_STATE_SYNC          = "ttsp_state_sync"

  # 時間関連の関数
  FNC_STOP_TICK           = "ttsp_target_stop_tick"
  FNC_START_TICK          = "ttsp_target_start_tick"
  FNC_GAIN_TICK           = "ttsp_target_gain_tick"
  FNC_GAIN_TICK_PE        = "ttsp_target_gain_tick_pe"

  # 割込み関連の関数
  FNC_INT_RAISE     = "ttsp_int_raise"
  FNC_CLEAR_INT_REQ = "ttsp_clear_int_req"
  FNC_I_BEGIN_INT   = "i_begin_int"
  FNC_I_END_INT     = "i_end_int"

  # CPU例外関連
  FNC_CPUEXC_RAISE = "ttsp_cpuexc_raise"
  FNC_CPUEXC_HOOK  = "ttsp_cpuexc_hook"

  # TTSPのAPIコード関数化のマクロ定義
  FNC_REF_TSK       = "ttsp_ref_tsk"
  FNC_REF_ALM       = "ttsp_ref_alm"
  FNC_REF_CYC       = "ttsp_ref_cyc"
  FNC_REF_TEX       = "ttsp_ref_tex"
  FNC_REF_DTQ       = "ttsp_ref_dtq"
  FNC_REF_DATA      = "ttsp_ref_data"
  FNC_REF_SWAIT_DTQ = "ttsp_ref_swait_dtq"
  FNC_REF_RWAIT_DTQ = "ttsp_ref_rwait_dtq"
  FNC_REF_PDQ       = "ttsp_ref_pdq"
  FNC_REF_PRI_DATA  = "ttsp_ref_pridata"
  FNC_REF_SWAIT_PDQ = "ttsp_ref_swait_pdq"
  FNC_REF_RWAIT_PDQ = "ttsp_ref_rwait_pdq"
  FNC_REF_SEM       = "ttsp_ref_sem"
  FNC_REF_WAIT_SEM  = "ttsp_ref_wait_sem"
  FNC_REF_MPF       = "ttsp_ref_mpf"
  FNC_REF_WAIT_MPF  = "ttsp_ref_wait_mpf"
  FNC_REF_MBX       = "ttsp_ref_mbx"
  FNC_REF_MSG       = "ttsp_ref_msg"
  FNC_REF_RWAIT_MBX = "ttsp_ref_rwait_mbx"
  FNC_REF_FLG       = "ttsp_ref_flg"
  FNC_REF_RWAIT_FLG = "ttsp_ref_wait_flg"
  FNC_REF_SPN       = "ttsp_ref_spn"
  FNC_GET_IPM       = "ttsp_get_ipm"
  FNC_SUS_TSK       = "ttsp_sus_tsk"

  # GCOV関連
  FNC_GCOV_INIT         = "gcov_init"
  FNC_GCOV_PAUSE        = "gcov_pause"
  FNC_GCOV_RESUME       = "gcov_resume"
  FNC_GCOV_DUMP         = "gcov_dump"
  FNC_GCOV_C_PAUSE      = "#{FNC_GCOV_PAUSE}()"
  FNC_GCOV_C_RESUME     = "#{FNC_GCOV_RESUME}()"
  FNC_GCOV_C_DUMP       = "#{FNC_GCOV_DUMP}()"
  FNC_GCOV_INI          = "ttg_gcov_ini"
  FNC_GCOV_TER          = "ttg_gcov_ter"
  FNC_GCOV_TTG_PAUSE    = "ttg_gcov_pause"
  FNC_GCOV_TTG_RESUME   = "ttg_gcov_resume"
  FNC_GCOV_TTG_C_PAUSE  = "#{FNC_GCOV_TTG_PAUSE}()"
  FNC_GCOV_TTG_C_RESUME = "#{FNC_GCOV_TTG_RESUME}()"

  # テストライブラリ用変数初期化関連
  FNC_TEST_LIB_INI        = "ttsp_test_lib_init"
  FNC_INITIALIZE_TEST_LIB = "ttsp_initialize_test_lib()"

  #===================================================================
  # APIのマクロ定義(カーネルAPI)
  #===================================================================
  # APIのマクロ定義
  API_ACT_TSK   = "act_tsk"
  API_IACT_TSK  = "iact_tsk"
  API_MACT_TSK  = "mact_tsk"
  API_IMACT_TSK = "imact_tsk"
  API_SLP_TSK   = "slp_tsk"
  API_TSLP_TSK  = "tslp_tsk"
  API_DLY_TSK   = "dly_tsk"
  API_SUS_TSK   = "sus_tsk"
  API_TER_TSK   = "ter_tsk"
  API_WUP_TSK   = "wup_tsk"
  API_IWUP_TSK  = "iwup_tsk"
  API_STA_ALM   = "sta_alm"
  API_ISTA_ALM  = "ista_alm"
  API_MSTA_ALM  = "msta_alm"
  API_STP_ALM   = "stp_alm"
  API_STA_CYC   = "sta_cyc"
  API_MSTA_CYC  = "msta_cyc"
  API_STP_CYC   = "stp_cyc"
  API_CHG_PRI   = "chg_pri"
  API_ROT_RDQ   = "rot_rdq"
  API_SND_DTQ   = "snd_dtq"
  API_TSND_DTQ  = "tsnd_dtq"
  API_RCV_DTQ   = "rcv_dtq"
  API_TRCV_DTQ  = "trcv_dtq"
  API_LOC_CPU   = "loc_cpu"
  API_UNL_CPU   = "unl_cpu"
  API_ILOC_CPU  = "iloc_cpu"
  API_IUNL_CPU  = "iunl_cpu"
  API_DIS_DSP   = "dis_dsp"
  API_CHG_IPM   = "chg_ipm"
  API_ENA_DSP   = "ena_dsp"
  API_SIG_SEM   = "sig_sem"
  API_WAI_SEM   = "wai_sem"
  API_TWAI_SEM  = "twai_sem"
  API_CAN_ACT   = "can_act"
  API_SND_PDQ   = "snd_pdq"
  API_TSND_PDQ  = "tsnd_pdq"
  API_RCV_PDQ   = "rcv_pdq"
  API_TRCV_PDQ  = "trcv_pdq"
  API_ENA_TEX   = "ena_tex"
  API_RAS_TEX   = "ras_tex"
  API_SET_FLG   = "set_flg"
  API_CLR_FLG   = "clr_flg"
  API_WAI_FLG   = "wai_flg"
  API_TWAI_FLG  = "twai_flg"
  API_SND_MBX   = "snd_mbx"
  API_RCV_MBX   = "rcv_mbx"
  API_TRCV_MBX  = "trcv_mbx"
  API_GET_MPF   = "get_mpf"
  API_TGET_MPF  = "tget_mpf"
  API_SNS_LOC   = "sns_loc"
  API_SNS_DSP   = "sns_dsp"
  API_ENA_INT   = "ena_int"
  API_DIS_INT   = "dis_int"
  API_EXT_KER   = "ext_ker"
  API_MIG_TSK   = "mig_tsk"
  API_LOC_SPN   = "loc_spn"
  API_UNL_SPN   = "unl_spn"
  API_ILOC_SPN  = "iloc_spn"
  API_IUNL_SPN  = "iunl_spn"
  API_SNS_KER   = "sns_ker"
  API_GET_TIM   = "get_tim"
  API_GET_UTM   = "get_utm"

  # 静的APIのマクロ定義
  API_CRE_TSK = "CRE_TSK"
  API_CRE_ALM = "CRE_ALM"
  API_CRE_CYC = "CRE_CYC"
  API_CRE_SEM = "CRE_SEM"
  API_CRE_DTQ = "CRE_DTQ"
  API_CRE_PDQ = "CRE_PDQ"
  API_CRE_FLG = "CRE_FLG"
  API_CRE_MBX = "CRE_MBX"
  API_CRE_MPF = "CRE_MPF"
  API_CRE_SPN = "CRE_SPN"
  API_DEF_TEX = "DEF_TEX"
  API_DEF_INH = "DEF_INH"
  API_DEF_EXC = "DEF_EXC"
  API_CFG_INT = "CFG_INT"
  API_ATT_ISR = "ATT_ISR"
  API_ATT_INI = "ATT_INI"
  API_ATT_TER = "ATT_TER"

  # SILのマクロ定義
  SIL_PRE_LOC = "SIL_PRE_LOC"
  SIL_LOC_SPN = "SIL_LOC_SPN()"
  SIL_UNL_SPN = "SIL_UNL_SPN()"

  #===================================================================
  # Configureのマクロ定義(configre)
  #===================================================================
  CFG_MACRO             = "macro"                       # マクロ設定がされているキー
  CFG_MAIN_PRCID        = "main_prcid"                  # メインタスクID
  CFG_DEFAULT_CLASS     = "main_class"                  # メインタスククラス
  CFG_OUT_FILE          = "out_file_name"               # 出力ファイル名
  CFG_PRC_NUM           = "prc_num"                     # プロセッサ数
  CFG_TIMER_ARCH        = "timer_arch"                  # タイマアーキテクチャ
  CFG_TIME_MANAGE_CLASS = "time_manage_class"           # グローバルタイマ用クラス
  CFG_TIME_MANAGE_PRCID = "time_manage_prcid"           # システム時刻管理プロセッサID
  CFG_SPINLOCK_NUM      = "spinlock_num"                # スピンロック数
  CFG_WAIT_SPIN_LOOP    = "wait_spin_loop"              # スピンロック取得待ちを確認するためのループ回数
  CFG_STACK_SHARE       = "stack_share"                 # スタック共有モード
  CFG_ALL_GAIN_TIME     = "all_gain_time"               # 全テストシナリオで時間を進めるか
  CFG_FUNC_TIME         = "func_time"                   # 時間操作関数利用可否
  CFG_FUNC_INTERRUPT    = "func_interrupt"              # 割込み発生関数利用可否
  CFG_FUNC_EXCEPTION    = "func_exception"              # CPU例外発生関数利用可否
  CFG_OWN_IPI_RAISE     = "own_ipi_raise"               # 自プロセッサへのプロセッサ間割込みを発行可能
  CFG_ENA_EXC_LOCK      = "enable_exc_in_cpulock"       # CPUロック中のCPU例外発生のサポート
  CFG_ENA_CHGIPM        = "enable_chg_ipm_in_non_task"  # 非タスクコンテキストからの割込み優先度マスク変更サポート
  CFG_EXCEPT_ARG_NAME   = "exception_arg_name"          # CPU例外ハンドラの引数とする変数名
  CFG_IRC_ARCH          = "irc_arch"                    # IRCアーキテクチャ
  CFG_YAML_LIBRARY      = "yaml_lib"                    # YAMLライブラリ
  CFG_ENABLE_GCOV       = "enable_gcov"                 # GCOVを取得するか
  CFG_ENABLE_LOG        = "enable_log"                  # ログを取得するか
  CFG_SUPPORT_GET_UTM   = "api_support_get_utm"         # APIのget_utmをサポートするか
  CFG_SUPPORT_ENA_INT   = "api_support_ena_int"         # APIのena_intをサポートするか
  CFG_SUPPORT_DIS_INT   = "api_support_dis_int"         # APIのdis_intをサポートするか
  CFG_TIMER_INT_PRI     = "timer_int_pri"               # タイマ割込みの割込み優先度

  CFG_PROFILE           = "profile"                     # プロファイル設定
  CFG_FILE              = "configure_file"              # 読み込む設定ファイル
  CFG_TEST_FLOW         = "test_flow_mode"              # テストフロー出力モード
  CFG_DEBUG_MODE        = "debug_mode"                  # デバッグモード
  CFG_NO_PROGRESS_BAR   = "no progress bar"             # プログレスバー非表示モード
  CFG_ENABLE_TTJ        = "enable_ttj"                  # TTJを有効化

  # configureに指定必須な項目
  GRP_CFG_NECESSARY_KEYS = [
    CFG_MACRO,
    CFG_MAIN_PRCID,
    CFG_DEFAULT_CLASS,
    CFG_OUT_FILE,
    CFG_PRC_NUM,
    CFG_TIMER_ARCH,
    CFG_TIME_MANAGE_CLASS,
    CFG_TIME_MANAGE_PRCID,
    CFG_SPINLOCK_NUM,
    CFG_WAIT_SPIN_LOOP,
    CFG_STACK_SHARE,
    CFG_ALL_GAIN_TIME,
    CFG_FUNC_TIME,
    CFG_FUNC_INTERRUPT,
    CFG_FUNC_EXCEPTION,
    CFG_OWN_IPI_RAISE,
    CFG_ENA_EXC_LOCK,
    CFG_ENA_CHGIPM,
    CFG_EXCEPT_ARG_NAME,
    CFG_IRC_ARCH,
    CFG_YAML_LIBRARY,
    CFG_ENABLE_GCOV,
    CFG_ENABLE_LOG,
    CFG_SUPPORT_GET_UTM,
    CFG_SUPPORT_ENA_INT,
    CFG_SUPPORT_DIS_INT,
    CFG_TIMER_INT_PRI
  ]

  CFG_PROFILE_ASP = "asp"
  CFG_PROFILE_FMP = "fmp"

  CFG_LIB_YAML    = "yaml"
  CFG_LIB_KWALIFY = "kwalify"

  # prcidのマクロ
  CFG_MCR_REX_PRCID         = /^PRC_(SELF|OTHER(_[12]|))$/  # yamlに記述されているマクロの正規表現
  CFG_MCR_PRC_SELF          = "PRC_SELF"
  CFG_MCR_PRC_OTHER         = "PRC_OTHER"
  CFG_MCR_PRC_OTHER_1       = "PRC_OTHER_1"
  CFG_MCR_PRC_OTHER_2       = "PRC_OTHER_2"

  CFG_MCR_PRC_TIMER_SELF    = "PRC_TIMER_SELF"
  CFG_MCR_PRC_TIMER_OTHER   = "PRC_TIMER_OTHER"
  CFG_MCR_PRC_TIMER_OTHER_1 = "PRC_TIMER_OTHER_1"
  CFG_MCR_PRC_TIMER_OTHER_2 = "PRC_TIMER_OTHER_2"

  # classのマクロ
  CFG_MCR_REX_CLASS                            = /^CLS_(SELF|OTHER(_[12]|))_(ALL|ONLY_\1)$/  # yamlに記述されているマクロの正規表現
  CFG_MCR_CLS_SELF_ALL                         = "CLS_SELF_ALL"
  CFG_MCR_CLS_OTHER_ALL                        = "CLS_OTHER_ALL"
  CFG_MCR_CLS_OTHER_1_ALL                      = "CLS_OTHER_1_ALL"
  CFG_MCR_CLS_OTHER_2_ALL                      = "CLS_OTHER_2_ALL"
  CFG_MCR_CLS_SELF_ONLY_SELF                   = "CLS_SELF_ONLY_SELF"
  CFG_MCR_CLS_OTHER_ONLY_OTHER                 = "CLS_OTHER_ONLY_OTHER"
  CFG_MCR_CLS_OTHER_1_ONLY_OTHER_1             = "CLS_OTHER_1_ONLY_OTHER_1"
  CFG_MCR_CLS_OTHER_2_ONLY_OTHER_2             = "CLS_OTHER_2_ONLY_OTHER_2"

  CFG_MCR_CLS_TIMER_SELF_ALL                   = "CLS_TIMER_SELF_ALL"
  CFG_MCR_CLS_TIMER_OTHER_ALL                  = "CLS_TIMER_OTHER_ALL"
  CFG_MCR_CLS_TIMER_OTHER_1_ALL                = "CLS_TIMER_OTHER_1_ALL"
  CFG_MCR_CLS_TIMER_OTHER_2_ALL                = "CLS_TIMER_OTHER_2_ALL"
  CFG_MCR_CLS_TIMER_ONLY_TIMER                 = "CLS_TIMER_ONLY_TIMER"
  CFG_MCR_CLS_TIMER_OTHER_ONLY_TIMER_OTHER     = "CLS_TIMER_OTHER_ONLY_TIMER_OTHER"
  CFG_MCR_CLS_TIMER_OTHER_1_ONLY_TIMER_OTHER_1 = "CLS_TIMER_OTHER_1_ONLY_TIMER_OTHER_1"
  CFG_MCR_CLS_TIMER_OTHER_2_ONLY_TIMER_OTHER_2 = "CLS_TIMER_OTHER_2_ONLY_TIMER_OTHER_2"

  CFG_MCR_REX_INTNO = /^(INTNO_((INVALID|NOT_SET)_|)(SELF|OTHER(_[12]|)))((_\w+|)_[A-C]|)$/
  CFG_MCR_REX_INHNO = /^(INHNO_(SELF|OTHER(_[12]|)))(_[A-C]|)$/
  CFG_MCR_REX_EXCNO = /^EXCNO_(SELF|OTHER(_[12]|))_A$/

  # 定義が必要なマクロ
  GRP_CFG_NECESSARY_MACRO = [
    "MAIN_PRCID", CFG_MCR_PRC_SELF, CFG_MCR_PRC_OTHER, CFG_MCR_PRC_OTHER_1, CFG_MCR_PRC_OTHER_2,

    CFG_MCR_CLS_SELF_ALL, CFG_MCR_CLS_OTHER_ALL, CFG_MCR_CLS_OTHER_1_ALL, CFG_MCR_CLS_OTHER_2_ALL,
    CFG_MCR_CLS_SELF_ONLY_SELF, CFG_MCR_CLS_OTHER_ONLY_OTHER,
    CFG_MCR_CLS_OTHER_1_ONLY_OTHER_1, CFG_MCR_CLS_OTHER_2_ONLY_OTHER_2,

    "INTNO_SELF_INH_A", "INTNO_SELF_INH_B", "INTNO_SELF_INH_C",
    "INTNO_SELF_ISR_A", "INTNO_SELF_ISR_B", "INTNO_SELF_ISR_C",
    "INTNO_OTHER_INH_A", "INTNO_OTHER_INH_B", "INTNO_OTHER_INH_C",
    "INTNO_OTHER_ISR_A", "INTNO_OTHER_ISR_B", "INTNO_OTHER_ISR_C",
    "INTNO_OTHER_1_INH_A", "INTNO_OTHER_1_INH_B", "INTNO_OTHER_1_INH_C",
    "INTNO_OTHER_1_ISR_A", "INTNO_OTHER_1_ISR_B", "INTNO_OTHER_1_ISR_C",
    "INTNO_OTHER_2_INH_A", "INTNO_OTHER_2_INH_B", "INTNO_OTHER_2_INH_C",
    "INTNO_OTHER_2_ISR_A", "INTNO_OTHER_2_ISR_B", "INTNO_OTHER_2_ISR_C",
    "INTNO_GLOBAL_IRC_INH_A", "INTNO_GLOBAL_IRC_INH_B", "INTNO_GLOBAL_IRC_INH_C",
    "INTNO_GLOBAL_IRC_ISR_A", "INTNO_GLOBAL_IRC_ISR_B", "INTNO_GLOBAL_IRC_ISR_C",
    "INHNO_SELF_A", "INHNO_SELF_B", "INHNO_SELF_C",
    "INHNO_OTHER_A", "INHNO_OTHER_B", "INHNO_OTHER_C",
    "INHNO_OTHER_1_A", "INHNO_OTHER_1_B", "INHNO_OTHER_1_C",
    "INHNO_OTHER_2_A", "INHNO_OTHER_2_B", "INHNO_OTHER_2_C",
    "INHNO_GLOBAL_IRC_SELF_A", "INHNO_GLOBAL_IRC_SELF_B", "INHNO_GLOBAL_IRC_SELF_C",
    "INTNO_INVALID_SELF", "INTNO_INVALID_OTHER", "INTNO_INVALID_OTHER_1", "INTNO_INVALID_OTHER_2",
    "INTNO_NOT_SET_SELF", "INTNO_NOT_SET_OTHER", "INTNO_NOT_SET_OTHER_1", "INTNO_NOT_SET_OTHER_2",

    "EXCNO_SELF_A", "EXCNO_OTHER_A", "EXCNO_OTHER_1_A", "EXCNO_OTHER_2_A",

    CFG_MCR_PRC_TIMER_SELF, CFG_MCR_PRC_TIMER_OTHER,
    CFG_MCR_PRC_TIMER_OTHER_1, CFG_MCR_PRC_TIMER_OTHER_2,

    CFG_MCR_CLS_TIMER_SELF_ALL, CFG_MCR_CLS_TIMER_OTHER_ALL,
    CFG_MCR_CLS_TIMER_OTHER_1_ALL, CFG_MCR_CLS_TIMER_OTHER_2_ALL,
    CFG_MCR_CLS_TIMER_ONLY_TIMER, CFG_MCR_CLS_TIMER_OTHER_ONLY_TIMER_OTHER,
    CFG_MCR_CLS_TIMER_OTHER_1_ONLY_TIMER_OTHER_1, CFG_MCR_CLS_TIMER_OTHER_2_ONLY_TIMER_OTHER_2,

    "INTNO_TIMER_SELF_INH_A", "INTNO_TIMER_SELF_INH_B", "INTNO_TIMER_SELF_INH_C",
    "INTNO_TIMER_SELF_ISR_A", "INTNO_TIMER_SELF_ISR_B", "INTNO_TIMER_SELF_ISR_C",
    "INTNO_TIMER_OTHER_INH_A", "INTNO_TIMER_OTHER_INH_B", "INTNO_TIMER_OTHER_INH_C",
    "INTNO_TIMER_OTHER_ISR_A", "INTNO_TIMER_OTHER_ISR_B", "INTNO_TIMER_OTHER_ISR_C",
    "INTNO_TIMER_OTHER_1_INH_A", "INTNO_TIMER_OTHER_1_INH_B", "INTNO_TIMER_OTHER_1_INH_C",
    "INTNO_TIMER_OTHER_1_ISR_A", "INTNO_TIMER_OTHER_1_ISR_B", "INTNO_TIMER_OTHER_1_ISR_C",
    "INTNO_TIMER_OTHER_2_INH_A", "INTNO_TIMER_OTHER_2_INH_B", "INTNO_TIMER_OTHER_2_INH_C",
    "INTNO_TIMER_OTHER_2_ISR_A", "INTNO_TIMER_OTHER_2_ISR_B", "INTNO_TIMER_OTHER_2_ISR_C",

    "INHNO_TIMER_SELF_A", "INHNO_TIMER_SELF_B", "INHNO_TIMER_SELF_C",
    "INHNO_TIMER_OTHER_A", "INHNO_TIMER_OTHER_B", "INHNO_TIMER_OTHER_C",
    "INHNO_TIMER_OTHER_1_A", "INHNO_TIMER_OTHER_1_B", "INHNO_TIMER_OTHER_1_C",
    "INHNO_TIMER_OTHER_2_A", "INHNO_TIMER_OTHER_2_B", "INHNO_TIMER_OTHER_2_C",

    "INTNO_TIMER_INVALID_SELF", "INTNO_TIMER_INVALID_OTHER", "INTNO_TIMER_INVALID_OTHER_1", "INTNO_TIMER_INVALID_OTHER_2",
    "INTNO_TIMER_NOT_SET_SELF", "INTNO_TIMER_NOT_SET_OTHER", "INTNO_TIMER_NOT_SET_OTHER_1", "INTNO_TIMER_NOT_SET_OTHER_2",
    "EXCNO_TIMER_SELF_A", "EXCNO_TIMER_OTHER_A", "EXCNO_TIMER_OTHER_1_A", "EXCNO_TIMER_OTHER_2_A",

    "TSK_PRI_HIGH", "TSK_PRI_MID", "TSK_PRI_LOW",
    "TSK_PRI_LE_4", "TSK_PRI_LE_LE_4", "TSK_PRI_GE_13", "TSK_PRI_LE_GE_13",

    "DATA_PRI_HIGH", "DATA_PRI_MID", "DATA_PRI_LOW", "DATA_PRI_MAX",
    "MSG_PRI_HIGH", "MSG_PRI_MID", "MSG_PRI_LOW", "MSG_PRI_MAX",

    "BIT_PATTERN_A", "BIT_PATTERN_B", "BIT_PATTERN_C", "BIT_PATTERN_D", "BIT_PATTERN_E",
    "BIT_PATTERN_0A", "BIT_PATTERN_0B", "BIT_PATTERN_0C",

    "WAIT_FLG_MODE_A", "WAIT_FLG_MODE_B", "WAIT_FLG_MODE_C", "WAIT_FLG_MODE_D", "WAIT_FLG_MODE_E",

    "DATA_A", "DATA_B", "DATA_C", "DATA_D", "DATA_E", "DATA_F",

    "EXINF_A", "EXINF_B", "EXINF_C", "EXINF_D", "EXINF_E",

    "FOREVER_TIME", "ANY_ELAPSED_TIME",
    "RELATIVE_TIME_A", "RELATIVE_TIME_B", "RELATIVE_TIME_C",
    "ACTIVATE_ALARM_TIME", "WAIT_ALARM_TIME",

    "ANY_MAX_SEMCNT", "ANY_NOW_SEMCNT", "ANY_INI_SEMCNT",
    "ANY_INI_BLKCNT", "ANY_NOW_BLKCNT", "ANY_BLKSZ",

    "TEXPTN_A", "TEXPTN_B", "TEXPTN_C", "TEXPTN_0A",

    "INT_PRI_TIMER", "INT_PRI_HIGH", "INT_PRI_MID", "INT_PRI_LOW",
    "ISR_PRI_HIGH", "ISR_PRI_MID", "ISR_PRI_LOW",

    "ANY_IPM", "ANY_IPM_FOR_TIMER",
    "ANY_ATT_CYC", "ANY_ATT_INH", "ANY_ATT_ISR", "ANY_ATT_SEM", "ANY_ATT_FLG",
    "ANY_ATT_DTQ", "ANY_ATT_PDQ", "ANY_ATT_MBX", "ANY_ATT_MPF",
    "ANY_OBJECT_ID", "ANY_TASK_STAT", "ANY_TASK_WAIT",
    "ANY_TEX_STAT", "ANY_ALARM_STAT", "ANY_CYCLIC_STAT",
    "ANY_DATA_CNT", "ANY_ADDRESS", "ANY_QUEUING_CNT", "ANY_SPINLOCK_STAT"
  ]

  #===================================================================
  # 構造体メンバーのマクロ定義(Struct)
  # ※カーネル側の構造体と代替関数の構造体が混在しているので注意する
  #===================================================================
  # TASK
  STR_TSKSTAT       = "tskstat"
  STR_TSKPRI        = "tskpri"
  STR_TSKBPRI       = "tskbpri"
  STR_TSKWAIT       = "tskwait"
  STR_WOBJID        = "wobjid"
  STR_LEFTTMO       = "lefttmo"
  STR_ACTCNT        = "actcnt"
  STR_WUPCNT        = "wupcnt"
  STR_EXINF         = "exinf"
  STR_ITSKPRI       = "itskpri"
  STR_PORDER        = "porder"
  STR_PRCID         = "prcid"
  STR_ACTPRC        = "actprc"

  # ALARM
  STR_ALMSTAT  = "almstat"
  STR_LEFTTIM  = "lefttim"

  # CYCLE
  STR_CYCSTAT  = "cycstat"
  STR_CYCATR   = "cycatr"
  STR_CYCTIM   = "cyctim"
  STR_CYCPHS   = "cycphs"

  # TASK_EXC
  STR_TEXSTAT  = "texstat"
  STR_PNDPTN   = "pndptn"

  # SEMAPHORE
  STR_WTSKID   = "wtskid"
  STR_SEMCNT   = "semcnt"
  STR_SEMATR   = "sematr"
  STR_ISEMCNT  = "isemcnt"
  STR_MAXSEM   = "maxsem"
  STR_WAITCNT  = "waitcnt"

  # EVENTFLAG
  STR_FLGATR   = "flgatr"
  STR_FLGPTN   = "flgptn"
  STR_IFLGPTN  = "iflgptn"

  # DATAQUEUE
  STR_STSKID   = "stskid"
  STR_RTSKID   = "rtskid"
  STR_DTQATR   = "dtqatr"
  STR_DTQCNT   = "dtqcnt"
  STR_SDTQCNT  = "sdtqcnt"
  STR_SWAITCNT = "swaitcnt"
  STR_RWAITCNT = "rwaitcnt"

  # P_DATAQUEUE
  STR_PDQATR   = "pdqatr"
  STR_PDQCNT   = "pdqcnt"
  STR_MAXDPRI  = "maxdpri"
  STR_SPDQCNT  = "spdqcnt"

  # MAILBOX
  STR_PK_MSG   = "pk_msg"
  STR_MBXATR   = "mbxatr"
  STR_MSGCNT   = "msgcnt"
  STR_MAXMPRI  = "maxmpri"
  STR_MSGPRI   = "msgpri"
  STR_MPF      = "mpf"

  # MEMORYPOOL
  STR_MPFATR   = "mpfatr"
  STR_BLKCNT   = "blkcnt"
  STR_FBLKCNT  = "fblkcnt"
  STR_BLKSZ    = "blksz"

  # SPINLOCK
  STR_SPNSTAT  = "spnstat"

  # メッセージ用構造体
  STR_T_MSG    = "t_msg"

  # 構造体のメンバ変数名，型，デフォルト初期値
  STR_INITIAL_INFO = {TYP_T_RTSK    => [[STR_TSKSTAT, TYP_STAT,    0],
                                        [STR_TSKPRI,  TYP_PRI,     0],
                                        [STR_TSKBPRI, TYP_PRI,     0],
                                        [STR_TSKWAIT, TYP_STAT,    0],
                                        [STR_WOBJID,  TYP_ID,      0],
                                        [STR_LEFTTMO, TYP_TMO,     0],
                                        [STR_ACTCNT,  TYP_UINT_T,  0],
                                        [STR_WUPCNT,  TYP_UINT_T,  0],
                                        [STR_PRCID,   TYP_ID,      0],
                                        [STR_ACTPRC,  TYP_ID,      0]],
                      TYP_T_RTEX    => [[STR_TEXSTAT, TYP_STAT,    0],
                                        [STR_PNDPTN,  TYP_TEXPTN,  0]],
                      TYP_T_RSEM    => [[STR_WTSKID,  TYP_ID,      0],
                                        [STR_SEMCNT,  TYP_UINT_T,  0]],
                      TYP_T_RFLG    => [[STR_WTSKID,  TYP_ID,      0],
                                        [STR_FLGPTN,  TYP_FLGPTN,  0]],
                      TYP_T_RDTQ    => [[STR_STSKID,  TYP_ID,      0],
                                        [STR_RTSKID,  TYP_ID,      0],
                                        [STR_SDTQCNT, TYP_UINT_T,  0]],
                      TYP_T_RPDQ    => [[STR_STSKID,  TYP_ID,      0],
                                        [STR_RTSKID,  TYP_ID,      0],
                                        [STR_SPDQCNT, TYP_UINT_T,  0]],
                      TYP_T_RMBX    => [[STR_WTSKID,  TYP_ID,      0],
                                        [STR_PK_MSG,  TYP_T_P_MSG, 0]],
                      TYP_T_RMPF    => [[STR_WTSKID,  TYP_ID,      0],
                                        [STR_FBLKCNT, TYP_UINT_T,  0]],
                      TYP_T_RCYC    => [[STR_CYCSTAT, TYP_STAT,    0],
                                        [STR_LEFTTIM, TYP_RELTIM,  0],
                                        [STR_PRCID,   TYP_ID,      0]],
                      TYP_T_RALM    => [[STR_ALMSTAT, TYP_STAT,    0],
                                        [STR_LEFTTIM, TYP_RELTIM,  0],
                                        [STR_PRCID,   TYP_ID,      0]],
                      TYP_T_RSPN    => [[STR_SPNSTAT, TYP_STAT,    0]],
                      TYP_T_MSG     => [[STR_T_MSG,   TYP_T_P_MSG, "{NULL}"]],
                      TYP_T_MSG_PRI => [[STR_T_MSG,   TYP_T_P_MSG, "{NULL}"],
                                        [STR_MSGPRI,  TYP_PRI,     0]]}

  #===================================================================
  # TESRYのマクロ定義(TESRY)
  #===================================================================
  # オブジェクト名のマクロ定義
  TSR_OBJ_TASK        = "TASK"
  TSR_OBJ_ALARM       = "ALARM"
  TSR_OBJ_CYCLE       = "CYCLE"
  TSR_OBJ_TASK_EXC    = "TASK_EXC"
  TSR_OBJ_SEMAPHORE   = "SEMAPHORE"
  TSR_OBJ_EVENTFLAG   = "EVENTFLAG"
  TSR_OBJ_DATAQUEUE   = "DATAQUEUE"
  TSR_OBJ_P_DATAQUEUE = "P_DATAQUEUE"
  TSR_OBJ_MAILBOX     = "MAILBOX"
  TSR_OBJ_MEMORYPOOL  = "MEMORYPOOL"
  TSR_OBJ_CPU_STATE   = "CPU_STATE"
  TSR_OBJ_SPINLOCK    = "SPINLOCK"
  TSR_OBJ_INTHDR      = "INTHDR"
  TSR_OBJ_ISR         = "ISR"
  TSR_OBJ_EXCEPTION   = "EXCEPTION"
  TSR_OBJ_INIRTN      = "INIRTN"
  TSR_OBJ_TERRTN      = "TERRTN"

  # テストシナリオバージョン情報
  TSR_LBL_VERSION = "version"

  # condition名のマクロ定義
  TSR_LBL_PRE       = "pre_condition"
  TSR_LBL_DO        = "do"
  TSR_LBL_POST      = "post_condition"
  TSR_LBL_VARIATION = "variation"
  TSR_LBL_NOTE      = "note"

  # do_，post_condition_のマクロ定義
  TSR_UNS_DO   = TSR_LBL_DO + "_"
  TSR_UNS_POST = TSR_LBL_POST + "_"

  # do_x，post_condition_xの正規表現のマクロ定義
  TSR_REX_DO       = /^#{TSR_UNS_DO}(\d*)?\z/
  TSR_REX_PRE_DO   = /^#{TSR_LBL_DO}(?:\_(\d+)|)$/    # シーケンス番号なしにもマッチ（do）
  TSR_REX_PRE_POST = /^#{TSR_LBL_POST}(?:\_(\d+)|)$/  # シーケンス番号なしにもマッチ（post）

  # 各種命名規則
  TSR_REX_TEST_ID        = /^[a-zA-Z]\w*$/               # 正しいテストID
  TSR_REX_OBJECT_ID      = /^[A-Z][A-Z0-9_]*$/           # 正しいオブジェクトID
  TSR_REX_VARIABLE       = /^[a-zA-Z\_][a-zA-Z0-9\_]+$/  # 正しい変数名

  # 統合するパラメータ(TESRYでは指定不可)
  TSR_PRM_STATE     = "state"
  TSR_PRM_ATR       = "atr"

  # 共通部分(処理単位オブジェクト)
  TSR_PRM_TYPE      = "type"
  TSR_PRM_EXINF     = "exinf"
  TSR_PRM_PRCID     = "prcid"
  TSR_PRM_BOOTCNT   = "bootcnt"
  TSR_PRM_LEFTTMO   = "lefttmo"
  TSR_PRM_HDLSTAT   = "hdlstat"
  TSR_PRM_CLASS     = "class"
  TSR_PRM_VAR       = "var"
  TSR_PRM_VAR_TYPE  = "type"
  TSR_PRM_VAR_VALUE = "value"
  TSR_PRM_SPINID    = "spinid"
  TSR_PRM_LEFTTIM   = "lefttim"  # yaml互換用

  # TASK
  TSR_PRM_TSKSTAT = "tskstat"
  TSR_PRM_TSKPRI  = "tskpri"
  TSR_PRM_ITSKPRI = "itskpri"
  TSR_PRM_ACTCNT  = "actcnt"
  TSR_PRM_WUPCNT  = "wupcnt"
  TSR_PRM_TSKWAIT = "tskwait"
  TSR_PRM_WOBJID  = "wobjid"
  TSR_PRM_PORDER  = "porder"
  TSR_PRM_ACTPRC  = "actprc"

  # ALARM
  TSR_PRM_ALMSTAT = "almstat"

  # CYCLE
  TSR_PRM_CYCSTAT = "cycstat"
  TSR_PRM_CYCATR  = "cycatr"
  TSR_PRM_CYCTIM  = "cyctim"
  TSR_PRM_CYCPHS  = "cycphs"

  # TASK_EXC
  TSR_PRM_TEXSTAT = "texstat"
  TSR_PRM_TASK    = "task"
  TSR_PRM_PNDPTN  = "pndptn"
  TSR_PRM_TEXPTN  = "texptn"

  # 共通部分(同期・通信オブジェクト)
  TSR_PRM_DATACNT  = "datacnt"
  TSR_PRM_WTSKLIST = "wtsklist"
  TSR_PRM_STSKLIST = "stsklist"
  TSR_PRM_RTSKLIST = "rtsklist"
  TSR_PRM_DATALIST = "datalist"

  # SEMAPHORE
  TSR_PRM_SEMATR  = "sematr"
  TSR_PRM_MAXSEM  = "maxsem"
  TSR_PRM_ISEMCNT = "isemcnt"
  TSR_PRM_SEMCNT  = "semcnt"

  # EVENTFLAG
  TSR_PRM_FLGATR  = "flgatr"
  TSR_PRM_IFLGPTN = "iflgptn"
  TSR_PRM_FLGPTN  = "flgptn"

  # DATAQUEUE
  TSR_PRM_DTQATR  = "dtqatr"
  TSR_PRM_DTQCNT  = "dtqcnt"

  # P_DATAQUEUE
  TSR_PRM_PDQATR  = "pdqatr"
  TSR_PRM_PDQCNT  = "pdqcnt"
  TSR_PRM_MAXDPRI = "maxdpri"

  # MAILBOX
  TSR_PRM_MBXATR  = "mbxatr"
  TSR_PRM_MAXMPRI = "maxmpri"
  TSR_PRM_MSGLIST = "msglist"

  # MEMORYPOOL
  TSR_PRM_MPFATR  = "mpfatr"
  TSR_PRM_BLKCNT  = "blkcnt"
  TSR_PRM_FBLKCNT = "fblkcnt"
  TSR_PRM_BLKSZ   = "blksz"
  TSR_PRM_MPF     = "mpf"

  # CPU_STATE
  TSR_PRM_LOCCPU  = "loc_cpu"
  TSR_PRM_DISDSP  = "dis_dsp"
  TSR_PRM_CHGIPM  = "chg_ipm"

  # SPINLOCK
  TSR_PRM_SPNSTAT = "spnstat"
  TSR_PRM_PROCID  = "procid"

  # 共通部分(割込みハンドラ・割込みサービスルーチン)
  TSR_PRM_INTATR  = "intatr"
  TSR_PRM_INTNO   = "intno"
  TSR_PRM_INTSTAT = "intstat"
  TSR_PRM_INTPRI  = "intpri"

  # INTHDR
  TSR_PRM_INHNO   = "inhno"

  # ISR
  TSR_PRM_ISRPRI  = "isrpri"

  # EXCEPTION
  TSR_PRM_EXCNO   = "excno"

  # INIRTN，TERRTN
  TSR_PRM_DO      = "do"
  TSR_PRM_GLOBAL  = "global"

  # do配下のマクロ定義
  TSR_PRM_ID      = "id"
  TSR_PRM_SYSCALL = "syscall"
  TSR_PRM_ERCD    = "ercd"
  TSR_PRM_ERUINT  = "eruint"
  TSR_PRM_BOOL    = "bool"
  TSR_PRM_CODE    = "code"
  TSR_PRM_GCOV    = "gcov"

  # variation配下のマクロ定義
  TSR_PRM_GAIN_TIME       = "gain_time"
  TSR_PRM_TIMER_ARCH      = CFG_TIMER_ARCH
  TSR_PRM_TIMER_GLOBAL    = "global"
  TSR_PRM_TIMER_LOCAL     = "local"
  TSR_PRM_ENA_EXC_LOCK    = CFG_ENA_EXC_LOCK
  TSR_PRN_ENA_CHGIPM      = CFG_ENA_CHGIPM
  TSR_PRM_IRC_ARCH        = CFG_IRC_ARCH
  TSR_PRM_IRC_GLOBAL      = "global"
  TSR_PRM_IRC_LOCAL       = "local"
  TSR_PRM_IRC_COMBINATION = "combination"
  TSR_PRM_GCOV_ALL        = "gcov_all"
  TSR_PRM_SUPPORT_GET_UTM = CFG_SUPPORT_GET_UTM
  TSR_PRM_SUPPORT_ENA_INT = CFG_SUPPORT_ENA_INT
  TSR_PRM_SUPPORT_DIS_INT = CFG_SUPPORT_DIS_INT

  # 処理単位オブジェクト状態のマクロ定義
  TSR_STT_RUNNING    = "running"
  TSR_STT_READY      = "ready"
  TSR_STT_WAITING    = "waiting"
  TSR_STT_SUSPENDED  = "suspended"
  TSR_STT_W_SUSPEND  = "waiting-suspended"
  TSR_STT_R_SUSPEND  = "running-suspended"
  TSR_STT_R_WAITSPN  = "running-waitspin"
  TSR_STT_DORMANT    = "dormant"
  TSR_STT_ACTIVATE   = "ACTIVATE"
  TSR_STT_A_WAITSPN  = "ACTIVATE-waitspin"
  TSR_STT_STP        = "STP"
  TSR_STT_TALM_STA   = "TALM_STA"
  TSR_STT_TALM_STP   = "TALM_STP"
  TSR_STT_TCYC_STA   = "TCYC_STA"
  TSR_STT_TCYC_STP   = "TCYC_STP"
  TSR_STT_TTEX_ENA   = "TTEX_ENA"
  TSR_STT_TTEX_DIS   = "TTEX_DIS"
  TSR_STT_TA_ENAINT  = "TA_ENAINT"
  TSR_STT_TA_DISINT  = "TA_DISINT"
  TSR_STT_TSPN_UNL   = "TSPN_UNL"
  TSR_STT_TSPN_LOC   = "TSPN_LOC"

  # 待ち状態のマクロ定義(SLEEP，DELAYのみ)
  TSR_STT_SLEEP = "SLEEP"
  TSR_STT_DELAY = "DELAY"

  # true，false
  TSR_STT_TRUE    = true
  TSR_STT_FALSE   = false

  # 変数名のマクロ定義
  TSR_VAR_DATA    = "data"
  TSR_VAR_DATAPRI = "datapri"
  TSR_VAR_VARDATA = "vardata"
  TSR_VAR_VARPRI  = "varpri"
  TSR_VAR_WAIPTN  = "waiptn"
  TSR_VAR_WFMODE  = "wfmode"
  TSR_VAR_MSG     = "msg"
  TSR_VAR_MSGPRI  = "msgpri"
  TSR_VAR_VAR     = "var"

  #===================================================================
  # カーネルのマクロ定義(Kernel)
  #===================================================================
  # 処理単位オブジェクト状態のマクロ定義
  KER_TTS_RUN   = "TTS_RUN"
  KER_TTS_RDY   = "TTS_RDY"
  KER_TTS_WAI   = "TTS_WAI"
  KER_TTS_SUS   = "TTS_SUS"
  KER_TTS_WAS   = "TTS_WAS"
  KER_TTS_RUS   = "TTS_RUS"
  KER_TTS_DMT   = "TTS_DMT"
  KER_TALM_STA  = "TALM_STA"
  KER_TALM_STP  = "TALM_STP"
  KER_TCYC_STA  = "TCYC_STA"
  KER_TCYC_STP  = "TCYC_STP"
  KER_TTEX_ENA  = "TTEX_ENA"
  KER_TTEX_DIS  = "TTEX_DIS"
  KER_TA_ENAINT = "TA_ENAINT"
  KER_TA_DISINT = "TA_DISINT"

  # 待ちオブジェクトの待ち状態のマクロ定義
  KER_TTW_SLP  = "TTW_SLP"
  KER_TTW_DLY  = "TTW_DLY"
  KER_TTW_SEM  = "TTW_SEM"
  KER_TTW_FLG  = "TTW_FLG"
  KER_TTW_SDTQ = "TTW_SDTQ"
  KER_TTW_RDTQ = "TTW_RDTQ"
  KER_TTW_SPDQ = "TTW_SPDQ"
  KER_TTW_RPDQ = "TTW_RPDQ"
  KER_TTW_MBX  = "TTW_MBX"
  KER_TTW_MPF  = "TTW_MPF"

  # 静的APIのマクロ定義
  KER_TA_ACT    = "TA_ACT"
  KER_TA_NULL   = "TA_NULL"
  KER_TA_TPRI   = "TA_TPRI"
  KER_TA_WMUL   = "TA_WMUL"
  KER_TA_CLR    = "TA_CLR"
  KER_TA_MPRI   = "TA_MPRI"
  KER_TA_STA    = "TA_STA"

  KER_TNFY_HANDLER = "TNFY_HANDLER"
  KER_TNFY_SETVAR = "TNFY_SETVAR"
  KER_TNFY_INCVAR = "TNFY_INCVAR"
  KER_TNFY_ACTTSK = "TNFY_ACTTSK"
  KER_TNFY_WUPTSK = "TNFY_WUPTSK"
  KER_TNFY_SIGSEM = "TNFY_SIGSEM"
  KER_TNFY_SETFLG = "TNFY_SETFLG"
  KER_TNFY_SNDDTQ = "TNFY_SNDDTQ"

  # 割込み優先度マスクのマクロ定義
  KER_TIPM_ENAALL = "TIPM_ENAALL"

  # イベントフラグ待ちモード
  KER_TWF_ANDW = "TWF_ANDW"
  KER_TWF_ORW  = "TWF_ORW"

  # 割り付けプロセッサ
  KER_TPRC_NONE = "TPRC_NONE"

  #===================================================================
  # グループのマクロ定義(マクロGroup)
  #===================================================================
  # 処理単位オブジェクトの状態のマクロ定義
  GRP_OBJECT_STATE = {
    # TASK
    TSR_STT_RUNNING   => KER_TTS_RUN,
    TSR_STT_READY     => KER_TTS_RDY,
    TSR_STT_WAITING   => KER_TTS_WAI,
    TSR_STT_SUSPENDED => KER_TTS_SUS,
    TSR_STT_W_SUSPEND => KER_TTS_WAS,
    TSR_STT_R_SUSPEND => KER_TTS_RUS,
    TSR_STT_R_WAITSPN => TSR_STT_R_WAITSPN, # カーネルでは区別できない
    TSR_STT_DORMANT   => KER_TTS_DMT,

    # ALARM
    TSR_STT_TALM_STA => KER_TALM_STA,
    TSR_STT_TALM_STP => KER_TALM_STP,

    # CYCLE
    TSR_STT_TCYC_STA => KER_TCYC_STA,
    TSR_STT_TCYC_STP => KER_TCYC_STP,

    # TASK_EXC
    TSR_STT_TTEX_ENA => KER_TTEX_ENA,
    TSR_STT_TTEX_DIS => KER_TTEX_DIS,

    # INTHDR，ISR
    TSR_STT_TA_ENAINT => KER_TA_ENAINT,
    TSR_STT_TA_DISINT => KER_TA_DISINT
  }

  # ASPで指定できる状態リスト
  GRP_OBJECT_STATE_AVAILABLE_ASP = {
    TSR_PRM_TSKSTAT => [
      TSR_STT_RUNNING, TSR_STT_READY, TSR_STT_WAITING,
      TSR_STT_SUSPENDED, TSR_STT_W_SUSPEND, TSR_STT_DORMANT
    ],
    TSR_PRM_ALMSTAT => [TSR_STT_TALM_STA, TSR_STT_TALM_STP],
    TSR_PRM_CYCSTAT => [TSR_STT_TCYC_STA, TSR_STT_TCYC_STP],
    TSR_PRM_TEXSTAT => [TSR_STT_TTEX_ENA, TSR_STT_TTEX_DIS],
    TSR_PRM_INTSTAT => [TSR_STT_TA_ENAINT, TSR_STT_TA_DISINT]
  }

  # FMPで指定できる状態リスト
  GRP_OBJECT_STATE_AVAILABLE_FMP = {
    TSR_PRM_TSKSTAT => [
      TSR_STT_RUNNING, TSR_STT_READY, TSR_STT_WAITING,
      TSR_STT_SUSPENDED, TSR_STT_W_SUSPEND, TSR_STT_DORMANT,
      TSR_STT_R_SUSPEND, TSR_STT_R_WAITSPN
    ],
    TSR_PRM_ALMSTAT => [TSR_STT_TALM_STA, TSR_STT_TALM_STP],
    TSR_PRM_CYCSTAT => [TSR_STT_TCYC_STA, TSR_STT_TCYC_STP],
    TSR_PRM_TEXSTAT => [TSR_STT_TTEX_ENA, TSR_STT_TTEX_DIS],
    TSR_PRM_INTSTAT => [TSR_STT_TA_ENAINT, TSR_STT_TA_DISINT]
  }


  # オブジェクト状態のマクロ定義
  GRP_ACTIVATE        = [TSR_STT_ACTIVATE, KER_TTS_RUN, KER_TTS_RUS, TSR_STT_R_WAITSPN, TSR_STT_A_WAITSPN]
  GRP_SUSPENDED       = [KER_TTS_SUS, KER_TTS_WAS]
  GRP_WAIT_NON_OBJECT = [TSR_STT_SLEEP, TSR_STT_DELAY]
  GRP_TIME_EVENT_STA  = [TSR_STT_TALM_STA, TSR_STT_TCYC_STA]
  GRP_TIME_EVENT_STP  = [TSR_STT_TALM_STP, TSR_STT_TCYC_STP]

  # 処理単位オブジェクトのマクロ定義
  GRP_PROCESS_UNIT = [
    TSR_OBJ_TASK,
    TSR_OBJ_ALARM,
    TSR_OBJ_CYCLE,
    TSR_OBJ_ISR,
    TSR_OBJ_TASK_EXC
  ]

  GRP_PROCESS_UNIT_ALL = [
    TSR_OBJ_TASK,
    TSR_OBJ_ALARM,
    TSR_OBJ_CYCLE,
    TSR_OBJ_TASK_EXC,
    TSR_OBJ_INTHDR,
    TSR_OBJ_ISR,
    TSR_OBJ_EXCEPTION,
    TSR_OBJ_INIRTN,
    TSR_OBJ_TERRTN
  ]

  # 同期・通信オブジェクトのマクロ定義
  GRP_SC_OBJECT = [
    TSR_OBJ_SEMAPHORE,
    TSR_OBJ_EVENTFLAG,
    TSR_OBJ_DATAQUEUE,
    TSR_OBJ_P_DATAQUEUE,
    TSR_OBJ_MAILBOX,
    TSR_OBJ_MEMORYPOOL
  ]

  # 非タスクコンテキストのマクロ定義
  GRP_NON_CONTEXT = [
    TSR_OBJ_ALARM,
    TSR_OBJ_CYCLE,
    TSR_OBJ_INTHDR,
    TSR_OBJ_ISR,
    TSR_OBJ_EXCEPTION
  ]

  # タイムイベントハンドラのマクロ定義
  GRP_TIME_EVENT_HDL = [TSR_OBJ_ALARM, TSR_OBJ_CYCLE]

  # 割込み処理のマクロ定義
  GRP_INTERRUPT = [TSR_OBJ_INTHDR, TSR_OBJ_ISR]

  # TestScenarioクラス内で保持する際に共通のキーで持つパラメータ名の関連付け
  GRP_PRM_KEY_PROC_STAT = {
    TSR_OBJ_TASK     => TSR_PRM_TSKSTAT,
    TSR_OBJ_ALARM    => TSR_PRM_ALMSTAT,
    TSR_OBJ_CYCLE    => TSR_PRM_CYCSTAT,
    TSR_OBJ_TASK_EXC => TSR_PRM_TEXSTAT,
    TSR_OBJ_INTHDR   => TSR_PRM_INTSTAT,
    TSR_OBJ_ISR      => TSR_PRM_INTSTAT
  }
  GRP_PRM_KEY_PROC_ATR = {
    TSR_OBJ_CYCLE  => TSR_PRM_CYCATR,
    TSR_OBJ_INTHDR => TSR_PRM_INTATR,
    TSR_OBJ_ISR    => TSR_PRM_INTATR
  }
  GRP_PRM_KEY_SC_DATACNT = {
    TSR_OBJ_DATAQUEUE   => TSR_PRM_DTQCNT,
    TSR_OBJ_P_DATAQUEUE => TSR_PRM_PDQCNT
  }
  GRP_PRM_KEY_SC_ATR = {
    TSR_OBJ_SEMAPHORE   => TSR_PRM_SEMATR,
    TSR_OBJ_EVENTFLAG   => TSR_PRM_FLGATR,
    TSR_OBJ_DATAQUEUE   => TSR_PRM_DTQATR,
    TSR_OBJ_P_DATAQUEUE => TSR_PRM_PDQATR,
    TSR_OBJ_MAILBOX     => TSR_PRM_MBXATR,
    TSR_OBJ_MEMORYPOOL  => TSR_PRM_MPFATR
  }

  # 状態
  GRP_ENUM_HDLSTAT_ASP = [TSR_STT_ACTIVATE, TSR_STT_STP]
  GRP_ENUM_HDLSTAT_FMP = [TSR_STT_ACTIVATE, TSR_STT_STP, TSR_STT_A_WAITSPN]
  GRP_ENUM_SPNSTAT     = [TSR_STT_TSPN_UNL, TSR_STT_TSPN_LOC]
  GRP_AVAILABLE_OBJATR = {
    TSR_OBJ_CYCLE       => [KER_TA_NULL, KER_TA_STA],
    TSR_OBJ_SEMAPHORE   => [KER_TA_NULL, KER_TA_TPRI],
    TSR_OBJ_EVENTFLAG   => [KER_TA_NULL, KER_TA_TPRI, KER_TA_WMUL, KER_TA_CLR],
    TSR_OBJ_DATAQUEUE   => [KER_TA_NULL, KER_TA_TPRI],
    TSR_OBJ_P_DATAQUEUE => [KER_TA_NULL, KER_TA_TPRI],
    TSR_OBJ_MAILBOX     => [KER_TA_NULL, KER_TA_TPRI, KER_TA_MPRI],
    TSR_OBJ_MEMORYPOOL  => [KER_TA_NULL, KER_TA_TPRI],
    TSR_OBJ_INTHDR      => [KER_TA_NULL, KER_TA_ENAINT],
    TSR_OBJ_ISR         => [KER_TA_NULL, KER_TA_ENAINT]
  }
  GRP_TASK_WAITING = [KER_TTS_WAI, KER_TTS_WAS]

  # イベントフラグ待ちモード
  GRP_EVENTFLAG_MODE = [KER_TWF_ANDW, KER_TWF_ORW]

  #===================================================================
  # TTGのマクロ定義
  #===================================================================
  TTG_IDX_LAST       = -1                          # 配列の最後の要素をアクセスするときの定数
  TTG_FINISH_MESSAGE = "All check points passed."  # 終了ルーチンが存在する場合に出力するメッセージ

  # 特殊な処理単位に用いるラベル
  TTG_LBL_INIRTN     = "ttg_ini_rtn"
  TTG_LBL_TERRTN     = "ttg_ter_rtn"
  TTG_LBL_INTHDR     = "TTG_INTHDR"
  TTG_LBL_ISR        = "TTG_ISR"
  TTG_LBL_EXCEPTION  = "TTG_EXCEPTION"
  TTG_LBL_SPINLOCK   = "TTG_SPINLOCK"
  TTG_LBL_CHK_ENAINT = "TTG_CHK_ENAINT"
  TTG_LBL_RESERVED   = "TTG_"  # テストIDの先頭文字の予約語

  # メインタスクと優先度のマクロ定義
  TTG_MAIN_TASK    = "MAIN_TASK"
  TTG_MAIN_BOOTCNT = 0

  # TTG優先度のマクロ定義
  TTG_TSK_SELF = "TSK_SELF"
  TTG_WAIT_PRI = 16
  TTG_MAIN_PRI = 1

  # その他
  TTG_TB = "\t" # Tab
  TTG_NL = "\n" # New Line
  TTG_ENUM_INVALID = "INVALID"

  # アラームハンドラをmsta_almでマイグレートする際に
  # stp_almするまで起動しない十分な時間(1秒)
  TTG_ENOUGH_MIG_TIME = 1000

  # [進捗表示用]最大テストID文字数(これ以上は切り捨てる)
  TTG_MAX_TEST_ID_SIZE = 40

  # 各進捗完了後に表示するメッセージ
  TTG_PROGRESS_MSG = " %d test cases passed."
  TTC_PROGRESS_MSG = " %d files passed."

  # 各ループ用定数定義
  TTG_SIL_DLY_NSE_TIME = "TTSP_SIL_DLY_NSE_TIME"
  TTG_LOOP_COUNT       = "TTSP_LOOP_COUNT"

  #===================================================================
  # 中間コードのマクロ定義(Inter Mediate Code)
  #===================================================================
  # 中間コードのエレメントタイプ
  IMC_COMMON              = "Common"
  IMC_TTG_MAIN            = "TTG_Main"
  IMC_BLOCK               = "block"
  IMC_CONDITION_SYNC      = "condition_sync"
  IMC_ACTIVE_SYNC         = "active_sync"
  IMC_STATE_SYNC          = "state_sync"
  IMC_TTG_INIRTN          = "TTG_INIRTN"
  IMC_TTG_TERRTN          = "TTG_TERRTN"

  # ASPの場合のクラス名
  IMC_NO_CLASS = "no_class"

  # 関数の処理単位種別
  IMC_FUNC_TYPE = "func"

  # カウント対象外とするラベル名
  GRP_IGNORE_LABEL = [IMC_COMMON, IMC_TTG_INIRTN, IMC_TTG_TERRTN, TTG_LBL_CHK_ENAINT]

  #===================================================================
  # TTCのマクロ定義
  #===================================================================
  # ディレクトリ・ファイルパス設定
  DEFAULT_CONFIG_FILE  = "#{TOOL_ROOT}/bin/configure.yaml"  # デフォルトのconfig

  TTC_DEBUG_FILE_BEFORE = "ttg_before.yaml"
  TTC_DEBUG_FILE_AFTER  = "ttg_after.yaml"
  TTC_DEBUG_FILE_GLOBAL = "ttg_global.yaml"
  TTC_EXCLUSION_FILE    = "ttg_exclusion_list.txt"

  TTC_REX_NUM = /^\d+$/             # 10進数
  TTC_REX_HEX = /^0x[0-9a-fA-F]+$/  # 16進数

  TTC_MAX_PRI     = 1   # 最大優先度
  TTC_MIN_PRI     = 16  # 最小優先度
  TTC_MAX_QUEUING = 1   # 最大キューイング数

  # post_conditionに補完しないオブジェクト
  GRP_NOT_COMPLEMENT_TYPE = [TSR_OBJ_INIRTN, TSR_OBJ_TERRTN]

  # 定義できるオブジェクトと属性（ASP）
  # 属性ごとの配列 [定義必須フラグ, pre_conditionのみ指定可能フラグ]
  GRP_DEF_OBJECT_ASP = {
    TSR_OBJ_TASK => {
      TSR_PRM_TSKSTAT => [true, false],
      TSR_PRM_TSKPRI  => [false, false],
      TSR_PRM_ITSKPRI => [false, true],
      TSR_PRM_EXINF   => [false, true],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_VAR     => [false, false],
      TSR_PRM_ACTCNT  => [false, false],
      TSR_PRM_WUPCNT  => [false, false],
      TSR_PRM_WOBJID  => [false, false],
      TSR_PRM_PORDER  => [false, false],
      TSR_PRM_LEFTTMO => [false, false]
    },
    TSR_OBJ_ALARM => {
      TSR_PRM_HDLSTAT => [true, false],
      TSR_PRM_ALMSTAT => [true, false],
      TSR_PRM_EXINF   => [false, true],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_VAR     => [false, false],
      TSR_PRM_LEFTTIM => [false, false]
    },
    TSR_OBJ_CYCLE => {
      TSR_PRM_CYCSTAT => [true, false],
      TSR_PRM_HDLSTAT => [true, false],
      TSR_PRM_CYCTIM  => [true, true],
      TSR_PRM_CYCPHS  => [true, true],
      TSR_PRM_CYCATR  => [false, true],
      TSR_PRM_EXINF   => [false, true],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_VAR     => [false, false],
      TSR_PRM_LEFTTIM => [false, false],
    },
    TSR_OBJ_TASK_EXC => {
      TSR_PRM_TASK    => [true, true],
      TSR_PRM_TEXSTAT => [false, false],
      TSR_PRM_HDLSTAT => [true, false],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_TEXPTN  => [false, false],
      TSR_PRM_PNDPTN  => [false, false],
      TSR_PRM_VAR     => [false, false]
    },
    TSR_OBJ_EXCEPTION => {
      TSR_PRM_EXCNO   => [true, true],
      TSR_PRM_HDLSTAT => [true, false],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_VAR     => [false, false]
    },
    TSR_OBJ_INTHDR => {
      TSR_PRM_INTNO   => [true, true],
      TSR_PRM_INHNO   => [true, true],
      TSR_PRM_INTPRI  => [true, true],
      TSR_PRM_INTATR  => [false, true],
      TSR_PRM_INTSTAT => [true, false],
      TSR_PRM_HDLSTAT => [true, false],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_VAR     => [false, false]
    },
    TSR_OBJ_ISR => {
      TSR_PRM_INTNO   => [true, true],
      TSR_PRM_INTPRI  => [true, true],
      TSR_PRM_INTATR  => [false, true],
      TSR_PRM_INTSTAT => [true, false],
      TSR_PRM_HDLSTAT => [true, false],
      TSR_PRM_ISRPRI  => [false, true],
      TSR_PRM_EXINF   => [false, true],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_VAR     => [false, false]
    },
    TSR_OBJ_INIRTN => {
      TSR_PRM_DO     => [true, false],
      TSR_PRM_EXINF  => [false, true]
    },
    TSR_OBJ_TERRTN => {
      TSR_PRM_DO     => [true, false],
      TSR_PRM_EXINF  => [false, true]
    },
    TSR_OBJ_SEMAPHORE => {
      TSR_PRM_SEMATR   => [false, true],
      TSR_PRM_MAXSEM   => [false, true],
      TSR_PRM_ISEMCNT  => [false, true],
      TSR_PRM_SEMCNT   => [false, false],
      TSR_PRM_WTSKLIST => [false, false]
    },
    TSR_OBJ_EVENTFLAG => {
      TSR_PRM_FLGATR   => [false, true],
      TSR_PRM_IFLGPTN  => [false, true],
      TSR_PRM_FLGPTN   => [false, false],
      TSR_PRM_WTSKLIST => [false, false]
    },
    TSR_OBJ_DATAQUEUE => {
      TSR_PRM_DTQATR   => [false, true],
      TSR_PRM_DTQCNT   => [false, true],
      TSR_PRM_STSKLIST => [false, false],
      TSR_PRM_RTSKLIST => [false, false],
      TSR_PRM_DATALIST => [false, false]
    },
    TSR_OBJ_P_DATAQUEUE => {
      TSR_PRM_PDQATR   => [false, true],
      TSR_PRM_MAXDPRI  => [false, true],
      TSR_PRM_PDQCNT   => [false, true],
      TSR_PRM_STSKLIST => [false, false],
      TSR_PRM_RTSKLIST => [false, false],
      TSR_PRM_DATALIST => [false, false]
    },
    TSR_OBJ_MAILBOX => {
      TSR_PRM_MBXATR   => [false, true],
      TSR_PRM_MAXMPRI  => [false, true],
      TSR_PRM_WTSKLIST => [false, false],
      TSR_PRM_MSGLIST  => [false, false]
    },
    TSR_OBJ_MEMORYPOOL => {
      TSR_PRM_MPFATR   => [false, true],
      TSR_PRM_BLKCNT   => [false, true],
      TSR_PRM_FBLKCNT  => [false, false],
      TSR_PRM_BLKSZ    => [false, true],
      TSR_PRM_WTSKLIST => [false, false],
      TSR_PRM_MPF      => [false, false]
    },
    TSR_OBJ_CPU_STATE => {
      TSR_PRM_CHGIPM => [false, false],
      TSR_PRM_LOCCPU => [false, false],
      TSR_PRM_DISDSP => [false, false]
    }
  }

  # 定義できるdoの属性（ASP）
  GRP_DEF_DO_ASP = {
    TSR_PRM_ID      => [String],
    TSR_PRM_SYSCALL => [String],
    TSR_PRM_GCOV    => [TrueClass, FalseClass],
    TSR_PRM_ERCD    => [String],
    TSR_PRM_ERUINT  => [String, Fixnum],
    TSR_PRM_BOOL    => [TrueClass, FalseClass],
    TSR_PRM_CODE    => [String]
  }

  # 定義できるオブジェクトと属性（FMP）
  # 属性ごとの配列 [定義必須フラグ, pre_conditionのみ指定可能フラグ]
  GRP_DEF_OBJECT_FMP = {
    TSR_OBJ_TASK => {
      TSR_PRM_TSKSTAT => [true, false],
      TSR_PRM_TSKPRI  => [false, false],
      TSR_PRM_ITSKPRI => [false, true],
      TSR_PRM_EXINF   => [false, true],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_VAR     => [false, false],
      TSR_PRM_ACTCNT  => [false, false],
      TSR_PRM_WUPCNT  => [false, false],
      TSR_PRM_WOBJID  => [false, false],
      TSR_PRM_PORDER  => [false, false],
      TSR_PRM_LEFTTMO => [false, false],
      TSR_PRM_CLASS   => [false, true],
      TSR_PRM_PRCID   => [false, false],
      TSR_PRM_ACTPRC  => [false, false],
      TSR_PRM_SPINID  => [false, false]
    },
    TSR_OBJ_ALARM => {
      TSR_PRM_HDLSTAT => [true, false],
      TSR_PRM_ALMSTAT => [true, false],
      TSR_PRM_EXINF   => [false, true],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_VAR     => [false, false],
      TSR_PRM_LEFTTIM => [false, false],
      TSR_PRM_CLASS   => [false, true],
      TSR_PRM_PRCID   => [false, false],
      TSR_PRM_SPINID  => [false, false]
    },
    TSR_OBJ_CYCLE => {
      TSR_PRM_CYCSTAT => [true, false],
      TSR_PRM_HDLSTAT => [true, false],
      TSR_PRM_CYCTIM  => [true, true],
      TSR_PRM_CYCPHS  => [true, true],
      TSR_PRM_CYCATR  => [false, true],
      TSR_PRM_EXINF   => [false, true],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_VAR     => [false, false],
      TSR_PRM_LEFTTIM => [false, false],
      TSR_PRM_CLASS   => [false, true],
      TSR_PRM_PRCID   => [false, false],
      TSR_PRM_SPINID  => [false, false]
    },
    TSR_OBJ_TASK_EXC => {
      TSR_PRM_TASK    => [true, true],
      TSR_PRM_TEXSTAT => [false, false],
      TSR_PRM_HDLSTAT => [true, false],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_TEXPTN  => [false, false],
      TSR_PRM_PNDPTN  => [false, false],
      TSR_PRM_VAR     => [false, false],
      TSR_PRM_SPINID  => [false, false]
    },
    TSR_OBJ_EXCEPTION => {
      TSR_PRM_EXCNO   => [true, true],
      TSR_PRM_HDLSTAT => [true, false],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_VAR     => [false, false],
      TSR_PRM_CLASS   => [false, true],
      TSR_PRM_PRCID   => [false, true],
      TSR_PRM_SPINID  => [false, false]
    },
    TSR_OBJ_INTHDR => {
      TSR_PRM_INTNO   => [true, true],
      TSR_PRM_INHNO   => [true, true],
      TSR_PRM_INTPRI  => [true, true],
      TSR_PRM_INTATR  => [false, true],
      TSR_PRM_INTSTAT => [true, false],
      TSR_PRM_HDLSTAT => [true, false],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_VAR     => [false, false],
      TSR_PRM_CLASS   => [false, true],
      TSR_PRM_PRCID   => [false, true],
      TSR_PRM_SPINID  => [false, false]
    },
    TSR_OBJ_ISR => {
      TSR_PRM_INTNO   => [true, true],
      TSR_PRM_INTPRI  => [true, true],
      TSR_PRM_INTATR  => [false, true],
      TSR_PRM_INTSTAT => [true, false],
      TSR_PRM_HDLSTAT => [true, false],
      TSR_PRM_ISRPRI  => [false, true],
      TSR_PRM_EXINF   => [false, true],
      TSR_PRM_BOOTCNT => [false, false],
      TSR_PRM_VAR     => [false, false],
      TSR_PRM_CLASS   => [false, true],
      TSR_PRM_PRCID   => [false, true],
      TSR_PRM_SPINID  => [false, false]
    },
    TSR_OBJ_INIRTN => {
      TSR_PRM_DO     => [true, false],
      TSR_PRM_GLOBAL => [false, true],
      TSR_PRM_EXINF  => [false, true],
      TSR_PRM_CLASS  => [false, true],
      TSR_PRM_PRCID  => [false, true]
    },
    TSR_OBJ_TERRTN => {
      TSR_PRM_DO     => [true, false],
      TSR_PRM_GLOBAL => [false, true],
      TSR_PRM_EXINF  => [false, true],
      TSR_PRM_CLASS  => [false, true],
      TSR_PRM_PRCID  => [false, true]
    },
    TSR_OBJ_SEMAPHORE => {
      TSR_PRM_SEMATR   => [false, true],
      TSR_PRM_MAXSEM   => [false, true],
      TSR_PRM_ISEMCNT  => [false, true],
      TSR_PRM_SEMCNT   => [false, false],
      TSR_PRM_WTSKLIST => [false, false],
      TSR_PRM_CLASS    => [false, true]
    },
    TSR_OBJ_EVENTFLAG => {
      TSR_PRM_FLGATR   => [false, true],
      TSR_PRM_IFLGPTN  => [false, true],
      TSR_PRM_FLGPTN   => [false, false],
      TSR_PRM_WTSKLIST => [false, false],
      TSR_PRM_CLASS    => [false, true]
    },
    TSR_OBJ_DATAQUEUE => {
      TSR_PRM_DTQATR   => [false, true],
      TSR_PRM_DTQCNT   => [false, true],
      TSR_PRM_STSKLIST => [false, false],
      TSR_PRM_RTSKLIST => [false, false],
      TSR_PRM_DATALIST => [false, false],
      TSR_PRM_CLASS    => [false, true]
    },
    TSR_OBJ_P_DATAQUEUE => {
      TSR_PRM_PDQATR   => [false, true],
      TSR_PRM_MAXDPRI  => [false, true],
      TSR_PRM_PDQCNT   => [false, true],
      TSR_PRM_STSKLIST => [false, false],
      TSR_PRM_RTSKLIST => [false, false],
      TSR_PRM_DATALIST => [false, false],
      TSR_PRM_CLASS    => [false, true]
    },
    TSR_OBJ_MAILBOX => {
      TSR_PRM_MBXATR   => [false, true],
      TSR_PRM_MAXMPRI  => [false, true],
      TSR_PRM_WTSKLIST => [false, false],
      TSR_PRM_MSGLIST  => [false, false],
      TSR_PRM_CLASS    => [false, true]
    },
    TSR_OBJ_MEMORYPOOL => {
      TSR_PRM_MPFATR   => [false, true],
      TSR_PRM_BLKCNT   => [false, true],
      TSR_PRM_FBLKCNT  => [false, false],
      TSR_PRM_BLKSZ    => [false, true],
      TSR_PRM_WTSKLIST => [false, false],
      TSR_PRM_MPF      => [false, false],
      TSR_PRM_CLASS    => [false, true]
    },
    TSR_OBJ_SPINLOCK => {
      TSR_PRM_SPNSTAT => [true, false],
      TSR_PRM_PROCID  => [false, false],
      TSR_PRM_CLASS   => [false, true]
    },
    TSR_OBJ_CPU_STATE => {
      TSR_PRM_CHGIPM => [false, false],
      TSR_PRM_LOCCPU => [false, false],
      TSR_PRM_DISDSP => [false, false],
      TSR_PRM_PRCID  => [false, true]
    }
  }

  # 定義できるdoの属性（FMP）
  GRP_DEF_DO_FMP = GRP_DEF_DO_ASP

  # タスクがdormant時にも指定できるパラメータ
  GRP_ACTIVATE_PRM_ON_DORMANT = {
    TSR_OBJ_TASK => [
      TSR_PRM_TSKSTAT,
      TSR_PRM_ITSKPRI,
      TSR_PRM_EXINF,
      TSR_PRM_CLASS,
      TSR_PRM_PRCID,
      TSR_PRM_VAR,
      TSR_PRM_ACTCNT,
      TSR_PRM_ACTPRC
    ],
    TSR_OBJ_TASK_EXC => [
      TSR_PRM_TASK,
      TSR_PRM_HDLSTAT,
      TSR_PRM_VAR
    ]
  }

  # dormant時にも補完するパラメータ
  GRP_COMPLEMENT_PRM_ON_DORMANT = [
    TSR_PRM_STATE,
    TSR_PRM_ITSKPRI,
    TSR_PRM_EXINF,
    TSR_PRM_CLASS,
    TSR_PRM_PRCID,
    TSR_PRM_VAR,
    TSR_PRM_ACTCNT,
    TSR_PRM_ACTPRC,
    TSR_PRM_BOOTCNT
  ]

  # グローバル対応用
  TTC_GLOBAL_KEY_SELF    = :self
  TTC_GLOBAL_KEY_OTHER   = :other
  TTC_GLOBAL_KEY_OTHER_1 = :other_1
  TTC_GLOBAL_KEY_OTHER_2 = :other_2

  TTC_GLOBAK_KEY_PRCID         = :prcid
  TTC_GLOBAK_KEY_INTNO_PREFIX  = :intno_prefix
  TTC_GLOBAK_KEY_INTNO_INVALID = :intno_invalid
  TTC_GLOBAK_KEY_INTNO_NOT_SET = :intno_not_set
  TTC_GLOBAK_KEY_INHNO_PREFIX  = :inhno_prefix
  TTC_GLOBAK_KEY_EXCNO         = :excno

  # グローバル対応テーブル
  TTC_GLOBAL_TABLE = {
    TTC_GLOBAK_KEY_PRCID => {
      :local => {
        TTC_GLOBAL_KEY_SELF    => CFG_MCR_PRC_SELF,
        TTC_GLOBAL_KEY_OTHER   => CFG_MCR_PRC_OTHER,
        TTC_GLOBAL_KEY_OTHER_1 => CFG_MCR_PRC_OTHER_1,
        TTC_GLOBAL_KEY_OTHER_2 => CFG_MCR_PRC_OTHER_2
      },
      :global => {
        TTC_GLOBAL_KEY_SELF    => CFG_MCR_PRC_TIMER_SELF,
        TTC_GLOBAL_KEY_OTHER   => CFG_MCR_PRC_TIMER_OTHER,
        TTC_GLOBAL_KEY_OTHER_1 => CFG_MCR_PRC_TIMER_OTHER_1,
        TTC_GLOBAL_KEY_OTHER_2 => CFG_MCR_PRC_TIMER_OTHER_2
      }
    },
    TTC_GLOBAK_KEY_INTNO_PREFIX => {
      :local => {
        TTC_GLOBAL_KEY_SELF    => "INTNO_SELF",
        TTC_GLOBAL_KEY_OTHER   => "INTNO_OTHER",
        TTC_GLOBAL_KEY_OTHER_1 => "INTNO_OTHER_1",
        TTC_GLOBAL_KEY_OTHER_2 => "INTNO_OTHER_2"
      },
      :global => {
        TTC_GLOBAL_KEY_SELF    => "INTNO_TIMER_SELF",
        TTC_GLOBAL_KEY_OTHER   => "INTNO_TIMER_OTHER",
        TTC_GLOBAL_KEY_OTHER_1 => "INTNO_TIMER_OTHER_1",
        TTC_GLOBAL_KEY_OTHER_2 => "INTNO_TIMER_OTHER_2"
      }
    },
    TTC_GLOBAK_KEY_INTNO_INVALID => {
      :local => {
        TTC_GLOBAL_KEY_SELF    => "INTNO_INVALID_SELF",
        TTC_GLOBAL_KEY_OTHER   => "INTNO_INVALID_OTHER",
        TTC_GLOBAL_KEY_OTHER_1 => "INTNO_INVALID_OTHER_1",
        TTC_GLOBAL_KEY_OTHER_2 => "INTNO_INVALID_OTHER_2"
      },
      :global => {
        TTC_GLOBAL_KEY_SELF    => "INTNO_TIMER_INVALID_SELF",
        TTC_GLOBAL_KEY_OTHER   => "INTNO_TIMER_INVALID_OTHER",
        TTC_GLOBAL_KEY_OTHER_1 => "INTNO_TIMER_INVALID_OTHER_1",
        TTC_GLOBAL_KEY_OTHER_2 => "INTNO_TIMER_INVALID_OTHER_2"
      }
    },
    TTC_GLOBAK_KEY_INTNO_NOT_SET => {
      :local => {
        TTC_GLOBAL_KEY_SELF    => "INTNO_NOT_SET_SELF",
        TTC_GLOBAL_KEY_OTHER   => "INTNO_NOT_SET_OTHER",
        TTC_GLOBAL_KEY_OTHER_1 => "INTNO_NOT_SET_OTHER_1",
        TTC_GLOBAL_KEY_OTHER_2 => "INTNO_NOT_SET_OTHER_2"
      },
      :global => {
        TTC_GLOBAL_KEY_SELF    => "INTNO_TIMER_NOT_SET_SELF",
        TTC_GLOBAL_KEY_OTHER   => "INTNO_TIMER_NOT_SET_OTHER",
        TTC_GLOBAL_KEY_OTHER_1 => "INTNO_TIMER_NOT_SET_OTHER_1",
        TTC_GLOBAL_KEY_OTHER_2 => "INTNO_TIMER_NOT_SET_OTHER_2"
      }
    },
    TTC_GLOBAK_KEY_INHNO_PREFIX => {
      :local => {
        TTC_GLOBAL_KEY_SELF    => "INHNO_SELF",
        TTC_GLOBAL_KEY_OTHER   => "INHNO_OTHER",
        TTC_GLOBAL_KEY_OTHER_1 => "INHNO_OTHER_1",
        TTC_GLOBAL_KEY_OTHER_2 => "INHNO_OTHER_2"
      },
      :global => {
        TTC_GLOBAL_KEY_SELF    => "INHNO_TIMER_SELF",
        TTC_GLOBAL_KEY_OTHER   => "INHNO_TIMER_OTHER",
        TTC_GLOBAL_KEY_OTHER_1 => "INHNO_TIMER_OTHER_1",
        TTC_GLOBAL_KEY_OTHER_2 => "INHNO_TIMER_OTHER_2"
      }
    },
    TTC_GLOBAK_KEY_EXCNO => {
      :local => {
        TTC_GLOBAL_KEY_SELF    => "EXCNO_SELF_A",
        TTC_GLOBAL_KEY_OTHER   => "EXCNO_OTHER_A",
        TTC_GLOBAL_KEY_OTHER_1 => "EXCNO_OTHER_1_A",
        TTC_GLOBAL_KEY_OTHER_2 => "EXCNO_OTHER_2_A"
      },
      :global => {
        TTC_GLOBAL_KEY_SELF    => "EXCNO_TIMER_SELF_A",
        TTC_GLOBAL_KEY_OTHER   => "EXCNO_TIMER_OTHER_A",
        TTC_GLOBAL_KEY_OTHER_1 => "EXCNO_TIMER_OTHER_1_A",
        TTC_GLOBAL_KEY_OTHER_2 => "EXCNO_TIMER_OTHER_2_A"
      }
    },
  }

  # グローバルタイマ時のclass変換テーブル（OTHERのみ）
  GRP_CONVERT_TABLE_CLASS_SINGLE_OTHER = {
    CFG_MCR_PRC_SELF  => {
      CFG_MCR_CLS_SELF_ALL       => {
        CFG_MCR_PRC_TIMER_SELF  => CFG_MCR_CLS_TIMER_SELF_ALL,
        CFG_MCR_PRC_TIMER_OTHER => CFG_MCR_CLS_TIMER_OTHER_ALL
      },
      CFG_MCR_CLS_OTHER_ALL      => {
        CFG_MCR_PRC_TIMER_SELF  => CFG_MCR_CLS_TIMER_OTHER_ALL,
        CFG_MCR_PRC_TIMER_OTHER => CFG_MCR_CLS_TIMER_SELF_ALL
      },
      CFG_MCR_CLS_SELF_ONLY_SELF => {
        CFG_MCR_PRC_TIMER_SELF  => CFG_MCR_CLS_TIMER_ONLY_TIMER,
        CFG_MCR_PRC_TIMER_OTHER => CFG_MCR_CLS_TIMER_OTHER_ONLY_TIMER_OTHER
      }
    },
    CFG_MCR_PRC_OTHER => {
      CFG_MCR_CLS_SELF_ALL        => {
        CFG_MCR_PRC_TIMER_SELF  => CFG_MCR_CLS_TIMER_OTHER_ALL,
        CFG_MCR_PRC_TIMER_OTHER => CFG_MCR_CLS_TIMER_SELF_ALL
      },
      CFG_MCR_CLS_OTHER_ALL       => {
        CFG_MCR_PRC_TIMER_SELF  => CFG_MCR_CLS_TIMER_SELF_ALL,
        CFG_MCR_PRC_TIMER_OTHER => CFG_MCR_CLS_TIMER_OTHER_ALL
      },
      CFG_MCR_CLS_OTHER_ONLY_OTHER => {
        CFG_MCR_PRC_TIMER_SELF  => CFG_MCR_CLS_TIMER_ONLY_TIMER,
        CFG_MCR_PRC_TIMER_OTHER => CFG_MCR_CLS_TIMER_OTHER_ONLY_TIMER_OTHER
      }
    }
  }

  # グローバルタイマ時のclass変換テーブル（OTHER_1，OTHER_2）
  GRP_CONVERT_TABLE_CLASS_MULTI_OTHER = {
    CFG_MCR_PRC_SELF   => {
      CFG_MCR_CLS_SELF_ALL       => {
        CFG_MCR_PRC_TIMER_SELF    => CFG_MCR_CLS_TIMER_SELF_ALL,
        CFG_MCR_PRC_TIMER_OTHER_1 => CFG_MCR_CLS_TIMER_OTHER_1_ALL,
        CFG_MCR_PRC_TIMER_OTHER_2 => CFG_MCR_CLS_TIMER_OTHER_2_ALL
      },
      CFG_MCR_CLS_OTHER_1_ALL    => {
        CFG_MCR_PRC_TIMER_SELF    => CFG_MCR_CLS_TIMER_OTHER_1_ALL,
        CFG_MCR_PRC_TIMER_OTHER_1 => CFG_MCR_CLS_TIMER_SELF_ALL,
        CFG_MCR_PRC_TIMER_OTHER_2 => CFG_MCR_CLS_TIMER_OTHER_2_ALL
      },
      CFG_MCR_CLS_OTHER_2_ALL    => {
        CFG_MCR_PRC_TIMER_SELF    => CFG_MCR_CLS_TIMER_OTHER_2_ALL,
        CFG_MCR_PRC_TIMER_OTHER_1 => CFG_MCR_CLS_TIMER_OTHER_1_ALL,
        CFG_MCR_PRC_TIMER_OTHER_2 => CFG_MCR_CLS_TIMER_SELF_ALL
      },
      CFG_MCR_CLS_SELF_ONLY_SELF => {
        CFG_MCR_PRC_TIMER_SELF    => CFG_MCR_CLS_TIMER_ONLY_TIMER,
        CFG_MCR_PRC_TIMER_OTHER_1 => CFG_MCR_CLS_TIMER_OTHER_1_ONLY_TIMER_OTHER_1,
        CFG_MCR_PRC_TIMER_OTHER_2 => CFG_MCR_CLS_TIMER_OTHER_2_ONLY_TIMER_OTHER_2
      }
    },
    CFG_MCR_PRC_OTHER_1 => {
      CFG_MCR_CLS_SELF_ALL             => {
        CFG_MCR_PRC_TIMER_SELF    => CFG_MCR_CLS_TIMER_OTHER_1_ALL,
        CFG_MCR_PRC_TIMER_OTHER_1 => CFG_MCR_CLS_TIMER_SELF_ALL,
        CFG_MCR_PRC_TIMER_OTHER_2 => CFG_MCR_CLS_TIMER_OTHER_2_ALL
      },
      CFG_MCR_CLS_OTHER_1_ALL          => {
        CFG_MCR_PRC_TIMER_SELF    => CFG_MCR_CLS_TIMER_SELF_ALL,
        CFG_MCR_PRC_TIMER_OTHER_1 => CFG_MCR_CLS_TIMER_OTHER_1_ALL,
        CFG_MCR_PRC_TIMER_OTHER_2 => CFG_MCR_CLS_TIMER_OTHER_2_ALL
      },
      CFG_MCR_CLS_OTHER_2_ALL          => {
        CFG_MCR_PRC_TIMER_SELF    => CFG_MCR_CLS_TIMER_OTHER_2_ALL,
        CFG_MCR_PRC_TIMER_OTHER_1 => CFG_MCR_CLS_TIMER_OTHER_1_ALL,
        CFG_MCR_PRC_TIMER_OTHER_2 => CFG_MCR_CLS_TIMER_SELF_ALL
      },
      CFG_MCR_CLS_OTHER_1_ONLY_OTHER_1 => {
        CFG_MCR_PRC_TIMER_SELF    => CFG_MCR_CLS_TIMER_ONLY_TIMER,
        CFG_MCR_PRC_TIMER_OTHER_1 => CFG_MCR_CLS_TIMER_OTHER_1_ONLY_TIMER_OTHER_1,
        CFG_MCR_PRC_TIMER_OTHER_2 => CFG_MCR_CLS_TIMER_OTHER_2_ONLY_TIMER_OTHER_2
      }
    },
    CFG_MCR_PRC_OTHER_2 => {
      CFG_MCR_CLS_SELF_ALL             => {
        CFG_MCR_PRC_TIMER_SELF    => CFG_MCR_CLS_TIMER_OTHER_2_ALL,
        CFG_MCR_PRC_TIMER_OTHER_1 => CFG_MCR_CLS_TIMER_OTHER_1_ALL,
        CFG_MCR_PRC_TIMER_OTHER_2 => CFG_MCR_CLS_TIMER_SELF_ALL
      },
      CFG_MCR_CLS_OTHER_1_ALL          => {
        CFG_MCR_PRC_TIMER_SELF    => CFG_MCR_CLS_TIMER_OTHER_1_ALL,
        CFG_MCR_PRC_TIMER_OTHER_1 => CFG_MCR_CLS_TIMER_SELF_ALL,
        CFG_MCR_PRC_TIMER_OTHER_2 => CFG_MCR_CLS_TIMER_OTHER_2_ALL
      },
      CFG_MCR_CLS_OTHER_2_ALL          => {
        CFG_MCR_PRC_TIMER_SELF    => CFG_MCR_CLS_TIMER_SELF_ALL,
        CFG_MCR_PRC_TIMER_OTHER_1 => CFG_MCR_CLS_TIMER_OTHER_1_ALL,
        CFG_MCR_PRC_TIMER_OTHER_2 => CFG_MCR_CLS_TIMER_OTHER_2_ALL
      },
      CFG_MCR_CLS_OTHER_2_ONLY_OTHER_2 => {
        CFG_MCR_PRC_TIMER_SELF    => CFG_MCR_CLS_TIMER_ONLY_TIMER,
        CFG_MCR_PRC_TIMER_OTHER_1 => CFG_MCR_CLS_TIMER_OTHER_1_ONLY_TIMER_OTHER_1,
        CFG_MCR_PRC_TIMER_OTHER_2 => CFG_MCR_CLS_TIMER_OTHER_2_ONLY_TIMER_OTHER_2
      }
    }
  }

  #===================================================================
  # TTJのマクロ定義
  #===================================================================
  # TTJの結果物のファイル名
  TTJ_PRINT_FILE  = "ttj_test_scenario.txt"

  # 日本語化する際に使われるマクロ定義
  TTJ_TAB         = "\t"
  TTJ_NEW_LINE    = "\n"
  TTJ_PARTITION   = ": "
  TTJ_PAUSE       = ", "
  TTJ_ATTR_EMPTY  = "なし"
  TTJ_BLOCK_OPEN  = "{"
  TTJ_BLOCK_CLOSE = "}"
  TTJ_BLINK_INDEX = " "

  # IDの属性
  TTJ_STT_ID = "ID"

  # doとpost_conditionに何も書かれていない場合の文字列
  TTJ_DO_EMPTY   = "コード発行なし"
  TTJ_POST_EMPTY = "変更なし"

  # 構造体の出力順序
  GRP_TTJ_STRUCT_SEQUENCE = [
    STR_TSKSTAT,
    STR_TSKPRI,
    STR_TSKBPRI,
    STR_TSKWAIT,
    STR_WOBJID,
    STR_LEFTTMO,
    STR_ACTCNT,
    STR_WUPCNT,
    STR_PRCID,
    STR_ACTPRC,
    STR_TEXSTAT,
    STR_PNDPTN,
    STR_WTSKID,
    STR_SEMCNT,
    STR_FLGPTN,
    STR_STSKID,
    STR_RTSKID,
    STR_SDTQCNT,
    STR_SPDQCNT,
    STR_PK_MSG,
    STR_FBLKCNT,
    STR_CYCSTAT,
    STR_LEFTTIM,
    STR_ALMSTAT,
    STR_SPNSTAT,
    STR_MSGPRI
  ]

  #===================================================================
  # コメント用マクロ定義
  #===================================================================
  CMT_WAIT_SPIN_LOOP = "loop for well-confirming to wait for spinlock"

  #===================================================================
  # エラーのマクロ定義(Error)
  #===================================================================
  ERR_MSG       = "%s:%s: Fatal Error(TTGToolError)" #abort用エラーメッセージ(__FILE__, __LINE__)
  ERR_LINE      = "======================================================================\n"
  ERR_HEADER    = "\n#{ERR_LINE}Some errors occurred.\n#{ERR_LINE}"
  ERR_RESULT    = "%d test cases error."
  ERR_EXCLUDE_HEADER = "\n#{ERR_LINE}Some files are excluded by variation mismatch.\n#{ERR_LINE}"
  ERR_EXCLUDE_RESULT = "%d test cases are excluded.(%d%% passed, %d / %d test cases)"

  # 環境に起因する問題
  ERR_CONSOLE_WIDTH = "Please make this window more bigger."

  # configure，option
  ERR_CFG_INVALID_VALUE  = "\"%s\" of configure is invalid value.[%s]"                  # 値が不正
  ERR_CFG_UNDEFINED_KEY  = "\"%s\" of configure is undefined key."                      # キーが未定義
  ERR_CFG_REQUIRED_KEY   = "\"%s\" of configure is required key."                       # 指定が必要
  ERR_CFG_REQUIRED_MACRO = "\"%s\" of macro is required."                               # 定義が必要
  ERR_CFG_BE_INTEGER_GT  = "\"%s\" of configure must be greater than %s.[%s]"           # 指定値より大きい必要
  ERR_CFG_BE_INTEGER_GE  = "\"%s\" of configure must be greater than or equal %s.[%s]"  # 指定値以上の必要
  ERR_CFG_BE_INTEGER_LT  = "\"%s\" of configure must be less than %s.[%s]"              # 指定値より小さい必要

  # マクロの依存関係
  ERR_MCR_INVALID_VALUE    = "\"%s\" of macro is invalid value.[%s]"
  ERR_MCR_INVALID_TYPE     = "\"%s\" of macro must be %s.[%s]"
  ERR_MCR_BE_INTEGER_NE    = "\"%s\" of macro must be not equal %s.[%s]"              # 指定値ではない必要
  ERR_MCR_BE_INTEGER_GT    = "\"%s\" of macro must be greater than %s.[%s]"           # 指定値より大きい必要
  ERR_MCR_BE_INTEGER_GE    = "\"%s\" of macro must be greater than or equal %s.[%s]"  # 指定値以上の必要
  ERR_MCR_BE_INTEGER_LT    = "\"%s\" of macro must be less than %s.[%s]"              # 指定値より小さい必要
  ERR_MCR_BE_INTEGER_LE    = "\"%s\" of macro must be less than or equal %s.[%s]"     # 指定値以下の必要
  ERR_MCR_BE_INTEGER_RANGE = "\"%s\" of macro must be between from %d to %d.[%s]"     # 指定値の範囲内の必要
  ERR_MCR_NOT_BIT_UNIQUE   = "\"%s\" of macro is not bit unique."

  # TTC汎用
  ERR_EXPRESSION_INVALID  = "expression is invalid.[%s]"       # 計算式の文法が不正
  ERR_YAML_SYNTAX_INVALID = "yaml syntax is invalid. [%s]"     # YAMLの文法が不正
  ERR_OPTION_INVALID      = "invalid option specified."        # オプション指定が不正
  ERR_CANNOT_OPEN_FILE    = "can't open file. [%s]"            # ファイルが開けない

  # 構造チェック
  ERR_INVALID_TYPE_TESTCASE = "test case must be %s.[%s]"               # テストケースのタイプが不正
  ERR_SCENARIO_NOT_EXIST    = "test scenario does not exist."           # ファイル内にテストシナリオが存在しない
  ERR_INVALID_TYPE_SCENARIO = "test scenario must be %s.[%s]"           # テストシナリオのタイプが不正
  ERR_NO_AVAILABLE_SCENARIO = "available test scenario does not exist." # 有効なテストシナリオが存在しない
  ERR_INVALID_TESTID        = "test id is invalid.[%s]"                 # テストIDが不正
  ERR_INVALID_TESTID_PREFIX = "test id use the reserved word."          # テストIDに予約語が使われている
  ERR_INVALID_OBJECTID      = "object id is invalid.[%s]"               # オブジェクトIDが不正
  ERR_TESTID_MULTIPLE       = "\"%s\" of test id is multiple defined."  # 同一テストIDが存在
  ERR_CONDITION_MULTIPLE    = "\"%s_%d\" is multiple defined."          # コンディションが複数定義（同一シーケンス番号）
  ERR_CONDITION_UNDEFINED   = "undefined condition found.[%s]"          # 未定義のコンディションが記述されている
  ERR_CONDITION_NOT_EXIST   = "\"%s\" is not exist."                    # 必要なコンディションが存在しない
  ERR_CONDITION_NOT_MATCH   = "sequence number do not match."           # doとpostのシーケンス番号がマッチしない
  ERR_INVALID_TIMETICK      = "timetick is invalid. [%s]"               # タイムティックが不正

  ERR_INVALID_TYPE      = "\"%s\" must be %s.[%s]"
  ERR_INVALID_TYPE_NIL  = "\"%s\" must be %s or Nil.[%s]"
  ERR_INVALID_VAR_NAME  = "variable name is invalid.[%s]"
  ERR_BE_INTEGER_LT     = "\"%s\" must be less than %s.[%s]"
  ERR_BE_INTEGER_LE     = "\"%s\" must be less than or equal %s.[%s]"
  ERR_BE_INTEGER_GT     = "\"%s\" must be greater than %s.[%s]"
  ERR_BE_INTEGER_GE     = "\"%s\" must be greater than or equal %s.[%s]"
  ERR_BE_INTEGER_RANGE  = "\"%s\" must be between from %d to %d.[%s]"     # 指定値の範囲内の必要
  ERR_BE_INCLUDE        = "\"%s\" must be included in (%s).[%s]"

  ERR_REQUIRED_KEY         = "\"%s\" is required key."
  ERR_UNDEFINED_KEY        = "\"%s\" is undefined key."
  ERR_ONLY_PRE_ATR         = "\"%s\" can be defined only in pre_condition."
  ERR_CANNNOT_PRE_ATR      = "\"%s\" can't be defined in pre_condition."
  ERR_ATR_DORMANT_TASK     = "\"%s\" can't be defined in #{TSR_STT_DORMANT} #{TSR_OBJ_TASK}."
  ERR_ATR_DORMANT_TASK_EXC = "\"%s\" can't be defined in \"#{TSR_OBJ_TASK_EXC}\" of #{TSR_STT_DORMANT} #{TSR_OBJ_TASK}."

  ERR_OBJECT_UNDEFINED    = "object type \"%s\" is undefined."
  ERR_OBJECT_NOCHANGE     = "no changed object described.[%s]"
  ERR_OBJECT_NOT_ACTIVATE = "\"%s\" is not activate object in pre_condition."

  ERR_VAR_UNDEFINED_MEMBER      = "\"%s\" is undefined member in struct type \"%s\"."
  ERR_VAR_INVALID_DEFINE_VALUE  = "\"#{TSR_PRM_VAR_VALUE}\" attribute can be defined only when type is data."
  ERR_VAR_INVALID_DEFINE_MEMBER = "member attributes can be defined only when type is struct."

  ERR_LIST_INVALID_TYPE          = "\"%s\"'s item must be %s.[%s]"
  ERR_LIST_MUST_BE_SINGLE        = "\"%s\"'s item's Hash size must be 1.[%d]"
  ERR_LIST_ITEM_INVALID_TYPE     = "\"%s\"'s item's value must be %s.[%s]"
  ERR_LIST_ITEM_INVALID_TYPE_NIL = "\"%s\"'s item's value must be %s or Nil.[%s]"

  # 処理単位
  ERR_NO_SPINID_WAITING      = "waiting for spinlock process unit must have \"#{TSR_PRM_SPINID}\"."
  # TASK
  ERR_NO_WOBJID_WAITING      = "waiting task must have \"#{TSR_PRM_WOBJID}\"."
  ERR_SLEEPING_CANNOT_WUP    = "sleeping task can't have \"#{TSR_PRM_WUPCNT}\""
  ERR_SET_WOBJID_NO_WAITING  = "non-waiting task have \"#{TSR_PRM_WOBJID}\"."
  ERR_NO_LEFTTMO_DELAY       = "delay task in pre_condition must have \"#{TSR_PRM_LEFTTMO}\"."
  ERR_SET_ACTPRC_NO_ACTCNT   = "non queuing task have \"#{TSR_PRM_ACTPRC}\"."
  # ALARM
  ERR_NO_LEFTTIM_ON_TALM_STA = "starting alarm in pre_condition must have \"#{TSR_PRM_LEFTTIM}\"."
  # CYCLE
  ERR_NOT_STA_ON_ACTIVATE    = "\"#{TSR_PRM_CYCSTAT}\" must be \"#{TSR_STT_TCYC_STA}\" when \"#{TSR_PRM_HDLSTAT}\" is \"#{TSR_STT_ACTIVATE}\"."
  # TASK_EXC
  ERR_NO_TEXPTN_ON_ACTIVATE  = "\"#{TSR_PRM_TEXPTN}\" is required when \"#{TSR_PRM_HDLSTAT}\" is \"#{TSR_STT_ACTIVATE}\"."
  ERR_TEXPTN_0_ON_ACTIVATE   = "\"#{TSR_PRM_TEXPTN}\" must be not 0 when \"#{TSR_PRM_HDLSTAT}\" is \"#{TSR_STT_ACTIVATE}\"."
  ERR_SET_PNDPTN_ON_ENABLE   = "\"#{TSR_PRM_PNDPTN}\" must be 0 when \"#{TSR_PRM_HDLSTAT}\" is \"#{TSR_STT_ACTIVATE}\" and \"#{TSR_PRM_TEXSTAT}\" is \"#{TSR_STT_TTEX_ENA}\"."
  # INIRTN，TERRTN
  ERR_ON_GLOBAL_SET_CLASS    = "\"%s\" is defined as global object so cannot define \"#{TSR_PRM_CLASS}\"."
  ERR_ON_GLOBAL_SET_PRCID    = "\"%s\" is defined as global object so cannot define \"#{TSR_PRM_PRCID}\"."
  # INTHDR・ISR
  ERR_ENABLE_INT_ON_ACTIVATE   = "\"#{TSR_PRM_INTSTAT}\" is not enable when \"#{TSR_PRM_HDLSTAT}\" is \"#{TSR_STT_ACTIVATE}\"."

  # 同期通信オブジェクト
  ERR_TASK_NAME_DUPLICATE    = "\"%s\" is duplicate in task list."
  # DATAQUEUE
  ERR_SEND_WAITING_HAVE_SPACE = "\"#{TSR_PRM_DATALIST}\" have some space but task is waiting to send."
  ERR_RECV_WAITING_HAVE_DATA  = "\"#{TSR_PRM_DATALIST}\" have some data but task is waiting to receive."
  ERR_DATALIST_HAVE_OVER_DATA = "\"#{TSR_PRM_DATALIST}\" have too many data than \"#{TSR_PRM_DTQCNT}\"."
  # SEMAPHORE
  ERR_OVER_THAN_MAXSEM      = "\"%s\" is too large than \"#{TSR_PRM_MAXSEM}\"."
  # MAILBOX
  ERR_MSGLIST_VARS_MUST_DEFINE = "\"#{TSR_VAR_MSG}\" and \"#{TSR_VAR_MSGPRI}\" must be defined when \"#{TSR_PRM_MBXATR}\" is \"#{KER_TA_MPRI}\"."
  ERR_CANNOT_BE_DEFINED_MSGPRI = "\"#{TSR_VAR_MSGPRI}\" can't be defined when \"#{TSR_PRM_MBXATR}\" is not \"#{KER_TA_MPRI}\"."
  ERR_MSGLIST_VAR_DUPLICATE    = "\"%s\" of variable name is duplicate."
  # SPINLOCK
  ERR_NO_PROCID_ON_TSPN_LOC = "spinlock must have \"#{TSR_PRM_PROCID}\" when \"#{TSR_PRM_SPNSTAT}\" is \"#{TSR_STT_TSPN_LOC}\"."

  # コンディションチェック
  ERR_NO_RUNNING_PROCESS_UNIT       = "no running process unit exist."
  ERR_NO_TASK                       = "no task exist."
  ERR_INVERTED_STATE_IN_PRE         = "inverted running state during cpulock in pre_condition is not supported.(prc: %d)"
  ERR_INVERTED_PORDER_IN_PRE        = "inverted \"#{TSR_PRM_PORDER}\" in pre_condition is not supported.(prc: %d)"
  ERR_SET_VALUE_NON_ACTIVATE        = "variable's value is defined in non-running process unit."
  ERR_PLURAL_RUNNING_TASK           = "plural running task exists.(prc: %d)"
  ERR_PLURAL_ACTIVATE_NON_CONTEXT   = "plural activate non-context exists.(prc: %d)"
  ERR_ONLY_ALARM_SET_CPU_STATE      = "only \"#{TSR_OBJ_ALARM}\" is running but \"#{TSR_OBJ_CPU_STATE}\" is defined excluding \"#{TSR_PRM_LOCCPU}\".(prc: %d)"
  ERR_CPULOCK_NOT_TASK_ALARM_RUN    = "\"#{TSR_PRM_LOCCPU}\" is true in spite of \"#{TSR_OBJ_TASK}\" or \"#{TSR_OBJ_ALARM}\" is not running.(prc: %d)"
  ERR_NO_WAITING_OBJECT             = "waiting object ID \"%s\" is not defined."
  ERR_PORDER_NOT_UNIQUE             = "porder is not unique.(%s)"
  ERR_PORDER_NOT_SEQUENCE           = "tskpri \"%d\"'s porder is not sequence.(prc: %d)"
  ERR_PLURAL_CPU_STATE              = "plural \"#{TSR_OBJ_CPU_STATE}\" exists.(prc: %d)"
  ERR_CANNNOT_REF_CPU_STATE         = "can not reference \"#{TSR_OBJ_CPU_STATE}\".(prc: %d)"
  ERR_CANNNOT_DEFINED_IN_POST       = "\"%s\" can't be defined in \"#{TSR_LBL_POST}\"."
  ERR_DUPLICATE_EXCNO               = "\"#{TSR_PRM_EXCNO}\" \"%s\" is duplicate.[%s]"
  ERR_TARGET_NOT_DEFINED            = "\"%s\" is not defined."
  ERR_TARGET_NOT_ACTIVATE           = "\"%s\" is not activate."
  ERR_TARGET_NOT_TASK               = "\"%s\" is not \"#{TSR_OBJ_TASK}\" object."
  ERR_TARGET_NOT_PROCESS_UNIT       = "\"%s\" is not process_unit."
  ERR_TARGET_NOT_SPINLOCK           = "\"%s\" is not \"#{TSR_OBJ_SPINLOCK}\" object."
  ERR_TASK_EXC_CANNOT_ACTIVATE      = "\"#{TSR_PRM_HDLSTAT}\" is \"#{TSR_STT_ACTIVATE}\" in spite of \"%s\" is \"#{TSR_STT_DORMANT}\"."
  ERR_NO_TEXSTAT_STT_NOT_DORMANT    = "\"#{TSR_PRM_TEXSTAT}\" is not defined in spite of \"%s\" is not \"#{TSR_STT_DORMANT}\"."
  ERR_INTNO_DUPLICATE               = "\"#{TSR_PRM_INTNO}\" \"%s\" is duplicate during \"#{TSR_OBJ_INTHDR}\" and \"#{TSR_OBJ_ISR}\"."
  ERR_INTHDR_DUPLICATE              = "\"#{TSR_PRM_INTNO}\" \"%s\" is duplicate.[%s]"
  ERR_ISR_PRM_MISMATCH              = "\"#{TSR_PRM_INTSTAT}\" is different between \"#{TSR_OBJ_ISR}\" \"#{TSR_PRM_INTNO}\" \"%s\" is same."
  ERR_TSKLIST_TASK_NOT_WAITING      = "\"%s\" is not \"#{TSR_STT_WAITING}\" or \"#{TSR_STT_W_SUSPEND}\"."
  ERR_TSKLIST_TASK_TARGET_MISMATCH  = "\"%s\"'s \"#{TSR_PRM_WOBJID}\" is not \"%s\"."
  ERR_MPF_NOT_ACTIVATE              = "\"%s\" that has variable \"%s\" is not activate."
  ERR_NO_CPU_STATE_IN_SPINLOCK      = "\"#{TSR_OBJ_CPU_STATE}\" is not defined during spinlock.(prc: %d)"
  ERR_NOT_CPU_LOCK_IN_SPINLOCK      = "\"#{TSR_PRM_LOCCPU}\" is false during spinlock.(prc: %d)"
  ERR_CANNOT_RUNNING_SUSPENDED      = "appropriate \"#{TSR_OBJ_CPU_STATE}\" or activate non-context is necessary to make #{TSR_PRM_TSKSTAT} #{TSR_STT_R_SUSPEND}."
  ERR_NO_TASK_CONTEXT_ON_OBJ        = "no activate \"#{TSR_OBJ_TASK}\" or \"#{TSR_OBJ_TASK_EXC}\" exist in spite of \"%s\" is activate."
  ERR_VARIABLE_NOT_DEFINED          = "\"%s\" of \"%s\" is not defined."
  ERR_VARIABLE_TYPE_MISMATCH        = "\"%s\" of \"%s\"'s type is not match.(expected: %s)"
  ERR_NOT_ACTIVATE_TASK_EXC         = "\"#{TSR_OBJ_TASK_EXC}\" can be activate."
  ERR_SET_ERRCODE_SET_CODE          = "return value can't be defined when processing defined by \"#{TSR_PRM_CODE}\"."
  ERR_SET_SYSCALL_AND_CODE          = "\"#{TSR_PRM_SYSCALL}\" and \"#{TSR_PRM_CODE}\" can't be defined in same condition."

  # シナリオチェック
  ERR_OBJECT_NOT_DEFINED_IN_PRE  = "object \"%s\" is not defined in pre_condition."
  ERR_VAR_NOT_DEFINED_IN_PRE     = "variable \"%s\" is not defined in pre_condition."
  ERR_OBJECT_NOT_ACTIVATE_PREV   = "\"%s\" is not activate object in prev condition."
  ERR_CANNOT_BE_RUNNING_WAITSPIN = "\"#{TSR_PRM_TSKSTAT}\" cannot be \"#{TSR_STT_R_WAITSPN}\" in pre_condition or last post_condition."
  ERR_DO_ID_MISMATCH_RUS_TASK    = "\"#{TSR_PRM_ID}\" must be non-context when rus-task and activate non-context exist.(expected: %s)"
  ERR_TIME_RETURN                = "timetick returns from %d to %d."
  ERR_TIME_PROGRESS_IN_CPU_LOCK  = "timetick progresses though cpulock.(timetick: %d to %d)"
  ERR_TIME_PROGRESS_CHG_IPM      = "timetick progresses though ipm is changed.(timetick: %d to %d)"
  ERR_TIME_PROGRESS_NON_CONTEXT  = "timetick progresses though activate non-context exists.(timetick: %d to %d)"
  ERR_NOT_DEFINED_ERROR_CODE     = "\"%s\" can check error code in later post_condition."
  ERR_CANNNOT_CHECK_ERROR_CODE   = "\"%s\" cannot check error code in later post_condition."
  ERR_PRCID_MACRO_COMBINATION    = "\"#{TSR_PRM_PRCID}\" macro's combination is invalid.(using: %s)"
  ERR_TIMEEVENT_PRCID_GLOBAL     = "all of time event handler's \"#{TSR_PRM_PRCID}\" must be single when \"#{TSR_PRM_TIMER_ARCH}\" is \"#{TSR_PRM_TIMER_GLOBAL}\""

  # シナリオ間チェック
  ERR_UNIFIED_ATR_INTHDR    = "\"#{TSR_OBJ_INTHDR}\"'s attributes must be unified."
  ERR_UNIFIED_ATR_ISR       = "\"#{TSR_OBJ_ISR}\"'s attributes must be unified."
  ERR_UNIFIED_ATR_EXCEPTION = "\"#{TSR_OBJ_EXCEPTION}\"'s attributes must be unified."
  ERR_CONFLICT_INTNO        = "\"#{TSR_PRM_INTNO}\" \"%s\" of \"#{TSR_OBJ_INTHDR}\" and \"#{TSR_OBJ_ISR}\" is conflict."
  ERR_OVER_SPINLOCK_SUM     = "spinlock sum is more than configure setting.\n(you can use emulate mode.)"
  ERR_OVER_INTNO_SUM        = "\"#{TSR_PRM_INTNO}\" sum is more than configure setting."

  # バリエーション設定
  ERR_VARIATION_INVALID_VALUE = "\"%s\" of variation is invalid value.[%s]"  # 値が不正
  ERR_VARIATION_INVALID_TYPE  = "\"%s\" of variation must be %s.[%s]"
  ERR_VARIATION_UNDEFINED_KEY = "\"%s\" of variation is undefined key."      # キーが未定義

  # バリエーションチェック
  ERR_VARIATION_NO_FUNC              = "\"%s\" is not available."
  ERR_VARIATION_OVER_PRC_NUM         = "processor number (configure: %d)[%d]"
  ERR_VARIATION_OVER_SPINLOCK_NUM    = "spinlock count is more than configure setting. (configure: %s)[%s]\n(you can use emulate mode.)"
  ERR_VARIATION_OVER_INTNO_NUM       = "\"#{TSR_PRM_INTNO}\" count is more than configure setting. (configure: %s)[%s]"
  ERR_VARIATION_NOT_MATCH_TIMER_ARCH = "timer architecture (configure: %s)[%s]"
  ERR_VARIATION_NOT_MATCH_IRC_ARCH   = "irc architecture (configure: %s)[%s]"
  ERR_VARIATION_TIME_GAIN            = "\"#{CFG_ALL_GAIN_TIME}\" is false and \"#{CFG_FUNC_TIME}\" is false"
  ERR_VARIATION_CANNOT_OWN_IPI       = "own ipi raise off"
  ERR_VARIATION_EXCEPT_IN_CPULOCK    = "cpu exception during cpulock"
  ERR_VARIATION_CHGIPM_IN_NONTASK    = "chg_ipm in non-task"
  ERR_VARIATION_CANNOT_CONTROL_TIME  = "can't control time"
  ERR_VARIATION_CANNOT_STOP_TIME     = "can't stop time"
  ERR_VARIATION_MUST_STOP_TIME       = "must stop time"
  ERR_VARIATION_NOT_SUPPORT_GET_UTM  = "\"#{API_GET_UTM}\" is not supported"
  ERR_VARIATION_NOT_SUPPORT_ENA_INT  = "\"#{API_ENA_INT}\" is not supported"
  ERR_VARIATION_NOT_SUPPORT_DIS_INT  = "\"#{API_DIS_INT}\" is not supported"
  ERR_VARIATION_STATUS_INVALID       = "interruput object can't change its status when interrupt API is not supported"

  #===================================================================
  # 関数
  #===================================================================

  # trueとfalseを一つのクラスで使えるようにするための真偽値マクロ
  Bool = [TrueClass, FalseClass]
  # dup，cloneできないクラス
  CANNOT_CLONE_CLASS = [Fixnum, NilClass, TrueClass, FalseClass, Symbol, Float]

  #=================================================================
  # 概　要: オブジェクトを複製可能な場合は複製
  #=================================================================
  def safe_dup(cObj)
    check_class(Object, cObj, true)  # 複製したいオブジェクト

    # 配列の場合全要素にも適用
    if (cObj.is_a?(Array))
      cObj = cObj.dup()
      cObj.each_with_index{|val, index|
        cObj[index] = safe_dup(val)
      }
      return cObj
    # ハッシュの場合，全値に適用
    elsif(cObj.is_a?(Hash))
      cObj = cObj.dup()
      cObj.each{|atr, val|
        cObj[atr] = safe_dup(val)
      }
      return cObj
    else
      # 複製できる場合はdup
      begin
        unless (CANNOT_CLONE_CLASS.include?(cObj.class))
          return cObj.dup()  # [Object]複製可能な場合は複製後の値，複製不可能な場合はそのまま
        else
          return cObj  # [Object]複製可能な場合は複製後の値，複製不可能な場合はそのまま
        end
      # 既知の複製できないクラスに漏れがあった場合の対策
      rescue TypeError
        return cObj
      end
    end
  end

  #===================================================================
  # 概　要: 変数の型をチェックするメソッド
  #===================================================================
  def check_class(obj, instance, nullable = false)
    # 指定したクラス属性の正当性チェックしてクラスではない場合はエラー終了
    if (obj.is_a?(Array))
      obj.each{|val|
        if (val.class != Class)
          raise(ArgumentError.new("#{obj * ", "} : \"#{val}\" is not Class"))
        end
      }
    elsif (obj.class != Class)
      raise(ArgumentError.new("#{obj} : \"#{obj}\" is not Class"))
    end

    # nullableがtrueまたはfalseではない場合はエラー終了
    unless (nullable.is_a?(FalseClass) || nullable.is_a?(TrueClass))
      raise(ArgumentError.new("nullable is either true or false"))
    end

    # nullableがfalseなのに変数がnilの場合はエラー終了
    if (instance.nil?() && (nullable == false))
      raise(ArgumentError.new("nullable condition is false"))
    elsif (instance.nil?() && (nullable == true))
      return # [Object]class正当性チェック完了の値
    end

    # 指定したクラス属性と変数のクラス型の正当性チェック
    if (obj.is_a?(Array))
      obj.each{|val|
        if (instance.is_a?(val))
          return # [Object]class正当性チェック完了の値
        end
      }

      raise(ArgumentError.new("class mismatch: #{instance.class} for #{obj * ", "}"))
    elsif (!instance.is_a?(obj))
      raise(ArgumentError.new("class mismatch: #{instance.class} for #{obj}"))
    end

    return # [Object]class正当性チェック完了の値
  end

  #===================================================================
  # 概　要: 処理の進捗を表示
  #===================================================================
  def print_progress(sPhase, sTestID, nCnt, nTotal)
    # 呼び出す処理が局所的のためcheck_classはしない
    Curses.init_screen

    # コンソールの幅取得
    nCols = Curses.cols

    # 進捗率計算
    fProgress = nCnt.to_f / nTotal.to_f

    # プログレスバーサイズ計算(幅-フェーズ-進捗率-最大テストID-括弧等)
    nProgressSize = nCols - TTG_MAX_TEST_ID_SIZE - 16

    # 整数にしたプログレスバー上の進捗サイズ
    nIntProg = (fProgress * nProgressSize).to_i

    # テストID文字数チェック
    if (TTG_MAX_TEST_ID_SIZE < sTestID.size())
      sTestID = sTestID.slice(0, TTG_MAX_TEST_ID_SIZE)
    end

    # 文字列作成
    sProgress = "[#{sPhase}]"
    sProgress += "#" * nIntProg
    sProgress += "-" * (nProgressSize - nIntProg)
    sProgress += ":#{"%5.1f" % (fProgress * 100)}\%"
    sProgress += " [#{sTestID}"
    sProgress += "\s"  * (TTG_MAX_TEST_ID_SIZE - sTestID.size())
    sProgress += "]"
    $stderr.print("\r#{sProgress}\r")

    Curses.close_screen
  end

  #===================================================================
  # 概　要: 進捗表示の終了
  #===================================================================
  def finish_progress(sPhase, nTotal)
    # 呼び出す処理が局所的のためcheck_classはしない
    Curses.init_screen

    # コンソールの幅取得
    nCols = Curses.cols

    # プログレスバーサイズ計算(幅-フェーズ-進捗率-最大テストID-括弧等)
    nProgressSize = nCols - TTG_MAX_TEST_ID_SIZE - 16

    # メッセージ作成
    if (sPhase == "TTC")
      sMsg = TTC_PROGRESS_MSG % nTotal
    else
      sMsg = TTG_PROGRESS_MSG % nTotal
    end

    # 文字列作成
    sProgress = "[#{sPhase}]"
    sProgress += "#" * nProgressSize
    sProgress += ":100.0\%"
    sProgress += " [#{sMsg}"
    sProgress += "\s"  * (TTG_MAX_TEST_ID_SIZE - sMsg.size())
    sProgress += "]"
    $stderr.print(sProgress)

    Curses.close_screen

    $stderr.print(TTG_NL)
  end

  #===================================================================
  # 概  要: コンソールの横幅チェック
  #===================================================================
  def check_console_width()
    Curses.init_screen

    # コンソールの幅取得
    nCols = Curses.cols

    # プログレスバーサイズ計算(幅-フェーズ-進捗率-最大テストID-括弧等)
    nSize = nCols - TTG_MAX_TEST_ID_SIZE - 17

    Curses.close_screen

    if (nSize > 0)
      return true  # [Bool]実行可能
    else
      return false  # [Bool]実行不可能
    end
  end
end
