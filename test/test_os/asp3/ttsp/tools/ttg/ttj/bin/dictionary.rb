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
#  $Id: dictionary.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/CommonModule.rb"

#=====================================================================
# TTJModule
#=====================================================================
module TTJModule
  include CommonModule

  #===================================================================
  # Variation
  #===================================================================
  GRP_TTJ_VARIATION = {
    TSR_PRM_GAIN_TIME       => "時間動作",
    TSR_PRM_TIMER_ARCH      => "タイマーアーキテクチャ",
    TSR_PRM_ENA_EXC_LOCK    => "CPUロック中のCPU例外発生",
    TSR_PRM_IRC_ARCH        => "IRCアーキテクチャ",
    TSR_PRM_GCOV_ALL        => "GCOV取得",
    TSR_PRM_SUPPORT_GET_UTM => "APIのget_utm",
    TSR_PRM_SUPPORT_ENA_INT => "APIのena_int",
    TSR_PRM_SUPPORT_DIS_INT => "APIのdis_int"
  }

  #===================================================================
  # Condition
  #===================================================================
  GRP_TTJ_CONDITION = {
    TSR_LBL_VARIATION => "バリエーション",
    TSR_LBL_PRE       => "前状態",
    TSR_LBL_DO        => "処理",
    TSR_LBL_POST      => "後状態"
  }

  #===================================================================
  # Object
  #===================================================================
  GRP_TTJ_OBJECT = [
    {TSR_OBJ_INIRTN      => "初期化ルーチン"        },
    {TSR_OBJ_TERRTN      => "終了ルーチン"          },

    {TSR_OBJ_ALARM       => "アラームハンドラ"      },
    {TSR_OBJ_CYCLE       => "周期ハンドラ"          },
    {TSR_OBJ_TASK        => "タスク"                },
    {TSR_OBJ_TASK_EXC    => "タスク例外処理ルーチン"},

    {TSR_OBJ_EXCEPTION   => "CPU例外ハンドラ"       },
    {TSR_OBJ_INTHDR      => "割込みハンドラ"        },
    {TSR_OBJ_ISR         => "割込みサービスルーチン"},

    {TSR_OBJ_SEMAPHORE   => "セマフォ"              },
    {TSR_OBJ_EVENTFLAG   => "イベントフラグ"        },
    {TSR_OBJ_DATAQUEUE   => "データキュー"          },
    {TSR_OBJ_P_DATAQUEUE => "優先度データキュー"    },
    {TSR_OBJ_MAILBOX     => "メールボックス"        },
    {TSR_OBJ_MEMORYPOOL  => "固定長メモリプール"    },
    {TSR_OBJ_SPINLOCK    => "スピンロック"          },

    {TSR_OBJ_CPU_STATE   => "CPU状態"               }
  ]

  #===================================================================
  # Attribute
  #===================================================================
  GRP_TTJ_ATTRIBUTE = [
    # TTJのID処理
    {TTJ_STT_ID       => TTJ_STT_ID                    },
    
    # 処理単位
    {TSR_PRM_TYPE     => "タイプ"                      },
    {TSR_PRM_ATR      => "属性"                        },
    {TSR_PRM_EXINF    => "拡張情報"                    },

    {TSR_PRM_STATE    => "状態"                        },
    {TSR_PRM_SPNSTAT  => "状態"                        },

    {TSR_PRM_HDLSTAT  => "実行状態"                    },
    {TSR_PRM_WOBJID   => "待ち要因"                    },
    {TSR_PRM_LEFTTMO  => "タイムアウト値"              },

    {TSR_PRM_ITSKPRI  => "初期優先度"                  },
    {TSR_PRM_TSKPRI   => "優先度"                      },

    {TSR_PRM_ACTCNT   => "起動要求キューイング"        },
    {TSR_PRM_WUPCNT   => "起床要求キューイング"        },
    {TSR_PRM_PORDER   => "優先順位"                    },
    {TSR_PRM_ACTPRC   => "次回起動時割付けプロセッサ"  },
    {TSR_PRM_BOOTCNT  => "起動回数"                    },
    {TSR_PRM_PRCID    => "割付けプロセッサ"            },
    {TSR_PRM_VAR      => "変数"                        },

    {TSR_PRM_CYCATR   => "属性"                        },
    {TSR_PRM_CYCTIM   => "周期"                        },
    {TSR_PRM_CYCPHS   => "位相"                        },

    {TSR_PRM_TASK     => "対象タスクID"                },
    {TSR_PRM_TEXPTN   => "タスク例外要因"              },
    {TSR_PRM_PNDPTN   => "保留例外要因"                },

    # 同期・通信オブジェクト
    {TSR_PRM_MAXSEM   => "最大資源数"                  },
    {TSR_PRM_ISEMCNT  => "初期資源数"                  },
    {TSR_PRM_SEMCNT   => "現在資源数"                  },

    {TSR_PRM_IFLGPTN  => "初期ビットパターン"          },
    {TSR_PRM_FLGPTN   => "現在ビットパターン"          },

    {TSR_PRM_DATACNT  => "格納できるデータ数"          },
    {TSR_PRM_MAXDPRI  => "データ優先度の最大値"        },

    {TSR_PRM_MAXMPRI  => "メッセージ優先度の最大値"    },
    {TSR_PRM_MSGLIST  => "受信待ちメッセージのリスト"  },

    {TSR_PRM_BLKCNT   => "獲得できるブロックの初期数"  },
    {TSR_PRM_FBLKCNT  => "獲得できるブロックの現在数"  },
    {TSR_PRM_BLKSZ    => "ブロックのサイズ"            },
    {TSR_PRM_MPF      => "固定長メモリプールの先頭番地"},

    {TSR_PRM_WTSKLIST => "待ちタスクのリスト"          },
    {TSR_PRM_STSKLIST => "送信待ちタスクのリスト"      },
    {TSR_PRM_RTSKLIST => "受信待ちタスクのリスト"      },
    {TSR_PRM_DATALIST => "管理領域のデータのリスト"    },

    {TSR_PRM_LOCCPU   => "CPUロック"                   },
    {TSR_PRM_DISDSP   => "ディスパッチ禁止"            },
    {TSR_PRM_CHGIPM   => "割込み優先度マスク"          },

    {TSR_PRM_SPINID   => "取得要求スピンロック"        },
    {TSR_PRM_PROCID   => "取得先"                      },

    {TSR_PRM_EXCNO    => "例外番号"                    },

    {TSR_PRM_INTNO    => "割込み番号"                  },
    {TSR_PRM_INHNO    => "割込みハンドラ番号"          },
    {TSR_PRM_INTPRI   => "割込み優先度"                },

    {TSR_PRM_ISRPRI   => "割込みサービスルーチン優先度"},

    {TSR_PRM_DO       => "処理"                        },
    {TSR_PRM_GLOBAL   => "グローバル"                  },

    {TSR_PRM_CLASS    => "所属クラス"                  }
  ]

  #===================================================================
  # Status
  #===================================================================
  GRP_TTJ_STATUS = {
    # オブジェクトの状態
    KER_TTS_RUN       => "実行",
    KER_TTS_RDY       => "実行可能",
    KER_TTS_WAI       => "待ち",
    KER_TTS_SUS       => "強制待ち",
    KER_TTS_WAS       => "二重待ち",
    KER_TTS_RUS       => "強制待ち[実行継続中]",
    KER_TTS_DMT       => "休止",
    TSR_STT_R_WAITSPN => "スピンロック取得待ち[実行継続中]",
    TSR_STT_A_WAITSPN => "スピンロック取得待ち[実行継続中]",

    TSR_PRM_STATE => {
      TSR_OBJ_TASK     => "状態",
      TSR_OBJ_ALARM    => "動作状態",
      TSR_OBJ_CYCLE    => "動作状態",
      TSR_OBJ_TASK_EXC => "タスク例外禁止フラグ状態",
      TSR_OBJ_INTHDR   => "割込み禁止フラグ状態",
      TSR_OBJ_ISR      => "割込み禁止フラグ状態"
    },

    # 状態
    TSR_STT_TALM_STA  => "起動中",
    TSR_STT_TALM_STP  => "停止中",
    TSR_STT_TCYC_STA  => "起動中",
    TSR_STT_TCYC_STP  => "停止中",
    TSR_STT_TTEX_ENA  => "許可状態",
    TSR_STT_TTEX_DIS  => "禁止状態",
    TSR_STT_TA_ENAINT => "クリア",
    TSR_STT_TA_DISINT => "セット",
    TSR_STT_ACTIVATE  => "実行中",
    TSR_STT_STP       => "停止中",
    TSR_STT_SLEEP     => "起床待ち",
    TSR_STT_DELAY     => "時間経過待ち",

    # 属性
    TSR_PRM_ATR => {
      KER_TA_NULL   => "オブジェクト属性を指定しない．",
      KER_TA_TPRI   => "タスクの待ち行列をタスクの優先度順にする．",
      KER_TA_WMUL   => "複数タスクが待つのを許す．",
      KER_TA_CLR    => "タスクの待ち解除時にイベントフラグをクリアする．",
      KER_TA_MPRI   => "メッセージキューをメッセージの優先度順にする．",
      KER_TA_STA    => "周期ハンドラの生成時に周期ハンドラを動作開始する．",
      KER_TA_ENAINT => "割込み要求禁止フラグをクリアする．",
      KER_TA_DISINT => "割込み要求禁止フラグをセットする．"
    },

    TSR_PRM_SYSCALL => "システムコール",
    TSR_PRM_BOOL    => "カーネル非動作状態",

    # trueの区別
    TSR_STT_TRUE => {
      TSR_PRM_LOCCPU          => "ロック状態",
      TSR_PRM_DISDSP          => "禁止状態",
      TSR_PRM_GAIN_TIME       => "時間進行",
      TSR_PRM_ENA_EXC_LOCK    => "CPUロック中のCPU例外発生を許可",
      TSR_PRM_DO              => "カーネル非動作中",
      TSR_PRM_GLOBAL          => {TSR_OBJ_INIRTN => "グローバル初期化ルーチンにする．",
                                  TSR_OBJ_TERRTN => "グローバル終了ルーチンにする．"},
      TSR_PRM_GCOV_ALL        => "すべての処理を取得",
      TSR_PRM_GCOV            => "取得する．",
      TSR_PRM_SUPPORT_GET_UTM => "APIのget_utmをサポートする．",
      TSR_PRM_SUPPORT_ENA_INT => "APIのena_intをサポートする．",
      TSR_PRM_SUPPORT_DIS_INT => "APIのdis_intをサポートする．"
    },

    # falseの区別
    TSR_STT_FALSE => {
      TSR_PRM_LOCCPU          => "ロック解除状態",
      TSR_PRM_DISDSP          => "禁止解除状態",
      TSR_PRM_GAIN_TIME       => "時間停止",
      TSR_PRM_ENA_EXC_LOCK    => "CPUロック中のCPU例外発生を禁止",
      TSR_PRM_DO              => "カーネル動作中",
      TSR_PRM_GLOBAL          => {TSR_OBJ_INIRTN => "グローバル初期化ルーチンにしない．",
                                  TSR_OBJ_TERRTN => "グローバル終了ルーチンにしない．"},
      TSR_PRM_GCOV_ALL        => "該当の処理のみ取得",
      TSR_PRM_GCOV            => "取得しない．",
      TSR_PRM_SUPPORT_GET_UTM => "APIのget_utmをサポートしない．",
      TSR_PRM_SUPPORT_ENA_INT => "APIのena_intをサポートしない．",
      TSR_PRM_SUPPORT_DIS_INT => "APIのdis_intをサポートしない．"
    },

    # 割込み優先度マスクの状態
    TSR_PRM_CHGIPM => {
      KER_TIPM_ENAALL => "全解除状態",
      0               => "全解除状態"
    },

    # variationのタイマーと割込みの状態
    TSR_PRM_TIMER_GLOBAL    => "グローバル",
    TSR_PRM_TIMER_LOCAL     => "ローカル",
    TSR_PRM_IRC_GLOBAL      => "グローバル",
    TSR_PRM_IRC_LOCAL       => "ローカル",
    TSR_PRM_IRC_COMBINATION => "グローバルとローカル",

    # タスク例外処理の状態
    TSR_STT_TTEX_DIS => "禁止状態",
    TSR_STT_TTEX_ENA => "許可状態",

    # スピンロックの状態
    TSR_STT_TSPN_LOC => "取得されている状態",
    TSR_STT_TSPN_UNL => "取得されていない状態",

    # プロセッサ
    CFG_MCR_PRC_SELF    => "自プロセッサ",
    CFG_MCR_PRC_OTHER   => "他プロセッサ",
    CFG_MCR_PRC_OTHER_1 => "他プロセッサ_1",
    CFG_MCR_PRC_OTHER_2 => "他プロセッサ_2",

    # イベントフラグの待ちモード
    KER_TWF_ORW  => "OR",
    KER_TWF_ANDW => "AND",

    # 変数タイプ
    TYP_VOID_P      => "固定長メモリブロックの先頭番地(void*)",
    TYP_INTPTR_T    => "ポインタを格納できる符号付き整数(intptr_t)",
    TYP_PRI         => "優先度(PRI)",
    TYP_ID          => "タスクIDを入れるメモリ領域へのポインタ(ID)",
    TYP_FLGPTN      => "イベントフラグのビットパターン(FLGPTN)",
    TYP_T_RALM      => "アラームハンドラの現在状態を入れるパケット(T_RALM)",
    TYP_T_RCYC      => "周期ハンドラの現在状態を入れるパケット(T_RCYC)",
    TYP_T_RTSK      => "タスクの現在状態を入れるパケット(T_RTSK)",
    TYP_T_TTSP_RTSK => "タスクの現在状態を入れるパケット(T_TTSP_RTSK)",
    TYP_T_RTEX      => "タスク例外処理の現在状態を入れるパケット(T_RTEX)",
    TYP_T_RSEM      => "セマフォの現在状態を入れるパケット(T_RSEM)",
    TYP_T_RFLG      => "イベントフラグの現在状態を入れるパケット(T_RFLG)",
    TYP_T_RDTQ      => "データキューの現在状態を入れるパケット(T_RDTQ)",
    TYP_T_RPDQ      => "優先度データキューの現在状態を入れるパケット(T_RPDQ)",
    TYP_T_RMBX      => "メールボックスの現在状態を入れるパケット(T_RMBX)",
    TYP_T_RMPF      => "固定長メモリプールの現在状態を入れるパケット(T_RMPF)",
    TYP_T_RSPN      => "スピンロックの現在状態をいれるパケット(T_RSPN)",
    TYP_T_MSG       => "メールボックスのメッセージヘッダ(T_MSG)",
    TYP_T_P_MSG     => "メールボックスのメッセージヘッダポインタ(T_MSG*)",
    TYP_T_MSG_PRI   => "優先度付きメッセージヘッダ(T_MSG_PRI)",
    TYP_SYSTIM      => "システム時刻を入れるメモリ領域(SYSTIM)",
    TYP_SYSUTM      => "システム時刻を入れるメモリ領域(SYSUTM)"
  }
end
