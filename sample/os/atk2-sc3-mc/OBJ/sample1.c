/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2015 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2015 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2015 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2015 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2015 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2004-2015 by Witz Corporation
 *  Copyright (C) 2014-2015 by AISIN COMCRUISE Co., Ltd., JAPAN
 *  Copyright (C) 2014-2015 by eSOL Co.,Ltd., JAPAN
 *  Copyright (C) 2014-2015 by SCSK Corporation, JAPAN
 *  Copyright (C) 2015 by SUZUKI MOTOR CORPORATION
 *
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 *
 *  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
 *  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
 *  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
 *  用する者に対して，AUTOSARパートナーになることを求めている．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: sample1.c 425 2015-12-07 08:06:19Z witz-itoyo $
 */

/*
 *		サンプルプログラム(1)の本体
 *
 *  ATK2-SC3の基本的な動作を確認するためのサンプルプログラム．
 *
 *  プログラムの概要：PCとボード間でシリアル通信を行い，ユーザ入力
 *  コマンドに応じた動作とログ出力を行なう．
 *
 *  ＜コマンド一覧＞
 *  ・タスク指定
 *    '1' : 以降のコマンドを Task1 に対して行う
 *    '2' : 以降のコマンドを Task2 に対して行う
 *    '3' : 以降のコマンドを Task3 に対して行う
 *    '4' : 以降のコマンドを Task4 に対して行う
 *    '5' : 以降のコマンドを Task5 に対して行う
 *    '6' : 以降のコマンドを Task6 に対して行う
 *    '7' : 以降のコマンドを Task7 に対して行う
 *    '8' : 以降のコマンドを Task8 に対して行う(一部タスク管理コマンド使用できない)
 *    '9' : 以降のコマンドを Task9 に対して行う(一部タスク管理コマンド使用できない)
 *  ・タスク管理機能
 *    'a' : ActivateTask にてタスクを起動する．
 *    'A' : TerminateTask にてタスクを終了する．
 *    '!'(Shift+1) : ChainTask にてタスクを終了し，Task1 を起動する
 *    '"'(Shift+2) : ChainTask にてタスクを終了し，Task2 を起動する
 *    '#'(Shift+3) : ChainTask にてタスクを終了し，Task3 を起動する
 *    '$'(Shift+4) : ChainTask にてタスクを終了し，Task4 を起動する
 *    '%'(Shift+5) : ChainTask にてタスクを終了し，Task5 を起動する
 *    's' : ノンプリエンプティブ属性であるタスク MainTask にて最高
 *          優先度タスク HighPriorityTask を起動し，Schedule にて
 *          再スケジューリングを行う．
 *    'S' : ノンプリエンプティブタスク NonPriTask を起動する
 *          ノンプリエンプティブタスク NonPriTask にて最高優先度
 *          タスク HighPriorityTask を起動し，タスク終了する
 *    'y' : タスク11(スタック共有)を呼び出し，信頼関数で処理を行う
 *    'Y' : タスク12(スタック共有)を呼び出し，信頼関数で処理を行う
 *          タスク不正終了により，エラーコードはE_OS_MISSINGENDで
 *          エラーフックが呼ばれる
 *    'z' : 実行中のタスクから GetTaskID を実行して，実行状態の TaskID を取得する
 *    'Z' : メインタスクから GetTaskState を実行して，対象タスクの状態を取得する
 *    'x' : div命令を実行して，未定義命令例外を発生させる
 *  ・割込み管理機能
 *    'd' : DisableAllInterrupts を実行後，割込みタイマ値を3回表
 *          示し， EnableAllInterrupts を実行する
 *    'D' : SuspendAllInterrupts を実行後，割込みタイマ値を3回表
 *          示し，さらに SuspendAllInterrupts を実行後，割込みタイ
 *          マー値を3回表示し，ResumeAllInterrupts を実行後，割込み
 *          タイマ値を3回表示し，ResumeAllInterrupts を実行する
 *    'f' : SuspendOSInterrupts を実行後，割込みタイマ値を3回表
 *          示し，さらに SuspendOSInterrupts を実行後，割込みタイ
 *          マー値を3回表示し，さらに SuspendAllInterruptsを実行
 *          後，割込みタイマ値を3回表示し， ResumeAllInterrupts
 *          を実行後，割込みタイマ値を3回表示し，
 *          ResumeOSInterrupts を実行後割込みタイマ値を3回表示し，
 *          ResumeOSInterrupts を実行する
 *    'T' : 割込みタイマ値を3回表示する
 *  ・リソース管理機能
 *    'k' : GetResource にてリソース TskLevelRes を取得する．なお，
 *          Task3 は，このリソースより優先度が高いためエラーとなる
 *    'K' : ReleaseResource にてリソース TskLevelRes を解放する
 *    'i' : GetResource にてリソース CntRes を取得後，割込み
 *          タイマ値を3回表示し，ReleaseResource にてリソース
 *           CntRes を解放する
 *  ・イベント制御機能
 *    'e' : SetEvent にてイベントを設定する．Task2 と Task3 以外は
 *          割り当てがないためエラーとなる
 *    'w' : ClearEvent にて自タスクのイベントをクリアする．Task2 と
 *          Task3 以外は割り当てがないためエラーとなる
 *    'E' : GetEvent にてイベント状態を取得する．Task2 と Task3
 *          以外は割り当てがないためエラーとなる
 *    'W' : WaitEvent にて自タスクのイベントを待つ．Task3 と Task4
 *          以外はエラーとなる
 *  ・アラーム機能
 *    'b' : GetAlarmBase にてアラーム MainCycArm のアラームベース
 *          情報を取得する
 *    'B' : GetAlarm にてアラーム MainCycArm の残りカウント値を2回
 *          連続で取得する
 *    'v' : SetRelAlarm にてアラーム ActTskArm を起動し，500ms 後に
 *          タスク Task1 を起動する
 *    'V' : SetRelAlarm にてアラーム SetEvtArm を起動し，500ms 後に
 *          イベント T3Evt を設定する
 *    'h' : CancelAlarm にてアラーム ActTskArm をキャンセルする
 *  ・カウンタ操作機能
 *    'c' : タスクにて IncrementCounter を実行し，カウンタ SampleCnt
 *          にシグナル通知する
 *          1 シグナルでアラーム SampleArm が満了し，コールバックを
 *          実行する
 *    'C' : タスクにて IncrementCounter を実行し，カウンタ SampleCnt2
 *          にシグナル通知する
 *          1 シグナルでアラーム SampleArm1 が満了し，SampleCnt3 を
 *          インクリメントする． SampleCnt3 は 1 シグナルで SampleAlm2
 *          が満了しコールバックを実行する
 *    'j' : タスクにて GetCounterValue を実行する
 *          取得した値は保存する
 *    'J' : タスクにて GetEalsedCounterValue を実行する
 *          元の値は前回のGetCounterValue または GetEalsedCounterValue にて取得
 *          した値とする．初回，または，GetCounterValueを実行していない場合は0
 *          とする
 *
 *    'r' : タスクにて GetISRIDを実行する
 *          タスクコンテキストからはGetISRIDを呼出した場合はエラーとして
 *          INVALID_ISRが返却される
 *
 *          割込みコンテキストからのGetISRID発行は，SystemTimerCntのコールバック
 *          ルーチン（'n' コマンドで実行）にて行う
 *
 *          タスクコンテキストからのGetISRID発行は，SampleCnt2のコールバック
 *          ルーチン（'C' または 'c' コマンドで実行）にて行う
 *
 *  ・スケジュールテーブル制御機能
 *    't' : スケジュールテーブル動作制御ON/OFF
 *
 *        'i' : SchtblSampleCntカウンタをインクリメントする
 *              SchtblSampleCntカウンタはスケジュールテーブルの駆動カウンタ
 *        '1' : 以降のコマンドを scheduletable1 に対して行う(デフォルト値)
 *        '2' : 以降のコマンドを scheduletable2 に対して行う
 *        '3' : 以降のコマンドを scheduletable3(存在しないID) に対して行う
 *              コマンドのエラー確認にも使用する
 *        's' : StartScheduleTableRelを実行し，相対値(5)でスケジュールテーブルを開始する
 *        'S' : StartScheduleTableAbsを実行し，絶対値(5)でスケジュールテーブルを開始する
 *        'f' : StopScheduleTableを実行し，スケジュールテーブルを停止する
 *        'n' : NextScheduleTableを実行し，scheduletable2に切替える
 *        'N' : NextScheduleTableを実行し，scheduletable1に切替える
 *        'g' : GetScheduleTableStatusを実行し，スケジュールテーブル状態を取得する
 *        'q' : ShutdownAllCores( E_OK ) OS実行制御機能の'q'と同等機能
 *        'Q' : ShutdownAllCores( E_OS_STATE ) OS実行制御機能の'Q'と同等機能
 *
 *  ・IOC送受信機能
 *    'I' : IOC送受信モード
 *
 *        IOC APIを使用するタスク(IocTaskX)を選択
 *        '1' : IocTask1(OSAP_CORE0に所属)
 *        '2' : IocTask1(OSAP_CORE0に所属)
 *        '3' : IocTask1(OSAP_CORE1に所属)
 *        '4' : IocTask1(OSAP_CORE1に所属)
 *
 *        IOC APIを選択
 *        '1' : IocSend_IOC_QUE_0(uint8 in1)
 *        '2' : IocSend_IOC_QUE_1(uint8 in1)
 *        '3' : IocWriteGroup_IOC_DEQUE(uint8 in1, uint8 in2, uint8 in3)
 *        '4' : IocReceive_IOC_QUE(uint8 *out1)
 *        '5' : IocReadGroup_IOC_DEQUE(uint8 *out1, uint8 *out2, uint8 *out3)
 *        '6' : IocEmptyQueue_IOC_QUE(void)
 *
 *        送信API選択時，送信データを入力する
 *
 *        また，IocTask内でコマンドを受け付けるため，本モード中ではMainTaskは待ち状態(IocEvt)となる
 *
 *  ・OS実行制御機能
 *    'p' : GetActiveApplicationMode にてアプリケーションモードを
 *          取得する
 *    'q' : ShutdownAllCores をコード E_OK で実行し，サンプルプログラムを
 *          終了する
 *    'Q' : ShutdownAllCores をコード E_OS_STATE で実行し，サンプルプログラム
 *          を終了する
 *
 *  ＜オブジェクト一覧＞
 *  ・OS
 *    スタートアップフック：使用
 *    シャットダウンフック：使用
 *    エラーフック：使用
 *    プレタスクフック：未使用
 *    ポストタスクフック：未使用
 *  ・タスク
 *  メインタスク
 *    タスクID：MainTask
 *    優先度：14
 *    多重起動数：1
 *    スケジュール：ノンプリエンプティブ
 *    信頼OSアプリケーションに所属(コア1所属)
 *    自動起動：AppMode1, AppMode2, AppMode3
 *    概要：ユーザインタフェース（シリアルIOよりコマンドを受信し，
 *          それに対応した動作を行なう）
 *          周期アラーム MainCycArm により，10msごとに待ち解除し
 *          コマンドの受信有無をポーリングする
 *          イベント（ID：MainEvt0, IocEvt）を関連付けている
 *  最高優先度タスク
 *    タスクID：HighPriorityTask
 *    優先度：15
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    信頼OSアプリケーションに所属(コア1所属)
 *    自動起動：なし
 *    概要：起動ログを出力して終了する．ノンプリエンプティブタスク
 *          から起動され，プリエンプトしているかどうかの確認用
 *  ノンプリエンプティブタスク
 *    タスクID：NonPriTask
 *    優先度：1
 *    多重起動数：8
 *    スケジュール：ノンプリエンプティブ
 *    自動起動：なし
 *    信頼OSアプリケーションに所属(コア1所属)
 *    概要：起動ログを出力し，最高優先度タスク HighPriorityTask を
 *          起動後，終了ログを出力してタスクを終了する
 *  タスク1
 *    タスクID：Task1
 *    優先度：4
 *    スケジュール：フルプリエンプティブ
 *    信頼OSアプリケーションに所属(コア1所属)
 *    自動起動：AppMode2
 *    多重起動数：8
 *    概要：並列処理タスク（メインタスクからの指令により動作）
 *          起動されするとスピンロックを取得し，Task9を起動してから，
 *          lock_var1とlock_var2変数をそれぞれインクリメットした後，
 *          スピンロックを解放する．その後，自コアと他コアに対して，
 *          それぞれコア間割込みを起こしてから無限ループに入り，コマ
 *          ンド処理を実行する
 *          リソース TskLevelRes を関連付けている
 *          リソース CntRes を関連付けている
 *  タスク2
 *    タスクID：Task2
 *    優先度：7
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    信頼OSアプリケーションに所属(コア1所属)
 *    自動起動：なし
 *    概要：並列処理タスク（メインタスクからの指令により動作）
 *          起動されすると無限ループに入り，コマンド処理を実行する
 *          リソース TskLevelRes を関連付けている
 *          リソース CntRes を関連付けている
 *          イベント T2Evt を関連付けている
 *  タスク3
 *    タスクID：Task3
 *    優先度：12
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    信頼OSアプリケーションに所属(コア1所属)
 *    自動起動：AppMode3
 *    概要：並列処理タスク（メインタスクからの指令により動作）
 *          起動されすると無限ループに入り，コマンド処理を実行する
 *          イベント待ちすることが可能である
 *          リソース CntRes を関連付けている
 *          イベント T3Evt を関連付けている
 *  タスク4
 *    タスクID：Task4
 *    優先度：6
 *    多重起動数：5
 *    スケジュール：フルプリエンプティブ
 *    信頼OSアプリケーションに所属(コア1所属)
 *    自動起動：なし
 *    概要：並列処理タスク（メインタスクからの指令により動作）
 *          起動されすると無限ループに入り，コマンド処理を実行する
 *          リソース TskLevelRes を関連付けている
 *          リソース CntRes を関連付けている
 *          内部リソース GroupRes を関連付けている
 *  タスク5
 *    タスクID：Task5
 *    優先度：9
 *    多重起動数：5
 *    スケジュール：フルプリエンプティブ
 *    信頼OSアプリケーションに所属(コア1所属)
 *    自動起動：なし
 *    概要：並列処理タスク（メインタスクからの指令により動作）
 *          起動されすると無限ループに入り，コマンド処理を実行する
 *          リソース TskLevelRes を関連付けている
 *          リソース CntRes を関連付けている
 *          内部リソース GroupRes を関連付けている
 *  タスク6
 *    タスクID：Task6
 *    優先度：9
 *    多重起動数：-
 *    スケジュール：フルプリエンプティブ
 *    自動起動：AppMode1，AppMode2，AppMode3
 *    信頼OSアプリケーションに所属(コア0所属)
 *    概要：並列処理タスク
 *          起動されする文字入力受信待ちになり，受信した場合，
 *          受信文字を出力した上，自分自身をChainTaskする
 *          スタックはユーザ指定
 *  タスク7
 *    タスクID：Task7
 *    優先度：9
 *    多重起動数：-
 *    スケジュール：フルプリエンプティブ
 *    自動起動：AppMode1，AppMode2，AppMode3
 *    信頼OSアプリケーションに所属(コア0所属)
 *    概要：自動起動とメインタスクからの指令により動作
 *          起動されすると即時関数を終了する
 *          この場合でも正しくTerminateTaskが実行されるかを確認
 *          何度起動してもOKとなる
 *  タスク8
 *    タスクID：Task8
 *    優先度：13
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    信頼OSアプリケーションに所属(コア1所属)
 *    自動起動：なし
 *    概要：起動メッセージ出力し，イベント(T8Evt)を待ち
 *  タスク9
 *    タスクID：Task9
 *    優先度：13
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    信頼OSアプリケーションに所属(コア1所属)
 *    自動起動：なし
 *    概要：起動メッセージ出力し，イベント(T9Evt)を待ち
 *  タスク10
 *    タスクID：Task10
 *    優先度：13
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    信頼OSアプリケーションに所属(コア1所属)
 *    自動起動：なし
 *    概要：起動メッセージ出力し，イベント(T10Evt)を待ち
 *  タスク11
 *    タスクID：Task11
 *    優先度：9
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    信頼OSアプリケーションに所属(コア1所属)
 *    自動起動：なし
 *    概要：信頼関数動作確認用
 *          非信頼OSアプリケーションに所属
 *          Task12とスタックを共有
 *  タスク12
 *    タスクID：Task12
 *    優先度：8
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    信頼OSアプリケーションに所属(コア1所属)
 *    自動起動：なし
 *    概要：信頼関数動作確認用
 *          非信頼OSアプリケーションに所属
 *          Task11とスタックを共有
 *  タスク13
 *    タスクID：Task13
 *    優先度：11
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：AppMode1，AppMode2，AppMode3
 *    概要：非信頼タスク動作確認用
 *          非信頼OSアプリケーションに所属(コア0所属)
 *          起動メッセージ出力後，ChainTask(Task14)発行
 *  タスク14
 *    タスクID：Task14
 *    優先度：10
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：非信頼タスク動作確認用
 *          非信頼OSアプリケーションに所属(コア0所属)
 *          Task13により起動されメッセージ出力後終了
 *  タスク15
 *    タスクID：Task15
 *    優先度：11
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：AppMode1，AppMode2，AppMode3
 *    概要：非信頼タスク動作確認用
 *          非信頼OSアプリケーションに所属(コア1所属)
 *          起動メッセージ出力後，ChainTask(Task16)発行
 *  タスク16
 *    タスクID：Task16
 *    優先度：10
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：非信頼タスク動作確認用(コア1所属)
 *          非信頼OSアプリケーションに所属
 *          Task15により起動されメッセージ出力後終了
 *  タスク17
 *    タスクID：Task17
 *    優先度：12
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：起動メッセージ出力し，スピンロック取得せずlock_var1変数を
 *          インクリメントして，スピンロックを取得後lock_var2変数を
 *          インクリメットしてからlock_var1とlock_var2の値を表示し，
 *          スピンロックを解放し，終了する
 *          数回実行して見れば，lock_var1は不定値となり，lock_var2は
 *          固定値となる
 *  IocTask1
 *    タスクID：IocTask1
 *    優先度：9
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：IOC通信処理(IOC APIの選択，送信データ設定，実行)を行う
 *          信頼OSアプリケーションに所属(コア0所属)
 *          イベント（ID：IocEvt）を関連付けている
 *  IocTask2
 *    タスクID：IocTask2
 *    優先度：9
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：IOC通信処理(IOC APIの選択，送信データ設定，実行)を行う
 *          非信頼OSアプリケーションに所属(コア0所属)
 *          イベント（ID：IocEvt）を関連付けている
 *  IocTask3
 *    タスクID：IocTask3
 *    優先度：9
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：IOC通信処理(IOC APIの選択，送信データ設定，実行)を行う
 *          信頼OSアプリケーションに所属(コア1所属)
 *          イベント（ID：IocEvt）を関連付けている
 *  IocTask4
 *    タスクID：IocTask4
 *    優先度：9
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：IOC通信処理(IOC APIの選択，送信データ設定，実行)を行う
 *          非信頼OSアプリケーションに所属(コア1所属)
 *          イベント（ID：IocEvt）を関連付けている
 *
 *  ・割込みサービスルーチン
 *  コア0のシリアルIO受信割込み
 *    ISRID：RxHwSerialInt0
 *    優先度：2
 *    カテゴリ：2
 *    概要：コマンドを受信する
 *  コア1のシリアルIO受信割込み
 *    ISRID：RxHwSerialInt1
 *    優先度：2
 *    カテゴリ：2
 *    概要：文字を受信する
 *  ・リソース
 *  タスクレベルリソース
 *    リソースID：TskLevelRes
 *    プロパティ：標準
 *  タスク用カウンタリソース
 *    リソースID：CntRes
 *    プロパティ：標準
 *  タスクグループリソース
 *    リソースID：GroupRes
 *    プロパティ：内部
 *  ・イベント
 *  メインタスクイベント
 *    イベントID：MainEvt
 *  タスク2イベント
 *    イベントID：T2Evt
 *  タスク3イベント
 *    イベントID：T3Evt
 *  ・カウンタ
 *  コア0のシステムタイマカウンタ
 *    カウンタID：MAIN_HW_COUNTER_CORE0
 *    カウント値：0〜999
 *    加算値：1
 *  コア1のシステムタイマカウンタ
 *    カウンタID：MAIN_HW_COUNTER_CORE1
 *    カウント値：0〜999
 *    加算値：1
 *  サンプルカウンタ
 *    カウンタID：SampleCnt
 *    カウント値：0〜99
 *    加算値：10
 *  ・アラーム
 *  メイン周期アラーム
 *    アラームID：MainCycArm
 *    ベースカウンタID：MAIN_HW_COUNTER_CORE1
 *    アクション：イベント設定 MainEvt
 *    自動起動：なし
 *  タスク起動アラーム
 *    アラームID：ActTskArm
 *    ベースカウンタID：MAIN_HW_COUNTER_CORE1
 *    アクション：タスク起動 Task1
 *    自動起動：なし
 *  イベント設定アラーム
 *    アラームID：SetEvtArm
 *    ベースカウンタID：MAIN_HW_COUNTER_CORE1
 *    アクション：イベント設定 T3Evt
 *    自動起動：なし
 *  IncrementCounter確認用アラーム
 *    アラームID：SampleArm
 *    ベースカウンタID：SampleCnt
 *    アクション：コールバック関数実行
 *    自動起動：なし
 *  ・アプリケーションモード
 *  自動起動なしモード
 *    アプリケーションモードID：AppMode1
 *  Task1自動起動モード
 *    アプリケーションモードID：AppMode2
 *  Task3自動起動モード
 *    アプリケーションモードID：AppMode3
 *  ・信頼関数
 *    信頼関数tfnt1
 *      非信頼OSアプリケーションから渡された
 *      ポインタ引数の読書き操作して，計算結果を
 *      非信頼OSアプリケーションに返す
 *    信頼関数tfnt2
 *      非信頼OSアプリケーションから渡された
 *      ポインタ引数の読書き操作して，計算結果を
 *      非信頼OSアプリケーションに返す
 *    信頼関数tfnt3
 *      非信頼OSアプリケーションからのShutdownOSを
 *      受け付ける
 *    信頼関数tfnt4
 *      非信頼OSアプリケーションから信頼OSアプリ
 *      ケーションに所属するタスクを起動する
 *
 *  本ファイルはコア0に所属しているオブジェクト,
 *  sample2.cではコア1に所属しているオブジェクト
 *  を記述している
 */

#include "Os.h"
#include "t_syslog.h"
#include "t_stdlib.h"
#include "sysmod/serial.h"
#include "sysmod/syslog.h"
#include "Ioc.h"
#include "sample1.h"

#include "sysmod/banner.h"
#include "target_sysmod.h"
#include "target_serial.h"

extern volatile sint32	lock_var1;          /* スピンロック動作確認用グローバル変数1 */
extern volatile sint32	lock_var2;          /* スピンロック動作確認用グローバル変数2 */

extern void TaskProk(uint8 task_no);
extern uint8 GetCommand(void);
extern void PutActTsk(uint8 task_no);
extern void PutTermTsk(uint8 task_no);
extern void PutChainTsk(uint8 from_task_no, uint8 to_task_no);
extern void PutSchedule(void);
extern void PutTaskID(void);
extern void PutTaskState(uint8 task_no);
extern void PutDisAllInt(void);
extern void PutSusAllInt(void);
extern void PutSusOSInt(void);
extern void PutHwCnt3(void);
extern void PutGetIntRes(void);
extern void PutGetTskRes(void);
extern void PutRelTskRes(void);
extern void PutSetEvt(uint8 task_no);
extern void PutClrEvt(uint8 task_no);
extern void PutGetEvt(uint8 task_no);
extern void PutWaitEvt(uint8 task_no);
extern void PutArmBase(void);
extern void PutArmTick(void);
extern void PutSetRel(uint8 alarm_no, uint8 tick_no, uint8 cycle_no);
extern void PutCanArm(void);
extern void PutAppMode(void);
extern void schedule_table_sample_routine(void);
extern void PutActNonPriTsk(void);
static void IocProk1(void);
static uint8 GetCommandNoWait1(void);


/*
 *  内部関数プロトタイプ宣言
 */
TASK(NonPriTask);
TASK(Task6);
TASK(Task7);
TASK(IocTask3);
TASK(IocTask4);

DEFINE_VAR_SSTACK(StackType, stack_00[COUNT_STK_T(0x200)]);

/*
 *  APIエラーログマクロ
 *
 *  ErrorHookが有効の場合はErrorHookから
 *  エラーログを出力し, ErrorHookが無効の場合は
 *  以下のマクロよりエラーログ出力を行う
 */
#if defined(CFG_USE_ERRORHOOK)
#define error_log(api)	(api)
#define error_log_ioc(api)	(api)
#else /* !defined( CFG_USE_ERRORHOOK ) */
#define	error_log(api)										   \
	{														   \
		StatusType ercd;									   \
		ercd = api;     /* 各API実行 */						   \
		if (ercd != E_OK) {									   \
			syslog(LOG_INFO, "Error:%d", atk2_strerror(ercd)); \
		}													   \
	}

#define	error_log_ioc(api)										   \
	{															   \
		StatusType ercd;										   \
		ercd = api;     /* 各API実行 */							   \
		if (ercd != E_OK) {										   \
			syslog(LOG_INFO, "Error:%d", atk2_ioc_strerror(ercd)); \
		}														   \
	}
#endif /* defined( CFG_USE_ERRORHOOK ) */

/*
 *  パラメータ構造体の定義
 *  信頼関数提供者がヘッダファイルで提供する
 */

struct parameter_struct {
	sint32		*name1;
	sint32		name2;
	TaskType	task_no;
	StatusType	ercd;
	StatusType	return_value;   /* 返戻値は構造体に含む */
};

/*
 *  信頼関数
 */

TRUSTEDFUNCTION(TRUSTED_tfnt1, FunctionIndex, FunctionParams)
{
	struct parameter_struct *local = FunctionParams;

	sint32					*p1;
	sint32					p2;

	p1 = local->name1;
	p2 = local->name2;

	local->return_value = (StatusType) ((*p1 * 2) + p2);
	return(E_OK);
}

/*
 *  信頼関数tfnt2
 *    非信頼OSアプリケーションから渡された
 *    ポインタ引数の読書き操作して，計算結果を
 *    非信頼OSアプリケーションに返す
 */
TRUSTEDFUNCTION(TRUSTED_tfnt2, FunctionIndex, FunctionParams)
{
	struct parameter_struct *local = FunctionParams;

	sint32					*p1;
	sint32					p2;

	p1 = local->name1;
	p2 = local->name2;

	local->return_value = (StatusType) ((*p1 * 4) + p2);
	return(E_OK);
}

/*
 *  信頼関数tfnt3
 *    非信頼OSアプリケーションからのShutdownAllCoresを
 *    受け付ける
 */
TRUSTEDFUNCTION(TRUSTED_tfnt3, FunctionIndex, FunctionParams)
{
	struct parameter_struct *local = FunctionParams;

	StatusType				ercd;

	ercd = local->ercd;
	ShutdownAllCores(ercd);

	return(E_OK);
}

/*
 *  信頼関数tfnt4
 *    非信頼OSアプリケーションから信頼OSアプリ
 *    ケーションに所属するタスクを起動する
 */
TRUSTEDFUNCTION(TRUSTED_tfnt4, FunctionIndex, FunctionParams)
{
	struct parameter_struct *local = FunctionParams;

	TaskType				task_no;

	task_no = local->task_no;

	error_log(ActivateTask(task_no));

	return(E_OK);
}

/*
 *  ノンプリエンプティブタスク
 *
 *  実行中はプリエンプトしないことの確認用
 */
TASK(NonPriTask)
{
	syslog(LOG_INFO, "NonPriTask ACTIVATE");
	syslog(LOG_INFO, "Call ActivateTask(HighPriorityTask)");
	error_log(ActivateTask(HighPriorityTask));
	syslog(LOG_INFO, "NonPriTask TERMINATE");

	error_log(TerminateTask());
}   /* TASK( NonPriTask ) */

/*
 *  並列実行タスク6
 */
TASK(Task6)
{
	uint8 command;          /* コマンド受信バッファ */

	syslog(LOG_INFO, "Task6 ACTIVATE");

	command = '\0';
	do {
		WaitEvent(T6Evt);
		ClearEvent(T6Evt);
		RecvPolSerialChar(&command);
		if (command == '\n') {
			command = '\0';
		}
	} while (command == '\0');

	syslog(LOG_INFO, "Inputed char = '%c'", command);

	syslog(LOG_INFO, "Task6 TERMINATE");

	error_log(ChainTask(Task6));
}   /* TASK( Task6 ) */

/*
 *  並列実行タスク7
 */
TASK(Task7)
{
	syslog(LOG_INFO, "Task7");
	TerminateTask();
}   /* TASK( Task7 ) */


TASK(Task13)
{
	syslog(LOG_INFO, "Task13 ACTIVATE!!!");
	syslog(LOG_INFO, "Task13 -> ChainTask(Task14)!");
	ChainTask(Task14);
}

TASK(Task14)
{
	syslog(LOG_INFO, "Task14 ACTIVATE!!!");
	TerminateTask();
}

TASK(Task17)
{
	TryToGetSpinlockType	success = TRYTOGETSPINLOCK_NOSUCCESS;
	volatile sint32			j;

	syslog(LOG_INFO, "Task17 ACTIVATE");

	/* スピンロック取得せず，変数操作 */
	for (j = 0; j < 10000; j++) {
		lock_var1++;
	}

	/* スピンロック取得 */
	error_log(TryToGetSpinlock(SmpSpin, &success));
	while (success != TRYTOGETSPINLOCK_SUCCESS) {
		error_log(TryToGetSpinlock(SmpSpin, &success));
		syslog(LOG_INFO, "TryToGetSpinlock(SmpSpin, &success) success = %d wait...", success);
		/* 一定時間のウェイト */
		for (j = 0; j < 10000; j++) {
		}
	}

	/* スピンロック取得して，変数操作 */
	for (j = 0; j < 10000; j++) {
		lock_var2++;
	}
	/* スピンロック取得しないままlock_var1変数をアクセス */
	syslog(LOG_INFO, "(Task17)(1)global variable lock_var1 = %d", lock_var1);
	syslog(LOG_INFO, "(Task17)(2)global variable lock_var2 = %d", lock_var2);
	/* スピンロック解放 */
	error_log(ReleaseSpinlock(SmpSpin));

	syslog(LOG_INFO, "Task17 TERMINATE");
	TerminateTask();
}

/*
 *  IOC確認用タスク3
 */
TASK(IocTask3)
{
	syslog(LOG_INFO, "IocTask3 ACTIVATE");

	IocProk1();

	syslog(LOG_INFO, "IocTask3 TERMINATE");
	error_log(SetEvent(MainTask, IocEvt));
	TerminateTask();
}   /* TASK( IocTask3 ) */

/*
 *  IOC確認用タスク4
 */
TASK(IocTask4)
{
	syslog(LOG_INFO, "IocTask4 ACTIVATE");

	IocProk1();

	syslog(LOG_INFO, "IocTask4 TERMINATE");
	error_log(CallTrustedFunction(setiocevt2, NULL));
	TerminateTask();
}   /* TASK( IocTask4 ) */

/*
 *  コマンド受信処理(ウェイト無し)
 */
uint8
GetCommandNoWait1(void)
{
	uint8 command;          /* コマンド受信バッファ */

	/*
	 *  コマンドを受信するまでループ
	 */
	command = '\0';
	do {
		RecvPolSerialChar(&command);    /* 受信バッファポーリング */
		if (command == '\n') {
			command = '\0';
		}
	} while (command == '\0');

	return(command);
}   /* GetCommandNoWait1 */

/*
 *  IOC確認用タスク内部処理(sample2)
 *
 */
static void
IocProk1(void)
{
	uint8			api;        /* コマンド受信バッファ(API) */
	uint8			data[3];    /* 通信バッファ */

	volatile sint32 i;
	Std_ReturnType	ercd;

	syslog(LOG_INFO, "Select IOC API:");
	api = GetCommandNoWait1();
#ifndef OMIT_ECHO
	syslog(LOG_INFO, "%c", api);
#endif /* OMIT_ECHO */

	switch (api) {
	case '1':
		syslog(LOG_INFO, "Input send data:");
		data[0] = GetCommandNoWait1();
#ifndef OMIT_ECHO
		syslog(LOG_INFO, "%c", data[0]);
#endif /* OMIT_ECHO */

		syslog(LOG_INFO, "Call IocSend_IOC_QUE_0");
		error_log_ioc(IocSend_IOC_QUE_0(data[0]));
		break;
	case '2':
		syslog(LOG_INFO, "Input send data:");
		data[0] = GetCommandNoWait1();
#ifndef OMIT_ECHO
		syslog(LOG_INFO, "%c", data[0]);
#endif /* OMIT_ECHO */

		syslog(LOG_INFO, "Call IocSend_IOC_QUE_1");
		error_log_ioc(IocSend_IOC_QUE_1(data[0]));
		break;
	case '3':
		for (i = 0; i < 3U; i++) {
			syslog(LOG_INFO, "Input send data%d:", i);
			data[i] = GetCommandNoWait1();
#ifndef OMIT_ECHO
			syslog(LOG_INFO, "%c", data[i]);
#endif /* OMIT_ECHO */
		}
		syslog(LOG_INFO, "Call IocWriteGroup_IOC_DEQUE");
		error_log_ioc(IocWriteGroup_IOC_DEQUE(data[0], data[1], data[2]));
		break;
	case '4':
		syslog(LOG_INFO, "Call IocReceive_IOC_QUE");
		ercd = IocReceive_IOC_QUE(&data[0]);
#if !defined(CFG_USE_ERRORHOOK)
		if (ercd != E_OK) {
			syslog(LOG_INFO, "Error:%d", atk2_ioc_strerror(ercd));
		}
#endif /* !defined( CFG_USE_ERRORHOOK ) */
		if ((ercd == IOC_E_OK) || (ercd == IOC_E_LOST_DATA)) {
			syslog(LOG_INFO, "recv data:%c", data[0]);
		}
		break;
	case '5':
		syslog(LOG_INFO, "Call IocReadGroup_IOC_DEQUE");
		error_log_ioc(IocReadGroup_IOC_DEQUE(&data[0], &data[1], &data[2]));
		for (i = 0; i < 3U; i++) {
			syslog(LOG_INFO, "recv data%d:%c", i, data[i]);
		}
		break;
	case '6':
		syslog(LOG_INFO, "Call IocEmptyQueue_IOC_QUE");
		error_log_ioc(IocEmptyQueue_IOC_QUE());
		break;
	default:
		syslog(LOG_INFO, "undefined command");
		break;
	}
}   /* IocProk1 */

TRUSTEDFUNCTION(TRUSTED_setiocevt2, FunctionIndex, FunctionParams)
{
	error_log(SetEvent(MainTask, IocEvt));

	return(E_OK);
}
