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
 *  $Id: sample1.c 485 2015-12-17 08:21:50Z witz-itoyo $
 */

/*
 *		信頼OSアプリケーション所属サンプルプログラムの本体
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
 *          CntRes を解放する
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
 *        'q' : ShutdownOS( E_OK ) OS実行制御機能の'q'と同等機能
 *        'Q' : ShutdownOS( E_OS_STATE ) OS実行制御機能の'Q'と同等機能
 *
 *  ・OS実行制御機能
 *    'p' : GetActiveApplicationMode にてアプリケーションモードを
 *          取得する
 *    'q' : ShutdownOS をコード E_OK で実行し，サンプルプログラムを
 *          終了する
 *    'Q' : ShutdownOS をコード E_OS_STATE で実行し，サンプルプログラム
 *          を終了する
 *
 *  ・IOC送受信機能
 *    'I' : IOC送受信モード
 *
 *        IOC APIを使用するタスク(IocTaskX)を選択
 *        '1' : IocTask1(NT_osap1に所属)
 *        '2' : IocTask2(NT_osap2に所属)
 *        '3' : IocTask3(KT_osap1に所属)
 *        '4' : IocTask4(KT_osap2に所属)
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
 *  ＜オブジェクト一覧＞
 *  ・OS
 *    スタートアップフック：使用
 *    シャットダウンフック：使用
 *    エラーフック：使用
 *    プレタスクフック：未使用
 *    ポストタスクフック：未使用
 *  ・タスク
 *    メインタスク
 *      タスクID：MainTask
 *      優先度：14
 *      多重起動数：1
 *      スケジュール：ノンプリエンプティブ
 *      自動起動：AppMode1, AppMode2, AppMode3
 *      概要：ユーザインタフェース（シリアルIOよりコマンドを受信し，
 *            それに対応した動作を行なう）
 *            周期アラーム MainCycArm により，10msごとに待ち解除し
 *            コマンドの受信有無をポーリングする
 *            イベント（ID：MainEvt）を関連付けている
 *    最高優先度タスク
 *      タスクID：HighPriorityTask
 *      優先度：15
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：起動ログを出力して終了する．ノンプリエンプティブタスク
 *            から起動され，プリエンプトしているかどうかの確認用
 *    ノンプリエンプティブタスク
 *      タスクID：NonPriTask
 *      優先度：1
 *      多重起動数：8
 *      スケジュール：ノンプリエンプティブ
 *      自動起動：なし
 *      信頼OSアプリケーションに所属
 *      概要：起動ログを出力し，最高優先度タスク HighPriorityTask を
 *            起動後，終了ログを出力してタスクを終了する
 *    タスク1
 *      タスクID：Task1
 *      優先度：4
 *      スケジュール：フルプリエンプティブ
 *      自動起動：AppMode2
 *      多重起動数：8
 *      概要：並列処理タスク（メインタスクからの指令により動作）
 *            起動されすると無限ループに入り，コマンド処理を実行する
 *            リソース TskLevelRes を関連付けている
 *            リソース CntRes を関連付けている
 *    タスク2
 *      タスクID：Task2
 *      優先度：7
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：並列処理タスク（メインタスクからの指令により動作）
 *            起動されすると無限ループに入り，コマンド処理を実行する
 *            リソース TskLevelRes を関連付けている
 *            リソース CntRes を関連付けている
 *            イベント T2Evt を関連付けている
 *    タスク3
 *      タスクID：Task3
 *      優先度：12
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：AppMode3
 *      概要：並列処理タスク（メインタスクからの指令により動作）
 *            起動されすると無限ループに入り，コマンド処理を実行する
 *            イベント待ちすることが可能である
 *            リソース CntRes を関連付けている
 *            イベント T3Evt を関連付けている
 *    タスク4
 *      タスクID：Task4
 *      優先度：6
 *      多重起動数：5
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：並列処理タスク（メインタスクからの指令により動作）
 *            起動されすると無限ループに入り，コマンド処理を実行する
 *            リソース TskLevelRes を関連付けている
 *            リソース CntRes を関連付けている
 *            内部リソース GroupRes を関連付けている
 *    タスク5
 *      タスクID：Task5
 *      優先度：9
 *      多重起動数：5
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：並列処理タスク（メインタスクからの指令により動作）
 *            起動されすると無限ループに入り，コマンド処理を実行する
 *            リソース TskLevelRes を関連付けている
 *            リソース CntRes を関連付けている
 *            内部リソース GroupRes を関連付けている
 *    タスク6
 *      タスクID：Task6
 *      優先度：9
 *      多重起動数：-
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      信頼OSアプリケーションに所属
 *      概要：並列処理タスク（メインタスクからの指令により動作）
 *            SC3ではシステムタスクとして起動
 *            起動されすると即時関数を終了する
 *            この場合でも正しくTerminateTaskが実行されるかを確認
 *            何度起動してもOKとなる
 *            Task7とスタックを共有
 *    タスク7
 *      タスクID：Task7
 *      優先度：8
 *      多重起動数：-
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      信頼OSアプリケーションに所属
 *      概要：並列処理タスク（メインタスクからの指令により動作）
 *            SC3ではユーザタスクとして起動
 *            起動されすると即時関数を終了する
 *            この場合でも正しくTerminateTaskが実行されるかを確認
 *            何度起動してもOKとなる
 *            Task6とスタックを共有
 *    タスク8
 *      タスクID：Task8
 *      優先度：8
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：起動メッセージ出力し　，アクセス権限のないTask1に対して
 *            GetTaskState()を発行して，エラーフックが上がる
 *    タスク9
 *      タスクID：Task9
 *      優先度：8
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：起動メッセージ出力し，アクセス権限のあるTask2に対して
 *            GetTaskState()を発行して，Task2のステータスを出力する
 *    タスク10
 *      タスクID：Task10
 *      優先度：14
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：起動メッセージ出力し，イベント(T10Evt)を待ち
 *    タスク11
 *      タスクID：Task11
 *      優先度：14
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：起動メッセージ出力し，イベント(T11Evt)を待ち
 *    タスク12
 *      タスクID：Task12
 *      優先度：14
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：起動メッセージ出力し，イベント(T12Evt)を待ち
 *    タスク13
 *      タスクID：Task13
 *      優先度：9
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：信頼関数動作確認用(メインタスクからの指令により動作)
 *            非信頼OSアプリケーションに所属
 *            起動されする信頼関数実行後TerminateTaskで終了する
 *            Task12とスタックを共有
 *    タスク14
 *      タスクID：Task14
 *      優先度：8
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：信頼関数動作確認用(メインタスクからの指令により動作)
 *            非信頼OSアプリケーションに所属
 *            起動されする信頼関数実行後関数を終了する
 *            この場合でも正しくTerminateTaskが実行されるかを確認
 *            Task11とスタックを共有
 *    IocTask1
 *      タスクID：IocTask1
 *      優先度：9
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：IOC通信処理(IOC APIの選択，送信データ設定，実行)を行う
 *            非信頼OSアプリケーションに所属
 *    IocTask2
 *      タスクID：IocTask2
 *      優先度：9
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：IOC通信処理(IOC APIの選択，送信データ設定，実行)を行う
 *            非信頼OSアプリケーションに所属
 *    IocTask3
 *      タスクID：IocTask3
 *      優先度：9
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：IOC通信処理(IOC APIの選択，送信データ設定，実行)を行う
 *            信頼OSアプリケーションに所属
 *    IocTask4
 *      タスクID：IocTask4
 *      優先度：9
 *      多重起動数：1
 *      スケジュール：フルプリエンプティブ
 *      自動起動：なし
 *      概要：IOC通信処理(IOC APIの選択，送信データ設定，実行)を行う
 *            非信頼OSアプリケーションに所属
 *
 *  ・割込みサービスルーチン
 *    シリアルIO受信割込み
 *      ISRID：RxHwSerialInt
 *      優先度：2
 *      カテゴリ：2
 *      概要：コマンドを受信する
 *  ・リソース
 *    タスクレベルリソース
 *      リソースID：TskLevelRes
 *      プロパティ：標準
 *    タスク用カウンタリソース
 *      リソースID：CntRes
 *      プロパティ：標準
 *    タスクグループリソース
 *      リソースID：GroupRes
 *      プロパティ：内部
 *  ・イベント
 *    メインタスクイベント
 *      イベントID：MainEvt
 *    タスク2イベント
 *      イベントID：T2Evt
 *    タスク3イベント
 *      イベントID：T3Evt
 *  ・カウンタ
 *    システムタイマカウンタ
 *      カウンタID：MAIN_HW_COUNTER
 *      カウント値：0〜999
 *      加算値：1
 *    サンプルカウンタ
 *      カウンタID：SampleCnt
 *      カウント値：0〜99
 *      加算値：10
 *  ・アラーム
 *    メイン周期アラーム
 *      アラームID：MainCycArm
 *      ベースカウンタID：MAIN_HW_COUNTER
 *      アクション：イベント設定 MainEvt
 *      自動起動：なし
 *    タスク起動アラーム
 *      アラームID：ActTskArm
 *      ベースカウンタID：MAIN_HW_COUNTER
 *      アクション：タスク起動 Task1
 *      自動起動：なし
 *    イベント設定アラーム
 *      アラームID：SetEvtArm
 *      ベースカウンタID：MAIN_HW_COUNTER
 *      アクション：イベント設定 T3Evt
 *      自動起動：なし
 *    IncrementCounter確認用アラーム
 *      アラームID：SampleArm
 *      ベースカウンタID：SampleCnt
 *      アクション：コールバック関数実行
 *      自動起動：なし
 *    他OSAP所属タスク起動アラーム
 *      アラームID：ActOtherOSAPTskArm
 *      ベースカウンタID：SchtblSampleCnt
 *      アクション：タスク起動 Task4
 *      自動起動：AppMode1, AppMode2, AppMode3
 *      設定：カウンタ5，周期0(単発)
 *  ・アプリケーションモード
 *    自動起動なしモード(MainTaskのみ自動起動)
 *      アプリケーションモードID：AppMode1
 *    Task1自動起動モード
 *      アプリケーションモードID：AppMode2
 *    Task3自動起動モード
 *      アプリケーションモードID：AppMode3
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
 *  本ファイルは信頼OSアプリケーションに
 *  所属しているオブジェクト,sample2.cでは
 *  非信頼OSアプリケーションに所属している
 *  オブジェクトを記述している
 */

#define TOPPERS_SVC_FUNCCALL

#include "sample.h"
#include "sample1.h"
#include "Ioc.h"
#include "target_hw_counter.h"

#define GetAppModeInfo()	(0)

/*
 *  プロテクション発生時の処理レベル
 */
#ifdef CFG_USE_PROTECTIONHOOK
static const char8	*faulty_context_tbl[5] = {
	"FC_INVALID",
	"FC_TASK",
	"FC_C2ISR",
	"FC_SYSTEM_HOOK",
	"FC_TRUSTED_FUNC"
};
#endif /* CFG_USE_PROTECTIONHOOK */

/*
 *  ファイル名，行番号の参照用の変数
 */
extern const char8	*fatal_file_name;   /* ファイル名 */
extern sint32		fatal_line_num;     /* 行番号 */

/*
 *  内部関数プロトタイプ宣言
 */
sint32 main(void);

DEFINE_VAR_SSTACK(StackType, stack_00[COUNT_STK_T(0x200)]);

extern uint8 GetCommand(void);
extern void PutActTsk(uint8 task_no);
extern void PutActNonPriTsk(void);
extern void PutTermTsk(uint8 task_no);
extern void PutChainTsk(uint8 from_task_no, uint8 to_task_no);
extern void PutSchedule(void);
extern void PutTaskID(void);
extern void PutTaskState(uint8 task_no);
extern void PutDisAllInt(void);
extern void PutSusAllInt(void);
extern void PutSusOSInt(void);
extern void PutHwCnt3(void);
extern void PutGetCntRes(void);
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
extern void PutActIocTsk(void);
extern void schedule_table_sample_routine(void);

TASK(IocTask3);
TASK(IocTask4);
static void IocProk2(void);
static uint8 GetCommandNoWait1(void);

/*
 *  内部データバッファ
 */
extern volatile uint8 command_tbl[14];                         /* コマンド引渡しテーブル */

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
 *  信頼関数tfnt1
 *    非信頼OSアプリケーションから渡された
 *    ポインタ引数の読書き操作して，計算結果を
 *    非信頼OSアプリケーションに返す
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
 *    非信頼OSアプリケーションからのShutdownOSを
 *    受け付ける
 */
TRUSTEDFUNCTION(TRUSTED_tfnt3, FunctionIndex, FunctionParams)
{
	struct parameter_struct *local = FunctionParams;

	StatusType				ercd;

	ercd = local->ercd;
	ShutdownOS(ercd);

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
 *  メインタスク
 *
 *  ユーザコマンドの受信と，コマンドごとの処理実行
 */
TASK(MainTask)
{
	uint8					command;
	uint8					task_no;
	uint32					i;
	struct parameter_struct	local;

	TickType				val = 0U;
	TickType				eval = 0U;

	/*
	 *  タスク番号・コマンドバッファ初期化
	 */
	task_no = (uint8) (0);
	for (i = 0U; i < (sizeof(command_tbl) / sizeof(command_tbl[0])); i++) {
		command_tbl[i] = 0U;
	}

	/*
	 *  MainCycArmを周期アラームとして設定
	 */
	SetRelAlarm(MainCycArm, TICK_FOR_10MS, TICK_FOR_10MS);

	/*
	 *  コマンド実行ループ
	 */
	while (1) {
		WaitEvent(MainEvt);     /* 10msの作業時間待ち */
		ClearEvent(MainEvt);

		/*
		 *  入力コマンド取得
		 */
		syslog(LOG_INFO, "Input Command:");
		command = GetCommand();

		/*
		 *  入力コマンドチェック
		 */
		if ((command <= (uint8) (0x1fU)) || (command >= (uint8) (0x80U))) {
			syslog(LOG_INFO, "Not ASCII character");
		}
		else {
#ifndef OMIT_ECHO
			syslog(LOG_INFO, "%c", command);
#endif /* OMIT_ECHO */

			/*
			 *  コマンド判別
			 */
			switch (command) {
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
				/*
				 *  処理対象タスクの変更
				 */
				task_no = (uint8) (command - '1');
				break;
			case 'A':
			case '!':
			case '"':
			case '#':
			case '$':
			case '%':
			case 'z':
			case 'k':
			case 'K':
			case 'l':
			case 'i':
			case 'w':
			case 'W':
				/*
				 *  タスクへのコマンド通知
				 */
				command_tbl[task_no] = command;
				break;
			/*
			 *  以降はメインタスクでコマンド処理
			 */
			case 'a':
				PutActTsk(task_no);
				break;
			case 's':
				PutSchedule();
				break;
			case 'S':
				PutActNonPriTsk();
				break;
			case 'Z':
				PutTaskState(task_no);
				break;
			case 'x':
				syslog(LOG_INFO, "call RAISE_CPU_EXCEPTION");
				RAISE_CPU_EXCEPTION;
				break;
			case 'd':
				PutDisAllInt();
				break;
			case 'D':
				PutSusAllInt();
				break;
			case 'f':
				PutSusOSInt();
				break;
			case 'T':
				PutHwCnt3();
				break;
			case 'e':
				PutSetEvt(task_no);
				break;
			case 'E':
				PutGetEvt(task_no);
				break;
			case 'b':
				PutArmBase();
				break;
			case 'B':
				PutArmTick();
				PutArmTick();
				break;
			case 'v':
				/* SetRelAlarm(ActTskArm, 500, 0)を実行 */
				PutSetRel(0U, 0U, 0U);
				break;
			case 'V':
				/* SetRelAlarm(SetEvtArm, 500, 0)を実行 */
				PutSetRel(1U, 0U, 0U);
				break;
			case 'h':
				/* CancelAlarm(CallBackArm)を実行 */
				PutCanArm();
				break;
			case 'c':
				syslog(LOG_INFO, "Call IncrementCounter(SampleCnt)");
				IncrementCounter(SampleCnt);
				break;
			case 'C':
				syslog(LOG_INFO, "Call IncrementCounter(SampleCnt2)");
				IncrementCounter(SampleCnt2);
				break;
			case 'j':
				syslog(LOG_INFO, "GetCounterValue(MAIN_HW_COUNTER, val)");
				GetCounterValue(MAIN_HW_COUNTER, &val);
				syslog(LOG_INFO, " val = %d", val);
				break;
			case 'J':
				syslog(LOG_INFO, "Pre val = %d", val);
				syslog(LOG_INFO, "GetElapsedValue(MAIN_HW_COUNTER, val ,eval)");
				GetElapsedValue(MAIN_HW_COUNTER, &val, &eval);
				syslog(LOG_INFO, " val = %d", val);
				syslog(LOG_INFO, " eval = %d", eval);
				break;
			case 'r':
				syslog(LOG_INFO, "GetISRID() Call from Task Context");
				syslog(LOG_INFO, "GetISRID() = %d", GetISRID());
				break;
			case 'p':
				PutAppMode();
				break;
			case 't':
				schedule_table_sample_routine();
				break;
			case 'q':
				local.ercd = E_OK;
				CallTrustedFunction(tfnt3, &local); /* ShutdownOS( E_OK ) */
				break;
			case 'Q':
				local.ercd = E_OS_STATE;
				CallTrustedFunction(tfnt3, &local); /* ShutdownOS( E_OS_STATE ) */
				break;
			case 'y':
				PutActTsk(10U);
				break;
			case 'Y':
				PutActTsk(11U);
				break;
			case 'I':
				PutActIocTsk();
				break;
			default:
				/* 上記のコマンド以外の場合，処理を行わない */
				break;
			}
		}
	}

	/*
	 *  ここにはこない
	 */
	syslog(LOG_INFO, "MainTask TERMINATE");
	error_log(TerminateTask());
}   /* TASK( MainTask ) */

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
	TaskProk(5U);
}   /* TASK( Task6 ) */

/*
 *  並列実行タスク7
 */
TASK(Task7)
{
	TaskProk(6U);
}   /* TASK( Task7 ) */

/*
 *  IOC確認用タスク3
 */
TASK(IocTask3)
{
	syslog(LOG_INFO, "IocTask3 ACTIVATE");

	IocProk2();

	syslog(LOG_INFO, "IocTask3 TERMINATE");
	TerminateTask();
}   /* TASK( IocTask3 ) */

/*
 *  IOC確認用タスク4
 */
TASK(IocTask4)
{
	syslog(LOG_INFO, "IocTask4 ACTIVATE");

	IocProk2();

	syslog(LOG_INFO, "IocTask4 TERMINATE");
	TerminateTask();
}   /* TASK( IocTask4 ) */

/*
 *  IOC確認用タスク内部処理
 *
 */
static void
IocProk2(void)
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
}   /* IocProk2 */

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
 *  ユーザメイン関数
 *
 *  アプリケーションモードの判断と，カーネル起動
 */
sint32
main(void)
{
	AppModeType	crt_app_mode;

	/*
	 *  アプリケーションモードの判断
	 */
	switch (GetAppModeInfo()) {
	case 0:
		crt_app_mode = AppMode1;
		break;
	case 1:
		crt_app_mode = AppMode2;
		break;
	default:
		crt_app_mode = AppMode3;
		break;
	}

	/*
	 *  カーネル起動
	 */
	StartOS(crt_app_mode);

	while (1) {
	}
}   /* main */

/*
 *  エラーフックルーチン
 */
#ifdef CFG_USE_ERRORHOOK
void
ErrorHook(StatusType Error)
{

	/*
	 *  エラー要因ごとのパラメータログ出力
	 */
	switch (OSErrorGetServiceId()) {
	case OSServiceId_ActivateTask:
		syslog(LOG_INFO, "Error:%s=ActivateTask(%d)", atk2_strerror(Error), OSError_ActivateTask_TaskID());
		break;
	case OSServiceId_TerminateTask:
		syslog(LOG_INFO, "Error:%s=TerminateTask()", atk2_strerror(Error));
		break;
	case OSServiceId_ChainTask:
		syslog(LOG_INFO, "Error:%s=ChainTask(%d)", atk2_strerror(Error), OSError_ChainTask_TaskID());
		break;
	case OSServiceId_Schedule:
		syslog(LOG_INFO, "Error:%s=Schedule()", atk2_strerror(Error));
		break;
	case OSServiceId_GetTaskID:
		syslog(LOG_INFO, "Error:%s=GetTaskID(0x%p)", atk2_strerror(Error), OSError_GetTaskID_TaskID());
		break;
	case OSServiceId_GetTaskState:
		syslog(LOG_INFO, "Error:%s=GetTaskState(%d, 0x%p)", atk2_strerror(Error),
			   OSError_GetTaskState_TaskID(), OSError_GetTaskState_State());
		break;
	case OSServiceId_EnableAllInterrupts:
		syslog(LOG_INFO, "Error:%s=EnableAllInterrupts()", atk2_strerror(Error));
		break;
	case OSServiceId_DisableAllInterrupts:
		syslog(LOG_INFO, "Error:%s=DisableAllInterrupts()", atk2_strerror(Error));
		break;
	case OSServiceId_ResumeAllInterrupts:
		syslog(LOG_INFO, "Error:%s=ResumeAllInterrupts()", atk2_strerror(Error));
		break;
	case OSServiceId_SuspendAllInterrupts:
		syslog(LOG_INFO, "Error:%s=SuspendAllInterrupts()", atk2_strerror(Error));
		break;
	case OSServiceId_ResumeOSInterrupts:
		syslog(LOG_INFO, "Error:%s=ResumeOSInterrupts()", atk2_strerror(Error));
		break;
	case OSServiceId_SuspendOSInterrupts:
		syslog(LOG_INFO, "Error:%s=SuspendOSInterrupts()", atk2_strerror(Error));
		break;
	case OSServiceId_GetISRID:
		syslog(LOG_INFO, "Error:%s=GetISRID()", atk2_strerror(Error));
		break;
	case OSServiceId_GetResource:
		syslog(LOG_INFO, "Error:%s=GetResource(%d)", atk2_strerror(Error), OSError_GetResource_ResID());
		break;
	case OSServiceId_ReleaseResource:
		syslog(LOG_INFO, "Error:%s=ReleaseResource(%d)", atk2_strerror(Error), OSError_ReleaseResource_ResID());
		break;
	case OSServiceId_SetEvent:
		syslog(LOG_INFO, "Error:%s=SetEvent(%d, 0x%x)", atk2_strerror(Error),
			   OSError_SetEvent_TaskID(), OSError_SetEvent_Mask());
		break;
	case OSServiceId_ClearEvent:
		syslog(LOG_INFO, "Error:%s=ClearEvent(0x%x)", atk2_strerror(Error), OSError_ClearEvent_Mask());
		break;
	case OSServiceId_GetEvent:
		syslog(LOG_INFO, "Error:%s=GetEvent(%d, 0x%p)", atk2_strerror(Error),
			   OSError_GetEvent_TaskID(), OSError_GetEvent_Event());
		break;
	case OSServiceId_WaitEvent:
		syslog(LOG_INFO, "Error:%s=WaitEvent(0x%x)", atk2_strerror(Error), OSError_WaitEvent_Mask());
		break;
	case OSServiceId_GetAlarmBase:
		syslog(LOG_INFO, "Error:%s=GetAlarmBase(0x%p)", atk2_strerror(Error), OSError_GetAlarmBase_AlarmID());
		break;
	case OSServiceId_GetAlarm:
		syslog(LOG_INFO, "Error:%s=GetAlarm(%d, 0x%p)", atk2_strerror(Error),
			   OSError_GetAlarm_AlarmID(), OSError_GetAlarm_Tick());
		break;
	case OSServiceId_SetRelAlarm:
		syslog(LOG_INFO, "Error:%s=SetRelAlarm(%d, %d, %d)", atk2_strerror(Error),
			   OSError_SetRelAlarm_AlarmID(), OSError_SetRelAlarm_increment(), OSError_SetRelAlarm_cycle());
		break;
	case OSServiceId_SetAbsAlarm:
		syslog(LOG_INFO, "Error:%s=SetAbsAlarm(%d, %d, %d)", atk2_strerror(Error),
			   OSError_SetAbsAlarm_AlarmID(), OSError_SetAbsAlarm_start(), OSError_SetAbsAlarm_cycle());
		break;
	case OSServiceId_CancelAlarm:
		syslog(LOG_INFO, "Error:%s=CancelAlarm(%d)", atk2_strerror(Error), OSError_CancelAlarm_AlarmID());
		break;
	case OSServiceId_StartScheduleTableRel:
		syslog(LOG_INFO, "Error:%s=StartScheduleTableRel(%d, %d)", atk2_strerror(Error),
			   OSError_StartScheduleTableRel_ScheduleTableID(), OSError_StartScheduleTableRel_Offset());
		break;
	case OSServiceId_StartScheduleTableAbs:
		syslog(LOG_INFO, "Error:%s=StartScheduleTableAbs(%d, %d)", atk2_strerror(Error),
			   OSError_StartScheduleTableAbs_ScheduleTableID(), OSError_StartScheduleTableAbs_Start());
		break;
	case OSServiceId_StopScheduleTable:
		syslog(LOG_INFO, "Error:%s=StopScheduleTable(%d)", atk2_strerror(Error), OSError_StopScheduleTable_ScheduleTableID());
		break;
	case OSServiceId_NextScheduleTable:
		syslog(LOG_INFO, "Error:%s=NextScheduleTable(%d, %d)", atk2_strerror(Error),
			   OSError_NextScheduleTable_ScheduleTableID_From(), OSError_NextScheduleTable_ScheduleTableID_To());
		break;
	case OSServiceId_GetScheduleTableStatus:
		syslog(LOG_INFO, "Error:%s=GetScheduleTableStatus(%d, 0x%p)", atk2_strerror(Error),
			   OSError_GetScheduleTableStatus_ScheduleTableID(), OSError_GetScheduleTableStatus_ScheduleStatus());
		break;
	case OSServiceId_GetActiveApplicationMode:
		syslog(LOG_INFO, "Error:%s=GetActiveApplicationMode()", atk2_strerror(Error));
		break;
	case OSServiceId_StartOS:
		syslog(LOG_INFO, "Error:%s=StartOS()", atk2_strerror(Error));
		break;
	case OSServiceId_ShutdownOS:
		syslog(LOG_INFO, "Error:%s=ShutdownOS()", atk2_strerror(Error));
		break;
	case OSServiceId_IncrementCounter:
		syslog(LOG_INFO, "Error:%s=IncrementCounter(%d)", atk2_strerror(Error), OSError_IncrementCounter_CounterID());
		break;
	case OSServiceId_TaskMissingEnd:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_strerror(Error));
		break;
	case IOCServiceId_IOC_Send:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	case IOCServiceId_IOC_Write:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	case IOCServiceId_IOC_SendGroup:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	case IOCServiceId_IOC_WriteGroup:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	case IOCServiceId_IOC_Receive:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	case IOCServiceId_IOC_Read:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	case IOCServiceId_IOC_ReceiveGroup:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	case IOCServiceId_IOC_ReadGroup:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	case IOCServiceId_IOC_EmptyQueue:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	default:
		syslog(LOG_INFO, "Error:%s=UnKnownFunc()", atk2_strerror(Error));
		break;
	}

}
#endif /* CFG_USE_ERRORHOOK */

/*
 *  プレタスクフックルーチン
 *
 *  Task13/Task14実行前のプレタスクフックの場合，メッセージを出力
 */
#ifdef CFG_USE_PRETASKHOOK
void
PreTaskHook(void)
{
	TaskType tskid;

	GetTaskID(&tskid);
	if (tskid == Task13) {
		syslog(LOG_INFO, "\nPreTaskHook before Task13!\n");
	}
	if (tskid == Task14) {
		syslog(LOG_INFO, "\nPreTaskHook before Task14!\n");
	}
}
#endif /* CFG_USE_PRETASKHOOK */

/*
 *  ポストタスクフックルーチン
 *
 *  Task13/Task14実行後のポストタスクフックの場合，メッセージを出力
 */
#ifdef CFG_USE_POSTTASKHOOK
void
PostTaskHook(void)
{
	TaskType tskid;

	GetTaskID(&tskid);
	if (tskid == Task13) {
		syslog(LOG_INFO, "\nPostTaskHook after Task13!\n");
	}
	if (tskid == Task14) {
		syslog(LOG_INFO, "\nPostTaskHook after Task14!\n");
	}
}
#endif /* CFG_USE_POSTTASKHOOK */

/*
 *  スタートアップフックルーチン
 */
#ifdef CFG_USE_STARTUPHOOK
#ifdef TOPPERS_ENABLE_SYS_TIMER
extern void target_timer_initialize(void);
#endif /* TOPPERS_ENABLE_SYS_TIMER */

void
StartupHook(void)
{
#ifdef TOPPERS_ENABLE_SYS_TIMER
	target_timer_initialize();
#endif /* TOPPERS_ENABLE_SYS_TIMER */
	syslog_initialize();
	syslog_msk_log(LOG_UPTO(LOG_INFO));
	InitSerial();
	print_banner();
}
#endif /* CFG_USE_STARTUPHOOK */

/*
 *  シャットダウンフックルーチン
 */
#ifdef CFG_USE_SHUTDOWNHOOK
#ifdef TOPPERS_ENABLE_SYS_TIMER
extern void target_timer_terminate(void);
#endif /* TOPPERS_ENABLE_SYS_TIMER */

void
ShutdownHook(StatusType Error)
{
	/* 終了ログ出力 */
	syslog(LOG_INFO, "Sample System ShutDown");
	syslog(LOG_INFO, "ShutDownCode:%s", atk2_strerror(Error));
	syslog(LOG_INFO, "");

	if (Error == E_OS_SYS_ASSERT_FATAL) {
		syslog(LOG_INFO, "fatal_file_name:%s", fatal_file_name);
		syslog(LOG_INFO, "fatal_line_num:%d", fatal_line_num);
	}

#ifdef TOPPERS_ENABLE_SYS_TIMER
	target_timer_terminate();
#endif /* TOPPERS_ENABLE_SYS_TIMER */
	TermSerial();

}
#endif /* CFG_USE_SHUTDOWNHOOK */

/*
 *  プロテクションフックルーチン
 */
#ifdef CFG_USE_PROTECTIONHOOK
ProtectionReturnType
ProtectionHook(StatusType FatalError)
{
	StatusType			ercd;
	FaultyContextType	faulty_context;
	TaskType			taskid;

	syslog(LOG_INFO, "ProtectionHook");

	if (FatalError == E_OS_STACKFAULT) {
		syslog(LOG_INFO, "E_OS_STACKFAULT");
		ercd = PRO_SHUTDOWN;
	}
	else if (FatalError == E_OS_PROTECTION_EXCEPTION) {
		faulty_context = GetFaultyContext();
		syslog(LOG_INFO, "E_OS_PROTECTION_EXCEPTION and GetFaultyContext() = %s", faulty_context_tbl[faulty_context]);

		switch (faulty_context) {
		case FC_TASK:
			ercd = GetTaskID(&taskid);
			if (taskid == MainTask) {
				ercd = PRO_IGNORE;
			}
			else {
				ercd = PRO_TERMINATETASKISR;
			}
			break;
		case FC_C2ISR:
			ercd = PRO_TERMINATETASKISR;
			break;
		case FC_SYSTEM_HOOK:
			ercd = PRO_IGNORE;
			break;
		case FC_TRUSTED_FUNC:
			ercd = PRO_IGNORE;
			break;
		case FC_INVALID:
			ercd = PRO_SHUTDOWN;
			break;
		default:
			ercd = PRO_SHUTDOWN;
			syslog(LOG_INFO, "unknown error fo GetFaultyContext() = %d", faulty_context);
		}
	}
	else {
		ercd = PRO_SHUTDOWN;
	}

	return(ercd);
}
#endif /* CFG_USE_PROTECTIONHOOK */
