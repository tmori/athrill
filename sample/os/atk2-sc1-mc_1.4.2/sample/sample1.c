/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2004-2017 by Witz Corporation
 *  Copyright (C) 2014-2016 by AISIN COMCRUISE Co., Ltd., JAPAN
 *  Copyright (C) 2014-2016 by eSOL Co.,Ltd., JAPAN
 *  Copyright (C) 2014-2017 by SCSK Corporation, JAPAN
 *  Copyright (C) 2015-2017 by SUZUKI MOTOR CORPORATION
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
 *  $Id: sample1.c 739 2017-01-24 10:05:05Z nces-hibino $
 */

/*
 *		サンプルプログラム(1)の本体
 *
 *  ATK2-SC1-MCの基本的な動作を確認するためのサンプルプログラム
 *
 *  プログラムの概要：PCとボード間でシリアル通信を行い，ユーザ入力
 *  コマンドに応じた動作とログ出力を行なう
 *
 *  ＜コマンド一覧＞
 *  ・タスク指定
 *    '1' : 以降のコマンドを Task1 に対して行う
 *    '2' : 以降のコマンドを Task2 に対して行う
 *    '3' : 以降のコマンドを Task3 に対して行う
 *    '4' : 以降のコマンドを Task4 に対して行う
 *    '5' : 以降のコマンドを Task5 に対して行う
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
 *          再スケジューリングを行う
 *    'S' : ノンプリエンプティブタスク NonPriTask を起動する
 *          ノンプリエンプティブタスク NonPriTask にて最高優先度
 *          タスク HighPriorityTask を起動し，タスク終了する
 *    'z' : 実行中のタスクから GetTaskID を実行して，実行状態の TaskID を取得する
 *    'Z' : メインタスクから GetTaskState を実行して，対象タスクの状態を取得する
 *    'x' : div命令を実行して，未定義命令例外を発生させる
 *  ・割込み管理機能
 *    'd' : DisableAllInterrupts を実行後，ハードウェアカウンタ値を
 *          3回表示し， EnableAllInterrupts を実行する
 *    'D' : SuspendAllInterrupts を実行後，ハードウェアカウンタ値を
 *          3回表示し，さらに SuspendAllInterrupts を実行後，ハード
 *          ウェアカウンタ値を3回表示し，ResumeAllInterrupts を実行後，
 *          ハードウェアカウンタ値を3回表示し，ResumeAllInterrupts を
 *          実行する
 *    'f' : SuspendOSInterrupts を実行後，ハードウェアカウンタ値を
 *          3回表示し，さらに SuspendOSInterrupts を実行後，ハード
 *          ウェアカウンタ値を3回表示し，さらに SuspendAllInterrupts
 *          を実行後，ハードウェアカウンタ値を3回表示し，
 *          ResumeAllInterrupts を実行後，ハードウェアカウンタを3回
 *          表示し，ResumeOSInterrupts を実行後ハードウェアカウンタ
 *          値を3回表示し，ResumeOSInterrupts を実行する
 *    'T' : ハードウェアカウンタ値を3回表示する
 *  ・リソース管理機能
 *    'k' : GetResource にてリソース TskLevelRes を取得する．なお，
 *          Task3 は，このリソースより優先度が高いためエラーとなる
 *    'K' : ReleaseResource にてリソース TskLevelRes を解放する
 *    'i' : GetResource にてリソース CntRes を取得後，ハードウェア
 *          カウンタ値を3回表示し，ReleaseResource にてリソース
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
 *    'b' : GetAlarmBase にてアラーム MainCycArm0 のアラームベース
 *          情報を取得する
 *    'B' : GetAlarm にてアラーム MainCycArm0 の残りカウント値を2回
 *          連続で取得する
 *    'v' : SetRelAlarm にてアラーム ActTskArm を起動し，500ms 後に
 *          タスク Task1 を起動する
 *    'V' : SetRelAlarm にてアラーム SetEvtArm を起動し，500ms 後に
 *          イベント T3Evt を設定する
 *    'n' : SetRelAlarm にてアラーム CallBackArm を，パラメータ
 *          900ms 後に満了・単発アラーム指定で設定する
 *    'N' : SetRelAlarm にてアラーム CallBackArm を，パラメータ
 *          900ms 後に満了・ 500ms 周期アラーム指定で設定する
 *    'm' : SetAbsAlarm にてアラーム CallBackArm を，パラメータ
 *          カウンタ値900に満了・単発アラーム指定で設定する
 *    'M' : SetAbsAlarm にてアラーム CallBackArm を，パラメータ
 *          カウンタ値900に満了・ 500ms 周期アラームで設定する
 *    'h' : CancelAlarm にてアラーム CallBackArm をキャンセルする
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
 *    自動起動：AppMode1, AppMode2, AppMode3
 *    概要：ユーザインタフェース（シリアルIOよりコマンドを受信し，
 *          それに対応した動作を行なう）
 *          周期アラーム MainCycArm0 により，10msごとに待ち解除し
 *          コマンドの受信有無をポーリングする
 *          イベント（ID：MainEvt0, IocEvt）を関連付けている
 *  最高優先度タスク
 *    タスクID：HighPriorityTask
 *    優先度：15
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：起動ログを出力して終了する．ノンプリエンプティブタスク
 *          から起動され，プリエンプトしているかどうかの確認用
 *  ノンプリエンプティブタスク
 *    タスクID：NonPriTask
 *    優先度：1
 *    多重起動数：8
 *    スケジュール：ノンプリエンプティブ
 *    自動起動：なし
 *    概要：起動ログを出力し，最高優先度タスク HighPriorityTask を
 *          起動後，終了ログを出力してタスクを終了する
 *  タスク1
 *    タスクID：Task1
 *    優先度：4
 *    スケジュール：フルプリエンプティブ
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
 *    自動起動：なし
 *    概要：並列処理タスク（メインタスクからの指令により動作）
 *          起動されすると無限ループに入り，コマンド処理を実行する
 *          リソース TskLevelRes を関連付けている
 *          リソース CntRes を関連付けている
 *          内部リソース GroupRes を関連付けている
 *  タスク6
 *    タスクID：Task6
 *    優先度：13
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：起動メッセージ出力し，イベント(T6Evt)を待ち
 *  タスク7
 *    タスクID：Task7
 *    優先度：13
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：起動メッセージ出力し，イベント(T7Evt)を待ち
 *  タスク8
 *    タスクID：Task8
 *    優先度：13
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：起動メッセージ出力し，イベント(T8Evt)を待ち
 *  タスク9
 *    タスクID：Task9
 *    優先度：6
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：起動メッセージ出力し，スピンロック取得せずlock_var1変数を
 *          インクリメントして，スピンロックを取得後lock_var2変数を
 *          インクリメットしてからlock_var1とlock_var2の値を表示し，
 *          スピンロックを解放し，終了する
 *          数回実行して見れば，lock_var1は不定値となり，lock_var2は
 *          固定値となる
 *  IdleTask
 *    タスクID：IdleTask
 *    優先度：6
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：起動メッセージ出力して，文字受信出力を行う
 *  IocTask1
 *    タスクID：IocTask1
 *    優先度：9
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：IOC通信処理(IOC APIの選択，送信データ設定，実行)を行う
 *          イベント（ID：IocEvt）を関連付けている
 *  IocTask2
 *    タスクID：IocTask2
 *    優先度：9
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：IOC通信処理(IOC APIの選択，送信データ設定，実行)を行う
 *          イベント（ID：IocEvt）を関連付けている
 *  IocTask3
 *    タスクID：IocTask3
 *    優先度：9
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：IOC通信処理(IOC APIの選択，送信データ設定，実行)を行う
 *          イベント（ID：IocEvt）を関連付けている
 *  IocTask4
 *    タスクID：IocTask4
 *    優先度：9
 *    多重起動数：1
 *    スケジュール：フルプリエンプティブ
 *    自動起動：なし
 *    概要：IOC通信処理(IOC APIの選択，送信データ設定，実行)を行う
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
 *    イベントID：MainEvt0
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
 *    アラームID：MainCycArm0
 *    ベースカウンタID：MAIN_HW_COUNTER_CORE0
 *    アクション：イベント設定 MainEvt0
 *    自動起動：なし
 *  タスク起動アラーム
 *    アラームID：ActTskArm
 *    ベースカウンタID：MAIN_HW_COUNTER_CORE0
 *    アクション：タスク起動 Task1
 *    自動起動：なし
 *  イベント設定アラーム
 *    アラームID：SetEvtArm
 *    ベースカウンタID：MAIN_HW_COUNTER_CORE0
 *    アクション：イベント設定 T3Evt
 *    自動起動：なし
 *  コールバック実行アラーム
 *    アラームID：CallBackArm
 *    ベースカウンタID：MAIN_HW_COUNTER_CORE0
 *    アクション：コールバック関数実行
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
 *
 */

#include "Os.h"
#include "t_syslog.h"
#include "t_stdlib.h"
#include "sysmod/serial.h"
#include "sysmod/syslog.h"
#include "sample1.h"
#include "Ioc.h"

#include "sysmod/banner.h"
#include "target_sysmod.h"
#include "target_serial.h"
#include "sysmod/serial.h"
#include "target_hw_counter.h"


#define GetHwCnt(x, y)
#define GetAppModeInfo()	(0)

/*
 *  ファイル名，行番号の参照用の変数
 */
extern const char8	*kernel_fatal_file_name;    /* ファイル名 */
extern sint32		kernel_fatal_line_num;      /* 行番号 */

volatile sint32		lock_var1 = 0;          /* スピンロック動作確認用グローバル変数1 */
volatile sint32		lock_var2 = 0;          /* スピンロック動作確認用グローバル変数2 */

/*
 *  内部関数プロトタイプ宣言
 */
sint32 main(void);
TASK(MainTask);
TASK(HighPriorityTask);
TASK(NonPriTask);
TASK(Task1);
TASK(Task2);
TASK(Task3);
TASK(Task4);
TASK(Task5);
TASK(Task6);
TASK(Task7);
TASK(Task8);
TASK(Task9);
TASK(IocTask1);
TASK(IocTask2);
TASK(IocTask3);
TASK(IocTask4);
TASK(IdleTask);
ALARMCALLBACK(SysTimerAlmCb);
static void TaskProk(uint8 task_no);
static uint8 GetCommand(EventMaskType mask);
static uint8 GetCommandNoWait(void);
static void PutActTsk(uint8 task_no);
static void PutActNonPriTsk(void);
static void PutTermTsk(uint8 task_no);
static void PutChainTsk(uint8 from_task_no, uint8 to_task_no);
static void PutSchedule(void);
static void PutTaskID(void);
static void PutTaskState(uint8 task_no);
static void PutDisAllInt(void);
static void PutSusAllInt(void);
static void PutSusOSInt(void);
static void PutHwCnt3(void);
static void PutGetCntRes(void);
static void PutGetTskRes(void);
static void PutRelTskRes(void);
static void PutSetEvt(uint8 task_no);
static void PutClrEvt(uint8 task_no);
static void PutGetEvt(uint8 task_no);
static void PutWaitEvt(uint8 task_no);
static void PutArmBase(void);
static void PutArmTick(void);
static void PutSetRel(uint8 alarm_no, uint8 tick_no, uint8 cycle_no);
static void PutSetAbs(uint8 alarm_no, uint8 tick_no, uint8 cycle_no);
static void PutCanArm(void);
static void PutAppMode(void);
static void schedule_table_sample_routine(void);
static void PutActIocTsk(void);
static void IocProk(void);

/*
 *  内部データバッファ
 */
static volatile uint8		command_tbl[8];         /* コマンド引渡しテーブル */

/*
 *  内部定数データテーブル
 */
/* 無効イベントマスク値 */
#define invalid_mask	(EventMaskType) (0)

/* イベントマスクテーブル */
static const EventMaskType	event_mask_tbl[] = {
	invalid_mask,
	T2Evt,
	T3Evt,
	invalid_mask,
	invalid_mask
};

/* タスクIDテーブル */
static const TaskType		task_id_tbl[] = {
	Task1,
	Task2,
	Task3,
	Task4,
	Task5
};

/* アラームIDテーブル */
static const AlarmType		alarm_id_tbl[] = {
	ActTskArm,
	SetEvtArm,
	CallBackArm
};

/* ティック値テーブル */
static const TickType		tick_tbl[] = {
	(TickType) 500,
	(TickType) 900
};

/* サイクル値テーブル */
static const TickType		cycle_tbl[] = {
	(TickType) 0,
	(TickType) COUNTER_MIN_CYCLE
};

/* イベントマスク名文字列テーブル */
static const char8			*event_name_tbl[] = {
	"Invalid",
	"T2Evt",
	"T3Evt",
	"Invalid",
	"Invalid"
};

/* タスク名文字列テーブル */
static const char8			*task_name_tbl[] = {
	"Task1",
	"Task2",
	"Task3",
	"Task4",
	"Task5"
};

/* タスク状態文字列テーブル */
static const char8			*task_state_tbl[] = {
	"SUSPENDED",
	"RUNNING",
	"READY",
	"WAITING",
};

/* アラーム名文字列テーブル */
static const char8			*alarm_name_tbl[] = {
	"ActTskArm",
	"SetEvtArm",
	"CallBackArm"
};

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
 *  ユーザメイン関数
 *
 *  アプリケーションモードの判断と，カーネル起動
 */
sint32
main(void)
{
	AppModeType	crt_app_mode;
	StatusType	ercd;
	CoreIdType	i;

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

	if (GetCoreID() == OS_CORE_ID_MASTER) {
		for (i = 0; i < TNUM_HWCORE; i++) {
			if (i != OS_CORE_ID_MASTER) {
				StartCore(i, &ercd);
			}
		}
		/*
		 *  カーネル起動
		 */
		StartOS(crt_app_mode);
	}
	else {
		/*
		 *  カーネル起動
		 */
		StartOS(DONOTCARE);
	}


	while (1) {
	}
}   /* main */

/*
 *  メインタスク
 *
 *  ユーザコマンドの受信と，コマンドごとの処理実行
 */
TASK(MainTask)
{
	uint8		command;
	uint8		task_no;
	uint32		i;
	CoreIdType	coreid = GetCoreID();

	TickType	val = 0U;
	TickType	eval = 0U;

	syslog(LOG_EMERG, "activate MainTask! @ core%d", coreid);
	/*
	 *  タスク番号・コマンドバッファ初期化
	 */
	task_no = (uint8) (0);
	for (i = 0U; i < (sizeof(command_tbl) / sizeof(command_tbl[0])); i++) {
		command_tbl[i] = 0U;
	}

	/*
	 *  MainCycArm0，MainCycArm1を周期アラームとして設定
	 */
	SetRelAlarm(MainCycArm0, TICK_FOR_10MS, TICK_FOR_10MS);
	SetRelAlarm(MainCycArm1, TICK_FOR_10MS, TICK_FOR_10MS);

	/*
	 *  コマンド実行ループ
	 */
	while (1) {
		WaitEvent(MainEvt0);     /* 10msの作業時間待ち */
		ClearEvent(MainEvt0);

		/*
		 *  入力コマンド取得
		 */
		syslog(LOG_INFO, "Input Command:");
		command = GetCommand(MainEvt0);

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
			case 'n':
				/* SetRelAlarm(CallBackArm, 900, 0)を実行 */
				PutSetRel(2U, 1U, 0U);
				break;
			case 'N':
				/* SetRelAlarm(CallBackArm, 900, 500)を実行 */
				PutSetRel(2U, 1U, 1U);
				break;
			case 'm':
				/* SetAbsAlarm(CallBackArm, 900, 0)を実行 */
				PutSetAbs(2U, 1U, 0U);
				break;
			case 'M':
				/* SetAbsAlarm(CallBackArm, 900, 500)を実行 */
				PutSetAbs(2U, 1U, 1U);
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
				syslog(LOG_INFO, "GetCounterValue(MAIN_HW_COUNTER_CORE0, val)");
				GetCounterValue(MAIN_HW_COUNTER_CORE0, &val);
				syslog(LOG_INFO, " val = %d", val);
				break;
			case 'J':
				syslog(LOG_INFO, "Pre val = %d", val);
				syslog(LOG_INFO, "GetElapsedValue(MAIN_HW_COUNTER_CORE0, val ,eval)");
				GetElapsedValue(MAIN_HW_COUNTER_CORE0, &val, &eval);
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
				ShutdownAllCores(E_OK);
				break;
			case 'Q':
				ShutdownAllCores(E_OS_STATE);
				break;
			case 'I':
				PutActIocTsk();
				WaitEvent(IocEvt);     /* IOCテストが終わるまでWait(IOC用タスクでGetCommandするため) */
				ClearEvent(IocEvt);
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
 *  最高優先度タスク
 *
 *  各タスクのプリエンプト確認用
 */
TASK(HighPriorityTask)
{
	syslog(LOG_INFO, "HighPriorityTask ACTIVATE");
	error_log(TerminateTask());
}   /* TASK( HighPriorityTask ) */


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
 *  並列実行タスク1
 */
TASK(Task1)
{
	volatile sint32 j;

	lock_var1 = 0;
	lock_var2 = 0;

	/* スピンロック取得 */
	error_log(GetSpinlock(SmpSpin));

	ActivateTask(Task9);

	/* スピンロック取得して，変数操作 */
	for (j = 0; j < 10000; j++) {
		lock_var1++;
	}

	for (j = 0; j < 10000; j++) {
		lock_var2++;
	}

	/* スピンロック解放 */
	error_log(ReleaseSpinlock(SmpSpin));

	/* コア間割込みの動作確認 */
	syslog(LOG_INFO, "RaiseInterCoreInterrupt before");
	RaiseInterCoreInterrupt(Core0_ICI_1);   /* 自コアへの割込み */
	RaiseInterCoreInterrupt(Core1_ICI_1);   /* 他コアへの割込み */
	syslog(LOG_INFO, "RaiseInterCoreInterrupt finish");

	/* 並列実行タスク内部処理 */
	TaskProk(0U);
}   /* TASK( Task1 ) */


/*
 *  並列実行タスク2
 */
TASK(Task2)
{
	TaskProk(1U);
}   /* TASK( Task2 ) */


/*
 *  並列実行タスク3
 */
TASK(Task3)
{
	TaskProk(2U);
}   /* TASK( Task3 ) */


/*
 *  並列実行タスク4
 */
TASK(Task4)
{
	TaskProk(3U);
}   /* TASK( Task4 ) */


/*
 *  並列実行タスク5
 */
TASK(Task5)
{
	TaskProk(4U);
}   /* TASK( Task5 ) */


/*
 *  並列実行タスク内部処理
 *
 *  メインタスクから通知されたコマンドごとの処理実行
 */
static void
TaskProk(uint8 task_no)
{
	uint8 command;          /* コマンド退避バッファ */

	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "%s ACTIVATE", task_name_tbl[task_no]);

	/*
	 *  コマンド実行ループ
	 */
	while (1) {

		/*
		 *  コマンド取得
		 */
		while (command_tbl[task_no] == '\0') {
		}
		command = command_tbl[task_no];
		command_tbl[task_no] = 0U;

		/*
		 *  コマンド判定
		 */
		switch (command) {
		case 'A':
			PutTermTsk(task_no);
			break;
		case '!':
		case '"':
		case '#':
		case '$':
		case '%':
			PutChainTsk(task_no, (command - '!'));
			break;
		case 'z':
			PutTaskID();
			break;
		case 'k':
			PutGetTskRes();
			break;
		case 'K':
			PutRelTskRes();
			break;
		case 'i':
			PutGetCntRes();
			break;
		case 'w':
			PutClrEvt(task_no);
			break;
		case 'W':
			PutWaitEvt(task_no);
			break;
		default:
			/* 上記のコマンド以外の場合，処理は行わない */
			break;
		}
	}
}   /* TaskProk */

/*
 *  コマンド受信処理
 */
static uint8
GetCommand(EventMaskType mask)
{
	uint8 command;          /* コマンド受信バッファ */

	/*
	 *  コマンドを受信するまでループ
	 */
	command = '\0';
	do {
		WaitEvent(mask);     /* 10msウェイト */
		ClearEvent(mask);
		RecvPolSerialChar(&command);    /* 受信バッファポーリング */
		if (command == '\n') {
			command = '\0';
		}
	} while (command == '\0');


	return(command);
}   /* GetCommand */

/*
 *  コマンド受信処理(ウェイト無し)
 */
static uint8
GetCommandNoWait(void)
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
}   /* GetCommand */

/*
 *  ActivateTask 実行・ログ出力処理
 */
static void
PutActTsk(uint8 task_no)
{
	syslog(LOG_INFO, "Call ActivateTask(%s)", task_name_tbl[task_no]);

	error_log(ActivateTask(task_id_tbl[task_no]));

}   /* PutActTsk	*/

/*
 *  ActivateTask 実行(NonPriTask)・ログ出力処理
 */
static void
PutActNonPriTsk(void)
{
	syslog(LOG_INFO, "Call ActivateTask(NonPriTask)");

	error_log(ActivateTask(NonPriTask));
}   /* PutActNonPriTsk */

/*
 *  TerminateTask 実行・ログ出力処理
 */
static void
PutTermTsk(uint8 task_no)
{
	StatusType ercd;        /* エラーコード */

	syslog(LOG_INFO, "%s TERMINATE", task_name_tbl[task_no]);

	ercd = TerminateTask();
	ShutdownAllCores(ercd);
}

/*
 *  ChainTask 実行・ログ出力処理
 */
static void
PutChainTsk(uint8 from_task_no, uint8 to_task_no)
{
	StatusType ercd;            /* エラーコード */

	syslog(LOG_INFO, "Call ChainTask(%s)", task_name_tbl[to_task_no]);
	syslog(LOG_INFO, "%s TERMINATE", task_name_tbl[from_task_no]);

	ercd = ChainTask(task_id_tbl[to_task_no]);
	if (ercd == E_OS_LIMIT) {
		syslog(LOG_INFO, "Call TerminateTask()");
		syslog(LOG_INFO, "Because of ChainTask E_OS_LIMIT return");
		ercd = TerminateTask();
	}
	ShutdownAllCores(ercd);
}   /* PutChainTsk */

/*
 *  Schedule 実行・ログ出力処理
 */
static void
PutSchedule(void)
{
	syslog(LOG_INFO, "Call ActivateTask(HighPriorityTask)");

	error_log(ActivateTask(HighPriorityTask));
	syslog(LOG_INFO, "Call Schedule()");

	error_log(Schedule());
	syslog(LOG_INFO, "Retrun Schedule()");
}   /* PutSchedule	*/

/*
 *  GetTaskID 実行・ログ出力処理
 */
static void
PutTaskID(void)
{
	TaskType task_id;           /* タスクID取得バッファ */

	error_log(GetTaskID(&task_id));

	syslog(LOG_INFO, "TaskID:%d", task_id);
}   /* PutTaskID	*/

/*
 *  GetTaskState 実行・ログ出力処理
 */
static void
PutTaskState(uint8 task_no)
{
	TaskStateType state;        /* タスクID取得バッファ */

	error_log(GetTaskState(task_id_tbl[task_no], &state));

	syslog(LOG_INFO, task_name_tbl[task_no]);
	syslog(LOG_INFO, " State:%s", task_state_tbl[state]);
}   /* PutTaskState	*/

/*
 *  DisableAllInterrupts/EnableAllInterrupts 実行・ログ出力処理
 */
static void
PutDisAllInt(void)
{
	syslog(LOG_INFO, "Call DisableAllInterrupts");

	DisableAllInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call EnableAllInterrupts");

	EnableAllInterrupts();
}   /* PutDisAllInt	*/

/*
 *  SuspendAllInterrupts/ResumeAllInterrupts 実行・ログ出力処理
 */
static void
PutSusAllInt(void)
{
	syslog(LOG_INFO, "Call SuspendAllInterrupts");

	SuspendAllInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call SuspendAllInterrupts");

	SuspendAllInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call ResumeAllInterrupts");

	ResumeAllInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call ResumeAllInterrupts");

	ResumeAllInterrupts();
}   /* PutSusAllInt	*/

/*
 *  SuspendOSInterrupts/ResumeOSInterrupts 実行・ログ出力処理
 */
static void
PutSusOSInt(void)
{
	syslog(LOG_INFO, "Call SuspendOSInterrupts");

	SuspendOSInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call SuspendOSInterrupts");

	SuspendOSInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call SuspendAllInterrupts");

	SuspendAllInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call ResumeAllInterrupts");

	ResumeAllInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call ResumeOSInterrupts");

	ResumeOSInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call ResumeOSInterrupts");

	ResumeOSInterrupts();
}   /* PutSusOSInt */

/*
 *  割込み動作テスト用HWカウンタ値のログ出力処理
 */
static void
PutHwCnt3(void)
{
	uint8	isr1_cnt = 0U;      /* ISR1 カウント値取得バッファ */
	uint8	isr2_cnt = 0U;      /* ISR2 カウント値取得バッファ */
	uint8	cnt;                /* 出力回数カウンタ */

	for (cnt = 0U; cnt < 3U; cnt++) {
		GetHwCnt(&isr1_cnt, &isr2_cnt);
		syslog(LOG_INFO, "ISR1 Cnt:%d, ISR2 Cnt:%d",
			   isr1_cnt, isr2_cnt);
	}
}   /* PutHwCnt3 */

/*
 *  GetResource/ReleaseResource 実行(割込みレベル)・ログ出力処理
 */
static void
PutGetCntRes(void)
{
	syslog(LOG_INFO, "Call GetResource(CntRes)");
	error_log(GetResource(CntRes));

	PutHwCnt3();
	syslog(LOG_INFO, "Call ReleaseResource(CntRes)");

	error_log(ReleaseResource(CntRes));
}   /* PutGetCntRes	*/

/*
 *  GetResource 実行(タスクレベル)・ログ出力処理
 */
static void
PutGetTskRes(void)
{
	syslog(LOG_INFO, "Call GetResource(TskLevelRes)");

	error_log(GetResource(TskLevelRes));
}   /* PutGetTskRes */

/*
 *  ReleaseResource 実行(タスクレベル)・ログ出力処理
 */
static void
PutRelTskRes(void)
{
	syslog(LOG_INFO, "Call ReleaseResource(TskLevelRes)");

	error_log(ReleaseResource(TskLevelRes));
}

/*
 *  SetEvent 実行・ログ出力処理
 */
static void
PutSetEvt(uint8 task_no)
{
	syslog(LOG_INFO, "Call SetEvent(%s, %s)",
		   task_name_tbl[task_no], event_name_tbl[task_no]);

	error_log(SetEvent(task_id_tbl[task_no], event_mask_tbl[task_no]));
}   /* PutSetEvt */

/*
 *  ClearEvent 実行・ログ出力処理
 */
static void
PutClrEvt(uint8 task_no)
{
	syslog(LOG_INFO, "Call ClearEvent(%s)", event_name_tbl[task_no]);

	error_log(ClearEvent(event_mask_tbl[task_no]));
}   /* PutClrEvt */

/*
 *  GetEvent 実行・ログ出力処理
 */
static void
PutGetEvt(uint8 task_no)
{
	EventMaskType mask;             /* イベントマスク取得バッファ */

	error_log(GetEvent(task_id_tbl[task_no], &mask));

	syslog(LOG_INFO, "%s Event Mask:0x%x", task_name_tbl[task_no], mask);
}   /* PutGetEvt */

/*
 *  WaitEvent 実行・ログ出力処理
 */
static void
PutWaitEvt(uint8 task_no)
{
	syslog(LOG_INFO, "Call WaitEvent(%s)", event_name_tbl[task_no]);

	error_log(WaitEvent(event_mask_tbl[task_no]));
	syslog(LOG_INFO, "Return WaitEvent(%s)", event_name_tbl[task_no]);
}   /* PutWaitEvt */

/*
 *  GetAlarmBase 実行・ログ出力処理
 */
static void
PutArmBase(void)
{
	AlarmBaseType info;             /* アラームベース情報取得バッファ */

	error_log(GetAlarmBase(MainCycArm0, &info));

	syslog(LOG_INFO, "MainCycArm0 Base:");
	syslog(LOG_INFO, "\tMAXALLOWEDVALUE=%d", info.maxallowedvalue);
	syslog(LOG_INFO, "\tTICKSPERBASE=%d", info.ticksperbase);
	syslog(LOG_INFO, "\tMINCYCLE=%d", info.mincycle);
}   /* PutArmBase */

/*
 *  PutArmTick 実行・ログ出力処理
 */
static void
PutArmTick(void)
{
	TickType tick;              /* 残りティック取得バッファ */

	error_log(GetAlarm(MainCycArm0, &tick));

	syslog(LOG_INFO, "MainCycArm0 Tick:%d", tick);
}   /* PutArmTick */

/*
 *  SetRelAlarm 実行・ログ出力処理
 */
static void
PutSetRel(uint8 alarm_no, uint8 tick_no, uint8 cycle_no)
{
	syslog(LOG_INFO, "Call SetRelAlarm(%s, %d, %d)",
		   alarm_name_tbl[alarm_no], tick_tbl[tick_no], cycle_tbl[cycle_no]);

	error_log(SetRelAlarm(alarm_id_tbl[alarm_no],
						  tick_tbl[tick_no], cycle_tbl[cycle_no]));
}   /* PutSetRel	*/

/*
 *  SetAbsAlarm 実行・ログ出力処理
 */
static void
PutSetAbs(uint8 alarm_no, uint8 tick_no, uint8 cycle_no)
{
	syslog(LOG_INFO, "Call SetAbsAlarm(%s, %d, %d)",
		   alarm_name_tbl[alarm_no], tick_tbl[tick_no], cycle_tbl[cycle_no]);

	error_log(SetAbsAlarm(alarm_id_tbl[alarm_no],
						  tick_tbl[tick_no], cycle_tbl[cycle_no]));
}   /* PutSetAbs */

/*
 *  CancelAlarm 実行・ログ出力処理
 */
static void
PutCanArm(void)
{
	syslog(LOG_INFO, "Call CancelAlarm(CallBackArm)");

	error_log(CancelAlarm(CallBackArm));
}   /* PutCanArm */

/*
 *  GetActiveApplicationMode 実行・ログ出力処理
 */
static void
PutAppMode(void)
{
	switch (GetActiveApplicationMode()) {
	case AppMode1:
		syslog(LOG_INFO, "ActiveApplicationMode:AppMode1");
		break;
	case AppMode2:
		syslog(LOG_INFO, "ActiveApplicationMode:AppMode2");
		break;
	case AppMode3:
		syslog(LOG_INFO, "ActiveApplicationMode:AppMode3");
		break;
	default:
		syslog(LOG_INFO, "ActiveApplicationMode:Non");
		break;
	}
}   /* PutAppMode */


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
	case OSServiceId_StartCore:
		syslog(LOG_INFO, "Error:%s=StartCore(%d, %d)", atk2_strerror(Error), OSError_StartCore_CoreID(), OSError_StartCore_Status());
		break;
	case OSServiceId_StartNonAutosarCore:
		syslog(LOG_INFO, "Error:%s=StartNonAutosarCore(%d, %d)", atk2_strerror(Error), OSError_StartNonAutosarCore_CoreID(), OSError_StartNonAutosarCore_Status());
		break;
	case OSServiceId_StartOS:
		syslog(LOG_INFO, "Error:%s=StartOS(%d)", atk2_strerror(Error), OSError_StartOS_Mode());
		break;
	case OSServiceId_ShutdownAllCores:
		syslog(LOG_INFO, "Error:%s=ShutdownAllCores()", atk2_strerror(Error));
		break;
	case OSServiceId_IncrementCounter:
		syslog(LOG_INFO, "Error:%s=IncrementCounter(%d)", atk2_strerror(Error), OSError_IncrementCounter_CounterID());
		break;
	case OSServiceId_GetSpinlock:
		syslog(LOG_INFO, "Error:%s=GetSpinlock(%d)", atk2_strerror(Error), OSError_GetSpinlock_SpinlockId());
		break;
	case OSServiceId_ReleaseSpinlock:
		syslog(LOG_INFO, "Error:%s=ReleaseSpinlock(%d)", atk2_strerror(Error), OSError_ReleaseSpinlock_SpinlockId());
		break;
	case OSServiceId_TryToGetSpinlock:
		syslog(LOG_INFO, "Error:%s=TryToGetSpinlock(%d)", atk2_strerror(Error), OSError_GetSpinlock_SpinlockId());
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
	case IOCServiceId_ioc_send_generic:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	case IOCServiceId_ioc_write_generic:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	case IOCServiceId_ioc_receive_generic:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	case IOCServiceId_ioc_read_generic:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_ioc_strerror(Error));
		break;
	default:
		syslog(LOG_INFO, "Error:%s=UnKnownFunc()", atk2_strerror(Error));
		break;
	}

}   /* ErrorHook */
#endif /* CFG_USE_ERRORHOOK */

/*
 *  プレタスクフックルーチン
 *
 *  空ルーチンを呼出す
 */
#ifdef CFG_USE_PRETASKHOOK
void
PreTaskHook(void)
{
}   /* PreTaskHook */
#endif /* CFG_USE_PRETASKHOOK */

/*
 *  ポストタスクフックルーチン
 *
 *  空ルーチンを呼出す
 */
#ifdef CFG_USE_POSTTASKHOOK
void
PostTaskHook(void)
{
}   /* PostTaskHook */
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
	CoreIdType coreid = GetCoreID();

#ifdef TOPPERS_ENABLE_SYS_TIMER
	target_timer_initialize();
#endif /* TOPPERS_ENABLE_SYS_TIMER */
	if (coreid == OS_CORE_ID_MASTER) {
		syslog_initialize();
		syslog_msk_log(LOG_UPTO(LOG_INFO));
		InitSerial();
		print_banner();
	}
	else {
		InitSerial();
	}
	syslog(LOG_EMERG, "StartupHook @ core%d", coreid);
}   /* StartupHook */
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
	syslog(LOG_INFO, "");
	syslog(LOG_INFO, "Sample System ShutDown");
	syslog(LOG_INFO, "ShutDownCode:%s", atk2_strerror(Error));
	syslog(LOG_INFO, "");

	if (Error == E_OS_SYS_ASSERT_FATAL) {
		syslog(LOG_INFO, "fatal_file_name:%s", kernel_fatal_file_name);
		syslog(LOG_INFO, "fatal_line_num:%d", kernel_fatal_line_num);
	}

#ifdef TOPPERS_ENABLE_SYS_TIMER
	target_timer_terminate();
#endif /* TOPPERS_ENABLE_SYS_TIMER */
	TermSerial();

}   /* ShutdownHook */
#endif /* CFG_USE_SHUTDOWNHOOK */

/*
 *  プロテクションフックルーチン
 */
#ifdef CFG_USE_PROTECTIONHOOK
ProtectionReturnType
ProtectionHook(StatusType FatalError)
{
	StatusType ercd;

	syslog(LOG_INFO, "");
	syslog(LOG_INFO, "ProtectionHook");

	if (FatalError == E_OS_STACKFAULT) {
		syslog(LOG_INFO, "E_OS_STACKFAULT");
		ercd = PRO_SHUTDOWN;
	}
	else if (FatalError == E_OS_PROTECTION_EXCEPTION) {
		syslog(LOG_INFO, "E_OS_PROTECTION_EXCEPTION");
		ercd = PRO_IGNORE;
	}
	else {
		ercd = PRO_SHUTDOWN;
	}

	return(ercd);
}
#endif /* CFG_USE_PROTECTIONHOOK */

/*
 *  システムタイマによるアラームコールバック
 */
ALARMCALLBACK(SysTimerAlmCb)
{
	/*
	 *  コールバック実行ログ出力
	 */
	syslog(LOG_INFO, "CallBackArm Expire");

}   /* ALARMCALLBACK(SysTimerAlmCb) */

/*
 *  IncrementCounter確認用アラームコールバック
 */
ALARMCALLBACK(SampleAlmCb)
{
	/*
	 *  コールバック実行ログ出力
	 */
	syslog(LOG_INFO, "SampleArm Expire");

}   /* ALARMCALLBACK( SampleAlmCb ) */

/*
 *  IncrementCounter確認用アラームコールバック
 */
ALARMCALLBACK(SampleAlmCb2)
{
	/*
	 *  コールバック実行ログ出力
	 */
	syslog(LOG_INFO, "SampleArm2 Expire");

}   /* ALARMCALLBACK( SampleAlmCb2 ) */

/*
 *  スケジュールテーブルテスト用メインループ
 *
 *  ユーザコマンドの受信と，コマンドごとの処理実行
 */
static void
schedule_table_sample_routine(void)
{
	uint8					command;                /* コマンドバッファ */
	ScheduleTableType		scheduletable_id;       /* コマンド引数バッファ */
	ScheduleTableStatusType	status;                 /* スケジュール状態引数 */
	TickType				val;                    /* カウンタの現在値 */
	uint8					flag = FALSE;           /* リターンするか判定するためのフラグ */

	syslog(LOG_INFO, "\t[ schedule table sample routine IN, press 't' OUT ]");
	syslog(LOG_INFO, "");

	scheduletable_id = scheduletable1;
	/*
	 *  コマンド実行ループ
	 */
	while (1) {

		WaitEvent(MainEvt0);     /* 10msの作業時間待ち */
		ClearEvent(MainEvt0);

		/*
		 *  入力コマンド取得
		 */
		syslog(LOG_INFO, "Input Command:");
		command = GetCommand(MainEvt0);
		syslog(LOG_INFO, "%c", command);

		/*
		 *  コマンド判定
		 */
		switch (command) {
		case '1':
			scheduletable_id = scheduletable1;
			break;
		case '2':
			scheduletable_id = scheduletable2;
			break;
		case 'i':
			IncrementCounter(SchtblSampleCnt);
			val = 0U;
			GetCounterValue(SchtblSampleCnt, &val);
			if ((val % 5U) == 0U) {
				syslog(LOG_INFO, "\tGetCounterValue(SchtblSampleCnt ) = %d", val);
			}
			break;
		case 's':
			syslog(LOG_INFO, "\tStartScheduleTableRel(scheduletable%d, 5)", scheduletable_id + 1U);
			error_log(StartScheduleTableRel(scheduletable_id, 5U));
			break;
		case 'S':
			syslog(LOG_INFO, "\tStartScheduleTableAbs(scheduletable%d, 5)", scheduletable_id + 1U);
			error_log(StartScheduleTableAbs(scheduletable_id, 5U));
			break;
		case 'f':
			syslog(LOG_INFO, "\tStopScheduleTable(scheduletable%d)", scheduletable_id + 1U);
			error_log(StopScheduleTable(scheduletable_id));
			break;
		case 'n':
			syslog(LOG_INFO, "\tNextScheduleTable(scheduletable%d, scheduletable%d)", scheduletable_id + 1U, scheduletable2 + 1U);
			error_log(NextScheduleTable(scheduletable_id, scheduletable2));
			break;
		case 'N':
			syslog(LOG_INFO, "\tNextScheduleTable(scheduletable%d, scheduletable%d)", scheduletable_id + 1U, scheduletable1 + 1U);
			error_log(NextScheduleTable(scheduletable_id, scheduletable1));
			break;
		case 'g':
			status = 0U;
			syslog(LOG_INFO, "\tGetScheduleTableStatus(scheduletable%d, status)", scheduletable_id + 1U);
			error_log(GetScheduleTableStatus(scheduletable_id, &status));
			syslog(LOG_INFO, "\tstatus = %d", status);
			break;
		case 't':
			syslog(LOG_INFO, "\t[ schedule table sample routine OUT, press 't' IN ]");
			flag = TRUE;
			break;
		case 'q':
			ShutdownAllCores(E_OK);
			break;
		case 'Q':
			ShutdownAllCores(E_OS_STATE);
			break;
		default:
			/* コマンドが上記のケース以外なら処理は行わない */
			break;
		}
		/*  フラグが立っていた場合，リターンする  */
		if (flag == TRUE) {
			return;
		}
	}
}   /* schedule_table_sample_routine */

/*
 *  スケジュールテーブル確認用タスク6
 */
TASK(Task6)
{
	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "Task6 ACTIVATE");
	WaitEvent(T6Evt);
	syslog(LOG_INFO, "Task6 FINISH");
	TerminateTask();
}   /* TASK( Task6 ) */


/*
 *  スケジュールテーブル確認用タスク7
 */
TASK(Task7)
{
	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "Task7 ACTIVATE");
	WaitEvent(T7Evt);
	syslog(LOG_INFO, "Task7 FINISH");
	TerminateTask();
}   /* TASK( Task7 ) */


/*
 *  スケジュールテーブル確認用タスク8
 */
TASK(Task8)
{
	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "Task8 ACTIVATE");
	WaitEvent(T8Evt);
	syslog(LOG_INFO, "Task8 FINISH");
	TerminateTask();
}   /* TASK( Task8 ) */

TASK(IdleTask)
{
	uint8 command;          /* コマンド受信バッファ */

	syslog(LOG_INFO, "IdleTask ACTIVATE");
	while (1) {
		command = GetCommand(MainEvt1);
		syslog(LOG_INFO, "%c", command);
	}
}

TASK(Task9)
{
	TryToGetSpinlockType	success = TRYTOGETSPINLOCK_NOSUCCESS;
	volatile sint32			j;

	syslog(LOG_INFO, "Task9 ACTIVATE");

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
	syslog(LOG_INFO, "(Task9)(1)global variable lock_var1 = %d", lock_var1);
	syslog(LOG_INFO, "(Task9)(2)global variable lock_var2 = %d", lock_var2);
	/* スピンロック解放 */
	error_log(ReleaseSpinlock(SmpSpin));

	syslog(LOG_INFO, "Task9 TERMINATE");
	TerminateTask();
}


/*
 *  ActivateTask(IOC用) 実行・ログ出力処理
 */
static void
PutActIocTsk(void)
{
	uint8 command;          /* コマンド受信バッファ */

	syslog(LOG_INFO, "Select IocTask:");
	command = GetCommand(MainEvt0);
#ifndef OMIT_ECHO
	syslog(LOG_INFO, "%c", command);
#endif /* OMIT_ECHO */

	switch (command) {
	case '1':
		error_log(ActivateTask(IocTask1));
		break;
	case '2':
		error_log(ActivateTask(IocTask2));
		break;
	case '3':
		error_log(ActivateTask(IocTask3));
		break;
	case '4':
		error_log(ActivateTask(IocTask4));
		break;
	default:
		syslog(LOG_INFO, "undefined command");
		SetEvent(MainTask, IocEvt);
		break;
	}
}   /* PutActIocTsk	*/

/*
 *  IOC確認用タスク1
 */
TASK(IocTask1)
{
	syslog(LOG_INFO, "IocTask1 ACTIVATE");

	IocProk();

	syslog(LOG_INFO, "IocTask1 TERMINATE");
	SetEvent(MainTask, IocEvt);
	TerminateTask();
}   /* TASK( IocTask1 ) */

/*
 *  IOC確認用タスク2
 */
TASK(IocTask2)
{
	syslog(LOG_INFO, "IocTask2 ACTIVATE");

	IocProk();

	syslog(LOG_INFO, "IocTask2 TERMINATE");
	SetEvent(MainTask, IocEvt);
	TerminateTask();
}   /* TASK( IocTask2 ) */

/*
 *  IOC確認用タスク3
 */
TASK(IocTask3)
{
	syslog(LOG_INFO, "IocTask3 ACTIVATE");

	IocProk();

	syslog(LOG_INFO, "IocTask3 TERMINATE");
	SetEvent(MainTask, IocEvt);
	TerminateTask();
}   /* TASK( IocTask3 ) */

/*
 *  IOC確認用タスク4
 */
TASK(IocTask4)
{
	syslog(LOG_INFO, "IocTask4 ACTIVATE");

	IocProk();

	syslog(LOG_INFO, "IocTask4 TERMINATE");
	SetEvent(MainTask, IocEvt);
	TerminateTask();
}   /* TASK( IocTask4 ) */

/*
 *  IOC確認用タスク内部処理
 *
 */
static void
IocProk(void)
{
	uint8			api;        /* コマンド受信バッファ(API) */
	uint8			data[3];    /* 通信バッファ */

	volatile sint32 i;
	Std_ReturnType	ercd;

	syslog(LOG_INFO, "Select IOC API:");
	api = GetCommandNoWait();
#ifndef OMIT_ECHO
	syslog(LOG_INFO, "%c", api);
#endif /* OMIT_ECHO */

	switch (api) {
	case '1':
		syslog(LOG_INFO, "Input send data:");
		data[0] = GetCommandNoWait();
#ifndef OMIT_ECHO
		syslog(LOG_INFO, "%c", data[0]);
#endif /* OMIT_ECHO */

		syslog(LOG_INFO, "Call IocSend_IOC_QUE_0");
		error_log_ioc(IocSend_IOC_QUE_0(data[0]));
		break;
	case '2':
		syslog(LOG_INFO, "Input send data:");
		data[0] = GetCommandNoWait();
#ifndef OMIT_ECHO
		syslog(LOG_INFO, "%c", data[0]);
#endif /* OMIT_ECHO */

		syslog(LOG_INFO, "Call IocSend_IOC_QUE_1");
		error_log_ioc(IocSend_IOC_QUE_1(data[0]));
		break;
	case '3':
		for (i = 0; i < 3U; i++) {
			syslog(LOG_INFO, "Input send data%d:", i);
			data[i] = GetCommandNoWait();
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
}   /* IocProk */

ICISR(Core0_ICI_1) {

	syslog(LOG_INFO, "/***********************/\nCoer0_ICI_1 run!");
	syslog(LOG_INFO, "Coer0_ICI_1 GetISRID() = %d", GetISRID());
	syslog(LOG_INFO, "Coer0_ICI_1 GetApplicationID() = %d", GetApplicationID());
	RaiseInterCoreInterrupt(Core1_ICI_2);
}

ICISR(Core1_ICI_1) {
	syslog(LOG_INFO, "/***********************/\nCoer1_ICI_1 run!");
	syslog(LOG_INFO, "Coer1_ICI_1 GetISRID() = %d", GetISRID());
	syslog(LOG_INFO, "Coer1_ICI_1 GetApplicationID() = %d", GetApplicationID());
	RaiseInterCoreInterrupt(Core0_ICI_2);
}

ICISR(Core0_ICI_2) {
	syslog(LOG_INFO, "/***********************/\nCoer0_ICI_2 run!");
	syslog(LOG_INFO, "Coer0_ICI_2 GetISRID() = %d", GetISRID());
	syslog(LOG_INFO, "Coer0_ICI_2 GetApplicationID() = %d", GetApplicationID());
}

ICISR(Core1_ICI_2) {
	syslog(LOG_INFO, "/***********************/\nCoer1_ICI_2 run!");
	syslog(LOG_INFO, "Coer1_ICI_2 GetISRID() = %d", GetISRID());
	syslog(LOG_INFO, "Coer1_ICI_2 GetApplicationID() = %d", GetApplicationID());
}

ICISR(Core0_ICI_3) {
	syslog(LOG_INFO, "/***********************/\nCoer0_ICI_3 run!");
	syslog(LOG_INFO, "Coer0_ICI_3 GetISRID() = %d", GetISRID());
	syslog(LOG_INFO, "Coer0_ICI_3 GetApplicationID() = %d", GetApplicationID());
}
