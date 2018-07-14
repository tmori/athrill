/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2015-2016 by Ushio Laboratory
 *              Graduate School of Engineering Science, Osaka Univ., JAPAN
 *  Copyright (C) 2015-2016 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
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
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  $Id: tSample2.c 509 2016-01-12 06:06:14Z ertl-hiro $
 */

/* 
 *  サンプルプログラム(1)の本体
 *
 *  ASPカーネルの基本的な動作を確認するためのサンプルプログラム．
 *
 *  プログラムの概要:
 *
 *  ユーザインタフェースを受け持つメインタスク（タスクID: MAIN_TASK，優
 *  先度: MAIN_PRIORITY）と，3つの並行実行されるタスク（タスクID:
 *  TASK1〜TASK3，初期優先度: MID_PRIORITY）で構成される．また，起動周
 *  期が2秒の周期ハンドラ（周期ハンドラID: CYCHDR1）を用いる．
 *
 *  並行実行されるタスクは，task_loop回空ループを実行する度に，タスクが
 *  実行中であることをあらわすメッセージを表示する．空ループを実行する
 *  のは，空ループなしでメッセージを出力すると，多量のメッセージが出力
 *  され，プログラムの動作が確認しずらくなるためである．また，低速なシ
 *  リアルポートを用いてメッセージを出力する場合に，すべてのメッセージ
 *  が出力できるように，メッセージの量を制限するという理由もある．
 *
 *  周期ハンドラは，三つの優先度（HIGH_PRIORITY，MID_PRIORITY，
 *  LOW_PRIORITY）のレディキューを回転させる．プログラムの起動直後は，
 *  周期ハンドラは停止状態になっている．
 *
 *  メインタスクは，シリアルI/Oポートからの文字入力を行い（文字入力を
 *  待っている間は，並列実行されるタスクが実行されている），入力された
 *  文字に対応した処理を実行する．入力された文字と処理の関係は次の通り．
 *  Control-Cまたは'Q'が入力されると，プログラムを終了する．
 *
 *  '1' : 対象タスクをTASK1に切り換える（初期設定）．
 *  '2' : 対象タスクをTASK2に切り換える．
 *  '3' : 対象タスクをTASK3に切り換える．
 *  'a' : 対象タスクをcTask_activateにより起動する．
 *  'A' : 対象タスクに対する起動要求をcTask_cancelActivateによりキャンセルする．
 *  'e' : 対象タスクにexitTaskを呼び出させ，終了させる．
 *  't' : 対象タスクをcTask_terminateにより強制終了する．
 *  '>' : 対象タスクの優先度をHIGH_PRIORITYにする．
 *  '=' : 対象タスクの優先度をMID_PRIORITYにする．
 *  '<' : 対象タスクの優先度をLOW_PRIORITYにする．
 *  'G' : 対象タスクの優先度をcTask_getPriorityで読み出す．
 *  's' : 対象タスクにsleepを呼び出させ，起床待ちにさせる．
 *  'S' : 対象タスクにsleepTimeout10秒)を呼び出させ，起床待ちにさせる．
 *  'w' : 対象タスクをcTask_wakeupにより起床する．
 *  'W' : 対象タスクに対する起床要求をcTask_cancelWakeupによりキャンセルする．
 *  'l' : 対象タスクをcTask_releaseWaitにより強制的に待ち解除にする．
 *  'u' : 対象タスクをcTask_suspendにより強制待ち状態にする．
 *  'm' : 対象タスクの強制待ち状態をcTask_resumeにより解除する．
 *  'd' : 対象タスクにdelay(10秒)を呼び出させ，時間経過待ちにさせる．
 *  'x' : 対象タスクにraiseTerminateにより終了要求する．
 *  'y' : 対象タスクにdisableTerminateを呼び出させ，タスク終了を禁止する．
 *  'Y' : 対象タスクにenableTerminateを呼び出させ，タスク終了を許可する．
 *  'r' : 3つの優先度（HIGH_PRIORITY，MID_PRIORITY，LOW_PRIORITY）のレ
 *        ディキューを回転させる．
 *  'c' : 周期ハンドラを動作開始させる．
 *  'C' : 周期ハンドラを動作停止させる．
 *  'b' : アラームハンドラを5秒後に起動するよう動作開始させる．
 *  'B' : アラームハンドラを動作停止させる．
 *  'z' : 対象タスクにCPU例外を発生させる（タスクを終了させる）．
 *  'Z' : 対象タスクにCPUロック状態でCPU例外を発生させる（プログラムを
 *        終了する）．
 *  'V' : fetchHighResolutionTimerで高分解能タイマを2回読む．
 *  'v' : 発行したシステムコールを表示する（デフォルト）．
 *  'q' : 発行したシステムコールを表示しない．
 * 呼び口関数 #_TCPF_#
 * require port: signature:sKernel context:task
 *   ER             getExtendedInformation( intptr_t* p_exinf );
 *   ER             sleep( );
 *   ER             sleepTimeout( TMO timeout );
 *   ER             delay( RELTIM delayTime );
 *   ER             exit( );
 *   ER             disableTerminate( );
 *   ER             enableTerminate( );
 *   bool_t         senseTerminate( );
 *   ER             setTime( SYSTIM systemTime );
 *   ER             getTime( SYSTIM* p_systemTime );
 *   ER             adjustTime( int32_t adjustTime );
 *   HRTCNT         fetchHighResolutionTimer( );
 *   ER             rotateReadyQueue( PRI taskPriority );
 *   ER             getTaskId( ID* p_taskId );
 *   ER             getLoad( PRI taskPriority, uint_t* p_load );
 *   ER             getNthTask( PRI taskPriority, uint_t nth, ID* p_taskID );
 *   ER             lockCpu( );
 *   ER             unlockCpu( );
 *   ER             disableDispatch( );
 *   ER             enableDispatch( );
 *   bool_t         senseContext( );
 *   bool_t         senseLock( );
 *   bool_t         senseDispatch( );
 *   bool_t         senseDispatchPendingState( );
 *   bool_t         senseKernel( );
 *   ER             exitKernel( );
 *   ER             changeInterruptPriorityMask( PRI interruptPriority );
 *   ER             getInterruptPriorityMask( PRI* p_interruptPriority );
 * require port: signature:siKernel context:non-task
 *   HRTCNT         ciKernel_fetchHighResolutionTimer( );
 *   ER             ciKernel_rotateReadyQueue( PRI taskPriority );
 *   ER             ciKernel_getTaskId( ID* p_taskId );
 *   ER             ciKernel_lockCpu( );
 *   ER             ciKernel_unlockCpu( );
 *   bool_t         ciKernel_senseContext( );
 *   bool_t         ciKernel_senseLock( );
 *   bool_t         ciKernel_senseDispatch( );
 *   bool_t         ciKernel_senseDispatchPendingState( );
 *   bool_t         ciKernel_senseKernel( );
 *   ER             ciKernel_exitKernel( );
 *   bool_t         ciKernel_exceptionSenseDispatchPendingState( const void* p_exceptionInformation );
 * call port: cTask signature: sTask context:task
 *   ER             cTask_activate( subscript );
 *   ER_UINT        cTask_cancelActivate( subscript );
 *   ER             cTask_getTaskState( subscript, STAT* p_tskstat );
 *   ER             cTask_changePriority( subscript, PRI priority );
 *   ER             cTask_getPriority( subscript, PRI* p_priority );
 *   ER             cTask_refer( subscript, T_RTSK* pk_taskStatus );
 *   ER             cTask_wakeup( subscript );
 *   ER_UINT        cTask_cancelWakeup( subscript );
 *   ER             cTask_releaseWait( subscript );
 *   ER             cTask_suspend( subscript );
 *   ER             cTask_resume( subscript );
 *   ER             cTask_raiseTerminate( subscript );
 *   ER             cTask_terminate( subscript );
 *       subscript:  0...(NCP_cTask-1)
 * call port: cExceptionTask signature: sTask context:task
 *   ER             cExceptionTask_activate( );
 *   ER_UINT        cExceptionTask_cancelActivate( );
 *   ER             cExceptionTask_getTaskState( STAT* p_tskstat );
 *   ER             cExceptionTask_changePriority( PRI priority );
 *   ER             cExceptionTask_getPriority( PRI* p_priority );
 *   ER             cExceptionTask_refer( T_RTSK* pk_taskStatus );
 *   ER             cExceptionTask_wakeup( );
 *   ER_UINT        cExceptionTask_cancelWakeup( );
 *   ER             cExceptionTask_releaseWait( );
 *   ER             cExceptionTask_suspend( );
 *   ER             cExceptionTask_resume( );
 *   ER             cExceptionTask_raiseTerminate( );
 *   ER             cExceptionTask_terminate( );
 * call port: cCyclic signature: sCyclic context:task
 *   ER             cCyclic_start( );
 *   ER             cCyclic_stop( );
 *   ER             cCyclic_refer( T_RCYC* pk_cyclicHandlerStatus );
 * call port: cAlarm signature: sAlarm context:task
 *   ER             cAlarm_start( RELTIM alarmTime );
 *   ER             cAlarm_stop( );
 *   ER             cAlarm_refer( T_RALM* pk_alarmStatus );
 * call port: cSerialPort signature: sSerialPort context:task optional:true
 *   bool_t     is_cSerialPort_joined()                     check if joined
 *   ER             cSerialPort_open( );
 *   ER             cSerialPort_close( );
 *   ER_UINT        cSerialPort_read( char* buffer, uint_t length );
 *   ER_UINT        cSerialPort_write( const char* buffer, uint_t length );
 *   ER             cSerialPort_control( uint_t ioControl );
 *   ER             cSerialPort_refer( T_SERIAL_RPOR* pk_rpor );
 * call port: cSysLog signature: sSysLog context:task
 *   ER             cSysLog_write( uint_t priority, const SYSLOG* p_syslog );
 *   ER_UINT        cSysLog_read( SYSLOG* p_syslog );
 *   ER             cSysLog_mask( uint_t logMask, uint_t lowMask );
 *   ER             cSysLog_refer( T_SYSLOG_RLOG* pk_rlog );
 *   ER             cSysLog_flush( );
 * #[</PREAMBLE>]# */  

#include "tSample2_tecsgen.h"
#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "kernel_cfg.h"
#include "tSample2.h"

/*
 *  サービスコールのエラーのログ出力
 */
Inline void
svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}

#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

/*
 *  並行実行されるタスクへのメッセージ領域
 */
char	message[3];

/*
 *  ループ回数
 */
ulong_t	task_loop;		/* タスク内でのループ回数 */

/*
 *  並行実行されるタスク
 */
void eSampleTask_main(int_t subscript)
{
	volatile ulong_t	i;
	int_t		n = 0;
	int_t		tskno = subscript + 1; 
	const char	*graph[] = { "|", "  +", "    *" };
	char		c;

	while (true) {
		syslog(LOG_NOTICE, "task%d is running (%03d).   %s",
										tskno, ++n, graph[tskno-1]);
		for (i = 0; i < task_loop; i++);
		c = message[tskno-1];
		message[tskno-1] = 0;
		switch (c) {
		case 'e':
			syslog(LOG_INFO, "#%d#exit()", tskno);
			SVC_PERROR(exit());
			assert(0);
		case 's':
			syslog(LOG_INFO, "#%d#sleep()", tskno);
			SVC_PERROR(sleep());
			break;
		case 'S':
			syslog(LOG_INFO, "#%d#sleepTimeout(10000)", tskno);
			SVC_PERROR(sleepTimeout(10000));
			break;
		case 'd':
			syslog(LOG_INFO, "#%d#delay(10000)", tskno);
			SVC_PERROR(delay(10000));
			break;
		case 'y':
			syslog(LOG_INFO, "#%d#disableTerminate()", tskno);
			SVC_PERROR(disableTerminate());
			break;
		case 'Y':
			syslog(LOG_INFO, "#%d#enableTerminate()", tskno);
			SVC_PERROR(enableTerminate());
			break;
#ifdef CPUEXC1
		case 'z':
			syslog(LOG_NOTICE, "#%d#raise CPU exception", tskno);
			RAISE_CPU_EXCEPTION;
			break;
		case 'Z':
			SVC_PERROR(lockCpu());
			syslog(LOG_NOTICE, "#%d#raise CPU exception", tskno);
			RAISE_CPU_EXCEPTION;
			SVC_PERROR(unlockCpu());
			break;
#endif /* CPUEXC1 */
		default:
			break;
		}
	}
}

/*
 *  CPU例外ハンドラ
 */

ID	cpuexc_tskid;		/* CPU例外を起こしたタスクのID */

#ifdef CPUEXC1

void
cpuexc_handler(void *p_excinf)
{
	
	syslog(LOG_NOTICE, "CPU exception handler (p_excinf = %08p).", p_excinf);
	if (ciKernel_senseContext() != true) {
		syslog(LOG_WARNING,
					"ciKernel_senseContext() is not true in CPU exception handler.");
	}
	if (ciKernel_senseDispatchPendingState() != true) {
		syslog(LOG_WARNING,
					"ciKernel_senseDispatchPendingState() is not true in CPU exception handler.");
	}
	syslog(LOG_INFO, "ciKernel_senseLock() = %d ciKernel_senseDispatch() = %d ciKernel_exceptionSenseDispatchPendingState() = %d",
		   ciKernel_senseLock(), ciKernel_senseDispatch(), ciKernel_exceptionSenseDispatchPendingState(p_excinf));

	if (ciKernel_exceptionSenseDispatchPendingState(p_excinf)) {
		syslog(LOG_NOTICE, "Sample program ends with exception.");
		SVC_PERROR(ciKernel_senseKernel());
		assert(0);
	}

#ifdef PREPARE_RETURN_CPUEXC
	PREPARE_RETURN_CPUEXC;
	SVC_PERROR(ciKernel_getTaskId(&cpuexc_tskid));
	cExceptionTask_activate();
#else /* PREPARE_RETURN_CPUEXC */
	syslog(LOG_NOTICE, "Sample program ends with exception.");
	SVC_PERROR(ciKernel_exitKernel());
	assert(0);
#endif /* PREPARE_RETURN_CPUEXC */
}

#endif /* CPUEXC1 */

/*
 *  周期ハンドラ
 *
 *  HIGH_PRIORITY，MID_PRIORITY，LOW_PRIORITY の各優先度のレディキュー
 *  を回転させる．
 */
/* #[<ENTRY_FUNC>]# eiCyclicHandler_main
 * name:         eiCyclicHandler_main
 * global_name:  tSample2_eiCyclicHandler_main
 * oneway:       
 * #[/ENTRY_FUNC>]# */
void
eiCyclicHandler_main()
{
	SVC_PERROR(ciKernel_rotateReadyQueue(HIGH_PRIORITY));
	SVC_PERROR(ciKernel_rotateReadyQueue(MID_PRIORITY));
	SVC_PERROR(ciKernel_rotateReadyQueue(LOW_PRIORITY));
}

/*
 *  アラームハンドラ
 *
 *  HIGH_PRIORITY，MID_PRIORITY，LOW_PRIORITY の各優先度のレディキュー
 *  を回転させる．
 */
/* #[<ENTRY_FUNC>]# eiAlarmHandler_main
 * name:         eiAlarmHandler_main
 * global_name:  tSample2_eiAlarmHandler_main
 * oneway:       
 * #[/ENTRY_FUNC>]# */
void
eiAlarmHandler_main()
{
	SVC_PERROR(ciKernel_rotateReadyQueue(HIGH_PRIORITY));
	SVC_PERROR(ciKernel_rotateReadyQueue(MID_PRIORITY));
	SVC_PERROR(ciKernel_rotateReadyQueue(LOW_PRIORITY));
}
/*
 *  例外処理タスク
 */
/* #[<ENTRY_FUNC>]# eExceptionTask_main
 * name:         eExceptionTask_main
 * global_name:  tSample2_eExceptionTask_main
 * oneway:       
 * #[/ENTRY_FUNC>]# */
void eExceptionTask_main()
{
	SVC_PERROR(ras_ter(cpuexc_tskid));
}
/*
 *  メインタスク
 */
/* 属性の設定 *//* #[<ENTRY_FUNC>]# eMainTask_main
 * name:         eMainTask_main
 * global_name:  tSample2_eMainTask_main
 * oneway:       
 * #[/ENTRY_FUNC>]# */
void
eMainTask_main()
{
	char	c;
	volatile ulong_t	i;
	ulong_t j;
	int_t	tskno = 1;
	ER_UINT	ercd;
	PRI		tskpri;
	SYSTIM	stime1, stime2;
	HRTCNT	hrtcnt1, hrtcnt2;

	if (is_cSerialPort_joined()) {
		SVC_PERROR(cSysLog_mask(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_ERROR)));
	} else {
		/*
		 *  シリアル出力ができない場合、ログメッセージを全て
		 *  低レベル出力により出力する。
		 */
		SVC_PERROR(cSysLog_mask(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_DEBUG)));
	}
	syslog(LOG_NOTICE, "Sample program starts.");

	/*
	 *  シリアルポートの初期化
	 *
	 *  システムログタスクと同じシリアルポートを使う場合など，シリアル
	 *  ポートがオープン済みの場合にはここでE_OBJエラーになるが，支障は
	 *  ない．
	 */

	if (is_cSerialPort_joined()) {
		ercd = cSerialPort_open();
		if (ercd < 0 && MERCD(ercd) != E_OBJ) {
			syslog(LOG_ERROR, "%s (%d) reported by `cSerialPort_open'.",
										itron_strerror(ercd), SERCD(ercd));
		}
		SVC_PERROR(cSerialPort_control(IOCTL_CRLF | IOCTL_FCSND | IOCTL_FCRCV));
	}

	/*
 	 *  ループ回数の設定
	 *
	 *  並行実行されるタスク内での空ループの回数（task_loop）は，空ルー
	 *  プの実行時間が約0.4秒になるように設定する．この設定のために，
	 *  LOOP_REF回の空ループの実行時間を，その前後でget_timを呼ぶことで
	 *  測定し，その測定結果から空ループの実行時間が0.4秒になるループ回
	 *  数を求め，task_loopに設定する．
	 *
	 *  LOOP_REFは，デフォルトでは1,000,000に設定しているが，想定したよ
	 *  り遅いプロセッサでは，サンプルプログラムの実行開始に時間がかか
	 *  りすぎるという問題を生じる．逆に想定したより速いプロセッサでは，
	 *  LOOP_REF回の空ループの実行時間が短くなり，task_loopに設定する値
	 *  の誤差が大きくなるという問題がある．
	 *
	 *  そこで，そのようなターゲットでは，target_test.hで，LOOP_REFを適
	 *  切な値に定義するのが望ましい．
	 *
	 *  また，task_loopの値を固定したい場合には，その値をTASK_LOOPにマ
	 *  クロ定義する．TASK_LOOPがマクロ定義されている場合，上記の測定を
	 *  行わずに，TASK_LOOPに定義された値を空ループの回数とする．
	 *
	 * ターゲットによっては，空ループの実行時間の1回目の測定で，本来よ
	 * りも長めになるものがある．このようなターゲットでは，MEASURE_TWICE
	 * をマクロ定義することで，1回目の測定結果を捨てて，2回目の測定結果
	 * を使う．
	 */
#ifdef TASK_LOOP
	task_loop = TASK_LOOP;
#else /* TASK_LOOP */

#ifdef MEASURE_TWICE
	task_loop = LOOP_REF;
	SVC_PERROR(get_tim(&stime1));
	for (i = 0; i < task_loop; i++);
	SVC_PERROR(get_tim(&stime2));
#endif /* MEASURE_TWICE */

	task_loop = LOOP_REF;
	SVC_PERROR(get_tim(&stime1));
	for (i = 0; i < task_loop; i++);
	SVC_PERROR(get_tim(&stime2));
	task_loop = LOOP_REF * 400LU / (ulong_t)(stime2 - stime1) * 1000LU;

#endif /* TASK_LOOP */

	/*
 	 *  タスクの起動
	 */

	SVC_PERROR(cTask_activate( 1 ));
	SVC_PERROR(cTask_activate( 2 ));
	SVC_PERROR(cTask_activate( 3 ));

	/*
 	 *  メインループ
	 */
	if (is_cSerialPort_joined()) {
		do {
			SVC_PERROR(cSerialPort_read(&c, 1));
			switch (c) {
			case 'e':
			case 's':
			case 'S':
			case 'd':
			case 'y':
			case 'Y':
			case 'z':
			case 'Z':
				message[tskno-1] = c;
				break;
			case '1':
				tskno = 1;
				break;
			case '2':
				tskno = 2;
				break;
			case '3':
				tskno = 3;
				break;
			case 'a':
				syslog(LOG_INFO, "#cTask_activate(%d)", tskno);
				SVC_PERROR(cTask_activate(tskno));
				break;
			case 'A':
				syslog(LOG_INFO, "#cTask_cancelActivate(%d)", tskno);
				SVC_PERROR(cTask_cancelActivate(tskno));

				if (ercd >= 0) {
					syslog(LOG_NOTICE, "cTask_cancelActivate(%d) returns %d", tskno, ercd);
				}
				break;
			case 't':
				syslog(LOG_INFO, "#cTask_terminate(%d)", tskno);
				SVC_PERROR(cTask_terminate(tskno));
				break;
			case '>':
				syslog(LOG_INFO, "#cTask_changePriority(%d, HIGH_PRIORITY)", tskno);
				SVC_PERROR(cTask_changePriority(tskno, HIGH_PRIORITY));
				break;
			case '=':
				syslog(LOG_INFO, "#cTask_changePriority(%d, MID_PRIORITY)", tskno);
				SVC_PERROR(cTask_changePriority(tskno, MID_PRIORITY));
				break;
			case '<':
				syslog(LOG_INFO, "#(cTask_changePriority(%d, LOW_PRIORITY)", tskno);
				SVC_PERROR(cTask_changePriority(tskno, LOW_PRIORITY));
				break;
			case 'G':
				syslog(LOG_INFO, "#cTask_getPriority(%d, &tskpri)", tskno);
				SVC_PERROR(ercd = cTask_getPriority(tskno, &tskpri));
				if (ercd >= 0) {
					syslog(LOG_NOTICE, "priority of task %d is %d", tskno, tskpri);
				}
				break;
			case 'w':
				syslog(LOG_INFO, "#cTask_wakeup(%d)", tskno);
				SVC_PERROR(cTask_wakeup(tskno));
				break;
			case 'W':
				syslog(LOG_INFO, "#cTask_cancelWakeup(%d)", tskno);
				SVC_PERROR(ercd = cTask_cancelWakeup(tskno));
				if (ercd >= 0) {
					syslog(LOG_NOTICE, "cTask_cancelWakeup(%d) returns %d", tskno, ercd);
				}
				break;
			case 'l':
				syslog(LOG_INFO, "#cTask_releaseWait(%d)", tskno);
				SVC_PERROR(cTask_releaseWait(tskno));
				break;
			case 'u':
				syslog(LOG_INFO, "#cTask_suspend(%d)", tskno);
				SVC_PERROR(cTask_suspend(tskno));
				break;
			case 'm':
				syslog(LOG_INFO, "#cTask_resume(%d)", tskno);
				SVC_PERROR(cTask_resume(tskno));
				break;
			case 'x':
				syslog(LOG_INFO, "#cTask_raiseTerminate(%d)", tskno);
				SVC_PERROR(cTask_raiseTerminate(tskno));
				break;
			case 'X':
				syslog(LOG_INFO, "#cTask_raiseTerminate(%d)", tskno);
				SVC_PERROR(cTask_raiseTerminate(tskno));
				break;
			case 'r':
				syslog(LOG_INFO, "#rotateReadyQueue(three priorities)");
				SVC_PERROR(rotateReadyQueue(HIGH_PRIORITY));
				SVC_PERROR(rotateReadyQueue(MID_PRIORITY));
				SVC_PERROR(rotateReadyQueue(LOW_PRIORITY));
				break;
			case 'c':
				syslog(LOG_INFO, "#cCyclic_start(1)");
				SVC_PERROR(cCyclic_start());
				break;
			case 'C':
				syslog(LOG_INFO, "#cCyclic_stop(1)");
				SVC_PERROR(cCyclic_stop());
				break;
			case 'b':
				syslog(LOG_INFO, "#cAlarm_start(1, 5000000)");
				SVC_PERROR(cAlarm_start(5000000));
				break;
			case 'B':
				syslog(LOG_INFO, "#cAlarm_stop()(1)");
				SVC_PERROR(cAlarm_stop());
				break;
			case 'V':
				hrtcnt1 = fetchHighResolutionTimer();
				hrtcnt2 = fetchHighResolutionTimer();
				syslog(LOG_NOTICE, "hrtcnt1 = %tu, hrtcnt2 = %tu",
											hrtcnt1, hrtcnt2);
				break;
			case 'v':
				SVC_PERROR(cSysLog_mask(LOG_UPTO(LOG_INFO),
											LOG_UPTO(LOG_EMERG)));
				break;
			case 'q':
				SVC_PERROR(cSysLog_mask(LOG_UPTO(LOG_NOTICE),
											LOG_UPTO(LOG_EMERG)));
				break;
#ifdef BIT_KERNEL
			case ' ':
				SVC_PERROR(lockCpu());
				{
					extern ER	bit_kernel(void);

					SVC_PERROR(ercd = bit_kernel());
					if (ercd >= 0) {
						syslog(LOG_NOTICE, "bit_kernel passed.");
					}
				}
				SVC_PERROR(unlockCpu());
				break;
#endif /* BIT_KERNEL */

			default:
				break;
			}
		} while (c != '\003' && c != 'Q');
	} else {
		syslog(LOG_NOTICE, "cSerialPort of tSample2 is not joined.");
		syslog(LOG_NOTICE, "Sample program will halt after 40 seconds.");
		for (j = 0; j < 100; j++) {
			for (i = 0; i < task_loop; i++);
		}
	}

	syslog(LOG_NOTICE, "Sample program ends.");
	SVC_PERROR(ciKernel_exitKernel());
	assert(0);
}
