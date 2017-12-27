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
 *  $Id: sample2.c 425 2015-12-07 08:06:19Z witz-itoyo $
 */

/*
 *		非信頼OSアプリケーション所属サンプルプログラムの本体
 *
 *  サンプルプログラムの動作はsample1.cのコメント参照
 *  本ファイルは非信頼OSアプリケーションに所属している
 *  オブジェクトを記述している
 */

#include "sample.h"
#include "sample2.h"
#include "Ioc.h"

#define GetHwCnt(x, y)

/*
 *  内部関数プロトタイプ宣言
 */
uint8 GetCommand(void);
void PutActTsk(uint8 task_no);
void PutActNonPriTsk(void);
void PutTermTsk(uint8 task_no);
void PutChainTsk(uint8 from_task_no, uint8 to_task_no);
void PutSchedule(void);
void PutTaskID(void);
void PutTaskState(uint8 task_no);
void PutDisAllInt(void);
void PutSusAllInt(void);
void PutSusOSInt(void);
void PutHwCnt3(void);
void PutGetCntRes(void);
void PutGetTskRes(void);
void PutRelTskRes(void);
void PutSetEvt(uint8 task_no);
void PutClrEvt(uint8 task_no);
void PutGetEvt(uint8 task_no);
void PutWaitEvt(uint8 task_no);
void PutArmBase(void);
void PutArmTick(void);
void PutSetRel(uint8 alarm_no, uint8 tick_no, uint8 cycle_no);
void PutCanArm(void);
void PutAppMode(void);
void schedule_table_sample_routine(void);

TASK(IocTask1);
TASK(IocTask2);
static void IocProk1(void);
static uint8 GetCommandNoWait2(void);

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
 *  内部データバッファ
 */
volatile uint8				command_tbl[14]; /* コマンド引渡しテーブル */

/*
 *  内部定数データテーブル
 */
/* 無効イベントマスク値 */
#define invalid_mask	(EventMaskType) (0)

/* イベントマスクテーブル */
static const EventMaskType	event_mask_tbl[5] = {
	invalid_mask,
	T2Evt,
	T3Evt,
	invalid_mask,
	invalid_mask
};

/* タスクIDテーブル */
static const TaskType		task_id_tbl[14] = {
	Task1,
	Task2,
	Task3,
	Task4,
	Task5,
	Task6,
	Task7,
	Task8,
	Task9,
	Task10,
	Task11,
	Task12,
	Task13,
	Task14
};

/* アラームIDテーブル */
static const AlarmType		alarm_id_tbl[2] = {
	ActTskArm,
	SetEvtArm
};

/* ティック値テーブル */
static const TickType		tick_tbl[2] = {
	(TickType) 500,
	(TickType) 900
};

/* サイクル値テーブル */
static const TickType		cycle_tbl[2] = {
	(TickType) 0,
	(TickType) COUNTER_MIN_CYCLE
};

/* イベントマスク名文字列テーブル */
static const char8			*event_name_tbl[5] = {
	"Invalid",
	"T2Evt",
	"T3Evt",
	"Invalid",
	"Invalid"
};

/* タスク名文字列テーブル */
static const char8			*task_name_tbl[14] = {
	"Task1",
	"Task2",
	"Task3",
	"Task4",
	"Task5",
	"Task6",
	"Task7",
	"Task8",
	"Task9",
	"Task10",
	"Task11",
	"Task12"
	"Task13"
	"Task14"
};

/* タスク状態文字列テーブル */
static const char8			*task_state_tbl[4] = {
	"SUSPENDED",
	"RUNNING",
	"READY",
	"WAITING"
};

/* アラーム名文字列テーブル */
static const char8			*alarm_name_tbl[2] = {
	"ActTskArm",
	"SetEvtArm"
};

DEFINE_VAR_USTACK(StackType, stack_1[COUNT_STK_T(0x200)], ".stack_section");
DEFINE_VAR_SSTACK(StackType, stack_2[COUNT_STK_T(0x200)]);

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
 *  並列実行タスク1
 */
TASK(Task1)
{
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
void
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
			if (task_no < 5U) {
				PutClrEvt(task_no);
			}
			break;
		case 'W':
			if (task_no < 5U) {
				PutWaitEvt(task_no);
			}
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
uint8
GetCommand(void)
{
	uint8 command;          /* コマンド受信バッファ */

	/*
	 *  コマンドを受信するまでループ
	 */
	command = '\0';
	do {
		WaitEvent(MainEvt);     /* 10msウェイト */
		ClearEvent(MainEvt);
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
void
PutActTsk(uint8 task_no)
{
	struct parameter_struct local;

	syslog(LOG_INFO, "Call ActivateTask(%s)", task_name_tbl[task_no]);

	if (task_no < 5U) {
		error_log(ActivateTask(task_id_tbl[task_no]));
	}
	else {
		local.task_no = task_id_tbl[task_no];
		CallTrustedFunction(tfnt4, &local); /* ActivateTask(task_name_tbl[task_no]) */
	}

}   /* PutActTsk	*/

/*
 *  ActivateTask 実行(NonPriTask)・ログ出力処理
 */
void
PutActNonPriTsk(void)
{
	struct parameter_struct local;

	syslog(LOG_INFO, "Call ActivateTask(NonPriTask)");

	local.task_no = NonPriTask;
	CallTrustedFunction(tfnt4, &local); /* ActivateTask(NonPriTask) */

}   /* PutActNonPriTsk	*/

/*
 *  TerminateTask 実行・ログ出力処理
 */
void
PutTermTsk(uint8 task_no)
{
	StatusType ercd;        /* エラーコード */

	syslog(LOG_INFO, "%s TERMINATE", task_name_tbl[task_no]);

	ercd = TerminateTask();
	ShutdownOS(ercd);
}

/*
 *  ChainTask 実行・ログ出力処理
 */
void
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
	ShutdownOS(ercd);
}   /* PutChainTsk */

/*
 *  Schedule 実行・ログ出力処理
 */
void
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
void
PutTaskID(void)
{
	TaskType task_id;           /* タスクID取得バッファ */

	error_log(GetTaskID(&task_id));

	syslog(LOG_INFO, "TaskID:%d", task_id);
}   /* PutTaskID	*/

/*
 *  GetTaskState 実行・ログ出力処理
 *  Task1〜Task5，Task7取得時エラーとならない
 *  Task6取得時エラーとなる
 */
void
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
void
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
void
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
void
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
void
PutHwCnt3(void)
{
	uint32	isr1_cnt = 0U;      /* C1ISR カウント値取得バッファ */
	uint32	isr2_cnt = 0U;      /* C2ISR カウント値取得バッファ */
	uint8	cnt;                /* 出力回数カウンタ */

	for (cnt = 0U; cnt < 3U; cnt++) {
		GetHwCnt(&isr1_cnt, &isr2_cnt);
		syslog(LOG_INFO, "C1ISR Cnt:%d, C2ISR Cnt:%d",
			   isr1_cnt, isr2_cnt);
	}
}   /* PutHwCnt3 */

/*
 *  GetResource/ReleaseResource 実行(割込みレベル)・ログ出力処理
 */
void
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
void
PutGetTskRes(void)
{
	syslog(LOG_INFO, "Call GetResource(TskLevelRes)");

	error_log(GetResource(TskLevelRes));
}   /* PutGetTskRes */

/*
 *  ReleaseResource 実行(タスクレベル)・ログ出力処理
 */
void
PutRelTskRes(void)
{
	syslog(LOG_INFO, "Call ReleaseResource(TskLevelRes)");

	error_log(ReleaseResource(TskLevelRes));
}

/*
 *  SetEvent 実行・ログ出力処理
 */
void
PutSetEvt(uint8 task_no)
{
	syslog(LOG_INFO, "Call SetEvent(%s, %s)",
		   task_name_tbl[task_no], event_name_tbl[task_no]);

	error_log(SetEvent(task_id_tbl[task_no], event_mask_tbl[task_no]));
}   /* PutSetEvt */

/*
 *  ClearEvent 実行・ログ出力処理
 */
void
PutClrEvt(uint8 task_no)
{
	syslog(LOG_INFO, "Call ClearEvent(%s)", event_name_tbl[task_no]);

	error_log(ClearEvent(event_mask_tbl[task_no]));
}   /* PutClrEvt */

/*
 *  GetEvent 実行・ログ出力処理
 */
void
PutGetEvt(uint8 task_no)
{
	EventMaskType mask;             /* イベントマスク取得バッファ */

	error_log(GetEvent(task_id_tbl[task_no], &mask));

	syslog(LOG_INFO, "%s Event Mask:0x%x", task_name_tbl[task_no], mask);
}   /* PutGetEvt */

/*
 *  WaitEvent 実行・ログ出力処理
 */
void
PutWaitEvt(uint8 task_no)
{
	syslog(LOG_INFO, "Call WaitEvent(%s)", event_name_tbl[task_no]);

	error_log(WaitEvent(event_mask_tbl[task_no]));
	syslog(LOG_INFO, "Return WaitEvent(%s)", event_name_tbl[task_no]);
}   /* PutWaitEvt */

/*
 *  GetAlarmBase 実行・ログ出力処理
 */
void
PutArmBase(void)
{
	AlarmBaseType info;             /* アラームベース情報取得バッファ */

	error_log(GetAlarmBase(MainCycArm, &info));

	syslog(LOG_INFO, "MainCycArm Base:");
	syslog(LOG_INFO, "\tMAXALLOWEDVALUE=%d", info.maxallowedvalue);
	syslog(LOG_INFO, "\tTICKSPERBASE=%d", info.ticksperbase);
	syslog(LOG_INFO, "\tMINCYCLE=%d", info.mincycle);
}   /* PutArmBase */

/*
 *  PutArmTick 実行・ログ出力処理
 */
void
PutArmTick(void)
{
	TickType tick;              /* 残りティック取得バッファ */

	error_log(GetAlarm(MainCycArm, &tick));

	syslog(LOG_INFO, "MainCycArm Tick:%d", tick);
}   /* PutArmTick */

/*
 *  SetRelAlarm 実行・ログ出力処理
 */
void
PutSetRel(uint8 alarm_no, uint8 tick_no, uint8 cycle_no)
{
	syslog(LOG_INFO, "Call SetRelAlarm(%s, %d, %d)",
		   alarm_name_tbl[alarm_no], tick_tbl[tick_no], cycle_tbl[cycle_no]);

	error_log(SetRelAlarm(alarm_id_tbl[alarm_no],
						  tick_tbl[tick_no], cycle_tbl[cycle_no]));
}   /* PutSetRel	*/

/*
 *  CancelAlarm 実行・ログ出力処理
 */
void
PutCanArm(void)
{
	syslog(LOG_INFO, "Call CancelAlarm(ActTskArm)");

	error_log(CancelAlarm(ActTskArm));
}   /* PutCanArm */

/*
 *  GetActiveApplicationMode 実行・ログ出力処理
 */
void
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
 *  スケジュールテーブルテスト用メインループ
 *
 *  ユーザコマンドの受信と，コマンドごとの処理実行
 */
void
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

		WaitEvent(MainEvt);     /* 10msの作業時間待ち */
		ClearEvent(MainEvt);

		/*
		 *  入力コマンド取得
		 */
		syslog(LOG_INFO, "Input Command:");
		command = GetCommand();
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
TASK(Task10)
{
	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "Task10 ACTIVATE");
	WaitEvent(T10Evt);
	syslog(LOG_INFO, "Task10 FINISH");
	TerminateTask();
}   /* TASK( Task10 ) */


/*
 *  スケジュールテーブル確認用タスク7
 */
TASK(Task11)
{
	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "Task11 ACTIVATE");
	WaitEvent(T11Evt);
	syslog(LOG_INFO, "Task11 FINISH");
	TerminateTask();
}   /* TASK( Task11 ) */


/*
 *  スケジュールテーブル確認用タスク8
 */
TASK(Task12)
{
	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "Task12 ACTIVATE");
	WaitEvent(T12Evt);
	syslog(LOG_INFO, "Task12 FINISH");
	TerminateTask();
}   /* TASK( Task12 ) */

/*
 *  信頼関数動作確認用タスク11
 */
TASK(Task13)
{
	struct parameter_struct	local;
	sint32					pp = 10;

	local.name1 = (sint32 *) &pp;
	local.name2 = 2;

	CallTrustedFunction(tfnt1, &local); /* 10*2 + 2 = 22 */
	syslog(LOG_NOTICE, "Task13 tfnt1 ret = %d", local.return_value);

	TerminateTask();
}   /* TASK( Task13 ) */

/*
 *  信頼関数動作確認用タスク12
 */
TASK(Task14)
{
	struct parameter_struct	local;
	sint32					pp = 20;

	local.name1 = (sint32 *) &pp;
	local.name2 = 4;

	CallTrustedFunction(tfnt2, &local); /* 20*4 + 4 = 84 */
	syslog(LOG_NOTICE, "Task14 tfnt2 ret = %d", local.return_value);

}   /* TASK( Task14 ) */

/*
 *  非信頼OSアプリケーション間アクセス権確認用タスク
 */
TASK(Task8)
{
	TaskStateType state;
	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "Task8 ACTIVATE");
	if (E_OK == GetTaskState(Task1, &state)) {
		syslog(LOG_INFO, "GetTaskState(Task1, &state), state = %s", task_state_tbl[state]);
	}
	syslog(LOG_INFO, "Task8 FINISH");
	TerminateTask();
}   /* TASK( Task8 ) */

/*
 *  非信頼OSアプリケーション間アクセス権確認用タスク
 */
TASK(Task9)
{
	TaskStateType state;
	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "Task9 ACTIVATE");
	if (E_OK == GetTaskState(Task2, &state)) {
		syslog(LOG_INFO, "GetTaskState(Task2, &state), state = %s", task_state_tbl[state]);
	}
	syslog(LOG_INFO, "Task9 FINISH");
	TerminateTask();
}   /* TASK( Task8 ) */

/*
 *  IOC確認用タスク1
 */
TASK(IocTask1)
{
	syslog(LOG_INFO, "IocTask1 ACTIVATE");

	IocProk1();

	syslog(LOG_INFO, "IocTask1 TERMINATE");

	TerminateTask();
}   /* TASK( IocTask1 ) */

/*
 *  IOC確認用タスク2
 */
TASK(IocTask2)
{
	syslog(LOG_INFO, "IocTask2 ACTIVATE");

	IocProk1();

	syslog(LOG_INFO, "IocTask2 TERMINATE");

	TerminateTask();
}   /* TASK( IocTask2 ) */

/*
 *  IOC確認用タスク内部処理
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
	api = GetCommandNoWait2();
#ifndef OMIT_ECHO
	syslog(LOG_INFO, "%c", api);
#endif /* OMIT_ECHO */

	switch (api) {
	case '1':
		syslog(LOG_INFO, "Input send data:");
		data[0] = GetCommandNoWait2();
#ifndef OMIT_ECHO
		syslog(LOG_INFO, "%c", data[0]);
#endif /* OMIT_ECHO */
		syslog(LOG_INFO, "Call IocSend_IOC_QUE_0");
		error_log_ioc(IocSend_IOC_QUE_0(data[0]));
		break;
	case '2':
		syslog(LOG_INFO, "Input send data:");
		data[0] = GetCommandNoWait2();
#ifndef OMIT_ECHO
		syslog(LOG_INFO, "%c", data[0]);
#endif /* OMIT_ECHO */

		syslog(LOG_INFO, "Call IocSend_IOC_QUE_1");
		error_log_ioc(IocSend_IOC_QUE_1(data[0]));
		break;
	case '3':
		for (i = 0; i < 3U; i++) {
			syslog(LOG_INFO, "Input send data%d:", i);
			data[i] = GetCommandNoWait2();
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

/*
 *  コマンド受信処理(ウェイト無し)
 */
uint8
GetCommandNoWait2(void)
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
}   /* GetCommandNoWait2 */


/*
 *  ActivateTask(IOC用) 実行・ログ出力処理
 */
void
PutActIocTsk(void)
{
	uint8 command;          /* コマンド受信バッファ */

	syslog(LOG_INFO, "Select IocTask:");
	command = GetCommand();
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
		error_log(CallTrustedFunction(actioctask3, NULL));
		break;
	case '4':
		error_log(CallTrustedFunction(actioctask4, NULL));
		break;
	default:
		syslog(LOG_INFO, "undefined command");
		break;
	}
}   /* PutActIocTsk	*/

TRUSTEDFUNCTION(TRUSTED_actioctask3, FunctionIndex, FunctionParams)
{
	error_log(ActivateTask(IocTask3));

	return(E_OK);
}

TRUSTEDFUNCTION(TRUSTED_actioctask4, FunctionIndex, FunctionParams)
{
	error_log(ActivateTask(IocTask4));

	return(E_OK);
}
