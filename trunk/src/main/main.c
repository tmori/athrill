#include "front/parser/dbg_parser.h"
#include "loader/loader.h"
#include "option/option.h"
#include "cpu_control/dbg_cpu_control.h"
#include "cpu_control/dbg_cpu_thread_control.h"
#include "cpuemu_ops.h"
#include "cui/cui_ops.h"
#include "cui/stdio/cui_ops_stdio.h"
#include "cui/udp/cui_ops_udp.h"
#include "file_address_mapping.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include "target/target_os_api.h"
#include <sched.h>
#include <limits.h>
#ifdef OS_LINUX
#include <signal.h>
#endif

/*
 * version: X.Y.Z
 *  X: generation
 *  Y: function
 *  Z: bug fix, small changes
 */
#define ATHRILL_CORE_VERSION "1.0.0"

#ifndef ATHRILL_TARGET_ARCH
#define ATHRILL_TARGET_ARCH "UNKNOWN"
#define ATHRILL_TARGET_VERSION "0.0.0"
#endif


static void do_cui(void)
{
	DbgCmdExecutorType *res;
	bool is_dbgmode;
	char buffer[1024];
	int len;

	while (TRUE) {
		is_dbgmode = cpuctrl_is_debug_mode();
		printf("%s", (is_dbgmode == TRUE) ? "[DBG>" : "[CPU>");
		fflush(stdout);
retry:
		len = cui_getline(buffer, 1024);
		if (len < 0) {
			cui_close();
			target_os_api_sleep(1000);
			goto retry;
		}
		buffer[len] = '\0';
		res = dbg_parse((uint8*)buffer, (uint32)len);

		if (res != NULL) {
			res->run(res);
		}
	}
}

/*
 * コマンドオプション仕様
 * -i		インタラクションモード
 * 	・あり：インタラクションモード
 * 	・なし：バックグラウンド実行モード
 * -r(インタラクションモードのみ有効)
 * 	・あり：リモートモード
 * 	・なし：直接モード
 * -t<time>	終了時間(単位：clock)
 * 	・あり：終了時間
 * 	・なし：無制限
 * -b	入力ファイル形式
 * 	・あり：バイナリデータ
 * 	・なし：ELFファイル
 * -p<fifo config file path>
 * 	・あり：対抗ECUとの通信あり
 * 	・なし：シングルECU構成
 */
int main(int argc, const char *argv[])
{
	Std_ReturnType err;
	CmdOptionType *opt;
	MemoryAddressMapType memmap;
	memset(&memmap, 0, sizeof(MemoryAddressMapType));

	if (argc == 1) {
		printf("Athrill is licensed under the TOPPERS License Agreement (http://www.toppers.jp/en/license.html).\n");
		printf("ARCH:%s (VERSION CORE:%s TARGET:%s)\n\n", ATHRILL_TARGET_ARCH, ATHRILL_CORE_VERSION, ATHRILL_TARGET_VERSION);

		printf("Usage:%s -c<core num> -m <memory config file> [OPTION]... <load_file>\n", "athrill");
		printf(" %-30s : set core num. if -c is not set, core num = 2.\n", "-c");
		printf(" %-30s : execute on the interaction mode. if -i is not set, execute on the background mode.\n", "-i");
		printf(" %-30s : execute on the remote mode. this option is valid on the interaction mode.\n", "-r");
		printf(" %-30s : set program end time using <timeout> clocks. this option is valid on the background mode.\n", "-t<timeout>");
		printf(" %-30s : set athrill memory configuration. rom, ram region is configured on your system.\n", "-m<memory config file>");
		//printf(" %-30s : set communication path with an another emulator.\n", "-p<fifo config file>");
		printf(" %-30s : set device parameter.\n", "-d<device config file>");
		return -11;
	}

	winsock_init();
#if 0
	struct sched_param sp;

	sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
	int ret = sched_setscheduler(0, SCHED_FIFO, &sp);
	if (ret) {
		perror("sched_setscheduler");
		return 1;
	}
#endif
#ifdef OS_LINUX
	signal(SIGPIPE, SIG_IGN);
#endif

	opt = parse_args(argc, argv);
	if (opt == NULL) {
		return 1;
	}
	printf("core id num=%u\n", opt->core_id_num);
	if (opt->fifocfgpath != NULL) {
		err = cpuemu_set_comm_fifocfg(opt->fifocfgpath);
		if (err != STD_E_OK) {
			return -1;
		}
	}
	if (opt->devcfgpath != NULL) {
		err = cpuemu_load_devcfg(opt->devcfgpath);
		if (err != STD_E_OK) {
			return -1;
		}
	}
	if (opt->memfilepath != NULL) {
		err = cpuemu_load_memmap(opt->memfilepath, &memmap);
		if (err != STD_E_OK) {
			return -1;
		}
	}

	if (opt->is_binary_data) {
		binary_load((uint8*)opt->load_file.buffer, 0U, opt->load_file.size);
	}
	else {
		elf_load((uint8*)opt->load_file.buffer, &memmap);
		if (cpuemu_symbol_set() != STD_E_OK) {
			return -1;
		}
		file_address_mapping_init();
	}

	if (opt->is_interaction == TRUE) {
		if (opt->is_remote == TRUE) {
			cui_ops_udp_init();
		}
		else {
			cui_ops_stdio_init();
		}

		cpuemu_init(cpuemu_thread_run, opt);
		do_cui();
	}
	else {
		cpuemu_init(NULL, opt);
		cpuemu_set_cpu_end_clock(opt->timeout);
		(void)cpuemu_thread_run(NULL);
	}

	return 0;
}
