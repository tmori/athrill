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
#include<windows.h>
#include "winsock_wrapper/winsock_wrapper.h"


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
			Sleep(1000);
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

	if (argc == 1) {
		printf("Usage:%s [OPTION]... <load_file>\n", "athrill");
		printf(" %-30s : execute on the interaction mode. if -i is not set, execute on the background mode.\n", "-i");
		printf(" %-30s : execute on the remote mode. this option is valid on the interaction mode.\n", "-r");
		printf(" %-30s : set program end time using <timeout> clocks. this option is valid on the background mode.\n", "-t<timeout>");
		printf(" %-30s : set communication path with an another emulator.\n", "-p<fifo config file>");
		printf(" %-30s : set device parameter.\n", "-d<device config file>");
		return -11;
	}

	winsock_init();

	opt = parse_args(argc, argv);
	if (opt == NULL) {
		return 1;
	}
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

	if (opt->is_binary_data) {
		binary_load((uint8*)opt->load_file.buffer, 0U, opt->load_file.size);
	}
	else {
		elf_load((uint8*)opt->load_file.buffer);
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

		cpuemu_init(cpuemu_thread_run);
		do_cui();
	}
	else {
		cpuemu_init(NULL);
		cpuemu_set_cpu_end_clock(opt->timeout);
		(void)cpuemu_thread_run(NULL);
	}

	return 0;
}
