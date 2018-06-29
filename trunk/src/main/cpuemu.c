#include "cpu.h"
#include "bus.h"
#include "cpuemu_ops.h"
#include "bus.h"
#include "std_device_ops.h"
#include "std_cpu_ops.h"
#include "dbg_log.h"
#include "cpu_control/dbg_cpu_control.h"
#include "cpu_control/dbg_cpu_thread_control.h"
#include "cpu_control/dbg_cpu_callback.h"
#include "elf_section.h"
#include "symbol_ops.h"
#include "token.h"
#include "file.h"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "target/target_os_api.h"
#include "option/option.h"
#include <fcntl.h>
#include <string.h>
#ifdef OS_LINUX
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#endif /* OS_LINUX */
#include "athrill_device.h"
#include "assert.h"

static DeviceClockType cpuemu_dev_clock;
static bool cpuemu_is_cui_mode = FALSE;
static uint64 cpuemu_cpu_end_clock = -1LLU;

Std_ReturnType cpuemu_symbol_set(void)
{
	uint32 i;
	uint32 num;
	Std_ReturnType err;
	DbgSymbolType sym;
	ElfSymbolType elfsym;


	err = elfsym_get_symbol_num(&num);
	if (err != STD_E_OK) {
		return err;
	}

	for (i = 0; i < num; i++) {
		err = elfsym_get_symbol(i, &elfsym);
		if (err != STD_E_OK) {
			return err;
		}
		sym.name = &elfsym.name[1];
		sym.addr = elfsym.addr;
		sym.size = elfsym.size;
		switch (elfsym.type) {
		case SYMBOL_TYPE_OBJECT:
			if (symbol_gl_add(&sym) < 0) {
				return STD_E_INVALID;
			}
			break;
		case SYMBOL_TYPE_FUNC:
			if (symbol_func_add(&sym) < 0) {
				return STD_E_INVALID;
			}
			break;
		default:
			break;
		}

	}

	return STD_E_OK;
}

bool cpuemu_cui_mode(void)
{
	return cpuemu_is_cui_mode;
}
int cpu_config_get_core_id_num(void)
{
	return (int)virtual_cpu.core_id_num;
}

void cpuemu_init(void *(*cpu_run)(void *), void *opt)
{
	CmdOptionType *copt = (CmdOptionType*)opt;
	CoreIdType i;
	dbg_log_init("./log.txt");
	if (copt->core_id_num > 0) {
		virtual_cpu.core_id_num = copt->core_id_num;
	}
	else {
		virtual_cpu.core_id_num = CPU_CONFIG_CORE_NUM;
	}
#ifdef OS_LINUX
	memset(&cpuemu_dev_clock.start_tv, 0, sizeof(struct timeval));
	memset(&cpuemu_dev_clock.elaps_tv, 0, sizeof(struct timeval));
#endif /* OS_LINUX */

	cpu_init();
	device_init(&virtual_cpu, &cpuemu_dev_clock);
	cputhr_control_init();
	cpuctrl_init();
	if (cpu_run != NULL) {
		cpuemu_is_cui_mode = TRUE;
		for (i = 0; i < cpu_config_get_core_id_num(); i++) {
			dbg_cpu_debug_mode_set(i, TRUE);
		}
		cputhr_control_start(cpu_run);
	}
	else {
		cpuemu_is_cui_mode = FALSE;
	}
#ifdef OS_LINUX
	device_init_athrill_device();
#endif /* OS_LINUX */
	return;
}

uint64 cpuemu_get_cpu_end_clock(void)
{
	return cpuemu_cpu_end_clock;
}
void cpuemu_set_cpu_end_clock(uint64 clock)
{
	cpuemu_cpu_end_clock = clock;
	return;
}

void cpuemu_get_elaps(CpuEmuElapsType *elaps)
{
	elaps->total_clocks = cpuemu_dev_clock.clock;
	elaps->intr_clocks = cpuemu_dev_clock.intclock;
#ifdef OS_LINUX
	elaps->elaps_tv = cpuemu_dev_clock.elaps_tv;
#endif /* OS_LINUX */

	return;
}

#ifdef OS_LINUX
void cpuemu_start_elaps(void)
{
	(void) gettimeofday(&cpuemu_dev_clock.start_tv, NULL);
	//printf("start sec=%ld usec=%ld\n", cpuemu_dev_clock.start_tv.tv_sec, cpuemu_dev_clock.start_tv.tv_usec);
	return;
}

void cpuemu_end_elaps(void)
{
	struct timeval current;
	struct timeval elaps_tv;

	if (cpuemu_dev_clock.start_tv.tv_sec == 0) {
		return;
	}

	(void) gettimeofday(&current, NULL);
	cpuemu_timeval_sub(&current, &cpuemu_dev_clock.start_tv, &elaps_tv);
	cpuemu_timeval_add(&elaps_tv, &cpuemu_dev_clock.elaps_tv, &cpuemu_dev_clock.elaps_tv);
	//printf("current sec=%ld usec=%ld\n", current.tv_sec, current.tv_usec);
	//printf("elaps sec=%ld usec=%ld\n", elaps_tv.tv_sec, elaps_tv.tv_usec);
	//printf("total sec=%ld usec=%ld\n", cpuemu_dev_clock.elaps_tv.tv_sec, cpuemu_dev_clock.elaps_tv.tv_usec);

	return;
}
#endif /* OS_LINUX */

uint32 cpuemu_get_retaddr(CoreIdType core_id)
{
	return cpu_get_return_addr((const TargetCoreType *)&virtual_cpu.cores[core_id]);
}

Std_ReturnType cpuemu_get_addr_pointer(uint32 addr, uint8 **data)
{
	return bus_get_pointer(CPU_CONFIG_CORE_ID_0, addr, data);
}


void cpuemu_get_register(CoreIdType core_id, TargetCoreType *cpu)
{
	*cpu = virtual_cpu.cores[core_id].core;
	return;
}

static TokenContainerType cpuemu_rom_parameter;

static void cpuemu_set_debug_romdata(void)
{
	uint32 param_num;
	uint32 i;
	Std_ReturnType err;
	char *rom_variable_name;
	static char parameter[4096];

	err = cpuemu_get_devcfg_value("DEBUG_ROM_DEFINE_NUM", &param_num);
	if (err != STD_E_OK) {
		return;
	}
	printf("rom_param_num=%d\n", param_num);
	for (i = 0; i < param_num; i++) {
		snprintf(parameter, sizeof(parameter), "DEBUG_ROM_DEFINE_%d", i);
		err = cpuemu_get_devcfg_string(parameter, &rom_variable_name);
		if (err != STD_E_OK) {
			printf("not found param=%s\n", parameter);
			return;
		}
		err = token_split_with_delimiter(&cpuemu_rom_parameter, (uint8*)rom_variable_name, strlen(rom_variable_name), '=');
		if (err != STD_E_OK) {
			printf("can not get param value=%s\n", rom_variable_name);
			return;
		}
		printf("ROM:redefine %s = %d\n",
				cpuemu_rom_parameter.array[0].body.str.str,
				cpuemu_rom_parameter.array[1].body.dec.value);
		{
			int ret;
			char *gl_name = (char*)cpuemu_rom_parameter.array[0].body.str.str;
			uint32 gl_len = strlen(gl_name);
			uint32 addr;
			uint32 size;
			uint8* datap;
			ret = symbol_get_gl(gl_name, gl_len, &addr, &size);
			if (ret < 0) {
				printf("can not found global variable=%s\n", gl_name);
				return;
			}
			err = bus_get_pointer(0U, addr, &datap);
			if (err != 0) {
				printf("can not found memory pointer=%s\n", gl_name);
				return;
			}
			switch (size) {
			case 1:
				*((uint8*)datap) = (uint8)cpuemu_rom_parameter.array[1].body.dec.value;
				break;
			case 2:
				*((uint16*)datap) = (uint16)cpuemu_rom_parameter.array[1].body.dec.value;
				break;
			case 4:
				*((uint32*)datap) = (uint32)cpuemu_rom_parameter.array[1].body.dec.value;
				break;
			default:
				printf("can not set pointer because gl_size(%s:%u) > 4\n", gl_name, size);
				break;
			}
		}
	}
	return;
}

void *cpuemu_thread_run(void* arg)
{
	CoreIdType i;
	Std_ReturnType err;
	DbgCpuCallbackFuncEnableType enable_dbg;
	bool is_halt;
	int core_id_num = cpu_config_get_core_id_num();

	enable_dbg.enable_bt = TRUE;
	enable_dbg.enable_ft = TRUE;
	enable_dbg.enable_watch = TRUE;
	enable_dbg.enable_prof = TRUE;
	cpuemu_dev_clock.enable_skip = FALSE;

	(void)cpuemu_get_devcfg_value("DEBUG_FUNC_ENABLE_BT", &enable_dbg.enable_bt);
	(void)cpuemu_get_devcfg_value("DEBUG_FUNC_ENABLE_FT", &enable_dbg.enable_ft);
	(void)cpuemu_get_devcfg_value("DEBUG_FUNC_ENABLE_PROF", &enable_dbg.enable_watch);
	(void)cpuemu_get_devcfg_value("DEBUG_FUNC_ENABLE_WATCH", &enable_dbg.enable_prof);

	(void)cpuemu_get_devcfg_value("DEBUG_FUNC_ENABLE_SKIP_CLOCK", (uint32*)&cpuemu_dev_clock.enable_skip);
	cpuemu_set_debug_romdata();

	while (TRUE) {
		if (cpuemu_dev_clock.clock >= cpuemu_get_cpu_end_clock()) {
			dbg_log_sync();
			//printf("EXIT for timeout(%I64u).\n", cpuemu_dev_clock.clock);
			printf("EXIT for timeout("PRINT_FMT_UINT64").\n", cpuemu_dev_clock.clock);
			exit(1);
		}

		/**
		 * デバイス実行実行
		 */
#ifdef OS_LINUX
		device_supply_clock_athrill_device();
#endif /* OS_LINUX */
		device_supply_clock(&cpuemu_dev_clock);

		/**
		 * CPU 実行
		 */
		is_halt = TRUE;
		for (i = 0; i < core_id_num; i++) {
			cpu_set_current_core(i);

			/*
			 * バスのアクセスログをクリアする
			 */
			bus_access_set_log(BUS_ACCESS_TYPE_NONE, 8U, 0, 0);

			/**
			 * CPU 実行開始通知
			 */
			dbg_notify_cpu_clock_supply_start(&virtual_cpu.cores[i].core);

			err = cpu_supply_clock(i);
			if (err != STD_E_OK) {
				printf("CPU(pc=0x%x) Exception!!\n", cpu_get_pc(&virtual_cpu.cores[i].core));
				fflush(stdout);
				if (cpuemu_cui_mode() == TRUE) {
					cpuctrl_set_force_break();
		#if 0
					cpu_illegal_opcode_trap(&CpuManager);
		#endif
				}
				else {
					exit(1);
				}
			}
			/**
			 * CPU 実行完了通知
			 */
			dbg_notify_cpu_clock_supply_end(&virtual_cpu.cores[i].core, &enable_dbg);

			if (cpu_is_halt(i) != TRUE) {
				is_halt = FALSE;
			}
		}
		if (cpuemu_dev_clock.enable_skip == TRUE) {
			if ((is_halt == TRUE) && (cpuemu_dev_clock.can_skip_clock == TRUE)) {
				cpuemu_dev_clock.clock += (cpuemu_dev_clock.min_intr_interval - 1);
			}
		}
	}

	return NULL;
}


typedef struct {
	TokenStringType	folder_path;
	TokenStringType rx_path;
	TokenStringType tx_path;
} CpuEmuFifoFileType;

static const TokenStringType fifo_tx_string = {
		.len = 2,
		.str = { 'T', 'X', '\0' },
};
static const TokenStringType fifo_rx_string = {
		.len = 2,
		.str = { 'R', 'X', '\0' },
};


static CpuEmuFifoFileType cpuemu_fifo_file;
static FileType cpuemu_fifocfg;

static Std_ReturnType parse_fifopath(char *buffer, uint32 len)
{
	Std_ReturnType err;
	TokenContainerType token_container;

	err = token_split(&token_container, (uint8*)buffer, len);
	if (err != STD_E_OK) {
		goto errdone;
	}
	err = STD_E_INVALID;
	if (token_container.num != 2) {
		goto errdone;
	}
	if (token_container.array[0].type != TOKEN_TYPE_STRING) {
		goto errdone;
	}
	else if (token_container.array[1].type != TOKEN_TYPE_STRING) {
		goto errdone;
	}

	if (token_strcmp(&fifo_tx_string, &token_container.array[0].body.str) == TRUE) {
		if (cpuemu_fifo_file.tx_path.len > 0) {
			printf("ERROR: INVALID parameter number of TX >= 2\n");
			return STD_E_INVALID;
		}
		cpuemu_fifo_file.tx_path = cpuemu_fifo_file.folder_path;
		if (token_merge(&cpuemu_fifo_file.tx_path, &token_container.array[1].body.str) == FALSE) {
			printf("ERROR: INVALID filename is too long: %s\n", token_container.array[1].body.str.str);
			return STD_E_INVALID;
		}
	}
	else if (token_strcmp(&fifo_rx_string, &token_container.array[0].body.str) == TRUE) {
		if (cpuemu_fifo_file.rx_path.len > 0) {
			printf("ERROR: INVALID parameter number of RX >= 2\n");
			return STD_E_INVALID;
		}
		cpuemu_fifo_file.rx_path = cpuemu_fifo_file.folder_path;
		if (token_merge(&cpuemu_fifo_file.rx_path, &token_container.array[1].body.str) == FALSE) {
			printf("ERROR: INVALID filename is too long: %s\n", token_container.array[1].body.str.str);
			return STD_E_INVALID;
		}
	}

	return STD_E_OK;
errdone:
	printf("ERROR: Invalid parameter. Format should b {TX|RX} <fifo name>\n");
	return err;
}

/*
 * 出力
 * ・fifoファイル配置フォルダパス
 * ・tx fifo ファイル配置パス
 * ・rx fifo ファイル配置パス
 */
Std_ReturnType cpuemu_set_comm_fifocfg(const char* fifocfg)
{
	uint32 len;
	Std_ReturnType err;
	char buffer[4096];
	bool ret;

	ret = token_string_set(&cpuemu_fifocfg.filepath, fifocfg);
	if (ret == FALSE) {
		return STD_E_INVALID;
	}

	ret = file_ropen(&cpuemu_fifocfg);
	if (ret == FALSE) {
		return STD_E_NOENT;
	}

	//fifoファイル配置フォルダパス
	len = file_get_parent_folder_pathlen(fifocfg);
	memcpy(cpuemu_fifo_file.folder_path.str, fifocfg, len);
	cpuemu_fifo_file.folder_path.str[len] = '\0';
	cpuemu_fifo_file.folder_path.len = len;

	err = STD_E_INVALID;
	//fifo ファイル配置パス取得(1回目)
	len = file_getline(&cpuemu_fifocfg, buffer, 4096);
	if (len > 0) {
		err = parse_fifopath(buffer, len);
		if (err != STD_E_OK) {
			goto errdone;
		}
	}
	else {
		printf("ERROR: can not found data on %s...\n", fifocfg);
		goto errdone;
	}

	//fifo ファイル配置パス取得(2回目)
	len = file_getline(&cpuemu_fifocfg, buffer, 4096);
	if (len > 0) {
		err = parse_fifopath(buffer, len);
		if (err != STD_E_OK) {
			goto errdone;
		}
	}
	else {
		printf("ERROR: can not found data on %s...\n", fifocfg);
		goto errdone;
	}

	err = STD_E_INVALID;
	if (file_exist(cpuemu_get_comm_rx_fifo()) == FALSE) {
		printf("ERROR: can not found fifo file %s...\n", cpuemu_get_comm_rx_fifo());
		goto errdone;
	}
	if (file_exist(cpuemu_get_comm_tx_fifo()) == FALSE) {
		printf("ERROR: can not found fifo file %s...\n", cpuemu_get_comm_rx_fifo());
		goto errdone;
	}

	printf("RX fifo:%s\n", cpuemu_get_comm_rx_fifo());
	printf("TX fifo:%s\n", cpuemu_get_comm_tx_fifo());

	file_close(&cpuemu_fifocfg);
	return STD_E_OK;
errdone:
	cpuemu_fifo_file.tx_path.len = 0;
	cpuemu_fifo_file.rx_path.len = 0;
	file_close(&cpuemu_fifocfg);
	return err;
}

const char* cpuemu_get_comm_rx_fifo(void)
{
	if (cpuemu_fifo_file.rx_path.len > 0) {
		return (const char*)cpuemu_fifo_file.rx_path.str;
	}
	else {
		return NULL;
	}
}

const char* cpuemu_get_comm_tx_fifo(void)
{
	if (cpuemu_fifo_file.tx_path.len > 0) {
		return (const char*)cpuemu_fifo_file.tx_path.str;
	}
	else {
		return NULL;
	}
}

#define CPUEMU_DEVCFG_PARAM_MAXNUM	128
typedef struct {
	uint32			param_num;
	struct {
		TokenValueType	key;
		TokenValueType	value;
	} param[CPUEMU_DEVCFG_PARAM_MAXNUM];
} CpuEmuDevCfgType;

static CpuEmuDevCfgType cpuemu_devcfg;
static char dvcfg_buffer[4096];
static TokenContainerType devcfg_token_container;
static FileType devcfg_file;

Std_ReturnType cpuemu_load_devcfg(const char *path)
{
	Std_ReturnType err = STD_E_OK;
	uint32 len;
	bool ret;

	cpuemu_devcfg.param_num = 0;

	ret = token_string_set(&devcfg_file.filepath, path);
	if (ret == FALSE) {
		return STD_E_INVALID;
	}
	ret = file_ropen(&devcfg_file);
	if (ret == FALSE) {
		return STD_E_NOENT;
	}
	while (TRUE) {
		err = STD_E_INVALID;

		len = file_getline(&devcfg_file, dvcfg_buffer, 4096);
		if (len <= 0) {
			break;
		}

		err = token_split(&devcfg_token_container, (uint8*)dvcfg_buffer, len);
		if (err != STD_E_OK) {
			printf("ERROR: can not parse data on %s...\n", path);
			goto errdone;
		}
		if (devcfg_token_container.num != 2) {
			printf("ERROR: the token is invalid %s on %s...\n", dvcfg_buffer, path);
			goto errdone;
		}
		cpuemu_devcfg.param[cpuemu_devcfg.param_num].key = devcfg_token_container.array[0];
		cpuemu_devcfg.param[cpuemu_devcfg.param_num].value = devcfg_token_container.array[1];
		cpuemu_devcfg.param_num++;
		//printf("param=%s\n", devcfg_token_container.array[0].body.str.str);
		//printf("value=%s\n", devcfg_token_container.array[1].body.str.str);
	}

	file_close(&devcfg_file);
	return STD_E_OK;
errdone:
	file_close(&devcfg_file);
	return err;
}

static FileType memcfg_file;
static char memcfg_buffer[4096];
static TokenContainerType memcfg_token_container;
Std_ReturnType cpuemu_load_memmap(const char *path, MemoryAddressMapType *map)
{
	Std_ReturnType err = STD_E_OK;
	uint32 len;
	bool ret;
	MemoryAddressType *memp;

	map->ram_num = 0;
	map->rom_num = 0;
	map->ram = NULL;
	map->rom = NULL;

	ret = token_string_set(&memcfg_file.filepath, path);
	if (ret == FALSE) {
		return STD_E_INVALID;
	}
	ret = file_ropen(&memcfg_file);
	if (ret == FALSE) {
		return STD_E_NOENT;
	}
	while (TRUE) {
		err = STD_E_INVALID;

		len = file_getline(&memcfg_file, memcfg_buffer, 4096);
		if (len <= 0) {
			break;
		}

		err = token_split(&memcfg_token_container, (uint8*)memcfg_buffer, len);
		if (err != STD_E_OK) {
			printf("ERROR: can not parse data on %s...\n", path);
			goto errdone;
		}
		if (memcfg_token_container.num != 3) {
			printf("ERROR: the token is invalid %s on %s...\n", memcfg_buffer, path);
			goto errdone;
		}
		if (!strcmp("ROM", (char*)memcfg_token_container.array[0].body.str.str)) {
			printf("ROM");
			map->rom_num++;
			map->rom = realloc(map->rom, map->rom_num * sizeof(MemoryAddressType));
			ASSERT(map->rom != NULL);
			memp = &map->rom[map->rom_num - 1];
			memp->type = MemoryAddressImplType_ROM;
			memp->size = memcfg_token_container.array[2].body.dec.value;
			memp->mmap_addr = NULL;
		}
		else if (!strcmp("RAM", (char*)memcfg_token_container.array[0].body.str.str)) {
			printf("RAM");
			map->ram_num++;
			map->ram = realloc(map->ram, map->ram_num * sizeof(MemoryAddressType));
			ASSERT(map->ram != NULL);
			memp = &map->ram[map->ram_num - 1];
			memp->type = MemoryAddressImplType_RAM;
			memp->size = memcfg_token_container.array[2].body.dec.value;
			memp->mmap_addr = NULL;
		}
#ifdef OS_LINUX
		else if (!strcmp("MMAP", (char*)memcfg_token_container.array[0].body.str.str)) {
			map->ram_num++;
			map->ram = realloc(map->ram, map->ram_num * sizeof(MemoryAddressType));
			ASSERT(map->ram != NULL);
			memp = &map->ram[map->ram_num - 1];
			memp->type = MemoryAddressImplType_MMAP;
			{
				char* filepath = (char*)memcfg_token_container.array[2].body.str.str;
				int fd;
				int err;
				struct stat statbuf;
				AthrillDeviceMmapInfoType info;
				fd = open(filepath, O_RDWR);
				ASSERT(fd >= 0);
				err = fstat(fd, &statbuf);
				ASSERT(err >= 0);
				memp->size = ((statbuf.st_size + 8191) / 8192) * 8;
				if (memp->size == 0) {
					memp->size = 8;
				}
				memp->mmap_addr = mmap(NULL, memp->size * 1024, (PROT_READ|PROT_WRITE), MAP_SHARED, fd, 0);
				ASSERT(memp->mmap_addr != NULL);
				printf("MMAP(%s filesize=%lu)", filepath, statbuf.st_size);
				info.fd = fd;
				info.addr = memcfg_token_container.array[1].body.hex.value;
				athrill_device_set_mmap_info(&info);
			}
		}
#endif /* OS_LINUX */
		else {
			printf("WARNING: unknown memory type=%s\n", (char*)memcfg_token_container.array[0].body.str.str);
			continue;
		}
		memp->start = memcfg_token_container.array[1].body.hex.value;
		printf(" : START=0x%x SIZE=%u\n", memp->start, memp->size);
	}

	file_close(&memcfg_file);
	return STD_E_OK;
errdone:
	file_close(&memcfg_file);
	return err;
}


Std_ReturnType cpuemu_get_devcfg_value(const char* key, uint32 *value)
{
	int i;
	TokenStringType token;

	token.len = strlen(key);
	memcpy(token.str, key, token.len);
	token.str[token.len] = '\0';

	for (i = 0; i < cpuemu_devcfg.param_num; i++) {
		if (cpuemu_devcfg.param[i].value.type != TOKEN_TYPE_VALUE_DEC) {
			continue;
		}
		if (token_strcmp(&cpuemu_devcfg.param[i].key.body.str, &token) == FALSE) {
			continue;
		}
		*value = cpuemu_devcfg.param[i].value.body.dec.value;
		return STD_E_OK;
	}
	return STD_E_NOENT;
}
Std_ReturnType cpuemu_get_devcfg_string(const char* key, char **value)
{
	int i;
	TokenStringType token;

	token.len = strlen(key);
	memcpy(token.str, key, token.len);
	token.str[token.len] = '\0';

	for (i = 0; i < cpuemu_devcfg.param_num; i++) {
		if (cpuemu_devcfg.param[i].value.type != TOKEN_TYPE_STRING) {
			continue;
		}
		if (token_strcmp(&cpuemu_devcfg.param[i].key.body.str, &token) == FALSE) {
			continue;
		}
		*value = (char*)cpuemu_devcfg.param[i].value.body.str.str;
		return STD_E_OK;
	}
	return STD_E_NOENT;
}

void cpuemu_raise_intr(uint32 intno)
{
	(void)intc_raise_intr(intno);
	return;
}

