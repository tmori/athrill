#include "serial.h"
#include "reg.h"
#include "section.h"
#include "interrupt.h"
#include <string.h>
#include "timer.h"
#include "v850_ins.h"
#include "digital.h"

unsigned char stack_data[STACK_SIZE] __attribute__ ((section(".bss_noclr")));

/*
 * up/down/start/stop SWITCH SPECIFICATION
 *
 * check interval: 100 msec
 *
 * -------------------+-------+-----------+-----------+
 *                    | RESET | JUDGE(1)  | SWITCH-ON |
 * -------------------+-------+-----------+-----------+
 * 0->1               | JUDGE |    -      |     -     |
 * -------------------+-------+-----------+-----------+
 * 1->0               |   -   | RESET     |     -     |
 * -------------------+-------+-----------+-----------+
 * 0->0               |   -   |    -      |     -     |
 * -------------------+-------+-----------+-----------+
 * 1->1               |   -   | stay      |     -     |
 * -------------------+-------+-----------+-----------+
 * 1 keeps 0.5sec     |   -   | SWITCH-ON |     -     |
 * -------------------+-------+-----------+-----------+
 * 0 keeps 0.5sec     |   -   | -         |     -     |
 * -------------------+-------+-----------+-----------+
 * RESET-EVENT        | RESET | RESET     | RESET     |
 * -------------------+-------+-----------+-----------+
 */
typedef enum {
	SwitchId_START = 0,
	SwitchId_STOP,
	SwitchId_UP1,
	SwitchId_DOWN1,
	SwitchId_UP2,
	SwitchId_DOWN2,
} SwitchIdTye;

typedef enum {
	SwitchState_RESET = 0,
	SwitchState_JUDGE,
	SwitchState_SWITCH_ON,
} SwitchStateType;
#define SWITCH_NUM	10
//static uint8 switch_prev_value[SWITCH_NUM];
//static uint8 switch_value[SWITCH_NUM];
static SwitchStateType switch_state[SWITCH_NUM];
static uint32 switch_counter[SWITCH_NUM];

static void switch_judge(uint32 switch_id, uint8 current_value)
{
	switch (switch_state[switch_id]) {
	case SwitchState_RESET:
		if (current_value > 0) {
			switch_state[switch_id] = SwitchState_JUDGE;
			switch_counter[switch_id]++;
		}
		break;
	case SwitchState_JUDGE:
		if (current_value > 0) {
			switch_counter[switch_id]++;
			if (switch_counter[switch_id] >= 5) {
				switch_state[switch_id] = SwitchState_SWITCH_ON;
			}
		}
		else {
			switch_counter[switch_id] = 0;
			switch_state[switch_id] = SwitchState_RESET;
		}
		break;
	case SwitchState_SWITCH_ON:
		switch_counter[switch_id] = 0;
		switch_state[switch_id] = SwitchState_RESET;
		break;
	default:
		break;
	}
	//switch_prev_value[switch_id] = current_value;
	return;
}


static uint32 sys_time = 0;
static uint32 count = 0;
typedef enum {
	HeatMethod_NONE = 0,
	HeatMethod_WARM,
	HeatMethod_THAW,
	HeatMethod_NUM,
} HeatMethodType;
const char *heat_method_string[HeatMethod_NUM] = {
	"NONE",
	"WARN",
	"THAW",
};

static HeatMethodType heat_method;
static bool heat_count_start;
static uint32 heat_count;
#define HEAT_LAST_COUNT_DEFAULT_VALUE	100
static uint32 heat_last_count = HEAT_LAST_COUNT_DEFAULT_VALUE;


static void do_update(void)
{
	if (heat_count_start == FALSE) {
		if (switch_state[SwitchId_START] == SwitchState_SWITCH_ON) {
			heat_count_start = TRUE;
			heat_count = 0;
		}
	}
	else if (switch_state[SwitchId_STOP] == SwitchState_SWITCH_ON) {
		heat_count_start = FALSE;
	}

	if (switch_state[SwitchId_UP2] == SwitchState_SWITCH_ON) {
		heat_method++;
		if (heat_method >= HeatMethod_NUM) {
			heat_method = HeatMethod_NONE;
		}
	}
	else if (switch_state[SwitchId_DOWN2] == SwitchState_SWITCH_ON) {
		if (heat_method <= HeatMethod_NONE) {
			heat_method = HeatMethod_THAW;
		}
		else {
			heat_method--;
		}
	}
	if (switch_state[SwitchId_UP1] == SwitchState_SWITCH_ON) {
		heat_last_count += 10;
	}
	else if (switch_state[SwitchId_DOWN1] == SwitchState_SWITCH_ON) {
		if (heat_last_count >= 0) {
			heat_last_count -= 10;
		}
	}
	return;
}

/*
 * 100msec timer
 */
static void timer_interrupt_handler(void)
{
	static HeatMethodType prev_heat_method = HeatMethod_NONE;
	static uint32 prev_heat_last_count = HEAT_LAST_COUNT_DEFAULT_VALUE;

	volatile unsigned char *p = DIGITAL_REG_ADDR;
	volatile uint8 data = *p;
	if ( (data & DIGITAL_SWITCH_START) != 0) {
		switch_judge(SwitchId_START, 1);
	}
	else {
		switch_judge(SwitchId_START, 0);
	}
	if ( (data & DIGITAL_SWITCH_STOP) != 0) {
		switch_judge(SwitchId_STOP, 1);
	}
	else {
		switch_judge(SwitchId_STOP, 0);
	}
	if ( (data & DIGITAL_SWITCH_UP1) != 0) {
		switch_judge(SwitchId_UP1, 1);
	}
	else {
		switch_judge(SwitchId_UP1, 0);
	}
	if ( (data & DIGITAL_SWITCH_DOWN1) != 0) {
		switch_judge(SwitchId_DOWN1, 1);
	}
	else {
		switch_judge(SwitchId_DOWN1, 0);
	}
	if ( (data & DIGITAL_SWITCH_UP2) != 0) {
		switch_judge(SwitchId_UP2, 1);
	}
	else {
		switch_judge(SwitchId_UP2, 0);
	}
	if ( (data & DIGITAL_SWITCH_DOWN2) != 0) {
		switch_judge(SwitchId_DOWN2, 1);
	}
	else {
		switch_judge(SwitchId_DOWN2, 0);
	}
	do_update();
	if (prev_heat_method != heat_method) {
		athrill_fputs("heat_method");
		athrill_fputs(heat_method_string[heat_method]);
		prev_heat_method = heat_method;
	}
	if (prev_heat_last_count != heat_last_count) {
		athrill_fputs("heat_time");
		athrill_fputi(heat_last_count);
		prev_heat_last_count = heat_last_count;
	}

	count++;
	if (count > 10) {
		athrill_fputs("sys_time");
		athrill_fputi(sys_time++);

		if (heat_count_start == TRUE) {
			heat_count++;
			if (heat_count < heat_last_count) {
				athrill_fputs("last_time");
				athrill_fputi(heat_last_count - heat_count);
			}
			else {
				athrill_fputs("last_time");
				athrill_fputs("DONE!!");
			}
		}
		count = 0;
	}
	return;
}

int main(void)
{
	athrill_fputs("heat_method");
	athrill_fputs(heat_method_string[heat_method]);
	athrill_fputs("heat_time");
	athrill_fputi(heat_last_count);
	athrill_fputs("last_time");
	athrill_fputi(heat_last_count - heat_count);

	timer_init(timer_interrupt_handler);
	timer_start(10000);

    while (TRUE) {
        do_idle();
    }
    return 0;
}


void bss_clear(void)
{
	unsigned char *p = &_bss_start;
	unsigned char *e = &_bss_end;
	for (;p < e; p++) {
		*p = 0;
	}
	return;
}

void data_init(void)
{
	unsigned char *p_rom = &_idata_start;
	unsigned char *e_rom = &_idata_end;
	unsigned char *p_ram = &_data_start;

	for (;p_rom < e_rom; p_ram++, p_rom++) {
		*p_ram = *p_rom;
	}
}
