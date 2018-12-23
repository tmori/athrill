#include "test_serial.h"
#include "interrupt_table.h"

static void timer_interrupt_handler(void)
{
	test_print("timer_interrupt_handler!!\n");
	return;
}

void timer_init(void)
{
    register_interrupt_handler(22, timer_interrupt_handler);
}

