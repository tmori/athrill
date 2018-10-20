#include "types.h"
#include "interrupt_table.h"
#include "interrupt.h"
#include "v850_ins.h"


static void (*interrupt_handler_table[INTERRUPT_TABLE_SIZE]) (void);

void register_interrupt_handler(unsigned int intrno, void (*handler) (void))
{
    if (intrno < INTERRUPT_TABLE_SIZE) {
        disable_int_all();
        x_enable_int(intrno);
        interrupt_handler_table[intrno] = handler;
        enable_int_all();
    }
    return;
}

void do_interrupt_handler(unsigned int intrno)
{
    if (intrno < INTERRUPT_TABLE_SIZE) {
        if (interrupt_handler_table[intrno] != NULL) {
            interrupt_handler_table[intrno]();
        }
    }
}
