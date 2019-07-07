#include "digital.h"
#include "device_io.h"

/*
 * 0.5 sec cycle
 */
void sample_program(void)
{
    uint8 data = sil_reb_mem(DIGITAL_REG_ADDR);

    if ((data & DIGITAL_LED1) != 0) {
        data &= ~DIGITAL_LED1;
        sil_wrb_mem(DIGITAL_REG_ADDR, data);
    }
    else {
        data |= DIGITAL_LED1;
        sil_wrb_mem(DIGITAL_REG_ADDR, data);
    }
    return;
}
