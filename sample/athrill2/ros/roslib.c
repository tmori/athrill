#include "roslib.h"
#include "athrill_comm.h"
#include "test_serial.h"

void roslib_init(void)
{
    (void)athrill_comm_init();
    return;
}

void roslib_publish(int busid, int elmid, unsigned char *can_datap)
{
    acomm_rtype ret;
    
    ret = athrill_comm_send_force((acomm_busid)busid, (acomm_elmid)elmid, (acomm_uint8 *)can_datap, 8U);
    if (ret != ACOMM_E_OK) {
        printf("ERROR:roslib_publish()\n");
    }
    return;
}

int roslib_subscribe(int busid, int elmid, unsigned char *can_datap)
{
    acomm_rtype ret;
    ret = athrill_comm_recv((acomm_busid)busid, (acomm_elmid)elmid, (acomm_uint8 *)can_datap, 8U);
    if (ret != ACOMM_E_OK) {
        return -1;
    }
    return 0;
}
