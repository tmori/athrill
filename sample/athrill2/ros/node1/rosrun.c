#include "rosrun.h"
#include "roslib.h"
#include "athrill_comm.h"

void rosrun(void)
{
    unsigned char can_data[8];
    int i;

    for (i = 0; i < 8; i++) {
        can_data[i] = i * 10;
    }
    roslib_publish(0, 0, can_data);
    return;
}
