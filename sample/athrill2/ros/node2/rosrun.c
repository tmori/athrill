#include "rosrun.h"
#include "roslib.h"
#include "athrill_comm.h"

void rosrun(void)
{
    unsigned char can_data[8];

    roslib_subscribe(0, 0, can_data);
    return;
}
