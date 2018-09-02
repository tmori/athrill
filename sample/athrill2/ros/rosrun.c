#include "rosrun.h"
#include "roslib.h"
#include "athrill_comm.h"

void rosrun(void)
{
    unsigned char can_data[8];
    if (roslib_subscribe(0, 1, can_data) >= 0) {
        roslib_publish(1, 0, can_data);
    }
    if (roslib_subscribe(1, 1, can_data) >= 0) {
        roslib_publish(0, 0, can_data);
    }
    return;
}
