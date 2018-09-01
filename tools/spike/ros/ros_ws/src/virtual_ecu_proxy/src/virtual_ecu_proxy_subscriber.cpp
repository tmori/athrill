// ros/ros.h　ROSに関する基本的なAPIのためのヘッダ
#include "ros/ros.h"
#include "virtual_can_bus/can.h"
#include "acomm_init.h"
#include "athrill_comm.h"
#include <stdio.h>

void sub_callback(const virtual_can_bus::can msg)
{
    printf("msg.c0=%u\n", msg.c0);
}

int main(int argc, char **argv)
{
    char *path = (char*)"/mnt/c/project/esm/athrill/tools/spike/ros/library/c/command/athrill_bus1.bin";
    acomm_bus_metadata_type *p;

    ros::init(argc, argv, "virtual_ecu_proxy_subscriber");

    p = ::acomm_open(path);


    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("bus1/CANID_RX100", 100, sub_callback);

    ros::spin();
    
    acomm_close(p);
    return 0;
}