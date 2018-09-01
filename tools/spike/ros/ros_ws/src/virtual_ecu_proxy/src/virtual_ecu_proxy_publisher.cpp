// ros/ros.h　ROSに関する基本的なAPIのためのヘッダ
#include "ros/ros.h"
#include "virtual_can_bus/can.h"
#include "acomm_init.h"
#include "athrill_comm.h"
#include <stdio.h>


static void do_task(ros::Publisher &pub)
{
    acomm_rtype ret;
    acomm_uint8 can_data[8];
    ret = athrill_comm_recv(0, 0, &can_data[0], 8U);
    if (ret != ACOMM_E_OK) {
        return;
    }

    virtual_can_bus::can msg;
    msg.c0 = can_data[0];
    msg.c1 = can_data[1];
    msg.c2 = can_data[2];
    msg.c3 = can_data[3];
    msg.c4 = can_data[4];
    msg.c5 = can_data[5];
    msg.c6 = can_data[6];
    msg.c7 = can_data[7];

    pub.publish(msg);
    return;
}

int main(int argc, char **argv)
{
    char *path = (char*)"/mnt/c/project/esm/athrill/tools/spike/ros/library/c/command/athrill_bus1.bin";
    acomm_bus_metadata_type *p;

    ros::init(argc, argv, "virtual_ecu_proxy_publisher");

    p = acomm_open(path);
    if (p == NULL) {
        fprintf(stderr, "ERROR: acomm_open() error\n");
        return 1;
    }

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<virtual_can_bus::can>("bus1/CANID_TX200", 1000);

    ros::Rate loop_rate(1);


    int count = 0;
    while (ros::ok())
    {
        do_task(pub);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    acomm_close(p);
    return 0;
}