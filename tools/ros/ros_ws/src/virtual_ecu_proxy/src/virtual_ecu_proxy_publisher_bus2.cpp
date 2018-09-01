// ros/ros.h　ROSに関する基本的なAPIのためのヘッダ
#include "ros/ros.h"
#include "virtual_can_bus/can.h"
#include "acomm_init.h"
#include "athrill_comm.h"
#include <stdio.h>

#define FILEPATH_MAX    4096
static char mmap_filepath[FILEPATH_MAX];


/*****************************
 * BUS: bus2
 *****************************/

/*****************************
 * ELM: CANID_0x200
 *****************************/
 static void bus2_TX_CANID_0x200_do_task(ros::Publisher &pub)
 {
     acomm_rtype ret;
     acomm_uint8 can_data[8];

     ret = athrill_comm_recv(1, 0, &can_data[0], 8U);
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

/*****************************
 * ELM: CANID_0x203
 *****************************/
 static void bus2_TX_CANID_0x203_do_task(ros::Publisher &pub)
 {
     acomm_rtype ret;
     acomm_uint8 can_data[8];

     ret = athrill_comm_recv(1, 3, &can_data[0], 8U);
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
    acomm_bus_metadata_type *p;
    memset(mmap_filepath, 0, FILEPATH_MAX);
    sprintf(mmap_filepath, "%s/%s_bus2.bin", std::getenv("GENERATED_MMAP_PATH"), std::getenv("GENERATED_MMAP_FILE_PREFIX"));

    ros::init(argc, argv, "virtual_ecu_proxy_publisher_bus2");

    p = acomm_open(mmap_filepath);
    if (p == NULL) {
        fprintf(stderr, "ERROR: acomm_open() error\n");
        return 1;
    }

    ros::NodeHandle n;


    ros::Publisher pub_bus2_TX_CANID_0x200 = n.advertise<virtual_can_bus::can>("bus2/TX_CANID_0x200", 1000);

    ros::Publisher pub_bus2_TX_CANID_0x203 = n.advertise<virtual_can_bus::can>("bus2/TX_CANID_0x203", 1000);


    ros::Rate loop_rate(1);
    while (ros::ok())
    {

        bus2_TX_CANID_0x200_do_task(pub_bus2_TX_CANID_0x200);

        bus2_TX_CANID_0x203_do_task(pub_bus2_TX_CANID_0x203);

        ros::spinOnce();
        loop_rate.sleep();
    }

    acomm_close(p);
    return 0;
}