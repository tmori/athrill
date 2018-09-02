#include "ros/ros.h"
#include "virtual_can_bus/can.h"
#include "acomm_init.h"
#include "athrill_comm.h"
#include <stdio.h>
#include <cstdlib>

#define FILEPATH_MAX    4096
static char mmap_filepath[FILEPATH_MAX];
static acomm_bus_metadata_type *map_busp;


/*****************************
 * BUS: bus2
 *****************************/

/*****************************
 * ELM: CANID_0x201
 *****************************/
 static void bus2_RX_CANID_0x201_sub_callback(const virtual_can_bus::can msg)
 {
     acomm_rtype ret;
     acomm_uint8 can_data[8];
 
     can_data[0] = msg.c0;
     can_data[1] = msg.c1;
     can_data[2] = msg.c2;
     can_data[3] = msg.c3;
     can_data[4] = msg.c4;
     can_data[5] = msg.c5;
     can_data[6] = msg.c6;
     can_data[7] = msg.c7;
 
     ret = athrill_comm_send_force(1, 1, &can_data[0], 8U);
     if (ret != ACOMM_E_OK) {
         fprintf(stderr, "ERROR: athrill_comm_send_force() err=%u\n", ret);
     }
 
     return;
 }



int main(int argc, char **argv)
{
    memset(mmap_filepath, 0, FILEPATH_MAX);
    sprintf(mmap_filepath, "%s/%s_bus2.bin", std::getenv("GENERATED_MMAP_PATH"), std::getenv("GENERATED_MMAP_FILE_PREFIX"));

    ros::init(argc, argv, "virtual_ecu_proxy_subscriber_bus2");

    map_busp = ::acomm_open(mmap_filepath);

    ros::NodeHandle n;


    ros::Subscriber sub_bus2_RX_CANID_0x201 = n.subscribe("bus2/RX_CANID_0x201", 100, bus2_RX_CANID_0x201_sub_callback);

    ros::spin();
    
    acomm_close(map_busp);
    return 0;
}