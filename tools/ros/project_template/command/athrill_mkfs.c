#include "athrill_comm_image.h"
#include "athrill_comm_config.h"
#include <stdio.h>
#include <stdlib.h>


int main(int argc, const char* argv[])
{
    char *node_name;
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <node_name>\n", argv[0]);
        return 1;
    }
    node_name = (char*)argv[1];
    athrill_comm_make_image();
    athrill_comm_generate_image((const char*)node_name, ".");
    return 0;
}