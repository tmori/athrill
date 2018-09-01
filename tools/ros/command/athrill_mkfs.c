#include "athrill_comm_image.h"
#include "athrill_comm_config.h"
#include <stdio.h>
#include <stdlib.h>


int main(int argc, const char* argv[])
{
    athrill_comm_make_image();
    athrill_comm_generate_image(".");
    return 0;
}