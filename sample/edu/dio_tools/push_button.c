#include "tool_config.h"
#include "tool_file.h"
#include "dio.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>


int main(int argc, const char* argv[])
{
    if (argc != 3) {
        printf("Usage: %s digital.config <SWITCH-NAME>\n", argv[0]);
        return 1;
    }
    ToolReturnType err;
    ToolConfigType config;
    const char* switch_name;

    err = tool_config_load(argv[1], &config);
    if (err != 0) {
        printf("ERROR: tool_config_load() err=%d\n", err);
        return 1;
    }
    switch_name = argv[2];

    ToolFileType file;
    file.filepath = config.digital_configs[0].param[DIO_CONFIG_PARAM_INDEX_FPATH];

    err = open_file(&file);
    if (err != 0) {
        printf("ERROR: open_file() err=%d\n", err);
        return 1;
    }

    int len = strlen(switch_name);
    int i;
    for (i = 0; i < config.digital_num; i++) {
        int clen = strlen(config.digital_configs[i].param[DIO_CONFIG_PARAM_INDEX_NAME]);
        if (len != clen) {
            continue;
        }
        if (strcmp(switch_name, config.digital_configs[i].param[DIO_CONFIG_PARAM_INDEX_NAME]) != 0) {
            continue;
        }
        dio_set_bit(&file, &config.digital_configs[i], DIO_VALUE_ON);

        printf("SWITCH STATE UPDATED: ON\n");
        printf("Please push Enter key for OFF\n");

        (void)fgetc(stdin);
        dio_set_bit(&file, &config.digital_configs[i], DIO_VALUE_OFF);
        printf("SWITCH STATE UPDATED: OFF\n");
        break;
    }

    close_file(&file);
    return 0;
}