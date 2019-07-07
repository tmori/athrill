#include "tool_config.h"
#include "tool_file.h"
#include "dio.h"
#include "stdlibs.h"
#include <stdio.h>
#include <unistd.h>


int main(int argc, const char* argv[])
{
    char *SW_STATE[] = {
        "OFF",
        "ON",
    };

    if (argc != 2) {
        printf("Usage: %s digital.config\n", argv[0]);
        return 1;
    }
    ToolReturnType err;
    ToolConfigType config;

    err = tool_config_load(argv[1], &config);
    if (err != 0) {
        printf("ERROR: tool_config_load() err=%d\n", err);
        return 1;
    }
    ToolFileType file;
    file.filepath = config.digital_configs[0].param[DIO_CONFIG_PARAM_INDEX_FPATH];

    err = open_file(&file);
    if (err != 0) {
        printf("ERROR: open_file() err=%d\n", err);
        return 1;
    }

    /* display */
#define START_LINE  4U
#define START_CLMN  4U
    while (1) {
        int i;
        clr();
        location(START_LINE, START_LINE);
        printf("**** LED STATE ****");
        for (i = 0; i < config.digital_num; i++) {
            DioValueType value;
            dio_get_bit(&file, &config.digital_configs[i], &value);
            location(START_LINE + i + 1U, START_CLMN + 5U);
            printf("%s : %4s\n", config.digital_configs[i].param[DIO_CONFIG_PARAM_INDEX_NAME],
                SW_STATE[value]);
            fflush(stdout);
        }
        usleep(2000);
    }

    close_file(&file);
    return 0;
}
