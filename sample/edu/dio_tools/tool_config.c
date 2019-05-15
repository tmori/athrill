#include "tool_config.h"
#include "tool_file.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#define MAX_COLUMN  4U

static int32 get_dio_config(char *linep, DioConfigTye *dio_configp)
{
    DioConfigParamIndexType column = 0;
    uint32 off = 0;
    uint32 head;

    for (column = 0; column < DIO_CONFIG_PARAM_INDEX_NUM; column++) {
        dio_configp->param[column] = NULL;
    }
    /*
     * <filename>,<name>,<fileoff>,<bitoff>
     */
    column = 0;
    head = off;
    while (1) {
        switch (linep[off]) {
        case '\0':
        case '\n':
            linep[off] = '\0';
            dio_configp->param[column] = &linep[head];
            column++;
            off++;
            goto done;
        case ',':
            linep[off] = '\0';
            dio_configp->param[column] = &linep[head];
            column++;
            head = off + 1;
            off = head;
            break;
        default:
            off++;
            break;
        }
    }
done:
    if (column < DIO_CONFIG_PARAM_INDEX_NUM) {
        return -ERANGE;
    }
    else {
        dio_configp->file_offset = strtol(dio_configp->param[DIO_CONFIG_PARAM_INDEX_FOFF], NULL, 10);
        dio_configp->bit_offset = strtol(dio_configp->param[DIO_CONFIG_PARAM_INDEX_BOFF], NULL, 10);
    }
    return off;
}

ToolReturnType tool_config_load(const char *filepath, ToolConfigType *config)
{
    ToolReturnType err;
    ToolFileType file;
    int32 off = 0;

    config->filepath = filepath;
    config->digital_num = 0;
    config->digital_configs = NULL;
    file.filepath = filepath;

    err = open_file(&file);
    if (err != 0) {
        return err;
    }
    char *memory = (char*)malloc(file.filesize);
    if (memory == NULL) {
        return ENOMEM;
    }
    memcpy(memory, file.mmap, file.filesize);

    while (off < file.filesize) {
        char *linep = &memory[off];

        config->digital_configs = realloc(config->digital_configs, sizeof(DioConfigTye) * (config->digital_num + 1));
        if (config->digital_configs == NULL) {
            fprintf(stderr, "ERROR: ENOMEM:lineno=%d %s\n", config->digital_num, linep);
            return ENOMEM;
        }
        err = get_dio_config(linep, &config->digital_configs[config->digital_num]);
        if (err < 0) {
            fprintf(stderr, "ERROR: Invalid argument:lineno=%d %s\n", config->digital_num, linep);
            return EINVAL;
        }
        config->digital_num++;
        off += err;
    }

    (void)close_file(&file);
    return 0;
}