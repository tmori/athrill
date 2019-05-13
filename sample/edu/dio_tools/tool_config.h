#ifndef _TOOL_CONFIG_H_
#define _TOOL_CONFIG_H_

#include "tool_types.h"

typedef enum {
    DIO_CONFIG_PARAM_INDEX_FPATH = 0,
    DIO_CONFIG_PARAM_INDEX_NAME,
    DIO_CONFIG_PARAM_INDEX_FOFF,
    DIO_CONFIG_PARAM_INDEX_BOFF,
    DIO_CONFIG_PARAM_INDEX_NUM,
} DioConfigParamIndexType;

typedef struct {
    const char* param[DIO_CONFIG_PARAM_INDEX_NUM];
    uint32      file_offset; /* byte */
    uint32      bit_offset;  /* 8bit size */
} DioConfigTye;

typedef struct {
    const char     *filepath;
    uint32         digital_num;
    DioConfigTye   *digital_configs;
} ToolConfigType;


extern ToolReturnType tool_config_load(const char *filepath, ToolConfigType *config);

#endif /* _TOOL_CONFIG_H_ */
