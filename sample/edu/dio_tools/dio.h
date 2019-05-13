#ifndef _DIO_H_
#define _DIO_H_

#include "tool_file.h"
#include "tool_config.h"
#include <errno.h>

typedef enum {
    DIO_VALUE_OFF = 0,
    DIO_VALUE_ON,
} DioValueType;

extern ToolReturnType dio_set_bit(ToolFileType *fp, DioConfigTye *config, DioValueType value);
extern ToolReturnType dio_get_bit(ToolFileType *fp, DioConfigTye *config, DioValueType *value);

#endif /* _DIO_H_ */