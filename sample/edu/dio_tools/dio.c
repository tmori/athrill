#include "dio.h"
#include <stdlib.h>

ToolReturnType dio_set_bit(ToolFileType *fp, DioConfigTye *config, DioValueType value)
{
    if (fp->mmap == NULL) {
        return EINVAL;
    }
    if (config->file_offset >= fp->filesize) {
        return ERANGE;
    }

	if (value == DIO_VALUE_ON) {
		fp->mmap[config->file_offset] |= (1U << config->bit_offset);
	}
	else {
		fp->mmap[config->file_offset] &= ~(1U << config->bit_offset);
	}
    return 0;
}

ToolReturnType dio_get_bit(ToolFileType *fp, DioConfigTye *config, DioValueType *value)
{
    if (fp->mmap == NULL) {
        return EINVAL;
    }
    if (config->file_offset >= fp->filesize) {
        return ERANGE;
    }

	if ((fp->mmap[config->file_offset] & (1U << config->bit_offset)) != 0U) {
		*value = DIO_VALUE_ON;
	}
	else {
		*value = DIO_VALUE_OFF;
	}
    return 0;
}
