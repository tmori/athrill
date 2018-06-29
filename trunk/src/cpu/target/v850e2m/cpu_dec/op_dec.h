#ifndef _OP_DEC_H_
#define _OP_DEC_H_

#include "cpu_dec/op_dec_types.h"


typedef struct {
	int (*decode) (uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
} OpDecoderType;

extern OpDecoderType OpDecoder[OP_CODE_FORMAT_NUM];

#endif /* _OP_DEC_H_ */
