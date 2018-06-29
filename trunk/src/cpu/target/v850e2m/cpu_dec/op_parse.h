#ifndef _OP_PARSE_H_
#define _OP_PARSE_H_

#include "cpu_dec/op_dec_types.h"

typedef struct {
	OpCodeFormatId	format_id;
	OpCodeId		code_id;
} OperationCodeType;

extern int op_parse(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code, OperationCodeType *optype);


#endif /* _OP_PARSE_H_ */
