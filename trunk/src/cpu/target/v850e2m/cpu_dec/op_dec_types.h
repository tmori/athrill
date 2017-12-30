
#ifndef _OP_DEC_TYPES_H_
#define _OP_DEC_TYPES_H_

#include "../cpu_dec/op_format.h"

#define OP_DECODE_MAX	(2)

typedef struct {
	OpCodeFormatId type_id;
	OpCodeFormatType1 type1;
	OpCodeFormatType2 type2;
	OpCodeFormatType3 type3;
	OpCodeFormatType4_1 type4_1;
	OpCodeFormatType4_2 type4_2;
	OpCodeFormatType5 type5;
	OpCodeFormatType6 type6;
	OpCodeFormatType7 type7;
	OpCodeFormatType8 type8;
	OpCodeFormatType9 type9;
	OpCodeFormatType10 type10;
	OpCodeFormatType11 type11;
	OpCodeFormatType12 type12;
	OpCodeFormatType13 type13;
} OpDecodedCodeType;

#endif /* _OP_DEC_TYPES_H_ */
