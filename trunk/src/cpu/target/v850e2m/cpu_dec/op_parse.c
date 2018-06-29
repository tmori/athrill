#include "cpu_dec/op_parse.h"
#include "cpu_dec/op_parse_private.h"
#include "cpu_dec/op_dec.h"

int op_parse(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code, OperationCodeType *optype)
{
	uint16 base_id;
	uint16 sub_id;
	uint32 value;
	int ret;

	value = (((uint32)code[1]) << 16U) | ((uint32)code[0]);

	if (((code[0] & BIT_MASK_10_5) == BIT_UP_10_5)
		&& ((value & BIT_MASK_16) == BIT_DOWN_16)) {
		/*
		 * Extend code
		 */
		base_id = GET_VALUE_BIT_26_23(value);
		sub_id  = GET_VALUE_BIT_22_21(value);
		ret = op_parse_extend_code_table[OP_PARSE_TABLE_INDEX(base_id, sub_id)].parse(code, optype);
	}
	else {
		/*
		 * Base code
		 */
		base_id = GET_VALUE_BIT_10_7(code[0]);
		sub_id  = GET_VALUE_BIT_6_5(code[0]);
		ret = op_parse_base_code_table[OP_PARSE_TABLE_INDEX(base_id, sub_id)].parse(code, optype);
	}
	/*
	 * decode
	 */
	if (ret == 0) {
		ret = OpDecoder[optype->format_id].decode(code, decoded_code);
	}
	return ret;
}
