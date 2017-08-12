#include "../cpu_dec/op_format.h"

#define FORMALT_LV_NUM	(16)
static OpCodeFormatId op_code2format_lv1[FORMALT_LV_NUM] = {
		OP_CODE_FORMAT_1, /* 0 */
		OP_CODE_FORMAT_1, /* 1 */
		OP_CODE_FORMAT_1, /* 2 */
		OP_CODE_FORMAT_1, /* 3 */
		OP_CODE_FORMAT_2, /* 4 */
		OP_CODE_FORMAT_2, /* 5 */
		OP_CODE_FORMAT_4, /* 6 */
		OP_CODE_FORMAT_4, /* 7 */
		OP_CODE_FORMAT_4, /* 8 */
		OP_CODE_FORMAT_4, /* 9 */
		OP_CODE_FORMAT_4, /* 10 */
		OP_CODE_FORMAT_3, /* 11 */
		OP_CODE_FORMAT_6, /* 12 */
		OP_CODE_FORMAT_6, /* 13 */
		OP_CODE_FORMAT_7, /* 14 */
		OP_CODE_FORMAT_UNKNOWN, /* 15 */
};
static OpCodeFormatId op_code2format_lv2[FORMALT_LV_NUM] = {
		OP_CODE_FORMAT_9, 		/* 0 */
		OP_CODE_FORMAT_9, 		/* 1 */
		OP_CODE_FORMAT_10, 		/* 2 */
		OP_CODE_FORMAT_UNKNOWN, /* 3 */
		OP_CODE_FORMAT_11, 		/* 4 */
		OP_CODE_FORMAT_11, 		/* 5 */
		OP_CODE_FORMAT_11, 		/* 6 */
		OP_CODE_FORMAT_UNKNOWN, /* 7 */
		OP_CODE_FORMAT_UNKNOWN, /* 8 */
		OP_CODE_FORMAT_UNKNOWN, /* 9 */
		OP_CODE_FORMAT_UNKNOWN, /* 10 */
		OP_CODE_FORMAT_UNKNOWN, /* 11 */
		OP_CODE_FORMAT_UNKNOWN, /* 12 */
		OP_CODE_FORMAT_UNKNOWN, /* 13 */
		OP_CODE_FORMAT_UNKNOWN, /* 14 */
		OP_CODE_FORMAT_UNKNOWN, /* 15 */
};

//static OpCodeFormatId op_code2format_lv2[FORMALT_LV_NUM];

OpCodeFormatId OpCode2FormatId(uint8 opcode, uint8 subcode)
{
	OpCodeFormatId ret;
	uint8 opcode4	= ( (opcode  >> 2) & 0x0F );
	uint8 subcode4	= ( (subcode >> 2) & 0x0F );

	//printf("opcode4 = 0x%x\n", opcode4);
	ret = op_code2format_lv1[opcode4];
	if (ret != OP_CODE_FORMAT_UNKNOWN) {
		return ret;
	}
	if ((opcode & 0x02) == 0) {
		return OP_CODE_FORMAT_5;
	}
	if ((opcode4 == 0xF) && ((opcode & 0x3) == 0x02)) {
		/*
		 * 1	1	1	1	1	0
		 */
		return OP_CODE_FORMAT_8;
	}
	if ((subcode4 == 4U)) {
		if (((subcode & 0x02) != 0x0)) {
			return OP_CODE_FORMAT_12;
		}
		else {
			return OP_CODE_FORMAT_11;
		}
	}
	return op_code2format_lv2[subcode4];
}
