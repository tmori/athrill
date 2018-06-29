#include "cpu_dec/op_parse.h"
#include "cpu_dec/op_parse_private.h"

/*
 * base
 */
static int op_parse_base_code_0000_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0000_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0000_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0000_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0001_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0001_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0001_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0001_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0010_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0010_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0010_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0010_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0011_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0011_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0011_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0011_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0100_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0100_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0100_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0100_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0101_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0101_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0101_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0101_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0110_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0110_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0110_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0110_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0111_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0111_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0111_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_0111_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1000_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1000_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1000_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1000_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1001_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1001_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1001_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1001_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1010_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1010_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1010_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1010_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1011_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1011_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1011_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1011_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1100_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1100_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1100_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1100_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1101_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1101_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1101_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1101_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1110_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1110_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1110_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1110_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1111_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1111_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1111_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_base_code_1111_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);

/*
 * extend
 */
static int op_parse_extend_code_0000_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0000_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0000_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0000_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0001_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0001_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0001_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0001_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0010_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0010_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0010_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0010_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0011_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0011_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0011_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0011_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0100_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0100_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0100_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0100_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0101_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0101_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0101_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0101_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0110_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0110_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0110_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0110_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0111_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0111_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0111_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_0111_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1000_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1000_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1000_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1000_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1001_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1001_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1001_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1001_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1010_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1010_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1010_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1010_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1011_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1011_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1011_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1011_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1100_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1100_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1100_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1100_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1101_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1101_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1101_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1101_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1110_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1110_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1110_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1110_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1111_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1111_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1111_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
static int op_parse_extend_code_1111_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype);

/*
 * base table
 */
OpParserType op_parse_base_code_table[OP_PARSE_CODE_TABLE_NUM] = {
	{ op_parse_base_code_0000_00 },
	{ op_parse_base_code_0000_01 },
	{ op_parse_base_code_0000_10 },
	{ op_parse_base_code_0000_11 },
	{ op_parse_base_code_0001_00 },
	{ op_parse_base_code_0001_01 },
	{ op_parse_base_code_0001_10 },
	{ op_parse_base_code_0001_11 },
	{ op_parse_base_code_0010_00 },
	{ op_parse_base_code_0010_01 },
	{ op_parse_base_code_0010_10 },
	{ op_parse_base_code_0010_11 },
	{ op_parse_base_code_0011_00 },
	{ op_parse_base_code_0011_01 },
	{ op_parse_base_code_0011_10 },
	{ op_parse_base_code_0011_11 },
	{ op_parse_base_code_0100_00 },
	{ op_parse_base_code_0100_01 },
	{ op_parse_base_code_0100_10 },
	{ op_parse_base_code_0100_11 },
	{ op_parse_base_code_0101_00 },
	{ op_parse_base_code_0101_01 },
	{ op_parse_base_code_0101_10 },
	{ op_parse_base_code_0101_11 },
	{ op_parse_base_code_0110_00 },
	{ op_parse_base_code_0110_01 },
	{ op_parse_base_code_0110_10 },
	{ op_parse_base_code_0110_11 },
	{ op_parse_base_code_0111_00 },
	{ op_parse_base_code_0111_01 },
	{ op_parse_base_code_0111_10 },
	{ op_parse_base_code_0111_11 },
	{ op_parse_base_code_1000_00 },
	{ op_parse_base_code_1000_01 },
	{ op_parse_base_code_1000_10 },
	{ op_parse_base_code_1000_11 },
	{ op_parse_base_code_1001_00 },
	{ op_parse_base_code_1001_01 },
	{ op_parse_base_code_1001_10 },
	{ op_parse_base_code_1001_11 },
	{ op_parse_base_code_1010_00 },
	{ op_parse_base_code_1010_01 },
	{ op_parse_base_code_1010_10 },
	{ op_parse_base_code_1010_11 },
	{ op_parse_base_code_1011_00 },
	{ op_parse_base_code_1011_01 },
	{ op_parse_base_code_1011_10 },
	{ op_parse_base_code_1011_11 },
	{ op_parse_base_code_1100_00 },
	{ op_parse_base_code_1100_01 },
	{ op_parse_base_code_1100_10 },
	{ op_parse_base_code_1100_11 },
	{ op_parse_base_code_1101_00 },
	{ op_parse_base_code_1101_01 },
	{ op_parse_base_code_1101_10 },
	{ op_parse_base_code_1101_11 },
	{ op_parse_base_code_1110_00 },
	{ op_parse_base_code_1110_01 },
	{ op_parse_base_code_1110_10 },
	{ op_parse_base_code_1110_11 },
	{ op_parse_base_code_1111_00 },
	{ op_parse_base_code_1111_01 },
	{ op_parse_base_code_1111_10 },
	{ op_parse_base_code_1111_11 },
};

/*
 * extend table
 */
OpParserType op_parse_extend_code_table[OP_PARSE_CODE_TABLE_NUM] = {
	{ op_parse_extend_code_0000_00 },
	{ op_parse_extend_code_0000_01 },
	{ op_parse_extend_code_0000_10 },
	{ op_parse_extend_code_0000_11 },
	{ op_parse_extend_code_0001_00 },
	{ op_parse_extend_code_0001_01 },
	{ op_parse_extend_code_0001_10 },
	{ op_parse_extend_code_0001_11 },
	{ op_parse_extend_code_0010_00 },
	{ op_parse_extend_code_0010_01 },
	{ op_parse_extend_code_0010_10 },
	{ op_parse_extend_code_0010_11 },
	{ op_parse_extend_code_0011_00 },
	{ op_parse_extend_code_0011_01 },
	{ op_parse_extend_code_0011_10 },
	{ op_parse_extend_code_0011_11 },
	{ op_parse_extend_code_0100_00 },
	{ op_parse_extend_code_0100_01 },
	{ op_parse_extend_code_0100_10 },
	{ op_parse_extend_code_0100_11 },
	{ op_parse_extend_code_0101_00 },
	{ op_parse_extend_code_0101_01 },
	{ op_parse_extend_code_0101_10 },
	{ op_parse_extend_code_0101_11 },
	{ op_parse_extend_code_0110_00 },
	{ op_parse_extend_code_0110_01 },
	{ op_parse_extend_code_0110_10 },
	{ op_parse_extend_code_0110_11 },
	{ op_parse_extend_code_0111_00 },
	{ op_parse_extend_code_0111_01 },
	{ op_parse_extend_code_0111_10 },
	{ op_parse_extend_code_0111_11 },
	{ op_parse_extend_code_1000_00 },
	{ op_parse_extend_code_1000_01 },
	{ op_parse_extend_code_1000_10 },
	{ op_parse_extend_code_1000_11 },
	{ op_parse_extend_code_1001_00 },
	{ op_parse_extend_code_1001_01 },
	{ op_parse_extend_code_1001_10 },
	{ op_parse_extend_code_1001_11 },
	{ op_parse_extend_code_1010_00 },
	{ op_parse_extend_code_1010_01 },
	{ op_parse_extend_code_1010_10 },
	{ op_parse_extend_code_1010_11 },
	{ op_parse_extend_code_1011_00 },
	{ op_parse_extend_code_1011_01 },
	{ op_parse_extend_code_1011_10 },
	{ op_parse_extend_code_1011_11 },
	{ op_parse_extend_code_1100_00 },
	{ op_parse_extend_code_1100_01 },
	{ op_parse_extend_code_1100_10 },
	{ op_parse_extend_code_1100_11 },
	{ op_parse_extend_code_1101_00 },
	{ op_parse_extend_code_1101_01 },
	{ op_parse_extend_code_1101_10 },
	{ op_parse_extend_code_1101_11 },
	{ op_parse_extend_code_1110_00 },
	{ op_parse_extend_code_1110_01 },
	{ op_parse_extend_code_1110_10 },
	{ op_parse_extend_code_1110_11 },
	{ op_parse_extend_code_1111_00 },
	{ op_parse_extend_code_1111_01 },
	{ op_parse_extend_code_1111_10 },
	{ op_parse_extend_code_1111_11 },
};

/*
 * Base
 */
static int op_parse_base_code_0000_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg1 = GET_REG1(code[0]);
	uint16 reg2 = GET_REG2(code[0]);

	if (( reg1 == 0 ) && ( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_NOP_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg1 == 31 ) && ( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_SYNCP_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg1 == 30 ) && ( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_SYNCM_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg1 == 29 ) && ( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_SYNCE_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( ( reg1 >= 1 ) && ( reg1 <= 27 ) ) && ( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_MOV_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg1 == 0 ) && ( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_MOV_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg1 !=  0 ) && ( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_MOV_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0000_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_NOT_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0000_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg1 = GET_REG1(code[0]);
	uint16 reg2 = GET_REG2(code[0]);

	if (( reg1 == 0 ) && ( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_RIE_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg1 !=  0 ) && ( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_SWITCH_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg1 == 0 ) && ( ( reg2 >= 1 ) && ( reg2 <= 15 ) )) 
	{
		optype->code_id = OpCodeId_FETRAP_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg1 !=  0 ) && ( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_DIVH_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0000_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );

	if (( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_JMP_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg2 !=  0 ) && ( value & ( (1U << 4U) ) ) == ( (0U << 4U) )) 
	{
		optype->code_id = OpCodeId_SLD_BU_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	if (( reg2 !=  0 ) && ( value & ( (1U << 4U) ) ) == ( (1U << 4U) )) 
	{
		optype->code_id = OpCodeId_SLD_HU_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0001_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);

	if (( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_SATSUBR_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_ZXB_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0001_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);

	if (( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_SATSUB_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_SXB_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0001_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);

	if (( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_SATADD_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_ZXH_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0001_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);

	if (( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_MULH_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	if (( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_SXH_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0010_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_OR_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0010_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_XOR_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0010_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_AND_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0010_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_TST_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0011_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SUBR_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0011_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SUB_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0011_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_ADD_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0011_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_CMP_1;
		optype->format_id = OP_CODE_FORMAT_1;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0100_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);

	if (( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_MOV_2;
		optype->format_id = OP_CODE_FORMAT_2;
		return 0;
	}
	if (( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_CALLT_2;
		optype->format_id = OP_CODE_FORMAT_2;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0100_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);

	if (( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_SATADD_2;
		optype->format_id = OP_CODE_FORMAT_2;
		return 0;
	}
	if (( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_CALLT_2;
		optype->format_id = OP_CODE_FORMAT_2;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0100_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_ADD_2;
		optype->format_id = OP_CODE_FORMAT_2;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0100_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_CMP_2;
		optype->format_id = OP_CODE_FORMAT_2;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0101_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SHR_2;
		optype->format_id = OP_CODE_FORMAT_2;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0101_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SAR_2;
		optype->format_id = OP_CODE_FORMAT_2;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0101_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SHL_2;
		optype->format_id = OP_CODE_FORMAT_2;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0101_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg1 = GET_REG1(code[0]);
	uint16 reg2 = GET_REG2(code[0]);

	if (( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_MULH_2;
		optype->format_id = OP_CODE_FORMAT_2;
		return 0;
	}
	if (( reg1 == 0 ) && ( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_JR_6;
		optype->format_id = OP_CODE_FORMAT_6;
		return 0;
	}
	if (( reg1 !=  0 ) && ( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_JARL_6;
		optype->format_id = OP_CODE_FORMAT_6;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0110_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SLD_B_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0110_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SLD_B_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0110_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SLD_B_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0110_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SLD_B_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0111_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SST_B_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0111_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SST_B_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0111_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SST_B_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_0111_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SST_B_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1000_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SLD_H_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1000_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SLD_H_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1000_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SLD_H_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1000_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SLD_H_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1001_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SST_H_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1001_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SST_H_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1001_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SST_H_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1001_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_SST_H_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1010_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );

	if (( value & ( (1U << 0U) ) ) == ( (0U << 0U) )) 
	{
		optype->code_id = OpCodeId_SLD_W_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	if (( value & ( (1U << 0U) ) ) == ( (1U << 0U) )) 
	{
		optype->code_id = OpCodeId_SST_W_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1010_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );

	if (( value & ( (1U << 0U) ) ) == ( (0U << 0U) )) 
	{
		optype->code_id = OpCodeId_SLD_W_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	if (( value & ( (1U << 0U) ) ) == ( (1U << 0U) )) 
	{
		optype->code_id = OpCodeId_SST_W_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1010_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );

	if (( value & ( (1U << 0U) ) ) == ( (0U << 0U) )) 
	{
		optype->code_id = OpCodeId_SLD_W_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	if (( value & ( (1U << 0U) ) ) == ( (1U << 0U) )) 
	{
		optype->code_id = OpCodeId_SST_W_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1010_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );

	if (( value & ( (1U << 0U) ) ) == ( (0U << 0U) )) 
	{
		optype->code_id = OpCodeId_SLD_W_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	if (( value & ( (1U << 0U) ) ) == ( (1U << 0U) )) 
	{
		optype->code_id = OpCodeId_SST_W_4;
		optype->format_id = OP_CODE_FORMAT_4;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1011_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_Bcond_3;
		optype->format_id = OP_CODE_FORMAT_3;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1011_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_Bcond_3;
		optype->format_id = OP_CODE_FORMAT_3;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1011_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_Bcond_3;
		optype->format_id = OP_CODE_FORMAT_3;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1011_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_Bcond_3;
		optype->format_id = OP_CODE_FORMAT_3;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1100_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_ADDI_6;
		optype->format_id = OP_CODE_FORMAT_6;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1100_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);

	if (( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_MOVEA_6;
		optype->format_id = OP_CODE_FORMAT_6;
		return 0;
	}
	if (( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_MOV_6;
		optype->format_id = OP_CODE_FORMAT_6;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1100_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);

	if (( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_MOVHI_6;
		optype->format_id = OP_CODE_FORMAT_6;
		return 0;
	}
	if (( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_DISPOSE_13;
		optype->format_id = OP_CODE_FORMAT_13;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1100_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);

	if (( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_SATSUBI_6;
		optype->format_id = OP_CODE_FORMAT_6;
		return 0;
	}
	if (( reg2 == 0 )) 
	{
		optype->code_id = OpCodeId_DISPOSE_13;
		optype->format_id = OP_CODE_FORMAT_13;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1101_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_ORI_6;
		optype->format_id = OP_CODE_FORMAT_6;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1101_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_XORI_6;
		optype->format_id = OP_CODE_FORMAT_6;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1101_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_ANDI_6;
		optype->format_id = OP_CODE_FORMAT_6;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1101_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );

	if (( reg2 !=  0 )) 
	{
		optype->code_id = OpCodeId_MULHI_6;
		optype->format_id = OP_CODE_FORMAT_6;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 16U) ) ) == ( (0U << 16U) )) 
	{
		optype->code_id = OpCodeId_JMP_6;
		optype->format_id = OP_CODE_FORMAT_6;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1110_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_LD_B_7;
		optype->format_id = OP_CODE_FORMAT_7;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1110_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );

	if (( value & ( (1U << 16U) ) ) == ( (0U << 16U) )) 
	{
		optype->code_id = OpCodeId_LD_H_7;
		optype->format_id = OP_CODE_FORMAT_7;
		return 0;
	}
	if (( value & ( (1U << 16U) ) ) == ( (1U << 16U) )) 
	{
		optype->code_id = OpCodeId_LD_W_7;
		optype->format_id = OP_CODE_FORMAT_7;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1110_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{

	if (TRUE) 
	{
		optype->code_id = OpCodeId_ST_B_7;
		optype->format_id = OP_CODE_FORMAT_7;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1110_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );

	if (( value & ( (1U << 16U) ) ) == ( (0U << 16U) )) 
	{
		optype->code_id = OpCodeId_ST_H_7;
		optype->format_id = OP_CODE_FORMAT_7;
		return 0;
	}
	if (( value & ( (1U << 16U) ) ) == ( (1U << 16U) )) 
	{
		optype->code_id = OpCodeId_ST_W_7;
		optype->format_id = OP_CODE_FORMAT_7;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1111_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );

	if (( reg2 == 0 ) && ( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) | (1U << 5U) ) ) == ( (0U << 19U) | (1U << 18U) | (0U << 17U) | (1U << 16U) | (0U << 5U) )) 
	{
		optype->code_id = OpCodeId_LD_B_14;
		optype->format_id = OP_CODE_FORMAT_14;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) | (1U << 5U) ) ) == ( (0U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) | (0U << 5U) )) 
	{
		optype->code_id = OpCodeId_LD_H_14;
		optype->format_id = OP_CODE_FORMAT_14;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) | (1U << 5U) ) ) == ( (1U << 19U) | (0U << 18U) | (0U << 17U) | (1U << 16U) | (0U << 5U) )) 
	{
		optype->code_id = OpCodeId_LD_W_14;
		optype->format_id = OP_CODE_FORMAT_14;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) | (1U << 5U) ) ) == ( (1U << 19U) | (1U << 18U) | (0U << 17U) | (1U << 16U) | (0U << 5U) )) 
	{
		optype->code_id = OpCodeId_ST_B_14;
		optype->format_id = OP_CODE_FORMAT_14;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) | (1U << 5U) ) ) == ( (1U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) | (0U << 5U) )) 
	{
		optype->code_id = OpCodeId_ST_W_14;
		optype->format_id = OP_CODE_FORMAT_14;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 16U) ) ) == ( (0U << 16U) )) 
	{
		optype->code_id = OpCodeId_JR_5;
		optype->format_id = OP_CODE_FORMAT_5;
		return 0;
	}
	if (( reg2 !=  0 ) && ( value & ( (1U << 16U) ) ) == ( (0U << 16U) )) 
	{
		optype->code_id = OpCodeId_JARL_5;
		optype->format_id = OP_CODE_FORMAT_5;
		return 0;
	}
	if (( reg2 !=  0 ) && ( value & ( (1U << 16U) ) ) == ( (1U << 16U) )) 
	{
		optype->code_id = OpCodeId_LD_BU_7;
		optype->format_id = OP_CODE_FORMAT_7;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 18U) | (1U << 17U) | (1U << 16U) ) ) == ( (0U << 18U) | (1U << 17U) | (1U << 16U) )) 
	{
		optype->code_id = OpCodeId_PREPARE_13;
		optype->format_id = OP_CODE_FORMAT_13;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) ) ) == ( (0U << 19U) | (0U << 18U) | (0U << 17U) | (1U << 16U) )) 
	{
		optype->code_id = OpCodeId_PREPARE_13;
		optype->format_id = OP_CODE_FORMAT_13;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1111_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );

	if (( reg2 == 0 ) && ( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) | (1U << 5U) ) ) == ( (0U << 19U) | (1U << 18U) | (0U << 17U) | (1U << 16U) | (1U << 5U) )) 
	{
		optype->code_id = OpCodeId_LD_BU_14;
		optype->format_id = OP_CODE_FORMAT_14;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) | (1U << 5U) ) ) == ( (0U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) | (1U << 5U) )) 
	{
		optype->code_id = OpCodeId_LD_HU_14;
		optype->format_id = OP_CODE_FORMAT_14;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) | (1U << 5U) ) ) == ( (1U << 19U) | (1U << 18U) | (0U << 17U) | (1U << 16U) | (1U << 5U) )) 
	{
		optype->code_id = OpCodeId_ST_H_14;
		optype->format_id = OP_CODE_FORMAT_14;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 16U) ) ) == ( (0U << 16U) )) 
	{
		optype->code_id = OpCodeId_JR_5;
		optype->format_id = OP_CODE_FORMAT_5;
		return 0;
	}
	if (( reg2 !=  0 ) && ( value & ( (1U << 16U) ) ) == ( (0U << 16U) )) 
	{
		optype->code_id = OpCodeId_JARL_5;
		optype->format_id = OP_CODE_FORMAT_5;
		return 0;
	}
	if (( reg2 !=  0 ) && ( value & ( (1U << 16U) ) ) == ( (1U << 16U) )) 
	{
		optype->code_id = OpCodeId_LD_BU_7;
		optype->format_id = OP_CODE_FORMAT_7;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 18U) | (1U << 17U) | (1U << 16U) ) ) == ( (0U << 18U) | (1U << 17U) | (1U << 16U) )) 
	{
		optype->code_id = OpCodeId_PREPARE_13;
		optype->format_id = OP_CODE_FORMAT_13;
		return 0;
	}
	if (( reg2 == 0 ) && ( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) | (1U << 16U) ) ) == ( (0U << 19U) | (0U << 18U) | (0U << 17U) | (1U << 16U) )) 
	{
		optype->code_id = OpCodeId_PREPARE_13;
		optype->format_id = OP_CODE_FORMAT_13;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1111_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );

	if (( value & ( (1U << 15U) | (1U << 14U) ) ) == ( (0U << 15U) | (0U << 14U) )) 
	{
		optype->code_id = OpCodeId_SET1_8;
		optype->format_id = OP_CODE_FORMAT_8;
		return 0;
	}
	if (( value & ( (1U << 15U) | (1U << 14U) ) ) == ( (0U << 15U) | (1U << 14U) )) 
	{
		optype->code_id = OpCodeId_NOT1_8;
		optype->format_id = OP_CODE_FORMAT_8;
		return 0;
	}
	if (( value & ( (1U << 15U) | (1U << 14U) ) ) == ( (1U << 15U) | (0U << 14U) )) 
	{
		optype->code_id = OpCodeId_CLR1_8;
		optype->format_id = OP_CODE_FORMAT_8;
		return 0;
	}
	if (( value & ( (1U << 15U) | (1U << 14U) ) ) == ( (1U << 15U) | (1U << 14U) )) 
	{
		optype->code_id = OpCodeId_TST1_8;
		optype->format_id = OP_CODE_FORMAT_8;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_base_code_1111_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint16 reg2 = GET_REG2(code[0]);
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );

	if (( reg2 !=  0 ) && ( value & ( (1U << 16U) ) ) == ( (1U << 16U) )) 
	{
		optype->code_id = OpCodeId_LD_HU_7;
		optype->format_id = OP_CODE_FORMAT_7;
		return 0;
	}
	else {
		return -1;
	}
}


/*
 * Extend
 */
static int op_parse_extend_code_0000_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 4U) ) ) == ( (0U << 4U) )) 
	{
		optype->code_id = OpCodeId_SETF_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	if (( value & ( (1U << 4U) ) ) == ( (1U << 4U) )) 
	{
		optype->code_id = OpCodeId_RIE_10;
		optype->format_id = OP_CODE_FORMAT_10;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0000_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	if (TRUE) 
	{
		optype->code_id = OpCodeId_LDSR_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0000_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	if (TRUE) 
	{
		optype->code_id = OpCodeId_STSR_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0000_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_0001_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 20U) | (1U << 17U) ) ) == ( (0U << 20U) | (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_SHR_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	if (( value & ( (1U << 20U) | (1U << 17U) ) ) == ( (0U << 20U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_SHR_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0001_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 20U) | (1U << 17U) ) ) == ( (0U << 20U) | (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_SAR_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	if (( value & ( (1U << 20U) | (1U << 17U) ) ) == ( (0U << 20U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_SAR_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0001_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 20U) | (1U << 18U) | (1U << 17U) ) ) == ( (0U << 20U) | (0U << 18U) | (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_SHL_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	if (( value & ( (1U << 20U) | (1U << 18U) | (1U << 17U) ) ) == ( (0U << 20U) | (0U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_SHL_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0001_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 18U) | (1U << 17U) ) ) == ( (0U << 18U) | (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_SET1_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	if (( value & ( (1U << 18U) | (1U << 17U) ) ) == ( (0U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_NOT1_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	if (( value & ( (1U << 18U) | (1U << 17U) ) ) == ( (1U << 18U) | (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_CLR1_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	if (( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) == ( (0U << 19U) | (1U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_TST1_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	if (( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) == ( (1U << 19U) | (1U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_CAXI_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0010_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	if (TRUE) 
	{
		optype->code_id = OpCodeId_TRAP_10;
		optype->format_id = OP_CODE_FORMAT_10;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0010_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 11U) ) ) == ( (0U << 11U) )) 
	{
		optype->code_id = OpCodeId_HALT_10;
		optype->format_id = OP_CODE_FORMAT_10;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0010_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) == ( (0U << 19U) | (0U << 18U) | (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_RETI_10;
		optype->format_id = OP_CODE_FORMAT_10;
		return 0;
	}
	if (( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) == ( (1U << 19U) | (0U << 18U) | (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_EIRET_10;
		optype->format_id = OP_CODE_FORMAT_10;
		return 0;
	}
	if (( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) == ( (1U << 19U) | (0U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_FERET_10;
		optype->format_id = OP_CODE_FORMAT_10;
		return 0;
	}
	if (( value & ( (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) == ( (0U << 19U) | (1U << 18U) | (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_CTRET_10;
		optype->format_id = OP_CODE_FORMAT_10;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0010_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 15U) | (1U << 14U) | (1U << 13U) | (1U << 12U) | (1U << 11U) ) ) == ( (0U << 15U) | (0U << 14U) | (0U << 13U) | (0U << 12U) | (0U << 11U) )) 
	{
		optype->code_id = OpCodeId_DI_10;
		optype->format_id = OP_CODE_FORMAT_10;
		return 0;
	}
	if (( value & ( (1U << 15U) | (1U << 14U) | (1U << 13U) | (1U << 12U) | (1U << 11U) ) ) == ( (1U << 15U) | (0U << 14U) | (0U << 13U) | (0U << 12U) | (0U << 11U) )) 
	{
		optype->code_id = OpCodeId_EI_10;
		optype->format_id = OP_CODE_FORMAT_10;
		return 0;
	}
	if (( value & ( (1U << 15U) | (1U << 14U) | (1U << 13U) | (1U << 12U) | (1U << 11U) ) ) == ( (1U << 15U) | (1U << 14U) | (0U << 13U) | (1U << 12U) | (0U << 11U) )) 
	{
		optype->code_id = OpCodeId_SYSCALL_10;
		optype->format_id = OP_CODE_FORMAT_10;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0011_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_0011_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_0011_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_0011_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_0100_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	if (TRUE) 
	{
		optype->code_id = OpCodeId_SASF_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0100_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 17U) ) ) == ( (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_MUL_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	if (( value & ( (1U << 17U) ) ) == ( (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_MULU_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0100_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 17U) ) ) == ( (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_MUL_12;
		optype->format_id = OP_CODE_FORMAT_12;
		return 0;
	}
	if (( value & ( (1U << 17U) ) ) == ( (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_MULU_12;
		optype->format_id = OP_CODE_FORMAT_12;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0100_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 17U) ) ) == ( (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_MUL_12;
		optype->format_id = OP_CODE_FORMAT_12;
		return 0;
	}
	if (( value & ( (1U << 17U) ) ) == ( (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_MULU_12;
		optype->format_id = OP_CODE_FORMAT_12;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0101_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 17U) ) ) == ( (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_DIVH_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	if (( value & ( (1U << 17U) ) ) == ( (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_DIVHU_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0101_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_0101_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 17U) ) ) == ( (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_DIV_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	if (( value & ( (1U << 17U) ) ) == ( (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_DIVU_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0101_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 17U) ) ) == ( (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_DIVQ_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	if (( value & ( (1U << 17U) ) ) == ( (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_DIVQU_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0110_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	if (TRUE) 
	{
		optype->code_id = OpCodeId_CMOV_12;
		optype->format_id = OP_CODE_FORMAT_12;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0110_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	if (TRUE) 
	{
		optype->code_id = OpCodeId_CMOV_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0110_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 18U) | (1U << 17U) ) ) == ( (0U << 18U) | (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_BSW_12;
		optype->format_id = OP_CODE_FORMAT_12;
		return 0;
	}
	if (( value & ( (1U << 18U) | (1U << 17U) ) ) == ( (0U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_BSH_12;
		optype->format_id = OP_CODE_FORMAT_12;
		return 0;
	}
	if (( value & ( (1U << 18U) | (1U << 17U) ) ) == ( (1U << 18U) | (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_HSW_12;
		optype->format_id = OP_CODE_FORMAT_12;
		return 0;
	}
	if (( value & ( (1U << 18U) | (1U << 17U) ) ) == ( (1U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_HSH_12;
		optype->format_id = OP_CODE_FORMAT_12;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0110_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 20U) | (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) == ( (0U << 20U) | (0U << 19U) | (0U << 18U) | (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_SCH0R_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	if (( value & ( (1U << 20U) | (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) == ( (0U << 20U) | (0U << 19U) | (0U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_SCH1R_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	if (( value & ( (1U << 20U) | (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) == ( (0U << 20U) | (0U << 19U) | (1U << 18U) | (0U << 17U) )) 
	{
		optype->code_id = OpCodeId_SCH0L_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	if (( value & ( (1U << 20U) | (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) == ( (0U << 20U) | (0U << 19U) | (1U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_SCH1L_9;
		optype->format_id = OP_CODE_FORMAT_9;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0111_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 20U) | (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) == ( (1U << 20U) | (1U << 19U) | (0U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_SATSUB_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	if (( value & ( (1U << 20U) | (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) != ( (1U << 20U) | (1U << 19U) | (0U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_SBF_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0111_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	uint32 value = ( (((uint32)code[1]) << 16U) | ((uint32)code[0]) );
	if (( value & ( (1U << 20U) | (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) == ( (1U << 20U) | (1U << 19U) | (0U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_SATADD_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	if (( value & ( (1U << 20U) | (1U << 19U) | (1U << 18U) | (1U << 17U) ) ) != ( (1U << 20U) | (1U << 19U) | (0U << 18U) | (1U << 17U) )) 
	{
		optype->code_id = OpCodeId_ADF_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0111_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	if (TRUE) 
	{
		optype->code_id = OpCodeId_MAC_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_0111_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	if (TRUE) 
	{
		optype->code_id = OpCodeId_MACU_11;
		optype->format_id = OP_CODE_FORMAT_11;
		return 0;
	}
	else {
		return -1;
	}
}

static int op_parse_extend_code_1000_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1000_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1000_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1000_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1001_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1001_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1001_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1001_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1010_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1010_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1010_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1010_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1011_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1011_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1011_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1011_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1100_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1100_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1100_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1100_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1101_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1101_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1101_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1101_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1110_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1110_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1110_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1110_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1111_00(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1111_01(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1111_10(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

static int op_parse_extend_code_1111_11(uint16 code[OP_DECODE_MAX], OperationCodeType *optype)
{
	return -1;
}

