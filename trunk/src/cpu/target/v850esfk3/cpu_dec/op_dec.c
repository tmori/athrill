#include "../cpu_dec/op_dec.h"

typedef struct {
	int (*decode) (uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
} OpDecoderType;


static int OpDecode1(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static int OpDecode2(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static int OpDecode3(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static int OpDecode4(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static int OpDecode5(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static int OpDecode6(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static int OpDecode7(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static int OpDecode8(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static int OpDecode9(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static int OpDecode10(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static int OpDecode11(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static int OpDecode12(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static int OpDecode13(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code);
static OpDecoderType OpDecoder[OP_CODE_FORMAT_NUM] = {
	{ OpDecode1 },
	{ OpDecode2 },
	{ OpDecode3 },
	{ OpDecode4 },
	{ OpDecode5 },
	{ OpDecode6 },
	{ OpDecode7 },
	{ OpDecode8 },
	{ OpDecode9 },
	{ OpDecode10 },
	{ OpDecode11 },
	{ OpDecode12 },
	{ OpDecode13 },
};


static OpCodeFormatId is_LD_HU(uint16 code[OP_DECODE_MAX])
{
	uint16 opcode = code[0] >> 5U;
	uint16 reg2 = code[0] & 0xF800;
	uint16 bit16 = code[1] & 0x0001;

	opcode = opcode & 0x003F;
	if (opcode != 0b111111) {
		return OP_CODE_FORMAT_UNKNOWN;
	}
	if ((reg2 == 0U) || (bit16 == 0U)) {
		return OP_CODE_FORMAT_UNKNOWN;
	}
	return OP_CODE_FORMAT_7;
}
static OpCodeFormatId is_SLD_BU_HU(uint16 code[OP_DECODE_MAX])
{
	uint16 opcode = code[0] >> 4U;
	uint16 reg2 = code[0] & 0xF800;

	opcode = opcode & 0x003E;
	if (opcode != 0b0000110) {
		return OP_CODE_FORMAT_UNKNOWN;
	}
	if ((reg2 == 0U)) {
		return OP_CODE_FORMAT_UNKNOWN;
	}
	return OP_CODE_FORMAT_4;
}
static OpCodeFormatId is_CMOV(uint16 code[OP_DECODE_MAX])
{
	uint16 opcode = code[0] >> 5U;
	uint16 subop = ((code[1] >> 5U) & 0x003F);

	opcode = opcode & 0x003F;
	if (opcode != 0b111111) {
		return OP_CODE_FORMAT_UNKNOWN;
	}
	switch (subop) {
	case 0b011000:
	//case 0b011010:
		return OP_CODE_FORMAT_12;
	case 0b011001:
	//case 0b011011:
		return OP_CODE_FORMAT_11;
	default:
		return OP_CODE_FORMAT_UNKNOWN;
	}
}
static OpCodeFormatId getSpecialFormatId(uint16 code[OP_DECODE_MAX])
{
	OpCodeFormatId id;

	id = is_LD_HU(code);
	if (id != OP_CODE_FORMAT_UNKNOWN) {
		return id;
	}
	id = is_SLD_BU_HU(code);
	if (id != OP_CODE_FORMAT_UNKNOWN) {
		return id;
	}
	id = is_CMOV(code);
	if (id != OP_CODE_FORMAT_UNKNOWN) {
		return id;
	}
	return OP_CODE_FORMAT_UNKNOWN;
}

/*
 *
 */
int OpDecode(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	uint16 opcode =  ( (code[0] >> 5) & 0x3F );
	uint16 subcode = ( (code[1] >> 5) & 0x3F );
	OpCodeFormatId id;

	//printf("code0=0x%x\n", code[0]);
	//printf("code1=0x%x\n", code[1]);
	//printf("opcode=0x%x\n", opcode);
	id = getSpecialFormatId(code);
	if (id == OP_CODE_FORMAT_UNKNOWN) {
		id = OpCode2FormatId(opcode, subcode);
		if (id == OP_CODE_FORMAT_UNKNOWN) {
			return -1;
		}
		if (id == OP_CODE_FORMAT_5) {
			/*
			 * この時点でFORMAT5と判断された場合，
			 * FORMAT7 or FORMAT13判定が必要となる．
			 */
			/*
			 * 16bit != 0 の場合はFORMAT7 or FORMAT13
			 */
			if ((code[1] & 0x0001) != 0) {
				/*
				 * reg2 != 0 の場合はFORMAT7
				 * reg2 == 0 の場合はFORMAT13
				 */
				uint16 reg2 = (code[0] >> 11U);
				if (reg2 != 0U) {
					id = OP_CODE_FORMAT_7;
				}
				else {
					id = OP_CODE_FORMAT_13;
				}
			}
		}
		else if (id == OP_CODE_FORMAT_6) {
			/*
			 * この時点でFORMAT6と判断された場合，
			 * FORMAT13判定が必要となる．
			 */
			/*
			 * opcode=0b11001X かつ　reg2==0 の場合はFORMAT13
			 */
			uint16 opcode5 = ( (code[0] >> 6U) & 0x001F );
			uint16 reg2 = (code[0] >> 11U);
			if ((opcode5 == 0b11001) && reg2 == 0U) {
				id = OP_CODE_FORMAT_13;
			}
		}
	}
	decoded_code->type_id = id;
	return OpDecoder[id].decode(code, decoded_code);
}
static int OpDecode1(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	decoded_code->type1.opcode = ( (code[0] >> 5) & 0x003F );
	decoded_code->type1.reg1   = ( (code[0]) & 0x001F );
	decoded_code->type1.reg2   = ( (code[0] >> 11) & 0x001F );
	return 0;
}
static int OpDecode2(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	decoded_code->type2.opcode = ( (code[0] >> 5) & 0x003F );
	decoded_code->type2.imm    = ( (code[0]) & 0x001F );
	decoded_code->type2.reg2   = ( (code[0] >> 11) & 0x001F );
	return 0;
}

static int OpDecode3(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	uint32 tmp1 =  ( (code[0] >> 11) & 0x001F ); /* 5bits */
	uint32 tmp2 =  ( (code[0] >>  4) & 0x0007 ); /* 3bits */

	decoded_code->type3.opcode = ( (code[0] >> 7) & 0x000F );
	decoded_code->type3.disp   = ( (tmp1 << 3) | (tmp2) );
	decoded_code->type3.cond   = ( (code[0]) & 0x000F );
	return 0;
}

static int OpDecode4(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	/* 4_1 */
	decoded_code->type4_1.opcode = ( (code[0] >> 7)  & 0x000F );
	decoded_code->type4_1.disp   = ( (code[0] >> 1)  & 0x3F );
	decoded_code->type4_1.reg2   = ( (code[0] >> 11) & 0x001F );
	decoded_code->type4_1.gen    = ( (code[0] >>  0) & 0x0001 );

	/* 4_2 */
	decoded_code->type4_2.opcode = ( (code[0] >> 4)  & 0x007F );
	decoded_code->type4_2.disp   = ( (sint8)(code[0] >> 0)  & 0x0F );
	decoded_code->type4_2.reg2   = ( (code[0] >> 11) & 0x001F );

	return 0;
}

static int OpDecode5(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	uint32 tmp1 = ( (code[0] >> 0) & 0x003F ); /* 6bits */;
	uint32 tmp2 = ( (code[1] >> 0) & 0xFFFF );/* 16bits */;

	decoded_code->type5.opcode = ( (code[0] >> 6)  & 0x001F );
	decoded_code->type5.reg2   = ( (code[0] >> 11) & 0x001F );
	decoded_code->type5.disp = ( (tmp1 << 16) | tmp2 );
	return 0;
}

static int OpDecode6(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	uint32 tmp2 = ( (code[1] >> 0) & 0xFFFF );/* 16bits */;

	decoded_code->type6.opcode = ( (code[0] >> 5)  & 0x003F );
	decoded_code->type6.reg2   = ( (code[0] >> 11) & 0x001F );
	decoded_code->type6.reg1   = ( code[0] & 0x001F );
	decoded_code->type6.imm   = tmp2;

	return 0;
}

static int OpDecode7(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	uint32 tmp2 = ( (code[1] >> 0) & 0xFFFF );/* 16bits */;

	decoded_code->type7.opcode = ( (code[0] >> 5)  & 0x003F );
	decoded_code->type7.reg2   = ( (code[0] >> 11) & 0x001F );
	decoded_code->type7.reg1   = ( code[0] & 0x001F );
	decoded_code->type7.disp   = ( (tmp2 >> 1) & 0x7FFF );
	decoded_code->type7.gen    = ( tmp2 & 0x0001 );

	return 0;
}

static int OpDecode8(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	sint16 tmp2 = ( (code[1] >> 0) & 0xFFFF );/* 16bits */;

	decoded_code->type8.opcode = ( (code[0] >> 5)  & 0x003F );
	decoded_code->type8.sub   = ( (code[0] >> 14) & 0x0003 );
	decoded_code->type8.bit   = ( (code[0] >> 11) & 0x0007 );
	decoded_code->type8.reg1   = ( code[0] & 0x001F );
	decoded_code->type8.disp   = tmp2;

	return 0;
}

static int OpDecode9(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	uint16 tmp1 = ( (code[0] >> 0) & 0xFFFF );/* 16bits */;
	uint16 tmp2 = ( (code[1] >> 0) & 0xFFFF );/* 16bits */;

	decoded_code->type9.opcode = ( (tmp1 >> 5)  & 0x003F );
	decoded_code->type9.reg2   = ( (tmp1 >> 11) & 0x001F );
	decoded_code->type9.gen    = ( tmp1 & 0x001F );
	decoded_code->type9.rfu1   = ( (tmp2 >>  1) & 0x000F );
	decoded_code->type9.rfu2   = ( (tmp2 >> 11) & 0x001F );
	decoded_code->type9.sub    = ( (tmp2 >>  5) & 0x003F );

	return 0;
}

static int OpDecode10(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	uint32 tmp2 = ( (code[1] >> 0) & 0xFFFF );/* 16bits */;

	decoded_code->type10.opcode = ( (code[0] >> 5)  & 0x003F );
	decoded_code->type10.rfu1   = ( (code[0] >> 11) & 0x0003 );
	decoded_code->type10.gen1   = ( (code[0] >> 13) & 0x0007 );
	decoded_code->type10.gen2   = ( (code[0] >> 0)  & 0x001F );

	decoded_code->type10.rfu2   = ( (tmp2 >> 11) & 0x001F );
	decoded_code->type10.rfu3   = ( (tmp2 >>  1) & 0x000F );
	decoded_code->type10.sub    = ( (tmp2 >>  5) & 0x003F );

	return 0;
}

static int OpDecode11(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	uint32 tmp2 = ( (code[1] >> 0) & 0xFFFF );/* 16bits */;

	decoded_code->type11.opcode = ( (code[0] >> 5)  & 0x003F );
	decoded_code->type11.reg2   = ( (code[0] >> 11) & 0x001F );
	decoded_code->type11.reg1   = ( (code[0] >>  0) & 0x001F );

	decoded_code->type11.rfu    = ( (tmp2 >>  2) & 0x0007 );
	decoded_code->type11.reg3   = ( (tmp2 >> 11) & 0x001F );
	decoded_code->type11.sub1   = ( (tmp2 >>  5) & 0x003F );
	decoded_code->type11.sub2   = ( (tmp2 >>  1) & 0x0001 );

	return 0;
}

static int OpDecode12(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	uint32 tmp2 = ( (code[1] >> 0) & 0xFFFF );/* 16bits */;

	decoded_code->type12.opcode = ( (code[0] >> 5)  & 0x003F );
	decoded_code->type12.reg2   = ( (code[0] >> 11) & 0x001F );
	decoded_code->type12.imml    = ( (code[0] >>  0) & 0x001F );

	decoded_code->type12.reg3   = ( (tmp2 >> 11) & 0x001F );
	decoded_code->type12.sub1   = ( (tmp2 >>  7) & 0x000F );
	decoded_code->type12.sub2   = ( (tmp2 >>  1) & 0x0001 );
	decoded_code->type12.immh   = ( (tmp2 >>  2) & 0x001F );
	return 0;
}

static int OpDecode13(uint16 code[OP_DECODE_MAX], OpDecodedCodeType *decoded_code)
{
	uint32 tmp2 = ( (code[1] >> 0) & 0xFFFF );/* 16bits */;
	uint16 t1 = ((code[0]) & 0x0001);
	uint16 t2 = ((code[1] >> 5) & 0x07FF);
	uint32 i;

	decoded_code->type13.opcode = ( (code[0] >> 6)  & 0x001F );
	decoded_code->type13.rfu    = ( (code[0] >> 11) & 0x001F );
	decoded_code->type13.imm    = ( (code[0] >>  1) & 0x001F );

	decoded_code->type13.gen   = ( (tmp2 >> 0) & 0x001F );


	for (i = 0; i < 32; i++) {
		decoded_code->type13.list[i] = 0;
	}
	decoded_code->type13.list[30] = (t1 != 0U) ? 1U : 0U;
	decoded_code->type13.list[31] = ((t2 & 0x0001) != 0) ? 1U : 0U; /* 21 */
	decoded_code->type13.list[29] = ((t2 & 0x0002) != 0) ? 1U : 0U; /* 22 */
	decoded_code->type13.list[28] = ((t2 & 0x0004) != 0) ? 1U : 0U; /* 23 */
	decoded_code->type13.list[23] = ((t2 & 0x0008) != 0) ? 1U : 0U; /* 24 */
	decoded_code->type13.list[22] = ((t2 & 0x0010) != 0) ? 1U : 0U; /* 25 */
	decoded_code->type13.list[21] = ((t2 & 0x0020) != 0) ? 1U : 0U; /* 26 */
	decoded_code->type13.list[20] = ((t2 & 0x0040) != 0) ? 1U : 0U; /* 27 */
	decoded_code->type13.list[27] = ((t2 & 0x0080) != 0) ? 1U : 0U; /* 28 */
	decoded_code->type13.list[26] = ((t2 & 0x0100) != 0) ? 1U : 0U; /* 29 */
	decoded_code->type13.list[25] = ((t2 & 0x0200) != 0) ? 1U : 0U; /* 30 */
	decoded_code->type13.list[24] = ((t2 & 0x0400) != 0) ? 1U : 0U; /* 31 */


	return 0;
}

