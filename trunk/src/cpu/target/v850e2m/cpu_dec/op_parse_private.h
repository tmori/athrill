#ifndef _OP_PARSE_PRIVATE_H_
#define _OP_PARSE_PRIVATE_H_

#define BIT_MASK_6_5		( (1U <<  6U) | (1U <<  5U) )
#define BIT_MASK_22_21		( (1U << 22U) | (1U << 21U) )

#define BIT_MASK_10_7		( (1U << 10U) | (1U <<  9U) | (1U <<  8U) | (1U <<  7U) )
#define BIT_MASK_26_23		( (1U << 26U) | (1U << 25U) | (1U << 24U) | (1U << 23U) )

#define BIT_MASK_10_5		( (1U << 10U) | (1U << 9U) | (1U << 8U) | (1U << 7U) | (1U << 6U) | (1U << 5U) )
#define BIT_MASK_16			( (1U << 16U) )


#define BIT_MASK_15_11		( (1U << 15U) | (1U << 14U) | (1U << 13U) | (1U << 12U) | (1U << 11U) )
#define BIT_MASK_4_0		( (1U <<  4U) | (1U <<  3U) | (1U <<  2U) | (1U <<  1U) | (1U <<  0U) )



#define BIT_UP_10_5			( (1U << 10U) | (1U << 9U) | (1U << 8U) | (1U << 7U) | (1U << 6U) | (1U << 5U) )
#define BIT_UP_16			( (1U << 16U) )
#define BIT_DOWN_16			( (0U << 16U) )


#define GET_VALUE_BIT_10_7(data)	( ( (data) & BIT_MASK_10_7 ) >> 7U )
#define GET_VALUE_BIT_26_23(data)	( ( (data) & BIT_MASK_26_23 ) >> 23U )

#define GET_VALUE_BIT_6_5(data)		( ( (data) & BIT_MASK_6_5 ) >> 5U )
#define GET_VALUE_BIT_22_21(data)	( ( (data) & BIT_MASK_22_21 ) >> 21U )

#define GET_VALUE_BIT_15_11(data)	( ( (data) & BIT_MASK_15_11 ) >> 11U )
#define GET_VALUE_BIT_4_0(data)		( ( (data) & BIT_MASK_4_0 ) >> 0U )

#define GET_REG1(data)	GET_VALUE_BIT_4_0(data)
#define GET_REG2(data)	GET_VALUE_BIT_15_11(data)

#define OP_PARSE_CODE_TABLE_NUM		64U
#define OP_PARSE_TABLE_INDEX(base_id, sub_id)	( ((base_id) << 2U) | (sub_id) )

typedef struct {
	int (*parse) (uint16 code[OP_DECODE_MAX], OperationCodeType *optype);
} OpParserType;

extern OpParserType op_parse_extend_code_table[OP_PARSE_CODE_TABLE_NUM];
extern OpParserType op_parse_base_code_table[OP_PARSE_CODE_TABLE_NUM];

#endif /* _OP_PARSE_PRIVATE_H_ */
