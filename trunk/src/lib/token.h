#ifndef _TOKEN_H_
#define _TOKEN_H_

#include "std_types.h"

typedef enum {
	TOKEN_TYPE_VALUE_HEX,
	TOKEN_TYPE_VALUE_DEC,
	TOKEN_TYPE_STRING,
	TOKEN_TYPE_UNKNOWN,
} TokenEnumType;

typedef struct {
	uint32 value;
} TokenValueHexType;

typedef struct {
	uint32 value;
} TokenValueDecType;

#define TOKEN_STRING_MAX_SIZE	4096
typedef struct {
	uint32 len;
	uint8 str[TOKEN_STRING_MAX_SIZE];
} TokenStringType;

typedef struct {
	TokenEnumType	type;
	union {
		TokenValueHexType	hex;
		TokenValueDecType	dec;
		TokenStringType		str;
	} body;
} TokenValueType;

#define TOKEN_CONTAINER_MAX_SIZE	128
typedef struct {
	uint32	num;
	TokenValueType	array[TOKEN_CONTAINER_MAX_SIZE];
} TokenContainerType;

extern Std_ReturnType token_split(TokenContainerType *token_container, uint8 *str, uint32 len);
extern bool token_split_merge(const TokenContainerType *token_container, uint8 start_index, TokenStringType *out);

extern bool token_string_set(TokenStringType *dest, const char* src);
extern bool token_strcmp(const TokenStringType *str1, const TokenStringType *str2);
extern bool token_merge(TokenStringType *dest, const TokenStringType *src);



#endif /* _TOKEN_H_ */
