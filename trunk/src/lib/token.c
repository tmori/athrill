#include "token.h"
#include "std_errno.h"
#include "std_types.h"
#include "assert.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

bool token_strcmp(const TokenStringType *str1, const TokenStringType *str2)
{
	if (str1->len != str2->len) {
		return FALSE;
	}
	if (strncmp((const char*)str1->str, (const char*)str2->str, str1->len) != 0) {
		return FALSE;
	}
	return TRUE;
}

/*
 * デミリタ=空白,",",":"
 */
static bool is_delimiter(char c)
{
	if (isspace(c)) {
		return TRUE;
	}
	else if (c == ',') {
		return TRUE;
	}
	else if (c == ':') {
		return TRUE;
	}
	else if (c == '\0') {
		return TRUE;
	}
	return FALSE;
}

static void set_token(TokenContainerType *token_container, TokenStringType *buffer)
{
	uint32 inx = token_container->num - 1;
	buffer->str[buffer->len] = '\0';
	char   *endptr;

	long long ret64;
	long ret32;

	/*
	 * 10進数チェック
	 */
	errno = 0;
	ret32 = strtol((const char*)buffer->str, &endptr, 10);
	if ((errno == 0) && (*endptr == '\0')) {
		token_container->array[inx].type = TOKEN_TYPE_VALUE_DEC;
		token_container->array[inx].body.dec.value = (uint32)ret32;
		return;
	}
	/*
	 * 16進数チェック
	 */
	if ((buffer->len > 2) && buffer->str[0] == '0' && buffer->str[1] == 'x') {
		errno = 0;
		ret64 = strtoull((const char*)buffer->str, &endptr, 16);
		if ((errno == 0) && (*endptr == '\0')) {
			token_container->array[inx].type = TOKEN_TYPE_VALUE_HEX;
			token_container->array[inx].body.hex.value = (uint32)ret64;
			return;
		}
	}
	token_container->array[inx].type = TOKEN_TYPE_STRING;
	token_container->array[inx].body.str = *buffer;
	return;
}

typedef enum {
	TOKEN_CHECK_STATE_DEMILITER = 0,
	TOKEN_CHECK_STATE_CODE,
} TokenCheckStateType;
Std_ReturnType token_split(TokenContainerType *token_container, uint8 *str, uint32 len)
{
	uint32 i;
	volatile TokenCheckStateType state;
	TokenStringType buffer;

	state = TOKEN_CHECK_STATE_DEMILITER;
	buffer.len = 0;

	token_container->num = 0;

	for (i = 0; i < len; i++) {
		switch (state) {
		case TOKEN_CHECK_STATE_DEMILITER:
			if (is_delimiter(str[i]) == TRUE) {
				break;
			}
			if (token_container->num > TOKEN_CONTAINER_MAX_SIZE) {
				return STD_E_INVALID;
			}
			token_container->num++;
			buffer.str[buffer.len] = str[i];
			buffer.len++;
			if (i == (len - 1)) { //終端
				set_token(token_container, &buffer);
				buffer.len = 0;
				state = TOKEN_CHECK_STATE_DEMILITER;
				break;
			}
			state = TOKEN_CHECK_STATE_CODE;
			break;
		case TOKEN_CHECK_STATE_CODE:
			if (is_delimiter(str[i]) == TRUE) {
				set_token(token_container, &buffer);
				buffer.len = 0;
				state = TOKEN_CHECK_STATE_DEMILITER;
				break;
			}
			if (buffer.len >= (TOKEN_STRING_MAX_SIZE - 1)) {
				return STD_E_INVALID;
			}
			buffer.str[buffer.len] = str[i];
			buffer.len++;
			if (i == (len - 1)) { //終端
				set_token(token_container, &buffer);
				buffer.len = 0;
				state = TOKEN_CHECK_STATE_DEMILITER;
				break;
			}
			break;
		default:
			return STD_E_INVALID;
		}
	}

	return STD_E_OK;
}

bool token_split_merge(const TokenContainerType *token_container, uint8 start_index, TokenStringType *out)
{
	uint32 i;
	TokenStringType tmp;

	for (i = start_index; i < token_container->num; i++) {
		switch (token_container->array[i].type) {
		case TOKEN_TYPE_STRING:
			tmp.len = sprintf((char*)&tmp.str[0], "%s ", (char*)token_container->array[i].body.str.str);
			(void)token_merge(out, &tmp);
			break;
		case TOKEN_TYPE_VALUE_DEC:
			tmp.len = sprintf((char*)&tmp.str[0], "%d ", token_container->array[i].body.dec.value);
			(void)token_merge(out, &tmp);
			break;
		case TOKEN_TYPE_VALUE_HEX:
			tmp.len = sprintf((char*)&tmp.str[0], "0x%x ", token_container->array[i].body.hex.value);
			(void)token_merge(out, &tmp);
			break;
		default:
			break;
		}
	}
	return TRUE;
}


bool token_merge(TokenStringType *dest, const TokenStringType *src)
{
	if ((dest->len + src->len) > (TOKEN_STRING_MAX_SIZE - 1)) {
		return FALSE;
	}
	memcpy(&dest->str[dest->len], src->str,src->len);
	dest->len += src->len;
	dest->str[dest->len] = '\0';

	return TRUE;
}

bool token_string_set(TokenStringType *dest, const char* src)
{
	uint32 len = strlen(src);
	if (len > TOKEN_STRING_MAX_SIZE) {
		return FALSE;
	}
	dest->len = strlen(src);
	memcpy(dest->str, src, dest->len);
	dest->str[dest->len] = '\0';
	return TRUE;
}
