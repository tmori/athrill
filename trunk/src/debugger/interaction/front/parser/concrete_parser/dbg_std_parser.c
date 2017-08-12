#include "front/parser/concrete_parser/dbg_std_parser.h"
#include "concrete_executor/dbg_std_executor.h"
#include <string.h>

/************************************************************************************
 * break コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType break_string = {
		.len = 5,
		.str = { 'b', 'r', 'e', 'a', 'k', '\0' },
};
static const TokenStringType break_string_short = {
		.len = 1,
		.str = { 'b', '\0' },
};


static const TokenStringType break_info_string = {
		.len = 4,
		.str = { 'i', 'n', 'f', 'o', '\0' },
};

DbgCmdExecutorType *dbg_parse_break(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	DbgCmdExecutorBreakType *parsed_args = (DbgCmdExecutorBreakType *)arg->parsed_args;

	if ((token_container->num != 2) && (token_container->num != 3)) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &break_string) == TRUE) ||
			(token_strcmp(&token_container->array[0].body.str, &break_string_short) == TRUE)) {
		if (token_container->array[1].type == TOKEN_TYPE_VALUE_HEX) {
			arg->std_id = DBG_CMD_STD_ID_BREAK;
			arg->run = dbg_std_executor_break;
			parsed_args->type = DBG_CMD_BBREAK_SET;
			parsed_args->break_addr = token_container->array[1].body.hex.value;
			return arg;
		}
		else if (token_container->array[1].type == TOKEN_TYPE_STRING) {
			if (token_container->num == 2) {
				arg->std_id = DBG_CMD_STD_ID_BREAK;
				parsed_args->type = DBG_CMD_BBREAK_SET_SYMBOL;
				parsed_args->symbol = token_container->array[1].body.str;
				arg->run = dbg_std_executor_break;
				return arg;
			}
			else if ((token_container->num == 3) && (token_container->array[2].type == TOKEN_TYPE_VALUE_DEC)) {
				arg->std_id = DBG_CMD_STD_ID_BREAK;
				parsed_args->type = DBG_CMD_BREAK_SET_FILE_LINE;
				parsed_args->symbol = token_container->array[1].body.str;
				parsed_args->line = token_container->array[2].body.dec.value;
				arg->run = dbg_std_executor_break;
				return arg;
			}
		}
	}
	else if (token_strcmp(&token_container->array[0].body.str, &break_info_string) == TRUE) {
		if (token_strcmp(&token_container->array[1].body.str, &break_string) == TRUE) {
			arg->std_id = DBG_CMD_STD_ID_BREAK;
			arg->run = dbg_std_executor_break;
			parsed_args->type = DBG_CMD_BREAK_INFO;
			return arg;
		}
	}
	return NULL;
}

/************************************************************************************
 * watch コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType watch_string = {
		.len = 5,
		.str = { 'w', 'a', 't', 'c', 'h', '\0' },
};
static const TokenStringType watch_string_short = {
		.len = 1,
		.str = { 'w', '\0' },
};
static const TokenStringType watch_info_string = {
		.len = 4,
		.str = { 'i', 'n', 'f', 'o', '\0' },
};
static const TokenStringType watch_option_string_readset = {
		.len = 1,
		.str = { 'r', '\0' },
};
static const TokenStringType watch_option_string_writeset = {
		.len = 1,
		.str = { 'w', '\0' },
};
static const TokenStringType watch_option_string_rwset = {
		.len = 2,
		.str = { 'r', 'w', '\0' },
};
static const TokenStringType watch_option_string_delete = {
		.len = 1,
		.str = { 'd', '\0' },
};

DbgCmdExecutorType *dbg_parse_watch_data(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	DbgCmdExecutorWatchDataType *parsed_args = (DbgCmdExecutorWatchDataType *)arg->parsed_args;

	if (token_container->num < 2) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}
	if (token_container->array[1].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &watch_string) == TRUE) ||
			(token_strcmp(&token_container->array[0].body.str, &watch_string_short) == TRUE)) {
		/*
		 * delete
		 */
		if ((token_strcmp(&token_container->array[1].body.str, &watch_option_string_delete) == TRUE)) {
			if (token_container->num == 2) {
				arg->std_id = DBG_CMD_STD_ID_DATA_WATCH;
				parsed_args->type = DBG_CMD_WATCH_DELETE_ALL;
				arg->run = dbg_std_executor_watch_data;
				return arg;
			}
			else if ((token_container->num == 3) && (token_container->array[2].type == TOKEN_TYPE_VALUE_DEC)) {
				arg->std_id = DBG_CMD_STD_ID_DATA_WATCH;
				parsed_args->type = DBG_CMD_WATCH_DELETE_ONE;
				parsed_args->delno = token_container->array[2].body.dec.value;
				arg->run = dbg_std_executor_watch_data;
				return arg;
			}
		}
		/*
		 * set
		 */
		if ((token_strcmp(&token_container->array[1].body.str, &watch_option_string_readset) == TRUE)) {
			parsed_args->watch_type = DBG_CMD_WATCH_TYPE_READ;
		}
		else if ((token_strcmp(&token_container->array[1].body.str, &watch_option_string_writeset) == TRUE)) {
			parsed_args->watch_type = DBG_CMD_WATCH_TYPE_WRITE;
		}
		else if ((token_strcmp(&token_container->array[1].body.str, &watch_option_string_rwset) == TRUE)) {
			parsed_args->watch_type = DBG_CMD_WATCH_TYPE_RW;
		}
		else {
			return NULL;
		}

		/*
		 * check symbol or addr/size
		 */
		if ((token_container->num == 3) &&
				(token_container->array[2].type == TOKEN_TYPE_STRING)) {
			arg->std_id = DBG_CMD_STD_ID_DATA_WATCH;
			parsed_args->type = DBG_CMD_WATCH_SET_SYMBOL;
			parsed_args->symbol = token_container->array[2].body.str;
			arg->run = dbg_std_executor_watch_data;
			return arg;
		}
		else if ((token_container->num == 4) &&
				(token_container->array[2].type == TOKEN_TYPE_VALUE_HEX) &&
				(token_container->array[3].type == TOKEN_TYPE_VALUE_DEC)) {
			arg->std_id = DBG_CMD_STD_ID_DATA_WATCH;
			parsed_args->type = DBG_CMD_WATCH_SET;
			parsed_args->addr = token_container->array[2].body.hex.value;
			parsed_args->size = token_container->array[3].body.dec.value;
			arg->run = dbg_std_executor_watch_data;
			return arg;
		}
	}
	else if (token_strcmp(&token_container->array[0].body.str, &watch_info_string) == TRUE) {
		if (token_strcmp(&token_container->array[1].body.str, &watch_string) == TRUE) {
			arg->std_id = DBG_CMD_STD_ID_DATA_WATCH;
			parsed_args->type = DBG_CMD_WATCH_INFO;
			arg->run = dbg_std_executor_watch_data;
			return arg;
		}
	}
	return NULL;
}
/************************************************************************************
 * delete コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType delete_string = {
		.len = 6,
		.str = { 'd', 'e', 'l', 'e', 't', 'e', '\0' },
};
static const TokenStringType delete_string_short = {
		.len = 1,
		.str = { 'd', '\0' },
};

DbgCmdExecutorType *dbg_parse_delete(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	DbgCmdExecutorDeleteType *parsed_args = (DbgCmdExecutorDeleteType *)arg->parsed_args;

	if (token_container->num > 2) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &delete_string) == TRUE) ||
			(token_strcmp(&token_container->array[0].body.str, &delete_string_short) == TRUE)) {
		if (token_container->num == 1) {
			arg->std_id = DBG_CMD_STD_ID_DELETE;
			parsed_args->type = DBG_CMD_DELETE_ALL;
			arg->run = dbg_std_executor_delete;
			return arg;
		}
		else if ((token_container->num == 2) &&  (token_container->array[1].type == TOKEN_TYPE_VALUE_DEC)) {
			arg->std_id = DBG_CMD_STD_ID_DELETE;
			parsed_args->type = DBG_CMD_DELETE_ONE;
			parsed_args->delete_break_no = token_container->array[1].body.dec.value;
			arg->run = dbg_std_executor_delete;
			return arg;
		}
	}
	return NULL;
}


/************************************************************************************
 * cont コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType cont_string = {
		.len = 4,
		.str = { 'c', 'o', 'n', 't', '\0' },
};
static const TokenStringType cont_string_short = {
		.len = 1,
		.str = { 'c', '\0' },
};

DbgCmdExecutorType *dbg_parse_cont(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	DbgCmdExecutorContType *parsed_args = (DbgCmdExecutorContType *)arg->parsed_args;

	if (token_container->num > 2) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}
	if ((token_container->num == 2) &&
			(token_container->array[1].type != TOKEN_TYPE_VALUE_DEC)) {
		return NULL;
	}


	if ((token_strcmp(&token_container->array[0].body.str, &cont_string) == TRUE) ||
			(token_strcmp(&token_container->array[0].body.str, &cont_string_short) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_CONT;
		if (token_container->num == 1) {
			parsed_args->type = DBG_CMD_CONT_ALL;
		}
		else {
			parsed_args->type = DBG_CMD_CONT_CLOCKS;
			parsed_args->cont_clocks = token_container->array[1].body.dec.value;
		}
		arg->run = dbg_std_executor_cont;
		return arg;
	}
	return NULL;
}

/************************************************************************************
 * intr コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType intr_string = {
		.len = 4,
		.str = { 'i', 'n', 't', 'r', '\0' },
};
static const TokenStringType intr_string_short = {
		.len = 1,
		.str = { 'i', '\0' },
};

DbgCmdExecutorType *dbg_parse_intr(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	DbgCmdExecutorIntrType *parsed_args = (DbgCmdExecutorIntrType *)arg->parsed_args;

	if (token_container->num != 2) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}
	if ((token_container->array[1].type != TOKEN_TYPE_VALUE_DEC)) {
		return NULL;
	}


	if ((token_strcmp(&token_container->array[0].body.str, &intr_string) == TRUE) ||
			(token_strcmp(&token_container->array[0].body.str, &intr_string_short) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_INTR;
		parsed_args->intno = token_container->array[1].body.dec.value;
		arg->run = dbg_std_executor_intr;
		return arg;
	}
	return NULL;
}



/************************************************************************************
 * elaps コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType elaps_string = {
		.len = 5,
		.str = { 'e', 'l', 'a', 'p', 's', '\0' },
};
static const TokenStringType elaps_string_short = {
		.len = 1,
		.str = { 'e', '\0' },
};
DbgCmdExecutorType *dbg_parse_elaps(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	if (token_container->num != 1) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &elaps_string) == TRUE) ||
			(token_strcmp(&token_container->array[0].body.str, &elaps_string_short) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_ELAPS;
		arg->run = dbg_std_executor_elaps;
		return arg;
	}
	return NULL;
}


/************************************************************************************
 * next コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType next_string = {
		.len = 4,
		.str = { 'n', 'e', 'x', 't', '\0' },
};
static const TokenStringType next_string_short = {
		.len = 1,
		.str = { 'n', '\0' },
};
DbgCmdExecutorType *dbg_parse_next(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	if (token_container->num != 1) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &next_string) == TRUE) ||
			(token_strcmp(&token_container->array[0].body.str, &next_string_short) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_NEXT;
		arg->run = dbg_std_executor_next;
		return arg;
	}
	return NULL;
}


/************************************************************************************
 * return コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType return_string = {
		.len = 6,
		.str = { 'r', 'e', 't', 'u', 'r', 'n', '\0' },
};
static const TokenStringType return_string_short = {
		.len = 1,
		.str = { 'r', '\0' },
};
DbgCmdExecutorType *dbg_parse_return(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	if (token_container->num != 1) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &return_string) == TRUE) ||
			(token_strcmp(&token_container->array[0].body.str, &return_string_short) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_RETURN;
		arg->run = dbg_std_executor_return;
		return arg;
	}
	return NULL;
}


/************************************************************************************
 * view コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType view_string = {
		.len = 4,
		.str = { 'v', 'i', 'e', 'w', '\0' },
};
static const TokenStringType view_string_short = {
		.len = 1,
		.str = { 'v', '\0' },
};
DbgCmdExecutorType *dbg_parse_view(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	if (token_container->num != 1) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &view_string) == TRUE) ||
			(token_strcmp(&token_container->array[0].body.str, &view_string_short) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_VIEW;
		arg->run = dbg_std_executor_view;
		return arg;
	}
	return NULL;
}


/************************************************************************************
 * print コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType print_string = {
		.len = 5,
		.str = { 'p', 'r', 'i', 'n', 't', '\0' },
};
static const TokenStringType print_string_short = {
		.len = 1,
		.str = { 'p', '\0' },
};
DbgCmdExecutorType *dbg_parse_print(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	DbgCmdExecutorPrintType *parsed_args = (DbgCmdExecutorPrintType *)arg->parsed_args;

	if ((token_container->num != 2) && (token_container->num != 3)) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &print_string) == FALSE) &&
			(token_strcmp(&token_container->array[0].body.str, &print_string_short) == FALSE)) {
		return NULL;
	}

	if (token_container->num == 2) {
		if (token_container->array[1].type == TOKEN_TYPE_STRING) {
			arg->std_id = DBG_CMD_STD_ID_PRINT;
			parsed_args->type = DBG_CMD_PRINT_SYMBOL;
			parsed_args->symbol = token_container->array[1].body.str;
			arg->run = dbg_std_executor_print;
			return arg;
		}
		else if (token_container->array[1].type == TOKEN_TYPE_VALUE_HEX) {
			arg->std_id = DBG_CMD_STD_ID_PRINT;
			parsed_args->type = DBG_CMD_PRINT_ADDR;
			parsed_args->addr = token_container->array[1].body.dec.value;
			arg->run = dbg_std_executor_print;
			return arg;
		}
	}
	else {
		if ((token_container->array[1].type == TOKEN_TYPE_VALUE_HEX) &&
				(token_container->array[2].type == TOKEN_TYPE_VALUE_DEC)) {
			arg->std_id = DBG_CMD_STD_ID_PRINT;
			parsed_args->type = DBG_CMD_PRINT_ADDR_SIZE;
			parsed_args->addr = token_container->array[1].body.dec.value;
			parsed_args->size = token_container->array[2].body.dec.value;
			arg->run = dbg_std_executor_print;
			return arg;
		}
	}

	return NULL;
}
/************************************************************************************
 * memset コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType memset_string = {
		.len = 3,
		.str = { 's', 'e', 't', '\0' },
};
static const TokenStringType memset_string_short = {
		.len = 1,
		.str = { 's', '\0' },
};
DbgCmdExecutorType *dbg_parse_memset(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	DbgCmdExecutorMemsetType *parsed_args = (DbgCmdExecutorMemsetType *)arg->parsed_args;

	if ((token_container->num != 3)) {
		return NULL;
	}
	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &memset_string) == FALSE) &&
			(token_strcmp(&token_container->array[0].body.str, &memset_string_short) == FALSE)) {
		return NULL;
	}

	if (token_container->array[1].type != TOKEN_TYPE_VALUE_HEX) {
		return NULL;
	}
	if (token_container->array[2].type != TOKEN_TYPE_VALUE_DEC) {
		return NULL;
	}

	parsed_args->addr = token_container->array[1].body.hex.value;
	parsed_args->value = token_container->array[2].body.dec.value;
	arg->run = dbg_std_executor_memset;
	return arg;
}

/************************************************************************************
 * quit コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType quit_string = {
		.len = 4,
		.str = { 'q', 'u', 'i', 't', '\0' },
};
static const TokenStringType quit_string_short = {
		.len = 1,
		.str = { 'q', '\0' },
};
DbgCmdExecutorType *dbg_parse_quit(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	if (token_container->num != 1) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &quit_string) == TRUE) ||
			(token_strcmp(&token_container->array[0].body.str, &quit_string_short) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_QUIT;
		arg->run = dbg_std_executor_quit;
		return arg;
	}
	return NULL;
}

/************************************************************************************
 * list コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType list_string = {
		.len = 4,
		.str = { 'l', 'i', 's', 't', '\0' },
};
static const TokenStringType list_string_short = {
		.len = 1,
		.str = { 'l', '\0' },
};
DbgCmdExecutorType *dbg_parse_list(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	if (token_container->num != 1) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &list_string) == TRUE) ||
			(token_strcmp(&token_container->array[0].body.str, &list_string_short) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_LIST;
		arg->run = dbg_std_executor_list;
		return arg;
	}
	return NULL;
}
/************************************************************************************
 * exit コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType exit_string = {
		.len = 4,
		.str = { 'e', 'x', 'i', 't', '\0' },
};
DbgCmdExecutorType *dbg_parse_exit(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	if (token_container->num != 1) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &exit_string) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_EXIT;
		arg->run = dbg_std_executor_exit;
		return arg;
	}
	return NULL;
}


/************************************************************************************
 * serial コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType serial_string = {
		.len = 6,
		.str = { 's', 'e', 'r', 'i', 'a', 'l', '\0' },
};
static const TokenStringType serial_string_short = {
		.len = 1,
		.str = { 'S', '\0' },
};
DbgCmdExecutorType *dbg_parse_serialin(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	DbgCmdExecutorSerialInType *parsed_args = (DbgCmdExecutorSerialInType *)arg->parsed_args;

	if (token_container->num < 3) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}
	if (token_container->array[1].type != TOKEN_TYPE_VALUE_DEC) {
		return NULL;
	}


	if ((token_strcmp(&token_container->array[0].body.str, &serial_string) == TRUE) ||
			(token_strcmp(&token_container->array[0].body.str, &serial_string_short) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_SERIALIN;
		arg->run = dbg_std_executor_serialin;
		parsed_args->channel = token_container->array[1].body.dec.value;
		(void)token_split_merge(token_container, 2, &parsed_args->input);
		//printf("%s\n", (char*)parsed_args->input.str);
		return arg;
	}
	return NULL;
}



/************************************************************************************
 * info cpu コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType info_cpu_string = {
		.len = 3,
		.str = { 'c', 'p', 'u', '\0' },
};

DbgCmdExecutorType *dbg_parse_info_cpu(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	if (token_container->num != 1) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &info_cpu_string) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_INFO_CPU;
		arg->run = dbg_std_executor_info_cpu;
		return arg;
	}
	return NULL;
}

/************************************************************************************
 * func trace コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType func_trace_string = {
		.len = 2,
		.str = { 'f', 't', '\0' },
};
extern DbgCmdExecutorType *dbg_parse_func_trace(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	DbgCmdExecutorFuncTraceType *parsed_args = (DbgCmdExecutorFuncTraceType *)arg->parsed_args;

	if (token_container->num != 2) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}
	if (token_container->array[1].type != TOKEN_TYPE_VALUE_DEC) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &func_trace_string) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_FUNC_TRACE;
		parsed_args->bt_number = token_container->array[1].body.dec.value;
		arg->run = dbg_std_executor_func_trace;
		return arg;
	}
	return NULL;
}

/************************************************************************************
 * data access info コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType data_access_info_string = {
		.len = 6,
		.str = { 'a', 'c', 'c', 'e', 's', 's', '\0' },
};
extern DbgCmdExecutorType *dbg_parse_data_access_info(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	DbgCmdExecutorDataAccessInfoType *parsed_args = (DbgCmdExecutorDataAccessInfoType *)arg->parsed_args;

	if (token_container->num != 2) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}
	if (token_container->array[1].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &data_access_info_string) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_DATA_ACCESS_INFO;
		parsed_args->symbol = token_container->array[1].body.str;
		arg->run = dbg_std_executor_data_access_info;
		return arg;
	}
	return NULL;
}


/************************************************************************************
 * back trace コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType back_trace_string = {
		.len = 2,
		.str = { 'b', 't', '\0' },
};
extern DbgCmdExecutorType *dbg_parse_back_trace(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	if (token_container->num != 1) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &back_trace_string) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_BACK_TRACE;
		arg->run = dbg_std_executor_back_trace;
		return arg;
	}
	return NULL;
}

/************************************************************************************
 * profile コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType prof_string = {
		.len = 4,
		.str = { 'p', 'r', 'o', 'f', '\0' },
};
DbgCmdExecutorType *dbg_parse_profile(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	if (token_container->num != 1) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &prof_string) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_PROFILE;
		arg->run = dbg_std_executor_profile;
		return arg;
	}
	return NULL;
}

/************************************************************************************
 * help コマンド
 *
 *
 ***********************************************************************************/
static const TokenStringType help_string = {
		.len = 4,
		.str = { 'h', 'e', 'l', 'p', '\0' },
};
static const DbgCmdHelpType help_list = {
	.cmd_num = DBG_CMD_STD_ID_NUM,
	.cmd = {
			{
					.name = &break_string,
					.name_shortcut = &break_string_short,
					.opt_num = 2,
					.opts = {
							{
									.semantics = "break {<addr(hex)>|<funcname>}",
									.description = "set a break point. Break points are shown using 'info break' command.",
							},
							{
									.semantics = "break <file> <lineno>",
									.description = "set a break point on the {<file>, <lineno>}. Break points are shown using 'info break' command.",
							},
					},
			},
			{
					.name = &delete_string,
					.name_shortcut = &delete_string_short,
					.opt_num = 2,
					.opts = {
							{
									.semantics = "delete",
									.description = "delete all break points",
							},
							{
									.semantics = "delete <break_no>",
									.description = "delete the break point of <break_no>",
							},
					},
			},
			{
					.name = &watch_string,
					.name_shortcut = &watch_string_short,
					.opt_num = 3,
					.opts = {
							{
									.semantics = "watch {r|w|rw} {<addr(hex)> size(dec)|<variable_name>}",
									.description = "set a watch point. Watch points are shown using 'info watch' command.",
							},
							{
									.semantics = "watch d",
									.description = "delete all watch points",
							},
							{
									.semantics = "watch d <watch_no>",
									.description = "delete the watch point of <watch_no>",
							},
					},
			},

			{
					.name = &cont_string,
					.name_shortcut = &cont_string_short,
					.opt_num = 2,
					.opts = {
							{
									.semantics = "cont",
									.description = "continue program",
							},
							{
									.semantics = "cont <clocks>",
									.description = "continue program until cpu has elapsed <clocks> times",
							},
					},
			},
			{
					.name = &intr_string,
					.name_shortcut = &intr_string_short,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "intr <intno>",
									.description = "generate an interruption of <intno>",
							},
					},
			},
			{
					.name = &elaps_string,
					.name_shortcut = &elaps_string_short,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "elaps",
									.description = "show elapsed cpu clocks",
							},
					},
			},
			{
					.name = &next_string,
					.name_shortcut = &next_string_short,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "next",
									.description = "step forward",
							},
					},
			},
			{
					.name = &return_string,
					.name_shortcut = &return_string_short,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "return",
									.description = "return from the current function",
							},
					},
			},
			{
					.name = &view_string,
					.name_shortcut = &view_string_short,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "view",
									.description = "select the logging mode",
							},
					},
			},
			{
					.name = &print_string,
					.name_shortcut = &print_string_short,
					.opt_num = 2,
					.opts = {
							{
									.semantics = "print <variable_name>",
									.description = "show memory information of the <variable_name>",
							},
							{
									.semantics = "print <addr(hex)> <size>",
									.description = "show memory information from <addr> to (<addr> + <size>)",
							},
					},
			},
			{
					.name = &memset_string,
					.name_shortcut = &memset_string_short,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "set <addr(hex)> <value>",
									.description = "set <value> on memory <addr> by byte",
							},
					},
			},
			{
					.name = &quit_string,
					.name_shortcut = &quit_string_short,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "quit",
									.description = "quit form CPU mode",
							},
					},
			},
			{
					.name = &exit_string,
					.name_shortcut = NULL,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "exit",
									.description = "exit from this program",
							},
					},
			},
			{
					.name = &serial_string,
					.name_shortcut = &serial_string_short,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "serial <channel> <input_data(string)>",
									.description = "set <input_data> on the serial(<channel>) as an input",
							},
					},
			},
			{
					.name = &info_cpu_string,
					.name_shortcut = NULL,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "cpu",
									.description = "show cpu registers",
							},
					},
			},
			{
					.name = &func_trace_string,
					.name_shortcut = NULL,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "ft <number>",
									.description = "show function trace log(show size=<number>)",
							},
					},
			},
			{
					.name = &data_access_info_string,
					.name_shortcut = NULL,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "access <variable_name>",
									.description = "show variable access functions",
							},
					},
			},
			{
					.name = &back_trace_string,
					.name_shortcut = NULL,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "bt",
									.description = "show back trace result",
							},
					},
			},
			{
					.name = &prof_string,
					.name_shortcut = NULL,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "profile",
									.description = "show profile result",
							},
					},
			},
			{
					.name = &list_string,
					.name_shortcut = &list_string_short,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "list",
									.description = "show the program code where the pc is stopped.",
							},
					},
			},
			{
					.name = &help_string,
					.name_shortcut = NULL,
					.opt_num = 1,
					.opts = {
							{
									.semantics = "help",
									.description = "show all commands",
							},
					},
			},
	},
};

DbgCmdExecutorType *dbg_parse_help(DbgCmdExecutorType *arg, const TokenContainerType *token_container)
{
	DbgCmdExecutorHelpType *parsed_args = (DbgCmdExecutorHelpType *)arg->parsed_args;
	if (token_container->num != 1) {
		return NULL;
	}

	if (token_container->array[0].type != TOKEN_TYPE_STRING) {
		return NULL;
	}

	if ((token_strcmp(&token_container->array[0].body.str, &help_string) == TRUE)) {
		arg->std_id = DBG_CMD_STD_ID_HELP;
		parsed_args->arg = &help_list;
		arg->run = dbg_std_executor_help;
		return arg;
	}
	return NULL;
}
