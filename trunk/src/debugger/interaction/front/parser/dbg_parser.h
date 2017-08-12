#ifndef _DBG_PARSER_H_
#define _DBG_PARSER_H_

#include "dbg_executor.h"

typedef enum {
	DBG_MODE_DEBUG,
	DBG_MODE_CPU,
} DbgModeType;
extern DbgCmdExecutorType *dbg_parse(uint8 *str, uint32 len);


#endif /* _DBG_PARSER_H_ */
