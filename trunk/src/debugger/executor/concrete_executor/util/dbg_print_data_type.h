#ifndef _DBG_PRINT_DATA_TYPE_H_
#define _DBG_PRINT_DATA_TYPE_H_

#include "std_types.h"

extern bool print_variable_with_data_type(char *variable_name, uint32 vaddr, uint8 *top_addr, uint32 size);
extern bool print_addr_with_data_type(uint32 vaddr, uint8 *top_addr, char* dataType, char* dataTypeName);

#endif /* _DBG_PRINT_DATA_TYPE_H_ */
