#ifndef _ELF_DWARF_UTIL_H_
#define _ELF_DWARF_UTIL_H_

#include "std_types.h"

extern uint32 elf_dwarf_decode_uleb128(uint8 *ptr, uint32 *size);

#define BYTESLEBMAX 10
extern sint32 elf_dwarf_decode_sleb128(uint8 *ptr, uint32 *size);

/*
 * reallocと同じ
 * 異常発生時はASSERTで失敗する．
 */
#define ELF_POINTER_ARRAY_SIZE_UNIT		16
typedef struct {
	uint32 max_array_size;
	uint32 current_array_size;
	void **data;
} ElfPointerArrayType;
extern ElfPointerArrayType *elf_array_alloc(void);
extern void elf_array_realloc(ElfPointerArrayType *array);
extern void elf_array_add_entry(ElfPointerArrayType *array, void *entry);

extern void *elf_obj_alloc(uint32 size);

#define DWARF_ARRAY_SIZE_UNIT32		16
typedef struct {
	uint32 max_array_size;
	uint32 current_array_size;
	uint32 *data;
} DwarfUint32ArrayType;
extern DwarfUint32ArrayType *dwarf_uint32_array_alloc(void);
extern void dwarf_uint32_array_realloc(DwarfUint32ArrayType *array);
extern void dwarf_uint32_array_add_entry(DwarfUint32ArrayType *array, uint32 entry);


#endif /* _ELF_DWARF_UTIL_H_ */
