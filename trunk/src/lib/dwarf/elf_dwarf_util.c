#include "elf_dwarf_util.h"
#include "assert.h"
#include <string.h>
#include <stdlib.h>

uint32 elf_dwarf_decode_uleb128(uint8 *ptr, uint32 *size)
{
	uint8 i;
	uint8 byte;
	uint32 data = 0;
	uint32 shift = 0;

    for (i = 0; i < 4; i++) {
        byte = ptr[i];
        data |= ((uint32)(byte & 0x7f)) << shift;
        shift += 7;
        if ((byte & 0x80) == 0) {
            break;
        }
    }
    *size = i + 1;
	return data;
}

sint32 elf_dwarf_decode_sleb128(uint8 *ptr, uint32 *size)
{
	uint8 i;
	uint8 sign = 0;
	uint8 byte;
	sint32 data = 0;
	sint32 shift = 0;

    for (i = 0; i < 5; i++) {
        byte = ptr[i];
        sign = byte & 0x40;
        data |= ((sint32)(byte & 0x7f)) << shift;
        shift += 7;
        if ((byte & 0x80) == 0) {
            break;
        }
    }
    *size = i + 1;

    if (sign) {
    	data |=  (- (1 << shift));
    }

	return data;
}

void elf_array_realloc(ElfPointerArrayType *array)
{
	void *new_ptr;

	if (array->current_array_size >= array->max_array_size) {
		array->max_array_size += ELF_POINTER_ARRAY_SIZE_UNIT;
		new_ptr = realloc(array->data, (array->max_array_size * sizeof(void*)));
		ASSERT(new_ptr != NULL);
		array->data = new_ptr;
	}
	return;
}
extern void elf_array_add_entry(ElfPointerArrayType *array, void *entry)
{
	if (array->current_array_size >= array->max_array_size) {
		elf_array_realloc(array);
	}
	array->data[array->current_array_size] = entry;
	array->current_array_size++;
	return;
}
ElfPointerArrayType *elf_array_alloc(void)
{
	ElfPointerArrayType *array = malloc(sizeof(ElfPointerArrayType));
	ASSERT(array != NULL);
	array->current_array_size = 0;
	array->max_array_size = 0;
	array->data = NULL;
	elf_array_realloc(array);
	return array;
}

void *elf_obj_alloc(uint32 size)
{
	void *obj;
	obj = malloc(size);
	ASSERT(obj != NULL);
	memset(obj, 0, size);
	return obj;
}

DwarfUint32ArrayType *dwarf_uint32_array_alloc(void)
{
	DwarfUint32ArrayType *array = malloc(sizeof(DwarfUint32ArrayType));
	ASSERT(array != NULL);
	array->current_array_size = 0;
	array->max_array_size = 0;
	array->data = NULL;
	dwarf_uint32_array_realloc(array);
	return array;
}
void dwarf_uint32_array_realloc(DwarfUint32ArrayType *array)
{
	void *new_ptr;

	if (array->current_array_size >= array->max_array_size) {
		array->max_array_size += DWARF_ARRAY_SIZE_UNIT32;
		new_ptr = realloc(array->data, (array->max_array_size * sizeof(uint32)));
		ASSERT(new_ptr != NULL);
		array->data = new_ptr;
	}
	return;
}
void dwarf_uint32_array_add_entry(DwarfUint32ArrayType *array, uint32 entry)
{
	if (array->current_array_size >= array->max_array_size) {
		dwarf_uint32_array_realloc(array);
	}
	array->data[array->current_array_size] = entry;
	array->current_array_size++;
	return;
}

