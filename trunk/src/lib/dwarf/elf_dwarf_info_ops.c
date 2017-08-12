#include "elf_dwarf_info_ops.h"
#include "elf_dwarf_util.h"
#include "assert.h"
#include <stdio.h>

static ElfDwarfStackType elf_dwaf_info_ops;

void elf_dwarf_info_ops_init(void)
{
	elf_dwaf_info_ops.stack_size = 0;
	elf_dwaf_info_ops.stack = dwarf_uint32_array_alloc();
	return;
}
uint32 elf_dwarf_info_ops_pop(void)
{
	if (elf_dwaf_info_ops.stack_size > 0) {
		elf_dwaf_info_ops.stack_size--;
		return elf_dwaf_info_ops.stack->data[elf_dwaf_info_ops.stack_size];
	}
	return 0;
}
void elf_dwarf_info_ops_push(uint32 data)
{
	if (elf_dwaf_info_ops.stack->current_array_size > elf_dwaf_info_ops.stack_size) {
		elf_dwaf_info_ops.stack->data[elf_dwaf_info_ops.stack_size] = data;
		elf_dwaf_info_ops.stack_size++;
	}
	else {
		dwarf_uint32_array_add_entry(elf_dwaf_info_ops.stack, data);
		elf_dwaf_info_ops.stack_size++;
	}
	return;
}

void elf_dwarf_info_ops_DW_OP_plus_uconst(uint8 *addr)
{
	uint32 size;
	uint32 data1 = elf_dwarf_info_ops_pop();
	uint32 data2 = elf_dwarf_decode_uleb128(addr, &size);
	//printf("data1=%u data2=%u\n", data1, data2);
	elf_dwarf_info_ops_push(data1 + data2);
	return;
}
void elf_dwarf_info_ops_DW_OP(ElfDwarfInfoOpsType optype, uint8 *data)
{
	switch (optype) {
	case DW_OP_plus_uconst:
		elf_dwarf_info_ops_DW_OP_plus_uconst(data);
		break;
	default:
		printf("not supported optype = 0x%x\n", optype);
		ASSERT(0);
		break;
	}
}
