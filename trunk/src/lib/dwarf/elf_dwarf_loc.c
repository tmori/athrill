#include "elf_dwarf_loc.h"
#include "elf_section.h"
#include "elf_dwarf_info_ops.h"
#include "cpu.h"
#include "assert.h"
#include <string.h>

#if 0
#define DBG_PRINTF(arg)	printf arg
#else
#define DBG_PRINTF(arg)
#endif
static ElfDwarfLocEntryType *elf_dwarf_loc_alloc_ElfDwarfLocEntry(uint8 *raw_data, uint32 off);

static ElfPointerArrayType *elf_dwarf_loc_lists;

Std_ReturnType elf_dwarf_loc_load(uint8 *elf_data)
{
	uint8 *section_data;
	uint32 section_size;
	Std_ReturnType err;
	uint32 current_size = 0;
	uint32 off = 0;
	ElfDwarfLocEntryType *entry;
	ElfDwarfLocEntryType *prev_entry;

	err = elf_section_get(elf_data, SECTION_DWARF_LOC_NAME, &section_data, &section_size);
	if (err != STD_E_OK) {
		return err;
	}
	DBG_PRINTF(("**loc section_size=%u\n", section_size));
	elf_dwarf_loc_lists = elf_array_alloc();
	ASSERT(elf_dwarf_loc_lists != NULL);

	prev_entry = NULL;
	while (current_size < section_size) {
		DBG_PRINTF(("**loc current_size=0x%x******\n", current_size));

		entry = elf_dwarf_loc_alloc_ElfDwarfLocEntry(&section_data[off], off);
		elf_array_add_entry(elf_dwarf_loc_lists, entry);
		if (prev_entry != NULL) {
			if ((prev_entry->relativeAddress0 == 0) && (prev_entry->relativeAddress1 == 0)) {
				prev_entry->next = NULL;
			}
			else {
				prev_entry->next = entry;
			}
		}

		off += entry->entrySize;
		current_size += entry->entrySize;

		prev_entry = entry;
	}
	DBG_PRINTF(("END\n"));
	return STD_E_OK;
}

static ElfDwarfLocEntryType *elf_dwarf_loc_alloc_ElfDwarfLocEntry(uint8 *raw_data, uint32 off)
{
	int i;
	ElfDwarfLocEntryType *obj;

	obj = elf_obj_alloc(sizeof(ElfDwarfLocEntryType));
	obj->entryOffset = off;
	obj->relativeAddress0 = *(uint32*)&raw_data[0];
	obj->relativeAddress1 = *(uint32*)&raw_data[4];

	DBG_PRINTF(("off=0x%x\n", obj->entryOffset));
	DBG_PRINTF(("addr1=0x%x\n", obj->relativeAddress0));
	DBG_PRINTF(("addr2=0x%x\n", obj->relativeAddress1));
	DBG_PRINTF(("range=0x%x\n", obj->relativeAddress1 - obj->relativeAddress0));
	if ((obj->relativeAddress0 == 0) && (obj->relativeAddress1 == 0)) {
		/*
		 * End of list
		 */
		obj->block_length = 0;
		obj->blocks = NULL;
		obj->entrySize = 8;
		return obj;
	}
	obj->block_length = *(uint16*)&raw_data[8];
	obj->blocks = (uint8*)&raw_data[10];
	DBG_PRINTF(("length=%d\n", obj->block_length));
	for (i = 0; i < obj->block_length; i++) {
		DBG_PRINTF((" block[%d]=0x%x\n", i, obj->blocks[i]));
	}
	obj->entrySize = 10 + obj->block_length;

	return obj;
}
static ElfDwarfLocEntryType *search_location_entry(uint32 off)
{
	int i;

	for (i = 0; elf_dwarf_loc_lists->current_array_size; i++) {
		ElfDwarfLocEntryType *entry = (ElfDwarfLocEntryType*)(elf_dwarf_loc_lists->data[i]);
		if (entry->entryOffset == off) {
			return entry;
		}
	}
	return NULL;

}

static void elf_dwarf_loc_compile_fbreg(DwarfDataSubprogramType *subprogram, DwarfLocalVariableType *localVariable, uint32 pc, uint32 funcaddr, uint32 *vaddr)
{
	uint32 size;
	ElfDwarfLocEntryType *loc;
	int value1;
	int value2;
	int regInx;
	int regData;
	uint32 off = funcaddr;

	loc = search_location_entry(subprogram->frame_loc_offset);
	ASSERT(loc != NULL);

	value1 = elf_dwarf_decode_sleb128(&localVariable->DW_AT_location->encoded.op.ops[1], &size);
	printf("value1=%d\n", value1);
	printf("pc=0x%x funcaddr=0x%x\n", pc, funcaddr);

	/*
	 * search loc on pc
	 */
	while (loc != NULL) {
		uint32 range = (loc->relativeAddress1 - loc->relativeAddress0);
		uint32 saddr = off;
		uint32 eaddr = off + range;
		//printf("relativeAddress0=0x%x relativeAddress1=0x%x\n", loc->relativeAddress0, loc->relativeAddress1);
		//printf("off=0x%x range=0x%x saddr=0x%x eaddr=0x%x\n", off, range, saddr, eaddr);
		if ((pc >= saddr) && (pc < eaddr)) {
			break;
		}
		off += range;
		loc = loc->next;
	}
	ASSERT(loc != NULL);

	if ((loc->blocks[0] >= DW_OP_breg0) && (loc->blocks[0] <= DW_OP_breg31)) {
		value2 = elf_dwarf_decode_sleb128(&loc->blocks[1], &size);
		regInx = loc->blocks[0] - DW_OP_breg0;
		regData = (int)cpu_get_current_core_register(regInx);
		regData = regData + value1 + value2;
		*vaddr = regData;
		printf("vaddr=0x%x\n", regData);
	}
	else {
		printf("not supported: elf_dwarf_loc_compile_fbreg:op=0x%x\n", loc->blocks[0]);
		ASSERT(0);
	}

	return;
}


bool printLocalValueV850(DwarfDataSubprogramType *subprogram, DwarfLocalVariableType *localVariable, uint32 pc, uint32 funcaddr, uint32 *vaddr)
{
	if (localVariable->DW_AT_location == NULL) {
		printf("Not Supported: DW_AT_location symbol=%s\n", localVariable->name);
		return FALSE;
	}
	switch (localVariable->DW_AT_location->type) {
	case DW_FORM_block1:
		switch (localVariable->DW_AT_location->encoded.op.ops[0]) {
		case DW_OP_fbreg:
			elf_dwarf_loc_compile_fbreg(subprogram, localVariable, pc, funcaddr, vaddr);
			break;
		default:
			printf("not supported ops[0]=0x%x\n", localVariable->DW_AT_location->encoded.op.ops[0]);
			break;
		}
		break;
	default:
		printf("Not Supported: DW_AT_location form=0x%x\n", localVariable->DW_AT_location->type);
		return FALSE;
	}
	return TRUE;
}
