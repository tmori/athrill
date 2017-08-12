#include "elf_dwarf_line.h"
#include "elf_section.h"
#include "elf_dwarf_util.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#if 0
#define DBG_PRINTF(arg)	printf arg
#else
#define DBG_PRINTF(arg)
#endif

static void parse_opcode(uint8 *opcode, ElfDwarfLineParsedOpCodeType *op);
static ElfDwarfLineType *elf_dwarf_line = NULL;

ElfDwarfLineType *elf_dwarf_line_get_ElfDwarfLine(void)
{
	return elf_dwarf_line;
}
void elf_dwarf_line_init_ElfDwarfLineStateMachineRegister(ElfDwarfLineStateMachineRegisterType *machine, ElfDwarfLineEntryHeaderType *hdr)
{
	machine->address = 0;
	machine->file = 1;
	machine->line = 1;
	machine->column = 0;
	machine->is_stmt = hdr->default_is_stmt;
	machine->basic_block = FALSE;
	machine->end_sequence = FALSE;
	return;
}


static uint32 parse_entry_header(ElfDwarfLineEntryHeaderType *header, uint8 *section_data);

static uint32 parse_entry_header(ElfDwarfLineEntryHeaderType *header, uint8 *section_data)
{
	int i;
	uint32 off = 0;

	header->total_length = elf_get_data32(section_data, off);
	off += 4;
	header->version = elf_get_data16(section_data, off);
	off += 2;
	header->prologue_length = elf_get_data32(section_data, off);
	off += 4;
	header->minimum_instruction_length = elf_get_data8(section_data, off);
	off += 1;
	header->default_is_stmt = elf_get_data8(section_data, off);
	off += 1;
	header->line_base = elf_get_data8(section_data, off);
	off += 1;
	header->line_range = elf_get_data8(section_data, off);
	off += 1;
	header->opcode_base = elf_get_data8(section_data, off);
	off += 1;

	DBG_PRINTF(("total_length=%u\n", header->total_length));
	DBG_PRINTF(("version=%u\n", header->version));
	DBG_PRINTF(("prologue_length=%u\n", header->prologue_length));
	DBG_PRINTF(("minimum_instruction_length=%u\n", header->minimum_instruction_length));
	DBG_PRINTF(("default_is_stmt=%u\n", header->default_is_stmt));
	DBG_PRINTF(("line_base=%d\n", header->line_base));
	DBG_PRINTF(("line_range=%u\n", header->line_range));
	DBG_PRINTF(("opcode_base=%u\n", header->opcode_base));

	for (i = 0; i < header->opcode_base - 1; i++) {
		uint8 standard_opcode_length = elf_get_data8(section_data, off);
		header->standard_opcode_lengths[i] = standard_opcode_length;
		DBG_PRINTF(("opcode_length[%d]=%u\n", i + 1, standard_opcode_length));
		off++;
	}

	/*
	 * The Directory Table:
	 */
	int len;
	char *dir;
	do {
		dir = (char*)&section_data[off];
		len = strlen(dir);
		off += len + 1;
		if (len > 0) {
			DBG_PRINTF(("dir=%s, len=%u\n", dir, len));
			elf_array_add_entry(header->include_directories, dir);
		}
	} while (len > 0);

	/*
	 *  The File Name Table:
	 */
	char *filename;
	uint32 size;
	do {
		filename = (char*)&section_data[off];
		len = strlen(filename);
		off += len + 1;

		if (len > 0) {
			ElfDwarfLineEntryHeaderFileType *obj = elf_obj_alloc(sizeof(ElfDwarfLineEntryHeaderFileType));
			obj->filename = filename;
			DBG_PRINTF(("filename=%s, len=%u ", filename, len));

			obj->dir = elf_dwarf_decode_uleb128(&section_data[off], &size);
			DBG_PRINTF(("Dir=%u ", obj->dir));
			off += size;

			obj->time = elf_dwarf_decode_uleb128(&section_data[off], &size);
			DBG_PRINTF(("Time=%u ", obj->time));
			off += size;

			obj->size = elf_dwarf_decode_uleb128(&section_data[off], &size);
			DBG_PRINTF(("Size=%u\n", obj->size));
			off += size;

			elf_array_add_entry(header->file_names, obj);
		}
	} while (len > 0);

	return off;
}

Std_ReturnType elf_dwarf_line_load(uint8 *elf_data)
{
	uint8 *section_data;
	Std_ReturnType err;
	uint32 section_size;
	uint32 current_size = 0;
	uint32 hdr_size;

	err = elf_section_get_dwarf_line(elf_data, &section_data, &section_size);
	if (err != STD_E_OK) {
		return err;
	}
	DBG_PRINTF(("**section_size=%u\n", section_size));

	elf_dwarf_line = elf_dwarf_line_alloc_empty_ElfDwarfLine();
	while (current_size < section_size) {
		DBG_PRINTF(("**current_size=0x%x******\n", current_size));
		/*
		 * header
		 */
		ElfDwarfLineEntryType *entry = elf_dwarf_line_alloc_empty_ElfDwarfLineEntry();
		entry->header = elf_dwarf_line_alloc_empty_ElfDwarfLineEntryHeader();
		hdr_size = parse_entry_header(entry->header, &section_data[current_size]);

		/*
		 * 6.2.5 The Statement Program
		 */
		uint32 off = hdr_size;
		while (off < (entry->header->total_length + 4)) {
			ElfDwarfLineParsedOpCodeType *op = elf_dwarf_line_alloc_empty_ElfDwarfLineParsedOpCode();
			op->hdr = entry->header;
			DBG_PRINTF(("opcode[0x%x]=%u ", current_size + off, section_data[current_size + off] - op->hdr->opcode_base));
			parse_opcode(&section_data[current_size + off], op);
			off += op->size;
			elf_array_add_entry(entry->ops, op);
		}
		current_size += off;
		elf_array_add_entry(elf_dwarf_line->entries, entry);
	}
	DBG_PRINTF(("END\n"));
	return STD_E_OK;
}
static void set_ExtDefineFile(uint8 *opcode, ElfDwarfLineParsedOpCodeType *op)
{
	int len = strlen((char*)opcode);
	int off = len + 1;
	uint32 size;

	/*
	 * The first is a null terminated string containing a source file name
	 */
	op->args.extDefineFile.file = (char*)opcode;
	/*
	 * The second is an unsigned LEB128 number representing the directory
	 * index of the directory in which the file was found.
	 */
	op->args.extDefineFile.dir = elf_dwarf_decode_uleb128(&opcode[off], &size);
	off += size;
	/*
	 * The third is an unsigned LEB128 number representing the time
	 * of last modification of the file.
	 */
	op->args.extDefineFile.time = elf_dwarf_decode_uleb128(&opcode[off], &size);
	off += size;
	/*
	 * The fourth is an unsigned LEB128 number representing the length
	 * in bytes of the file.
	 */
	op->args.extDefineFile.size = elf_dwarf_decode_uleb128(&opcode[off], &size);

	return;
}
static void set_ExtendedOp(uint8 *opcode, ElfDwarfLineParsedOpCodeType *op)
{
	uint32 size;
	switch (opcode[0]) {
	case DW_LNE_end_sequence:
		op->subtype = DW_LNE_end_sequence;
		DBG_PRINTF(("DW_LNE_end_sequence\n"));
		break;
	case DW_LNE_set_address:
		op->subtype = DW_LNE_set_address;
		op->args.extSetAddress.addr = elf_get_data32(&opcode[1], 0);
		DBG_PRINTF(("DW_LNE_set_address:addr=0x%x\n", op->args.extSetAddress.addr));
		break;
	case DW_LINE_define_file:
		op->subtype = DW_LINE_define_file;
		set_ExtDefineFile(&opcode[1], op);
		DBG_PRINTF(("DW_LINE_define_file\n"));
		break;
	case DW_LNE_set_discriminator:
		op->subtype = DW_LNE_set_discriminator;
		op->args.extSetDescriminator.discriminator = elf_dwarf_decode_uleb128(&opcode[1], &size);
		DBG_PRINTF(("DW_LNE_set_discriminator\n"));
		break;
	default:
		op->subtype = DW_LINE_unknown;
		break;
	}
	return;
}
static void set_StandardOp(uint8 *opcode, ElfDwarfLineParsedOpCodeType *op)
{
	uint32 size = 0;

	switch (op->subtype) {
	case DW_LNS_copy:
		DBG_PRINTF(("DW_LNS_copy\n"));
		break;
	case DW_LNS_advance_pc:
		op->args.stdAdvancePc.advance_size = elf_dwarf_decode_uleb128(opcode, &size) * ((uint32)op->hdr->minimum_instruction_length);
		DBG_PRINTF(("DW_LNS_advance_pc:advance_size=0x%x\n", op->args.stdAdvancePc.advance_size));
		break;
	case DW_LNS_advance_line:
		op->args.stdAdvanceLine.advance_line = elf_dwarf_decode_sleb128(opcode, &size);
		DBG_PRINTF(("DW_LNS_advance_line:advance_line=%d\n", op->args.stdAdvanceLine.advance_line));
		break;
	case DW_LNS_set_file:
		op->args.stdSetFile.file = elf_dwarf_decode_uleb128(opcode, &size);
		DBG_PRINTF(("DW_LNS_set_file:file=%d\n", op->args.stdSetFile.file));
		break;
	case DW_LNS_set_column:
		op->args.stdSetColumn.column = elf_dwarf_decode_uleb128(opcode, &size);
		DBG_PRINTF(("DW_LNS_set_column:ficolumnle=%d\n", op->args.stdSetColumn.column));
		break;
	case DW_LNS_negate_stmt:
		DBG_PRINTF(("DW_LNS_negate_stmt\n"));
		break;
	case DW_LNS_set_basic_block:
		DBG_PRINTF(("DW_LNS_set_basic_block\n"));
		break;
	case DW_LNS_const_add_pc:
		{
			uint32 adjusted_opcode = (255 - op->hdr->opcode_base);
			op->args.stdConstAddPc.const_add_pc = (adjusted_opcode / op->hdr->line_range) * op->hdr->minimum_instruction_length;
			size = 0;
		}
		DBG_PRINTF(("DW_LNS_const_add_pc\n"));
		break;
	case DW_LNS_fixed_advance_pc:
		op->args.stdFixedAdvancePc.fixed_advance_pc = elf_get_data16(opcode, 0) * op->hdr->minimum_instruction_length;
		DBG_PRINTF(("DW_LNS_fixed_advance_pc:fixed_advance_pc=%d\n", op->args.stdFixedAdvancePc.fixed_advance_pc));
		size = 2;
		break;
	default:
		break;
	}
	op->size = size + 1;
	return;
}
static void set_SpecialOp(uint8 *opcode, ElfDwarfLineParsedOpCodeType *op)
{
	sint32 adjusted_opcode;

	adjusted_opcode = ((opcode[0]) - (op->hdr->opcode_base));
	//printf("adjusted_opcode=%d ", adjusted_opcode);

	op->size = 1;
	op->args.special.advance_addr = ((uint32)(adjusted_opcode / op->hdr->line_range)) * ((uint32)op->hdr->minimum_instruction_length);
	op->args.special.advance_line = op->hdr->line_base + (adjusted_opcode % op->hdr->line_range);
	DBG_PRINTF(("set_SpecialOp:advance_addr=%d ", op->args.special.advance_addr));
	DBG_PRINTF(("advance_line=%d\n", op->args.special.advance_line));
	return;
}

static void parse_opcode(uint8 *opcode, ElfDwarfLineParsedOpCodeType *op)
{
	if (opcode[0] == OPTYPE_EXTENDED) {
		op->type = OPTYPE_EXTENDED;
		op->size = opcode[1] + 2;
		set_ExtendedOp(&opcode[2], op);
	}
	else if ((opcode[0] >= DW_LNS_copy) && (opcode[0] <= DW_LNS_fixed_advance_pc)) {
		op->type = OPTYPE_STANDARD;
		op->subtype = opcode[0];
		set_StandardOp(&opcode[1], op);
	}
	else {
		op->type = OPTYPE_SPECIAL;
		set_SpecialOp(&opcode[0], op);
	}
}


ElfDwarfLineType *elf_dwarf_line_alloc_empty_ElfDwarfLine(void)
{
	ElfDwarfLineType *obj;
	obj = elf_obj_alloc(sizeof(ElfDwarfLineType));
	obj->entries = elf_array_alloc();
	return obj;
}

ElfDwarfLineEntryHeaderType *elf_dwarf_line_alloc_empty_ElfDwarfLineEntryHeader(void)
{
	ElfDwarfLineEntryHeaderType *obj = elf_obj_alloc(sizeof(ElfDwarfLineEntryHeaderType));
	obj->file_names = elf_array_alloc();
	obj->include_directories = elf_array_alloc();
	return obj;
}

ElfDwarfLineParsedOpCodeType *elf_dwarf_line_alloc_empty_ElfDwarfLineParsedOpCode(void)
{
	ElfDwarfLineParsedOpCodeType *obj;

	obj = elf_obj_alloc(sizeof(ElfDwarfLineParsedOpCodeType));

	return obj;
}

ElfDwarfLineEntryType *elf_dwarf_line_alloc_empty_ElfDwarfLineEntry(void)
{
	ElfDwarfLineEntryType *obj;

	obj = elf_obj_alloc(sizeof(ElfDwarfLineEntryType));
	obj->header = elf_dwarf_line_alloc_empty_ElfDwarfLineEntryHeader();
	obj->ops = elf_array_alloc();

	return obj;
}

