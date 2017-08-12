#include "elf_dwarf_info.h"
#include "elf_dwarf_info_ops.h"
#include "elf_section.h"
#include "assert.h"
#include <string.h>

#if 0
#include <stdio.h>
#define DBG_PRINTF_DEFINED
#define DBG_PRINTF(arg)	printf arg
static void printCompilationUnitHeader(ElfDwarfCompilationUnitHeaderType *cu);
static void printAll(void);
#else
#define DBG_PRINTF(arg)
#endif


static char *debug_str = NULL;
static ElfPointerArrayType *compilation_unit_headers = NULL;


static ElfDwarfAttributeType *elf_dwarf_alloc_empty_ElfDwarfAttribute(void)
{
	return (ElfDwarfAttributeType *)elf_obj_alloc(sizeof(ElfDwarfAttributeType));
}
static ElfDwarfCompilationUnitHeaderType *elf_dwarf_alloc_empty_ElfDwarfCompilationUnitHeader(void)
{
	ElfDwarfCompilationUnitHeaderType *obj;

	obj = (ElfDwarfCompilationUnitHeaderType *)elf_obj_alloc(sizeof(ElfDwarfCompilationUnitHeaderType));

	obj->dies = elf_array_alloc();
	return obj;
}
static ElfDwarfDieType *elf_dwarf_alloc_empty_ElfDwarfDie(void)
{
	ElfDwarfDieType *obj;

	obj = (ElfDwarfDieType *)elf_obj_alloc(sizeof(ElfDwarfDieType));

	obj->attribute = elf_array_alloc();
	return obj;
}

uint32 elf_dwarf_info_get_value(DwFormType form, ElfDwarfAttributeType *obj, uint32 *size)
{
	uint32 value = -1;
	*size = 0;
	switch (form) {
	case DW_FORM_none:
		break;
	case DW_FORM_addr:
		value = obj->encoded.addr;
		*size = 4;
		break;
	case DW_FORM_data1:
		value = obj->encoded.const8;
		*size = 1;
		break;
	case DW_FORM_data2:
		value = obj->encoded.const16;
		*size = 2;
		break;
	case DW_FORM_data4:
		value = obj->encoded.const32;
		*size = 4;
		break;
	case DW_FORM_ref1:
		value = obj->encoded.ref8;
		*size = 1;
		break;
	case DW_FORM_ref2:
		value = obj->encoded.ref16;
		*size = 2;
		break;
	case DW_FORM_ref4:
		value = obj->encoded.ref32;
		*size = 4;
		break;
	case DW_FORM_sec_offset:
		value = obj->encoded.sec_offset;
		*size = 4;
		break;
	case DW_FORM_sdata:
		value = (uint32)obj->encoded.scont64;
		*size = 4;
		break;
	case DW_FORM_block1:
		//printf("block1:len=%u op[0]=%x, op[1]=%x\n", obj->encoded.op.len,
		//		obj->encoded.op.ops[0], obj->encoded.op.ops[1]);
		elf_dwarf_info_ops_push(0);
		elf_dwarf_info_ops_DW_OP(obj->encoded.op.ops[0], &obj->encoded.op.ops[1]);
		value = elf_dwarf_info_ops_pop();
		break;
	default:
		printf("invalid form=0x%x\n", form);
		ASSERT(0);
		break;
	}
	return value;
}

static ElfDwarfAttributeType *elf_dwarf_info_parse_attr(ElfDwarfCompilationUnitHeaderType *cu, ElfDwarfDieType *die, uint8 *addr, DwFormType form, uint32 *size)
{
	ElfDwarfAttributeType *obj = NULL;

	*size = 0;
	switch (form) {
	case DW_FORM_none:
		break;
	case DW_FORM_addr:
		*size = cu->pointer_size;
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.addr = elf_get_data32(addr, 0);
		obj->typename = "DW_FORM_addr";
		break;
	case DW_FORM_block:
		ASSERT(0);
		break;
	case DW_FORM_block1:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.op.len = elf_get_data8(addr, 0);
		obj->encoded.op.ops = &addr[1];
		*size = obj->encoded.op.len + 1;
		obj->typename = "DW_FORM_block1";
		break;
	case DW_FORM_block2:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.op.len = elf_get_data16(addr, 0);
		obj->encoded.op.ops = &addr[2];
		*size = obj->encoded.op.len + 2;
		obj->typename = "DW_FORM_block2";
		break;
	case DW_FORM_block4:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.op.len = elf_get_data32(addr, 0);
		obj->encoded.op.ops = &addr[4];
		obj->typename = "DW_FORM_block4";
		*size = obj->encoded.op.len + 4;
		break;
	case DW_FORM_data1:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.const8 = elf_get_data8(addr, 0);
		obj->typename = "DW_FORM_data1";
		*size = 1;
		break;
	case DW_FORM_data2:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.const16 = elf_get_data16(addr, 0);
		obj->typename = "DW_FORM_data2";
		*size = 2;
		break;
	case DW_FORM_data4:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.const32 = elf_get_data32(addr, 0);
		obj->typename = "DW_FORM_data4";
		*size = 4;
		break;
	case DW_FORM_data8:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.const64 = elf_get_data64(addr, 0);
		obj->typename = "DW_FORM_data8";
		*size = 8;
		break;
	case DW_FORM_sdata:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.scont64 = elf_dwarf_decode_sleb128(addr, size);
		obj->typename = "DW_FORM_sdata";
		break;
	case DW_FORM_udata:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.const64 = elf_dwarf_decode_uleb128(addr, size);
		obj->typename = "DW_FORM_udata";
		break;
	case DW_FORM_string:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.string = (char*)addr;
		*size = strlen((char*)addr) + 1;
		obj->typename = "DW_FORM_string";
		break;
	case DW_FORM_strp:
	{
		uint32 off = elf_get_data32(addr, 0);
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.string = &debug_str[off];
		obj->typename = "DW_FORM_strp";
		*size = 4;
	}
		break;
	case DW_FORM_flag:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.flag = *addr;
		obj->typename = "DW_FORM_flag";
		*size = 1;
		break;
	case DW_FORM_ref_addr:
		ASSERT(0);
		break;
	case DW_FORM_ref1:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.ref8 = elf_get_data8(addr, 0) + cu->offset;
		obj->typename = "DW_FORM_ref1";
		*size = 1;
		break;
	case DW_FORM_ref2:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.ref16 = elf_get_data16(addr, 0) + cu->offset;
		obj->typename = "DW_FORM_ref2";
		*size = 2;
		break;
	case DW_FORM_ref4:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.ref32 = elf_get_data32(addr, 0) + cu->offset;
		obj->typename = "DW_FORM_ref4";
		*size = 4;
		break;
	case DW_FORM_ref8:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.ref64 = elf_get_data64(addr, 0) + cu->offset;
		obj->typename = "DW_FORM_ref8";
		*size = 8;
		break;
	case DW_FORM_ref_udata:
		ASSERT(0);
		break;
	case DW_FORM_indirect:
		ASSERT(0);
		break;
	case DW_FORM_sec_offset:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.sec_offset = elf_get_data32(addr, 0);
		obj->typename = "DW_FORM_sec_offset";
		*size = 4;
		break;
	case DW_FORM_exprloc:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.op.len = elf_dwarf_decode_uleb128(addr, size);
		obj->encoded.op.ops = &addr[*size];
		obj->typename = "DW_FORM_exprloc";
		*size += obj->encoded.op.len;
		break;
	case DW_FORM_flag_present:
		obj = elf_dwarf_alloc_empty_ElfDwarfAttribute();
		obj->encoded.flag = TRUE;
		obj->typename = "DW_FORM_flag_present";
		*size = 0;
		break;
	case DW_FORM_ref_sig8:
		ASSERT(0);
		break;
	default:
		ASSERT(0);
		break;
	}

	return obj;
}

ElfPointerArrayType *elf_dwarf_info_get(void)
{
	return compilation_unit_headers;
}

Std_ReturnType elf_dwarf_info_load(uint8 *elf_data)
{
	uint8 *section_data;
	Std_ReturnType err;
	uint32 section_size;
	uint32 current_size = 0;
	uint32	header_size;
	ElfDwarfCompilationUnitHeaderType	*cu;
	ElfDwarfAbbrevType *entry;
	ElfDwarfAbbrevType *top;
	uint32 size;
	uint32 entry_size;
	uint8 *addr;
	uint32 code;
	ElfDwarfDieType *die = NULL;
	int i;
	uint32 level;
	ElfDwarfDieType *parent = NULL;

	elf_dwarf_info_ops_init();

	compilation_unit_headers = elf_array_alloc();

	err = elf_section_get(elf_data, SECTION_DWARF_STR_NAME, (uint8**)&debug_str, &section_size);
	if (err != STD_E_OK) {
		return err;
	}

	err = elf_section_get(elf_data, SECTION_DWARF_INFO_NAME, &section_data, &section_size);
	if (err != STD_E_OK) {
		return err;
	}
	DBG_PRINTF(("**section_size=%u\n", section_size));

	header_size = (4 + 2 + 4 + 1);
	while (current_size < section_size) {
		DBG_PRINTF(("**current_size=0x%x ******\n", current_size));

		cu = elf_dwarf_alloc_empty_ElfDwarfCompilationUnitHeader();

		cu->offset = current_size;
		cu->length = elf_get_data32(&section_data[current_size], 0);
		cu->version = elf_get_data16(&section_data[current_size], 4);
		cu->abbrev_offset = elf_get_data32(&section_data[current_size], 6);
		cu->pointer_size = elf_get_data8(&section_data[current_size], 10);

		top = elf_dwarf_abbrev_get(cu->abbrev_offset);
		ASSERT(top != NULL);
		level = 0;
		parent = NULL;

		entry_size = header_size;
		while (entry_size < (cu->length + 4)) {
			code = elf_dwarf_decode_uleb128(&section_data[current_size + entry_size], &size);
			DBG_PRINTF(("<%x>    entry_code=0x%x\n", current_size + entry_size, code));
			entry_size += size;
			if (code == 0x00) {
				level--;
				if (die != NULL) {
					parent = die->parent;
				}
				continue;
			}
			entry = elf_dwarf_abbrev_get_from_code(top, code);
			ASSERT(entry != NULL);
			die = elf_dwarf_alloc_empty_ElfDwarfDie();
			die->abbrev_code = code;
			die->abbrev_info = entry;
			die->offset = current_size + entry_size - size;
			die->level = level;
			if (parent != NULL) {
				die->parent = parent;
				elf_array_add_entry(parent->children, die);
			}
			if (entry->child == TRUE) {
				die->children = elf_array_alloc();
				parent = die;
				level++;
			}

			DBG_PRINTF(("    code=0x%x tag=0x%x\n", entry->code, entry->tag));
			for (i = 0; i < entry->attribute_name->current_array_size; i++) {
				ElfDwarfAttributeType *obj;
				addr = &section_data[current_size + entry_size];
				obj = elf_dwarf_info_parse_attr(cu, die, addr, entry->attribute_form->data[i], &size);

				if (obj != NULL) {
					obj->type = entry->attribute_form->data[i];
					obj->size = size;
					obj->offset = current_size + entry_size;
					elf_array_add_entry(die->attribute, obj);
				}

				DBG_PRINTF(("<%x>    name=0x%x form=0x%x\n",
						current_size + entry_size,
						entry->attribute_name->data[i],
						entry->attribute_form->data[i]));
				entry_size += size;
			}

			elf_array_add_entry(cu->dies, die);
			DBG_PRINTF(("    entry_size=0x%x\n", entry_size));
		}
		ASSERT(entry_size == (cu->length + 4));
		current_size += entry_size;

		elf_array_add_entry(compilation_unit_headers, cu);
	}
	//printAll();
	return STD_E_OK;
}
#ifdef DBG_PRINTF_DEFINED

static void printOps(ElfDwarfAttributeType *obj)
{
	int i;
	printf("size=%u",
			obj->encoded.op.len);
	for (i = 0; i < obj->encoded.op.len; i++) {
		printf(" 0x%x ", obj->encoded.op.ops[i]);
	}
	printf("\n");
	return;
}

static void printAttr(ElfDwarfAttributeType *obj)
{
	switch (obj->type) {
	case DW_FORM_none:
		break;
	case DW_FORM_addr:
		printf("0x%x\n", obj->encoded.addr);
		break;
	case DW_FORM_block:
		ASSERT(0);
		break;
	case DW_FORM_block1:
	case DW_FORM_block2:
	case DW_FORM_block4:
	case DW_FORM_exprloc:
		printOps(obj);
		break;
	case DW_FORM_data1:
		printf("0x%x\n", obj->encoded.const8);
		break;
	case DW_FORM_data2:
		printf("0x%x\n", obj->encoded.const16);
		break;
	case DW_FORM_data4:
		printf("0x%x\n", obj->encoded.const32);
		break;
	case DW_FORM_data8:
		printf("0x%I64x\n", obj->encoded.const64);
		break;
	case DW_FORM_sdata:
		printf("0x%I64x\n", obj->encoded.scont64);
		break;
	case DW_FORM_udata:
		printf("0x%I64x\n", obj->encoded.const64);
		break;
	case DW_FORM_string:
	case DW_FORM_strp:
		printf("%s\n", obj->encoded.string);
		break;
	case DW_FORM_flag:
		if (obj->encoded.flag == TRUE) {
			printf("true\n");
		}
		else {
			printf("false\n");
		}
		break;
	case DW_FORM_ref_addr:
		ASSERT(0);
		break;
	case DW_FORM_ref1:
		printf("0x%x\n", obj->encoded.ref8);
		break;
	case DW_FORM_ref2:
		printf("0x%x\n", obj->encoded.ref16);
		break;
	case DW_FORM_ref4:
		printf("0x%x\n", obj->encoded.ref32);
		break;
	case DW_FORM_ref8:
		printf("0x%I64x\n", obj->encoded.ref64);
		break;
	case DW_FORM_ref_udata:
		ASSERT(0);
		break;
	case DW_FORM_indirect:
		ASSERT(0);
		break;
	case DW_FORM_sec_offset:
		printf("0x%x\n", obj->encoded.sec_offset);
		break;
	case DW_FORM_flag_present:
		printf("true\n");
		break;
	case DW_FORM_ref_sig8:
		ASSERT(0);
		break;
	default:
		ASSERT(0);
		break;
	}
	return;
}

static void printDie(ElfDwarfDieType *die)
{
	int i;
	ElfDwarfAttributeType *obj;
	uint32 parent_offset = 0x0;
	uint32 children = 0;

	if (die->parent != NULL) {
		parent_offset = die->parent->offset;
	}
	if (die->children != NULL) {
		children = die->children->current_array_size;
	}

	printf("<%u><%x>: Abbrev Number: %u (TAG=0x%x) die=0x%x parent=0x%x children=%u\n",
			die->level, die->offset, die->abbrev_code, die->abbrev_info->tag,
			die->offset, parent_offset, children);
	for (i = 0; i < die->attribute->current_array_size; i++) {
		obj = (ElfDwarfAttributeType *)die->attribute->data[i];
		printf("	<%x> AT=0x%x %s(0x%x): ",
				obj->offset, die->abbrev_info->attribute_name->data[i], obj->typename, obj->type);
		printAttr(obj);
	}
	return;
}

static void printCompilationUnitHeader(ElfDwarfCompilationUnitHeaderType *cu)
{
	int i;
	printf("Compilation Unit @ offset 0x%x\n", cu->offset);
	printf(" Length:	0x%x\n", cu->length);
	printf(" Version:	%u\n", cu->version);
	printf(" Abbrev Offset:	0x%x\n", cu->abbrev_offset);
	printf(" Pointer Size:	%u\n", cu->pointer_size);

	for (i = 0; i < cu->dies->current_array_size; i++) {
		printDie(cu->dies->data[i]);
	}
	return;
}

static void printAll(void)
{
	int i;
	for (i = 0; i < compilation_unit_headers->current_array_size; i++) {
		printCompilationUnitHeader(compilation_unit_headers->data[i]);
	}
}
#endif
