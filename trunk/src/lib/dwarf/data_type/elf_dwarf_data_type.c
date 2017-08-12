#include "elf_dwarf_data_type.h"
#include "elf_dwarf_info.h"
#include "elf_dwarf_base_type.h"
#include "elf_dwarf_struct_type.h"
#include "elf_dwarf_typedef_type.h"
#include "elf_dwarf_pointer_type.h"
#include "elf_dwarf_array_type.h"
#include "elf_dwarf_enum_type.h"
#include "elf_dwarf_variable_type.h"
#include "assert.h"
#include <string.h>

static ElfPointerArrayType	*dwarf_data_type_set[DATA_TYPE_NUM] = {
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
};
ElfPointerArrayType	*dwarf_get_data_types(DwarfDataEnumType type)
{
	return dwarf_data_type_set[type];
}


typedef  void (*parse_func_table_t)(ElfDwarfDieType *die);

static parse_func_table_t parse_func_table[DATA_TYPE_NUM] = {
		elf_dwarf_build_base_type,
		elf_dwarf_build_struct_type,
		elf_dwarf_build_struct_type,
		elf_dwarf_build_array_type,
		elf_dwarf_build_pointer_type,
		elf_dwarf_build_typedef_type,
		elf_dwarf_build_enum_type,
		elf_dwarf_build_variable_type,
};

static DwarfDataEnumType get_dataType(DwTagType tag)
{
	DwarfDataEnumType ret = DATA_TYPE_NUM;
	switch (tag) {
	case DW_TAG_array_type:
		ret = DATA_TYPE_ARRAY;
		break;
	case DW_TAG_pointer_type:
		ret = DATA_TYPE_POINTER;
		break;
	case DW_TAG_base_type:
		ret = DATA_TYPE_BASE;
		break;
	case DW_TAG_structure_type:
		ret = DATA_TYPE_STRUCT;
		break;
	case DW_TAG_union_type:
		ret = DATA_TYPE_UNION;
		break;
	case DW_TAG_typedef:
		ret = DATA_TYPE_TYPEDEF;
		break;
	case DW_TAG_enumeration_type:
		ret = DATA_TYPE_ENUM;
		break;
	case DW_TAG_variable:
		ret = DATA_TYPE_VARIABLE;
		break;
	default:
		break;
	}
	return ret;
}

static void dwarf_search_die_recursive(ElfDwarfDieType *die)
{
	int i_childs;
	ElfDwarfDieType *child;
	DwarfDataEnumType type;

	if (die->attribute == NULL) {
		return;
	}
	type = get_dataType(die->abbrev_info->tag);
	if (type != DATA_TYPE_NUM) {
		//printf("die(%u)=0x%x\n", die->level, die->abbrev_info->tag);
		parse_func_table[type](die);
	}
	if (die->children == NULL) {
		return;
	}
	for (i_childs = 0; i_childs < die->children->current_array_size; i_childs++) {
		child = die->children->data[i_childs];
		dwarf_search_die_recursive(child);
	}
	return;
}

static void build_types(void)
{
	int i_cu;
	int i_die;
	ElfDwarfCompilationUnitHeaderType	*cu;
	ElfDwarfDieType						*die;

	ElfPointerArrayType *compilation_unit_set = elf_dwarf_info_get();

	for (i_cu = 0; i_cu < compilation_unit_set->current_array_size; i_cu++) {
		cu = (ElfDwarfCompilationUnitHeaderType *)compilation_unit_set->data[i_cu];
		//printf("cu->offset=0x%x\n", cu->offset);
		if (cu->dies == NULL) {
			continue;
		}
		for (i_die = 0; i_die < cu->dies->current_array_size; i_die++) {
			die = (ElfDwarfDieType *)cu->dies->data[i_die];
			if (die->parent != NULL) {
				continue;
			}
			//printf("die->abbrev_code=%u\n", die->abbrev_code);
			//printf("die->children=0x%p\n", die->children);
			dwarf_search_die_recursive(die);
		}
	}

	return;
}

static Std_ReturnType get_DW_AT_type_value(ElfDwarfDieType *die, uint32 *retp)
{
	int i;
	uint32 size;
	ElfDwarfAttributeType *attr;
	ElfDwarfAbbrevType *abbrev;
	DwAtType attr_type;

	if (die->attribute->current_array_size == 0) {
		//printf("get_DW_AT_type_value:not supported:off=0x%x die->attribute->current_array_size=%u\n", die->offset, die->attribute->current_array_size);
		//ASSERT(0);
		return STD_E_NOENT;
	}

	for (i = 0; i < die->attribute->current_array_size; i++) {
		abbrev = (ElfDwarfAbbrevType *)die->abbrev_info;
		attr = (ElfDwarfAttributeType*)die->attribute->data[i];
		attr_type = abbrev->attribute_name->data[i];
		//printf("get_DW_AT_type_value:off=0x%x 0x%x\n", die->offset, attr_type);
		switch (attr_type) {
		case DW_AT_type:
			*retp = elf_dwarf_info_get_value(abbrev->attribute_form->data[i], attr, &size);
			return STD_E_OK;
		case DW_AT_prototyped:
			//printf("Not Supported:get_DW_AT_type_value:off=0x%x 0x%x\n", die->offset, attr_type);
			return STD_E_NOENT;
		default:
			printf("get_DW_AT_type_value:not supported attr=0x%x\n", attr_type);
			break;
		}
	}
	printf("get_DW_AT_type_value:not supported:off=0x%x die->attribute->current_array_size=%u\n", die->offset, die->attribute->current_array_size);
	ASSERT(0);
	return STD_E_NOENT;
}


Std_ReturnType dwarf_get_real_type_offset(uint32 offset, uint32 *retp)
{
	uint32 ret_offset;
	int i_cu;
	int i_die;
	ElfDwarfCompilationUnitHeaderType	*cu;
	ElfDwarfDieType						*die;
	DwarfDataEnumType					type;
	Std_ReturnType err;

	ElfPointerArrayType *compilation_unit_set = elf_dwarf_info_get();

retry:
	for (i_cu = 0; i_cu < compilation_unit_set->current_array_size; i_cu++) {
		cu = (ElfDwarfCompilationUnitHeaderType *)compilation_unit_set->data[i_cu];
		//printf("cu->offset=0x%x\n", cu->offset);
		if (cu->dies == NULL) {
			continue;
		}
		for (i_die = 0; i_die < cu->dies->current_array_size; i_die++) {
			die = (ElfDwarfDieType *)cu->dies->data[i_die];
			if (die->offset != offset) {
				continue;
			}
			if (die->abbrev_info->tag == DW_TAG_subroutine_type) {
				//printf("Not supported:dwarf_get_real_type_offset(0x%x) DW_TAG_subroutine_type\n", die->offset);
				return STD_E_NOENT;
			}
			type = get_dataType(die->abbrev_info->tag);
			if (type == DATA_TYPE_NUM) {
				err = get_DW_AT_type_value(die, &ret_offset);
				if (err != STD_E_OK) {
					return err;
				}
				offset = ret_offset;
				goto retry;
			}
			*retp = die->offset;
			return STD_E_OK;
		}
	}
	ASSERT(0);
	return ret_offset;
}


DwarfDataType *elf_dwarf_get_data_type(uint32 debug_info_offset)
{
	int i;
	int j;
	for (i = 0; i < DATA_TYPE_NUM; i++) {
		if (dwarf_data_type_set[i] == NULL) {
			continue;
		}
		for (j = 0; j < dwarf_data_type_set[i]->current_array_size; j++) {
			DwarfDataType *dtype = (DwarfDataType*)dwarf_data_type_set[i]->data[j];
			if (dtype->die->offset == debug_info_offset) {
				return dtype;
			}
		}
	}
	return NULL;
}

static void resolve_reference(void)
{
	elf_dwarf_resolve_typedef_type();
	elf_dwarf_resolve_pointer_type();
	elf_dwarf_resolve_array_type();
	elf_dwarf_resolve_struct_type();
	elf_dwarf_resolve_variable_type();
	return;
}

/*
 * 1) build definition
 * 2) resolve reference
 */
void dwarf_build_data_type_set(void)
{
	build_types();
	resolve_reference();
}

void *dwarf_alloc_data_type(DwarfDataEnumType type)
{
	uint32 size;
	DwarfDataType *obj;

	switch (type) {
	case DATA_TYPE_BASE:
		size = sizeof(DwarfDataBaseType);
		break;
	case DATA_TYPE_STRUCT:
		size = sizeof(DwarfDataStructType);
		break;
	case DATA_TYPE_UNION:
		size = sizeof(DwarfDataStructType);
		break;
	case DATA_TYPE_ARRAY:
		size = sizeof(DwarfDataArrayType);
		break;
	case DATA_TYPE_POINTER:
		size = sizeof(DwarfDataPointerType);
		break;
	case DATA_TYPE_TYPEDEF:
		size = sizeof(DwarfDataTypedefType);
		break;
	case DATA_TYPE_ENUM:
		size = sizeof(DwarfDataEnumulatorType);
		break;
	case DATA_TYPE_VARIABLE:
		size = sizeof(DwarfDataVariableType);
		break;
	default:
		ASSERT(0);
		break;
	}
	obj = (DwarfDataType *)elf_obj_alloc(size);
	obj->type = type;
	return obj;
}

void *dwarf_search_data_type(DwarfDataEnumType type, char *dirname, char *filename, char *typename)
{
	int i;
#if 0
	int dirlen = strlen(dirname);
	int filelen = strlen(filename);
#endif
	int typelen = strlen(typename);

	//printf("type=%u name=%s\n", type, typename);
	if (type >= DATA_TYPE_NUM) {
		return NULL;
	}
	if (dwarf_data_type_set[type] == NULL) {
		return NULL;
	}

	for (i = 0; i < dwarf_data_type_set[type]->current_array_size; i++) {
		int len;
		DwarfDataType *entry = (DwarfDataType *)dwarf_data_type_set[type]->data[i];

		//printf("entry(0x%p\n", entry);
		if (entry->typename == NULL) {
			continue;
		}
		len = strlen(entry->typename);
		//printf("in_len=%u chk_len=%u in_name=%s chk_name=%s\n", typelen, len, typename, entry->typename);
		if (typelen != len) {
			continue;
		}
		if (strncmp(typename, entry->typename, len) != 0) {
			continue;
		}
#if 0
		len = strlen(entry->dirname);
		if (dirlen != len) {
			continue;
		}
		if (strncmp(dirname, entry->dirname, len) == 0) {
			continue;
		}
		len = strlen(entry->filename);
		if (filelen != len) {
			continue;
		}
		if (strncmp(filename, entry->filename, len) == 0) {
			continue;
		}
#endif

		return entry;
	}
	return NULL;
}
void *dwarf_search_data_type_from_die(DwarfDataEnumType type, uint32 die_off)
{
	int i;

	if (type >= DATA_TYPE_NUM) {
		return NULL;
	}
	if (dwarf_data_type_set[type] == NULL) {
		return NULL;
	}

	for (i = 0; i < dwarf_data_type_set[type]->current_array_size; i++) {
		DwarfDataType *entry = (DwarfDataType *)dwarf_data_type_set[type]->data[i];
		if (entry->die->offset == die_off) {
			return entry;
		}
	}
	return NULL;
}

void dwarf_register_data_type(DwarfDataType *entry)
{
	if (entry->type >= DATA_TYPE_NUM) {
		return;
	}
	if (dwarf_data_type_set[entry->type] == NULL) {
		dwarf_data_type_set[entry->type] = elf_array_alloc();
	}
	//printf("type=%u reg data=%s\n", entry->type, entry->typename);

	elf_array_add_entry(dwarf_data_type_set[entry->type], entry);
	return;
}


void dwarf_add_struct_member(DwarfDataStructType *obj, DwarfDataStructMember *org_mem)
{
	DwarfDataStructMember *member;

	member = (DwarfDataStructMember*)elf_obj_alloc(sizeof(DwarfDataStructMember));
	*member = *org_mem;
	if (obj->members == NULL) {
		obj->members = elf_array_alloc();
	}

	elf_array_add_entry(obj->members, member);

	return;
}

void dwarf_add_enum_member(DwarfDataEnumulatorType *obj, char *name, uint32 const_value)
{
	DwarfDataEnumMember *member;

	member = (DwarfDataEnumMember*)elf_obj_alloc(sizeof(DwarfDataEnumMember));
	member->name = name;
	member->const_value = const_value;
	if (obj->members == NULL) {
		obj->members = elf_array_alloc();
	}

	elf_array_add_entry(obj->members, member);

	return;
}

