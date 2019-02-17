#include "elf_dwarf_struct_type.h"
#include "assert.h"
#include <string.h>

static void elf_dwarf_build_struct_member(DwarfDataStructType *obj, ElfDwarfDieType *member)
{
	uint32 size;
	int j;
	DwarfDataStructMember mem;
	ElfDwarfAttributeType *attr;
	ElfDwarfAbbrevType *abbrev;
	DwAtType attr_type;
	uint32 offset;
	Std_ReturnType err;

	abbrev = (ElfDwarfAbbrevType *)member->abbrev_info;
	memset(&mem, 0, sizeof(mem));
	for (j = 0; j < member->attribute->current_array_size; j++) {
		attr = (ElfDwarfAttributeType*)member->attribute->data[j];
		attr_type = abbrev->attribute_name->data[j];
		//printf("member name=0x%x form=%s\n", attr_type, attr->typename);
		switch (attr_type) {
		case DW_AT_name:
			mem.name = attr->encoded.string;
			//printf("mem.name=%s\n", mem.name);
			break;
		case DW_AT_type:
			//value = elf_dwarf_info_get_value(abbrev->attribute_form->data[j], attr, &size);
			offset = elf_dwarf_info_get_value(abbrev->attribute_form->data[j], attr, &size);
			err = dwarf_get_real_type_offset(offset, &mem.ref_debug_info_offset);
			if (err == STD_E_OK) {
				mem.is_valid_ref_debug_info_offset = TRUE;
				//printf("struct location off=0x%x\n", mem.ref_debug_info_offset);
			}
			else {
				//printf("struct location type not found err=%u\n", err);
			}
			break;
		case DW_AT_data_member_location:
			mem.off = elf_dwarf_info_get_value(abbrev->attribute_form->data[j], attr, &size);
			//printf("struct location from=0x%x off=%u\n", abbrev->attribute_form->data[j], mem.off);
			break;
		case DW_AT_accessibility:
		case DW_AT_byte_size:
		case DW_AT_bit_offset:
		case DW_AT_bit_size:
		case DW_AT_declaration:
		case DW_AT_visibility:
		case DW_AT_decl_file:
		case DW_AT_decl_line:
		case DW_AT_decl_column:
		case DW_AT_external:
			break;
		default:
			printf("attr_type=0x%x\n", attr_type);
			ASSERT(0);
		}
	}
	dwarf_add_struct_member(obj, &mem);
	return;
}

static void elf_dwarf_build_struct_method(DwarfDataStructType *obj, ElfDwarfDieType *member)
{
	uint32 size;
	int j;
	DwarfDataStructMember mem;
	ElfDwarfAttributeType *attr;
	ElfDwarfAbbrevType *abbrev;
	DwAtType attr_type;
	uint32 offset;
	ElfDwarfDieType *sibling = NULL;

	abbrev = (ElfDwarfAbbrevType *)member->abbrev_info;
	memset(&mem, 0, sizeof(mem));
	for (j = 0; j < member->attribute->current_array_size; j++) {
		attr = (ElfDwarfAttributeType*)member->attribute->data[j];
		attr_type = abbrev->attribute_name->data[j];
		//printf("member name=0x%x form=%s\n", attr_type, attr->typename);
		switch (attr_type) {
		case DW_AT_name:
			mem.name = attr->encoded.string;
			//printf("mem.name=%s\n", mem.name);
			break;
		case DW_AT_sibling:
			offset = elf_dwarf_info_get_value(abbrev->attribute_form->data[j], attr, &size);
			sibling = dwarf_get_die(offset);
			//printf("sibling=%p offset=0x%x\n", sibling, offset);
			break;
		case DW_AT_MIPS_linkage_name:
			mem.linkage_name = attr->encoded.string;
			//printf("mem.linkage_name=%s\n", mem.linkage_name);
			break;
		case DW_AT_type:
		case DW_AT_data_member_location:
		case DW_AT_accessibility:
		case DW_AT_byte_size:
		case DW_AT_bit_offset:
		case DW_AT_bit_size:
		case DW_AT_declaration:
		case DW_AT_visibility:
		case DW_AT_decl_file:
		case DW_AT_decl_line:
		case DW_AT_decl_column:
		case DW_AT_external:
		case DW_AT_object_pointer:
		case DW_AT_inline:
		case DW_AT_low_pc:
		case DW_AT_high_pc:
		case DW_AT_frame_base:
		case DW_AT_prototyped:
		case DW_AT_explicit:
		case DW_AT_linkage_name:
		case DW_AT_artificial:
		case DW_AT_unknown_0x2117:
			break;
		default:
			printf("attr_type=0x%x\n", attr_type);
			ASSERT(0);
		}
	}
	dwarf_add_struct_member(obj, &mem);
	if (sibling != NULL) {
		elf_dwarf_build_struct_method(obj, sibling);
	}
	return;
}

void elf_dwarf_build_struct_type(ElfDwarfDieType *die)
{
	uint32 size;
	int i;
	DwarfDataStructType *obj;
	ElfDwarfAttributeType *attr;
	ElfDwarfAbbrevType *abbrev;
	DwAtType attr_type;
	ElfDwarfDieType *member;

	if (die->children == NULL) {
		// no member;
		return;
	}

	if (die->abbrev_info->tag == DW_TAG_structure_type) {
		obj = dwarf_alloc_data_type(DATA_TYPE_STRUCT);
	}
	else if (die->abbrev_info->tag == DW_TAG_class_type) {
		obj = dwarf_alloc_data_type(DATA_TYPE_CLASS);
	}
	else {
		obj = dwarf_alloc_data_type(DATA_TYPE_UNION);
	}


	//printf("struct_type\n");
	for (i = 0; i < die->attribute->current_array_size; i++) {
		abbrev = (ElfDwarfAbbrevType *)die->abbrev_info;
		attr = (ElfDwarfAttributeType*)die->attribute->data[i];
		attr_type = abbrev->attribute_name->data[i];
		//printf("name=0x%x form=%s\n", attr_type, attr->typename);
		switch (attr_type) {
		case DW_AT_name:
			obj->info.typename = attr->encoded.string;
			//printf("0x%p struct typename=%s\n", obj, obj->info.typename);
			break;
		case DW_AT_byte_size:
			obj->info.size = elf_dwarf_info_get_value(abbrev->attribute_form->data[i], attr, &size);
			//printf("0x%p struct size=%u\n", obj, obj->info.size);
			break;
		case DW_AT_sibling:
		case DW_AT_decl_file:
		case DW_AT_decl_line:
		case DW_AT_decl_column:
		case DW_AT_declaration:
		case DW_AT_accessibility:
		case DW_AT_linkage_name:
		case DW_AT_MIPS_linkage_name:
			break;
		default:
			printf("attr_type=0x%x\n", attr_type);
			ASSERT(0);
		}
	}

	/*
	 * members
	 */
	for (i = 0; i < die->children->current_array_size; i++) {
		member = (ElfDwarfDieType*)die->children->data[i];
		if (member->abbrev_info->tag == DW_TAG_member) {
			elf_dwarf_build_struct_member(obj, member);
		}
		else if(member->abbrev_info->tag == DW_TAG_subprogram) {
			if (die->abbrev_info->tag == DW_TAG_class_type) {
				elf_dwarf_build_struct_method(obj, member);
			}
		}
	}

	obj->info.die = die;
	dwarf_register_data_type(&obj->info);
	return;
}

char *elf_dwarf_get_class_method_linkagename(char *classname, char *methodname)
{
	int i;
	int j;
	ElfPointerArrayType	*my_types = dwarf_get_data_types(DATA_TYPE_CLASS);
	DwarfDataStructType *obj;
	DwarfDataStructMember *member;

	if (my_types == NULL) {
		return NULL;
	}

	for (i = 0; i < my_types->current_array_size; i++) {
		obj = (DwarfDataStructType *)my_types->data[i];
		if (strcmp(obj->info.typename, classname) != 0) {
			continue;
		}
		for (j = 0; j < obj->members->current_array_size; j++) {
			member = (DwarfDataStructMember *)obj->members->data[j];
			if (strcmp(member->name, methodname) != 0) {
				continue;
			}
			return member->linkage_name;
		}
	}
	return NULL;
}

static void elf_dwarf_resolve_struct_union_member(DwarfDataStructType *struct_obj)
{
	int i;
	DwarfDataStructMember *obj;

	//printf("struct or union:%s\n", struct_obj->info.typename);
	if (struct_obj->members == NULL) {
		return;
	}
	for (i = 0; i < struct_obj->members->current_array_size; i++) {
		obj = (DwarfDataStructMember *)struct_obj->members->data[i];
		//printf("member %s ref=%p flag=%u;\n", obj->name, obj->ref, obj->is_valid_ref_debug_info_offset);
		if (obj->ref != NULL) {
			continue;
		}
		if (obj->is_valid_ref_debug_info_offset == FALSE) {
			continue;
		}
		obj->ref = elf_dwarf_get_data_type(obj->ref_debug_info_offset);
		if (obj->ref == NULL) {
			//printf("Not supported:unknown typeref(%s) debug_offset=0x%x\n", obj->ref->typename, obj->ref_debug_info_offset);
		}
		else {
			//printf("member %s %s;\n", obj->ref->typename, obj->name);
		}

	}
}

static void elf_dwarf_resolve_union(void)
{
	int i;
	ElfPointerArrayType	*my_types = dwarf_get_data_types(DATA_TYPE_UNION);
	DwarfDataStructType *obj;

	if (my_types == NULL) {
		return;
	}

	for (i = 0; i < my_types->current_array_size; i++) {
		obj = (DwarfDataStructType *)my_types->data[i];
		elf_dwarf_resolve_struct_union_member(obj);
	}
	return;
}
static void elf_dwarf_resolve_struct(void)
{
	int i;
	ElfPointerArrayType	*my_types = dwarf_get_data_types(DATA_TYPE_STRUCT);
	DwarfDataStructType *obj;

	if (my_types == NULL) {
		return;
	}

	for (i = 0; i < my_types->current_array_size; i++) {
		obj = (DwarfDataStructType *)my_types->data[i];
		elf_dwarf_resolve_struct_union_member(obj);
	}
	return;
}

static void elf_dwarf_resolve_class(void)
{
	int i;
	ElfPointerArrayType	*my_types = dwarf_get_data_types(DATA_TYPE_CLASS);
	DwarfDataStructType *obj;

	if (my_types == NULL) {
		return;
	}

	for (i = 0; i < my_types->current_array_size; i++) {
		obj = (DwarfDataStructType *)my_types->data[i];
		elf_dwarf_resolve_struct_union_member(obj);
	}
	return;
}

void elf_dwarf_resolve_struct_type(void)
{
	elf_dwarf_resolve_struct();
	elf_dwarf_resolve_union();
	elf_dwarf_resolve_class();
	return;
}

