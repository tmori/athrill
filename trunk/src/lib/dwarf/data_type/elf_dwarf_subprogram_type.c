#include "elf_dwarf_subprogram_type.h"
#include "elf_dwarf_info_ops.h"
#include "elf_dwarf_data_type.h"
#include "assert.h"
#include <string.h>

static void elf_dwarf_build_subprogram_variable(DwarfDataSubprogramType *obj, ElfDwarfDieType *variable)
{
	uint32 size;
	int j;
	ElfDwarfAttributeType *attr;
	ElfDwarfAbbrevType *abbrev;
	DwAtType attr_type;
	uint32 offset;
	Std_ReturnType err;
	DwarfLocalVariableType localVariable;

	localVariable.is_valid_ref_debug_info_offset = FALSE;

	abbrev = (ElfDwarfAbbrevType *)variable->abbrev_info;
	memset(&localVariable, 0, sizeof(DwarfLocalVariableType));
	for (j = 0; j < variable->attribute->current_array_size; j++) {
		attr = (ElfDwarfAttributeType*)variable->attribute->data[j];
		attr_type = abbrev->attribute_name->data[j];
		//printf("variable name=0x%x form=%s\n", attr_type, attr->typename);
		switch (attr_type) {
		case DW_AT_name:
			//printf("=>%s\n", attr->encoded.string);
			localVariable.name = attr->encoded.string;
			break;
		case DW_AT_location:
			//printf("location:%d\n", attr->encoded.op.len);
			localVariable.DW_AT_location = attr;
			break;
		case DW_AT_type:
			offset = elf_dwarf_info_get_value(abbrev->attribute_form->data[j], attr, &size);
			err = dwarf_get_real_type_offset(offset, &localVariable.ref_debug_info_offset);
			if (err == STD_E_OK) {
				localVariable.is_valid_ref_debug_info_offset = TRUE;
			}
			break;
		case DW_AT_decl_file:
		case DW_AT_decl_line:
		case DW_AT_decl_column:
		case DW_AT_external:
		case DW_AT_abstract_origin:
		case DW_AT_declaration:
		case DW_AT_const_value:
		case DW_AT_artificial:
		case DW_AT_specification:
		case DW_AT_MIPS_linkage_name:
		case DW_AT_linkage_name:
		case DW_AT_unknown_0x2137:
			break;
		default:
			printf("attr_type=0x%x\n", attr_type);
			ASSERT(0);
		}
	}
	dwarf_add_subprogram_variable(obj, &localVariable);
	return;
}
void elf_dwarf_build_subprogram_type(ElfDwarfDieType *die)
{
	int i;
	DwarfDataSubprogramType *obj = dwarf_alloc_data_type(DATA_TYPE_SUBPROGRAM);
	ElfDwarfAttributeType *attr;
	ElfDwarfAbbrevType *abbrev;
	DwAtType attr_type;
	ElfDwarfDieType *entry;

	//printf("elf_dwarf_build_subprogram_type:off=0x%x\n", die->offset);
	//printf("subprogram_type\n");
	for (i = 0; i < die->attribute->current_array_size; i++) {
		abbrev = (ElfDwarfAbbrevType *)die->abbrev_info;
		attr = (ElfDwarfAttributeType*)die->attribute->data[i];
		attr_type = abbrev->attribute_name->data[i];
		//printf("name=0x%x form=%s\n", attr_type, attr->typename);
		switch (attr_type) {
		case DW_AT_name:
			obj->info.typename = attr->encoded.string;
			//printf("typename=%s\n", obj->info.typename);
			break;
		case DW_AT_frame_base:
			//printf("elf_dwarf_build_subprogram_type:off=0x%x\n", die->offset);
			if (attr->type == DW_FORM_data4) {
				obj->frame_loc_offset = attr->encoded.const32;
				//printf("name=0x%x form=%s off=0x%x\n", attr_type, attr->typename, obj->frame_loc_offset);
			}
			else if (attr->type == DW_FORM_exprloc) {
				//printf("ERROR: not supported elf_dwarf_build_subprogram_type:off=0x%x", die->offset);
			}
			else {
				//printf("ERROR: not supported elf_dwarf_build_subprogram_type:off=0x%x", die->offset);
				//printf("name=0x%x form=%s\n", attr_type, attr->typename);
			}
			break;
		case DW_AT_abstract_origin:
		case DW_AT_accessibility:
		case DW_AT_address_class:
		case DW_AT_artificial:
		case DW_AT_calling_convention:
		case DW_AT_declaration:
		case DW_AT_external:
		case DW_AT_high_pc:
		case DW_AT_inline:
		case DW_AT_low_pc:
		case DW_AT_prototyped:
		case DW_AT_return_addr:
		case DW_AT_segment:
		case DW_AT_sibling:
		case DW_AT_specification:
		case DW_AT_start_scope:
		case DW_AT_static_link:
		case DW_AT_type:
		case DW_AT_visibility:
		case DW_AT_virtuality:
		case DW_AT_vtable_elem_location:
		case DW_AT_decl_file:
		case DW_AT_decl_line:
		case DW_AT_decl_column:
		case DW_AT_linkage_name:
		case DW_AT_object_pointer:
		case DW_AT_explicit:
		case DW_AT_pure:
		case DW_AT_containing_type:
		case DW_AT_MIPS_linkage_name:
		case DW_AT_unknown_0x2116:
		case DW_AT_unknown_0x2117:
		case DW_AT_noreturn:
#if 0
		case DW_AT_linkage_name:
		case DW_AT_main_subprogram:
		case DW_AT_object_pointer:
		case DW_AT_pure:
		case DW_AT_ranges:
		case DW_AT_recursive:
		case DW_AT_trampoline:
		case DW_AT_description:
		case DW_AT_elemental:
		case DW_AT_entry_pc:
		case DW_AT_explicit:
#endif
			break;
		default:
			printf("die=0x%x attr_type=0x%x\n", die->offset, attr_type);
			ASSERT(0);
		}
	}

	/*
	 * entries
	 */
	if (die->children != NULL) {
		for (i = 0; i < die->children->current_array_size; i++) {
			entry = (ElfDwarfDieType*)die->children->data[i];
			if (entry->abbrev_info->tag == DW_TAG_variable) {
				elf_dwarf_build_subprogram_variable(obj, entry);
			}
			else if (entry->abbrev_info->tag == DW_TAG_formal_parameter) {
				elf_dwarf_build_subprogram_variable(obj, entry);
			}

		}

	}

	obj->info.die = die;
	//printf("variable=%s ref_offset=0x%x\n", obj->info.typename, obj->ref_debug_info_offset);

	dwarf_register_data_type(&obj->info);

	return;
}


static void elf_dwarf_resolve_local_variable(DwarfDataSubprogramType *obj)
{
	int i;
	DwarfLocalVariableType *localVariable;

	if (obj->variables == NULL) {
		return;
	}

	//printf("struct or union:%s\n", struct_obj->info.typename);
	for (i = 0; i < obj->variables->current_array_size; i++) {
		localVariable = (DwarfLocalVariableType *)obj->variables->data[i];

		if (localVariable->is_valid_ref_debug_info_offset == FALSE) {
			continue;
		}
		localVariable->ref = elf_dwarf_get_data_type(localVariable->ref_debug_info_offset);
		if (localVariable->ref == NULL) {
			//printf("Not supported:unknown typeref(%s) debug_offset=0x%x\n", localVariable->ref->typename, localVariable->ref_debug_info_offset);
		}
		else {
			//printf("func %s: %s %s;\n", obj->info.typename, localVariable->ref->typename, localVariable->name);
		}

	}
}

void elf_dwarf_resolve_subprogram_type(void)
{
	int i;
	ElfPointerArrayType	*my_types = dwarf_get_data_types(DATA_TYPE_SUBPROGRAM);
	DwarfDataSubprogramType *obj;

	if (my_types == NULL) {
		return;
	}

	for (i = 0; i < my_types->current_array_size; i++) {
		obj = (DwarfDataSubprogramType *)my_types->data[i];
		elf_dwarf_resolve_local_variable(obj);
	}
	return;
}

DwarfDataSubprogramType *elf_dwarf_search_subprogram(char *funcname)
{
	int namelen = strlen(funcname);
	int i;
	ElfPointerArrayType	*my_types = dwarf_get_data_types(DATA_TYPE_SUBPROGRAM);
	DwarfDataSubprogramType *obj = NULL;

	if (my_types == NULL) {
		return NULL;
	}

	for (i = 0; i < my_types->current_array_size; i++) {
		int len;
		obj = (DwarfDataSubprogramType *)my_types->data[i];
		if (obj->info.typename == NULL) {
			continue;
		}
		len = strlen(obj->info.typename);
		if (len != namelen) {
			continue;
		}
		if (strncmp(funcname, obj->info.typename, len) != 0) {
			continue;
		}
		return obj;
	}
	return NULL;
}

DwarfLocalVariableType *elf_dwarf_search_local_variable(DwarfDataSubprogramType *subprogram, char *local_variable)
{
	int namelen = strlen(local_variable);
	int i;

	DwarfLocalVariableType *localVariable;

	if (subprogram->variables == NULL) {
		return NULL;
	}

	for (i = 0; i < subprogram->variables->current_array_size; i++) {
		int len;
		localVariable = (DwarfLocalVariableType *)subprogram->variables->data[i];

		if (localVariable->ref == NULL) {
			continue;
		}
		if (localVariable->name == NULL) {
			continue;
		}
		len = strlen(localVariable->name);
		if (namelen != len) {
			continue;
		}
		if (strncmp(local_variable, localVariable->name, len) != 0) {
			continue;
		}
		return localVariable;
	}
	return NULL;
}
