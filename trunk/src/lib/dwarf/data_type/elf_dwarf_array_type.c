#include "elf_dwarf_array_type.h"
#include "assert.h"

static void elf_dwarf_build_array_dimension(DwarfDataArrayType *obj, ElfDwarfDieType *member)
{
	uint32 size;
	int j;
	uint32 array_size;
	ElfDwarfAttributeType *attr;
	ElfDwarfAbbrevType *abbrev;
	DwAtType attr_type;

	abbrev = (ElfDwarfAbbrevType *)member->abbrev_info;
	for (j = 0; j < member->attribute->current_array_size; j++) {
		attr = (ElfDwarfAttributeType*)member->attribute->data[j];
		attr_type = abbrev->attribute_name->data[j];
		//printf("dimension name=0x%x form=%s\n", attr_type, attr->typename);
		switch (attr_type) {
		case DW_AT_upper_bound:
			array_size = elf_dwarf_info_get_value(abbrev->attribute_form->data[j], attr, &size);
			dwarf_uint32_array_add_entry(obj->dimension, (array_size + 1));
			//printf("array_size=%u\n", array_size);
			break;
		case DW_AT_abstract_origin:
		case DW_AT_accessibility:
		case DW_AT_byte_size:
		case DW_AT_count:
		case DW_AT_declaration:
		case DW_AT_lower_bound:
		case DW_AT_name:
		case DW_AT_sibling:
		case DW_AT_type:
			break;
		default:
			ASSERT(0);
		}
	}

	return;
}
void elf_dwarf_build_array_type(ElfDwarfDieType *die)
{
	int i;
	uint32 size;
	DwarfDataArrayType *obj = dwarf_alloc_data_type(DATA_TYPE_ARRAY);
	ElfDwarfAttributeType *attr;
	ElfDwarfAbbrevType *abbrev;
	DwAtType attr_type;
	uint32 offset;
	Std_ReturnType err;

	obj->dimension = dwarf_uint32_array_alloc();

	//printf("elf_dwarf_build_typedef_type:off=0x%x\n", die->offset);
	//printf("typedef_type\n");
	for (i = 0; i < die->attribute->current_array_size; i++) {
		abbrev = (ElfDwarfAbbrevType *)die->abbrev_info;
		attr = (ElfDwarfAttributeType*)die->attribute->data[i];
		attr_type = abbrev->attribute_name->data[i];
		//printf("name=0x%x form=%s\n", attr_type, attr->typename);
		switch (attr_type) {
		case DW_AT_type:
			offset = elf_dwarf_info_get_value(abbrev->attribute_form->data[i], attr, &size);
			 err = dwarf_get_real_type_offset(offset, &obj->ref_debug_info_offset);
			 if (err == STD_E_OK) {
				 obj->is_valid_ref_debug_info_offset = TRUE;
			 }
			break;
		case DW_AT_sibling:
			break;
		default:
			ASSERT(0);
		}
	}
	//配列サイズ
	for (i = 0; i < die->children->current_array_size; i++) {
		ElfDwarfDieType *member = (ElfDwarfDieType*)die->children->data[i];
		if (member->abbrev_info->tag != DW_TAG_subrange_type) {
			continue;
		}
		elf_dwarf_build_array_dimension(obj, member);
	}


	obj->info.die = die;
	//printf("typedef=%s ref_offset=0x%x\n", obj->info.typename, obj->ref_debug_info_offset);

	dwarf_register_data_type(&obj->info);

	return;
}

void elf_dwarf_resolve_array_type(void)
{
	int i;
	ElfPointerArrayType	*my_types = dwarf_get_data_types(DATA_TYPE_ARRAY);
	DwarfDataTypedefType *obj;
	if (my_types == NULL) {
		return;
	}
	for (i = 0; i < my_types->current_array_size; i++) {
		obj = (DwarfDataTypedefType *)my_types->data[i];
		if (obj->ref != NULL) {
			continue;
		}
		if (obj->is_valid_ref_debug_info_offset == FALSE) {
			continue;
		}
		obj->ref = elf_dwarf_get_data_type(obj->ref_debug_info_offset);
		if (obj->ref == NULL) {
			//printf("Not supported:unknown typeref(%s) debug_offset=0x%x\n", obj->info.typename, obj->ref_debug_info_offset);
		}
		else {
			//printf("array %s %s\n", obj->ref->typename, obj->info.typename);
		}
	}
	return;
}
