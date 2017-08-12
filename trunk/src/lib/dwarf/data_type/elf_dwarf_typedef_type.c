#include "elf_dwarf_typedef_type.h"
#include "assert.h"

void elf_dwarf_build_typedef_type(ElfDwarfDieType *die)
{
	int i;
	uint32 size;
	DwarfDataTypedefType *obj = dwarf_alloc_data_type(DATA_TYPE_TYPEDEF);
	ElfDwarfAttributeType *attr;
	ElfDwarfAbbrevType *abbrev;
	DwAtType attr_type;
	uint32 offset;
	Std_ReturnType err;

	//printf("elf_dwarf_build_typedef_type:off=0x%x\n", die->offset);
	//printf("typedef_type\n");
	for (i = 0; i < die->attribute->current_array_size; i++) {
		abbrev = (ElfDwarfAbbrevType *)die->abbrev_info;
		attr = (ElfDwarfAttributeType*)die->attribute->data[i];
		attr_type = abbrev->attribute_name->data[i];
		//printf("name=0x%x form=%s\n", attr_type, attr->typename);
		switch (attr_type) {
		case DW_AT_name:
			obj->info.typename = attr->encoded.string;
			break;
		case DW_AT_type:
			offset = elf_dwarf_info_get_value(abbrev->attribute_form->data[i], attr, &size);
			 err = dwarf_get_real_type_offset(offset, &obj->ref_debug_info_offset);
			 if (err == STD_E_OK) {
				 obj->is_valid_ref_debug_info_offset = TRUE;
			 }
			break;
		case DW_AT_decl_file:
		case DW_AT_decl_line:
			break;
		default:
			ASSERT(0);
		}
	}
	obj->info.die = die;
	//printf("typedef=%s ref_offset=0x%x\n", obj->info.typename, obj->ref_debug_info_offset);

	dwarf_register_data_type(&obj->info);

	return;
}
void elf_dwarf_resolve_typedef_type(void)
{
	int i;
	ElfPointerArrayType	*my_types = dwarf_get_data_types(DATA_TYPE_TYPEDEF);
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
			//printf("typedef %s %s\n", obj->ref->typename, obj->info.typename);
		}
	}

	return;
}
