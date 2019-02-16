#include "elf_dwarf_pointer_type.h"
#include "assert.h"

void elf_dwarf_build_pointer_type(ElfDwarfDieType *die)
{
	int i;
	uint32 size;
	DwarfDataPointerType *obj = dwarf_alloc_data_type(DATA_TYPE_POINTER);
	ElfDwarfAttributeType *attr;
	ElfDwarfAbbrevType *abbrev;
	DwAtType attr_type;
	uint32 offset;
	Std_ReturnType err;

	obj->info.typename = "*";
	 obj->is_valid_ref_debug_info_offset = FALSE;

	//printf("elf_dwarf_build_pointer_type:off=0x%x\n", die->offset);
	for (i = 0; i < die->attribute->current_array_size; i++) {
		abbrev = (ElfDwarfAbbrevType *)die->abbrev_info;
		attr = (ElfDwarfAttributeType*)die->attribute->data[i];
		attr_type = abbrev->attribute_name->data[i];
		switch (attr_type) {
		case DW_AT_name:
			obj->info.typename = attr->typename;
			break;
		case DW_AT_byte_size:
			obj->info.size = elf_dwarf_info_get_value(abbrev->attribute_form->data[i], attr, &size);
			break;
		case DW_AT_type:
			offset = elf_dwarf_info_get_value(abbrev->attribute_form->data[i], attr, &size);
			 err = dwarf_get_real_type_offset(offset, &obj->ref_debug_info_offset);
			 if (err == STD_E_OK) {
				 obj->is_valid_ref_debug_info_offset = TRUE;
			 }
			//printf("ref_debug_info_offset:off=0x%x\n", obj->ref_debug_info_offset);
			break;
		default:
			printf("die=0x%x attr=0x%x\n", die->offset, attr_type);
			ASSERT(0);
		}
	}
	obj->info.die = die;

	dwarf_register_data_type(&obj->info);

	return;
}

void elf_dwarf_resolve_pointer_type(void)
{
	int i;
	ElfPointerArrayType	*my_types = dwarf_get_data_types(DATA_TYPE_POINTER);
	DwarfDataPointerType *obj;
	if (my_types == NULL) {
		return;
	}
	for (i = 0; i < my_types->current_array_size; i++) {
		obj = (DwarfDataPointerType *)my_types->data[i];
		if (obj->ref != NULL) {
			continue;
		}
		if (obj->is_valid_ref_debug_info_offset == FALSE) {
			continue;
		}
		obj->ref = elf_dwarf_get_data_type(obj->ref_debug_info_offset);
		//printf("pointer(0x%x):off=0x%x ref_off=0x%x\n", obj->info.die->offset, obj->ref_debug_info_offset, obj->ref->die->offset);
		if (obj->ref == NULL) {
			//printf("Not supported:unknown typeref(%s) debug_offset=0x%x\n", obj->info.typename, obj->ref_debug_info_offset);
		}
		else {
			//printf("pointer %s %s\n", obj->ref->typename, obj->info.typename);
		}
	}
	return;
}
