#include "elf_dwarf_enum_type.h"
#include "assert.h"

void elf_dwarf_build_enum_type(ElfDwarfDieType *die)
{
	uint32 size;
	int i;
	int j;
	DwarfDataEnumulatorType *obj = dwarf_alloc_data_type(DATA_TYPE_ENUM);
	ElfDwarfAttributeType *attr;
	ElfDwarfAbbrevType *abbrev;
	DwAtType attr_type;
	ElfDwarfDieType *member;

	for (i = 0; i < die->attribute->current_array_size; i++) {
		abbrev = (ElfDwarfAbbrevType *)die->abbrev_info;
		attr = (ElfDwarfAttributeType*)die->attribute->data[i];
		attr_type = abbrev->attribute_name->data[i];
		//printf("name=0x%x form=%s\n", attr_type, attr->typename);
		switch (attr_type) {
		case DW_AT_name:
			obj->info.typename = attr->encoded.string;
			break;
		case DW_AT_byte_size:
			obj->info.size = elf_dwarf_info_get_value(abbrev->attribute_form->data[i], attr, &size);
			break;
		case DW_AT_sibling:
		case DW_AT_decl_file:
		case DW_AT_decl_line:
			break;
		default:
			ASSERT(0);
		}
	}

	for (i = 0; i < die->children->current_array_size; i++) {
		DwarfDataEnumMember mem;
		mem.name = NULL;
		mem.const_value = 0;
		member = (ElfDwarfDieType*)die->children->data[i];
		abbrev = (ElfDwarfAbbrevType *)member->abbrev_info;
		if (member->abbrev_info->tag != DW_TAG_enumerator) {
			continue;
		}
		for (j = 0; j < member->attribute->current_array_size; j++) {
			attr = (ElfDwarfAttributeType*)member->attribute->data[j];
			attr_type = abbrev->attribute_name->data[j];
			//printf("member name=0x%x form=%s\n", attr_type, attr->typename);
			switch (attr_type) {
			case DW_AT_name:
				mem.name = attr->encoded.string;
				//printf("enum mem=%s\n", mem.name);
				break;
			case DW_AT_const_value:
				mem.const_value = elf_dwarf_info_get_value(abbrev->attribute_form->data[j], attr, &size);
				break;
			default:
				ASSERT(0);
			}
		}
		dwarf_add_enum_member(obj, mem.name, mem.const_value);
	}

	obj->info.die = die;
	dwarf_register_data_type(&obj->info);
}
