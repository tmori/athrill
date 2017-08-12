#include "elf_dwarf_base_type.h"
#include "assert.h"

void elf_dwarf_build_base_type(ElfDwarfDieType *die)
{
	int i;
	uint32 size;
	DwarfDataBaseType *obj = dwarf_alloc_data_type(DATA_TYPE_BASE);
	ElfDwarfAttributeType *attr;
	ElfDwarfAbbrevType *abbrev;
	DwAtType attr_type;

	for (i = 0; i < die->attribute->current_array_size; i++) {
		abbrev = (ElfDwarfAbbrevType *)die->abbrev_info;
		attr = (ElfDwarfAttributeType*)die->attribute->data[i];
		attr_type = abbrev->attribute_name->data[i];

		switch (attr_type) {
		case DW_AT_byte_size:
			obj->info.size = elf_dwarf_info_get_value(abbrev->attribute_form->data[i], attr, &size);
			break;
		case DW_AT_name:
			obj->info.typename = attr->encoded.string;
			break;
		case DW_AT_encoding:
			obj->encoding = elf_dwarf_info_get_value(abbrev->attribute_form->data[i], attr, &size);
			//printf("encoding=0x%x\n", obj->encoding);
			break;
		default:
			ASSERT(0);
		}
	}
	obj->info.die = die;

	dwarf_register_data_type(&obj->info);
	return;
}
