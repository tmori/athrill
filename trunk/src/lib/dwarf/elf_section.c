#include "elf_section.h"
#include <stdio.h>
#include <string.h>

/*
 * セクション情報(内部向け)
 */
typedef struct {
	Elf32_Shdr *shdr;
	uint8		*shdata;
} ElfSectionHeaderType;

/*
 * ストリングテーブルのセクション情報(内部向け)
 */
typedef ElfSectionHeaderType ElfStringTableSectionType;

/*
 * シンボルのセクション情報(内部向け)
 */
typedef struct {
	Elf32_Shdr 					*shdr;
	Elf32_Sym					*sym;
	uint32						symbol_num;
	ElfStringTableSectionType	*string_table;
} ElfSymbolSectionType;

static ElfSymbolSectionType 		elf_symbol_section;
static ElfStringTableSectionType	elf_string_table_section;
static char *section_name = NULL;
static uint8 *elf_binary_data = NULL;

Std_ReturnType elf_symbol_load(uint8 *elf_data)
{
	uint32 i;
	uint32 shdr_num;
	uint32 shdr_off;
	uint32 shdr_size;
	uint32 sec_shr_index;
	Elf32_Ehdr *hdr = (Elf32_Ehdr*)elf_data;
	Elf32_Shdr *shdr;

	elf_binary_data = elf_data;

	sec_shr_index = hdr->e_shstrndx;
	shdr_num = hdr->e_shnum;
	shdr_off = hdr->e_shoff;
	shdr_size = hdr->e_shentsize;

	/*
	 * search_section_name
	 */
	for (i = 0; i < shdr_num; i++) {
		if (i != sec_shr_index) {
			continue;
		}
		shdr = (Elf32_Shdr *)&elf_data[(shdr_off + (i * shdr_size))];
		section_name = (char*)&elf_data[shdr->sh_offset];
		break;
	}

	/*
	 * search section
	 */
	for (i = 0; i < shdr_num; i++) {
		shdr = (Elf32_Shdr *)&elf_data[(shdr_off + (i * shdr_size))];
		if (i == sec_shr_index) {
			continue;
		}

		if (shdr->sh_type == SHT_SYMTAB) {
			elf_symbol_section.shdr = shdr;
			elf_symbol_section.sym = (Elf32_Sym*)&elf_data[shdr->sh_offset];
			elf_symbol_section.symbol_num = shdr->sh_size / sizeof(Elf32_Sym);
			printf("ELF SYMBOL SECTION LOADED:index=%u\n", i);
			printf("ELF SYMBOL SECTION LOADED:sym_num=%u\n", elf_symbol_section.symbol_num);
		}
		else if (shdr->sh_type == SHT_STRTAB) {
			printf("ELF STRING TABLE SECTION LOADED:index=%u\n", i);
			elf_string_table_section.shdata = &elf_data[shdr->sh_offset];
			elf_string_table_section.shdr = shdr;
		}
	}

	if ((elf_symbol_section.shdr == NULL) || (elf_string_table_section.shdr == NULL)) {
		printf("ERROR: can not found symbol section...\n");
		return STD_E_INVALID;
	}
	elf_symbol_section.string_table = &elf_string_table_section;
	return STD_E_OK;
}

Std_ReturnType elfsym_get_symbol_num(uint32 *sym_num)
{
	if ((elf_symbol_section.shdr == NULL) || (elf_string_table_section.shdr == NULL)) {
		return STD_E_INVALID;
	}
	*sym_num = elf_symbol_section.symbol_num;
	return STD_E_OK;
}

Std_ReturnType elfsym_get_symbol(uint32 index, ElfSymbolType *elfsym)
{
	Elf32_Sym *sym;
	if ((elf_symbol_section.shdr == NULL) || (elf_string_table_section.shdr == NULL)) {
		return STD_E_INVALID;
	}
	sym = &elf_symbol_section.sym[index];
	elfsym->addr = sym->st_value;
	elfsym->size = sym->st_size;
	//printf("type=0x%x name=%s\n", sym->st_info, (char*)&(elf_symbol_section.string_table->shdata[sym->st_name]));
	switch ((sym->st_info & 0x0F)) {
	case STT_OBJECT:
		elfsym->type = SYMBOL_TYPE_OBJECT;
		break;
	case STT_FUNC:
		elfsym->type = SYMBOL_TYPE_FUNC;
		break;
	case STT_NOTYPE:
		elfsym->type = SYMBOL_TYPE_NOTYPE;
		break;
	default:
		elfsym->type = SYMBOL_TYPE_NOSUP;
		break;
	}
	elfsym->name = (char*)&(elf_symbol_section.string_table->shdata[sym->st_name]);

	return STD_E_OK;
}

Std_ReturnType elf_section_get(uint8 *elf_data, char *key, uint8 **section_data, uint32 *section_size)
{
	uint32 i;
	uint32 shdr_num;
	uint32 shdr_off;
	uint32 shdr_size;
	uint32 sec_shr_index;
	Elf32_Ehdr *hdr = (Elf32_Ehdr*)elf_data;
	Elf32_Shdr *shdr;
	uint32 key_len = strlen(key);

	sec_shr_index = hdr->e_shstrndx;
	shdr_num = hdr->e_shnum;
	shdr_off = hdr->e_shoff;
	shdr_size = hdr->e_shentsize;


	/*
	 * search section
	 */
	for (i = 0; i < shdr_num; i++) {
		shdr = (Elf32_Shdr *)&elf_data[(shdr_off + (i * shdr_size))];
		if (i == sec_shr_index) {
			continue;
		}
		uint32 len = strlen(&section_name[shdr->sh_name]);
		if (key_len != len) {
			continue;
		}
		if (strncmp(key, &section_name[shdr->sh_name], key_len) != 0) {
			continue;
		}
		*section_data = &elf_data[shdr->sh_offset];
		*section_size = shdr->sh_size;
		//printf("inx=%u sh_name=%s\n", i, &section_name[shdr->sh_name]);

		return STD_E_OK;
	}
	return STD_E_NOENT;
}

extern Std_ReturnType elf_section_get_dwarf_line(uint8 *elf_data, uint8 **section_data, uint32 *section_size)
{
	return elf_section_get(elf_data, SECTION_DWARF_LINE_NAME, section_data, section_size);
}

uint8 elf_get_data8(uint8 *elf_data, uint32 off)
{
	return elf_data[off];
}
uint16 elf_get_data16(uint8 *elf_data, uint32 off)
{
	uint16 data = 0;

	data |=  ( ((uint16)elf_data[off + 0]) << 0 );
	data |=  ( ((uint16)elf_data[off + 1]) << 8 );
	return data;
}
uint32 elf_get_data32(uint8 *elf_data, uint32 off)
{
	uint32 data = 0;

	data |=  ( ((uint32)elf_data[off + 0]) << 0 );
	data |=  ( ((uint32)elf_data[off + 1]) << 8 );
	data |=  ( ((uint32)elf_data[off + 2]) << 16 );
	data |=  ( ((uint32)elf_data[off + 3]) << 24 );
	return data;
}
uint64 elf_get_data64(uint8 *elf_data, uint32 off)
{
	uint64 *data;
	uint8 array[8];

	data = (uint64*)array;
	array[0] = elf_data[off + 0];
	array[1] = elf_data[off + 1];
	array[2] = elf_data[off + 2];
	array[3] = elf_data[off + 3];
	array[4] = elf_data[off + 4];
	array[5] = elf_data[off + 5];
	array[6] = elf_data[off + 6];
	array[7] = elf_data[off + 7];

	return *data;
}

