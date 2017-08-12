#ifndef _ELF_SECTION_H_
#define _ELF_SECTION_H_

#include "loader/elf.h"
#include "std_errno.h"

extern Std_ReturnType elf_symbol_load(uint8 *elf_data);
extern Std_ReturnType elfsym_get_symbol_num(uint32 *sym_num);

typedef enum {
	SYMBOL_TYPE_OBJECT,
	SYMBOL_TYPE_FUNC,
	SYMBOL_TYPE_NOTYPE,
	SYMBOL_TYPE_NOSUP,
} ElfSymbolEnumType;

typedef struct {
	char* 				name;
	uint32 				size;
	uint32 				addr;
	ElfSymbolEnumType	type;
} ElfSymbolType;
extern Std_ReturnType elfsym_get_symbol(uint32 index, ElfSymbolType *elfsym);


#define SECTION_DWARF_LINE_NAME		".debug_line"
#define SECTION_DWARF_ABBREV_NAME	".debug_abbrev"
#define SECTION_DWARF_INFO_NAME		".debug_info"
#define SECTION_DWARF_STR_NAME		".debug_str"
extern Std_ReturnType elf_section_get_dwarf_line(uint8 *elf_data, uint8 **section_data, uint32 *section_size);
extern Std_ReturnType elf_section_get(uint8 *elf_data, char *key, uint8 **section_data, uint32 *section_size);

extern uint8 elf_get_data8(uint8 *elf_data, uint32 off);
extern uint16 elf_get_data16(uint8 *elf_data, uint32 off);
extern uint32 elf_get_data32(uint8 *elf_data, uint32 off);
extern uint64 elf_get_data64(uint8 *elf_data, uint32 off);

#endif /* _ELF_SECTION_H_ */
