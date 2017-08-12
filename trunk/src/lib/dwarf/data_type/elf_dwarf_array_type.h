#ifndef _ELF_DWARF_ARRAY_TYPE_H_
#define _ELF_DWARF_ARRAY_TYPE_H_

#include "elf_dwarf_data_type.h"
#include "elf_dwarf_info.h"

extern void elf_dwarf_build_array_type(ElfDwarfDieType *die);
extern void elf_dwarf_resolve_array_type(void);

#endif /* _ELF_DWARF_ARRAY_TYPE_H_ */
