#ifndef _ELF_DWARF_STRUCT_TYPE_H_
#define _ELF_DWARF_STRUCT_TYPE_H_

#include "elf_dwarf_data_type.h"
#include "elf_dwarf_info.h"

extern void elf_dwarf_build_struct_type(ElfDwarfDieType *die);
extern void elf_dwarf_resolve_struct_type(void);

#endif /* _ELF_DWARF_STRUCT_TYPE_H_ */
