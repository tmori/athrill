#ifndef _ELF_DWARF_SUBPROGRAM_TYPE_H_
#define _ELF_DWARF_SUBPROGRAM_TYPE_H_

#include "elf_dwarf_data_type.h"
#include "elf_dwarf_info.h"

extern void elf_dwarf_build_subprogram_type(ElfDwarfDieType *die);
extern void elf_dwarf_resolve_subprogram_type(void);

#endif /* _ELF_DWARF_SUBPROGRAM_TYPE_H_ */
