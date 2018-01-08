#ifndef _ELF_DWARF_LOC_H_
#define _ELF_DWARF_LOC_H_

#include "loader/elf.h"
#include "elf_dwarf_util.h"
#include "data_type/elf_dwarf_subprogram_type.h"
#include "std_errno.h"

struct ElfDwarfLocEntryType_tag;
typedef struct ElfDwarfLocEntryType_tag {
	uint32	entryOffset;
	uint32	entrySize;
	uint32	relativeAddress0;
	uint32	relativeAddress1;
	uint16	block_length;
	uint8	*blocks;
	struct ElfDwarfLocEntryType_tag *next;
} ElfDwarfLocEntryType;

extern Std_ReturnType elf_dwarf_loc_load(uint8 *elf_data);
extern bool printLocalValueV850(DwarfDataSubprogramType *subprogram, DwarfLocalVariableType *localVariable, uint32 pc, uint32 funcaddr, uint32 *vaddr);

#endif /* _ELF_DWARF_LOC_H_ */
