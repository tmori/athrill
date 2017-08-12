#ifndef _ELF_DWARF_INFO_H_
#define _ELF_DWARF_INFO_H_
#include "std_types.h"
#include "std_errno.h"
#include "elf_dwarf_util.h"
#include "elf_dwarf_abbrev.h"


typedef struct {
	DwFormType	type;
	uint32		offset;
	uint32		size;
	char		*typename;
	union {
		uint32	addr;
		uint8	const8;
		uint16	const16;
		uint32	const32;
		uint64	const64;
		sint64	scont64;
		uint8	ref8;
		uint16	ref16;
		uint32	ref32;
		uint64	ref64;
		uint32	sec_offset;
		bool	flag;
		char 	*string;
		struct {
			uint32 len;
			uint8 *ops;
		} op;
	} encoded;
} ElfDwarfAttributeType;

struct ElfDwarfDie;
typedef struct ElfDwarfDie {
	uint32				offset;
	uint32				abbrev_code;
	ElfDwarfAbbrevType	*abbrev_info;
	ElfPointerArrayType	*attribute;
	uint32				level;
	struct ElfDwarfDie	*parent;
	ElfPointerArrayType	*children;
} ElfDwarfDieType;

typedef struct {
	uint32				offset;
	uint32				length;
	uint16				version;
	uint32				abbrev_offset;
	uint8				pointer_size;
	ElfPointerArrayType	*dies;
} ElfDwarfCompilationUnitHeaderType;

extern Std_ReturnType elf_dwarf_info_load(uint8 *elf_data);
extern ElfDwarfCompilationUnitHeaderType *elf_dwarf_info_alloc_empty_ElfDwarfCompilationUnitHeader(void);

extern ElfPointerArrayType *elf_dwarf_info_get(void);

extern uint32 elf_dwarf_info_get_value(DwFormType form, ElfDwarfAttributeType *obj, uint32 *size);



#endif /* _ELF_DWARF_INFO_H_ */
