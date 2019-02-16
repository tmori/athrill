#ifndef _ELF_DWARF_DATA_TYPE_H_
#define _ELF_DWARF_DATA_TYPE_H_

#include "std_types.h"
#include "elf_dwarf_util.h"
#include "elf_dwarf_info.h"

typedef enum {
	DATA_TYPE_BASE = 0,
	DATA_TYPE_STRUCT,
	DATA_TYPE_UNION,
	DATA_TYPE_ARRAY,
	DATA_TYPE_POINTER,
	DATA_TYPE_TYPEDEF,
	DATA_TYPE_ENUM,
	DATA_TYPE_VARIABLE,
	DATA_TYPE_SUBPROGRAM,
	DATA_TYPE_CLASS,
	DATA_TYPE_NUM,
} DwarfDataEnumType;

extern void *dwarf_alloc_data_type(DwarfDataEnumType type);
extern void *dwarf_search_data_type(DwarfDataEnumType type, char *dirname, char *filename, char *typename);
extern void *dwarf_search_data_type_from_die(DwarfDataEnumType type, uint32 die_off);
typedef struct {
	ElfDwarfDieType		*die;
	DwarfDataEnumType	type;
	char				*dirname;
	char				*filename;
	char				*typename;
	uint32				size;
} DwarfDataType;
extern void dwarf_register_data_type(DwarfDataType *entry);
extern ElfPointerArrayType	*dwarf_get_data_types(DwarfDataEnumType type);
extern void dwarf_build_data_type_set(void);
extern DwarfDataType *elf_dwarf_get_data_type(uint32 debug_info_offset);

extern Std_ReturnType dwarf_get_real_type_offset(uint32 offset, uint32 *retp);

/*
 * ex. uint32
 *
 * info.type = DATA_TYPE_BASE
 * info.name = "uint32"
 * info.size = 4
 */
typedef struct {
	DwarfDataType	info;
	DwAteType		encoding;
} DwarfDataBaseType;

/*
 * ex. uint32 *
 *
 * info.type = DATA_TYPE_POINTER
 * info.name = NULL
 * info.size = 4
 * info.ref = (ref of uint32)
 */
typedef struct {
	DwarfDataType	info;
	DwarfDataType	*ref;
	bool			is_valid_ref_debug_info_offset;
	uint32			ref_debug_info_offset;
} DwarfDataPointerType;


/*
 * ex. typedef struct a a_t;
 *
 * info.type = DATA_TYPE_TYPEDEF
 * info.name = "a_t"
 * info.size = sizeof (struct a)
 * info.ref = (ref of struct a )
 */
typedef struct {
	DwarfDataType	info;
	DwarfDataType	*ref;
	bool			is_valid_ref_debug_info_offset;
	uint32			ref_debug_info_offset;
} DwarfDataTypedefType;


typedef struct {
	DwarfDataType		info;
	ElfPointerArrayType	*members;
} DwarfDataStructType;

typedef struct {
	char				*name;
	char				*linkage_name;
	uint32				off;
	DwarfDataType		*ref;
	bool				is_valid_ref_debug_info_offset;
	uint32				ref_debug_info_offset;
} DwarfDataStructMember;
extern void dwarf_add_struct_member(DwarfDataStructType *obj, DwarfDataStructMember *member);


typedef struct {
	DwarfDataType		info;
	ElfPointerArrayType	*members;
} DwarfDataEnumulatorType;
typedef struct {
	char				*name;
	uint32				const_value;
} DwarfDataEnumMember;
extern void dwarf_add_enum_member(DwarfDataEnumulatorType *obj, char *name, uint32 const_value);


typedef struct {
	DwarfDataType			info;
	DwarfDataType			*ref;
	bool					is_valid_ref_debug_info_offset;
	uint32					ref_debug_info_offset;
	DwarfUint32ArrayType	*dimension;
} DwarfDataArrayType;

typedef struct {
	DwarfDataType			info;
	DwarfDataType			*ref;
	bool					is_valid_ref_debug_info_offset;
	uint32					ref_debug_info_offset;
} DwarfDataVariableType;

typedef struct {
	char					*name;
	bool					isSupported;
	sint32					stackLocOff;
	ElfDwarfAttributeType	*DW_AT_location;
	bool					is_valid_ref_debug_info_offset;
	uint32					ref_debug_info_offset;
	DwarfDataType			*ref;
	uint8					location_op;
} DwarfLocalVariableType;

typedef struct {
	DwarfDataType			info;
	uint32					frame_loc_offset;
	ElfPointerArrayType		*variables;
} DwarfDataSubprogramType;
extern void dwarf_add_subprogram_variable(DwarfDataSubprogramType *obj, DwarfLocalVariableType *org_val);

extern DwarfDataSubprogramType *elf_dwarf_search_subprogram(char *funcname);
extern DwarfLocalVariableType *elf_dwarf_search_local_variable(DwarfDataSubprogramType *subprogram, char *local_variable);

extern ElfDwarfDieType *dwarf_get_die(uint32 offset);

#endif /* _ELF_DWARF_DATA_TYPE_H_ */
