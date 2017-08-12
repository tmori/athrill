#include "concrete_executor/util/dbg_print_data_type.h"
#include "dwarf/data_type/elf_dwarf_data_type.h"
#include "dwarf/data_type/elf_dwarf_base_type.h"
#include "dwarf/elf_section.h"
#include <stdio.h>

typedef struct {
	uint32 vaddr;
	uint32 level;
} PrintControlType;

static bool print_any_data_type(PrintControlType *ctrl, DwarfDataType *obj, uint8 *top_addr, uint32 off);
static void print_ref_typename(PrintControlType *ctrl, DwarfDataType *type);
static uint32 get_ref_typesize(PrintControlType *ctrl, DwarfDataType *type);

static void print_base_type_address(uint8 *addr, uint32 size)
{
	uint32 value;
	value = elf_get_data32(addr, 0);
	printf("0x%x", value);
	return;
}
static void print_base_type_boolean(uint8 *addr, uint32 size)
{
	uint32 value = 0;
	switch (size) {
	case 1:
		value = elf_get_data8(addr, 0);
		break;
	case 2:
		value = elf_get_data16(addr, 0);
		break;
	case 4:
		value = elf_get_data32(addr, 0);
		break;
	default:
		break;
	}
	if (value == 0) {
		printf("FALSE");
	}
	else {
		printf("TRUE");
	}
	return;
}
static void print_base_type_signed(uint8 *addr, uint32 size)
{
	sint32 value = 0;
	sint64 value64 = 0;
	switch (size) {
	case 1:
		value = ((sint8)elf_get_data8(addr, 0));
		break;
	case 2:
		value = ((sint16)elf_get_data16(addr, 0));
		break;
	case 4:
		value = ((sint32)elf_get_data32(addr, 0));
		break;
	case 8:
		value64 = ((sint64)elf_get_data64(addr, 0));
		printf("%I64d", value64);
		return;
	default:
		break;
	}
	printf("%d", value);
	if (size == 1) {
		printf("(%c)", value);
	}
	return;
}
static void print_base_type_unsigned(uint8 *addr, uint32 size)
{
	sint32 value = 0;
	uint64 value64 = 0;
	switch (size) {
	case 1:
		value = ((uint8)elf_get_data8(addr, 0));
		break;
	case 2:
		value = ((uint16)elf_get_data16(addr, 0));
		break;
	case 4:
		value = ((uint32)elf_get_data32(addr, 0));
		break;
	case 8:
		value64 = ((uint64)elf_get_data64(addr, 0));
		printf("%I64u", value64);
		return;
	default:
		break;
	}
	printf("%u", value);
	return;
}

static void print_base_type_data(PrintControlType *ctrl, DwarfDataBaseType *obj, uint8 *addr, uint32 off)
{
	uint32 type_size = obj->info.size;
	switch (obj->encoding) {
	case DW_ATE_address:
		print_base_type_address(&addr[off], type_size);
		break;
	case DW_ATE_boolean:
		print_base_type_boolean(&addr[off], type_size);
		break;
	case DW_ATE_signed:
	case DW_ATE_signed_char:
		print_base_type_signed(&addr[off], type_size);
		break;
	case DW_ATE_unsigned:
	case DW_ATE_unsigned_char:
		print_base_type_unsigned(&addr[off], type_size);
		break;
	case DW_ATE_float:
		//TODO
	case DW_ATE_complex_float:
	default:
		printf("Unknown base type");
		break;
	}
	printf(" (%s:%u) @ 0x%x(0x%x)\n", obj->info.typename, type_size, ctrl->vaddr + off, off);
	return;
}
static void print_space(PrintControlType *ctrl)
{
	int space_i;
	for (space_i = 0; space_i < (ctrl->level); space_i++) {
		printf("  ");
	}
	return;
}

static void print_struct_type_data(PrintControlType *ctrl, DwarfDataStructType *type, uint8 *top_addr, uint32 off)
{
	int i;

	printf("struct %s { \n", type->info.typename);
	for (i = 0; i < type->members->current_array_size; i++) {
		DwarfDataStructMember *memp = (DwarfDataStructMember *)type->members->data[i];
		print_space(ctrl);
		printf("%s = ", memp->name);
		(void)print_any_data_type(ctrl, memp->ref, top_addr, off + memp->off);
	}
	print_space(ctrl);
	printf("}\n");
	return;
}
static void print_typedef_type_data(PrintControlType *ctrl, DwarfDataTypedefType *type, uint8 *top_addr, uint32 off)
{
	printf("(typedef %s )", type->info.typename);
	print_any_data_type(ctrl, type->ref, top_addr, off);
	return;
}

static void print_ref_base_typename(PrintControlType *ctrl, DwarfDataBaseType *type)
{
	printf("%s", type->info.typename);
	return;
}
static void print_ref_enum_typename(PrintControlType *ctrl, DwarfDataEnumulatorType *type)
{
#if 0
	int i;
	printf("enum %s {\n", type->info.typename);
	for (i = 0; i < type->members->current_array_size; i++) {
		DwarfDataEnumMember *memp = (DwarfDataEnumMember *)type->members->data[i];
		print_space(ctrl);
		printf("%s\n", memp->name);
	}
	printf("}");
#else
	printf("enum %s", type->info.typename);
#endif
	return;
}

static void print_ref_pointer_typename(PrintControlType *ctrl, DwarfDataPointerType *type)
{
	if (type->ref == NULL) {
		printf("Unknown type");
		return;
	}
	print_ref_typename(ctrl, type->ref);
	printf(" *");
	return;
}
static void print_ref_typedef_typename(PrintControlType *ctrl, DwarfDataTypedefType *type)
{
	printf("%s", type->info.typename);
	return;
}
static void print_ref_struct_typename(PrintControlType *ctrl, DwarfDataStructType *type)
{
	printf("struct %s", type->info.typename);
	return;
}

static void print_ref_typename(PrintControlType *ctrl, DwarfDataType *type)
{
	switch (type->type) {
	case DATA_TYPE_BASE:
		print_ref_base_typename(ctrl, (DwarfDataBaseType *)type);
		break;
	case DATA_TYPE_ENUM:
		print_ref_enum_typename(ctrl, (DwarfDataEnumulatorType *)type);
		break;
	case DATA_TYPE_POINTER:
		print_ref_pointer_typename(ctrl, (DwarfDataPointerType *)type);
		break;
	case DATA_TYPE_TYPEDEF:
		print_ref_typedef_typename(ctrl, (DwarfDataTypedefType *)type);
		break;
	case DATA_TYPE_STRUCT:
		print_ref_struct_typename(ctrl, (DwarfDataStructType *)type);
		break;
	case DATA_TYPE_ARRAY:
		//TODO
		break;
	case DATA_TYPE_UNION:
		break;
	default:
		break;
	}
	return;
}


static uint32 get_ref_base_typesize(PrintControlType *ctrl, DwarfDataBaseType *type)
{
	return type->info.size;
}
static uint32 get_ref_enum_typesize(PrintControlType *ctrl, DwarfDataEnumulatorType *type)
{
	return type->info.size;
}
static uint32 get_ref_pointer_typesize(PrintControlType *ctrl, DwarfDataPointerType *type)
{
	return type->info.size;
}
static uint32 get_ref_typedef_typesize(PrintControlType *ctrl, DwarfDataTypedefType *type)
{
	return get_ref_typesize(ctrl, type->ref);
}
static uint32 get_ref_struct_typesize(PrintControlType *ctrl, DwarfDataStructType *type)
{
	return type->info.size;
}
static uint32 get_ref_array_typesize(PrintControlType *ctrl, DwarfDataArrayType *type)
{
	return type->info.size;
}
static uint32 get_ref_typesize(PrintControlType *ctrl, DwarfDataType *type)
{
	switch (type->type) {
	case DATA_TYPE_BASE:
		return get_ref_base_typesize(ctrl, (DwarfDataBaseType *)type);
	case DATA_TYPE_ENUM:
		return get_ref_enum_typesize(ctrl, (DwarfDataEnumulatorType *)type);
	case DATA_TYPE_POINTER:
		return get_ref_pointer_typesize(ctrl, (DwarfDataPointerType *)type);
	case DATA_TYPE_TYPEDEF:
		return get_ref_typedef_typesize(ctrl, (DwarfDataTypedefType *)type);
	case DATA_TYPE_STRUCT:
		return get_ref_struct_typesize(ctrl, (DwarfDataStructType *)type);
	case DATA_TYPE_ARRAY:
		return get_ref_array_typesize(ctrl, (DwarfDataArrayType *)type);
	case DATA_TYPE_UNION:
		//TODO
		break;
	default:
		break;
	}
	return 0;
}

static void print_pointer_type_data(PrintControlType *ctrl, DwarfDataPointerType *type, uint8 *top_addr, uint32 off)
{
	printf("(");
	print_ref_typename(ctrl, (DwarfDataType *)type);
	printf(": %u )", type->info.size);
	printf(" 0x%x", elf_get_data32(&top_addr[off], 0));
	printf("  @ 0x%x(0x%x)\n", ctrl->vaddr + off, off);
	return;
}

static void print_enum_type_data(PrintControlType *ctrl, DwarfDataEnumulatorType *type, uint8 *top_addr, uint32 off)
{
	int i;
	DwarfDataEnumMember *memp = NULL;
	uint32 value;
	uint32 type_size = type->info.size;

	switch (type_size) {
	case 1:
		value = elf_get_data8(&top_addr[off], 0);
		break;
	case 2:
		value = elf_get_data16(&top_addr[off], 0);
		break;
	case 4:
		value = elf_get_data32(&top_addr[off], 0);
		break;
	default:
		printf("unknown enum\n");
		return;
	}

	for (i = 0; i < type->members->current_array_size; i++) {
		memp = (DwarfDataEnumMember *)type->members->data[i];
		if (memp->const_value == value) {
			printf("%s.%s(%d)", type->info.typename, memp->name, memp->const_value);
			printf("  @ 0x%x(0x%x)\n", ctrl->vaddr + off, off);
			return;
		}
	}
	printf("unknown enum\n");

	return;
}

static void print_array_type_data(PrintControlType *ctrl, DwarfDataArrayType *type, uint8 *top_addr, uint32 off)
{
	uint32 roff;
	int i;
	int j;
	int dim_size;
	int total_num = 1;
	int typesize;
	static int stack_level = 0;
	static ElfPointerArrayType *stack = NULL;
	DwarfUint32ArrayType *dims;

	if (stack == NULL) {
		stack = elf_array_alloc();
	}
	if (stack_level >= stack->current_array_size) {
		dims = dwarf_uint32_array_alloc();
		elf_array_add_entry(stack, dims);
	}
	else {
		dims = stack->data[stack_level];
	}

	/*
	 * 初期化
	 */
	typesize = get_ref_typesize(ctrl, type->ref);
	dim_size = type->dimension->current_array_size;
	dims->current_array_size = 0;
	for (i = 0; i < dim_size; i++) {
		dwarf_uint32_array_add_entry(dims, 0);
		total_num *= type->dimension->data[i];
	}

	print_space(ctrl);
	printf("( ");
	print_ref_typename(ctrl, type->ref);
	printf(" ) = { \n");

	for (j = 0; j < total_num; j++) {
		roff = (j * typesize);
		//インデックス表示
		print_space(ctrl);
		printf("  ");
		for (i = 0; i < dim_size; i++) {
			printf("[%u]", dims->data[i]);
		}
		printf(" = ");
		//値表示
		ctrl->level++;
		stack_level++;
		print_any_data_type(ctrl, type->ref, top_addr, off + roff);
		stack_level--;
		ctrl->level--;
		//ctrl->current_addr += type->ref->size;

		//桁上げ計算
		for (i = dim_size - 1; i >= 0; i--) {
			dims->data[i]++;
			if (i == 0) {
				break;
			}
			else if (dims->data[i] >= type->dimension->data[i]) {
				dims->data[i] = 0;
			}
			else {
				break;
			}
		}

	}
	print_space(ctrl);
	printf("}\n");
	return;
}
static bool print_any_data_type(PrintControlType *ctrl, DwarfDataType *obj, uint8 *top_addr, uint32 off)
{
	bool ret = FALSE;

	switch (obj->type) {
	case DATA_TYPE_BASE:
		print_base_type_data(ctrl, (DwarfDataBaseType *)obj, top_addr, off);
		ret = TRUE;
		break;
	case DATA_TYPE_ENUM:
		print_enum_type_data(ctrl, (DwarfDataEnumulatorType *)obj, top_addr, off);
		break;
	case DATA_TYPE_POINTER:
		print_pointer_type_data(ctrl, (DwarfDataPointerType *)obj, top_addr, off);
		ret = TRUE;
		break;
	case DATA_TYPE_TYPEDEF:
		print_typedef_type_data(ctrl, (DwarfDataTypedefType *)obj, top_addr, off);
		ret = TRUE;
		break;
	case DATA_TYPE_STRUCT:
		ctrl->level++;
		print_struct_type_data(ctrl, (DwarfDataStructType *)obj, top_addr, off);
		ctrl->level--;
		ret = TRUE;
		break;
	case DATA_TYPE_ARRAY:
		//ctrl->level++;
		print_array_type_data(ctrl, (DwarfDataArrayType *)obj, top_addr, off);
		//ctrl->level--;
		ret = TRUE;
		break;
	case DATA_TYPE_UNION:
		//TODO
	default:
		break;
	}

	return ret;
}

bool print_variable_with_data_type(char *variable_name, uint32 vaddr, uint8 *top_addr, uint32 size)
{
	DwarfDataVariableType *variable;
	DwarfDataType *type;
	PrintControlType ctrl;

	variable = (DwarfDataVariableType *)dwarf_search_data_type(DATA_TYPE_VARIABLE, NULL, NULL, variable_name);
	if (variable == NULL || variable->ref == NULL) {
		return FALSE;
	}
	//printf("%s 0x%x ref=0x%x\n", variable->info.typename, variable->info.die->offset, variable->ref->die->offset);
	type = (DwarfDataType *)dwarf_search_data_type_from_die(variable->ref->type, variable->ref->die->offset);
	if (type == NULL) {
		return FALSE;
	}
	ctrl.vaddr = vaddr;
	ctrl.level = 0;
	printf("%s = ", variable_name);
	return print_any_data_type(&ctrl, type, top_addr, 0);
}

