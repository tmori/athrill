#include "loader/loader.h"
#include "cpu.h"
#include "mpu.h"
#include "mpu_ops.h"
#include "loader/elf.h"
#include "elf_section.h"
#include "elf_dwarf_line.h"
#include "elf_dwarf_abbrev.h"
#include "elf_dwarf_info.h"
#include "elf_dwarf_data_type.h"
#include <string.h>
#include <stdio.h>

static Std_ReturnType Elf_Check(const Elf32_Ehdr *elf_image);
static Std_ReturnType Elf_LoadProgram(const Elf32_Ehdr *elf_image);

Std_ReturnType binary_load(uint8 *binary_data, uint32 load_addr, uint32 binary_data_len)
{
	Std_ReturnType err;
	uint8 *data = NULL;

	err = mpu_get_pointer(CPU_CONFIG_CORE_ID_0, load_addr, &data);
	if (err != STD_E_OK) {
		return err;
	}

	memcpy(data, binary_data, binary_data_len);

	return STD_E_OK;
}

Std_ReturnType elf_load(uint8 *elf_data)
{
	Std_ReturnType err;

	err = Elf_Check((const Elf32_Ehdr*)elf_data);
	if (err != STD_E_OK) {
		return err;
	}
	err = Elf_LoadProgram((const Elf32_Ehdr*)elf_data);
	if (err != STD_E_OK) {
		return err;
	}
	err = elf_symbol_load(elf_data);
	if (err != STD_E_OK) {
		return err;
	}
	err = elf_dwarf_line_load(elf_data);
	if (err != STD_E_OK) {
		return err;
	}
	err = elf_dwarf_abbrev_load(elf_data);
	if (err != STD_E_OK) {
		return err;
	}
	err = elf_dwarf_info_load(elf_data);
	if (err != STD_E_OK) {
		return err;
	}
	dwarf_build_data_type_set();


	return err;
}


static Std_ReturnType Elf_Check(const Elf32_Ehdr *elf_image)
{
	// magic check
	if (IS_ELF(*elf_image) == FALSE) {
		return STD_E_INVALID;
	}
	if (elf_image->e_ident[EI_CLASS] != ELFCLASS32) {
		return STD_E_INVALID;
	}
	if (elf_image->e_ident[EI_DATA] != ELFDATA2LSB) {
		return STD_E_INVALID;
	}
	if (elf_image->e_ident[EI_VERSION] != EV_CURRENT) {
		return STD_E_INVALID;
	}
	if (elf_image->e_ident[EI_OSABI] != ELFOSABI_SYSV) {
		return STD_E_INVALID;
	}
	if (elf_image->e_ident[EI_ABIVERSION] != 0U) {
		return STD_E_INVALID;
	}
	if (elf_image->e_type != ET_EXEC) {
		return STD_E_INVALID;
	}
	if (elf_image->e_machine != ELF_MACHINE_TYPE) {
		return STD_E_INVALID;
	}
	if (elf_image->e_version != EV_CURRENT) {
		return STD_E_INVALID;
	}
	return STD_E_OK;
}
static Std_ReturnType Elf_LoadProgram(const Elf32_Ehdr *elf_image)
{
	uint32_t i;
	Elf32_Phdr *phdr;
	Std_ReturnType err;
	uint8 *ptr;

	for (i = 0; i < elf_image->e_phnum; i++) {
		phdr = (Elf32_Phdr*) (
				((uint8_t*)elf_image)
				+ elf_image->e_phoff
				+ (elf_image->e_phentsize * i)
				);
		if (phdr->p_type != PT_LOAD) {
			continue;
		}
		/*
		 * ROM領域のみロードする．
		 */
		err = mpu_get_pointer(CPU_CONFIG_CORE_ID_0, phdr->p_paddr, &ptr);
		if (err != STD_E_OK) {
			printf("Invalid elf file: can not load addr=0x%x\n", phdr->p_vaddr);
			return STD_E_INVALID;
		}
		memcpy(ptr,
				( ((uint8_t*)elf_image) + phdr->p_offset ),
				phdr->p_filesz);

		printf("Elf loading was succeeded:0x%x - 0x%x : %u.%u KB\n", phdr->p_paddr, phdr->p_paddr + phdr->p_filesz, phdr->p_filesz/1024, phdr->p_filesz % 1024);

	}
	return 0;
}
