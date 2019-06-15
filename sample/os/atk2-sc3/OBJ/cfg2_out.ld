OUTPUT_FORMAT("elf32-v850-rh850","elf32-v850-rh850","elf32-v850-rh850")
OUTPUT_ARCH(v850-rh850)

MEMORY {
	INTROM_PE1 : ORIGIN = 0x0, LENGTH = 0x400000
	INTROM_PE2 : ORIGIN = 0x800000, LENGTH = 0x200000
	INTRAM : ORIGIN = 0xfebd0000, LENGTH = 0x30000
	INTRAM2 : ORIGIN = 0x10000000, LENGTH = 0x30000
}

PROVIDE(_hardware_init_hook = 0);
PROVIDE(_software_init_hook = 0);
PROVIDE(_software_term_hook = 0);
PROVIDE(_bsssecinib_table = 0);
PROVIDE(_tnum_bsssec = 0);
PROVIDE(_datasecinib_table = 0);
PROVIDE(_tnum_datasec = 0);

PROVIDE(___start_text_NT_osap2 = 0xfffffff0);
PROVIDE(___limit_text_NT_osap2 = 0xfffffff0);
PROVIDE(___start_sram_NT_osap2_21 = 0xfffffff0);
PROVIDE(___limit_sram_NT_osap2_21 = 0xfffffff0);
PROVIDE(___start_ram_NT_osap2 = 0xfffffff0);
PROVIDE(___limit_ram_NT_osap2 = 0xfffffff0);
PROVIDE(___start_sram_NT_osap2 = 0xfffffff0);
PROVIDE(___limit_sram_NT_osap2 = 0xfffffff0);
PROVIDE(___start_ram_NT_osap2_1_ffffffff = 0xfffffff0);
PROVIDE(___limit_ram_NT_osap2_1_ffffffff = 0xfffffff0);
PROVIDE(___start_sram_NT_osap2_1_ffffffff = 0xfffffff0);
PROVIDE(___limit_sram_NT_osap2_1_ffffffff = 0xfffffff0);
PROVIDE(___start_text_NT_osap1 = 0xfffffff0);
PROVIDE(___limit_text_NT_osap1 = 0xfffffff0);
PROVIDE(___start_sram_NT_osap1_21 = 0xfffffff0);
PROVIDE(___limit_sram_NT_osap1_21 = 0xfffffff0);
PROVIDE(___start_ram_NT_osap1 = 0xfffffff0);
PROVIDE(___limit_ram_NT_osap1 = 0xfffffff0);
PROVIDE(___start_sram_NT_osap1 = 0xfffffff0);
PROVIDE(___limit_sram_NT_osap1 = 0xfffffff0);
PROVIDE(___start_ram_NT_osap1_2_ffffffff = 0xfffffff0);
PROVIDE(___limit_ram_NT_osap1_2_ffffffff = 0xfffffff0);
PROVIDE(___start_sram_NT_osap1_2_ffffffff = 0xfffffff0);
PROVIDE(___limit_sram_NT_osap1_2_ffffffff = 0xfffffff0);


PROVIDE(___start_text_shared = 0xfffffff0);
PROVIDE(___limit_text_shared = 0xfffffff0);
PROVIDE(___start_sram_shared_21 = 0xfffffff0);
PROVIDE(___limit_sram_shared_21 = 0xfffffff0);
PROVIDE(___start_ram_shared = 0xfffffff0);
PROVIDE(___limit_ram_shared = 0xfffffff0);
PROVIDE(___start_sram_shared = 0xfffffff0);
PROVIDE(___limit_sram_shared = 0xfffffff0);
PROVIDE(___start_srpw_all = 0xfffffff0);
PROVIDE(___limit_srpw_all = 0xfffffff0);
PROVIDE(___start_ssrpw_all = 0xfffffff0);
PROVIDE(___limit_ssrpw_all = 0xfffffff0);
STARTUP(start.o);
ENTRY(__reset);

SECTIONS {
	.vector : AT(0) {
		*("*.reset.text")
		*("*.text")
		. += 1;
		. = ALIGN(16);
	} > INTROM_PE1

	.INTROM_PE2_text_kernel_0_45 : {
		___start_INTROM_PE2_text_kernel_0_45 = .;
		___start_INTROM_PE2_text_kernel__0_45 = .;
		*(.non_use_pe_rom) 
		___end_INTROM_PE2_text_kernel__0_45 = .;
		___end_INTROM_PE2_text_kernel_0_45 = .;
	} > INTROM_PE2
	. = ALIGN(16);
	___limit_INTROM_PE2_text_kernel__0_45 = .;
	___limit_INTROM_PE2_text_kernel_0_45 = .;

	.INTROM_PE2_align ALIGN(16) : {
		__INTROM_PE2_align = .;
	} > INTROM_PE2

	.shared_user_stack8 (NOLOAD) : {
		___start_shared_user_stack8 = .;
		Os_Lcfg.o(.shared_user_stack.8) 
		___end_shared_user_stack8 = .;
	} > INTRAM
	. = ALIGN(16);
	___limit_shared_user_stack8 = .;

	.shared_user_stack9 ALIGN(16) (NOLOAD) : {
		___start_shared_user_stack9 = .;
		Os_Lcfg.o(.shared_user_stack.9) 
		___end_shared_user_stack9 = .;
	} > INTRAM
	. = ALIGN(16);
	___limit_shared_user_stack9 = .;

	.shared_user_stack6 ALIGN(16) (NOLOAD) : {
		___start_shared_user_stack6 = .;
		Os_Lcfg.o(.shared_user_stack.6) 
		___end_shared_user_stack6 = .;
	} > INTRAM
	. = ALIGN(16);
	___limit_shared_user_stack6 = .;

	.user_stackTask3 ALIGN(16) (NOLOAD) : {
		___start_user_stackTask3 = .;
		Os_Lcfg.o(.user_stack.Task3) 
		___end_user_stackTask3 = .;
	} > INTRAM
	. = ALIGN(16);
	___limit_user_stackTask3 = .;

	.user_stackTask2 ALIGN(16) (NOLOAD) : {
		___start_user_stackTask2 = .;
		Os_Lcfg.o(.user_stack.Task2) 
		___end_user_stackTask2 = .;
	} > INTRAM
	. = ALIGN(16);
	___limit_user_stackTask2 = .;

	.shared_user_stack4 ALIGN(16) (NOLOAD) : {
		___start_shared_user_stack4 = .;
		Os_Lcfg.o(.shared_user_stack.4) 
		___end_shared_user_stack4 = .;
	} > INTRAM
	. = ALIGN(16);
	___limit_shared_user_stack4 = .;

	.user_stackTask12 ALIGN(16) (NOLOAD) : {
		___start_user_stackTask12 = .;
		Os_Lcfg.o(.user_stack.Task12) 
		___end_user_stackTask12 = .;
	} > INTRAM
	. = ALIGN(16);
	___limit_user_stackTask12 = .;

	.user_stackTask11 ALIGN(16) (NOLOAD) : {
		___start_user_stackTask11 = .;
		Os_Lcfg.o(.user_stack.Task11) 
		___end_user_stackTask11 = .;
	} > INTRAM
	. = ALIGN(16);
	___limit_user_stackTask11 = .;

	.user_stackTask10 ALIGN(16) (NOLOAD) : {
		___start_user_stackTask10 = .;
		Os_Lcfg.o(.user_stack.Task10) 
		___end_user_stackTask10 = .;
	} > INTRAM
	. = ALIGN(16);
	___limit_user_stackTask10 = .;

	.shared_user_stack15 ALIGN(16) (NOLOAD) : {
		___start_shared_user_stack15 = .;
		Os_Lcfg.o(.shared_user_stack.15) 
		___end_shared_user_stack15 = .;
	} > INTRAM
	. = ALIGN(16);
	___limit_shared_user_stack15 = .;

	.sbss_kernel_0_ffffffff ALIGN(16) (NOLOAD) : {
		__gp = . + 32K;
		__tp = . + 32K;
		PROVIDE (__ep = .);
		PROVIDE(__ctbp = .);
		___start_ssrpw_all = .;
		___start_sram_kernel_0_ffffffff = .;
		___start_sbss_kernel_0_ffffffff = .;
		___start_sbss_kernel__0_ffffffff = .;
		*(.srpw_bss_kernel) 
		___end_sbss_kernel__0_ffffffff = .;
		___end_sbss_kernel_0_ffffffff = .;
		___end_ssrpw_all = .;
	} > INTRAM
	. = ALIGN(16);
	___limit_sbss_kernel__0_ffffffff = .;
	___limit_sbss_kernel_0_ffffffff = .;
	___limit_sram_kernel_0_ffffffff = .;
	___limit_ssrpw_all = .;

	.INTRAM_align ALIGN(16) : {
		__INTRAM_align = .;
	} > INTRAM

	.stack_section : {
		*(.stack_section)
	} > INTRAM

	.dummy_rom_for_cpptest : {
		*(.ctors.65535) *(.eh_frame)
	} > INTROM_PE1
	. = ALIGN(64);

	.dummy_ram_for_cpptest : {
		_end = .;
	} > INTRAM
	. = ALIGN(64);

	/DISCARD/ : {
		*(.rel.dyn)
	}

	/* DWARF debug sections.
	Symbols in the DWARF debugging sections are relative to 
	the beginning of the section so we begin them at 0.  */

	/* DWARF 1 */
	.debug          0 : { *(.debug) }
	.line           0 : { *(.line) }

	/* GNU DWARF 1 extensions */
	.debug_srcinfo  0 : { *(.debug_srcinfo) }
	.debug_sfnames  0 : { *(.debug_sfnames) }

	/* DWARF 1.1 and DWARF 2 */
	.debug_aranges  0 : { *(.debug_aranges) }
	.debug_pubnames 0 : { *(.debug_pubnames) }

	/* DWARF 2 */
	.debug_info     0 : { *(.debug_info) }
	.debug_abbrev   0 : { *(.debug_abbrev) }
	.debug_line     0 : { *(.debug_line) }
	.debug_frame    0 : { *(.debug_frame) }
	.debug_str      0 : { *(.debug_str) }
	.debug_loc      0 : { *(.debug_loc) }
	.debug_macinfo  0 : { *(.debug_macinfo) }
	.debug_ranges   0 : { *(.debug_ranges) }

	/* SGI/MIPS DWARF 2 extensions */
	.debug_weaknames 0 : { *(.debug_weaknames) }
	.debug_funcnames 0 : { *(.debug_funcnames) }
	.debug_typenames 0 : { *(.debug_typenames) }
	.debug_varnames  0 : { *(.debug_varnames) }

}
 
