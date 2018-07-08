MEMORY
{
	KERNEL_ROM(raxi) : org = 0x00000000, len = 512k
	KERNEL_RAM(wax)  : org = 0x06FF0000, len = 512k
	USER_ROM(raxi)   : org = 0x00100000, len = 512k
	USER_RAM(wax)    : org = 0x07FF0000, len = 512k
};


OUTPUT_FORMAT("elf32-v850-rh850", "elf32-v850-rh850",
	      "elf32-v850-rh850")
OUTPUT_ARCH(v850-rh850)

SECTIONS
{
	.vector : AT(0)
	{
		*(.vectors)
	} > KERNEL_ROM

	.text_kernel :	
	{
		__tp = . ;
		start.o(.text)
		timer.o(.text)
		interrupt.o(.text)
		interrupt_asm.o(.text)
		interrupt_table.o(.text)
		main.o(.text)
		*(.text_kernel)
	} > KERNEL_ROM

	.text_user :	
	{
		user_*.o(.text)
	} > USER_ROM

	.rodata_kernel : 
	{ 
		start.o(.rodata)
		timer.o(.rodata)
		interrupt.o(.rodata)
		interrupt_asm.o(.rodata)
		interrupt_table.o(.rodata)
		main.o(.rodata)
		*(.rodata_kernel)
	} > KERNEL_ROM
	. = ALIGN(4);

	__idata_kernel_start = . ;
	
	.data_kernel	:
	{
		. = ALIGN(4);
		__data_kernel_start = . ;
		start.o(.data)
		timer.o(.data)
		interrupt.o(.data)
		interrupt_asm.o(.data)
		interrupt_table.o(.data)
		main.o(.data)
		*(.data_kernel)
		. = ALIGN(4);
		_edata_kernel	=  . ;
	} > KERNEL_RAM AT>KERNEL_ROM
	__idata_kernel_start = LOADADDR(.data_kernel);
	__idata_kernel_end = __idata_kernel_start + SIZEOF(.data_kernel);

	.bss_kernel	   :
	{
		. = ALIGN(4);
		__bss_kernel_start = .;
		start.o(.bss)
		timer.o(.bss)
		interrupt.o(.bss)
		interrupt_asm.o(.bss)
		interrupt_table.o(.bss)
		main.o(.bss)
		interrupt_table.o(.bss)
		*(.bss_kernel)
		. = ALIGN(4);
		__bss_kernel_end = .;
	} > KERNEL_RAM

	.data_user	:
	{
		. = ALIGN(4);
		__data_user_start = . ;
		user_*.o(.data)
		. = ALIGN(4);
		_edata_user	=  . ;
	} > USER_RAM AT>USER_ROM
	__idata_user_start = LOADADDR(.data_user);
	__idata_user_end = LOADADDR(.data_user) + SIZEOF(.data_user);

	.rodata_user : 
	{ 
		user_*.o(.rodata)
	} > USER_ROM


	.bss_user	   :
	{
		. = ALIGN(4);
		__bss_user_start = .;
		user_*.o(.bss)
		. = ALIGN(4);
		__bss_user_end = .;
	} > USER_RAM

	.bss_noclr_kernel	   :
	{
		. = ALIGN(4);
		__bss_noclr_kernel_start = .;
		*(.bss_noclr_kernel)
		*(COMMON)
		. = ALIGN(4);
	  __bss_kernel_end = .;
	} > KERNEL_RAM



	.comment 0 : { *(.comment) }	
	.debug			0 : { *(.debug) }
	.line			0 : { *(.line) }
	.debug_srcinfo	0 : { *(.debug_srcinfo) }
	.debug_sfnames	0 : { *(.debug_sfnames) }
	.debug_aranges	0 : { *(.debug_aranges) }
	.debug_pubnames 0 : { *(.debug_pubnames) }	
}



