MEMORY
{
	KERNEL_ROM(raxi) : org = 0x00000000, len = 512k
	USER1_ROM(raxi)  : org = 0x00100000, len = 512k
	USER2_ROM(raxi)  : org = 0x00200000, len = 512k
	USER_ROM(raxi)   : org = 0x00300000, len = 512k
	KERNEL_RAM(wax)  : org = 0x06000000, len = 512k
	USER1_RAM(wax)   : org = 0x07000000, len = 512k
	USER2_RAM(wax)   : org = 0x08000000, len = 512k
	USER_RAM(wax)    : org = 0x09000000, len = 512k
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
		kernel_*.o(.text)
		*(.text_kernel)
	} > KERNEL_ROM

	.text_user1 :	
	{
		user1_*.o(.text)
		*(.text_user1)
	} > USER1_ROM

	.text_user2 :	
	{
		user2_*.o(.text)
		*(.text_user2)
	} > USER2_ROM

	.rodata_kernel : 
	{ 
		start.o(.rodata)
		timer.o(.rodata)
		interrupt.o(.rodata)
		interrupt_asm.o(.rodata)
		interrupt_table.o(.rodata)
		main.o(.rodata)
		kernel_*.o(.rodata)
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
		kernel_*.o(.data)
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
		kernel_*.o(.bss)
		*(.bss_kernel)
		. = ALIGN(4);
		__bss_kernel_end = .;
	} > KERNEL_RAM

	.data_user1	:
	{
		. = ALIGN(4);
		__data_user1_start = . ;
		user1_*.o(.data)
		*(.data_user1)
		. = ALIGN(4);
		_edata1_user	=  . ;
	} > USER1_RAM AT>USER1_ROM
	__idata_user1_start = LOADADDR(.data_user1);
	__idata_user1_end = LOADADDR(.data_user1) + SIZEOF(.data_user1);

	.data_user2	:
	{
		. = ALIGN(4);
		__data_user2_start = . ;
		user2_*.o(.data)
		*(.data_user2)
		. = ALIGN(4);
		_edata2_user	=  . ;
	} > USER2_RAM AT>USER2_ROM
	__idata_user2_start = LOADADDR(.data_user2);
	__idata_user2_end = LOADADDR(.data_user2) + SIZEOF(.data_user2);

	.data_user	:
	{
		. = ALIGN(4);
		__data_user_start = . ;
		user_*.o(.data)
		*(.data_user)
		. = ALIGN(4);
		_edata_user	=  . ;
	} > USER_RAM AT>USER_ROM
	__idata_user_start = LOADADDR(.data_user);
	__idata_user_end = LOADADDR(.data_user) + SIZEOF(.data_user);

	.rodata_user1 : 
	{ 
		user1_*.o(.rodata)
		*(.rodata_user1)
	} > USER1_ROM

	.rodata_user2 : 
	{ 
		user2_*.o(.rodata)
		*(.rodata_user2)
	} > USER2_ROM

	.rodata_user : 
	{ 
		user_*.o(.rodata)
		*(.rodata_user)
	} > USER_ROM

	.bss_user1	   :
	{
		. = ALIGN(4);
		__bss_user1_start = .;
		user1_*.o(.bss)
		*(.bss_user1)
		. = ALIGN(4);
		__bss_user1_end = .;
	} > USER1_RAM

	.bss_user2	   :
	{
		. = ALIGN(4);
		__bss_user2_start = .;
		user2_*.o(.bss)
		*(.bss_user2)
		. = ALIGN(4);
		__bss_user2_end = .;
	} > USER2_RAM

	.bss_user	   :
	{
		. = ALIGN(4);
		__bss_user_start = .;
		user_*.o(.bss)
		*(.bss_user)
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

	.bss_noclr_user1	   :
	{
		. = ALIGN(4);
		__bss_noclr_user1_start = .;
		*(.bss_noclr_user1)
		*(COMMON)
		. = ALIGN(4);
	  __bss_user1_end = .;
	} > USER1_RAM

	.bss_noclr_user2	   :
	{
		. = ALIGN(4);
		__bss_noclr_user2_start = .;
		*(.bss_noclr_user2)
		*(COMMON)
		. = ALIGN(4);
	  __bss_user2_end = .;
	} > USER2_RAM

	.bss_noclr_user	   :
	{
		. = ALIGN(4);
		__bss_noclr_user_start = .;
		*(.bss_noclr_user)
		*(COMMON)
		. = ALIGN(4);
	  __bss_user_end = .;
	} > USER_RAM

	.comment 0 : { *(.comment) }	
	.debug			0 : { *(.debug) }
	.line			0 : { *(.line) }
	.debug_srcinfo	0 : { *(.debug_srcinfo) }
	.debug_sfnames	0 : { *(.debug_sfnames) }
	.debug_aranges	0 : { *(.debug_aranges) }
	.debug_pubnames 0 : { *(.debug_pubnames) }	
}



