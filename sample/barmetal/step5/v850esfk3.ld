MEMORY
{
	INTROM(raxi) : org = 0x00000000, len = 512k
	INTRAM(wax)  : org = 0x06FF7000, len = 512k
	INTMMAP(wax)  : org = 0x07FF0000, len = 512k
};


OUTPUT_FORMAT("elf32-v850-rh850", "elf32-v850-rh850",
	      "elf32-v850-rh850")
OUTPUT_ARCH(v850-rh850)

SECTIONS
{
	.vector : AT(0)
	{
		*(.vectors)
	} > INTROM

	.text :	
	{
		__text = . ;
		__tp = . ;
		*(.text)
		_etext = . ;
	} > INTROM

	.rodata : { *(.rodata) } > INTROM
	. = ALIGN(4);
	__idata_start = . ;
	
	.data	:  AT(__idata_start) 
	{
		. = ALIGN(4);
		__data_start = . ;
		*(.data)
		. = ALIGN(4);
		_edata	=  . ;
	} > INTRAM

	__idata_end = __idata_start + SIZEOF(.data);
	
	.bss	   :
	{
	  . = ALIGN(4);
	  __bss_start = .;
	  *(.bss)
	  *(COMMON)
	  . = ALIGN(4);
	  __bss_end = .;
	} > INTRAM

	.bss_noclr		:
	{
		. = ALIGN(4);
		*(.bss_noclr)
		. = ALIGN(4);
	} > INTRAM

	.mmap_section	: 
	{
		. = ALIGN(4);
		*(.mmap_section)
		. = ALIGN(4);
	} > INTMMAP


	.comment 0 : { *(.comment) }	
	.debug			0 : { *(.debug) }
	.line			0 : { *(.line) }
	.debug_srcinfo	0 : { *(.debug_srcinfo) }
	.debug_sfnames	0 : { *(.debug_sfnames) }
	.debug_aranges	0 : { *(.debug_aranges) }
	.debug_pubnames 0 : { *(.debug_pubnames) }	
}



