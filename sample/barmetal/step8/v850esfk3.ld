MEMORY
{
	INTROM(raxi) : org = 0x00000000, len = 512k
	INTRAM(wax)  : org = 0x06FF7000, len = 512k
	INTMMAP(wax)  : org = 0x07FF0000, len = 512k
	ADEV(wax)  : org = 0xDEAD0000, len = 1k
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
    .init :
    {
        KEEP (*(.init))
    }
	.text :	
	{
		__text = . ;
		__tp = . ;
		*(.text .stub .text.* .gnu.linkonce.t.*)
		_etext = . ;
	} > INTROM
    .fini :
    {
        KEEP (*(.fini))
    }
    _etext = . ;
    PROVIDE (etext = .) ;

    .gcc_except_table : { *(.gcc_except_table) }
    .ctors :
    {
        KEEP (*crtbegin.o(.ctors))
        KEEP (*(EXCLUDE_FILE (*crtend.o ) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*(.ctors))
    } > INTROM
	___ctors_end = .;
	___dtors = .;
    .dtors :
    {
        KEEP (*crtbegin.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend.o ) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*(.dtors))
    } > INTROM
	___dtors_end = .;

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
    _end = . ;
    __bss_end = . ;

	.mmap_section	: 
	{
		. = ALIGN(4);
		*(.mmap_section)
		. = ALIGN(4);
	} > INTMMAP

	.athrill_device_section	: 
	{
		. = ALIGN(4);
		*(.athrill_device_section)
		. = ALIGN(4);
	} > ADEV


	.comment 0 : { *(.comment) }	
	.debug			0 : { *(.debug) }
	.line			0 : { *(.line) }
	.debug_srcinfo	0 : { *(.debug_srcinfo) }
	.debug_sfnames	0 : { *(.debug_sfnames) }
	.debug_aranges	0 : { *(.debug_aranges) }
	.debug_pubnames 0 : { *(.debug_pubnames) }	
}



