/*

 *	V850E2/FX4-4012リンカスクリプト
 */

DEFAULTS {
   heap_reserve = 8K
}

MEMORY
{
	INTROM(raxi) : org = 0x00000000, len = 2M
	INTRAM(wax)  : org = 0xFEDDC000, len = 144k
}

SECTIONS
{
    provide(_hardware_init_hook = 0);
    provide(_software_init_hook = 0);
    provide(_software_term_hook = 0);
    provide(_hardware_term_hook = 0);
    provide(_bsssecinib_table = 0);
    provide(_tnum_bsssec = 0);
    provide(_datasecinib_table = 0);
    provide(_tnum_datasec = 0);
    provide(_StartupHook = 0);
    provide(_ShutdownHook = 0);
    provide(_PreTaskHook = 0);
    provide(_PostTaskHook = 0);
    provide(_ErrorHook = 0);
    provide(_ProtectionHook = 0);

    .CODEFLASH   0x00000000 :
	.vector : AT(0)
	{
		*(.vector)
	} > INTROM
	.text :	
	{
		__text = . ;
		__tp = . ;
		*(.text)
		_etext = . ;
	} > INTROM

	.call_table_data ALIGN (4) :
	{
	PROVIDE(__ctbp = .);
	*(.call_table_data)
	} > INTROM
	
	.call_table_text :
	{
	*(.call_table_text)
	} > INTROM

	. = ALIGN(4);
	.rosdata : { *(.rosdata) } > INTROM

	. = ALIGN(4);
	.rodata : { *(.rodata) } > INTROM

	_textend = .;
    .syscall			                            : > INTROM
    .secinfo						    : > INTROM
    .fixaddr						    : > INTROM
    .fixtype						    : > INTROM
	. = ALIGN(4);
	__idata_start = . ;

	.data	:  AT(__idata_start) 
	{
		. = ALIGN(4);
		__data_start = . ;
		*(.data)
		. = ALIGN(8);
	} > INTRAM

	.sdabase  : AT(__idata_start + SIZEOF(.data))
	.sdata    : AT(__idata_start + SIZEOF(.data))
	{
		*(.sdata)
		_edata	=  . ;
	} > INTRAM

	__idata_end = __idata_start + SIZEOF(.data) + SIZEOF(.sdata);

	.sbss                  :
	{
	  . = ALIGN(4);
	  __bss_start = .;
		*(.sbss)
	} > INTRAM

	.bss	   :
	{
	  . = ALIGN(4);
	  *(.bss)
	  *(COMMON)
	  . = ALIGN(4);
	  __bss_end = .;
	} > INTRAM
	
	
	.tdata ALIGN (4) :
	{
		PROVIDE (__ep = .);
		*(.tbyte)
		*(.tcommon_byte)
		*(.tdata)
		*(.tbss)
		*(.tcommon)
	} > INTRAM
    .heap    ALIGN(8) PAD(heap_reserve)   : > INTRAM
	
	.comment 0 : { *(.comment) }	
	.debug			0 : { *(.debug) }
	.line			0 : { *(.line) }
	.debug_srcinfo	0 : { *(.debug_srcinfo) }
	.debug_sfnames	0 : { *(.debug_sfnames) }
	.debug_aranges	0 : { *(.debug_aranges) }
	.debug_pubnames 0 : { *(.debug_pubnames) }	
}
/* provide a pointer for the stack */
 