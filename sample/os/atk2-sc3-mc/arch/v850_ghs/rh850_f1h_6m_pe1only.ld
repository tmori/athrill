/*
 *	RH850/F1L-2Mリンカスクリプト
 */

MEMORY
{
	INTROM(raxi) : org = 0x00000000, len = 4M
	INTROMPE2    : org = 0x00800000, len = 2M
	INTRAM(wax)  : org = 0xFEBD0000, len = 192k
}

SECTIONS
{
    provide(_hardware_init_hook = 0);
    provide(_software_init_hook = 0);
    provide(software_term_hook = 0);

    /* =begin modified for SC3 */
    provide(_bsssecinib_table = 0);
    provide(_tnum_bsssec = 0);
    provide(_datasecinib_table = 0);
    provide(_tnum_datasec = 0);
    /* =end modified for SC3 */

    .CODEFLASH   0x00000000 :
	.text :  AT(0)
	{
		__text = . ;
		*(.reset.text)
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

	.pe2_rom :
	{
	  *(.non_use_pe_rom)
	} > INTROMPE2

	.tdata ALIGN (4) :
	{
		PROVIDE (__ep = .);
		*(.tbyte)
		*(.tcommon_byte)
		*(.tdata)
		*(.tbss)
		*(.tcommon)
	} > INTRAM
	
	.comment 0 : { *(.comment) }	
	.debug			0 : { *(.debug) }
	.line			0 : { *(.line) }
	.debug_srcinfo	0 : { *(.debug_srcinfo) }
	.debug_sfnames	0 : { *(.debug_sfnames) }
}
/* provide a pointer for the stack */
 
