/*
 *	RH850/F1L-2Mリンカスクリプト
 */

MEMORY
{
	INTROM(raxi) : org = 0x00000000, len = 4M
	INTROMPE2    : org = 0x00800000, len = 2M
	INTRAM(wax)  : org = 0xfef00000, len = 128k
	LRAM_PE1(wax) : org = 0xFEBD0000, len = 192k
	LRAM_PE2(wax) : org = 0xFE9D0000, len = 192k
}

SECTIONS
{
    provide(hardware_init_hook = 0);
    provide(software_init_hook = 0);
    provide(software_term_hook = 0);

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

	. = ALIGN(4);
	 __bss_start = .;
	.sbss                  :
	{
		. = ALIGN(4);
		*(.sbss)
	} > INTRAM

	.bss	   :
	{
	  . = ALIGN(4);
	  *(.bss)
	  *(COMMON)
	} > INTRAM

	. = ALIGN(4);
	__bss_end = .;

	.pe2_rom :
	{
	  *(.pe2_reset)
	  *(.pe2_code)
	} > INTROMPE2

	.pe1_lram :
	{
	  __pe1_lram_start = .;
	  *(.pe1_lram)
	  __pe1_lram_end = .;
	} > LRAM_PE1

	.pe2_lram :
	{
	  __pe2_lram_start = .;
	  *(.pe2_lram)
	  __pe2_lram_end = .;
	} > LRAM_PE2

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
 