/*
 *	V850E2/PX4-3507リンカスクリプト
 */

MEMORY
{
	INTROM(raxi) : org = 0x00000000, len = 512K
	INTRAM(wax)  : org = 0xFEDF6000, len = 40k
};

PROVIDE(_software_init_hook = 0);
PROVIDE(_software_term_hook = 0);
PROVIDE(_hardware_init_hook = 0);
PROVIDE(_hardware_term_hook = 0);
PROVIDE(_bsssecinib_table = 0);
PROVIDE(_tnum_bsssec = 0);
PROVIDE(_datasecinib_table = 0);
PROVIDE(_tnum_datasec = 0);

PROVIDE(_StartupHook = 0);
PROVIDE(_ShutdownHook = 0);
PROVIDE(_PreTaskHook = 0);
PROVIDE(_PostTaskHook = 0);
PROVIDE(_ErrorHook = 0);
PROVIDE(_ProtectionHook = 0);

OUTPUT_FORMAT("elf32-v850-rh850", "elf32-v850-rh850",
	      "elf32-v850-rh850")
OUTPUT_ARCH(v850-rh850)
SECTIONS
{
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
	.rodata : { *(.rodata) } > INTROM

	_textend = .;

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
	
	.sdata ALIGN (4) :
	{
		PROVIDE (__gp = . + 0x8000);
		*(.sdata)
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
	
	.comment 0 : { *(.comment) }	
	.debug			0 : { *(.debug) }
	.line			0 : { *(.line) }
	.debug_srcinfo	0 : { *(.debug_srcinfo) }
	.debug_sfnames	0 : { *(.debug_sfnames) }
	.debug_aranges	0 : { *(.debug_aranges) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
}
