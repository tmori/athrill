/*
 *  @(#) $Id: shelf.ld 1824 2010-07-02 06:50:03Z mit-kimai $
 */

OUTPUT_FORMAT("elf32-sh","elf32-sh","elf32-sh")
OUTPUT_ARCH(sh)
STARTUP(start.o)

MEMORY
{
    rom : ORIGIN = 0, LENGTH = 768K
    ram : ORIGIN = 0xfff80000, LENGTH = 32K
}

PROVIDE(_hardware_init_hook = 0);
PROVIDE(_software_init_hook = 0);
PROVIDE(_software_term_hook = 0);
PROVIDE(__kernel_bsssecinib_table = 0);
PROVIDE(__kernel_tnum_bsssec = 0);
PROVIDE(__kernel_datasecinib_table = 0);
PROVIDE(__kernel_tnum_datasec = 0);
SECTIONS
{
    .text :	
    {
        __text = . ;
		*(.vector)
        *(*.text*)
	
    } > rom
	
    .vector_entry :
	{
		__vector_entry_start = .;
		*(.vector_entry)

	} > rom
	__vector_entry_end = .;
    _etext = .	;
    PROVIDE (etext = .)	;
    
    __rodata_start = .;
    .rodata : {
        *(*.rodata*)
    } > rom
    . = ALIGN(4);
    __rodata_end = .;
    
    __idata_start = . ;
    /*.data   :  AT(__idata_start) */
    /*.data   :  AT(ADDR(.rodata)+SIZEOF(.rodata)) */
    .data   :
    {
        __data_start = . ;
        KEEP(*(*.data*))
    } > ram
    AT > rom
    __idata_end = __idata_start + SIZEOF(.data);
    _edata  =  . ;
    PROVIDE (edata = .);
    
    . = ALIGN(4);
    .bss       :
    {
        __bss_start = .	;		

      *(*.bss*)
      *(COMMON)
    } > ram
    __bss_end = ADDR(.bss)+SIZEOF(.bss);		
    _end = .		;
    PROVIDE (end = .)	;
    /*
     * .comment 0 : { *(.comment) }	
     * .debug          0 : { *(.debug) }
     * .line           0 : { *(.line) }
     * .debug_srcinfo  0 : { *(.debug_srcinfo) }
     * .debug_sfnames  0 : { *(.debug_sfnames) }
     * .debug_aranges  0 : { *(.debug_aranges) }
     * .debug_pubnames 0 : { *(.debug_pubnames) }
     */
}	
