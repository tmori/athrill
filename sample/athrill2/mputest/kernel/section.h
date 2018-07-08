#ifndef _SECTION_H_
#define _SECTION_H_

/*
 * RAM BSS
 */
extern unsigned char _bss_kernel_start;
extern unsigned char _bss_kernel_end;

/*
 * RAM IDATA
 */
extern unsigned char _data_kernel_start;

/*
 * ROM IDATA
 */
extern unsigned char _idata_kernel_start;
extern unsigned char _idata_kernel_end;

extern void bss_clear(void);
extern void data_init(void);

#endif /* _SECTION_H_ */