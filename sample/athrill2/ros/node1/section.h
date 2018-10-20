#ifndef _SECTION_H_
#define _SECTION_H_

/*
 * RAM BSS
 */
extern unsigned char _bss_start;
extern unsigned char _bss_end;

/*
 * RAM IDATA
 */
extern unsigned char _data_start;

/*
 * ROM IDATA
 */
extern unsigned char _idata_start;
extern unsigned char _idata_end;

extern void bss_clear(void);
extern void data_init(void);

#endif /* _SECTION_H_ */