#ifndef _CUI_OPS_H_
#define _CUI_OPS_H_

#include "std_types.h"

typedef struct {
	int  (*cui_getline) (char *line, int size);
	void (*cui_write) (char *line, int size);
	void (*cui_close) (void);
} FileOpType;
extern int	cui_fileop_register(FileOpType *fileop);

extern int  cui_getline(char *line, int size);
extern void cui_write(char *line, int size);
extern void cui_close(void);

#define CUI_BUFP_LEN		(1024U)
typedef struct {
	uint32 write_len;
	char p[CUI_BUFP_LEN];
} CuiPrintBufferType;

extern CuiPrintBufferType CuiPrintBuffer;
#define CPU_PRINT_BUF()		((CuiPrintBuffer.p))
#define CPU_PRINT_BUF_LEN()	(CUI_BUFP_LEN)

#define CUI_PRINTF(arg)	\
do { \
	CuiPrintBuffer.write_len = snprintf	arg;	\
	cui_write(CPU_PRINT_BUF(), CuiPrintBuffer.write_len);	\
} while (0)
#endif /* _CUI_OPS_H_ */
