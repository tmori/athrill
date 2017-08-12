#include "cui/cui_ops.h"

static FileOpType *cui_fileop;
CuiPrintBufferType CuiPrintBuffer;

int	cui_fileop_register(FileOpType *fileop)
{
	cui_fileop = fileop;
	return 0;
}

int  cui_getline(char *line, int size)
{
	return cui_fileop->cui_getline(line, size);
}
void cui_write(char *line, int size)
{
	return cui_fileop->cui_write(line, size);
}

void cui_close(void)
{
	return cui_fileop->cui_close();
}
