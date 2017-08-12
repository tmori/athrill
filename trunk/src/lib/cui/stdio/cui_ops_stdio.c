#include "cui/stdio/cui_ops_stdio.h"
#include "cui/cui_ops.h"
#include <unistd.h>
#include <stdio.h>

typedef struct {
	int read_fd;
	int write_fd;
	FileOpType op;
} StdioFileOpType;

static void cui_close_stdio(void);
static int  cui_getline_stdio(char *line, int size);
static void cui_write_stdio(char *line, int size);

static StdioFileOpType cui_fileop_stdio = {
	.read_fd  = 0,
	.write_fd = 1,
	.op = {
			.cui_getline = cui_getline_stdio,
			.cui_write = cui_write_stdio,
			.cui_close = cui_close_stdio,
	},
};


void cui_ops_stdio_init(void)
{
	(void)cui_fileop_register(&cui_fileop_stdio.op);
	return;
}

static void cui_close_stdio(void)
{
	//nothing to do
	return;
}

static int  cui_getline_stdio(char *line, int size)
{
	int n = 0;
	char c;
	int rc;

	while (TRUE) {
		if (n >= size) {
			printf("ERROR:input is too long\n");
			return -1;
		}
		rc = read(cui_fileop_stdio.read_fd, &c, 1);
		if (rc <= 0) {
			return -1;
		}
		if (c < 0 || c == '\n') {
			break;
		}
		line[n] = c;
		n++;
	}
	return n;
}

static void cui_write_stdio(char *line, int size)
{
	//nothing to do
	return;
}
