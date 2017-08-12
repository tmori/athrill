#ifndef _FILE_H_
#define _FILE_H_

#include "std_types.h"
#include "token.h"

extern bool file_exist(const char *path);

#define MAX_FILE_SIZE	(1024 * 1024 * 5)
typedef struct {
	TokenStringType filepath;
	void			*fp;
	uint32			size;
	uint8			buffer[MAX_FILE_SIZE];
} FileType;
extern bool file_load(FileType *file);

extern uint32 file_get_parent_folder_pathlen(const char *filepath);

extern bool file_ropen(FileType *file);
extern bool file_wopen(FileType *file);
extern bool file_ropen_filepath(const char *dir, const char *filename, FileType *file);
extern uint32 file_getline(FileType *file, char *line, int size);
extern uint32 file_readline(FileType *file, char *line, int size, int lineno);

extern void file_putline(FileType *file, char *line, int size);

extern void file_close(FileType *file);

extern bool file_printline(const char *dir, const char *filename, FileType *file, uint32 start, uint32 end);

#endif /* _FILE_H_ */
