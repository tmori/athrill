#ifndef _OPTION_H_
#define _OPTION_H_

#include "std_types.h"
#include "file.h"

typedef struct {
	bool	is_interaction;
	uint64	timeout;
	bool	is_binary_data;
	bool	is_remote;

	char	*fifocfgpath;
	char	buffer_fifopath[4096];

	char	*load_filepath;
	FileType load_file;

	char	*devcfgpath;
	char	buffer_devcfgpath[4096];
} CmdOptionType;

extern CmdOptionType *parse_args(int argc, const char* argv[]);


#endif /* _OPTION_H_ */
