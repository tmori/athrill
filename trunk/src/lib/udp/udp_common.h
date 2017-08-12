#ifndef _UDP_COMMON_H_
#define _UDP_COMMON_H_

#include "std_types.h"
#include "std_errno.h"

typedef struct {
	uint16	server_port;
	uint16	client_port;
	bool	is_wait;
} UdpCommConfigType;

#define UDP_BUFFER_LEN	4096
typedef struct {
	int len;
	char buffer[UDP_BUFFER_LEN];
} UdpBufferType;

#endif /* _UDP_COMMON_H_ */
