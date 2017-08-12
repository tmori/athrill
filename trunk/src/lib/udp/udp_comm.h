#ifndef _UDP_SERVER_H_
#define _UDP_SERVER_H_

#include "winsock_wrapper/winsock_wrapper.h"
#include "udp_common.h"

typedef struct {
	SOCKET 			srv_sock;
	UdpBufferType	read_data;

	uint32			client_port;
	SOCKET 			clt_sock;
	UdpBufferType	write_data;
} UdpCommType;

extern Std_ReturnType udp_comm_create(const UdpCommConfigType *config, UdpCommType *comm);
extern Std_ReturnType udp_comm_read(UdpCommType *comm);
extern Std_ReturnType udp_comm_write(UdpCommType *comm);
extern void udp_server_delete(UdpCommType *comm);

#endif /* _UDP_SERVER_H_ */
