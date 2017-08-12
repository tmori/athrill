#ifndef _TCP_SERVER_H_
#define _TCP_SERVER_H_

#include "std_types.h"
#include "std_errno.h"

typedef struct {
	uint16	server_port;
} TcpServerConfigType;

typedef struct {
	int server_socket;
} TcpServerType;

typedef struct {
	int client_socket;
} TcpConnectionType;

extern Std_ReturnType tcp_server_create(const TcpServerConfigType *config, TcpServerType *server);
extern Std_ReturnType tcp_server_poll(const TcpServerType *server, TcpConnectionType *connection);
extern void tcp_connection_close(TcpServerType *server);

#endif /* _TCP_SERVER_H_ */
