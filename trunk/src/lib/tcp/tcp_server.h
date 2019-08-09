#ifndef _TCP_SERVER_H_
#define _TCP_SERVER_H_

#include "tcp_connection.h"

typedef struct {
	uint16	server_port;
} TcpServerConfigType;

typedef struct {
	TcpServerConfigType		config;
	TcpSocketType			socket;
} TcpServerType;


extern Std_ReturnType tcp_server_create(const TcpServerConfigType *config, TcpServerType *server);
extern Std_ReturnType tcp_server_accept(const TcpServerType *server, TcpConnectionType *connection);
extern void tcp_server_close(TcpServerType *server);

#endif /* _TCP_SERVER_H_ */
