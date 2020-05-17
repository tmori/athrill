#ifndef _TCP_CLIENT_H_
#define _TCP_CLIENT_H_

#include "tcp_connection.h"

typedef struct {
	const char* ipaddr;
	uint16	server_port;
} TcpClientConfigType;

typedef struct {
	TcpClientConfigType		config;
	TcpConnectionType		connection;
} TcpClientType;


extern Std_ReturnType tcp_client_create(const TcpClientConfigType *config, TcpClientType *client);
extern Std_ReturnType tcp_client_connect(TcpClientType *client);
extern void tcp_client_close(TcpClientType *server);

#endif /* _TCP_CLIENT_H_ */
