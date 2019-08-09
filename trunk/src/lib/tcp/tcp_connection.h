#ifndef _TCP_CONNECTION_H_
#define _TCP_CONNECTION_H_

#include "tcp_socket.h"

typedef struct {
	TcpSocketType	socket;
	bool			connected;
} TcpConnectionType;

extern Std_ReturnType tcp_connection_send(TcpConnectionType *connection, const char *data, uint32 size, uint32 *res);
extern Std_ReturnType tcp_connection_receive(TcpConnectionType *connection, char *data, uint32 size, uint32 *res);
extern void tcp_connection_close(TcpConnectionType *connection);

#endif /* _TCP_CONNECTION_H_ */
