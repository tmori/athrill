#include "tcp/tcp_server.h"
#include <stdio.h>
#include <winsock2.h>
#include <ws2tcpip.h>

static WSADATA wsaData;

Std_ReturnType tcp_server_create(const TcpServerConfigType *config, TcpServerType *server)
{
	return STD_E_OK;
}

Std_ReturnType tcp_server_poll(const TcpServerType *server, TcpConnectionType *connection)
{
	return STD_E_OK;
}

void tcp_connection_close(TcpServerType *server)
{
	return;
}
