#include "tcp_connection.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <stdio.h>

Std_ReturnType tcp_connection_send(TcpConnectionType *connection, const char *data, uint32 size, uint32 *res)
{
	ssize_t snd_size;

	snd_size = send(connection->socket.fd, data, size, 0);
	if (snd_size < 0) {
		printf("ERROR: tcp_connection_send() errno=%d\n", errno);
		connection->connected = FALSE;
		return STD_E_NOENT;
	}
	else if (snd_size == 0) {
		connection->connected = FALSE;
		return STD_E_NOENT;
	}
	*res = snd_size;
	return STD_E_OK;
}

Std_ReturnType tcp_connection_receive(TcpConnectionType *connection, char *data, uint32 size, uint32 *res)
{
	ssize_t rcv_size;
	rcv_size = recv(connection->socket.fd, data, size, 0);
	if (rcv_size < 0) {
		printf("ERROR: tcp_connection_receive() errno=%d\n", errno);
		connection->connected = FALSE;
		return STD_E_NOENT;
	}
	else if (rcv_size == 0) {
		connection->connected = FALSE;
		return STD_E_NOENT;
	}
	*res = rcv_size;
#if 0
	{
		uint32 i;
		for (i = 0; i < rcv_size; i++) {
			printf("buf[%u]=%c\n", i, data[i]);
		}
	}
#endif
	return STD_E_OK;
}

void tcp_connection_close(TcpConnectionType *connection)
{
	if (connection != NULL) {
		tcp_socket_close(&connection->socket);
	}
	return;
}
