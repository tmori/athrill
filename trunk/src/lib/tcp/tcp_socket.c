#include "tcp_socket.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <errno.h>

Std_ReturnType tcp_socket_open(TcpSocketType *sock)
{
	sock->fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock->fd < 0) {
		printf("%s %s() %u errno=%d\n", __FILE__, __FUNCTION__, __LINE__,  errno);
		return STD_E_INVALID;
	}
	//printf("tcp_socket_open: fd=%d\n", sock->fd);
	return STD_E_OK;
}

void tcp_socket_close(TcpSocketType *socket)
{
	if (socket->fd >= 0) {
		close(socket->fd);
		socket->fd = -1;
	}
	return;
}
