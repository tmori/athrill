#include "tcp/tcp_server.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

Std_ReturnType tcp_server_create(const TcpServerConfigType *config, TcpServerType *server)
{
	int ret;
	struct sockaddr_in addr;

    ret = tcp_socket_open(&server->socket);
    if (ret != STD_E_OK) {
    	return ret;
    }

    memset(&addr, 0, sizeof(struct sockaddr_in));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(config->server_port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server->config.server_port = config->server_port;

    ret = bind(server->socket.fd, (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
    if (ret < 0) {
		printf("%s %s() %u errno=%d\n", __FILE__, __FUNCTION__, __LINE__,  errno);
		return STD_E_NOENT;
    }
    ret = listen(server->socket.fd, 10);
    if (ret < 0) {
		printf("%s %s() %u errno=%d\n", __FILE__, __FUNCTION__, __LINE__,  errno);
		return STD_E_NOENT;
    }

	return STD_E_OK;
}

Std_ReturnType tcp_server_accept(const TcpServerType *server, TcpConnectionType *connection)
{
	struct sockaddr_in addr;
    socklen_t len = sizeof(struct sockaddr_in);

	//printf("tcp_server_accept: fd=%d\n", server->socket.fd);
    connection->socket.fd = accept(server->socket.fd, (struct sockaddr *)&addr, &len);
    if (connection->socket.fd < 0) {
		printf("%s %s() %u ret=%d\n", __FILE__, __FUNCTION__, __LINE__,  errno);
		return STD_E_NOENT;
    }
    connection->connected = TRUE;
	return STD_E_OK;
}

void tcp_server_close(TcpServerType *server)
{
	if (server != NULL) {
		tcp_socket_close(&server->socket);
	}
	return;
}
