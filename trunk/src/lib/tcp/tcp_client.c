#include "tcp/tcp_client.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

Std_ReturnType tcp_client_create(const TcpClientConfigType *config, TcpClientType *client)
{
	int ret;

    client->connection.connected = FALSE;
    ret = tcp_socket_open(&client->connection.socket);
    if (ret != STD_E_OK) {
    	return ret;
    }
    client->config.server_port = config->server_port;
    client->config.ipaddr = config->ipaddr;
	return STD_E_OK;
}

Std_ReturnType tcp_client_connect(TcpClientType *client)
{
	uint32 ipaddr;
	int ret;
	struct sockaddr_in addr;

    ret = tcp_inet_get_ipaddr(client->config.ipaddr, &ipaddr);
    if (ret != STD_E_OK) {
    	return ret;
    }

    memset(&addr, 0, sizeof(struct sockaddr_in));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(client->config.server_port);
    addr.sin_addr.s_addr = htonl(ipaddr);

    ret = connect(client->connection.socket.fd, (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
    if (ret < 0) {
		printf("%s %s() %u errno=%d\n", __FILE__, __FUNCTION__, __LINE__,  errno);
		return STD_E_NOENT;
    }
    client->connection.connected = TRUE;
    return STD_E_OK;
}

void tcp_client_close(TcpClientType *client)
{
	if (client != NULL) {
		tcp_socket_close(&client->connection.socket);
	}
	return;
}
