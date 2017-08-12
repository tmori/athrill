#include "cui/tcp/cui_ops_tcp.h"
#include "tcp/tcp_server.h"

static TcpServerConfigType tcp_server_config;
static TcpServerType tcp_server;
static TcpConnectionType tcp_connection;

static FileOpType cui_fileop_tcp;

void cui_ops_tcp_init(void)
{
	Std_ReturnType err;

	err = tcp_server_create(&tcp_server_config, &tcp_server);
	if (err != STD_E_OK) {
		printf("ERROR:internal error: tcp_server_create()\n");
		exit(1);
	}

	err = tcp_server_poll(&tcp_server, &tcp_connection);
	if (err != STD_E_OK) {
		printf("ERROR:internal error: tcp_server_poll()\n");
		exit(1);
	}
	cui_fileop_tcp.write_fd = tcp_connection.client_socket;
	cui_fileop_tcp.read_fd = tcp_connection.client_socket;
	cui_fileop_register(&cui_fileop_tcp);
	return;
}

void cui_close(void)
{
	Std_ReturnType err;
	tcp_connection_close(&tcp_connection);

	err = tcp_server_poll(&tcp_server, &tcp_connection);
	if (err != STD_E_OK) {
		printf("ERROR:internal error: tcp_server_poll()\n");
		exit(1);
	}
	cui_fileop_tcp.write_fd = tcp_connection.client_socket;
	cui_fileop_tcp.read_fd = tcp_connection.client_socket;
	return;
}
