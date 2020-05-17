#include "tcp_socket.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
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

Std_ReturnType tcp_inet_get_ipaddr(const char *hostname, uint32 *ipaddr)
{
	sint32 result;
	uint8 addr_array[5];
    uint8 *paddr = addr_array;

    result = sscanf(hostname, "%hhu.%hhu.%hhu.%hhu",
    		(uint8*)&addr_array[0],
			(uint8*)&addr_array[1],
			(uint8*)&addr_array[2],
			(uint8*)&addr_array[3]);

    if (result != 4) {
    	 struct hostent *host_address = gethostbyname(hostname);
        if (host_address == NULL) {
    		printf("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, STD_E_INVALID);
        	return STD_E_INVALID;
        }
        paddr = (uint8*)host_address->h_addr_list[0];
    }
    memcpy((void*)ipaddr, (void*)paddr, 4U);
    return STD_E_OK;
}
