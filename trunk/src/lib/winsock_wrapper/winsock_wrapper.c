#include "winsock_wrapper.h"

int winsock_init(void)
{
	WSADATA wsaData;
	int err;

	err = WSAStartup(MAKEWORD(2,0), &wsaData);
	if (err < 0) {
		return -1;
	}

	return 0;
}

void winsock_fini(void)
{
	WSACleanup();
}
