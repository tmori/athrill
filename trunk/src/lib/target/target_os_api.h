#ifndef _TARGET_OS_API_H_
#define _TARGET_OS_API_H_

#ifdef OS_LINUX
//#define  __USE_GNU
/*
 * Sleep
 */
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/ioctl.h>

#define target_os_api_sleep(arg) usleep((arg)*1000)
#define winsock_init()
#define winsock_fini()
#ifdef OS_MAC
#define PRINT_FMT_UINT64    "%llu"
#define FMT_UINT64    "llu"
#define FMT_SINT64    "lld"
#define PRINT_FMT_SINT64    "%lld"
#ifndef SOCK_NONBLOCK
#define SOCK_NONBLOCK O_NONBLOCK
#endif
#else
#define PRINT_FMT_UINT64	"%Lu"
#define FMT_UINT64	"Lu"
#define FMT_SINT64	"Ld"
#define PRINT_FMT_SINT64	"%Ld"
#endif /* OS_MAC */
#define target_os_api_sockaddr_type	sockaddr_in
#define target_os_api_closesocket	close
#define target_os_api_ioctlsock		ioctl

#define target_os_sockaddr_sin_addr	s_addr

#define target_os_api_open_ctw(filepath, mode)	open((filepath), O_CREAT | O_TRUNC |O_WRONLY, (mode))
#define target_os_api_open_aw(filepath)	open((filepath), O_APPEND |O_WRONLY)
#define target_os_api_open_w(filepath)	open((filepath), O_WRONLY)
#define target_os_api_open_r(filepath)	open((filepath), O_RDONLY)

/*
 * Winsock
 */
#define TARGET_OS_SOCKET_TYPE	int
#else
#include <windows.h>
#include <winsock2.h>
/*
 * Winsock
 */
#include "winsock_wrapper/winsock_wrapper.h"
#define TARGET_OS_SOCKET_TYPE	SOCKET
#define target_os_api_sockaddr_type	sockaddr_in


/*
 * Sleep
 */
#define target_os_api_sleep(arg) Sleep(arg)
#define PRINT_FMT_UINT64	"%I64u"
#define FMT_UINT64	"I64u"
#define PRINT_FMT_SINT64	"%I64d"
#define FMT_SINT64	"I64d"
#define target_os_api_open_ctw(filepath, mode)	open((filepath), O_CREAT | O_TRUNC |O_WRONLY | O_BINARY, (mode))
#define target_os_api_open_aw(filepath)	open((filepath), O_APPEND |O_WRONLY | O_BINARY)
#define target_os_api_open_w(filepath)	open((filepath), O_WRONLY | O_BINARY)
#define target_os_api_open_r(filepath)	open((filepath), O_RDONLY | O_BINARY)
#define target_os_api_closesocket	closesocket
#define target_os_sockaddr_sin_addr	S_un.S_addr

//#define target_os_api_ioctlsock		ioctlsock
static inline void target_os_api_ioctlsock(SOCKET sock, int flag, unsigned long *valp) 
{
	ioctlsocket(sock, flag, valp);
}
#endif /* OS_LINUX */
/*
 * File I/O
 */




#endif /* _TARGET_OS_API_H_ */
