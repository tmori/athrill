#ifndef _ATHRILL_POSIX_SOCKET_H_
#define _ATHRILL_POSIX_SOCKET_H_

#include "athrill_syscall.h"

extern sys_int32 athrill_posix_socket(sys_int32 domain, sys_int32 type, sys_int32 protocol);   
extern sys_int32 athrill_posix_connect(sys_int32 sockfd, const struct sys_sockaddr_in *addr, sys_uint32 addrlen);

#define ATHRILL_POSIX_MSG_DONTWAIT 0
extern sys_int32 athrill_posix_send(sys_int32 sockfd, const sys_addr buf, sys_uint32 len, sys_int32 flags);
extern sys_int32 athrill_posix_recv(sys_int32 sockfd, sys_addr buf, sys_uint32 len, sys_int32 flags);

#define ATHRILL_POSIX_SHUT_RDWR 0
extern  sys_int32 athrill_posix_shutdown(sys_int32 sockfd, sys_int32 how);

#endif /* _ATHRILL_POSIX_SOCKET_H_ */