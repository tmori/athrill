#ifndef _SOCKET_H_
#define _SOCKET_H_

#include <sys/types.h>

extern int athrill_posix_socket(int domain, int type, int protocol);   
extern int athrill_posix_bind(int sockfd, const struct sockaddr *addr, size_t addrlen);
extern int athrill_posix_connect(int sockfd, const struct sockaddr *addr, size_t addrlen);

#define ATHRILL_POSIX_MSG_DONTWAIT 0
extern ssize_t athrill_posix_send(int sockfd, const void *buf, size_t len, int flags);
extern ssize_t athrill_posix_recv(int sockfd, void *buf, size_t len, int flags);

#define ATHRILL_POSIX_SHUT_RDWR 0
extern  int athrill_posix_shutdown(int sockfd, int how);

#endif /* _SOCKET_H_ */