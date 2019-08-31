#include <lwip/sockets.h>
#include <unistd.h>
#include "kernel.h"
#include <pthread.h>

void lwip_init(void)
{
	return;
}
extern int get_tskid(void);

int lwip_accept(int s, struct sockaddr *addr, socklen_t *addrlen)
{
	int ret;
	ret = accept(s, (struct sockaddr *)addr, addrlen);
	return ret;
}

int lwip_bind(int s, const struct sockaddr *name, socklen_t namelen)
{
	return bind(s, (struct sockaddr *)name, namelen);
}
int lwip_shutdown(int s, int how)
{
	return shutdown(s, how);
}

int lwip_getpeername (int s, struct sockaddr *name, socklen_t *namelen)
{
	//TODO
	return 0;
}

int lwip_getsockname (int s, struct sockaddr *name, socklen_t *namelen)
{
	//TODO
	return 0;
}

int lwip_getsockopt (int s, int level, int optname, void *optval, socklen_t *optlen)
{
	//TODO
	return 0;
}

int lwip_setsockopt (int s, int level, int optname, const void *optval, socklen_t optlen)
{
	//TODO
	return 0;
}

int lwip_close(int s)
{
	return close(s);
}


int lwip_connect(int s, const struct sockaddr *name, socklen_t namelen)
{
	OsSaveLockType save;
	os_save_unlock(&save);
	int ret = connect(s, name, namelen);
	os_restore_lock(&save);
	return ret;
}


int lwip_listen(int s, int backlog)
{
	return listen(s, backlog);
}


int lwip_recv(int s, void *mem, size_t len, int flags)
{
	int ret;
	OsSaveLockType save;
	os_save_unlock(&save);
	ret = recv(s, mem, len, flags);
	os_restore_lock(&save);
	return ret;
}

int lwip_read(int s, void *mem, size_t len)
{
	return read(s, mem, len);
}

int lwip_recvfrom(int s, void *mem, size_t len, int flags,
      struct sockaddr *from, socklen_t *fromlen)
{
	return 0;
}

int lwip_send(int s, const void *dataptr, size_t size, int flags)
{
	return send(s, dataptr, size, flags);
}

int lwip_sendto(int s, const void *dataptr, size_t size, int flags,
    const struct sockaddr *to, socklen_t tolen)
{
	return 0;
}

int lwip_socket(int domain, int type, int protocol)
{
	return socket(domain, type, protocol);
}

int lwip_write(int s, const void *dataptr, size_t size)
{
	return write(s, dataptr, size);
}


int lwip_select(int maxfdp1, fd_set *readset, fd_set *writeset, fd_set *exceptset,
                struct timeval *timeout)
{
	int ret;
	OsSaveLockType save;
	os_save_unlock(&save);
	ret = select(maxfdp1, readset, writeset, exceptset, timeout);
	os_restore_lock(&save);
	return ret;
}

int lwip_ioctl(int s, long cmd, void *argp)
{
	//TODO
	return 0;
}

int lwip_fcntl(int s, int cmd, int val)
{
	//TODO
	return 0;
}

struct hostent *lwip_gethostbyname(const char *name)
{
	return gethostbyname(name);
}

char *ipaddr_ntoa_r(const ip_addr_t *addr, char *buf, int buflen)
{
	//TODO
	return 0;
}


