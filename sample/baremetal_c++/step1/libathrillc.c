#include <string.h>
#include <stdlib.h>
#include "athrill_syscall.h"

void *__dso_handle=0;

void *malloc(size_t size)
{
	return (void*)athrill_posix_malloc((sys_uint32)size);
}
void free(void *addr)
{
	athrill_posix_free((sys_addr)addr);
}

void *calloc(size_t nmemb, size_t size)
{
	return (void*)athrill_posix_calloc((sys_uint32)nmemb, (sys_uint32)size);
}

void *realloc(void *ptr, size_t size)
{
	return (void*)athrill_posix_realloc((sys_addr)ptr, (sys_uint32)size);
}

void *_malloc_r(struct _reent *reent, unsigned int size)
{
	return malloc(size);
}
void *_calloc_r(struct _reent *reent, size_t nmemb, size_t size)
{
	return calloc(nmemb, size);
}
void *_realloc_r(struct _reent *reent, void *ptr, size_t size)
{
	return realloc(ptr, size);
}

void _free_r(struct _reent *reent, void *addr)
{
	free(addr);
}
int raise(int sig)
{
	return 0;
}

void _fflush_r(void)
{
}

void __sinit(void)
{
}

void _cleanup_r(void){}
void __register_exitproc(void){}
