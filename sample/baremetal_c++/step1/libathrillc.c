#include <string.h>

void *__dso_handle=0;

void *malloc(unsigned int size)
{
	return NULL;
}
void free(void *addr)
{
	return;
}

void *calloc(size_t nmemb, size_t size)
{
	return NULL;
}

void *realloc(void *ptr, size_t size)
{
	return NULL;
}

void *_malloc_r(void *reent, unsigned int size)
{
	return NULL;
}
void *_calloc_r(void *reent, size_t nmemb, size_t size)
{
	return NULL;
}
void *_realloc_r(void *reent, void *ptr, size_t size)
{
	return NULL;
}

void _free_r(void *reent, void *addr)
{
	return;
}
int raise(int sig)
{
	return;
}
