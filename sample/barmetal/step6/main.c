#include "test_serial.h"
#include "test_reg.h"
#include "section.h"
#include "interrupt.h"
#include <string.h>
#include "timer.h"
#include "athrill_syscall.h"

unsigned char stack_data[STACK_SIZE] __attribute__ ((section(".bss_noclr")));

unsigned char mmap_data[1024] __attribute__ ((section(".mmap_section")));

sys_addr athrill_device_func_call __attribute__ ((section(".athrill_device_section")));


int main(void)
{
	timer_init();

	printf("Hello World!!\n");

	sys_int32 sockfd;
	sys_int32 err;
	struct sys_sockaddr_in sockaddr;
	
	sockfd = athrill_posix_socket(ATHRILL_SYSCALL_SOCKET_DOMAIN_AF_INET, ATHRILL_SYSCALL_SOCKET_TYPE_STREAM, ATHRILL_SYSCALL_SOCKET_PROTOCOL_ZERO);
	if (sockfd < 0) {
		printf("socket error\n");
	}
	else {
		test_print_line("OK:socket=", sockfd);
	}

	sockaddr.sin_family = ATHRILL_SYSCALL_SOCKADDR_FAMILIY_PF_INET;
	sockaddr.sin_addr = ATHRILL_SYSCALL_IPADDR(127, 0, 0, 1);
	sockaddr.sin_port = 50005;

	err = athrill_posix_connect(sockfd, &sockaddr, sizeof(sockaddr));
	if (err < 0) {
		test_print_line("connect error=", -err);
	}
	else {
		printf("OK:connect\n");
	}
	do {
		err = athrill_posix_sense(sockfd, SYS_API_ID_CONNECT);
		if (err < 0) {
			test_print_line("sense error=", -err);
		}
		else {
			printf("OK:sense\n");
		}
	} while (err != 0);

	err = athrill_posix_send(sockfd, (sys_addr)"test data sended", sizeof("test data sended"), ATHRILL_POSIX_MSG_DONTWAIT);
	if (err < 0) {
		test_print_line("send error=", -err);
	}
	else {
		printf("OK:send\n");
	}



	while (1) {
		;
	}
}

void bss_clear(void)
{
	unsigned char *p = &_bss_start;
	unsigned char *e = &_bss_end;
	for (;p < e; p++) {
		*p = 0;
	}
	return;
}

void data_init(void)
{
	unsigned char *p_rom = &_idata_start;
	unsigned char *e_rom = &_idata_end;
	unsigned char *p_ram = &_data_start;

	for (;p_rom < e_rom; p_ram++, p_rom++) {
		*p_ram = *p_rom;
	}
}

