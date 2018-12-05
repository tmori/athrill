#include "athrill_device.h"
#include "mpu_ops.h"
#define ATHRILL_SYSCALL_DEVICE
#include "athrill_syscall.h"
#include <stdio.h>
#include <sys/fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

struct athrill_syscall_functable {
    void (*func) (AthrillSyscallArgType *arg);
};

static void athrill_syscall_none(AthrillSyscallArgType *arg);
static void athrill_syscall_socket(AthrillSyscallArgType *arg);
static void athrill_syscall_sense(AthrillSyscallArgType *arg);
static void athrill_syscall_connect(AthrillSyscallArgType *arg);
static void athrill_syscall_send(AthrillSyscallArgType *arg);
static void athrill_syscall_rev(AthrillSyscallArgType *arg);
static void athrill_syscall_shutdown(AthrillSyscallArgType *arg);
static void athrill_syscall_system(AthrillSyscallArgType *arg);

static struct athrill_syscall_functable syscall_table[SYS_API_ID_NUM] = {
    { athrill_syscall_none },
    { athrill_syscall_socket },
    { athrill_syscall_sense },
    { athrill_syscall_connect },
    { athrill_syscall_send },
    { athrill_syscall_rev },
    { athrill_syscall_shutdown },
    { athrill_syscall_system },
};

void athrill_syscall_device(uint32 addr)
{
    Std_ReturnType err;
    AthrillSyscallArgType *argp;

    err = mpu_get_pointer(0U, addr, (uint8 **)&argp);
    if (err != 0) {
        return;
    }
    if (argp->api_id >= SYS_API_ID_NUM) {
        return;
    }
    syscall_table[argp->api_id].func(argp);
    return;
}

static void athrill_syscall_none(AthrillSyscallArgType *arg)
{
    //nothing to do
    return;
}
static void athrill_syscall_socket(AthrillSyscallArgType *arg)
{
    int sockfd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (sockfd < 0) {
        return;
    }
    arg->ret_value = sockfd;
    return;
}

static void athrill_syscall_sense(AthrillSyscallArgType *arg)
{
    fd_set fds;
    struct timeval tv;
    int retval;
    int val;
    socklen_t len = sizeof(val);

    FD_ZERO(&fds);
    FD_SET(arg->body.api_sense.sockfd, &fds);

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    switch (arg->body.api_sense.dir) {
    case ATHRILL_SYSCALL_SENSE_DIR_READ:
        retval = select(arg->body.api_sense.sockfd + 1, &fds, NULL, NULL, &tv);
        break;
    case ATHRILL_SYSCALL_SENSE_DIR_WRITE:
        retval = select(arg->body.api_sense.sockfd + 1, NULL, &fds, NULL, &tv);
        break;
    case ATHRILL_SYSCALL_SENSE_DIR_EXCEPT:
        retval = select(arg->body.api_sense.sockfd + 1, NULL, NULL, &fds, &tv);
        break;
    default:
        return;;
    }
    if (retval < 0) {
        arg->ret_value = -errno;
    }
    else if (retval == 0) {
        arg->ret_value = -EAGAIN;
    }
    else {
        retval = getsockopt(arg->body.api_sense.sockfd, SOL_SOCKET, SO_ERROR, &val, &len);
        if (retval < 0) {
            arg->ret_value = -errno;
        }
        else {
            arg->ret_value = -val;
        }
    }

    return;
}

static void athrill_syscall_connect(AthrillSyscallArgType *arg)
{
    Std_ReturnType err;
    struct sockaddr_in client_addr;
    struct sys_sockaddr_in *sockaddrp;

    err = mpu_get_pointer(0U, arg->body.api_connect.sockaddr, (uint8 **)&sockaddrp);
    if (err != 0) {
        return;
    }
    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family = PF_INET;
    client_addr.sin_addr.s_addr = htonl(sockaddrp->sin_addr);
    client_addr.sin_port = htons(sockaddrp->sin_port);

    int ret = connect(arg->body.api_connect.sockfd, (struct sockaddr *)&client_addr, sizeof(client_addr));
    if (ret < 0) {
        arg->ret_value = -errno;
    }
    else {
        arg->ret_value = SYS_API_ERR_OK;
    }

    return;
}

static void athrill_syscall_send(AthrillSyscallArgType *arg)
{
    //TODO
    return;
}

static void athrill_syscall_rev(AthrillSyscallArgType *arg)
{
    //TODO
    return;
}

static void athrill_syscall_shutdown(AthrillSyscallArgType *arg)
{
    //TODO
    return;
}

static void athrill_syscall_system(AthrillSyscallArgType *arg)
{
	char cmd[256];
   	snprintf(cmd, sizeof(cmd), "athrill_extfunc.sh %u", arg->body.api_system.id);
   	if (system(cmd) < 0) {
   		printf("can not execute athrill_extfunc.sh\n");
        return;
   	}
    arg->ret_value = SYS_API_ERR_OK;
    return;
}

