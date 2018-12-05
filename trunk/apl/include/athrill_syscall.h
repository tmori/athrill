#ifndef _ATHRILL_SYSCALL_H_
#define _ATHRILL_SYSCALL_H_

typedef unsigned int sys_uint32;
typedef unsigned short sys_uint16;
typedef unsigned char sys_uint8;
typedef signed int sys_int32;
typedef signed short sys_int16;
typedef signed char sys_int8;
typedef unsigned int sys_addr;

struct sys_sockaddr_in {
    sys_uint8  sin_family;
    sys_uint16 sin_port;
    sys_uint32 sin_addr;
    sys_int8   sin_zero[8];
};
struct api_arg_socket {
    sys_int32 domain;
    sys_int32 type;
    sys_int32 protocol;
};
struct api_arg_connect {
    sys_int32 sockfd;
    sys_addr sockaddr;
    sys_uint32 addrlen;
};
struct api_arg_send {
    sys_int32 sockfd;
    sys_addr buf;
    sys_uint32 len;
    sys_int32 flags;
};
struct api_arg_recv {
    sys_int32 sockfd;
    sys_addr buf;
    sys_uint32 len;
    sys_int32 flags;
};
struct api_arg_shutdown {
    sys_int32 sockfd;
    sys_int32 how;
};
struct api_arg_system {
    sys_uint32 id;
};

typedef enum {
    SYS_API_ID_NONE = 0,
    SYS_API_ID_SOCKET,
    SYS_API_ID_CONNECT,
    SYS_API_ID_SEND,
    SYS_API_ID_RECV,
    SYS_API_ID_SHUTDOWN,
    SYS_API_ID_SYSTEM,
    SYS_API_ID_NUM,
} AthrillSyscallApiIdType;

#define SYS_API_ERR_OK      0
#define SYS_API_ERR_PERM    -1
#define SYS_API_ERR_NOENT   -2
#define SYS_API_ERR_AGAIN   -11
#define SYS_API_ERR_NOMEM   -12
#define SYS_API_ERR_ACCESS  -13
#define SYS_API_ERR_FAULT   -14
#define SYS_API_ERR_EXSIT   -17
#define SYS_API_ERR_INVAL   -22
#define SYS_API_ERR_BADFD   -77

typedef struct {
    sys_uint32 api_id;
    sys_int32 ret_value;
    union {
        struct api_arg_socket api_socket;
        struct api_arg_connect api_connect;
        struct api_arg_send api_send;
        struct api_arg_recv api_recv;
        struct api_arg_shutdown api_shutdown;
        struct api_arg_system api_system;
    } body;
} AthrillSyscallArgType;

#ifndef ATHRILL_SYSCALL_DEVICE
extern sys_addr athrill_device_func_call __attribute__ ((section(".athrill_device_section")));

#define ATHRILL_SYSCALL(api_argp)   \
do {    \
    athrill_device_func_call = (sys_addr)(api_argp);    \
} while (0) \

static inline sys_int32 athrill_posix_socket(sys_int32 domain, sys_int32 type, sys_int32 protocol)
{
    AthrillSyscallArgType args;

    args.api_id = SYS_API_ID_SOCKET;
    args.ret_value = SYS_API_ERR_INVAL;
    args.body.api_socket.domain = domain;
    args.body.api_socket.type = type;
    args.body.api_socket.protocol = protocol;

    ATHRILL_SYSCALL(&args);
    
    return args.ret_value;
}
static inline sys_int32 athrill_posix_connect(sys_int32 sockfd, const struct sys_sockaddr_in *addr, sys_uint32 addrlen)
{
    AthrillSyscallArgType args;

    args.api_id = SYS_API_ID_CONNECT;
    args.ret_value = SYS_API_ERR_INVAL;
    args.body.api_connect.sockfd = sockfd;
    args.body.api_connect.sockaddr = (sys_addr)addr;
    args.body.api_connect.addrlen = addrlen;

    ATHRILL_SYSCALL(&args);

    return args.ret_value;
}

#define ATHRILL_POSIX_MSG_DONTWAIT 0
static inline sys_uint32 athrill_posix_send(sys_int32 sockfd, const sys_addr buf, sys_uint32 len, sys_int32 flags)
{
    AthrillSyscallArgType args;

    args.api_id = SYS_API_ID_SEND;
    args.ret_value = SYS_API_ERR_INVAL;
    args.body.api_send.sockfd = sockfd;
    args.body.api_send.buf = buf;
    args.body.api_send.len = len;
    args.body.api_send.flags = flags;

    ATHRILL_SYSCALL(&args);

    return args.ret_value;
}
static inline sys_uint32 athrill_posix_recv(sys_int32 sockfd, sys_addr buf, sys_uint32 len, sys_int32 flags)
{
    AthrillSyscallArgType args;

    args.api_id = SYS_API_ID_RECV;
    args.ret_value = SYS_API_ERR_INVAL;
    args.body.api_recv.sockfd = sockfd;
    args.body.api_recv.buf = buf;
    args.body.api_recv.len = len;
    args.body.api_recv.flags = flags;

    ATHRILL_SYSCALL(&args);

    return args.ret_value;
}

#define ATHRILL_POSIX_SHUT_RDWR 0
static inline sys_int32 athrill_posix_shutdown(sys_int32 sockfd, sys_int32 how)
{
    AthrillSyscallArgType args;

    args.api_id = SYS_API_ID_SHUTDOWN;
    args.ret_value = SYS_API_ERR_INVAL;
    args.body.api_shutdown.sockfd = sockfd;
    args.body.api_shutdown.how = how;

    ATHRILL_SYSCALL(&args);

    return args.ret_value;
}

static inline sys_int32 athrill_posix_system(sys_uint32 id)
{
    AthrillSyscallArgType args;
    args.api_id = SYS_API_ID_SYSTEM;
    args.ret_value = SYS_API_ERR_INVAL;

    ATHRILL_SYSCALL(&args);

    return args.ret_value;
}

#endif /* ATHRILL_SYSCALL_DEVICE */

#endif /* _ATHRILL_SYSCALL_H_ */