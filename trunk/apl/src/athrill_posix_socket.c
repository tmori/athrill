#include "athrill_posix_socket.h"

sys_int32 athrill_posix_socket(sys_int32 domain, sys_int32 type, sys_int32 protocol)
{
    AthrillSyscallArgType args;

    args.api_id = SYS_API_ID_RECV;
    args.ret_value = SYS_API_ERR_INVAL;
    args.body.api_socket.domain = domain;
    args.body.api_socket.type = type;
    args.body.api_socket.protocol = protocol;

    ATHRILL_SYSCALL(&args);
    
    return args.ret_value;
}
sys_int32 athrill_posix_connect(sys_int32 sockfd, const struct sys_sockaddr_in *addr, sys_uint32 addrlen)
{
    AthrillSyscallArgType args;

    args.api_id = SYS_API_ID_RECV;
    args.ret_value = SYS_API_ERR_INVAL;
    args.body.api_connect.sockfd = sockfd;
    args.body.api_connect.sockaddr = (sys_addr)addr;
    args.body.api_connect.addrlen = addrlen;

    ATHRILL_SYSCALL(&args);

    return args.ret_value;
}
sys_uint32 athrill_posix_send(sys_int32 sockfd, const sys_addr buf, sys_uint32 len, sys_int32 flags)
{
    AthrillSyscallArgType args;

    args.api_id = SYS_API_ID_RECV;
    args.ret_value = SYS_API_ERR_INVAL;
    args.body.api_send.sockfd = sockfd;
    args.body.api_send.buf = buf;
    args.body.api_send.len = len;
    args.body.api_send.flags = flags;

    ATHRILL_SYSCALL(&args);

    return args.ret_value;
}
sys_uint32 athrill_posix_recv(sys_int32 sockfd, sys_addr buf, sys_uint32 len, sys_int32 flags)
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

sys_int32 athrill_posix_shutdown(sys_int32 sockfd, sys_int32 how)
{
    AthrillSyscallArgType args;

    args.api_id = SYS_API_ID_SHUTDOWN;
    args.ret_value = SYS_API_ERR_INVAL;
    args.body.api_shutdown.sockfd = sockfd;
    args.body.api_shutdown.how = how;

    ATHRILL_SYSCALL(&args);

    return args.ret_value;
}
