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
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "mpu_malloc.h"
#include "assert.h"
#include "target/target_os_api.h"

struct athrill_syscall_functable {
    void (*func) (AthrillSyscallArgType *arg);
};

static void athrill_syscall_none(AthrillSyscallArgType *arg);
static void athrill_syscall_socket(AthrillSyscallArgType *arg);
static void athrill_syscall_sense(AthrillSyscallArgType *arg);
static void athrill_syscall_bind(AthrillSyscallArgType *arg);
static void athrill_syscall_listen(AthrillSyscallArgType *arg);
static void athrill_syscall_accept(AthrillSyscallArgType *arg);
static void athrill_syscall_connect(AthrillSyscallArgType *arg);
static void athrill_syscall_select(AthrillSyscallArgType *arg);
static void athrill_syscall_send(AthrillSyscallArgType *arg);
static void athrill_syscall_recv(AthrillSyscallArgType *arg);
static void athrill_syscall_shutdown(AthrillSyscallArgType *arg);
static void athrill_syscall_system(AthrillSyscallArgType *arg);
static void athrill_syscall_malloc(AthrillSyscallArgType *arg);
static void athrill_syscall_calloc(AthrillSyscallArgType *arg);
static void athrill_syscall_realloc(AthrillSyscallArgType *arg);
static void athrill_syscall_free(AthrillSyscallArgType *arg);

static void athrill_syscall_open_r(AthrillSyscallArgType *arg);
static void athrill_syscall_read_r(AthrillSyscallArgType *arg);
static void athrill_syscall_write_r(AthrillSyscallArgType *arg);
static void athrill_syscall_close_r(AthrillSyscallArgType *arg);
static void athrill_syscall_lseek_r(AthrillSyscallArgType *arg);


static void athrill_syscall_fopen(AthrillSyscallArgType *arg);
static void athrill_syscall_fclose(AthrillSyscallArgType *arg);
static void athrill_syscall_fread(AthrillSyscallArgType *arg);
static void athrill_syscall_fwrite(AthrillSyscallArgType *arg);
static void athrill_syscall_fseek(AthrillSyscallArgType *arg);
static void athrill_syscall_set_virtfs_top(AthrillSyscallArgType *arg);
static void athrill_syscall_fflush(AthrillSyscallArgType *arg);

static void athrill_syscall_ev3_opendir(AthrillSyscallArgType *arg);
static void athrill_syscall_ev3_readdir(AthrillSyscallArgType *arg);
static void athrill_syscall_ev3_closedir(AthrillSyscallArgType *arg);



static struct athrill_syscall_functable syscall_table[SYS_API_ID_NUM] = {
    { athrill_syscall_none },
    { athrill_syscall_socket },
    { athrill_syscall_sense },
    { athrill_syscall_bind },
    { athrill_syscall_listen },
    { athrill_syscall_accept },
    { athrill_syscall_connect },
    { athrill_syscall_select },
    { athrill_syscall_send },
    { athrill_syscall_recv },
    { athrill_syscall_shutdown },
    { athrill_syscall_system },
    { athrill_syscall_malloc },
    { athrill_syscall_calloc },
    { athrill_syscall_realloc },
    { athrill_syscall_free },

    { athrill_syscall_open_r },
    { athrill_syscall_read_r },
    { athrill_syscall_write_r },
    { athrill_syscall_close_r },
    { athrill_syscall_lseek_r },
    { athrill_syscall_set_virtfs_top },

    { athrill_syscall_ev3_opendir },
    { athrill_syscall_ev3_readdir },
    { athrill_syscall_ev3_closedir },
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
    	printf("ERROR:%s(): errno=%d\n", __FUNCTION__, errno);
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

    switch (arg->body.api_sense.api_id) {
    case SYS_API_ID_CONNECT:
        retval = select(arg->body.api_sense.sockfd + 1, NULL, &fds, NULL, &tv);
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
            return;
        }
        else {
            arg->ret_value = -val;
        }
    }
    return;
}


static void athrill_syscall_bind(AthrillSyscallArgType *arg)
{
    Std_ReturnType err;
    struct sockaddr_in server_addr;
    struct sys_sockaddr_in *sockaddrp;
    int yes = 1;

    err = mpu_get_pointer(0U, arg->body.api_bind.sockaddr, (uint8 **)&sockaddrp);
    if (err != 0) {
        return;
    }
    err = setsockopt(arg->body.api_bind.sockfd, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes));
    if (err != 0) {
        return;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = PF_INET;
    server_addr.sin_addr.s_addr = (sockaddrp->sin_addr);
    server_addr.sin_port = (sockaddrp->sin_port);

    int ret = bind(arg->body.api_bind.sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (ret < 0) {
        arg->ret_value = -errno;
    }
    else {
        arg->ret_value = SYS_API_ERR_OK;
    }

    return;
}


static void athrill_syscall_listen(AthrillSyscallArgType *arg)
{
    int ret = listen(arg->body.api_listen.sockfd, arg->body.api_listen.backlog);
    if (ret < 0) {
        arg->ret_value = -errno;
    }
    else {
        arg->ret_value = SYS_API_ERR_OK;
    }
    return;
}

static void athrill_syscall_accept(AthrillSyscallArgType *arg)
{
    Std_ReturnType err;
    struct sockaddr_in client_addr;
    struct sys_sockaddr_in *sockaddrp;
    socklen_t addrlen;
    sys_uint32 *addrlenp;

    err = mpu_get_pointer(0U, arg->body.api_accept.sockaddr, (uint8 **)&sockaddrp);
    if (err != 0) {
        return;
    }
    err = mpu_get_pointer(0U, arg->body.api_accept.addrlen, (uint8 **)&addrlenp);
    if (err != 0) {
        return;
    }

    memset(&client_addr, 0, sizeof(client_addr));
    addrlen = sizeof(client_addr);
    int ret = accept(arg->body.api_accept.sockfd, (struct sockaddr *)&client_addr, &addrlen);
    if (ret < 0) {
        arg->ret_value = -errno;
    }
    else {
        sockaddrp->sin_family = PF_INET;
        sockaddrp->sin_port = ntohs(client_addr.sin_port);
        sockaddrp->sin_addr = ntohl(client_addr.sin_addr.s_addr);
        *addrlenp = addrlen;
        arg->ret_value = ret;
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
    client_addr.sin_addr.s_addr = (sockaddrp->sin_addr);
    client_addr.sin_port = (sockaddrp->sin_port);

    int ret = connect(arg->body.api_connect.sockfd, (struct sockaddr *)&client_addr, sizeof(client_addr));
    if (ret < 0) {
        arg->ret_value = -errno;
    }
    else {
        arg->ret_value = SYS_API_ERR_OK;
    }

    return;
}

static sys_int32 fd_set_copy(unsigned char *dst, unsigned char *src)
{
    sys_int32 i;
    sys_int32 j;
    sys_int32 count = 0;
    sys_int32 fd_size = sizeof(fd_set);
    sys_int32 num = (fd_size < sizeof(sys_fd_set)) ? fd_size: sizeof(sys_fd_set);

    if ((dst == NULL) || (src == NULL)) {
        return 0;
    }

    for (i = 0; i < num; i++) {
        dst[i] = src[i];
        //printf("fd_set_copy:dst[%d]=0x%x\n", i, dst[i]);
        //printf("fd_set_copy:src[%d]=0x%x\n", i, src[i]);
        for (j = 0; j < 8; j++) {
            if ( (dst[i] & (1 << j)) != 0) {
                count++;
            }
        }
    }
    return count;
}

static void athrill_syscall_select(AthrillSyscallArgType *arg)
{
    Std_ReturnType err;
    sys_fd_set *sys_readfds;
    sys_fd_set *sys_writefds;
    sys_fd_set *sys_exceptfds;
    fd_set readfds;
    fd_set writefds;
    fd_set exceptfds;
    struct timeval tmo;
    sys_int32 count = 0;

    if (arg->body.api_select.readfds == 0) {
        sys_readfds = NULL;
    }
    else {
        err = mpu_get_pointer(0U, (uint32)arg->body.api_select.readfds, (uint8 **)&sys_readfds);
        if (err != 0) {
            return;
        }
    }
    if (arg->body.api_select.writefds == 0) {
        sys_writefds = NULL;
    }
    else {
        err = mpu_get_pointer(0U, (uint32)arg->body.api_select.writefds, (uint8 **)&sys_writefds);
        if (err != 0) {
            return;
        }
    }
    if (arg->body.api_select.exceptfds == 0) {
        sys_exceptfds = NULL;
    }
    else {
        err = mpu_get_pointer(0U, (uint32)arg->body.api_select.exceptfds, (uint8 **)&sys_exceptfds);
        if (err != 0) {
            return;
        }
    }
    FD_ZERO(&readfds);
    FD_ZERO(&writefds);
    FD_ZERO(&exceptfds);
    count = fd_set_copy((unsigned char*)&readfds, (unsigned char*)sys_readfds);
    count += fd_set_copy((unsigned char*)&writefds, (unsigned char*)sys_writefds);
    count += fd_set_copy((unsigned char*)&exceptfds, (unsigned char*)sys_exceptfds);
    tmo.tv_sec = 0;
    tmo.tv_usec = 0;
    int ret = select(sizeof(sys_fd_set) * 8, &readfds, &writefds, &exceptfds, &tmo);
    if (ret < 0) {
        arg->ret_value = -errno;
    }
    else {
        if (sys_readfds != NULL) {
            memset((unsigned char*)sys_readfds, 0, sizeof(sys_fd_set));
        }
        if (sys_writefds != NULL) {
            memset((unsigned char*)sys_writefds, 0, sizeof(sys_fd_set));
        }
        if (sys_exceptfds != NULL) {
            memset((unsigned char*)sys_exceptfds, 0, sizeof(sys_fd_set));
        }

        count = fd_set_copy((unsigned char*)sys_readfds, (unsigned char*)&readfds);
        count += fd_set_copy((unsigned char*)sys_writefds, (unsigned char*)&writefds);
        count += fd_set_copy((unsigned char*)sys_exceptfds, (unsigned char*)&exceptfds);
        arg->ret_value = count;
    }

    return;
}
static void athrill_syscall_send(AthrillSyscallArgType *arg)
{
    Std_ReturnType err;
    char *bufp;
    ssize_t ret;

    err = mpu_get_pointer(0U, arg->body.api_send.buf, (uint8 **)&bufp);
    if (err != 0) {
        return;
    }
    ret = send(arg->body.api_send.sockfd, bufp, arg->body.api_send.len, MSG_DONTWAIT);
    if (ret < 0) {
        arg->ret_value = -errno;
    }
    arg->ret_value = ret;
    return;
}

static void athrill_syscall_recv(AthrillSyscallArgType *arg)
{
    Std_ReturnType err;
    char *bufp;
    ssize_t ret;

    err = mpu_get_pointer(0U, arg->body.api_recv.buf, (uint8 **)&bufp);
    if (err != 0) {
        return;
    }
    ret = recv(arg->body.api_recv.sockfd, bufp, arg->body.api_recv.len, MSG_DONTWAIT);
    if (ret < 0) {
        arg->ret_value = -errno;
    }
    arg->ret_value = ret;
    return;
}

static void athrill_syscall_shutdown(AthrillSyscallArgType *arg)
{
    arg->ret_value = SYS_API_ERR_OK;
    (void)close(arg->body.api_shutdown.sockfd);
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

static void athrill_syscall_malloc(AthrillSyscallArgType *arg)
{
    if (arg->body.api_malloc.size == 0) {
        arg->body.api_malloc.rptr = 0;
    }
    else {
        arg->body.api_malloc.rptr = mpu_malloc_get_memory(arg->body.api_malloc.size);
    }
    return;
}

static void athrill_syscall_calloc(AthrillSyscallArgType *arg)
{
    Std_ReturnType err;
    uint32 size;
    uint8 *addrp;

    if ((arg->body.api_calloc.size == 0) || (arg->body.api_calloc.nmemb == 0)) {
        arg->body.api_calloc.rptr = 0;
        return;
    }
    size = arg->body.api_calloc.size * arg->body.api_calloc.nmemb;

    arg->body.api_calloc.rptr = mpu_malloc_get_memory(size);
    if (arg->body.api_calloc.rptr == 0) {
        return;
    }

    err = mpu_get_pointer(0U, arg->body.api_calloc.rptr, (uint8 **)&addrp);
    ASSERT(err == 0);

    memset((void*)addrp, 0, size);
    return;
}

static void athrill_syscall_realloc(AthrillSyscallArgType *arg)
{
    Std_ReturnType err;
    uint8 *src_addrp;
    uint8 *dest_addrp;

    arg->body.api_realloc.rptr = mpu_malloc_get_memory(arg->body.api_realloc.size);
    if (arg->body.api_realloc.rptr == 0) {
        return;
    }

    err = mpu_get_pointer(0U, arg->body.api_realloc.ptr, (uint8 **)&src_addrp);
    ASSERT(err == 0);

    err = mpu_get_pointer(0U, arg->body.api_realloc.rptr, (uint8 **)&dest_addrp);
    ASSERT(err == 0);

    uint32 size = mpu_malloc_ref_size(arg->body.api_realloc.ptr);
 
    memcpy((void*)dest_addrp, (void*)src_addrp, size);

    mpu_malloc_rel_memory(arg->body.api_realloc.ptr);
    return;
}

static void athrill_syscall_free(AthrillSyscallArgType *arg)
{
    mpu_malloc_rel_memory(arg->body.api_free.ptr);

    return;
}

const char *virtual_file_top;
static char *getVirtualFileName(const char *file_name, char *buf)
{
    strcpy(buf,virtual_file_top);
    strcat(buf,"/");
    strcat(buf,file_name);
    return buf;
}

static void athrill_syscall_open_r(AthrillSyscallArgType *arg)
{
    Std_ReturnType err;
    char *file_name;
    int mode = arg->body.api_open_r.mode;
    char buf[255];
    int flags = arg->body.api_open_r.flags;
    int fd;

    err = mpu_get_pointer(0U, arg->body.api_open_r.file_name,(uint8**)&file_name);
    ASSERT(err == 0);
    fd = open(getVirtualFileName(file_name,buf), flags, mode); 

    //printf("open_r file=%s real_path=%s mode=%x fd=%d\n",file_name,buf,mode,fd);

    arg->ret_value = fd;

    return;

}
static void athrill_syscall_read_r(AthrillSyscallArgType *arg)
{
    Std_ReturnType err;
    char *buf;
    int fd = arg->body.api_read_r.fd;
    size_t size = (size_t)arg->body.api_read_r.size;

    err = mpu_get_pointer(0U, arg->body.api_read_r.buf,(uint8**)&buf);

    arg->ret_value = read(fd, buf, size);
    //printf("read_r fd=%d buf=0x%x(real:%p) size=%zu ret=%d\n",fd,arg->body.api_read_r.buf,buf,size,arg->ret_value);

    return;

}

static void athrill_syscall_write_r(AthrillSyscallArgType *arg)
{
    Std_ReturnType err;
    char *buf;
    int fd = arg->body.api_write_r.fd;
    size_t size = (size_t)arg->body.api_write_r.size;

    err = mpu_get_pointer(0U, arg->body.api_write_r.buf,(uint8**)&buf);

    arg->ret_value = write(fd, buf, size);

    //printf("write_r fd=%d buf=0x%x(real:%p) size=%zu ret=%d\n",fd,arg->body.api_write_r.buf,buf,size,arg->ret_value);

    return;

}
static void athrill_syscall_close_r(AthrillSyscallArgType *arg)
{
    int fd = arg->body.api_close_r.fd;
    arg->ret_value = close(fd);

    //printf("close_r fd=%d ret=%d\n",fd,arg->ret_value);

    return;

}

static void athrill_syscall_lseek_r(AthrillSyscallArgType *arg)
{
    int fd = arg->body.api_lseek_r.fd;
    off_t offset = arg->body.api_lseek_r.offset;
    int whence = arg->body.api_lseek_r.whence;

    arg->ret_value = lseek( fd, (size_t)offset, whence );

    //printf("lseek_r fd=%d offset=%d whence=%d ret=%d\n", fd, offset, whence, arg->ret_value);

    return;
}



static void athrill_syscall_set_virtfs_top(AthrillSyscallArgType *arg)
{
    arg->ret_value = -1;

    if ( virtual_file_top ) {
        // virtual_file_top is already set
    } else {
        Std_ReturnType err;
        char *buf;
        char *top_dir;

        err = mpu_get_pointer(0U, arg->body.api_set_virtfs_top.top_dir,(uint8**)&top_dir);
        ASSERT(err == 0);

        if ( (mkdir(top_dir,0777) == -1) && (errno != EEXIST) ) {
            printf("SYSCAL]set_virtfs_top mkdir failed path=%s errno=0x%x",
            top_dir, errno);
        } else {
            int len = strlen(top_dir);
            buf = malloc(len+1);
            strcpy(buf,top_dir);
            virtual_file_top = buf;
            arg->ret_value = 0;
            printf("SYSCAL]set_virtfs_top success path=%s",top_dir);

        }
    }
    return;
}

#include <sys/types.h>
#include <dirent.h>

struct dir_element {
    int is_used;
    char path[256+20]; // 20 for virtual topdir
    DIR *dir;
};
// this variable asuume that bss area is cleared with NULL
static struct dir_element dir_table[10]; // MAX 10

#define ENDOF(table) (table + sizeof(table)/sizeof(table[0]))
#define GETDIRID(p) (sys_int32)(p-dir_table +1 )
static struct dir_element *get_free_dir(void)
{
    struct dir_element *p;
    // TODO:thread safe
    for ( p = dir_table; p < ENDOF(dir_table); p++ ) {
        if ( !p->is_used ) {
            break;
        }
    }
    if ( p == ENDOF(dir_table) ) return 0;

    // found free space;
    p->is_used = 1;
    return p;
}

static struct dir_element* GETDIR(sys_int32 dirid)
{
    dirid--; // convert to index;
    ASSERT( dirid >= 0 && dirid < sizeof(dir_table)/sizeof(dir_table[0]));
    return dir_table+dirid;
}

static void release_dir(struct dir_element *p)
{
    p->is_used = 0;
}

static void athrill_syscall_ev3_opendir(AthrillSyscallArgType *arg)
{
    char *path = 0;
    char buf[256];
    struct dir_element *p = get_free_dir();

    ASSERT(virtual_file_top);

    if ( !p ) {
        arg->ret_value = -34; // E_NOID
    } else {
        Std_ReturnType err = mpu_get_pointer(0U, arg->body.api_ev3_opendir.path,(uint8**)&path);
        ASSERT(err == 0);
        p->dir = opendir(getVirtualFileName(path,buf));

        if ( !p->dir ) {
            switch( errno ) {
                case EACCES:
                case EBADF:
                case ENOTDIR:
                    arg->ret_value = -17; // E_PAR
                    break;
                default:
                    arg->ret_value = -17; // E_PAR
                    break;   
            }
            release_dir(p);
        } else {
            arg->ret_value = 0;
            arg->body.api_ev3_opendir.dirid = GETDIRID(p);
            strcpy(p->path,buf);
        }
    }

    // printf("ev3_opendir path=%s real_path=%s ret=%d dirid=%d\n",path,buf,arg->ret_value,arg->body.api_ev3_opendir.dirid);

    return;

}
        
static void athrill_syscall_ev3_readdir(AthrillSyscallArgType *arg)
{
    int dirid = arg->body.api_ev3_readdir.dirid;
    struct dir_element  *de= GETDIR(dirid);
    char path[255];

    path[0] = 0;
    if ( !de) {
        arg->ret_value = -18; // E_ID
    } else {
        Std_ReturnType err;
        DIR *dirp = de->dir;
        char *name;
        errno = 0;
        struct dirent *dir_ent;
        while ( dir_ent = readdir(dirp) ) {
            if ( strcmp(dir_ent->d_name,".") && strcmp(dir_ent->d_name,"..") ) break;
        }
        if ( dir_ent ) {
            err = mpu_get_pointer(0U, (uint32)arg->body.api_ev3_readdir.name,(uint8**)&name);
            ASSERT(err == 0);

            strcpy(name, dir_ent->d_name);

            strcpy(path,de->path);
            strcat(path,"/");
            strcat(path,name);
            struct stat stat_buf;
            if ( stat(path, &stat_buf) == 0 ) {
                // TODO: fix date/time handling
                struct tm *my_tm = localtime(&stat_buf.st_mtime);
                arg->body.api_ev3_readdir.date = my_tm->tm_yday;
                arg->body.api_ev3_readdir.time = my_tm->tm_hour * 60*60 + my_tm->tm_min*60 + my_tm->tm_sec;
                arg->body.api_ev3_readdir.size = stat_buf.st_size;
                arg->body.api_ev3_readdir.attrib = 0;
                if ( S_ISDIR(stat_buf.st_mode) ) arg->body.api_ev3_readdir.attrib |= ((1 << 0)); // TA_FILE_DIR
                // TODO: TA_FILE_HID,TA_FILE_RDO
                arg->ret_value = 0;
            } else {
                arg->ret_value = -5; // E_SYS
            }
        } else {
            switch (errno) {
                case 0:
                    arg->ret_value = -41; // E_OBJ
                    break;
                case EBADF:
                default:
                    arg->ret_value = -18; // E_ID
                    break;
            }
        }
    }
    /*
    if ( arg->ret_value == 0 ) {
        printf("ev3readdir dirid=%d ret=%d name=%s size=%d attrib=%d date=%d time=%d\n",
        dirid, arg->ret_value, path,arg->body.api_ev3_readdir.size, arg->body.api_ev3_readdir.attrib,
         arg->body.api_ev3_readdir.date,  arg->body.api_ev3_readdir.time);
    } else {
        printf("ev3readdir dirid=%d ret=%d path=%s errno=0x%x\n",
           dirid,  arg->ret_value , path, errno);
    }
    */
   return;
}


static void athrill_syscall_ev3_closedir(AthrillSyscallArgType *arg)
{
    int dirid = arg->body.api_ev3_closedir.dirid;
    struct dir_element  *de= GETDIR(dirid);

    int ret = closedir(de->dir);

    if ( ret == 0) {
        // Success
        release_dir(de);
        arg->ret_value = 0; // E_OK
    } else {
        arg->ret_value = -18; // E_ID
        
    }

//    printf("ev3closedir dirid=%d ret=%d", dirid, arg->ret_value );
    return;


}
