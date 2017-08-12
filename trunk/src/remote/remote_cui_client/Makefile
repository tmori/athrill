ROOTDIR		:= ../../
TARGETDIR	:= .
TARGET		:= athrill_remote

WFLAGS		:= -g -Wall
GCC			:= gcc

IFLAGS		:= -I$(ROOTDIR)/inc
IFLAGS		+= -I$(ROOTDIR)/lib
IFLAGS		+= -I$(ROOTDIR)/config

VPATH		:= $(TARGETDIR)
VPATH		+= $(ROOTDIR)/lib/winsock_wrapper
VPATH		+= $(ROOTDIR)/lib/udp



CFLAGS		:= $(WFLAGS)
CFLAGS		+= $(IFLAGS)

LFLAGS		:= -Wl,--allow-multiple-definition
LFLAGS		+= -Wl,--enable-stdcall-fixup

LIBS		:= -lpthread -lwsock32 -lws2_32

OBJS		:= main.o
OBJS		+= winsock_wrapper.o
OBJS		+= udp_comm.o


.SUFFIXES:	.c .o

all:	$(TARGET)

$(TARGET):	$(OBJS)
	$(GCC) -O3 $(LFLAGS) $(OBJS) -o $(TARGET)  $(LIBS)

.c.o:	$<
	$(GCC) -O3 -c $(CFLAGS) $<

clean:
	rm -f $(OBJS) $(TARGET).exe