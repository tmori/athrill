TARGET1	:= athrill_mkfs
TARGET2	:= acomm_read

IFLAGS	:= -I.
IFLAGS	+= -I../include
IFLAGS	+= -I../fixed
IFLAGS	+= -I../generated

VPATH	:= .
VPATH	+= ../fixed
VPATH	+= ../generated

CFLAGS	:= -g -Wall
CFLAGS	+= -O0

DFLAGS	:=

OBJS	:= athrill_mkfs.o
OBJS	+= athrill_comm_generated_image.o
OBJS	+= acomm_read.o
OBJS	+= athrill_comm.o

ATHRILL_MKFS_OBS := athrill_mkfs.o athrill_comm_generated_image.o
ACOMM_READ_OBS := acomm_read.o athrill_comm.o

.c.o:
	gcc -c $(CFLAGS) $(IFLAGS) $<

all: $(OBJS)
	gcc -o $(TARGET1) $(CFLAGS) $(ATHRILL_MKFS_OBS)
	gcc -o $(TARGET2) $(CFLAGS) $(ACOMM_READ_OBS)

clean:
	rm -f *.o
	rm -f $(TARGET1)
	rm -f $(TARGET2)
