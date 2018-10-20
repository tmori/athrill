TARGET1	:= athrill_mkfs
TARGET2	:= acomm_read
TARGET3	:= acomm_write
TARGET4	:= acomm_stat

IFLAGS	:= -I.
IFLAGS	+= -I../library/c/include
IFLAGS	+= -I../library/c/fixed
IFLAGS	+= -I../library/c/generated

VPATH	:= .
VPATH	+= ../library/c/fixed
VPATH	+= ../library/c/generated

CFLAGS	:= -g -Wall
CFLAGS	+= -O0

DFLAGS	:= -DACOMM_COMMAND

OBJS	:= athrill_mkfs.o
OBJS	+= athrill_comm_generated_image.o
OBJS	+= acomm_init.o
OBJS	+= acomm_read.o
OBJS	+= acomm_write.o
OBJS	+= acomm_stat.o
OBJS	+= athrill_comm.o

ATHRILL_MKFS_OBS := athrill_mkfs.o athrill_comm_generated_image.o
ACOMM_READ_OBS := acomm_init.o acomm_read.o athrill_comm.o
ACOMM_WRITE_OBS := acomm_init.o acomm_write.o athrill_comm.o
ACOMM_STAT_OBS := acomm_init.o acomm_stat.o athrill_comm.o

.c.o:
	gcc -c $(CFLAGS) $(IFLAGS) $<

all: $(OBJS)
	gcc -o $(TARGET1) $(CFLAGS) $(ATHRILL_MKFS_OBS)
	gcc -o $(TARGET2) $(CFLAGS) $(ACOMM_READ_OBS)
	gcc -o $(TARGET3) $(CFLAGS) $(ACOMM_WRITE_OBS)
	gcc -o $(TARGET4) $(CFLAGS) $(ACOMM_STAT_OBS)

clean:
	rm -f *.o
	rm -f *.bin
	rm -f $(TARGET1)
	rm -f $(TARGET2)
	rm -f $(TARGET3)
	rm -f $(TARGET4)
