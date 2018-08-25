TARGET	:= athrill_mkfs

IFLAGS	:= -I.
IFLAGS	+= -I../include
IFLAGS	+= -I../fixed
IFLAGS	+= -I../generated

VPATH	:= .
VPATH	+= ../fixed
VPATH	+= ../generated

CFLAGS	:= -Wall
CFLAGS	+= -O0

DFLAGS	:=

OBJS	:= athrill_mkfs.o
OBJS	+= athrill_comm_generated_image.o

.c.o:
	gcc -c $(CFLAGS) $(IFLAGS) $<

all: $(OBJS)
	gcc -o $(TARGET) $(CFLAGS) $(OBJS)

clean:
	rm -f *.o