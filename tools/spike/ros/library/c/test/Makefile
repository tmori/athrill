TARGET	:= test.elf

IFLAGS	:= -I.
IFLAGS	+= -I../include
IFLAGS	+= -I../fixed
IFLAGS	+= -I../generated

VPATH	:= .
VPATH	+= ../fixed
VPATH	+= ../generated

CFLAGS	:= -Wall

DFLAGS	:=

OBJS	:= athrill_comm.o
OBJS	+= athrill_comm_generated_config.o
OBJS	+= acomm_test.o

.c.o:
	gcc -c $(CFLAGS) $(IFLAGS) $<

all: $(OBJS)
	gcc -o $(TARGET) $(CFLAGS) $(OBJS)

clean:
	rm -f *.o