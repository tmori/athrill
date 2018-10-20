TARGET	:= libacomm.a

IFLAGS	:= -I.
IFLAGS	+= -I../include
IFLAGS	+= -I../fixed
IFLAGS	+= -I../generated

VPATH	:= ../../../command
VPATH	+= ../fixed
VPATH	+= ../generated

CFLAGS	:= -g -Wall
CFLAGS	+= -O0

DFLAGS	:= -DACOMM_COMMAND

OBJS	:= acomm_init.o
OBJS	+= athrill_comm.o

.c.o:
	gcc -c $(CFLAGS) $(IFLAGS) $<

all: $(OBJS)
	ar -rcSv $(TARGET) $(OBS)
	ranlib $(TARGET)

clean:
	rm -f *.o
	rm -f $(TARGET)
