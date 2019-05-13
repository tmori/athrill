GCC := gcc

OBJS := dio.o
OBJS += tool_file.o
OBJS += tool_config.o

WFLAGS := -Wall

IFLAGS := -I.

.c.o:	$<
	$(GCC) -O3 -c $(WFLAGS) $(IFLAGS) $(DFLAGS) $<

all: $(OBJS)
	$(GCC) -o do_test test.c $(OBJS)
clean:
	rm -f *.o
