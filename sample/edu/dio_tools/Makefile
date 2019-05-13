GCC := gcc

LED	:= led
LED_OBJ := led.o

OBJS := dio.o
OBJS += tool_file.o
OBJS += tool_config.o

WFLAGS := -Wall

IFLAGS := -I.

.c.o:	$<
	$(GCC) -O3 -c $(WFLAGS) $(IFLAGS) $(DFLAGS) $<

all: $(OBJS) $(LED_OBJ)
	$(GCC) -o do_test test.c $(OBJS)
	$(GCC) -o $(LED) $(OBJS) $(LED_OBJ)

clean:
	rm -f *.o $(LED)
