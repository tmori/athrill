GCC := gcc

LED	:= led
LED_OBJ := led.o

DSW	:= dip_switch
DSW_OBJ := dip_switch.o

PSW	:= push_button
PSW_OBJ := push_button.o

OBJS := dio.o
OBJS += tool_file.o
OBJS += tool_config.o

WFLAGS := -Wall

IFLAGS := -I.

.c.o:	$<
	$(GCC) -O3 -c $(WFLAGS) $(IFLAGS) $(DFLAGS) $<

all: $(OBJS) $(LED_OBJ) $(DSW_OBJ) $(PSW_OBJ)
	$(GCC) -o do_test test.c $(OBJS)
	$(GCC) -o $(LED) $(OBJS) $(LED_OBJ)
	$(GCC) -o $(DSW) $(OBJS) $(DSW_OBJ)
	$(GCC) -o $(PSW) $(OBJS) $(PSW_OBJ)

clean:
	rm -f *.o $(LED) $(DSW) $(PSW)
