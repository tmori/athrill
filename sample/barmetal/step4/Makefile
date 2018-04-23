ROOT_PATH 		:= ..
ATHRILL_ROOT	:= $(ROOT_PATH)/../../trunk/src
BUILD_PATH		:= $(ROOT_PATH)/build/v850esfk3
CPU_CONFIG_PATH	:= $(ATHRILL_ROOT)/config/target/v850esfk3


TARGET	:= test_main.elf

CC		:= v850-elf-gcc
LINK	:= v850-elf-gcc

OBJS	:= start.o
OBJS	+= vector.o
OBJS	+= training.o
OBJS	+= main.o
OBJS	+= list.o
OBJS	+= list_private.o
OBJS	+= list_test.o
OBJS	+= test_suite.o
OBJS	+= test_instruction.o


CFLAGS	:= -O0 -mdisable-callt -mno-app-regs -mtda=0
CFLAGS	+= -gdwarf-2 -Wall -Wno-unused-label -Wpointer-arith  -mv850e2 -Wa,-mno-bcond17 -msoft-float -Wa,-mwarn-signed-overflow -Wa,-mwarn-unsigned-overflow


LDSCRIPT = $(BUILD_PATH)/v850esfk3.ld
LDFLAGS	:= -nostdlib -T $(LDSCRIPT)

TEST_PATH := .

IFLAGS	:= -I.
IFLAGS	+= -I./list
IFLAGS	+= -I$(ROOT_PATH)/common
IFLAGS	+= -I$(CPU_CONFIG_PATH)

VPATH	:= $(ROOT_PATH)/common
VPATH	+= $(TEST_PATH)
VPATH	+= $(TEST_PATH)/list

LIBS := -lm -lgcc -lc


%.o: %.S
	$(CC) -c $(IFLAGS) $(CFLAGS) $<

%.o: %.c
	$(CC) -c $(IFLAGS) $(CFLAGS) $<

all: $(OBJS)
	$(LINK) $(CFLAGS) $(LDFLAGS) -o $(TARGET) $(OBJS) -Wl,-Map,$(TARGET).map $(LIBS)
	v850-elf-objdump -d $(TARGET) > $(TARGET).dump



clean:
	rm -f $(OBJS) $(TARGET) *.map
