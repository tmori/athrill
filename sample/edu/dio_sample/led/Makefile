SAMPLE_NAME		:= led
ROOT_PATH 		:= ..
ATHRILL_ROOT	:= $(ROOT_PATH)/../../../trunk/src
BUILD_PATH		:= .
BIN_PATH		:= .
APL_DIR			:= .
CPU_CONFIG_PATH	:= $(ATHRILL_ROOT)/config/target/v850esfk3


TARGET	:= $(SAMPLE_NAME)_sample.elf

CC		:= v850-elf-gcc
LINK	:= v850-elf-gcc

OBJS	:= start.o
OBJS	+= vector.o
OBJS	+= interrupt.o
OBJS	+= interrupt_table.o
OBJS	+= timer.o
OBJS	+= interrupt_asm.o
OBJS	+= sample.o
OBJS	+= main.o


CFLAGS	:= -O0 -mdisable-callt -mno-app-regs -mtda=0
CFLAGS	+= -gdwarf-2 -Wall -Wno-unused-label -Wpointer-arith  -mv850e2 -Wa,-mno-bcond17 -Wa,-mwarn-signed-overflow -Wa,-mwarn-unsigned-overflow


LDSCRIPT = $(ROOT_PATH)/common/v850esfk3.ld
LDFLAGS	:= -nostdlib -T $(LDSCRIPT)

TEST_PATH := .

IFLAGS	:= -I.
IFLAGS	+= -I$(ROOT_PATH)/common
IFLAGS	+= -I$(CPU_CONFIG_PATH)

VPATH	:= $(ROOT_PATH)/common
VPATH	+= $(APL_DIR)

LIBS := -lm -lgcc -lc


%.o: %.S
	$(CC) -c $(IFLAGS) $(CFLAGS) $<

%.o: %.c
	$(CC) -c $(IFLAGS) $(CFLAGS) $<

all: $(OBJS)
	$(LINK) $(CFLAGS) $(LDFLAGS) -o $(TARGET) $(OBJS) -Wl,-Map,$(TARGET).map $(LIBS)
	v850-elf-objdump -d $(TARGET) > $(TARGET).dump



clean:
	rm -f $(OBJS) $(TARGET) *.map *.dump
