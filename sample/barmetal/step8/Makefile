ROOT_PATH 		:= ..
APL_DIR		:= ../../../trunk/apl
ATHRILL_ROOT	:= $(ROOT_PATH)/../../trunk/src
BUILD_PATH		:= .
CPU_CONFIG_PATH	:= $(ATHRILL_ROOT)/config/target/v850e2m


TARGET	:= test_main.elf

CC		:= v850-elf-gcc

LINK	:= v850-elf-gcc

OBJS	:= start.o
OBJS	+= vector.o
OBJS	+= interrupt.o
OBJS	+= interrupt_table.o
OBJS	+= timer.o
OBJS	+= interrupt_asm.o
OBJS	+= software_init.o
OBJS	+= main.o

CXXFLAGS := -fno-use-cxa-atexit -fno-exceptions -fno-rtti 
CFLAGS	:= -O0 -mdisable-callt -mno-app-regs -mtda=0
CFLAGS	+= -gdwarf-2 -Wall -Wno-unused-label -Wpointer-arith  -mv850e2v3 -Wa,-mno-bcond17 -msoft-float -Wa,-mwarn-signed-overflow -Wa,-mwarn-unsigned-overflow


LDSCRIPT = $(BUILD_PATH)/v850esfk3.ld
LDFLAGS	:= -nostdlib -T $(LDSCRIPT)

TEST_PATH := .

IFLAGS	:= -I.
IFLAGS	+= -I./list
IFLAGS	+= -I$(ROOT_PATH)/common
IFLAGS	+= -I$(CPU_CONFIG_PATH)
IFLAGS	+= -I$(APL_DIR)/include

VPATH	:= $(ROOT_PATH)/common
VPATH	+= $(TEST_PATH)

LIBS := -lm -lc -lgcc  libathrillc.o


%.o: %.S
	$(CC) -c $(IFLAGS) $(CFLAGS) $<

%.o: %.c
	$(CC) -c $(IFLAGS) $(CFLAGS) $<

all: $(OBJS)
	v850-elf-gcc -c $(IFLAGS) $(CFLAGS) libathrillc.c
	$(LINK) $(CFLAGS) $(LDFLAGS) -o $(TARGET) $(OBJS) -Wl,-Map,$(TARGET).map $(LIBS)
	v850-elf-objdump -d $(TARGET) > $(TARGET).dump



clean:
	rm -f $(OBJS) $(TARGET) libathrillc.o *.map
