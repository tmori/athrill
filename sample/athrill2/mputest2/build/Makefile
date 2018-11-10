ROOT_PATH 		:= ..
ATHRILL_ROOT	:= $(ROOT_PATH)/../../../trunk/src
BUILD_PATH		:= .
CPU_CONFIG_PATH	:= $(ATHRILL_ROOT)/config/target/v850e2m


TARGET	:= mputest.elf

CC		:= v850-elf-gcc
LINK	:= v850-elf-gcc

KERNEL_OBJS	:= start.o
KERNEL_OBJS	+= vector.o
KERNEL_OBJS	+= interrupt.o
KERNEL_OBJS	+= interrupt_table.o
KERNEL_OBJS	+= timer.o
KERNEL_OBJS	+= interrupt_asm.o
KERNEL_OBJS	+= main.o
KERNEL_OBJS += kernel_task.o
KERNEL_OBJS += kernel_service.o
KERNEL_OBJS += svc_asm.o

USER_OBJS	:= user1_task.o
USER_OBJS	+= user2_task.o
USER_OBJS	+= user_shared_data.o

OBJS	:= $(KERNEL_OBJS) $(USER_OBJS)

CFLAGS	:= -O0 -mdisable-callt -mno-app-regs -mtda=0
#CFLAGS	+= -gdwarf-2 -Wall -Wno-unused-label -Wpointer-arith  -mv850e2v3 -Wa,-mno-bcond17 -msoft-float -Wa,-mwarn-signed-overflow -Wa,-mwarn-unsigned-overflow
CFLAGS	+= -gdwarf-2 -Wall -Wno-unused-label -Wpointer-arith  -mv850e2v3 -Wa,-mno-bcond17  -Wa,-mwarn-signed-overflow -Wa,-mwarn-unsigned-overflow


LDSCRIPT = $(BUILD_PATH)/v850e2m.ld
LDFLAGS	:= -nostdlib -T $(LDSCRIPT)

IFLAGS	:= -I.
IFLAGS	+= -I$(CPU_CONFIG_PATH)
IFLAGS	+= -I$(ROOT_PATH)/kernel
IFLAGS	+= -I$(ROOT_PATH)/user
IFLAGS	+= -I$(ROOT_PATH)/inc

VPATH	:= $(ROOT_PATH)/kernel
VPATH	+= $(ROOT_PATH)/user
VPATH	+= $(ROOT_PATH)/user1
VPATH	+= $(ROOT_PATH)/user2

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
