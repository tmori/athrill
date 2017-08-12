ROOTDIR		:= ../../..

include Makefile.env

LIBCUI		:= libcui.a
LIBCPU		:= libcpu.a
LIBDEVICE	:= libdevice.a
LIBBUS		:= libbus.a
LIBMPU		:= libmpu.a
LIBLOADER	:= libloader.a
LIBSTD		:= libstd.a
LIBMAIN		:= libmain.a


AROBJS		:= $(LIBMAIN)
AROBJS		+= $(LIBCUI)
AROBJS		+= $(LIBCPU)
AROBJS		+= $(LIBDEVICE)
AROBJS		+= $(LIBBUS)
AROBJS		+= $(LIBMPU)
AROBJS		+= $(LIBLOADER)
AROBJS		+= $(LIBSTD)


all: $(AROBJS)
	$(RANLIB) $(AROBJS)
	$(GCC) -O3 $(LFLAGS) $(AROBJS) -o $(TARGET)  $(LIBS)
	$(CP) $(TARGET).exe $(ROOTDIR)/bin/

build_all:
	make -f Makefile.main all
	make -f Makefile.cui all
	make -f Makefile.cpu all
	make -f Makefile.device all
	make -f Makefile.bus all
	make -f Makefile.loader all
	make -f Makefile.mpu all
	make -f Makefile.std all
	$(RANLIB) $(AROBJS)
	$(GCC) -O3 $(LFLAGS) $(AROBJS) -o $(TARGET)  $(LIBS)
	$(CP) $(TARGET).exe $(ROOTDIR)/bin/

$(LIBMAIN):
	make -f Makefile.main all

$(LIBCUI): 
	make -f Makefile.cui all

$(LIBCPU): 
	make -f Makefile.cpu all

$(LIBDEVICE): 
	make -f Makefile.device all

$(LIBBUS): 
	make -f Makefile.bus all

$(LIBLOADER):
	make -f Makefile.loader all

$(LIBMPU): 
	make -f Makefile.mpu all

$(LIBSTD):
	make -f Makefile.std all

clean:
	make -f Makefile.cpu clean
	make -f Makefile.device clean
	make -f Makefile.bus clean
	make -f Makefile.cui clean
	make -f Makefile.loader clean
	make -f Makefile.mpu clean
	make -f Makefile.std clean
	make -f Makefile.main clean
	$(RM) -f $(TARGET).exe 
		
