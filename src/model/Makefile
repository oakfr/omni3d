include ../common.mk

SRC_ROOT:=..
LIB_PATH:=../../lib

CFLAGS := $(INCPATH) $(CFLAGS_NOOPT) $(CFLAGS_GTK) $(CFLAGS_GLIB) $(CFLAGS_GLADE) $(CFLAGS_IPP) \
		$(CFLAGS_JPEGCODEC) -I$(PROJECT_HOME)/src -I$(SRC_ROOT)/libpmk-1.5 \
		 -Wno-multichar -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE \
		 -ffast-math -msse2 -Wno-deprecated

LDFLAGS := $(LDFLAGS_GTK) $(LDFLAGS_GTK_UTIL) $(LDFLAGS_VL) \
		$(LDFLAGS_GLADE)  $(LDFLAGS_OPENGL) $(LDFLAGS_IPP) -lpthread $(LDFLAGS) -lglut -ljpegcodec \
		-lippj -lippcc -lippi -lipps -lippcore -lsift -lippflow -lcommon -ldetect -ltrack -limage -lsimcam -lmath -lcamera -lvl

fw_ui:=fw_ui
obj_fw_ui:=ui.o  events.o util.o interface.o draw.o frame.o ../libpmk-1.5/libpmk.o ../libpmk-1.5/kernel/kernel-matrix.o

targets:=$(fw_ui)
objects:=$(obj_fw_ui)

gladefiles:=../../bin/glade/ui.glade

all: $(targets) $(gladefiles)

$(fw_ui): $(obj_fw_ui)
	@echo "    [$@]"
	$(CC) -o $@ $(obj_fw_ui) $(LDFLAGS) 

../../bin/glade/%.glade: %.glade
	@echo "    [$@]"
	cp -f $< $@

%.o: %.cpp %.h
	@echo "    [$@]"
	$(CC) $(CFLAGS) -c $< 

%.o: %.cpp
	@echo "    [$@]"
	$(CC) $(CFLAGS) -c $< 

clean:
	rm -f *.o fw_ui *~ $(gladefiles)
#$(targets) $(objects)
