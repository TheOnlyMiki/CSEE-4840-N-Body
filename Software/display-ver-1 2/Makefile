CFLAGS = -Wall

OBJECTS = main.o nbody.o framebuffer.o usbkeyboard.o

TARFILES = Makefile main.c nbody.c nbody.h nbody_ioctl.h \
	framebuffer.h framebuffer.c \
	usbkeyboard.h usbkeyboard.c

nbody_app : $(OBJECTS)
	cc $(CFLAGS) -o nbody_app $(OBJECTS) -lusb-1.0 -pthread

nbody_app.tar.gz : $(TARFILES)
	rm -rf nbody_app_dir
	mkdir nbody_app_dir
	ln $(TARFILES) nbody_app_dir
	tar zcf nbody_app.tar.gz nbody_app_dir
	rm -rf nbody_app_dir

main.o : main.c framebuffer.h usbkeyboard.h nbody.h nbody_ioctl.h
nbody.o : nbody.c nbody.h framebuffer.h usbkeyboard.h nbody_ioctl.h
framebuffer.o : framebuffer.c framebuffer.h
usbkeyboard.o : usbkeyboard.c usbkeyboard.h

.PHONY : clean
clean :
	rm -rf *.o nbody_app
