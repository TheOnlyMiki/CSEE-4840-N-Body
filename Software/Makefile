ifneq ($(KERNELRELEASE),)

obj-m += accelerator_driver.o
obj-m += display_driver.o

else

CC ?= cc
KDIR ?= /usr/src/linux-headers-$(shell uname -r)
PWD := $(shell pwd)

CFLAGS += -Wall -Wextra -pthread
LDLIBS += -lusb-1.0 -pthread

APP = nbody_app
OBJECTS = main.o nbody.o display.o usbkeyboard.o

TARFILES = Makefile main.c nbody.c nbody.h nbody_ioctl.h \
        display.h display.c display_ioctl.h \
        usbkeyboard.h usbkeyboard.c accelerator_driver.c display_driver.c

.PHONY: all app modules modules-clean clean distclean

all: app

app: $(APP)

$(APP): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $(OBJECTS) $(LDLIBS)

main.o: main.c display.h usbkeyboard.h nbody.h nbody_ioctl.h display_ioctl.h
nbody.o: nbody.c nbody.h usbkeyboard.h nbody_ioctl.h
display.o: display.c display.h display_ioctl.h nbody.h nbody_ioctl.h
usbkeyboard.o: usbkeyboard.c usbkeyboard.h

modules:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

modules-clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean

nbody_app.tar.gz: $(TARFILES)
	rm -rf nbody_app_dir
	mkdir nbody_app_dir
	ln $(TARFILES) nbody_app_dir
	tar zcf nbody_app.tar.gz nbody_app_dir
	rm -rf nbody_app_dir

clean:
	rm -rf $(OBJECTS) $(APP) nbody_app.tar.gz nbody_app_dir

distclean: clean modules-clean

endif
