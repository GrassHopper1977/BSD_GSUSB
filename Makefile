VER_MAJOR=0
VER_MINOR=1

CFLAGS=-O2 -Wall -cheri-bounds=subobject-safe
LIBFLAGS=-fPIC -shared
LFLAGS=-lusb -lm -lpthread
PURECAP = -mabi=purecap
HYBRID = -mabi=aapcs

SOURCEFILES = gsusb.c ./utils/timestamp.c
TESTFILES = test.c
HEADERFILES = gsusb.h
ALLFILES= $(SOURCEFILES) $(TESTFILES) $(HEADERFILES)

libGSUSB.so: $(ALLFILES)
	cc $(PURECAP) $(CFLAGS) $(LIBFLAGS) $(SOURCEFILES) $(LFLAGS) -olibGSUSB.so
	cp libGSUSB.so /usr/lib
	cc $(PURECAP) $(CFLAGS) $(TESTFILES) $(LFLAGS) -lGSUSB -otest
	cc $(HYBRID) $(CFLAGS) $(LIBFLAGS) $(SOURCEFILES) $(LFLAGS) -olibGSUSBhy.so
	cp libGSUSBhy.so /usr/lib64/libGSUSB.so
	cc $(HYBRID) $(CFLAGS) $(TESTFILES) $(LFLAGS) -lGSUSB -otest_hy

.PHONY: clean

clean:
	rm -f libGSUSB.so
