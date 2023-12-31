VER_MAJOR=0
VER_MINOR=1

CFLAGS=-O2 -Wall -cheri-bounds=subobject-safe
LIBFLAGS=-fPIC -shared
LFLAGS=-lusb -lm -lpthread
PURECAP = -mabi=purecap
HYBRID = -mabi=aapcs

SOURCEFILES = gsusb.c ./utils/timestamp.c
TESTFILES = test.c
TEST2FILES = test2.c
HEADERFILES = gsusb.h
ALLFILES= $(SOURCEFILES) $(TESTFILES) $(TEST2FILES) $(HEADERFILES)

libGSUSB.so: $(ALLFILES)
	cc $(PURECAP) $(CFLAGS) $(LIBFLAGS) $(SOURCEFILES) $(LFLAGS) -olibGSUSB.so
	cp libGSUSB.so /usr/lib
	cc $(PURECAP) $(CFLAGS) $(TESTFILES) $(LFLAGS) -lGSUSB -otest
	cc $(PURECAP) $(CFLAGS) $(TEST2FILES) $(LFLAGS) -lGSUSB -otest2
	cc $(HYBRID) $(CFLAGS) $(LIBFLAGS) $(SOURCEFILES) $(LFLAGS) -olibGSUSBhy.so
	cp libGSUSBhy.so /usr/lib64/libGSUSB.so
	cc $(HYBRID) $(CFLAGS) $(TESTFILES) $(LFLAGS) -lGSUSB -otest_hy
	cc $(HYBRID) $(CFLAGS) $(TEST2FILES) $(LFLAGS) -lGSUSB -otest2_hy

.PHONY: clean

clean:
	rm -f libGSUSB.so
