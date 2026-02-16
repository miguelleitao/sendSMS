# Makefile for sendSMS

sendSMS: sendSMS.c sendSMS.h
	cc -Wall -Wextra -O1 -o $@ sendSMS.c

all: sendSMS lib
	
lib: libsendSMS.a

libsendSMS.a: sendSMS.o
	ar r $@ $^
	
sendSMS.o: sendSMS.c sendSMS.h
	cc -Wall -Wextra -O1 -c -D _LIB_ $<

push:
	git add .
	git commit -m update
	git push

install:
	systemctl stop ModemManager
	systemctl disable ModemManager
	install sendSMS

clean:
	rm -rf *.o sendSMS libsendSMS.a
