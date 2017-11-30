# Makefile for sendSMS

sendSMS: sendSMS.c
	cc -Wall -O1 -o $@ sendSMS.c
	
lib: libsendSMS.a

libsendSMS.a: sendSMS.o
	ar r $@ $^
	
sendSMS.o: sendSMS.c
	cc -Wall -O1 -c -D _LIB_ $<

