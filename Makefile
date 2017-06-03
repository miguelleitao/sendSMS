
CFLAGS=-Wall -O2

LIBS=-lssl -lcrypto ${BASE64_LIB}

BASE64_LIB=-l b64

all: smsprocd send_sms

http_client.o: http_client.c
	${CC} -c ${CFLAGS} $<

smsprocd.o: smsprocd.c http_client.h
	${CC} -c ${CFLAGS} $<  

smsprocd: smsprocd.o http_client.o
	${CC}  $^ -o $@ ${LIBS}

send_sms.o: send_sms.c http_client.h
	${CC} -c ${CFLAGS} -DSTANDALONE $<

send_sms: send_sms.o http_client.o
	${CC}  $^ -o $@ ${LIBS}


