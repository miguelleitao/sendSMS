/*
 *  sendSMS.c
 *
 *  Send Short Message through USB connected GSM adapter
 *
 *  Miguel Leitao, 2015
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include "sendSMS.h"

#define DEV_PORT	"/dev/ttyUSB1"

static char dev_port[24] = DEV_PORT;
static int debug = 1;
static int force_reset = 0;
static int simul = 0;

static void ErrorMsg(char *msg) {
  if ( debug ) fprintf(stderr,"Error: %s\n",msg);
}

static int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf(stderr, "error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        //         // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                               // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
// 	tty.c_cflag |= CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0) {
           fprintf(stderr, "error %d from tcsetattr", errno);
           return -1;
        }
        return 0;
                                                                                         }

static void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf(stderr, "error %d from tggetattr", errno);
                return;
        }
        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_lflag &= ~ICANON; /* Set non-canonical mode */

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                fprintf(stderr, "error %d setting term attributes", errno);
}

int usbReset() {
 return system("usbreset 19d2:0117");
} 

int WriteCmd(int fd, const char *msg) {
  int wr = 0;
  if ( debug>2 ) printf("> %s", msg );
  wr = write(fd, msg, strlen(msg));
  wr += write(fd, "\r\n", 2);
  if ( debug>2 ) printf(".\n");
  return wr;
}

static char *ReadRes(int fd) {
  static char buf[90];
  int rd;
  if ( debug>3 )
    printf("Reading...\n");
  rd = read(fd, buf, 80);
  if (rd < 0) {
	if ( debug ) perror("ReadRes: read error");
	buf[0] = '\0';
	return buf;
  }
  buf[rd] = 0;
  if ( rd>0 && debug>3 ) {
    printf("< %s", buf);
  }
  return buf;
}

 /*!
 *		ReadOK
 * 
 *		Read command result.
 *		Return 1 if result=='OK'.
 * 		0 otherwise.
 * 		Echo is supposed to be OFF.
 */
static int ReadOK(int fd) {
  char *res = ReadRes(fd);
  while ( *res==' ' || *res=='\n' || *res=='\r' ) res++;
  if ( strncmp(res, "OK", 2)==0 ) return 1;
  return 0;
}

 /*!
 *		ReadOKAT
 * 
 *		Read command result.
 *		Return 1 if result=='OK'.
 * 		0 otherwise.
 * 		Parses and consumes eventual command echo.
 */
static int ReadOKAT(int fd) {
  char *res = ReadRes(fd);
  while ( *res==' ' || *res=='\n' || *res=='\r' ) res++;
  if ( strncmp(res,"AT",2)==0 ) {
    // Echo is ON
    //res = ReadRes(fd);
    res += 2;
    while ( *res && *res!=' ' && *res!='\n' && *res!='\r' ) res++; 
    while ( *res==' ' || *res=='\n' || *res=='\r' ) res++;
  }
  
  if ( strncmp(res,"OK",2)==0 ) return 1;
  if ( *res ) return 0;
  return ReadOK(fd);
} 

/*!
 *		ReadUntilOK
 * 
 *		Read command result.
 *		Return 1 if result=='OK'.
 * 		0 otherwise.
 * 		Parses and consumes eventual command echo.
 */
static int ReadUntilOK(int fd) {
  char *res = ReadRes(fd);
  while ( *res==' ' || *res=='\n' || *res=='\r' ) res++;
  while ( strncmp(res,"AT",2)==0 ) {
    // Echo is ON
    //res = ReadRes(fd);
    res += 2;
    while ( *res && *res!=' ' && *res!='\n' && *res!='\r' ) res++; 
    while ( *res==' ' || *res=='\n' || *res=='\r' ) res++;
  }
  
  if ( strncmp(res,"OK",2)==0 ) return 1;
  if ( *res ) return 0;
  return ReadOK(fd);
} 

 /*!
 *		ReadString
 * 
 *		Read command result from modem device fd.
 *		Return 1 if result==goal.
 * 		0 otherwise.
 */
static char *ReadString(int fd, char *goal) {
  char *res, *pres;
  int n = 0;
  do {
    res = ReadRes(fd);
    pres = strstr(res, goal);
    if ( ! pres )
        pres = strstr(res, "ERROR");
    n++;
  } while ( pres==NULL && n<4 );
  return pres;
}

int controlModem(int pd, int level) {
	// AT+CFUN=0 turns off the USB modem.
	// AT+CFUN=1 turns on (restarts) the USB modem.
	char cmd[80];
	sprintf(cmd, "AT+CFUN=%d", level);
	WriteCmd(pd, cmd);
    usleep(10000);
    if ( ! ReadOKAT(pd) ) {
	    ErrorMsg("Modem not connected.");
	    close(pd);
	    return -4;
	}
	return 0;
}

int resetModem(int pd) {
	// ATZ
	WriteCmd(pd, "ATZ");
    usleep(10000);
    if ( ! ReadOKAT(pd) ) {
	    ErrorMsg("Modem not connected.");
	    close(pd);
	    return -4;
	}
	return 0;
}

	
int setupModem() {
  int nRun;
  int pd = -1;
  for( nRun=0 ; nRun<2 ; nRun++ ) {
    pd = open(dev_port, O_RDWR | O_SYNC ); //open(DEV_PORT, O_RDWR | O_NOCTTY | O_SYNC );
    if ( pd>=0 ) {
      set_interface_attribs (pd, B9600, 0);
      set_blocking(pd,1);
      fsync(pd);
      usleep(10000);
  
      // Test modem and turn echo OFF
      WriteCmd(pd, "ATE0");
      usleep(10000);
      if (  ReadOKAT(pd) ) break;
      ErrorMsg("Modem not working.");
      close(pd);
    }
    else {
        fprintf(stderr, "Error: Cannot open port '%s'\n", dev_port);
    }
    // Retry
    if ( debug ) fprintf(stderr, "Forcing USB Reset...\n");
    usbReset();
    usleep(10000);
  }
  if  ( pd<0 ) return -4;

  // Test Network Registing
  WriteCmd(pd, "AT+CREG");
  if ( ! ReadOK(pd) ) {
        ErrorMsg("Modem not registed.");
        close(pd);
        return -5;
  }

  //  Set SMS text mode
  WriteCmd(pd, "AT+CMGF=1");
  if ( ! ReadOK(pd) ) {
        ErrorMsg("SMS text mode not available.");
        close(pd);
        return -6;
  }
  if ( debug>4 ) {
        printf("Modem setup success.\n");
        WriteCmd(pd, "AT+CMEE=1");
        if ( ! ReadOK(pd) ) {
            ErrorMsg("SMS text mode not available.");
            close(pd);
            return -7;
        }
        printf("Extended error reporting mode.\n");
  }
  return pd;
}

char *checkSimPin(int pd) {
	WriteCmd(pd, "AT+CPIN?");
	char *res = ReadString(pd, "+CPIN:");
	if ( ! res || ! *res ) return NULL;
	return res+6;
}

int setSimPin(int pd, const char *pin) {
	char *cpin = checkSimPin(pd);
	if ( ! cpin ) {
		ErrorMsg("Checking PIN status,");
		return 0;
	}
	if ( ! strcmp(cpin, "READY") ) 	// Service is Ready.
		return 1;					// No pin required.
	// Set PIN
	char cmd[24];
	sprintf(cmd, "AT+CPIN=%s", pin);
	WriteCmd(pd, cmd);
	if ( ! ReadOK(pd) ) {
        ErrorMsg("PIN invalid.");
        return 0;
	}
    return 2;	// Success
}



 /*!
 *       SendSingleSMS
 * 
 *       Send a single message (msg) to a single receipient.
 *       using a previoulsy prepared modem channel (pd).
 */
int SendSingleSMS(int pd, char *num, char *msg) {
  // Destination
  char cmd[280];
  sprintf(cmd, "AT+CMGW=\"%s\"", num);
  WriteCmd(pd, cmd);
  ReadRes(pd);

  // Message
  WriteCmd(pd, msg);
  WriteCmd(pd, "\032");
  usleep(100);

  // Get stored message Idx
  char *mres = ReadString(pd, "+CMGW:");
  if ( strlen(mres)<1 || ! strncmp(mres,"ERROR",5) ) {
      ErrorMsg("Message not written.");
      return -8;
  }
  int mnum = atoi(mres+7);
  //printf("megnum:%d\n",mnum);

  // Send
  if ( ! simul ) {
    sprintf(cmd, "AT+CMSS=%d", mnum);
    WriteCmd(pd, cmd);
    usleep(400000);
    mres = ReadString(pd, "+CMSS:");
    if ( strlen(mres)<1 ) {
        ErrorMsg("Message not sent.");
        return -9;
    }
  }

  // Delete
  sprintf(cmd, "AT+CMGD=%d", mnum);
  WriteCmd(pd, cmd);
  if ( ! ReadOK(pd) ) {
        ErrorMsg("Message not deleted.");
        return -10;
  }
  
  // Success
  return 0;
}
 
 
 /*!
 *       GetListSMS
 * 
 *       Send a single message (msg) to a single receipient.
 *       using a previoulsy prepared modem channel (pd).
 */
int GetListSMS(int pd, int bSize, char *buffer) {
  // Destination
  char cmd[280];
  sprintf(cmd, "AT+CMGL=\"%s\"", "ALL");
  WriteCmd(pd, cmd);
  //ReadRes(pd);
  
  int len = 0;
  int rd = -1;
  char *buf = buffer;
  while( rd!=0 ) {
	  rd = read(pd, buf, bSize-len-1);
	  if (rd < 0) {
		if ( debug ) perror("ReadRes: read error");
		buf[0] = '\0';
		return len;
	  }
	  buf += rd;
	  len += rd;
  }
  buffer[len] = 0;
  if ( rd>0 && debug>3 ) {
    printf("< %s", buffer);
  }
  return len;
}

  
int DeleteSingleSMS(int pd, int mnum) {
  // Delete
  char cmd[280];
  sprintf(cmd, "AT+CMGD=%d", mnum);
  WriteCmd(pd, cmd);
  if ( ! ReadOK(pd) ) {
        ErrorMsg("Message not deleted.");
        return -10;
  }
  // Success
  return 0;
}

 
  
 /*!
 *       SendSMS
 * 
 *       Send a single message (msg) to a single receipient.
 */
int SendSMS(char *num, char *msg) {
  int pd = setupModem();
  if ( pd<0 ) return -1;

  SendSingleSMS(pd, num, msg);
  close(pd);
  return 0;
}

 /*!
 *       DeleteSMS
 * 
 *       Delete a single message (num).
 */
int DeleteSMS(char *num) {
  int pd = setupModem();
  if ( pd<0 ) return -1;
  DeleteSingleSMS(pd, atoi(num));
  close(pd);
  return 0;
}

int ListSMS() {
  int pd = setupModem();
  if ( pd<0 ) return -1;
  char smsText[2000];
  int res = GetListSMS(pd, 2000, smsText);
  puts(smsText);
  return res;
}


/*!
 *       SendBulkSMS
 * 
 *       Send a single message (msg) to multiple receipients.
 *       Receipients are passed in num_tab array of strings.
 */
int SendBulkSMS(char num_tab[MAX_BULK_DESTINATIONS][MAX_DESTINATION_LEN], char *msg) {
  int pd = setupModem();
  if ( pd<0 ) return -1;
  if ( debug )
	    printf("SendBulkSMS\n");
  for( int i=0; i<MAX_BULK_DESTINATIONS ; i++ ) {
	  char *num = num_tab[i];
	  if ( ! num || ! *num ) break;
	  if ( debug )
	    printf("SendBulkSMS, sending %lu bytes to '%s'\n", strlen(msg), num);

      SendSingleSMS(pd, num, msg);
      usleep(600000);
  }
  close(pd);
  return 0;
}

/*!
 *       SendBulkListSMS
 * 
 *       Send a single message (msg) to multiple receipients.
 *       Receipients are loaded at run time from the data file identified by fname.
 */
int SendBulkListSMS(char *fname, char *msg) {
  char num_tab[MAX_BULK_DESTINATIONS][MAX_DESTINATION_LEN];
  int nnums = 0; 	// Receipients index
  FILE *tabd = fopen(fname, "r");
  if ( ! tabd ) {
    fprintf(stderr, "Error opening destination list file '%s'.\n", fname);
    return -1;
  }
  while (fgets(num_tab[nnums], MAX_DESTINATION_LEN-2, tabd) != NULL) {  
    if ( *num_tab[nnums]=='#' ) continue;
	int nlen = strlen(num_tab[nnums])-1;
	while( nlen>=0 && num_tab[nnums][nlen] == '\n' ) {
		num_tab[nnums][nlen] = 0;
		nlen--;
	}
    if ( debug )
	  printf("SendBulkListSMS, got %d: '%s'\n", nnums, num_tab[nnums]);
	nnums++;
	if ( nnums>=MAX_BULK_DESTINATIONS-1 ) break;
  }
  fclose(tabd);
  num_tab[nnums][0] = 0;
  return SendBulkSMS(num_tab, msg);
}



#ifndef _LIB_

void Usage() {
  printf("Usage:\n  %s [options] DestNum Mesg\n", "SendSMS");
  printf("    Options:\n");
  printf("      -q      Quiet\n");
  printf("      -d      Show debug info\n");
  printf("      -D      Show full debug info\n");
  printf("	-l	List All SMS messages\n");
  printf("	-f	Force previous USB reset\n");
  printf("      -s      Simulate. Do not send message.\n");
  printf("      -i dev  Device. (Default: " DEV_PORT ").\n");
  printf("\n");
}

int main(int argc, char **argv) {
  if ( argc<3 ) {
	Usage();
	return -1;
  }
  int argp = 1;
  while ( argp<argc-2 && argv[argp][0]=='-' ) {
    switch (argv[argp][1]) {
      case 'd':
        debug = 3;
        break;
      case 'D':
        debug = 5;
        break;
      case 'q':
        debug = 0;
        break;
      case 's':
        simul = 1;
        break;
      case 'i':
        argp++;
        strncpy(dev_port, argv[argp], 22);
        break;
      case 'f':
	force_reset = 1;
	break;
      case 'l':
        return ListSMS();
      default:
        fprintf(stderr,"Bad option '-%c'\n", argv[argp][1]);
        Usage();
        return -2;
    }
    argp++;
  }
  if ( force_reset ) usbReset();
  if ( argv[argp][0] == '@' )
      return SendBulkListSMS(argv[argp]+1, argv[argp+1]);
  return SendSMS(argv[argp],argv[argp+1]);
}

#endif
