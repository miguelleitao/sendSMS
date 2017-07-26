
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#define DEV_PORT	"/dev/ttyUSB1"

int debug=1;
int simul = 0;

void Usage() {
  printf("Usage:\n  %s [options] DestNum Mesg\n", "SendSMS");
  printf("    Options:\n");
  printf("      -q      Quiet\n");
  printf("      -d      Show debug info\n");
  printf("      -D      Show full debug info\n");
  printf("      -s      Simulate. Do not send message.\n");
  printf("\n");
}

void ErrorMsg(char *msg) {
  if ( debug ) fprintf(stderr,"Error: %s\n",msg);
}

int
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

void
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

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                fprintf(stderr, "error %d setting term attributes", errno);
}

int WriteCmd(int fd, const char *msg) {
  int wr = 0;
  if ( debug>2 ) printf("> %s\n", msg );
  wr = write(fd, msg, strlen(msg));
  wr += write(fd, "\r\n", 2);
  return wr;
}

char *ReadRes(int fd) {
  static char buf[90];
  int rd;
  rd = read(fd, buf, 80);
  buf[rd] = 0;
  if ( rd>0 && debug>3 ) {
    printf("< %s", buf);
  }
  return buf;
}

int ReadOK(int fd) {
  char *res = ReadRes(fd);
  while ( *res==' ' || *res=='\n' || *res=='\r' ) res++;
  if ( strncmp(res,"OK",2)==0 ) return 1;
  return 0;
}

char *ReadString(int fd, char *goal) {
  char *res, *pres;
  do {
    res = ReadRes(fd);
    pres = strstr(res, goal);
  } while ( pres==NULL );
  return pres;
}

  
int SendSMS(char *num, char *msg) {
  int pd = open(DEV_PORT, O_RDWR | O_SYNC );//open(DEV_PORT, O_RDWR | O_NOCTTY | O_SYNC );
  if ( pd<0 ) {
        fprintf(stderr, "Error: Cannot open port '%s'\n", DEV_PORT);
        return -1;
  }
  set_interface_attribs (pd, B9600, 0);
  set_blocking(pd,1);
  fsync(pd);

  // Test modem
  WriteCmd(pd, "AT");
  if ( ! ReadOK(pd) ) {
	ErrorMsg("Modem not connected.");
	close(pd);
	return -4;
  }

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

  // Destination
  char cmd[280];
  sprintf(cmd, "AT+CMGW=\"%s\"", num);
  WriteCmd(pd, cmd);
  ReadRes(pd);

  // Message
  WriteCmd(pd, msg);
  WriteCmd(pd, "\032");

  char *mres = ReadString(pd, "+CMGW:");
  if ( strlen(mres)<1 ) {
	ErrorMsg("Message not written.");
        close(pd);
        return -8;
  }
  int mnum = atoi(mres+7);

  //printf("megnum:%d\n",mnum);

  // Send
  if ( ! simul ) {
    sprintf(cmd, "AT+CMSS=%d", mnum);
    WriteCmd(pd, cmd);
    sleep(1);
    mres = ReadString(pd, "+CMSS:");
    if ( strlen(mres)<1 ) {
        ErrorMsg("Message not sent.");
        close(pd);
        return -9;
    }
  }

  // Delete
  sprintf(cmd, "AT+CMGD=%d", mnum);
  WriteCmd(pd, cmd);
  if ( ! ReadOK(pd) ) {
        ErrorMsg("Message not deleted.");
        close(pd);
        return -10;
  }

  close(pd);
  return 0;
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
      default:
	fprintf(stderr,"Bad option '-%c'\n", argv[argp][1]);
	Usage();
	return -2;
    }
    argp++;
  }
  return SendSMS(argv[argp],argv[argp+1]);
}

