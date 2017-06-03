/*
 * send_sms.c
 *
 * jml, 2015
 *
 * Sumbmit SMS message to delivery server.
 *
 */

#define _USE_OPENSSL_


#include <stdio.h>
#include <stdlib.h>

#ifndef SEND_SMS_MAIN

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#ifdef _USE_OPENSSL_
#include <openssl/rsa.h>
#include <openssl/engine.h>
#include <openssl/pem.h>
#include <openssl/err.h>
#include "b64/cencode.h"
#endif
#include "http_client.h"



#define StrLenght(x) ( (int)strlen((char *)x) )

      
// Host to contact. Connection is done using HTTP. 
#define HOST "www.magnocomp.com"

// Page on HOST
#define DEFAULT_PAGE "/sms/send_sms.php"

// Destination port, on HOST
#define PORT 80


#define USERAGENT "mag_http_client"
    
char LocalIP[16] = "";
char Name[80] = "mag_device";
char Id[24] = "";
//char HostIP[16];
int mcidr = -1;
int verbose = 0;


#ifdef _USE_OPENSSL_

RSA *DevicePrivateKey;

RSA *LoadPrivateKey(const char *fname) 
{
printf("Loading private key %s\n", fname);
        FILE *private_key_file = fopen(fname, "r");
        if ( private_key_file==NULL ) {
                fprintf(stderr, "Cannot open private key file '%s'\n",fname);
		return NULL;
	}
        RSA *rsa_private_key = PEM_read_RSAPrivateKey(private_key_file, NULL, NULL, "");
        fclose( private_key_file );
        if ( rsa_private_key==NULL ) 
                fprintf(stderr, "Cannot read private key from file\n");
	return rsa_private_key;
}


void LoadDevicePrivateKey()
{
    // DevicePrivateKey = LoadPrivateKey("openssl/device1privatekey.pem");
    DevicePrivateKey = LoadPrivateKey("magkey_priv.pem");
}
#endif

char LocalIP[16], Mask[16], Name[80], Id[24];

char *SetupSMS(char **txt_arr)
{
	int msg_size = 0;
	int i=0;

	for( i=0 ; txt_arr[i]!=NULL && txt_arr[i][0]!='\0' ; i++ ) {
		msg_size += strlen(txt_arr[i]) + 1;
	}
//printf("Adcionou totsize:%d\n",msg_size);
	if ( msg_size<1 || msg_size>32000 ) {
		fprintf(stderr,"Invalid message size.\n");
		return NULL;
	}

	char *msg = malloc(msg_size+4);
	if ( !msg ) {
		fprintf(stderr,"Could not allocate message.\n");
		return NULL;
	}
	msg[0] = '\0';
	for( i=0 ; txt_arr[i]!=NULL && txt_arr[i][0]!='\0' ; i++ ) {

		strcat(msg,txt_arr[i]);
		strcat(msg,"+");
	}
	msg[msg_size-1] = '\0';
//printf("Preparou '%s'\n",msg);
	return msg;
}

int SendSMS(char *dst, char *msg)  {	
printf("sending\n");
	if ( *LocalIP==(char)0 || *Id==(char)0 || mcidr<0 )  {
        	GetIpMaskMac(LocalIP,Mask,Id);
		//cidr mask
		mcidr = MaskCidr(Mask);
	}

	if ( *LocalIP==(char)0 ) {
		fprintf(stderr, "Invalid local IP\n");
		return 0;
	}
	if ( *Id==(char)0 ){
		fprintf(stderr, "Invalid device Id\n");
		return 0;
	}

	strcpy(Name, "magdevice");
	char *page = NULL;
        char *get_params = malloc(strlen(LocalIP)+strlen(Mask)+strlen(Name)+strlen(Id)+40);
        sprintf(get_params,"Dst=%s&Name=%s&Time=%ld&Id=%s&Txt=%s", dst, Name, time(NULL), Id, msg );

//printf("params='%s'\n",get_params);
       

	#ifdef _USE_OPENSSL_
		if ( DevicePrivateKey!=NULL )
		{
	            unsigned char *cipher = malloc(512);
	            unsigned int siglen;
/*
	            int rsa_res = RSA_sign(NID_sha1, (unsigned char *)get_params, strlen(get_params)+1,
   		        cipher, &siglen, DevicePrivateKey);
		    if ( ! rsa_res ) Error!!
*/
		    siglen = RSA_private_encrypt( StrLenght(get_params)+1, (unsigned char *)get_params,
			cipher, DevicePrivateKey, RSA_PKCS1_PADDING);
		    if ( siglen<=0 ) {
			fprintf(stderr, "Cannot Sign message.\n%s\n",ERR_error_string(ERR_get_error(),NULL));
		    }
		    else {
			// Message encripted with success.
			//printf("msg size: %d,  signed size:%d\n", StrLenght(get_params), siglen);
			char *msg = malloc(siglen*2);   // ~6*siglen/4
			int blen = base64_encode(cipher,msg,siglen);
			free(cipher);
			//printf("msg size(b64): %d,  msg:'%s'\n", blen, msg);

		    	page = malloc(strlen(msg)+strlen(DEFAULT_PAGE)+20);
        	    	if (page) sprintf(page,"%s?Id=%s&cod=%s", DEFAULT_PAGE, Id, msg);    
		    	free(msg);
		    }

// decode
/*
		    FILE *public_key_file = fopen("openssl/device1publickey.pem", "r");
            	    if ( public_key_file==NULL )
                	    fprintf(stderr, "Cannot open public key file\n");
            	    else {
			// Try to load public key with PEM_read_RSA_PUBKEY or PEM_read_RSAPublicKey.
			// "There are two incompatible public key formats.",
			// 	Dr. Stephen Henson, http://markmail.org/message/dknd65cykdmgyyr2
			RSA *rsa_public_key;
			rsa_public_key = 	PEM_read_RSA_PUBKEY(public_key_file, NULL, NULL, NULL);
			if ( ! rsa_public_key ) PEM_read_RSAPublicKey(public_key_file, NULL, NULL, NULL);
	                fclose( public_key_file );
	       	        if ( rsa_public_key==NULL )
	                        fprintf(stderr, "Cannot read public key from file.\n%s\n",ERR_error_string(ERR_get_error(),NULL));
	                else {
			    unsigned char *outbuf = malloc(512);
			    unsigned int outlen;
		   	    outlen = RSA_public_decrypt( siglen, cipher, outbuf, rsa_public_key, RSA_PKCS1_PADDING);
			    if ( outlen<=0 ) {
				fprintf(stderr, "Cannot decode message.\n%s\n",ERR_error_string(ERR_get_error(),NULL));
                    	    }
                    	    else {
				// message decripted
                        	printf("out msg size: %d,  string size:%d:'%s'\n", outlen, StrLenght(outbuf), outbuf);
                    	    }
			    free(outbuf);
			    RSA_free(rsa_public_key);

			}
		    }
*/ 					// decode

		}
	
#endif	// USE_SSL
	if ( page==NULL ) {	// SSL is OFF, PrivKey not found or Encriptation failed
 		page = malloc(strlen(get_params)+strlen(DEFAULT_PAGE)+20);
        	if (page) sprintf(page,"%s?%s", DEFAULT_PAGE, get_params);
	}

	free(get_params);
	int res = 0;
	if ( page ) {
	    UserAgent = USERAGENT;
	    //printf("submit http page: %s %s\n",HOST,page);
	    res = HttpGet(HOST, page);
	    printf("submit done %d\n",res);
	    free(page);
	}
	return res;
}


#else	// Not SEND_SMS_MAIN
#define STANDALONE 

int LoadDevicePrivateKey(void);
extern int verbose;
#endif // SEND_SMS_MAIN
#ifdef STANDALONE 

void Usage(char *pname) {
    fprintf(stderr,"Usage: %s Destination_Number Message\n", pname);
    exit(1);
}

int main(int argc, char **argv)
{
  #ifdef _USE_OPENSSL_
  LoadDevicePrivateKey();
  #endif
  if (argc<3) {
	Usage(argv[0]);
	exit(0);
  }

  char *msg;
  msg = SetupSMS(argv+2);
  if ( msg ) {
	SendSMS(argv[1],msg);
	free(msg);
  }
  return 0;
}
#endif
 

