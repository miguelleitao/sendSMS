/*
 *  sendSMS.h
 *
 *  Send Short Message through USB connected GSM adapter
 *
 *  Miguel Leitao, 2015
 */
 
#define MAX_BULK_DESTINATIONS 65
#define MAX_DESTINATION_LEN 22

int setupModem();
int sendSingleSMS(int pd, char *num, char *msg);
int SendSMS(char *num, char *msg);
int SendBulkSMS(char num_tab[MAX_BULK_DESTINATIONS][MAX_DESTINATION_LEN], char *msg);
int SendBulkListSMS(char *fname, char *msg);
