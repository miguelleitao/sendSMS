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
int SendSingleSMS(int pd, char *num, const char *msg);
int SendSMS(char *num, const char *msg);
int SendBulkSMS(char num_tab[MAX_BULK_DESTINATIONS][MAX_DESTINATION_LEN], const char *msg);
int SendBulkListSMS(char *fname, const char *msg);
