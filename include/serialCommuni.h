#ifndef _SERIALCOMMUNI_HEADER
#define _SERIALCOMMUNI_HEADER

#ifdef __cplusplus
extern "C" {
#endif

int serial_open(int deviceIndex, float baudrate);
int receiveMessage(unsigned char *pPacket, int numPacket);
int sendMessage(unsigned char *pPacket, int numPacket);
int serial_close();

#ifdef __cplusplus
}
#endif

#endif
