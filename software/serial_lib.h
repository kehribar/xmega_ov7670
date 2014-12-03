/*-------------------------------------------------------------------------------------------------
/
/
/
/
/------------------------------------------------------------------------------------------------*/

#include <string.h>
#include <stdint.h>
#include <stdio.h>    
#include <unistd.h>   
#include <fcntl.h>    
#include <errno.h>    
#include <termios.h>  
#include <string.h>   
#include <sys/ioctl.h>

int serialport_init(const char* serialport, int baud,char parity);
int serialport_close(int fd);
int serialport_writebyte( int fd, uint8_t b);
int serialport_write(int fd, const char* str);
int serialport_read_until(int fd, char* buf, char until, int buf_max,int timeout);
int serialport_flush(int fd);
int readRawBytes(int fd,char* buffer,int desiredCount,int timeout);
