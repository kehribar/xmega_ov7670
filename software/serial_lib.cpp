/*-------------------------------------------------------------------------------------------------
/
/
/
/
/------------------------------------------------------------------------------------------------*/
#include "serial_lib.h"
/*-----------------------------------------------------------------------------------------------*/
int serialport_init(const char* serialport, int baud,char parity)
{
    struct termios toptions;
    int fd;
    
    //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open(serialport, O_RDWR | O_NONBLOCK );
    
    if (fd == -1)  {    
        perror("Unable to open port ");
        return -1;
    }
    
    //int iflags = TIOCM_DTR;
    //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
    //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

    if (tcgetattr(fd, &toptions) < 0) {
        perror("Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
    switch(baud) {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
#ifdef B14400
    case 14400:  brate=B14400;  break;
#endif
    case 19200:  brate=B19200;  break;
#ifdef B28800
    case 28800:  brate=B28800;  break;
#endif
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    switch (parity) 
    {
        case 'n':
        case 'N':
            toptions.c_cflag &= ~PARENB; // Clear parity enable
            break;
        case 'o':
        case 'O':
            toptions.c_cflag |= PARENB; // Parity enable
            toptions.c_cflag |= PARODD; // Enable odd parity 
            break;
        case 'e':
        case 'E':
            toptions.c_cflag |= PARENB; // Parity enable
            toptions.c_cflag &= ~PARODD; // Turn off odd parity = even
            break;
        default:
            // printf("Unsupported parity\n");         
            return -1;
    }

    // 8N1
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
    //toptions.c_cc[VTIME] = 20;
    
    tcsetattr(fd, TCSANOW, &toptions);
    if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
        perror(">Couldn't set term attributes");
        return -1;
    }

    return fd;
}
/*-----------------------------------------------------------------------------------------------*/
int serialport_close( int fd )
{
    return close( fd );
}
/*-----------------------------------------------------------------------------------------------*/
int serialport_writebyte( int fd, uint8_t b)
{
    int n = write(fd,&b,1);
    if( n!=1)
        return -1;
    return 0;
}
/*-----------------------------------------------------------------------------------------------*/
int serialport_write(int fd, const char* str)
{
    int len = strlen(str);
    int n = write(fd, str, len);
    if( n!=len ) {
        // perror("serialport_write: couldn't write whole string\n");
        return -1;
    }
    return 0;
}
/*-----------------------------------------------------------------------------------------------*/
int serialport_read_until(int fd, char* buf, char until, int buf_max, int timeout)
{
    char b[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    do { 
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 1 * 1000 );  // wait 1 msec try again
            timeout--;
            continue;
        }

        buf[i] = b[0]; 
        i++;
    } while( b[0] != until && i < buf_max && timeout>0 );

    buf[i] = 0;  // null terminate the string
    return 0;
}
/*-----------------------------------------------------------------------------------------------*/
int serialport_flush(int fd)
{
    sleep(1); //required to make flush work, for some reason
    return tcflush(fd, TCIOFLUSH);
}
/*-----------------------------------------------------------------------------------------------*/
int readRawBytes(int fd,char* buffer,int desiredCount,int timeout)
{
    int n;
    int i=0;    
    char b[1];

    while((i<desiredCount)&&(timeout>0))
    {
        n = read(fd, b, 1);
        if(n==-1) 
        {   
            /* read problem */
            printf("read problem\n");
            return -1;
        }
        else if(n==0) 
        {
            /* wait a little ... */
            usleep(1000);
            timeout--;
            // printf("t");
        }
        else
        {       
         	// printf("%2X-",(uint8_t)b[0]);            
            buffer[i++] = b[0];
        }
    }
    if(!(timeout>0))
    {        
        return -2;
    }
    else
    {    
        return 0;
    }
}
/*-----------------------------------------------------------------------------------------------*/
