#include <fcntl.h>
#include <termios.h>
#include <string.h>

//#define BAUDRATE_serial2 B9600
#define BAUDRATE_serial2 B57600
//#define BAUDRATE_serial2 B38400
//#define BAUDRATE_serial2 B115200

int init_serial2()
{
    struct termios newtio; //!
    
	int port;
	
    port=open("/dev/ttySAC0", O_RDWR | O_NOCTTY);

	/*	printf("Configuring a serial port...   ");
	 tcgetattr(fhserial,&oldtio);
	 bzero(&newtio, sizeof(newtio));
     newtio.c_cflag = BAUDRATE_serial2 | CS8 | CLOCAL | CREAD;
	 newtio.c_iflag = IGNPAR;
	 newtio.c_oflag = 0;
	 newtio.c_lflag = 0;
	 newtio.c_cc[VTIME]    = 0;
	 newtio.c_cc[VMIN]     = 67;
	 tcflush(fhserial, TCIFLUSH);
	 tcsetattr(fhserial,TCSANOW,&newtio);
	 printf("Done!\n");*/
	
	tcgetattr(port, &newtio);
  // tcgetattr(port, &oldtio);
    bzero(&newtio, sizeof(newtio)); // clear structure for new port setting

    cfsetospeed(&newtio, BAUDRATE_serial2);
    cfsetispeed(&newtio, BAUDRATE_serial2);
	
	//set the number of data bits.
	newtio.c_cflag &= ~CSIZE;  // Mask the character size bits
	newtio.c_cflag |= CS8;
	
	//set the number of stop bits to 1
	newtio.c_cflag &= ~CSTOPB;
	
	//Set parity to None
	newtio.c_cflag &=~PARENB;
	
	//set for non-canonical (raw processing, no echo, etc.)
	newtio.c_iflag = IGNPAR; // ignore parity check close_port(int
	newtio.c_oflag = 0; // raw output
	newtio.c_lflag = 0; // raw input
	
	   //Time-Outs -- won't work with NDELAY option in the call to open
	newtio.c_cc[VMIN]  = 24;   // block reading until RX x 
//characers. 
//If x = 0, it is non-blocking.
	//newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. 
//timeout= x*.1 s
	
	   //Set local mode and enable the receiver
	newtio.c_cflag |= (CLOCAL | CREAD);
	
	//tcflush(port, TCIOFLUSH);
	
	//Set the new options for the port...
	int status=tcsetattr(port, TCSANOW, &newtio);
	
	if (status != 0){ //For error message
		printf("Configuring comport failed\n");
		return status;
	}
	
	//tcflush(port, TCIFLUSH);

	return port;
}
