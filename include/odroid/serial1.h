#include <fcntl.h>
#include <termios.h>

#define BAUDRATE_serial1 B115200

int init_serial1()
{
    struct termios newtio; //!
    
	int port;
	
	port=open("/dev/ttyO0", O_RDWR | O_NOCTTY);
	
	tcgetattr(port, &newtio);
  // tcgetattr(port, &oldtio);
    bzero(&newtio, sizeof(newtio)); // clear structure for new port setting

    cfsetospeed(&newtio, BAUDRATE_serial1);
    cfsetispeed(&newtio, BAUDRATE_serial1);
	
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
	newtio.c_cc[VMIN]  = 0;   // block reading until RX x characers. If x = 0, it is non-blocking.
	newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s
	
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
