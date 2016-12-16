#ifndef XBEE_H
#define XBEE_H

#include <string.h>
#define XBEE_START_BYTE 0xFD

int XBee_send_float(int fd_xbee, float data[], int data_size)
{
    int i, bytes_sent;
    uint8_t send_buffer[sizeof(float)*data_size+3]; // 1 start byte + data + 2 checksum
    uint16_t checksum=0;

    // start byte
    send_buffer[0]=XBEE_START_BYTE;

    // float data
    for(i=0;i<data_size;i++)
    {
        memcpy(&send_buffer[sizeof(float)*i+1],&data[i],sizeof(float));
    }

    // checksum: sum of data without start byte (two bytes: rolls over at 65536)
    for(i=1;i<sizeof(float)*data_size+1;i++)
    {
        checksum+=send_buffer[i];
    }
    send_buffer[sizeof(float)*data_size+1]= checksum >> 8;  // the first 8 bits
    send_buffer[sizeof(float)*data_size+2]= checksum & 0xFF;// the last 8 bits

    // send data
    //tcflush(fd_xbee,TCIOFLUSH);
    bytes_sent = write(fd_xbee,send_buffer,sizeof(float)*data_size+3);

    return bytes_sent;
}

int XBee_receive_float(int fd_xbee, float data_received[], int data_size)
{
    struct timeval timeout;
    int bytes_received, attempts, i;
    int MAXATTEMPTS=100;
    int retval;
    uint8_t recv_byte;
    uint8_t recv_buffer[sizeof(float)*data_size+3]; // 1 start byte + data + 2 checksum
    fd_set readfs;
    uint16_t checksum_recv, checksum_calc=0;
    
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;


    FD_ZERO(&readfs);
    FD_SET(fd_xbee, &readfs);
    
    // wait for data
    // retval = select(fd_xbee+1, &readfs, NULL, NULL, NULL);
    retval = select(fd_xbee+1, &readfs, NULL, NULL, &timeout);

    //printf("Select Function Return Value: %d \n", retval);
    
    // check the start byte
    recv_byte=0;
    bytes_received=read(fd_xbee,&recv_byte,1);
        
    if(recv_byte==XBEE_START_BYTE)
    {
        // correct start byte

        bytes_received=0;
        attempts=0;

        recv_buffer[bytes_received]=recv_byte;
        bytes_received+=1;

        // receive data
        while(bytes_received < (sizeof(float)*data_size)+3 && attempts++ < MAXATTEMPTS)
        {
            if(read(fd_xbee,&recv_byte,1)==1)
            {
                recv_buffer[bytes_received]=recv_byte;
                bytes_received+=1;
            }
            else
            {
                usleep(1000);
            }

        }

        // Checksum: sum of data without start byte (two bytes: rolls over at 65536)
        for(i=1;i<sizeof(float)*data_size+1;i++)
        {
            checksum_calc+=recv_buffer[i];
        }
        checksum_recv=recv_buffer[sizeof(float)*data_size+1] << 8 | recv_buffer[sizeof(float)*data_size+2]; // high << 8 | low

        if(checksum_recv == checksum_calc)
        {
            // correct checksum
            // convert data to float
            for(i=0;i<data_size;i++)
            {
                data_received[i]=*(float *)&recv_buffer[sizeof(float)*i+1];
            }
            
         //   printf("Data received correctly\n");

            return bytes_received;
        }
        else
        {
            // incorrect checksum
//            printf("checksum_calc:%d, checksum_recv:%d\n",checksum_calc,checksum_recv);
            return -2;
        }

    }
    else
    {
        // incorrect start byte
        printf("incorrect start byte:%x, %x\n",recv_byte, XBEE_START_BYTE);
        return -1;
    }

    tcflush(fd_xbee,TCIOFLUSH);

}

int XBee_receive_float_altogether(int fd_xbee, float data_received[], int data_size)
{
    int bytes_received, i;
    uint8_t recv_buffer[sizeof(float)*data_size+1];
    fd_set readfs;
    uint16_t checksum_calc=0, checksum_recv;

    FD_ZERO(&readfs);
    FD_SET(fd_xbee, &readfs);

    select(fd_xbee+1, &readfs, NULL, NULL, NULL);

    bytes_received=read(fd_xbee,&recv_buffer,sizeof(float)*data_size+3);

    // Checksum: sum of data without start byte (two bytes: rolls over at 65536)
    for(i=1;i<sizeof(float)*data_size+1;i++)
    {
        checksum_calc+=recv_buffer[i];
    }
    checksum_recv=recv_buffer[sizeof(float)*data_size+1] << 8 | recv_buffer[sizeof(float)*data_size+2]; // high << 8 | low

    if(recv_buffer[0]==XBEE_START_BYTE && checksum_recv==checksum_calc && bytes_received==sizeof(float)*data_size+3)
    {
        for(i=0;i<data_size;i++)
        {
            data_received[i]=*(float *)&recv_buffer[sizeof(float)*i+1];
        }

        return bytes_received;

    }
    else
    {
        printf("start_byte=%x, checksum_recv=%d, checksum_calc=%d, bytes_receive=%d\n",recv_buffer[0],checksum_recv,checksum_calc,bytes_received);
        return -1;
    }

    tcflush(fd_xbee,TCIOFLUSH);

}

#endif // XBEE_H
