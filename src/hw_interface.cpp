#include <odroid/hw_interface.hpp>

hw_interface::hw_interface(){}
hw_interface::~hw_interface(){}
uint8_t* hw_interface::motor_command(int thr[4], bool MotorWarmup, bool MOTOR_ON){
  // Execute motor output commands
  int length = 6;
  char buffer[6];
  uint8_t  *msg = new uint8_t[24] ;
  for(int i=0;i<4;i++){
    tcflush(fhi2c, TCIOFLUSH);

    usleep(500);

    if(ioctl(fhi2c, I2C_SLAVE, mtr_addr[i])<0)
    printf("ERROR: ioctl\n");
    read(fhi2c,buffer,length);
    //printf("Motor:%d ",i);
    for(int k=0;k<length;k++){
      msg[i*6+k] = ((uint8_t)buffer[k]);
      //printf("%d, ", buffer[k]);
    }
    //printf("\n");
  }

  for(int i = 0; i < 4; i++){
    //printf("Motor %i I2C write command of %i to address %i (%e N).\n", i, thr[i], mtr_addr[i], f[i]);
    tcflush(fhi2c, TCIOFLUSH);
    usleep(500);
    if(ioctl(fhi2c, I2C_SLAVE, mtr_addr[i])<0)
    printf("ERROR: ioctl\n");
    if(MOTOR_ON == false)// set motor speed to zero
      thr[i] = 0;
    else if(MotorWarmup == true)// warm up motors at 20 throttle command
      thr[i] = 20;
    while(write(fhi2c, &thr[i], 1)!=1)
    printf("ERROR: Motor %i I2C write command of %i to address %i not sent.\n", i, thr[i], mtr_addr[i]);
  }
  return msg;
}

void hw_interface::open_I2C(){
  // Open i2c:
  fhi2c = open("/dev/i2c-1", O_RDWR);// Chris
  printf("Opening i2c port...\n");
  if(fhi2c!=3)
  printf("ERROR opening i2c port.\n");
  else
  printf("The i2c port is open.\n");
  usleep(100000);
  tcflush(fhi2c, TCIFLUSH);
  usleep(100000);

  // Call and response from motors
  printf("Checking motors...\n");
  int motornum, motoraddr, motorworks;
  int thr0 = 0;// 0 speed test command
  char PressEnter;

  for(motornum = 1; motornum <= 4; motornum++){
    motoraddr = motornum+40;// 1, 2, 3, ... -> 41, 42, 43, ...
    while(1){
      motorworks = 1;// will remain 1 if all works properly
      if(ioctl(fhi2c, I2C_SLAVE, motoraddr)<0){
        printf("ERROR: motor %i ioctl\n", motornum);
        motorworks = 0;// i2c address not called
      }
      usleep(10);
      if(write(fhi2c, &thr0, 1)!=1){
        printf("ERROR: motor %i write\n", motornum);
        motorworks = 0;
      }
      usleep(10);
      tcflush(fhi2c, TCIOFLUSH);
      if(motorworks == 1){
        printf("Motor %i working...\n", motornum);
        break;
      }
      else{
        printf("Fix motor %i, then press ENTER.\n\n", motornum);
        printf("Note: another i2c device may interupt the signal if\n");
        printf("any i2c wires are attached to unpowered components.\n");
        scanf("%c",&PressEnter);
        break;
      }
    }
  }
  printf("All motors are working.\n");
}
