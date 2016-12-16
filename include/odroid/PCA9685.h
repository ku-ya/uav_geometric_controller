/*
 * PCA9685.h
 * This Arduino header/C++ header code for servo control
 * (https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)
 * by Limor Fried/Ladyada is adapted for Gumstix Overo COM by
 * Evan Kaufman (tested on AirStorm).
 * This header is used to run servos on a PCA9685 Adafruit 12-bit
 * PWM/Servo Driver (http://www.adafruit.com/products/815)
 * with datasheet (http://www.adafruit.com/datasheets/PCA9685.pdf).
*/

void write8(int fh, uint8_t d0, uint8_t d1) {
    uint8_t d[2];
    d[0] = (uint8_t)d0;
    d[1] = (uint8_t)d1;
    if(ioctl(fh, I2C_SLAVE, i2c_addr)<0)
        printf("ERROR in write8: ioctl\n");
    if(write(fh, d, 2)!=2)
        printf("ERROR in write8: write\n");
    usleep(10);// 10 us pause before any other i2c commands
}

uint8_t read8(int fh, uint8_t addr) {

    if(ioctl(fh, I2C_SLAVE, i2c_addr)<0)
        printf("ERROR in read8: ioctl before write()\n");
    write(fh, &addr, 1);
    usleep(50);

    uint8_t data;
    if(ioctl(fh, I2C_SLAVE, i2c_addr)<0)
        printf("ERROR in read8: ioctl before read()\n");
    return read(fh, &data, 1);
}

void pwmPulse(int fh, uint8_t num, uint16_t on, uint16_t off) {

    uint8_t pulseData[5];
    pulseData[0] = LED0_ON_L+4*num;
    pulseData[1] = on;
    pulseData[2] = on>>8;
    pulseData[3] = off;
    pulseData[4] = off>>8;

    if(ioctl(fh, I2C_SLAVE, i2c_addr)<0)
        printf("ERROR in pwmPulse: ioctl\n");

    if(write(fh, pulseData, 5)!=5)
        printf("ERROR in pwmPulse: write\n");
}

int pwmFreq(int fh, float freq) {

    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    uint8_t prescale = floor(prescaleval + 0.5);
    float div = prescale*freq*4096/25000000;
    float ActualFreq = freq/div*1.0125;// tweeking from testing
    printf("Frequency was set to %f.\n", ActualFreq);

    uint8_t oldmode = read8(fh, PCA9685_MODE1);

    uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep

    write8(fh, PCA9685_MODE1, newmode); // go to sleep

    write8(fh, PCA9685_PRESCALE, prescale); // set the prescaler

    write8(fh, PCA9685_MODE1, oldmode);

    write8(fh, PCA9685_MODE1, oldmode | 0xa1);// MODE1 to auto increment

    return 1;
}

int resetDev(fh) {
    if(ioctl(fh, I2C_SLAVE, PCA9685_MODE1)<0)
        printf("ERROR in reset: ioctl\n");
    int reset = 0x06;
    if(write(fh, &reset, 1)!=1)
        printf("ERROR in reset: write\n");

    write8(fh, PCA9685_MODE1, 0x0);
    return 1;
}
