#include "imu.h"

#include "stm32f405xx.h"
void prvStartI2CTransaction(void);
void prvStopI2CTransaction(void);
void prvWriteI2C(uint8_t *buffer, unsigned int bufferLen);
void prvWriteI2Cbyte(uint8_t *buffer);
void prvReadI2Cbyte(uint8_t *buffer);
void prvReadI2C(uint8_t *buffer, unsigned int bufferLen);
void prvSelectI2CAddress(uint8_t address, uint8_t direction);
void prvI2CSend(uint8_t address, uint8_t *buffer, unsigned int bufferLen);
void prvI2CRead(uint8_t address, uint8_t *buffer, unsigned int bufferLen);
void prvI2CReadIMURegister(uint8_t address, uint8_t reg, uint8_t *buffer);


void initializeIMU(void){
    //Turn on I2C bus
    //enable power
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; //Use I2C1 on B6 and B7
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    //configure corrosponding gpio
    GPIOB->MODER |= GPIO_MODER_MODE6_1; //mode 0b10, alternate function mode
    GPIOB->AFR[0] |= GPIO_AFRL_AFRL6_2; //af 0b0100 (AF4) is I2C bus
    GPIOB->AFR[0] |= GPIO_AFRL_AFRL7_2; 
    //GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_0; //0b01 is pull-up //no pull up or down because they are external on the feather board (to help with I2C line rise time)
    //GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_0;
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED6; //0b11 is very high speed
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED7; //0b11 is very high speed

    //TODO: do we need open drain or push-pull? Internet suggests open-drain
    GPIOB->OTYPER |= GPIO_OTYPER_OT6; //0b1 is open-drain
    GPIOB->OTYPER |= GPIO_OTYPER_OT7; //0b1 is open-drain

    //configure I2C bus
        //configure I2C clock //400 kHz ideal for fast mode (fm)
            //configure peripheral bus clock //4 MHz minimum here (HSI will supply 16 MHz by default), tell CR2 that you are using 16 MHz by setting FREQ to 16
            //configure clock registers //downscale to 400 kHz here, select fm mode, also choose 9:16 duty cycle mode 
        //configure rise time register
        //Control Register 1 config and enable
        //set start bit to generate a start condition
    I2C1->CR2 &= ~I2C_CR2_FREQ;
    I2C1->CR2 |= (16U << I2C_CR2_FREQ_Pos);

    // I2C1->CCR |= I2C_CCR_FS; //fs mode
    // I2C1->CCR |= I2C_CCR_DUTY; //9:16 duty cycle mode
    // //Peripheral clock is 16 MHz, thus Tpclk1 = 1/16e6 = 62.5e-9 or 62.5 nanoseconds. in 16:9 mode there are 25 units in an I2C clock.
    // //The I2C clock needs to be 400 kHz = 2.5e-6 seconds or 2500 nanoseconds. That leaves 40 cycles between clocks. The ideal CCR option
    // //is therefore 2, putting the actual number of cycles between I2C clocks 50 cycles. At 50 cycles, the actual I2C clock is 320 kHz. If, 
    // //instead of 16:9, we use 1:2, that leaves 3 cycles per clock. With a CCR at 14, that leaves 42 cycles between I2C clocks, and an I2C
    // //clock rate of 381 MHz. I'll consider 320 close enough for now.
    // I2C1->CCR &= ~I2C_CCR_CCR;
    // I2C1->CCR |= 2U << I2C_CCR_CCR_Pos;

    // //the fm mode maximum rise time is 300 ns, so we will assume our bus capacitance is low enough that our 10k pullup resistors allow the
    // //rise time to be lower than the absolute maximum, and we will set this register to the absolute maximum rise time of 300 ns. Becasue
    // //the rise time is 300, the ideal TRISE value is 300 / 62.5 = 4.8 then round up and add one and you get TRISE = 6
    // I2C1->TRISE &= ~I2C_TRISE_TRISE;
    // I2C1->TRISE |= 6U << I2C_TRISE_TRISE_Pos;

    //HOWEVER! Considering that typical bus capacitances range from 100 pF to 400 pF depending on the load, and the external resistors are 10 kOhm,
    //the 100 pF rise time would be 1 us. 1 us is the specification for sm, not fm. Therefore configuration should be done in sm mode not fm mode.

    I2C1->CCR &= ~I2C_CCR_FS; //standard speed mode, max bus frequency 100 kHz
    I2C1->CCR &- ~I2C_CCR_DUTY; // 1:2 duty cycle
    //100 kHz -> 10 us, thus 10000 / 62.5 = 160 cycles, thus 160 / 3 = 53.33 is the ideal value for the CCR. Selecting 54 gives 10.125 us or 98.7 kHz
    I2C1->CCR &= ~I2C_CCR_CCR;
    I2C1->CCR |= 54U << I2C_CCR_CCR_Pos;

    //1000 ns max rise time from specification, thus 1000 / 62.5 + 1 = 16 + 1 = 17 is the value for the register
    I2C1->TRISE &= ~I2C_TRISE_TRISE;
    I2C1->TRISE |= 17U << I2C_TRISE_TRISE_Pos;


    I2C1->CR1 |= I2C_CR1_PE; //Enable the bus! You're good to start using it!


    //send initialization commands to the sensors
        //verify "WhoAmI"
    uint8_t value;
    prvI2CReadIMURegister(IMU_ADDRESS, 0x0F, &value);
    if(value == 0x6A){
        //Good!
    } else {
        //Bad :(
        while(1);
    }

    

}
void readAcc(float *xyz){
    //request reading by writing over I2C

    //read over I2C

    //decode data

}
void readGyro(float *xyz){
    //request reading by writing over I2C

    //read over I2C

    //decode data

}


void prvStartI2CTransaction(void){
    // while (I2C1->SR2 & I2C_SR2_BUSY);  // Wait until the bus is not busy
    I2C1->CR1 |= I2C_CR1_START; // Generate start condition
    while (!(I2C1->SR1 & I2C_SR1_SB)); // Wait for start condition to be sent
}

void prvStopI2CTransaction(void){
    I2C1->CR1 |= I2C_CR1_STOP; // Generate stop condition
}



void prvWriteI2C(uint8_t *buffer, unsigned int bufferLen){
    for(unsigned int i = 0; i < bufferLen; i++){
        prvWriteI2Cbyte((buffer + i)); //big endian or little endian?
    }
}
void prvWriteI2Cbyte(uint8_t *buffer){
    //TXNE?
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = *buffer;
    while (!(I2C1->SR1 & I2C_SR1_BTF));  // Wait until byte transfer finished

    //don't forget to set any flags or clear any other flags

}
void prvReadI2Cbyte(uint8_t *buffer){
    //RXE?
    //how long to wait if empty?
    while (!(I2C1->SR1 & I2C_SR1_RXNE)); // Wait for the receive buffer not empty
    *buffer = (uint8_t)(I2C1->DR);
    
    //maunaully clear flags?

}
void prvReadI2C(uint8_t *buffer, unsigned int bufferLen){
    for(unsigned int i = 0; i < bufferLen; i++){
        prvReadI2Cbyte((buffer + i)); //big endian or little endian?
        if( i < (bufferLen-1) ){
            //not the last byte, ACK all
            I2C1->CR1 |= I2C_CR1_ACK;
        } else {
            //this is the last byte, NACK
            I2C1->CR1 &= ~I2C_CR1_ACK;
        }
    }
}


void prvSelectI2CAddress(uint8_t address, uint8_t direction) {
    uint16_t addr = (address << 1) | direction;
    I2C1->DR = addr; // Send the address
    if (direction == I2C_DIRECTION_TRANSMIT) {
        while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Wait for address to be sent in transmitter mode
        (void)I2C1->SR2;
    } else {
        while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Wait for address to be sent in receiver mode
        I2C1->CR1 |= I2C_CR1_ACK; // Acknowledge the byte
        (void)I2C1->SR2;
    }
}

void prvI2CSend(uint8_t address, uint8_t *buffer, unsigned int bufferLen){
    prvStartI2CTransaction();
    prvSelectI2CAddress(address, I2C_DIRECTION_TRANSMIT);
    prvWriteI2C(buffer, bufferLen);
    prvStopI2CTransaction();
}
void prvI2CRecieve(uint8_t address, uint8_t *buffer, unsigned int bufferLen){
    prvStartI2CTransaction();
    prvSelectI2CAddress(address, I2C_DIRECTION_RECIEVE);
    prvReadI2C(buffer, bufferLen);
    prvStopI2CTransaction();
}
void prvI2CReadIMURegister(uint8_t address, uint8_t reg, uint8_t *buffer){
    prvStartI2CTransaction();
    prvSelectI2CAddress(address, I2C_DIRECTION_TRANSMIT);
    prvWriteI2C(reg, 1);
    //no stop bit
    prvI2CRecieve(address, *buffer, 1); //repeats the start bit as per specification
}
