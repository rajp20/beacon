/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#define RED 6
#define GREEN 9
#define ORANGE 8
#define BLUE 7


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void sendString(char* output);
void sendChar(char output);
void LED_On(int);
void LED_Off(int);
void LED_Toggle(int);
void I2C_Gyro_Read(void);
uint8_t readLoRaData(void);
uint8_t readSPIData(void);
void readFromReg(uint8_t reg);
void writeToReg(uint8_t reg, uint8_t data);
void resetLoRa(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
int inputFlag;

// Used to store the GPS data that is read in from the module. The messages will never exceed 256 bytes
// Current index will be used to insert into the appropiated position into the array
char GPS_buff[256];
char GPS_protocol[8];
int current_index;

char SPI_data;

/*
* Returns the size of the string that is sent in as a parameter
*/
uint8_t length_str(char *s){
	uint8_t i = 1;
	while (s[i] != '\0'){i++;}
	return i;
}

/*
 * Toggle the given LED.
 */
void LED_Toggle(int LED) {
	GPIOC->ODR ^= (1 << LED);
}

/*
*	Send the LoRa chip a 16-bit value
*/
void send_AD_HOC(uint16_t data){
	while (!(((SPI1->SR & SPI_SR_TXE) == SPI_SR_TXE))) /* Test Tx empty */ { }
	*(uint16_t *)&(SPI1->DR) = data;
}


/*
 * Turn off the given LED.
 */
void LED_Off(int LED){
  GPIOC->ODR &= ~(1 << LED);
}

/*
 * Turn on the given LED.
 */
void LED_On(int LED){
  GPIOC->ODR |= (1 << LED);
}

/*
*	Enter recieve mode to start receiving data and writing to the buffer
*/
void enterReceiveMode(){
	writeToReg(0x01, 133);
}

void resetLoRa() {
	
	GPIOB->ODR ^= (1 << 6);
	HAL_Delay(100);
	GPIOB->ODR ^= (1 << 6);
}

/*
*	Turn the LoRa chip into LoRa mode.
*/
void initializeLoRa(){
	uint16_t opReg = 0;
	opReg |= (1 << 8);

	// Set write mode to register 0x01 and write it into sleep mode
	opReg |= (1 << 15);
	opReg |= 0x08;

	// Send the LoRa chip this data
	send_AD_HOC(opReg);

	// Read the first register from the LoRa chip to make sure it was enabled to sleep mode
	opReg = 0;
	opReg |= (0x01 << 8);
	send_AD_HOC(opReg);

	// Set write mode to the register address 0x01
	opReg = 0;
	opReg |= (1 << 15);
	opReg |= (1 << 8);

	// Enable LoRa and keep the LoRa registers enabled
	opReg |= 136;

	send_AD_HOC(opReg);

	// Read from the register 0x01 to make sure LoRa mode was turned on
	opReg = 0;
	opReg |= (0x01 << 8);
	send_AD_HOC(opReg);

	// Turn it back to standby mode
	writeToReg(0x01, 137);
}

/*
 * USART handler.
 */
void USART3_4_IRQHandler() {
	char inputChar = USART3->RDR;
	sendChar(inputChar);
	inputFlag = 1;
}

void EXTI4_15_IRQHandler() {
	LED_On(RED);
}

/*
 * TMR Handler. Parses and sends the data using the radio chip.
 */
void TIM2_IRQHandler (void) {
  TIM2->SR = 0;
}

/*
*	Used for sending data to the LoRa to send out
*/
void transmitLoRaData(char *data){

	// Length of the string to transmit
	uint8_t length = length_str(data);
	uint8_t i = 0;

	// Send all of the bytes of the data that needs to be send
	while (i < length){

		// Set the RegFifoAddrPtr (0x0D) to RegFifoTxCurrentAddr (0x0E)
		readFromReg(0x0E);
		uint8_t address = readSPIData();
		sendString("Addr: ");
		sendChar(address + 48);
		writeToReg(0x0D, address);

		// Write the string into the FIFO data buffer
		writeToReg(0x0, data[i]);

		// Enter the transmit state
		writeToReg(0x01, 131);

		// WAIT FOR TX TO FINISH (3rd bit)
		uint8_t TX_DONE_FLAG = (1 << 3);
		readFromReg(0x12);
		char spi = readSPIData();
		sendString("Data: ");
		sendChar(spi + 48);
		while (!(spi & TX_DONE_FLAG)) {
			readFromReg(0x12);
			spi = readSPIData();
			sendString("Data: ");
			sendChar(spi + 48);
		}

		// Reset the flags
		writeToReg(0x12, TX_DONE_FLAG);

		
		sendString("Sent one char!");
		sendChar(data[i]);
		i++;
	}


}

/*
*	Used for reading data from the LoRa chip. Returns 0 if the data was read correctly, -1 if data was unable to be read
*/
uint8_t readLoRaData(){
	uint8_t returnData;

		// Ensure that ValidHeader, PayloadCrcError, RxDone and RxTimeout interrupts in the status register RegIrqFlags are not asserted (otherwise ignore the data)
	readFromReg(0x1D);
	returnData = readSPIData();
	
	sendChar(returnData + 48);

	// Ensure that ValidHeader, PayloadCrcError, RxDone and RxTimeout interrupts in the status register RegIrqFlags are not asserted (otherwise ignore the data)
	readFromReg(0x12);
	returnData = readSPIData();

	while (!(returnData & (1 << 6))) {	
		// Ensure that ValidHeader, PayloadCrcError, RxDone and RxTimeout interrupts in the status register RegIrqFlags are not asserted (otherwise ignore the data)
		readFromReg(0x12);
		returnData = readSPIData();
	}
	
	if ((returnData & (1 << 7)) | (returnData & (1 << 5)) | (returnData & (1 << 4))){

		// Reset all of the RegIrqFlags
		writeToReg(0x12, 255);
		return 0;
	}

	// Read from RegRxNbBytes (0x13) reg (num of bytes to read)
	readFromReg(0x13);
	uint8_t bytesToRead = readSPIData();

	// Set the RegFifoAddrPtr (0x0D) to RegFifoRxCurrentAddr (0x10)
	readFromReg(0x10);
	uint8_t address = readSPIData();
	writeToReg(0x0D, address);

	// Used to store the data read
	char data[bytesToRead];
	uint8_t i = 0;

	// Reading the register RegFifo (0x0), RegRxNbBytes times
	while (bytesToRead != 0){
		readFromReg(0x0);
		data[i++] = readSPIData();
		bytesToRead--;
	}

	sendString(data);
	return 0;
}

/*
*	Read specific the data stored in the register that is sent as a parameter
*/
void readFromReg(uint8_t reg){
	uint16_t dataRequest = 0;
	dataRequest |= (reg << 8);
	send_AD_HOC(dataRequest);
}

/*
*	Write to the register that is given as input, and write to this register the data that is sent as a parameter
*/
void writeToReg(uint8_t reg, uint8_t data){
	uint16_t dataRequest = 0;

	// Set the write bit
	dataRequest |= (1 << 15);

	//Shift the data into the request and the register and then send it
	dataRequest |= (reg << 8) | (data);
	send_AD_HOC(dataRequest);
}

/*
 * Handle GPS data
 */
int valid_GPS_data = 0;
void USART1_IRQHandler() {
	//turnOn(ORANGE);
	char input = USART1->RDR;

	if (current_index <= 8) {
		GPS_protocol[current_index] = input;
	}
	current_index++;

	if (current_index == 8) {
		if (GPS_protocol[3] == 'G' && GPS_protocol[4] == 'G' && GPS_protocol[5] == 'A') {
			valid_GPS_data = 1;
		} else {
			valid_GPS_data = 0;
		}
	}

	if (valid_GPS_data == 1) {
		sendChar(input);
		GPS_buff[current_index - 8] = input;
	}

	// If a new line character is reached, then we know that all the data has been read by the module
	if (input == '\n'){
		GPS_buff[current_index] = '\0';
		current_index = 0;
		valid_GPS_data = 0;
	}
}

/* USER CODE BEGIN 0 */

/*
 * Print the char to USART.
 */
void sendChar(char output) {
	while(!(USART3->ISR & USART_ISR_TXE)) {	}
	USART3->TDR = output;
}

/*
 * Print the given string to USART.
 */
void sendString(char* output) {
	int i = 0;
	while (output[i] != '\0') {
		sendChar(output[i]);
		i++;
	}
}

/*
 * Compare two strings. Return true/1 if the strings match, return false/0 otherwise.
 */
int compare(char* left, char* right, int length) {
	int i;
	for (i = 0; i < length; i++) {
		if (left[i] != right[i]) {
			return 0;
		}
	}
	return 1;
}

uint8_t readSPIData(){
	// Wait until the buffer is not empty
	while (!((SPI1->SR & SPI_SR_RXNE) == SPI_SR_RXNE)) {}

	return (uint8_t)SPI1->DR; /* receive data, clear flag */
}

/*
 * Initialize I2C.
 */
void I2C_Init() {

	// Turn PB11 and PB13  to AFM, and PB14 to GPOM
	GPIOB->MODER |= (1 << 23) | (1 << 27) | (1 << 28);

	// Turn on PC0 to GPOM
	GPIOC->MODER |= 1;
	GPIOB->OTYPER |= (1 << 13) | (1 << 11);

	// Turn on PB11 to I2C2_SDA and PB13 to I2C2_SCL
	GPIOB->AFR[1] |= (1 << 12) | (1 << 20) | (1 << 22);
	GPIOB->ODR |= (1 << 14);
	GPIOC->ODR |= 1;

	// Set up standard I2C mode
	I2C2->TIMINGR |= (1 << 28) | (0x13) | (0xF << 8) | (0x2 << 16) | (0x4 << 20);

	// Enable the I2C peripheral using the PE bit in CR1 register
  I2C2->CR1 |= 1;

  // Set the address to the correct peripheral.
  I2C2->CR2 = (0x6B << 1);

  // Set number of bytes to send to 1
  I2C2->CR2 |= (1 << 16);

  // Set the start bit
  I2C2->CR2 |= (1 << 13);

  while (!((I2C_ISR_NACKF & I2C2->ISR) | (I2C_ISR_TXIS & I2C2->ISR))){ }

  if ((I2C_ISR_NACKF & I2C2->ISR)){
    // BAD
  }
  if ((I2C_ISR_TXIS & I2C2->ISR)){
    // GOOD
  }


  // Send the correct WHO_AM_I information
  I2C2->TXDR = 0x0F;

  // Wait for TC flag to set
  while (!(I2C_ISR_TC & I2C2->ISR)){ }

  // Set the address to the correct peripheral.
  I2C2->CR2 = (0x6B << 1);

  // Set number of bytes to read to 1
  I2C2->CR2 |= (1 << 16);

  // Enable the read operation
  I2C2->CR2 |= (1 << 10);

  // Set the start bit
  I2C2->CR2 |= (1 << 13);

  while (!((I2C_ISR_NACKF & I2C2->ISR) | (I2C_ISR_RXNE & I2C2->ISR))){ }

  uint8_t WHO_AM_I = I2C2->RXDR;

  // Wait for TC flag to set
  while (!(I2C_ISR_TC & I2C2->ISR)){ }

  // Set the address to the correct peripheral.
  I2C2->CR2 = (0x6B << 1);

  // Set number of bytes to send to 2
  I2C2->CR2 |= (1 << 17);

  // Enable the write operation
  I2C2->CR2 &= ~(1 << 10);

  // Set the start bit
  I2C2->CR2 |= (1 << 13);

  while (!((I2C_ISR_NACKF & I2C2->ISR) | (I2C_ISR_TXIS & I2C2->ISR))){ }

  // Set the address to CTRL_REG1
  I2C2->TXDR = 0x20;

  // Wait for TXIS flag to set
  while (!(I2C_ISR_TXIS & I2C2->ISR)){ }

  // Set the XEN, YEN, and PEN
  I2C2->TXDR = 0x0B;

  // Wait for TC flag to set
  while (!(I2C_ISR_TC & I2C2->ISR)){ }
}

/*
 * Initialize the USART
 */
void USART_Init() {
    // Set GPIO pins PC4 and PC5 to AFM.
    GPIOC->MODER |= (1 << 9) | (1 << 11);
    //GPIOB->MODER &= ~((1 << 20) | (1 << 22));

	// Set AFM for PC4 & 5.
	GPIOC->AFR[0] |= (1 << 20) | (1 << 16);

	// Set the baud rate of USART3
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;

	// Enable RX and TX in USART3
	USART3->CR1 |= (1 << 2) | (1 << 3) | (1 << 5);

	// Enable USART3
	USART3->CR1 |= (1 << 0);

	NVIC_EnableIRQ(USART3_4_IRQn);
}

/*
 * Initalize LEDs
 */
void LED_Init() {
	//  LED CONFIGURATION
	GPIOC->MODER 		|= (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18);
	GPIOC->MODER 		&= ~((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));
	GPIOC->OTYPER 	&= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
	GPIOC->OSPEEDR 	&= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18));
	GPIOC->PUPDR		&= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15) |
											 (1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));
}

/*
 * Initialize SPI
 */
void SPI_Init() {
	// SET UP PINS

	// Setting pins PB3 to PB5 (SPI1_SCK, SPI1_MISO, & SPI1_MOSI respectively) on Alternate function Mode
	GPIOB->MODER |= (1 << 7) | (1 << 9) | (1 << 11);
	
	// Set pin PB6 (reset) to GPOM
	GPIOB->MODER |= (1 << 12);
	
	// Set reset pin low;
	GPIOB->ODR &= ~(1 << 6);

  // Set pin PA15 to alternate function mode for SPI1_NSS
  GPIOA->MODER |= (0x2u << 30);

	/********* Write to SP1_CR1 register *********/


	// a) Set Baud Rate to fPCLK/256
	SPI1->CR1 |= (0x7 << 3);

	// b) CPOL 0 and CPHA 0, first rising edge
	SPI1->CR1 &= ~((1 << 1) | (1 << 0));

	// c) Set up SPI for Full-Duplex mode (bit 10 = 0)
		SPI1->CR1 &= ~(1 << 10);

	// g) Set Master Config.
	SPI1->CR1 |= (1 << 2);

	/********* Write to SP1_CR2 register *********/

	// a) Set DS[3:0] bits to 16-bit data length transfer
	SPI1->CR2 |= (0xF << 8);

	// b) SSOE enabled
	SPI1->CR2 |= (1 << 2);

	// c) Enable NSSP bit for pulse generation
	SPI1->CR2 |= (1 << 3);

	// Enable the interrupts for getting an interrupt of data received
	SPI2->CR2 |= (1 << 6);

	// e) FRXTH (RXNE event is generated if the FIFO level is >= 1/2 ... 16-bit)
	SPI1->CR2 &= ~(1 << 12);

	// SPI Enabled
	SPI1->CR1 |= (1 << 6);

	/*			SPI EXTERNAL INTERRUPT SETUP 			*/

	GPIOB->PUPDR |= (1 << 25);

	// Enable EXTI0 to rising edge interrupt.
	EXTI->IMR |= (1 << 12);
	EXTI->RTSR |= (1 << 12);

	// Set the SYSCNFG to EXTI4, PB 12
	SYSCFG->EXTICR[3] |= (1 << 0);

	// Enable the EXIT0 interrupt line
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	// Set the priority to high for this interrupt.
	NVIC_SetPriority(EXTI4_15_IRQn, 3);
}


/*
*	Used for setting up the UART that is used by the GPS module
*/
void UART_GPS_Init(){
	// Set PA9 and PA10 to AFM (5V tolerant), use AF1 for USART1
	GPIOA->MODER |= (1 << 19) | (1 << 21);
	GPIOA->AFR[1] |= (1 << 4) | (1 << 8);

	// Set BAUD rate to 9600 bits/sec
	USART1->BRR = HAL_RCC_GetHCLKFreq() / 9600;

	// Enable RX and TX in USART1 and enable the interrupt flag to start
	USART1->CR1 |= (1 << 2) | (1 << 3) | (1 << 5);

	// Now turn on the USART
	USART1->CR1 |= (1 << 0);

	// Enable the interrupt for this USART
	NVIC_EnableIRQ(USART1_IRQn);

	// Set the current index to 0, since we are reading in fresh data
	current_index = 0;
}

/*
 * Initialize the timer. This will trigger data handeling for the GPS and send it to the
 * other STM board.
 */
void TMR_Init() {
	  // Divide the clock by 8000, and then count to 125 so the clock operates at 4 Hz
  TIM2->PSC = 19999;
  TIM2->ARR = 400;

	// Enable the UEV events for this timer
  TIM2->DIER = 1;

	// Configure and enable/start the timer
  TIM2->CR1 |= TIM_CR1_CEN;

	// Enable the interrupt
  NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	HAL_Init();
  SystemClock_Config();

	// Enable the RCC for USART
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Intialize all of the communication protocols and peripherals 
	LED_Init();
	USART_Init();
	//UART_GPS_Init();
	//I2C_Init();
	SPI_Init();
	// TMR_Init();
	initializeLoRa();

	resetLoRa();
	readFromReg(0x01);
	uint8_t address = readSPIData();
	sendChar(address);

	
	enterReceiveMode();


	
	while(1) {
		//readLoRaData();
		transmitLoRaData("Hello");
		//I2C_Gyro_Read();
	}
}

/*
 * Read Gyro data via I2C.
 */
void I2C_Gyro_Read(){

	  int16_t X_axis;
		int16_t Y_axis;

		HAL_Delay(100);

    // Set the address to the correct peripheral.
    I2C2->CR2 = (0x6B << 1);

    // Set number of bytes to send to 1
    I2C2->CR2 |= (1 << 16);

    // Enable the write operation
    I2C2->CR2 &= ~(1 << 10);

    // Set the start bit
    I2C2->CR2 |= (1 << 13);

    while (!((I2C_ISR_NACKF & I2C2->ISR) | (I2C_ISR_TXIS & I2C2->ISR))){ }

    if ((I2C_ISR_NACKF & I2C2->ISR)){
      // BAD
    }
    if ((I2C_ISR_TXIS & I2C2->ISR)){
      // GOOD
    }

    // Set the address to CTRL_REG1
    I2C2->TXDR = 0xA8;

    // Wait for TC flag to set
    while (!(I2C_ISR_TC & I2C2->ISR)){ }

    // Set the address to the correct peripheral.
    I2C2->CR2 = (0x6B << 1);

    // Set number of bytes to read to 2
    I2C2->CR2 |= (2 << 16);

    // Enable the read operation
    I2C2->CR2 |= (1 << 10);

    // Set the start bit
    I2C2->CR2 |= (1 << 13);

    while (!((I2C_ISR_NACKF & I2C2->ISR) | (I2C_ISR_RXNE & I2C2->ISR))){ }

    if ((I2C_ISR_NACKF & I2C2->ISR)){
      // BAD
    }
    if ((I2C_ISR_RXNE & I2C2->ISR)){
      // GOOD
    }

    X_axis = I2C2->RXDR;

    while (!(I2C_ISR_RXNE & I2C2->ISR)){ }

    X_axis |= I2C2->RXDR << 8;

      // Wait for TC flag to set
    while (!(I2C_ISR_TC & I2C2->ISR)){ }

    // Set the address to the correct peripheral.
    I2C2->CR2 = (0x6B << 1);

    // Set number of bytes to send to 1
    I2C2->CR2 |= (1 << 16);

    // Enable the write operation
    I2C2->CR2 &= ~(1 << 10);

    // Set the start bit
    I2C2->CR2 |= (1 << 13);

    while (!((I2C_ISR_NACKF & I2C2->ISR) | (I2C_ISR_TXIS & I2C2->ISR))){ }

    if ((I2C_ISR_NACKF & I2C2->ISR)){
      // BAD
    }
    if ((I2C_ISR_TXIS & I2C2->ISR)){
      // GOOD
    }

    // Set the address to CTRL_REG1
    I2C2->TXDR = 0xAA;

    // Wait for TC flag to set
    while (!(I2C_ISR_TC & I2C2->ISR)){ }

    // Set the address to the correct peripheral.
    I2C2->CR2 = (0x6B << 1);

    // Set number of bytes to read to 2
    I2C2->CR2 |= (2 << 16);

    // Enable the read operation
    I2C2->CR2 |= (1 << 10);

    // Set the start bit
    I2C2->CR2 |= (1 << 13);

    while (!((I2C_ISR_NACKF & I2C2->ISR) | (I2C_ISR_RXNE & I2C2->ISR))){ }

    if ((I2C_ISR_NACKF & I2C2->ISR)){
      // BAD
    }
    if ((I2C_ISR_RXNE & I2C2->ISR)){
      // GOOD
    }

    Y_axis = I2C2->RXDR;

    while (!(I2C_ISR_RXNE & I2C2->ISR)){ }

    Y_axis |= I2C2->RXDR << 8;

      // Wait for TC flag to set
    while (!(I2C_ISR_TC & I2C2->ISR)){ }

		if (X_axis > 5000) {
			//sendString("RIGHT");
			GPIOC->ODR |= (1 << GREEN);
			GPIOC->ODR &= ~(1 << ORANGE);
		} else if (X_axis < -5000) {
			//sendString("LEFT");
			GPIOC->ODR |= (1 << ORANGE);
			GPIOC->ODR &= ~(1 << GREEN);
		}

		if (Y_axis > 5000) {
			//sendString("TOP");
			GPIOC->ODR |= (1 << RED);
			GPIOC->ODR &= ~(1 << BLUE);
		} else if (Y_axis < -5000) {
			//sendString("BOTTOM");
			GPIOC->ODR |= (1 << BLUE);
			GPIOC->ODR &= ~(1 << RED);
		}
}

/* USER CODE END 0 */



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
