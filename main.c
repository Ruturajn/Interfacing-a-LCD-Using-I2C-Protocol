/******************************************************************************
 * This is the code for interfacing a 16x2 LCD with a I2C module using the
 * I2C protocol, and printing "HELLO" on the LCD Screen.
 *
 * Configurations are as follows:
 * PB6 - I2C1_SCL
 * PB7 - I2C1_SDA
 * (See Page 58 of the 203 Page user stm32f407vg user manual)
 *
 * For the I2C Module (PC58574T) to LCD the connections are as follows:
 * P0 - RS
 * P1 - R/W
 * P2 - E
 * P3 - Back Light
 * P4 - D4
 * P5 - D5
 * P6 - D6
 * P7 - D7
 * So, the data byte that we send through the I2C module will be mapped to the
 * LCD based on the above configuration. This will be helpful for understanding
 * how the data and commands are sent to the LCD in the LCD_Write_Data and
 * LCD_Write_Cmd function.
 *****************************************************************************/

//Header Files
#include <stm32f4xx.h>
#define ARM_MATH_CM4

//Register Definitions for 1602A I2C LCD
#define LCD_ADDR (0x4E)
#define INST_REG (0x00)
#define DATA_REG (0x40)

// Variable for clearing ADDR bit
uint8_t clc;

//User-defined Function Declarations
void GPIO_Init(void);
void I2C_Init(void);
void I2C_Start(void);
void I2C_Write(uint8_t var);
void I2C_Send_Addr(uint8_t Addr);
void I2C_Stop(void);
void LCD_Init(void);
void LCD_Write_Cmd(uint8_t Device_Addr, uint8_t Slave_Reg_Addr, uint8_t data);
void LCD_Write_Data(uint8_t Device_Addr, uint8_t Slave_Reg_Addr, uint8_t data);
void LCD_Cursor(int r, int c);
void TIM4_ms_Delay(uint16_t delay);


//Definitions of the User-defined Functions
void GPIO_Init(){
	//Enable GPIOB clock
	RCC->AHB1ENR |= 1UL<<1;

	// Configuring PB6 and PB7 in Alternate function
	GPIOB->MODER |= ( (2UL<<(6*2)) | (2UL<<(7*2)) );

	// Selecting PB6 and PB7 as Pull up pins
	GPIOB->PUPDR |= ( (1UL<<(6*2)) | (1UL<<(7*2)));

	// Setting PB6 and PB7 as open drain
	GPIOB->OTYPER |= ( (1UL<<6) | (1UL<<7) );

	// Setting PB6 and PB7 at high speed
	GPIOB->OSPEEDR |= ( (2UL<<(6*2)) | (2UL<<(7*2)) );

	// Selecting the Alternate function (AF4)
	GPIOB->AFR[0] |= ( (4UL<<(6*4)) | (4UL)<<(7*4));
}

void I2C_Init(){
	// Enable I2C1 clock
	RCC->APB1ENR |= 1UL<<21;

	// Reset I2C
	I2C1->CR1 |= 1UL<<15;
	I2C1->CR1 &= ~(1UL<<15);

	// Set I2C clock at 16MHz
	I2C1->CR2 |= 16UL<<0;

	// Needs to be set high by software for I2C
	I2C1->OAR1 |= 1UL<<14;

	// Set SCl at 100KHz
	I2C1->CCR |= 0x50UL<<0;

	// Configure rise time as 1000ns
	I2C1->TRISE |= 17UL<<0;

	// Enable I2C1
	I2C1->CR1 |= 1UL<<0;
}

void I2C_Start(){
	// Enable the acknowledgment ACK bit
	I2C1->CR1 |= 1UL<<10;

	// Set the START bit to Start Communication
	I2C1->CR1 |= 1UL<<8;

	// SB bit is set when START condition is generated
	// So, it is polled to make sure communication has started
	while(!(I2C1->SR1 & I2C_SR1_SB)){}
}

void I2C_Write(uint8_t var){
	// Polling the TxE bit in the I2C_SR1 register to see if
	// data register is empty
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}

	// Put the data to be written in the I2C_DR register
	I2C1->DR = var;

	// Poll the BTF (Byte Transfer Finished) bit in the I2C_SR1 register to confirm
	// Byte transfer
	while(!(I2C1->SR1 & I2C_SR1_BTF)){}
}

void I2C_Send_Addr(uint8_t Addr){
	//Put the address to be sent into the I2C_DR register
	I2C1->DR = Addr;

	// ADDR bit is polled in the I2C_SR1 register for end of
	// address transmission
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}

	// Variable that will read I2C_SR1 and I2C_SR2
	// for clearing the ADDR bit
	clc = (I2C1->SR1 | I2C1->SR2);
}

void I2C_Stop(){
	// Stop Communication after current byte transfer
	I2C1->CR1 |= 1UL<<9;
}

void LCD_Write_Cmd(uint8_t Device_Addr,uint8_t Slave_Reg_Addr, uint8_t data){
	uint8_t d_h, d_l, d1, d2, d3, d4;
	d_h = data & 0xF0;
	d_l = (data << 4) & 0xF0;

	// Set EN = 1 and RS = 0 and R/W =0 for writing to
	// Instruction Register and send the Upper 4 bits
	d1 = d_h | 0x0C;
	d2 = d_h | 0x08; // Set EN = 0 and RS = 0 and set Back Light on

	// Set EN = 1 and RS = 0 and R/W = 0
	// for Instruction Register and send the Lower 4 bits
	d3 = d_l | 0x0C;
	d4 = d_l | 0x08; // Set EN = 0 and RS = 0 and set Back Light on

	I2C_Start();
	I2C_Send_Addr(Device_Addr);
	I2C_Write(Slave_Reg_Addr);
	I2C_Write(d1);
	TIM4_ms_Delay(2); // Wait for 2 ms for the command to take action
	I2C_Write(d2);
	I2C_Write(d3);
	TIM4_ms_Delay(2); // Wait for 2 ms for the command to take action
	I2C_Write(d4);
	I2C_Stop();
}

void LCD_Write_Data(uint8_t Device_Addr, uint8_t Slave_Reg_Addr, uint8_t data){
	uint8_t d_h, d_l, d1, d2, d3, d4;
	d_h = data & 0xF0;
	d_l = (data << 4) & 0xF0;

	// Set EN = 1 and RS = 1 and R/W = 0 for writing to
	// Data Register and send the Upper 4 bits
	d1 = d_h | 0x0D;
	d2 = d_h | 0x09; // Set EN = 0 and RS = 1 and set Back Light on

	// Set EN = 1 and RS = 1 and R/W = 0 for writing to
	// Data Register and send the Lower 4 bits
	d3 = d_l | 0x0D;
	d4 = d_l | 0x09; // Set EN = 0 and RS = 1 and set Back Light on

	I2C_Start();
	I2C_Send_Addr(Device_Addr);
	I2C_Write(Slave_Reg_Addr);
	I2C_Write(d1);
	TIM4_ms_Delay(2); // Wait for 2 ms for the data to be written
	I2C_Write(d2);
	I2C_Write(d3);
	TIM4_ms_Delay(2); // Wait for 2 ms for the data to be written
	I2C_Write(d4);
	I2C_Stop();
}

void LCD_Cursor(int r, int c){
	if (r==1){
		c |= 0xC0;
		LCD_Write_Cmd(LCD_ADDR, INST_REG, c);
	}
	else{
		c |= 0x80;
		LCD_Write_Cmd(LCD_ADDR, INST_REG, c);
	}
}

void LCD_Init(){
	// 1. Initializing the LCD in 4-bit mode
	TIM4_ms_Delay(50);
	LCD_Write_Cmd(LCD_ADDR,INST_REG,0x30);
	TIM4_ms_Delay(5);

	LCD_Write_Cmd(LCD_ADDR,INST_REG,0x30);
	TIM4_ms_Delay(1);

	LCD_Write_Cmd(LCD_ADDR,INST_REG,0x30);
	TIM4_ms_Delay(10);

	LCD_Write_Cmd(LCD_ADDR,INST_REG,0x20); // Set the LCD in 4-bit Mode
	TIM4_ms_Delay(10);

	// 2. Initializing the Display

	// Function Set (DL=0 for 4-bit mode; N=1 for 2-line display;
	// F=0 for 5x8 characters)
	LCD_Write_Cmd(LCD_ADDR,INST_REG,0x28);
	TIM4_ms_Delay(5);

	// Display Control (D=0;C=0;B=0 - Display is off)
	LCD_Write_Cmd(LCD_ADDR,INST_REG,0x08);
	TIM4_ms_Delay(5);

	// Clear the display
	LCD_Write_Cmd(LCD_ADDR,INST_REG,0x01);
	TIM4_ms_Delay(5);

	TIM4_ms_Delay(1); // Wait for some time

	// Set Entry Mode (ID=1 for incrementing cursor and S=0 for no shift)
	LCD_Write_Cmd(LCD_ADDR,INST_REG,0x07);
	TIM4_ms_Delay(5);

	// Display Control (D=1;C=0;B=0 - Cursor blinks)
	LCD_Write_Cmd(LCD_ADDR,INST_REG,0x0F);
	TIM4_ms_Delay(5);
}

void TIM4_ms_Delay(uint16_t delay){
	RCC->APB1ENR |= 1<<2; //Start the clock for the timer peripheral
	TIM4->PSC = 16000-1; //Setting the clock frequency to 1kHz.
	TIM4->ARR = (delay); // Total period of the timer
	TIM4->CNT = 0;
	TIM4->CR1 |= 1; //Start the Timer
	while(!(TIM4->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
	TIM4->SR &= ~(0x0001); //Reset the update interrupt flag
}

int main(void){
	GPIO_Init();
	I2C_Init();
	LCD_Init();
	LCD_Cursor(0,0); // Set the cursor at the first row and first column
	LCD_Write_Data(LCD_ADDR,DATA_REG, 0x84); // Character Generation ROM Pattern for 'H'
	LCD_Write_Data(LCD_ADDR,DATA_REG, 0x54); // Character Generation ROM Pattern for 'E'
	LCD_Write_Data(LCD_ADDR,DATA_REG, 0xC4); // Character Generation ROM Pattern for 'L'
	LCD_Write_Data(LCD_ADDR,DATA_REG, 0xC4); // Character Generation ROM Pattern for 'L'
	LCD_Write_Data(LCD_ADDR,DATA_REG, 0xF4); // Character Generation ROM Pattern for '0'
	TIM4_ms_Delay(1000); // Delay of 1s
	while(1){ // Dead Loop
	}
}
