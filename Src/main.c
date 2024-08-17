#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dcmi.h"
#include <stdbool.h>
#include "stm32f4xx_usart.h"
#include <misc.h>
#include "stdlib.h"
#include "math.h"
#include <string.h>

#define LOCATE_CCMRAM __attribute__((section(".ccmram")))

//testpin parameters
#define GPIOAEN  (1U<<0)
#define PIN5 	 (1U<<5)
#define TEST_PIN  PIN5

//LED Register Parameters
#define GPIODEN 			(1U<<3) //shift 1 to position 3: GPIOD Enable
#define PIN12 				(1U<<12)
#define LED_PIN 			PIN12

//UART Parameters
#define GPIOAEN (1U<<0)
#define UART2EN (1U<<17)

#define CR1_TE (1U<<3)
#define CR1_RE (1U<<2)
#define CR1_UE (1U<<13)

#define SR_TXE (1U<<7)
#define SR_RXNE (1U<<5)

#define CR1_RXNEIE (1U<<5)
//Camera Parameters
#define OV7670_WRITE_ADDR 0x42	//I2C Slave Address
#define IMG_ROWS 144  			//Image Height
#define IMG_COLUMNS 160			//Image Width
#define OV7670_REG_NUM 123		//Number of registers

//Indicator Flags
static volatile bool frame_flag = false; 				//Flag to indicate DMA transfer of frame to frame_buffer
static volatile bool sendLine_flag = false;				//Flag to indicate Line request command received
static bool BrightSpotD = false;					//Flag to turn on Dynamic Bright Spot Detection
static bool CompHis = false;						//Flag to turn on Histogram computation
static bool SobelEdge = false;
static bool SobelT = false;

//Frame Buffers and Arrays
volatile static uint16_t frame_buffer[IMG_ROWS * IMG_COLUMNS]; 		//Buffer to store YUV422 Frame acquired from DCMI
volatile static uint16_t PixelCounter[256];				//Array to store occurrence of each pixel value
volatile static uint16_t EdgePixelCounter[256];
volatile static uint8_t buffer_2D[IMG_ROWS][IMG_COLUMNS];
volatile static uint16_t GaussianB [IMG_ROWS][IMG_COLUMNS]LOCATE_CCMRAM;
volatile static uint16_t SobelX [IMG_ROWS][IMG_COLUMNS];
volatile static uint16_t SobelY [IMG_ROWS][IMG_COLUMNS];
volatile static uint8_t SobelM [IMG_ROWS][IMG_COLUMNS];

//General Variables and Constants
static uint8_t startchar = 0xFF;  						// Start character used to indicate start of line in transmission
static uint8_t Histogram_DIS = 0xDD;						// Used to indicate that histogram is disabled

static volatile uint8_t temp = 0; 						// Used to store pixel value for transmission
static uint32_t GSLength = IMG_ROWS * IMG_COLUMNS;				// size of Greyscale buffer
static volatile uint8_t pixelval = 0;						// Used to store pixel value in Histogram computation
static volatile uint32_t Sum = 0;						// Used to store cumulative sum for Mean (32Bit used as max value abt 2mil)
static volatile uint32_t Mean = 0;						// Used to Store image Mean						
static volatile uint64_t StdDev1 = 0;
static volatile uint64_t StdDev2 = 0;
static volatile uint32_t StdDevInt = 0;						// Used to store image SD
static volatile uint64_t SumSquaredDiffs = 0;					// Used to store Sum of Squared differences
static volatile uint32_t threshold = 0;

//Function Declarations
void uart2_tx_init(void);								// Function to initialize UART TX/RX and INT
void uart2_write(uint8_t ch);								// Function to write values 0-255 via UART2
uint8_t Serial_read(void);								// Function to read data via RX
void Serial_log(char *s);								// Function to log error messages on Serial
void Serial_logi(int val);								// Function to transmit ASCII of values via Serial
void led_init(void);									// Function to initialize Yellow LED
void err_led(void);									// Function to blink Error LED
void Delay(volatile long nCount);							// Function for Delay
void MCO1_init(void);									// Function to intialize Microcontroller Clock Output(MCO)
void I2C1_init(void);									// Function to initialize I2C1 communication
bool I2C1_write(uint8_t reg_addr, uint8_t* data);					// Function to write to OV7670 registers via I2C (SCCB)
void camInit(void);									// Function to initialize all OV7670 registers via I2C
void DCMI_Configure(void);								// Function to configure DCMI interface and DMA transfer and DMA/DCMI Interrupts
void CaptureFrame(void);								// Function to enable DCMI to capture Frame and transfer to buffer via DMA
void FrameTransmit_ASCII(void);								// Function to transmit pixel data in readable format
void FrameTransmit_Line(void);								// Function to transmit frame line by line for viewing
void YUVtoGREY_2D(uint16_t volatile *frame_buffer, uint8_t volatile buffer_2D[IMG_ROWS][IMG_COLUMNS]); 			// Function to convert YUV422 pixel data to GS and store in temp_buffer
void Histogram(volatile uint16_t PixelCounter[256]);									// Function to perform Histogram computation
void MeanSD_Transmit(void);												// Function to transmit values of Mean and and SD to host
uint32_t sqr_root(uint64_t num);
void ApplyFilter3x3(volatile uint8_t input[IMG_ROWS][IMG_COLUMNS], uint16_t output[IMG_ROWS][IMG_COLUMNS], int kernel[3][3], int normalizer);
void ApplyFilter3x3_PP(volatile uint16_t input[IMG_ROWS][IMG_COLUMNS], uint16_t output[IMG_ROWS][IMG_COLUMNS], int kernel[3][3], int normalizer);
void TransmitLine_2D( volatile uint16_t PimaInitge[IMG_ROWS][IMG_COLUMNS]);
void TransmitLine_2D_UP( volatile uint8_t Pimage[IMG_ROWS][IMG_COLUMNS]);
void SobelEdgeDetection(uint8_t input[IMG_ROWS][IMG_COLUMNS], uint8_t output[IMG_ROWS][IMG_COLUMNS]);
void testpin_init(void);
void testpin_toggle(void);

//OV7670 Register List
const uint8_t OVRegisters[OV7670_REG_NUM][2] = { { 0x12, 0x80 }, //Reset all register values

		// Image format
		{ 0x12, 0x8 },		// 0x14 = QVGA size, RGB mode; 0x8 = QCIF, YUV; 0xc = QCIF (RGB)
		{ 0xc, 0x8 }, //
		{ 0x11, 0b1000000 }, //

		{ 0xb0, 0x84 },		//Color mode (Not documented??)

		// Hardware window
		{ 0x11, 0x01 },		//PCLK settings, 15fps
		{ 0x32, 0xa4 },		//HREF
		{ 0x17, 0x16 },		//HSTART initial = 17
		{ 0x18, 0x04 },		//HSTOP	 initial = 05
		{ 0x03, 0x0a },		//VREF
		{ 0x19, 0x02 },		//VSTART
		{ 0x1a, 0x7a },		//VSTOP

		// Scalling numbers
		// Scalling numbers
		{ 0x70, 0x3a },		//X_SCALING
		{ 0x71, 0x35 },		//Y_SCALING
		{ 0x72, 0x11 },		//DCW_SCALING
		{ 0x73, 0xf0 },		//PCLK_DIV_SCALING
		{ 0xa2, 0x02 },		//PCLK_DELAY_SCALING

		// Matrix coefficients
		{ 0x4f, 0x80 }, //
		{ 0x50, 0x80 }, //
		{ 0x51, 0x00 }, //
		{ 0x52, 0x22 }, //
		{ 0x53, 0x5e }, //
		{ 0x54, 0x80 }, //
		{ 0x58, 0x9e },

		// Gamma curve values
		{ 0x7a, 0x20 }, //
		{ 0x7b, 0x10 }, //
		{ 0x7c, 0x1e }, //
		{ 0x7d, 0x35 }, //
		{ 0x7e, 0x5a }, //
		{ 0x7f, 0x69 }, //
		{ 0x80, 0x76 }, //
		{ 0x81, 0x80 }, //
		{ 0x82, 0x88 }, //
		{ 0x83, 0x8f }, //
		{ 0x84, 0x96 }, //
		{ 0x85, 0xa3 }, //
		{ 0x86, 0xaf }, //
		{ 0x87, 0xc4 }, //
		{ 0x88, 0xd7 }, //
		{ 0x89, 0xe8 },

		// AGC and AEC parameters
		{ 0xa5, 0x05 }, //
		{ 0xab, 0x07 }, //
		{ 0x24, 0x95 }, //
		{ 0x25, 0x33 }, //
		{ 0x26, 0xe3 }, //
		{ 0x9f, 0x78 }, //
		{ 0xa0, 0x68 }, //
		{ 0xa1, 0x03 }, //
		{ 0xa6, 0xd8 }, //
		{ 0xa7, 0xd8 }, //
		{ 0xa8, 0xf0 }, //
		{ 0xa9, 0x90 }, //
		{ 0xaa, 0x94 }, //
		{ 0x10, 0x00 },

		// AWB parameters
		{ 0x43, 0x0a }, //
		{ 0x44, 0xf0 }, //
		{ 0x45, 0x34 }, //
		{ 0x46, 0x58 }, //
		{ 0x47, 0x28 }, //
		{ 0x48, 0x3a }, //
		{ 0x59, 0x88 }, //
		{ 0x5a, 0x88 }, //
		{ 0x5b, 0x44 }, //
		{ 0x5c, 0x67 }, //
		{ 0x5d, 0x49 }, //
		{ 0x5e, 0x0e }, //
		{ 0x6c, 0x0a }, //
		{ 0x6d, 0x55 }, //
		{ 0x6e, 0x11 }, //
		{ 0x6f, 0x9f }, //
		{ 0x6a, 0x40 }, //
		{ 0x01, 0x40 }, //
		{ 0x02, 0x60 }, //
		{ 0x13, 0xe7 },

		// Additional parameters
		{ 0x34, 0x11 }, //
		{ 0x3f, 0x00 }, //
		{ 0x75, 0x05 }, //
		{ 0x76, 0xe1 }, //
		{ 0x4c, 0x00 }, //
		{ 0x77, 0x01 }, //
		{ 0xb8, 0x0a }, //
		{ 0x41, 0x18 }, //
		{ 0x3b, 0x12 }, //
		{ 0xa4, 0x88 }, //
		{ 0x96, 0x00 }, //
		{ 0x97, 0x30 }, //
		{ 0x98, 0x20 }, //
		{ 0x99, 0x30 }, //
		{ 0x9a, 0x84 }, //
		{ 0x9b, 0x29 }, //
		{ 0x9c, 0x03 }, //
		{ 0x9d, 0x4c }, //
		{ 0x9e, 0x3f }, //
		{ 0x78, 0x04 }, //
		{ 0x0e, 0x61 }, //
		{ 0x0f, 0x4b }, //
		{ 0x16, 0x02 }, //
		{ 0x1e, 0x00 }, //
		{ 0x21, 0x02 }, //
		{ 0x22, 0x91 }, //
		{ 0x29, 0x07 }, //
		{ 0x33, 0x0b }, //
		{ 0x35, 0x0b }, //
		{ 0x37, 0x1d }, //
		{ 0x38, 0x71 }, //
		{ 0x39, 0x2a }, //
		{ 0x3c, 0x78 }, //
		{ 0x4d, 0x40 }, //
		{ 0x4e, 0x20 }, //
		{ 0x69, 0x00 }, //
		{ 0x6b, 0x3a }, //
		{ 0x74, 0x10 }, //
		{ 0x8d, 0x4f }, //
		{ 0x8e, 0x00 }, //
		{ 0x8f, 0x00 }, //
		{ 0x90, 0x00 }, //
		{ 0x91, 0x00 }, //
		{ 0x96, 0x00 }, //
		{ 0x9a, 0x00 }, //
		{ 0xb1, 0x0c }, //
		{ 0xb2, 0x0e }, //
		{ 0xb3, 0x82 }, //
		{ 0x4b, 0x01 }, };

int gaussian_kernel[3][3] =
	{
        { 1, 2, 1 },
        { 2, 4, 2 },
        { 1, 2, 1 }
    };

int boxblur_kernel[3][3] =
	{
        { 1, 1, 1 },
        { 1, 1, 1 },
        { 1, 1, 1 }
    };

    int sobel_x_kernel[3][3] =
	{
        { -1, 0, 1 },
        { -2, 0, 2 },
        { -1, 0, 1 }
    };
    int sobel_y_kernel[3][3] =
	{
        { -1, -2, -1 },
        { 0, 0, 0 },
        { 1, 2, 1 }
    };

int __io_putchar(uint8_t ch) //Allow printf for register configuration log
{

	uart2_write(ch);
	return ch;

}

int main(void)
{


		uart2_tx_init();	// Initiliaze UART to allow serial logs for error debug/frame transmission
		led_init();		// Initialize Error LED
		MCO1_init();		// Initialize MCO1 clock supply at 16MHz
		I2C1_init();		// Initialise I2C communication at 100KHz
		camInit();		// Configure all OV7670 registers
		DCMI_Configure();	// Configure DCMI and DMA for transmission
		CaptureFrame();		// Capture first frame


		while (1) 	{					//Repetitive loop to continously capture/process/transmit frame
				if (frame_flag == true)			// flag set if DMA transfer from DCMI to frame_buffer completed
				{
					frame_flag = false;		// Reset frame ready flag
					YUVtoGREY_2D(frame_buffer, buffer_2D);

					if(CompHis == true)
					{
						Histogram(PixelCounter);
						TransmitLine_2D_UP(buffer_2D);
					}

					else if(SobelEdge == true)
					{

						//ApplyFilter3x3(buffer_2D, GaussianB, boxblur_kernel,9);
						GPIOD->ODR ^=LED_PIN;
						SobelEdgeDetection(buffer_2D, SobelM);
						GPIOD->ODR ^=LED_PIN;
						Histogram(EdgePixelCounter);
						TransmitLine_2D_UP(SobelM);

					}

					else
					{
						TransmitLine_2D_UP(buffer_2D);
					}

					//ApplyFilter3x3(buffer_2D, GaussianB, gaussian_kernel,16);
					//ApplyFilter3x3(buffer_2D, GaussianB, boxblur_kernel,9);
					//ApplyFilter3x3_PP(GaussianB, SobelY, sobel_y_kernel,0);
					//TransmitLine_2D(SobelY);
					//memset(buffer_2D, 0, sizeof(buffer_2D));
					//memset(SobelY, 0, sizeof(SobelY));

					CaptureFrame();



				}
					}
}



////////////////////////////////////////////////Interrupt Handlers///////////////////////////////////////////////////////////////
void DMA2_Stream1_IRQHandler(void)
{
	// DMA complete
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET) { // Transfer complete
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
		frame_flag = true;
	} else if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TEIF1) != RESET) { // Transfer error
		// Not used, just for debug
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TEIF1);
	}
}

void DCMI_IRQHandler(void)
{
	if (DCMI_GetFlagStatus(DCMI_FLAG_FRAMERI) == SET) { // Frame received
		DCMI_ClearFlag(DCMI_FLAG_FRAMERI);
		// After receiving a full frame we disable capture and the DMA transfer. This is probably a very inefficient way of capturing and sending frames
		// but it's the only way I've gotten to reliably work.
		DMA_Cmd(DMA2_Stream1, DISABLE);
		DCMI_Cmd(DISABLE);
		DCMI_CaptureCmd(DISABLE);
	}
	if (DCMI_GetFlagStatus(DCMI_FLAG_OVFRI) == SET) { // Overflow
		// Not used, just for debug
		DCMI_ClearFlag(DCMI_FLAG_OVFRI);
	}
	if (DCMI_GetFlagStatus(DCMI_FLAG_ERRRI) == SET) { // Error
		// Not used, just for debug
		DCMI_ClearFlag(DCMI_FLAG_ERRRI);
	}
}



void USART2_IRQHandler(void)
{

	if (Serial_read() == 'L')
		{
			sendLine_flag = true;
		}


	if (Serial_read() == 'E')
		{

			CompHis = false;
			BrightSpotD = false;
			SobelEdge = !SobelEdge;


		}

	if (Serial_read() == 'T')
			{

				SobelT = !SobelT;


			}

	if (Serial_read() == 'D')
		{

		  BrightSpotD = !BrightSpotD;	//Enable dynamic BSD
		  SobelEdge = false;

		}

	if (Serial_read() == 'H')
		{
			CompHis = !CompHis;
			SobelEdge = false;
		}


	USART_ClearFlag(USART2, USART_FLAG_RXNE);
}

////////////////////////////////////////////////MCO1 Functions///////////////////////////////////////////////////////////////

void MCO1_init(void)
{

		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_ClockSecuritySystemCmd(ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//Enable AHB1 Clock

		//Configure PA8 to output clock
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				//PA8 - XCLK
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);


		GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);	//Configure PA8 AF

		//Define MCO1 clock source
		RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1);	// Use HSI clock at 16 MHz


}

////////////////////////////////////////////////LED Functions///////////////////////////////////////////////////////////////
void Delay(volatile long nCount)
{

	while (nCount--) {}

}

void led_init(void)
{

	//Enable clock access to GPIOD
	RCC->AHB1ENR |= GPIODEN;

	//Set PD12 as output pin
	GPIOD->MODER |= (1U<<24);
	GPIOD->MODER &=~(1U<<25);
}

void err_led(void)
{

	GPIOD->ODR ^=LED_PIN;
	for (int i =0;i<100000;i++){}
	GPIOD->ODR ^=LED_PIN;


}

void testpin_init(void)
{
	//Enable clock access to GPIOD
	RCC->AHB1ENR |= GPIOAEN;

	//Set PD12 as output pin
	GPIOD->MODER |= (1U<<10);
	GPIOD->MODER &=~(1U<<11);

}


void testpin_toggle(void)
{

	GPIOD->ODR ^=TEST_PIN;

}

////////////////////////////////////////////////I2C Functions///////////////////////////////////////////////////////////////
void I2C1_init(void){

	GPIO_InitTypeDef GPIO_InitStructure; // this is for the GPIO pins used as I2C1SDA and I2C1SCL
	 I2C_InitTypeDef I2C_InitStructure; // this is for the I2C1 initilization

	 /* enable APB1 peripheral clock for I2C1*/
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	 /* enable the peripheral clock for the pins used by
	  PB6 for I2C SCL and PB9 for I2C1_SDL*/
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	 /* This sequence sets up the I2C1SDA and I2C1SCL pins
	  * so they work correctly with the I2C1 peripheral
	  */
	 GPIO_StructInit(&GPIO_InitStructure);
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; // Pins 8(I2C1_SCL) and 9(I2C1_SDA)
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // the pins are configured as alternate function so the USART peripheral has access to them
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;// this defines the IO speed and has nothing to do with the baudrate!
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;// this defines the output type as open drain
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// this activates the pullup resistors on the IO pins
	 GPIO_Init(GPIOB, &GPIO_InitStructure);// now all the values are passed to the GPIO_Init()


	 /* The I2C1_SCL and I2C1_SDA pins are now connected to their AF
	  * so that the I2C1 can take over control of the
	  * pins
	  */
	 GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); //
	 GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	  /* Configure I2C1 */
	  I2C_StructInit(&I2C_InitStructure);
	  I2C_DeInit(I2C1);

	  /* Enable the I2C peripheral */
	  I2C_Cmd(I2C1, ENABLE);

	  /* Set the I2C structure parameters */
	  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	  I2C_InitStructure.I2C_ClockSpeed = 100000;
	  /* I2C Peripheral Enable */
	  I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
	  /* Initialize the I2C peripheral w/ selected parameters */
	  I2C_Init(I2C1,&I2C_InitStructure);
	  I2C_Cmd(I2C1, ENABLE);

}

bool I2C1_write(uint8_t reg_addr, uint8_t* data)
	{

	uint32_t timeout = 0x7FFFFF;

		while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
			if ((timeout--) == 0) {
				Serial_log("Busy Timeout\r\n");
				return true;
			}
		}

		// Send start bit
		I2C_GenerateSTART(I2C1, ENABLE);

		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
			if ((timeout--) == 0) {
				Serial_log("Start bit Timeout\r\n");
				return true;
			}
		}

		// Send slave address (camera write address)
		I2C_Send7bitAddress(I2C1, OV7670_WRITE_ADDR, I2C_Direction_Transmitter);

		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			if ((timeout--) == 0) {
				Serial_log("Slave address timeout\r\n");
				return true;
			}
		}

		// Send register address
		I2C_SendData(I2C1, reg_addr);

		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
			if ((timeout--) == 0) {
				Serial_log("Register timeout\r\n");
				return true;
			}
		}

		// Send new register value
		I2C_SendData(I2C1, *data);

		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
			if ((timeout--) == 0) {
				Serial_log("Value timeout\r\n");
				return true;
			}
		}

		// Send stop bit
		I2C_GenerateSTOP(I2C1, ENABLE);
		return false;
	}

////////////////////////////////////////////////OV7670 Register Configuration///////////////////////////////////////////////////////////////
void camInit(void) {


	bool err = false; 						// Set error flag to false
	uint8_t data, i = 0;

	for (i = 0; i < OV7670_REG_NUM; i++) {
		data = OVRegisters[i][1]; 					// Copy register value to data
		err = I2C1_write(OVRegisters[i][0], &data);	// Write values to OV register
		if (err == true)
		{
			err_led();	// If returned value is true, blink LED to indicate error
			printf("Register %d Error\n\r",i+1);
		}

		Delay(0xFF); // Add short delay between each write
	}

	printf("Registers Successfully Configured\n\r");
}

////////////////////////////////////////////////UART Functions///////////////////////////////////////////////////////////////
void uart2_tx_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // PA.2 USART2_TX, PA.3 USART2_RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Set Alternate Function for RX and TX pins
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	USART_InitStructure.USART_BaudRate = 230400; //230400 works fine for me but 115200 might be a safer option
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	// Enable interrupts for USART2 so that we can detect
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE);
}


void uart2_write(uint8_t ch)
{

	//make sure transmit data register is empty
	while(!(USART2->SR & SR_TXE)){}


	//write to transmit data register
	USART2->DR = (ch & 0xFF);

}


uint8_t Serial_read(void)
{
	return USART_ReceiveData(USART2);
}

void Serial_logi(int val) {
	char buffer[10];
	itoa(val, buffer, 10);
	Serial_log(&buffer[0]);
}

void Serial_log(char *s) {
	while (*s) {
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
			; // Wait for Empty

		USART_SendData(USART2, *s++); // Send Char
	}
}

////////////////////////////////////////////////DCMI/DMA Functions///////////////////////////////////////////////////////////////
void DCMI_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  DCMI_InitTypeDef DCMI_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

 /* GPIOD Periph clock enable */
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);



 /* DCMI GPIO configuration */

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

/* B7: VSYNC*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI);
    /* A4: HSYNC*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI);
    /* A6: PCLK*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI);
    /* C6: data0*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI);
    /* C7: data1*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);
    /* C8: data2*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_DCMI);
    /* C9: data3*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_DCMI);
    /* E4: data4*/
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI);
    /* B6: data5*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI);
    /* E5: data6*/
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI);
    /* E6: data7*/
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI);

      /* DCMI configuration */

    	DCMI_DeInit();
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot; //DCMI_CaptureMode_SnapShot
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame; //DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_Init(&DCMI_InitStructure);
	DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
	DCMI_ITConfig(DCMI_IT_OVF, ENABLE);
	DCMI_ITConfig(DCMI_IT_ERR, ENABLE);

	// DMA config
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&DCMI->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) frame_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = IMG_ROWS * IMG_COLUMNS / 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TE, ENABLE);

	/* DMA2 IRQ channel Configuration */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//DMA_Cmd(DMA2_Stream1, ENABLE);
	//DCMI_Cmd(ENABLE);
	//DCMI_CaptureCmd(ENABLE);
}

void CaptureFrame(void)
{
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
}

////////////////////////////////////////////////Image Processing Functions///////////////////////////////////////////////////////////////

//Convert YUV422 Buffer to GreyScale

void YUVtoGREY_2D(uint16_t volatile *frame_buffer, uint8_t volatile buffer_2D[IMG_ROWS][IMG_COLUMNS]) {

	uint8_t volatile *buffer = (uint8_t *) frame_buffer;
    uint16_t length = IMG_ROWS * IMG_COLUMNS * 2;
    memset(PixelCounter, 0, sizeof(PixelCounter));

    for (int i = 0; i < length; i += 2)
    {
    	uint16_t index = i / 2;
    	uint16_t row = index / IMG_COLUMNS;
    	uint16_t col = index % IMG_COLUMNS;
        buffer_2D[row][col] = buffer[i + 1];
		uint8_t graylevel = buffer[i + 1];
        PixelCounter[graylevel]++;
    }

}
// At the end of this function, buffer_2D now contains the Y components of frame_buffer (Greyscale of image)
//Pixelcounter is updated with occurrence of each pixel of values 0 - 255

//Perform Histogram processing to obtain Mean,SD,Threshold
 void Histogram(volatile uint16_t PixelCounter[256])
{
	SumSquaredDiffs = 0;
	Sum = 0;

	// Calculate Mean of Image
	for(int j = 0; j<256 ; j++)
	{
		Sum += (j*PixelCounter[j]);
	}


	Mean = (uint32_t)((Sum + (GSLength / 2)) / GSLength);


	// Calculate the sum of squared differences for standard deviation
    for (int i = 0; i < 256; i++)
    {
        int32_t diff = i - Mean;
        SumSquaredDiffs += (uint64_t)(diff * diff) * PixelCounter[i];
    }


    // Calculate standard deviation
    StdDev1 = (SumSquaredDiffs + (GSLength / 2));
    StdDev2 = StdDev1/GSLength;
    StdDevInt = sqr_root(StdDev2);
    // Mean and StdDev now contain the mean and standard deviation of the pixel values

	threshold = Mean + StdDevInt;



}



 uint32_t sqr_root(uint64_t num) {
     if (num == 0) {
         return 0;
     }

     uint64_t res = 0;
     uint64_t bit = 1ULL << 62;  // The second-to-top bit is set

     // "bit" starts at the highest power of four <= the argument.
     while (bit > num) {
         bit >>= 2;
     }

     while (bit != 0) {
         if (num >= res + bit) {
             num -= res + bit;
             res = (res >> 1) + bit;
         } else {
             res >>= 1;
         }
         bit >>= 2;
     }

     return (uint32_t)res;
 }

void ApplyFilter3x3(volatile uint8_t input[IMG_ROWS][IMG_COLUMNS], uint16_t output[IMG_ROWS][IMG_COLUMNS], int kernel[3][3], int normalizer) {
    int i, j, ki, kj;
    uint16_t sum;

    // Iterate over each pixel in the image, excluding the borders
    for (i = 1; i < IMG_ROWS - 1; i++) {
        for (j = 1; j < IMG_COLUMNS - 1; j++) {
            sum = 0;  // Reset sum for each pixel

            // Apply the kernel to the pixel and its neighbors
            for (ki = -1; ki < 2; ki++) {
                for (kj = -1; kj < 2; kj++) {
                    sum += input[i + ki][j + kj] * kernel[ki + 1][kj + 1];
                }
            }

            if (normalizer != 0) {
                          sum /= normalizer;
                      }
            // Clamp the result to the range [0, 255]
            if (sum < 0) sum = 0;
            if (sum > 255) sum = 255;
            output[i][j] = sum;

        }
    }
}

void ApplyFilter3x3_PP(volatile uint16_t input[IMG_ROWS][IMG_COLUMNS], uint16_t output[IMG_ROWS][IMG_COLUMNS], int kernel[3][3], int normalizer) {
    int i, j, ki, kj;
    uint16_t sum;

    // Iterate over each pixel in the image, excluding the borders
    for (i = 1; i < IMG_ROWS - 1; i++) {
        for (j = 1; j < IMG_COLUMNS - 1; j++) {
            sum = 0;  // Reset sum for each pixel

            // Apply the kernel to the pixel and its neighbors
            for (ki = -1; ki < 2; ki++) {
                for (kj = -1; kj < 2; kj++) {
                    sum += input[i + ki][j + kj] * kernel[ki + 1][kj + 1];
                }
            }

            if (normalizer != 0) {
                          sum /= normalizer;
                      }
            // Clamp the result to the range [0, 255]
            if (sum < 230) sum = 0;
            if (sum > 230) sum = 255;
            //not bad reversed, limit 120
            output[i][j] = sum;
        }
    }
}

void TransmitLine_2D( volatile uint16_t Pimage[IMG_ROWS][IMG_COLUMNS])
{

	for(int j = 0;j<IMG_ROWS;j++)
	{

		while(1)
		{

			if(sendLine_flag == true)
			{
				if(CompHis == true)
				{
					uart2_write(Mean);
					uart2_write(StdDevInt);

				}

				else
				{
					uart2_write(Histogram_DIS);
					uart2_write(Histogram_DIS);
				}

				uart2_write(startchar);
				uart2_write(j);
				uart2_write(IMG_COLUMNS);


				for(int k = 0; k<IMG_COLUMNS; k++)
					{


						temp = Pimage[j][k];

						if (CompHis && BrightSpotD)
						{
							if (temp > threshold)
							{
								temp = 255;
							}

							else
							{
								temp = 0;
							}
						}

						temp = (uint8_t)((temp*127)/(255));
						uart2_write(temp);

					}
				sendLine_flag = false;
				break;
			}

		}

	}


}

void TransmitLine_2D_UP( volatile uint8_t Pimage[IMG_ROWS][IMG_COLUMNS])
{

	for(int j = 0;j<IMG_ROWS;j++)
	{

		while(1)
		{

			if(sendLine_flag == true)
			{

				if(CompHis == true)
				{
					uart2_write(Mean);
					uart2_write(StdDevInt);

				}

				else
				{
					uart2_write(Histogram_DIS);
					uart2_write(Histogram_DIS);
				}

				uart2_write(startchar);
				uart2_write(j);
				uart2_write(IMG_COLUMNS);


				for(int k = 0; k<IMG_COLUMNS; k++)
					{


						temp = Pimage[j][k];

						if ((CompHis == true && BrightSpotD == true) || SobelT == true )
						{
							if (temp > threshold)
							{
								temp = 255;
							}

							else
							{
								temp = 0;
							}
						}

						temp = (uint8_t)((temp*127)/(255));
						uart2_write(temp);

					}
				sendLine_flag = false;
				break;
			}

		}

	}


}

void SobelEdgeDetection(uint8_t input[IMG_ROWS][IMG_COLUMNS], uint8_t output[IMG_ROWS][IMG_COLUMNS]) {
    int gx, gy;
    int magnitude;
    memset(EdgePixelCounter, 0, sizeof(EdgePixelCounter));

    for (int i = 1; i < IMG_ROWS - 1; i++) {
        for (int j = 1; j < IMG_COLUMNS - 1; j++) {
            gx = (-1 * input[i - 1][j - 1]) + (1 * input[i - 1][j + 1])
               + (-2 * input[i][j - 1])     + (2 * input[i][j + 1])
               + (-1 * input[i + 1][j - 1]) + (1 * input[i + 1][j + 1]);

            gy = (-1 * input[i - 1][j - 1]) + (-2 * input[i - 1][j]) + (-1 * input[i - 1][j + 1])
               + (1 * input[i + 1][j - 1])  + (2 * input[i + 1][j])  + (1 * input[i + 1][j + 1]);

            magnitude = sqr_root((gx * gx) + (gy * gy));

            if (magnitude < 0) magnitude = 0;
            if (magnitude > 255) magnitude = 255;

            EdgePixelCounter[magnitude]++;
            output[i][j] = magnitude;

        }
    }
}

