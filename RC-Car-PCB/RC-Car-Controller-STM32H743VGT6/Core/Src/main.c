/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "SSD1306.h"
#include "ST7789.h"
#include "STC3100.h"
#include "XBEE.h"
#include "MenuOLED.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OLED_ADDR 0x3C
#define STC_ADDR  0x70

#define JPEG_MAX_WIDTH  184
#define JPEG_MAX_HEIGHT 240

#define JPEG_HEADERSIZE 526

#define INPUT_DEBOUNCE 20

// WATCHDOG DEFINES
#define WDOG_NETWORK_CUTOFF 4

// SCHEDULING DEFINES
#define SCH_MS_OLED 33
#define SCH_MS_TX 	100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_tx;

JPEG_HandleTypeDef hjpeg;
MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
MDMA_HandleTypeDef hmdma_jpeg_outfifo_ne;

SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

// WATCHDOG VARIABLES
uint8_t wdog_network = 0;

// SCHEDULING VARIABLES
uint32_t sch_tim_oled = 0;
uint32_t sch_tim_tx = 0;

// INPUT VARIABLES
GPIO_TypeDef *i_ports[4] = {BTN_RB_GPIO_Port, BTN_LB_GPIO_Port, BTN_RF_GPIO_Port, BTN_LF_GPIO_Port};
uint16_t	  i_pins[4]  = {BTN_RB_Pin, BTN_LB_Pin, BTN_RF_Pin, BTN_LF_Pin};

uint8_t  istate_hold[4]    = {0};
uint32_t istate_time[4]    = {0};
uint8_t  istate_pressed[4] = {0};

// MENU VARIABLES
uint8_t test_vals[4] = {0};

// SSD1306 VARIABLES
SSD1306_HandleTypeDef hssd1;
SSD1306_HandleTypeDef hssd2;

uint8_t ssd1_vram[CACHE_SIZE_MEM] = {0};
uint8_t ssd2_vram[CACHE_SIZE_MEM] = {0};
uint8_t ssd_msg[100] = {0};	// Reserve 100 bytes for SSD1306 text

uint8_t usb_msg[100] = {0};	// Reserve 100 bytes for USB Debug messages

// MENU VARIABLES
Menu_HandleTypeDef hmenu;

// ST7789 VARIABLES
ST7789_HandleTypeDef hst7789;
uint8_t st7789_vram[LCD_WIDTH*LCD_HEIGHT*2] = {0};
uint8_t st7789_state = 0;	// ST7789 STATE
// ------------------------ 0: LCD Idle
// ------------------------ 1: LCD Requested
// ------------------------ 2: LCD Busy

uint8_t st_interlacing = 0;

// ADC VARIABLES
uint16_t adc_buffer[20] = {0};
uint16_t adc_average[2] = {0};
uint8_t slider_magnitude[2] = {0};
uint8_t slider_direction[2] = {0};

// STC3100ST VARIABLES
STC3100_HandleTypeDef hstc;

//uint16_t slider_midpoint[2] = {0x80, 0x80};	// Define midpoints for sliders
uint8_t slider_min_deadzone = 16;	// Slider deadzone at min
uint8_t slider_max_deadzone = 12;	// Slider deadzone at max

// XBEE VARIABLES
XBEE_HandleTypeDef hxbee;

uint16_t jpeg_img_lastRcvPkt = 0;
uint8_t uart_rx_skippedPackets = 0;
// ----------------------------------- 0: GOOD
// ----------------------------------- 1: MALFORMED
// ----------------------------------- 2: BUSY

uint32_t avg_ms_imgRecv = 0;
uint32_t tim_ms_imgRecv = 0;

// JPEG VARIABLES
uint8_t  jpeg_quality = 0;

uint16_t jpeg_mcu_widths[4]   = {6, 8,  12, 23};
uint16_t jpeg_mcu_heights[4]  = {8, 10, 15, 30};
//uint8_t  jpeg_scaleFactors[4] = {1, 1,  1,  1};
uint8_t  jpeg_scaleFactors[4] = {6, 5,  3,  2};

uint8_t  jpeg_currentraw = 0;
uint8_t  jpeg_raw1[JPEG_MAX_WIDTH*JPEG_MAX_HEIGHT] = {0};
uint8_t  jpeg_raw2[JPEG_MAX_WIDTH*JPEG_MAX_HEIGHT] = {0};
uint8_t  jpeg_out[JPEG_MAX_WIDTH*JPEG_MAX_HEIGHT] = {0};
uint16_t jpeg_size = 0;
uint8_t  jpeg_state = 0; // JPEG State
// ------------------------ 0: JPEG Idle
// ------------------------ 1: JPEG DMA Busy
// ------------------------ 2: JPEG Decoded
uint8_t current_mcu_y = 0;

// Store the header for the JPEG, this saves on transmission time
// 526 BYTES
uint8_t jpeg_header[] = {
		0xFF, 0xD8, 0xFF, 0xDB, 0x00, 0x43, 0x00, 0x28,
		0x1C, 0x1E, 0x23, 0x1E, 0x19, 0x28, 0x23, 0x21,
		0x23, 0x2D, 0x2B, 0x28, 0x30, 0x3C, 0x64, 0x41,
		0x3C, 0x37, 0x37, 0x3C, 0x7B, 0x58, 0x5D, 0x49,
		0x64, 0x91, 0x80, 0x99, 0x96, 0x8F, 0x80, 0x8C,
		0x8A, 0xA0, 0xB4, 0xE6, 0xC3, 0xA0, 0xAA, 0xDA,
		0xAD, 0x8A, 0x8C, 0xC8, 0xFF, 0xCB, 0xDA, 0xEE,
		0xF5, 0xFF, 0xFF, 0xFF, 0x9B, 0xC1, 0xFF, 0xFF,

		0xFF, 0xFA, 0xFF, 0xE6, 0xFD, 0xFF, 0xF8, 0xFF,
		0xC0, 0x00, 0x0B, 0x08, 0x00, 0x79, 0x00, 0x9E,
		0x01, 0x01, 0x11, 0x00, 0xFF, 0xC4, 0x00, 0x1F,
		0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
		0x07, 0x08, 0x09, 0x0A, 0x0B, 0xFF, 0xC4, 0x00,
		0xB5, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02,

		0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00,
		0x01, 0x7D, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11,
		0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51,
		0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91,
		0xA1, 0x08, 0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52,
		0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09,
		0x0A, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x25, 0x26,
		0x27, 0x28, 0x29, 0x2A, 0x34, 0x35, 0x36, 0x37,

		0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47,
		0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57,
		0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67,
		0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77,
		0x78, 0x79, 0x7A, 0x83, 0x84, 0x85, 0x86, 0x87,
		0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96,
		0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5,
		0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4,

		0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3,
		0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2,
		0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA,
		0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8,
		0xE9, 0xEA, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6,
		0xF7, 0xF8, 0xF9, 0xFA, 0xFF, 0xC4, 0x00, 0x1F,
		0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

		0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
		0x07, 0x08, 0x09, 0x0A, 0x0B, 0xFF, 0xC4, 0x00,
		0xB5, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04,
		0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01,
		0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04,
		0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07,
		0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14,
		0x42, 0x91, 0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33,

		0x52, 0xF0, 0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16,
		0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19,
		0x1A, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x35, 0x36,
		0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46,
		0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56,
		0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66,
		0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76,
		0x77, 0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85,

		0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94,
		0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3,
		0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2,
		0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA,
		0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9,
		0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8,
		0xD9, 0xDA, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
		0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6,

		0xF7, 0xF8, 0xF9, 0xFA, 0xFF, 0xDA, 0x00, 0x08,
		0x01, 0x01, 0x00, 0x00, 0x3F, 0x00
};

// SCHEDULING VARIABLES
// Displays use DMA STREAM 1
// ORDER: SSD1 -> ST7789(1/2) -> SSD2 -> ST7789(1/2)
// When do we have clocks to spare?
// SSD	  DMA takes ~1K  clocks
// ST7789 DMA takes ~65K clocks

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_MDMA_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_JPEG_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI4_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

// Watchdog Fucntions
void NetworkTimeout();

// Scheduling Functions
void SCH_XBeeRX();
void SCH_XBeeTX();
void SCH_PowerMon();
void SCH_ImageDecode();
void SCH_LCDUpdate();
void SCH_OLEDUpdate();
void SCH_LCDUpdate();
void SCH_GetInputs();

// Scoping Functions
void ParsePacket_JPEG_IMAGE(uint8_t* packet, uint16_t byte_num);
void ParsePacket_JPEG_HEADER(uint8_t* packet, uint16_t byte_num);

void ParsePacket_RAW(uint8_t* packet, uint16_t byte_num);
void ParsePacket_COMMAND(uint8_t* packet, uint16_t byte_num);

// Utility Functions
uint32_t DeltaTime(uint32_t start_t);
uint8_t GetState(uint8_t byte_num);

// SSD drawing funcs
void Draw_Slider(uint8_t slider_id);
void WriteDebug(uint8_t *str_ptr, uint8_t str_len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// EXTERN IN USBD_CDC_IF
uint8_t usb_device_rxFlag = 0x00;
uint16_t uart_rx_len = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_MDMA_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_JPEG_Init();
  MX_I2C1_Init();
  MX_SPI4_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

	// XBEE READ MODE
//		uint8_t uart_xbee_buffer[256] = {0};	// Get a reference to the start of buffer data
//		uint8_t uart_xbee_len = 0;			// Get a reference to the start of buffer data
//		while (1) {
//			  HAL_UARTEx_ReceiveToIdle(&huart1, uart_xbee_buffer, 256, &uart_xbee_len, 1000);
//			  if (uart_xbee_len > 0) {
//				  CDC_Transmit_FS(uart_xbee_buffer, uart_xbee_len);
//				  uart_xbee_len = 0;
//				  memset(uart_xbee_buffer, 0x00, 256);
//			  } else {
//				  //sprintf(usb_msg, "err\r\n");
//				  //HAL_UART_Transmit(&huart1, usb_msg, strlen(usb_msg), 1000);
//			  }
//		}


	uint8_t init_result = 0;

	// ------------------------------------------------------------ SETUP ADC DMA -- //

	HAL_ADC_Start_DMA(&hadc1, adc_buffer, 20);

	// ------------------------------------------------------------ SETUP STC3100 -- //

	hstc.address = STC_ADDR;
	hstc.i2c_handle = &hi2c2;
	init_result = STC3100_Init(&hstc);
	if (init_result) {
		sprintf(usb_msg, "Failed to Init STC3100: %d\r\n", init_result);
		CDC_Transmit_FS(usb_msg, strlen(usb_msg));
		// This state is non-functional, reset
		NVIC_SystemReset();
		while (1) {}
	}

	// ------------------------------------------------------------ SETUP SSD1306 -- //

	hssd1.i2c_handle = &hi2c2;
	hssd1.address = OLED_ADDR;
	hssd1.vram_full = ssd1_vram;
	hssd1.draw_inverted = 0;
	hssd1.draw_scale = 0;
	init_result = SSD1306_Init(&hssd1);
	if (init_result) {
		sprintf(usb_msg, "Failed to Init SSD1: %d\r\n", init_result);
		CDC_Transmit_FS(usb_msg, strlen(usb_msg));
		// This state is non-functional, reset
		NVIC_SystemReset();
		while (1) {}
	}

	hssd2.i2c_handle = &hi2c1;
	hssd2.address = OLED_ADDR;
	hssd2.vram_full = ssd2_vram;
	hssd2.draw_inverted = 0;
	hssd2.draw_scale = 0;
	init_result = SSD1306_Init(&hssd2);
	if (init_result) {
		sprintf(usb_msg, "Failed to Init SSD2: %d\r\n", init_result);
		CDC_Transmit_FS(usb_msg, strlen(usb_msg));
		// This state is non-functional, reset
		NVIC_SystemReset();
		while (1) {}
	}

	// ------------------------------------------------------------ SETUP ST7789 -- //
	hst7789.spi_handle = &hspi4;
	hst7789.spi_state = 0;
	hst7789.dc_gpio_handle = SPI4_DC_GPIO_Port;
	hst7789.dc_gpio_pin = SPI4_DC_Pin;
	hst7789.vram = st7789_vram;
	init_result = ST7789_Init(&hst7789);
	if (init_result) {
		sprintf(usb_msg, "Failed to Init ST7789: %d\r\n", init_result);
		CDC_Transmit_FS(usb_msg, strlen(usb_msg));
		// This state is non-functional, reset
		NVIC_SystemReset();
		while (1) {}
	}


	ST7789_Clear(&hst7789);		 // Clear the screen
	ST7789_Draw_NOSIG(&hst7789); // Draw the NOSIG symbol
	st7789_state = 1;			 // Flag LCD as requested

	// ------------------------------------------------------------ SETUP MENU -- //
	hmenu.ssdL_handle = &hssd1;
	hmenu.ssdR_handle = &hssd2;
	hmenu.page_anim = 0;
	//hmenu.alert_current_con = 1;
	//hmenu.alert_voltage_con = 1;
	MENU_Init(&hmenu);

	// ------------------------------------------------------------ SETUP JPEG DECODE -- //
	// override the header
	// DO NOT MODIFY THE JPEG_RAW BUF BELOW BYTE 526
	memcpy(jpeg_raw1, jpeg_header, JPEG_HEADERSIZE);
	memcpy(jpeg_raw2, jpeg_header, JPEG_HEADERSIZE);

	// ------------------------------------------------------------ SETUP XBEE -- //
	hxbee.uart_handle = &huart1;
	hxbee.pktRx_max = 5;
	hxbee.pktTx_max = 5;

	if (XBEE_Init(&hxbee)) {
		sprintf(ssd_msg, " Failed to Init XBEE");
		WriteDebug(ssd_msg, strlen(ssd_msg));
		// This state is non-functional, reset
		NVIC_SystemReset();
		while (1) {}
	}

	// ------------------------------------------------------------ SETUP WATCHDOG TIMER-- //
	if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
	{
		sprintf(ssd_msg, " Failed to Start Watchdog");
		WriteDebug(ssd_msg, strlen(ssd_msg));
		// This state is non-functional, reset
		NVIC_SystemReset();
		while (1) {}
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		SCH_XBeeRX();		// Process any incoming packets
		SCH_PowerMon();		// Monitor Power
		SCH_GetInputs();	// Get user inputs
		SCH_OLEDUpdate();	// Update the OLEDs
		SCH_LCDUpdate();	// Update the LCD

		// Network timeout condition:
		// Don't SEND anything
		// Don't bother processing images
		// Don't update the screen
		//if (wdog_network < WDOG_NETWORK_CUTOFF) {

			SCH_XBeeTX();		// Send any neccesarry outgoing packets
			SCH_ImageDecode();	// Decode pending MCU blocks
		//}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.PLL3.PLL3M = 2;
  PeriphClkInitStruct.PLL3.PLL3N = 12;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 3;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00401959;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00401959;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief JPEG Initialization Function
  * @param None
  * @retval None
  */
static void MX_JPEG_Init(void)
{

  /* USER CODE BEGIN JPEG_Init 0 */

  /* USER CODE END JPEG_Init 0 */

  /* USER CODE BEGIN JPEG_Init 1 */

  /* USER CODE END JPEG_Init 1 */
  hjpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN JPEG_Init 2 */

  /* USER CODE END JPEG_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 7499998;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 111111;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * Enable MDMA controller clock
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI4_DC_GPIO_Port, SPI4_DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI4_RST_GPIO_Port, SPI4_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI4_DC_Pin */
  GPIO_InitStruct.Pin = SPI4_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI4_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_LB_Pin */
  GPIO_InitStruct.Pin = BTN_LB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_LB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_RB_Pin */
  GPIO_InitStruct.Pin = BTN_RB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_RB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_LF_Pin BTN_RF_Pin */
  GPIO_InitStruct.Pin = BTN_LF_Pin|BTN_RF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI4_RST_Pin */
  GPIO_InitStruct.Pin = SPI4_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI4_RST_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Watchdog Fucntions
void NetworkTimeout() {
	if (wdog_network < WDOG_NETWORK_CUTOFF) {
		// Increment the timeout ctr
		wdog_network++;

		// A timeout occurs when wdog_network == the Cutoff
		if (wdog_network == WDOG_NETWORK_CUTOFF) {
			ST7789_Draw_NOSIG(&hst7789); // Draw the NOSIG symbol
			st7789_state = 1;			 // Flag LCD as requested
		}
	}
}

//  Scoping Funtions
void ParsePacket_JPEG_IMAGE(uint8_t* packet, uint16_t byte_num) {
	// Data was fully sent
	if (byte_num < jpeg_img_lastRcvPkt && jpeg_state == 0) {
		// Start the jpeg decode
		jpeg_size = jpeg_img_lastRcvPkt*PKT_DATASIZE + JPEG_HEADERSIZE;
		HAL_StatusTypeDef ret;
		if (jpeg_currentraw)
			ret = HAL_JPEG_Decode_DMA(&hjpeg, jpeg_raw1, jpeg_size, jpeg_out, jpeg_mcu_widths[jpeg_quality]*jpeg_mcu_heights[jpeg_quality]*64);
		else {
			ret = HAL_JPEG_Decode_DMA(&hjpeg, jpeg_raw2, jpeg_size, jpeg_out, jpeg_mcu_widths[jpeg_quality]*jpeg_mcu_heights[jpeg_quality]*64);
		}
		jpeg_currentraw = !jpeg_currentraw;

		if (ret) {
			sprintf(ssd_msg, " JPEG FAIL %d", ret);
			WriteDebug(ssd_msg, strlen(ssd_msg));
		} else {
			jpeg_state = 1;	// Flag JPEG as busy
		}
	}
	jpeg_img_lastRcvPkt = byte_num;

	// fill in the received data
	if (jpeg_currentraw)
		memcpy(jpeg_raw1 + JPEG_HEADERSIZE + byte_num * 64, packet, 64);
	else {
		memcpy(jpeg_raw2 + JPEG_HEADERSIZE + byte_num * 64, packet, 64);
	}
}

void ParsePacket_JPEG_HEADER(uint8_t* packet, uint16_t byte_num) {
	uint16_t byte_num_conv = (0xFFFF - byte_num) - 1;
	for (uint8_t i = 0; i < PKT_DATASIZE; i++) {
		// Bounds check on last packet
		if (byte_num*PKT_DATASIZE + i >= JPEG_HEADERSIZE)
			return;

		// Update both buffers' headers
		jpeg_raw1[byte_num*PKT_DATASIZE + i] = packet[i];
		jpeg_raw2[byte_num*PKT_DATASIZE + i] = packet[i];
	}
}

// ------------------------------------------------------------ SCHDULING FUNCTIONS -- //
void SCH_XBeeRX() {
	// If there's a packet, process it
	// If the packet is good, push it to the screen
	uint16_t rx_byte;
	uint8_t *rx_packet;
	uint8_t ret = XBEE_RXPacket(&hxbee, &rx_packet, &rx_byte);
	if (ret == 0) {
		// Network is active, reset the watchdog
		wdog_network = 0;

		// Packet contains telemetry
		if (rx_byte == 0xFFFF) {
			// TODO: Parse Telemetry
			return;
		}

		// Packet contains JPEG HEADER data
		if (GetState(OP_CAMERA_ENCODING) == 0 && rx_byte > 0xFFF0) {
			ParsePacket_JPEG_HEADER(rx_packet, rx_byte);
			return;
		}

		// Packet contains JPEG IMAGE data
		if (GetState(OP_CAMERA_ENCODING) == 0 && rx_byte < jpeg_mcu_widths[jpeg_quality]*jpeg_mcu_heights[jpeg_quality] + 1) {
			ParsePacket_JPEG_IMAGE(rx_packet, rx_byte);
			return;
		}

		// Packet contains RAW image data
		if (GetState(OP_CAMERA_ENCODING) == 1 && rx_byte < jpeg_mcu_widths[jpeg_quality]*jpeg_mcu_heights[jpeg_quality] + 1) {
			ParsePacket_JPEG_IMAGE(rx_packet, rx_byte);
			return;
		}

		// TODO: Parse JPEG Header data
	}
}

void SCH_XBeeTX() {
	// Get delta time and allow delay for screen refresh
	uint32_t delta_t = DeltaTime(sch_tim_tx);
	if (delta_t < SCH_MS_TX) return;

	// Set the tank controls just before send, minimize latency
	hmenu.state_packet[RESERVE_LTRACK_MAG] = slider_magnitude[0];
	hmenu.state_packet[RESERVE_RTRACK_MAG] = slider_magnitude[1];
	hmenu.state_packet[RESERVE_LTRACK_DIR] = slider_direction[0];
	hmenu.state_packet[RESERVE_RTRACK_DIR] = slider_direction[1];

	if (XBEE_TXPacket(&hxbee, hmenu.state_packet, 0xFFFF))  {
		// Line busy, retry ASAP
		return;
	}

	// Update the timer for the next DT period
	sch_tim_tx = HAL_GetTick();
}

void SCH_PowerMon() {
	uint8_t result = STC3100_Get(&hstc);
	// General read error
	if (result == 1)
		return;

	if (result == 2) {
		// Battery error, we need to kill ourselves NOW
		//TODO: implement this
		return;
	}

	hmenu.current_con = hstc.current;
	hmenu.voltage_con = hstc.voltage;
	hmenu.bat_temp_con = hstc.temperature;
	hmenu.bat_perc_con = hstc.charge_percent*100.0; // Convert to 0-100%
	hmenu.bat_time_con = hstc.charge_time/60.0;		// Convert to minutes

	hmenu.alert_battery_con = hstc.charge_percent < 0.20;
	hmenu.alert_voltage_con = hstc.voltage < 3.75;
	hmenu.alert_current_con = hstc.current > 3;
}

void SCH_ImageDecode() {
	if (jpeg_state != 2) return;
		// Loop through every mcu block

		for (uint16_t mcu_x = 0; mcu_x < jpeg_mcu_widths[jpeg_quality]; mcu_x++) {
			uint16_t mcu_idx = current_mcu_y*jpeg_mcu_widths[jpeg_quality] + mcu_x;

			for (uint16_t y = 0; y < 8; y++) {
				for (uint16_t x = 0; x < 8; x++) {
					// Bounds check
					if ((mcu_x*8 + x) > LCD_WIDTH) continue;
					// COLOR FORMAT
					// |RRRRR GGG|GGG BBBBB|

					uint32_t pix_x = (mcu_x*8 + x)*jpeg_scaleFactors[jpeg_quality];
					if (pix_x >= LCD_WIDTH-1) continue;
					pix_x = LCD_WIDTH - pix_x - 1;
					uint32_t pix_y = (current_mcu_y*8 + y)*jpeg_scaleFactors[jpeg_quality];
					if (pix_y >= LCD_HEIGHT-2) continue;

					uint8_t sample = jpeg_out[mcu_idx*64 + y*8 + x];
					uint8_t msb = (sample & 0b11111000) | ((sample & 0b11100000)>>5);
					uint8_t lsb = ((sample & 0b11111000) >> 3) | ((sample & 0b00011100)<<3);

					// TODO: Speed this up as much as possible, even if it means skipping lines
					// Loop through the pixels in a box
					uint8_t perfScaleFac = jpeg_scaleFactors[jpeg_quality];
					if (perfScaleFac == 0)
						perfScaleFac = 1;

					for (uint8_t yOff = 0; yOff < perfScaleFac; yOff++) {
						uint32_t cached_yOff = (pix_y+yOff)*LCD_WIDTH*2;	// Cache the Y offset so we don't compute it every loop

						for (uint8_t xOff = 0; xOff < perfScaleFac; xOff++) {
							uint32_t cached_xOff = (pix_x+xOff)*2;

							hst7789.vram[cached_yOff + cached_xOff    ] = msb;
							hst7789.vram[cached_yOff + cached_xOff + 1] = lsb;
						}
					}
				}
			}
		}

		current_mcu_y++;

		if (current_mcu_y >= jpeg_mcu_heights[jpeg_quality]) {
			current_mcu_y = 0;	// Reset the V-MCU counter

			avg_ms_imgRecv *= 0.8;
			avg_ms_imgRecv += DeltaTime(tim_ms_imgRecv)*0.2;	// Get the time since last frame
			tim_ms_imgRecv = HAL_GetTick();						// start the frame-timer

			// Plaster the FPS on top of VRAM
			if (!GetState(OP_CAMERA_FRAMETIME))
				ST7789_Draw_DATA(&hst7789, avg_ms_imgRecv);

			jpeg_state = 0;		// Flag JPEG as idle
			st7789_state = 1;	// Flag LCD as requested

			st_interlacing = !st_interlacing; // Toggle interlacing
		}
}

void SCH_OLEDUpdate() {
	// Get delta time and allow delay for screen refresh
	uint32_t delta_t = DeltaTime(sch_tim_oled);
	if (delta_t < SCH_MS_OLED) return;

	// Update the timer for the next DT period
	sch_tim_oled = HAL_GetTick();

	// Display the screen contents
	// Clear existing VRAM
	SSD1306_Clear(&hssd1);
	SSD1306_Clear(&hssd2);

	// Draw the sliders
	Draw_Slider(0);
	Draw_Slider(1);

	MENU_Draw(&hmenu, delta_t);

	// Update the screens
	SSD1306_Update(&hssd1);
	SSD1306_Update(&hssd2);
}

void SCH_LCDUpdate() {
	// Update the displays
	if (hst7789.spi_state == 0 && st7789_state == 1) {
		ST7789_UpdateAutomatic(&hst7789);
		st7789_state = 0;
	}
}

void SCH_GetInputs() {
	for (uint8_t i = 0; i < 4; i++) {
		// Debounce timer
		if (DeltaTime(istate_time[i]) < INPUT_DEBOUNCE) continue;

		// Get input
		uint8_t i_new = !HAL_GPIO_ReadPin(i_ports[i], i_pins[i]);

		// Update debounce timer
		if (i_new != istate_hold[i])
			istate_time[i] = HAL_GetTick();

		istate_pressed[i] = i_new && !istate_hold[i];	// Just pressed
		istate_hold[i] = i_new;							// Update sotred val

		// DEBUG
//		if (istate_pressed[i]) {
//			test_vals[i]++;
//		}
	}

	// Update the menu state
	MENU_ParseInput(&hmenu, istate_pressed);
	istate_pressed[0] = 0;
	istate_pressed[1] = 0;
	istate_pressed[2] = 0;
	istate_pressed[3] = 0;

	// Update the JPEG settings
	if (jpeg_quality != GetState(OP_CAMERA_QUALITY)) {
		jpeg_quality = GetState(OP_CAMERA_QUALITY);
		// Clear the screen
		ST7789_Clear(&hst7789);
	}

}

// ------------------------------------------------------------ UTILITY FUNCTIONS -- //

uint32_t DeltaTime(uint32_t start_t) {
	uint32_t now_t = HAL_GetTick();
	if (now_t < start_t) {
		// Overflow has occurred
		return (0xFFFFFFFF - start_t) + now_t;
	}

	return now_t - start_t;
}

uint8_t GetState(uint8_t byte_num) {
	if (byte_num >= 64) return 0;
	return hmenu.state_packet[byte_num];
}

// DEBUG FUNCTIONS

// ------------------------------------------------------------ OVERRIDE UART DMA CALLBACKS -- //
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uint8_t ret = XBEE_RX_DMACallback(&hxbee);
	if (ret) {
//		sprintf(ssd_msg, " PKT Err: %d", ret);
//		WriteDebug(ssd_msg, strlen(ssd_msg));
	}
}

// ------------------------------------------------------------ OVERRIDE JPEG DMA CALLBACKS -- //
// JPEG hardware has completed the current image
void HAL_JPEG_DecodeCpltCallback(JPEG_HandleTypeDef * hjpeg) {
	// Reset JPEG variables
	//sprintf(ssd_msg, " JPEG CPLT %d", hjpeg->OutDataLength);
	//WriteDebug(ssd_msg, strlen(ssd_msg));
	jpeg_state = 2;
}

// JPEG hardware encountered an error
void HAL_JPEG_ErrorCallback (JPEG_HandleTypeDef * hjpeg) {
	//HAL_JPEG_Abort(&hjpeg);
	jpeg_state = 0;
}

void HAL_JPEG_DataReadyCallback (JPEG_HandleTypeDef * hjpeg, uint8_t * pDataOut, uint32_t OutDataLength) {
	// Abort if it's too long
	if (OutDataLength > JPEG_MAX_WIDTH*JPEG_MAX_HEIGHT) {
		HAL_JPEG_Abort(hjpeg);
		jpeg_state = 2;
		return;
	}

	jpeg_state = 2;
}

// ------------------------------------------------------------ OVERRIDE SPI DMA CALLBACKS -- //
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	ST7789_DMATransmitCplt(&hst7789);
}

// ------------------------------------------------------------ OVERRIDE ADC DMA CALLBACKS -- //
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
//	adc_average[0] = 0;
//	adc_average[1] = 0;
//	for (int i = 0; i < 20; i++) {
//		// Accumulate the samples
//		// Have to pre-divide so the result fits in a uint16
//		adc_average[i%2] += (adc_buffer[i])/10;	// DIV 10*4, this includes the 4 for the interp. process
//	}

	for (int i = 0; i < 2; i++) {

		adc_average[i] = adc_buffer[i]; // Skip the averaging process

		slider_direction[i] = !(adc_average[i] >> 15); // shift right to only keep 1 MSB (sign bit)
		slider_magnitude[i] = adc_average[i] >> 7;	// shift right to chop off 1 MSB and 7 LSB
		if (slider_direction[i]) slider_magnitude[i] = 0xFF - slider_magnitude[i];	// Flip the magnitude if the slider is inverted

		if (slider_magnitude[i] < slider_min_deadzone)
			slider_magnitude[i] = 0;

		if (slider_magnitude[i] > 0xFF-slider_max_deadzone)
			slider_magnitude[i] = 0xFF;

	}
}

// ------------------------------------------------------------ DRAW COMMANDS -- //
void Draw_Slider(uint8_t slider_id) {
	uint8_t byte_sel = slider_magnitude[slider_id] >> 5;					// Byte threshold
	uint8_t bit_sel = (slider_magnitude[slider_id] >> 2) & 0b00000111;		// Partial byte threshold
	uint8_t subbit_sel = (slider_magnitude[slider_id]) & 0b00000111;	// Fine control display

	uint8_t slider_vram[8] = {0};

	if (slider_direction[slider_id]) {
		for (int i = 0; i < 8; i++) {
			if (i < byte_sel) slider_vram[7-i] = 0xFF;	// Before partial byte, fill
			if (i > byte_sel) slider_vram[7-i] = 0x00;	// After partial byte, empty
			if (i == byte_sel) slider_vram[7-i] = 0xFF << (7-bit_sel);	// Partial byte
		}
	} else {
		for (int i = 0; i < 8; i++) {
			if (i < byte_sel) slider_vram[i] = 0xFF;
			if (i > byte_sel) slider_vram[i] = 0x00;
			if (i == byte_sel) slider_vram[i] = 0xFF >> (7-bit_sel);
		}
	}

	uint8_t slider_str[4] = {0};
	sprintf(slider_str, "%03d", (uint8_t)(slider_magnitude[slider_id] / 2.55));

	if (slider_id == 0) {
		uint16_t curs = 1;
		for (int y = 0; y < 8; y++) {
			for (int x = 3; x < 8; x++)
				ssd1_vram[curs + y*128 + x] = slider_vram[y];	// Set large bar
			ssd1_vram[curs + y*128 + 0] = 0x80 >> subbit_sel;	// Set the fine control disp.
			ssd1_vram[curs + y*128 + 1] = 0x80 >> subbit_sel;
		}
		hssd1.str_cursor = 10;
		hssd1.draw_inverted = 1;
		SSD1306_DrawString(&hssd1, slider_str, strlen(slider_str));
		hssd1.draw_inverted = 0;

		// Draw the strip on top
		for (uint8_t i = 10; i <= 128; i++) {
			if (i%2 == 0)
				ssd1_vram[128 + i] = 0b00010110;
			else
				ssd1_vram[128 + i] = 0b00001110;
		}

		// Pad the inverted number
		ssd1_vram[10] = 0xFF;
		ssd1_vram[29] = 0xFF;
		for (uint8_t i = 10; i < 30; i++) {
			ssd1_vram[128 + i] |= 0b00000011;
		}

		// Draw the cool bars
		for (uint8_t i = 0; i <= 4; i++) {
			ssd1_vram[30+i*2] = 0xFF << i;
			ssd1_vram[30+i*2 + 128] |= 0b00000011;
		}

		for (uint16_t i = 1; i <= 7; i++) {
			ssd1_vram[i*128 + 10] = 0xFF;
			ssd1_vram[i*128 + 11] = 0xFF;
			ssd1_vram[i*128 + 12] |= 0x55;
			ssd1_vram[i*128 + 13] |= 0xAA;
		}

	} else {
		uint16_t curs = 120;
		for (int y = 0; y < 8; y++) {
			for (int x = 0; x < 5; x++)
				ssd2_vram[curs + y*128 + x] = slider_vram[y];	// Set large bar
			ssd2_vram[curs + y*128 + 6] = 0x80 >> subbit_sel;	// Set the fine control disp.
			ssd2_vram[curs + y*128 + 7] = 0x80 >> subbit_sel;
		}
		hssd2.str_cursor = 98;
		hssd2.draw_inverted = 1;
		SSD1306_DrawString(&hssd2, slider_str, strlen(slider_str));
		hssd2.draw_inverted = 0;

		// Draw the strip on top
		for (uint8_t i = 0; i <= 128-10; i++) {
			if (i%2 == 0)
				ssd2_vram[128 + i] = 0b00010110;
			else
				ssd2_vram[128 + i] = 0b00001110;
		}

		// Pad the inverted number
		ssd2_vram[128-10] = 0xFF;
		ssd2_vram[128-11] = 0xFF;
		for (uint8_t i = 128-30; i < 128-10; i++) {
			ssd2_vram[128 + i] |= 0b00000011;
		}

		// Draw the cool bars
		for (uint8_t i = 0; i <= 4; i++) {
			ssd2_vram[128 - (30+i*2)] = 0xFF << i;
			ssd2_vram[128 - (30+i*2) + 128] |= 0b00000011;
		}

		for (uint16_t i = 1; i <= 7; i++) {
			ssd2_vram[i*128 + 118] = 0xFF;
			ssd2_vram[i*128 + 117] = 0xFF;
			ssd2_vram[i*128 + 116] |= 0x55;
			ssd2_vram[i*128 + 115] |= 0xAA;
		}
	}
}

// Debug
void WriteDebug(uint8_t *str_ptr, uint8_t str_len) {
	SSD1306_Clear(&hssd1);
	SSD1306_Clear(&hssd2);
	SSD1306_DrawString(&hssd1, str_ptr, str_len);
	SSD1306_DrawString(&hssd2, str_ptr, str_len);
	SSD1306_Update(&hssd1);
	SSD1306_Update(&hssd2);
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
