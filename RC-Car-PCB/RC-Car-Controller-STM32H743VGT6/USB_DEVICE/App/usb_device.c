/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
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

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
USBD_CDC_HandleTypeDef* hcdc;
/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

extern uint8_t usb_device_rxFlag;

#define MIN(a,b) (((a)<(b))?(a):(b))

uint8_t SendDataWithCorrection(uint8_t* pData, uint32_t dLen, uint16_t packetSize) {
	hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassDataCmsit[hUsbDeviceFS.classId];
	for (int i = 0; i < dLen/packetSize; i++) {
		usb_device_rxFlag = 0x00; // Reset flag
		// Keep transmitting data as long as received does not match sent
		while (usb_device_rxFlag == 0) {// || !memcmp(hcdc->RxBuffer, pData + i*packetSize, MIN(hcdc->RxLength, packetSize))) {
			// Transmit byte number, preserves ordering on host side
			while (CDC_Transmit_FS(&i, 4)) { HAL_Delay(5); }
			HAL_Delay(2);
			// Transmit packet
			while (CDC_Transmit_FS(pData + i*packetSize, packetSize)) { HAL_Delay(1); }
			HAL_Delay(5);
		}
	}
	return 0;
}

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  HAL_PWREx_EnableUSBVoltageDetector();

  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassDataCmsit[hUsbDeviceFS.classId];

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

