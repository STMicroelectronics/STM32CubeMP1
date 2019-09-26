/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   Main program body.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUFFER_SIZE RPMSG_BUFFER_SIZE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IPCC_HandleTypeDef hipcc;
CRC_HandleTypeDef hcrc2;

/* USER CODE BEGIN PV */
VIRT_UART_HandleTypeDef huart0;

__IO FlagStatus VirtUart0RxMsg = RESET;
uint8_t VirtUart0ChannelBuffRx[MAX_BUFFER_SIZE];
uint16_t VirtUart0ChannelRxSize = 0;

__IO FlagStatus VirtUart0TxMsg = RESET;
uint8_t VirtUart0ChannelBuffTx[100];
uint16_t VirtUart0ChannelTxSize = 0;

uint8_t touch_coordinate_x = 0xFF;
uint8_t touch_coordinate_y = 0xFF;

static float ai_buffer_input[28][28];
uint8_t ai_input_type = INPUT_IS_LETTER_AND_DIGIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_IPCC_Init(void);
static void MX_CRC2_Init(void);
/* USER CODE BEGIN PFP */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t computeCRC(uint8_t *bb, uint8_t size)
{
  uint16_t sum = 0;
  uint8_t res = 0;

  for (int i = 0; i < (size - 1); i++)
    sum += (bb[i] & 0xFF);

  res = (uint8_t) (sum & 0xFF);
  res += (uint8_t) (sum >> 8);
  return res;
}

#define ABS(X)      ((X) > 0 ? (X) : -(X))
void buffer_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
  int8_t deltax = 0, deltay = 0;
  int8_t x = 0, y = 0;
  int8_t xinc1 = 0, xinc2 = 0, yinc1 = 0, yinc2 = 0;
  int8_t den = 0, num = 0, numadd = 0, numpixels = 0, curpixel = 0;

  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */

  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    ai_buffer_input[y][x] = 1.0F;                  /* Draw the current pixel */
    /* draw segment with a pitch of 2 pixels */
    ai_buffer_input[y + yinc1][x + xinc1] = 1.0F;

    num += numadd;                            /* Increase the numerator by the top of the fraction */
    if (num >= den)                           /* Check if numerator >= denominator */
    {
      num -= den;                             /* Calculate the new numerator value */
      x += xinc1;                             /* Change the x as appropriate */
      y += yinc1;                             /* Change the y as appropriate */
    }
    x += xinc2;                               /* Change the x as appropriate */
    y += yinc2;                               /* Change the y as appropriate */
  }
}

void compute_touch_screen_coordinate(uint8_t x, uint8_t y)
{
  log_dbg("touch screen coordinate {%d, %d}\n", x, y);

  /* a break line information is received */
  if (x == 0xFF && y == 0xFF) {
    touch_coordinate_x = 0xFF;
    touch_coordinate_y = 0xFF;
    return;
  }

  if (touch_coordinate_x == 0xFF && touch_coordinate_y == 0xFF) {
    touch_coordinate_x = x;
    touch_coordinate_y = y;
  } else {
    buffer_draw_line(touch_coordinate_x, touch_coordinate_y, x, y);
    touch_coordinate_x = x;
    touch_coordinate_y = y;
  }
}

void start_ai_nn_processing()
{
  uint8_t character;
  uint8_t accuracy = 0;
  uint16_t elapsetime = 0; /* ms */
  uint32_t time_start, time_stop;
  char picture[30 * 30];
  int i = 0;

  /* log out the 28x28 picture */
  memset(picture, 0x00 , sizeof(picture));
  for (int y=0 ; y < 28 ; y++) {
    for (int x=0 ; x < 28 ; x++) {
      if (ai_buffer_input[y][x] == 1.0F)
        sprintf(picture + i, "#");
      else
        sprintf(picture + i, "_");
      i++;
    }
    sprintf(picture + i, "\n");
    i++;
  }
  log_info("NN input:\n%s\n", picture);

  /* start ai nn processing */
  log_info("start ai nn processing\n");
  time_start = HAL_GetTick();
  character = MX_X_CUBE_AI_Process(&ai_buffer_input[0][0], &accuracy, ai_input_type);
  time_stop = HAL_GetTick();

  elapsetime = (uint16_t)(time_stop - time_start);

  /* send the result */
  memset(VirtUart0ChannelBuffTx, 0 , sizeof(VirtUart0ChannelBuffTx));
  VirtUart0ChannelTxSize = 7;
  VirtUart0ChannelBuffTx[0] = 0xF0;
  VirtUart0ChannelBuffTx[1] = 4;
  VirtUart0ChannelBuffTx[2] = character;
  VirtUart0ChannelBuffTx[3] = accuracy;
  VirtUart0ChannelBuffTx[4] = elapsetime >> 8;
  VirtUart0ChannelBuffTx[5] = elapsetime & 0xFF;
  VirtUart0ChannelBuffTx[6] = computeCRC(VirtUart0ChannelBuffTx, VirtUart0ChannelTxSize);
  VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffTx, VirtUart0ChannelTxSize);

  /* initialize the input buffer and tmp coordinate */
  memset(ai_buffer_input, 0.0F , 28 *  28 * sizeof(float));
  touch_coordinate_x = 0xFF;
  touch_coordinate_y = 0xFF;
}

void process_received_message(uint8_t *BuffRx, uint16_t RxSize)
{
  uint8_t crc;
  log_dbg("%s: BuffRx[0]=0x%x RxSize=%d\n", __func__, BuffRx[0], RxSize);

  /* check if the frame message is one of expected */
  if (BuffRx[0] == 0x20 || BuffRx[0] == 0x21 || BuffRx[0] == 0x22) {
    /* only 3 type of messages are expected:
     * 0x20: specify input type
     * 0x21: touch screen coordinate
     * 0x22: start ai nn processing
     */
    /* check message integrity size and crc */
    if (RxSize == (BuffRx[1] + 3)) {
      crc = computeCRC(BuffRx, RxSize);
      if (BuffRx[RxSize-1] == crc)
        goto good_ack;
    }
  }

  /* send a wrong ack */
  memset(VirtUart0ChannelBuffTx, 0 , sizeof(VirtUart0ChannelBuffTx));
  VirtUart0ChannelTxSize = 3;
  VirtUart0ChannelBuffTx[0] = 0xFF;
  VirtUart0ChannelBuffTx[1] = 0;
  VirtUart0ChannelBuffTx[2] = computeCRC(VirtUart0ChannelBuffTx, VirtUart0ChannelTxSize);
  VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffTx, VirtUart0ChannelTxSize);
  return;

good_ack:
  if (BuffRx[0] == 0x20) {
    /* configure the input type */
    ai_input_type = BuffRx[2];

    /* send a good ack */
    memset(VirtUart0ChannelBuffTx, 0 , sizeof(VirtUart0ChannelBuffTx));
    VirtUart0ChannelTxSize = 3;
    VirtUart0ChannelBuffTx[0] = 0xF0;
    VirtUart0ChannelBuffTx[1] = 0;
    VirtUart0ChannelBuffTx[2] = computeCRC(VirtUart0ChannelBuffTx, VirtUart0ChannelTxSize);
    VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffTx, VirtUart0ChannelTxSize);
  } else if (BuffRx[0] == 0x21) {
    /* compute touch screen coordinate */
    int nb_coordinates = BuffRx[1] / 2;
    for (int i = 0; i < nb_coordinates; i++) {
      uint8_t x = BuffRx[2 + (i * 2)];
      uint8_t y = BuffRx[3 + (i * 2)];
      compute_touch_screen_coordinate(x, y);
    }

    /* send end of touch screen ack */
    memset(VirtUart0ChannelBuffTx, 0 , sizeof(VirtUart0ChannelBuffTx));
    VirtUart0ChannelTxSize = 4;
    VirtUart0ChannelBuffTx[0] = 0xF0;
    VirtUart0ChannelBuffTx[1] = 1;
    VirtUart0ChannelBuffTx[2] = nb_coordinates;
    VirtUart0ChannelBuffTx[3] = computeCRC(VirtUart0ChannelBuffTx, VirtUart0ChannelTxSize);
    VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffTx, VirtUart0ChannelTxSize);
  } else if (BuffRx[0] == 0x22) {
    /* start ai nn processing */
    start_ai_nn_processing();
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the system clock */
    SystemClock_Config();
  }
  /* USER CODE END Init */


  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  MX_IPCC_Init();
  MX_OPENAMP_Init(RPMSG_REMOTE, NULL);

  /* USER CODE BEGIN SysInit */
  log_info("Cortex-M4 @%iMHz boot successful with STM32Cube FW version: v%ld.%ld.%ld \r\n",
                                          (int)HAL_RCC_GetSystemCoreClockFreq()/1000000,
                                          ((HAL_GetHalVersion() >> 24) & 0x000000FF),
                                          ((HAL_GetHalVersion() >> 16) & 0x000000FF),
                                          ((HAL_GetHalVersion() >> 8) & 0x000000FF));
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_CRC2_Init();
  MX_X_CUBE_AI_Init();

  /* USER CODE BEGIN 2 */
  /* Create Virtual UART device defined by a rpmsg channel attached to the remote device */
  log_info("Virtual UART0 OpenAMP-rpmsg channel creation\r\n");
  if (VIRT_UART_Init(&huart0) != VIRT_UART_OK) {
    log_err("VIRT_UART_Init UART0 failed.\r\n");
    Error_Handler();
  }

  /* Need to register callback for message reception by channels*/
  if(VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, VIRT_UART0_RxCpltCallback) != VIRT_UART_OK) {
    Error_Handler();
  }

  /* Initialize the input buffer and touch coordinate */
  memset(ai_buffer_input, 0.0F , 28 *  28 * sizeof(float));
  touch_coordinate_x = 0xFF;
  touch_coordinate_y = 0xFF;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    OPENAMP_check_for_message();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (VirtUart0RxMsg) {
      VirtUart0RxMsg = RESET;
      process_received_message(VirtUart0ChannelBuffRx, VirtUart0ChannelRxSize);
    }

    if (VirtUart0TxMsg) {
      VirtUart0TxMsg = RESET;
      VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffTx, VirtUart0ChannelTxSize);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_PLLInitTypeDef pll1 = {0};
  RCC_PLLInitTypeDef pll2 = {0};
  RCC_PLLInitTypeDef pll3 = {0};
  RCC_PLLInitTypeDef pll4 = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**PLL1 Config 
  */
  pll1.PLLState = RCC_PLL_NONE;
  if (RCC_PLL1_Config(&pll1) != HAL_OK)
  {
    Error_Handler();
  }
  /**PLL2 Config 
  */
  pll2.PLLState = RCC_PLL_NONE;
  if (RCCEx_PLL2_Config(&pll2) != HAL_OK)
  {
    Error_Handler();
  }
  /**PLL3 Config 
  */
  pll3.PLLState = RCC_PLL_NONE;
  if (RCCEx_PLL3_Config(&pll3) != HAL_OK)
  {
    Error_Handler();
  }
  /**PLL4 Config 
  */
  pll4.PLLState = RCC_PLL_NONE;
  if (RCCEx_PLL4_Config(&pll4) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**RCC Clock Config 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                              |RCC_CLOCKTYPE_PCLK5|RCC_CLOCKTYPE_MPU;
  RCC_ClkInitStruct.MPUInit.MPU_Clock = RCC_MPUSOURCE_HSI;
  RCC_ClkInitStruct.MPUInit.MPU_Div = RCC_MPU_DIV2;
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_HSI;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_HSI;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV1;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV1;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.EthClockSelection = RCC_ETHCLKSOURCE_OFF;
  PeriphClkInit.CkperClockSelection = RCC_CKPERCLKSOURCE_OFF;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_OFF;
  PeriphClkInit.Lptim23ClockSelection = RCC_LPTIM23CLKSOURCE_OFF;
  PeriphClkInit.Lptim45ClockSelection = RCC_LPTIM45CLKSOURCE_OFF;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.TIMG1PresSelection = RCC_TIMG1PRES_DEACTIVATED;
  PeriphClkInit.TIMG2PresSelection = RCC_TIMG2PRES_DEACTIVATED;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IPPC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
     Error_Handler();
  }
}

/**
  * @brief CRC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC2_Init(void)
{

  /* USER CODE BEGIN CRC2_Init 0 */

  /* USER CODE END CRC2_Init 0 */

  /* USER CODE BEGIN CRC2_Init 1 */

  /* USER CODE END CRC2_Init 1 */
  hcrc2.Instance = CRC2;
  hcrc2.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc2.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc2.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc2.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc2.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC2_Init 2 */

  /* USER CODE END CRC2_Init 2 */

}

/* USER CODE BEGIN 4 */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{
    /* copy received msg in a variable to sent it back to master processor in main infinite loop*/
    VirtUart0ChannelRxSize = huart->RxXferSize < MAX_BUFFER_SIZE? huart->RxXferSize : MAX_BUFFER_SIZE-1;
    memcpy(VirtUart0ChannelBuffRx, huart->pRxBuffPtr, VirtUart0ChannelRxSize);
    VirtUart0RxMsg = SET;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
