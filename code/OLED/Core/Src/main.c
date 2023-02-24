/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_ex.h"
#include "oled.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint8_t pos_x;      // stack space top
  uint8_t pos_y;    // stack space limit
  uint8_t size;          // current stack pointer
  uint8_t mode;          // current stack pointer
} draw_text_option_t;
typedef struct {
  uint8_t pos_x;      // stack space top
  uint8_t pos_y;    // stack space limit
  uint8_t mode;          // current stack pointer
} draw_point_option_t;
typedef struct {
  uint8_t pos_x1;      // stack space top
  uint8_t pos_y1;    // stack space limit
  uint8_t pos_x2;
  uint8_t pos_y2;
  uint8_t mode;          // current stack pointer
} draw_line_option_t;
typedef struct {
  uint8_t pos_x;      // stack space top
  uint8_t pos_y;    // stack space limit
  uint8_t radius;          // current stack pointer
  uint8_t mode;          // current stack pointer
} draw_circle_option_t;
typedef struct {
  uint8_t pos_x;      // stack space top
  uint8_t pos_y;    // stack space limit
  uint8_t size_x;      // stack space top
  uint8_t size_y;    // stack space limit
  uint8_t mode;          // current stack pointer
} draw_picture_option_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION 3
#define BUTTON_FILTER 500
#define BUTTON_FILTER_TIMEROUT BUTTON_FILTER*3
#define APPLICATION_ADDRESS     ((uint32_t)0x08001000) 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
volatile uint8_t fm_version = FIRMWARE_VERSION;
draw_text_option_t draw_text_para;
draw_point_option_t draw_point_para;
draw_line_option_t draw_line_para;
draw_circle_option_t draw_circle_para;
draw_picture_option_t draw_picture_para;
uint8_t invert_status = 0;
uint8_t color_invert_status = 0;
uint8_t display_on_off_status = 1;
uint8_t string_buffer[64] = {0};
uint8_t picture_buffer[1088] = {0};
uint8_t oled_clear_flag = 0;
uint8_t oled_show_flag = 0;
uint8_t oled_set_string_flag = 0;
//buzz mannual control
volatile uint16_t buzz_freq = 0;
volatile uint8_t buzz_duty = 0;
volatile uint8_t buzz_manual_flag = 0;
volatile uint8_t buzz_enable = 0;

volatile uint8_t key_l_status = 0;
volatile uint8_t key_r_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);
	//复制中断向量表到SRAM首地址
	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  //重映射 SRAM 地址到 0x00000000
  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

uint8_t read_button_status(uint16_t pin)
{
  uint8_t button_status = 0; 
  uint8_t last_button_status = 0;
  uint16_t counter = 0;

  last_button_status = HAL_GPIO_ReadPin(GPIOA,pin);
  for (uint16_t i = 0; i < BUTTON_FILTER_TIMEROUT; i++) {  
    button_status = HAL_GPIO_ReadPin(GPIOA,pin);
    if (button_status == last_button_status) {
      counter++;
    }
    else {
      last_button_status = button_status;
      counter = 0;
    }
    if (counter >= BUTTON_FILTER) {
      return button_status;
    }
  }
  //TODO: Just for debug
  return 1;  
}

void set_pwm(uint16_t freq, uint8_t duty)
{
  if (freq > 10000)
    return;
  uint32_t period, pulse;
  period = 1000000 / freq;
  pulse = (uint32_t)((float)duty/255.0f*period);
  LL_TIM_SetAutoReload(TIM3, period);
  LL_TIM_OC_SetCompareCH4(TIM3, pulse); 
  // user_tim3_init(period, pulse);
}

void i2c1_receive_callback(uint8_t *rx_data, uint16_t len)
{
  uint8_t rx_buf[16];
  uint8_t tx_buf[16];
  uint8_t rx_mark[16] = {0};
  uint16_t buffer_index = 0;

  if (len > 1) {
    if (rx_data[0] == 0x00) {
      if ((rx_data[1] & 1) == 1) {
        oled_clear_flag = 1;
      }
    }
    else if (rx_data[0] == 0x10) {
      if ((rx_data[1] & 1) == 1) {
        oled_show_flag = 1;
      }      
    }
    else if ((rx_data[0] >= 0x20) && (rx_data[0] <= 0x23)) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x20+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x20+i] = 1;     
      } 
      if (rx_mark[0]) {
        draw_text_para.pos_x = rx_buf[0];
      }
      if (rx_mark[1]) {
        draw_text_para.pos_y = rx_buf[1];
      }
      if (rx_mark[2]) {
        draw_text_para.size = rx_buf[2];
      }
      if (rx_mark[3]) {
        draw_text_para.mode = rx_buf[3];
      }
      oled_set_string_flag = 1;
    }
    else if ((rx_data[0] >= 0x30) && (rx_data[0] <= 0x32)) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x30+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x30+i] = 1;     
      }    
      if (rx_mark[0]) {
        draw_point_para.pos_x = rx_buf[0];
      }
      if (rx_mark[1]) {
        draw_point_para.pos_y = rx_buf[1];
      }
      if (rx_mark[2]) {
        draw_point_para.mode = rx_buf[2];
      }
      OLED_DrawPoint(draw_point_para.pos_x, draw_point_para.pos_y, draw_point_para.mode);
    } 
    else if ((rx_data[0] >= 0x40) && (rx_data[0] <= 0x44)) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x40+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x40+i] = 1;     
      }    
      if (rx_mark[0]) {
        draw_line_para.pos_x1 = rx_buf[0];
      }
      if (rx_mark[1]) {
        draw_line_para.pos_y1 = rx_buf[1];
      }
      if (rx_mark[2]) {
        draw_line_para.pos_x2 = rx_buf[2];
      }
      if (rx_mark[3]) {
        draw_line_para.pos_y2 = rx_buf[3];
      }
      if (rx_mark[4]) {
        draw_line_para.mode = rx_buf[4];
      }
      OLED_DrawLine(draw_line_para.pos_x1, draw_line_para.pos_y1, 
      draw_line_para.pos_x2, draw_line_para.pos_y2, draw_line_para.mode);
    } 
    else if ((rx_data[0] >= 0x50) && (rx_data[0] <= 0x53)) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x50+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x50+i] = 1;     
      }    
      if (rx_mark[0]) {
        draw_circle_para.pos_x = rx_buf[0];
      }
      if (rx_mark[1]) {
        draw_circle_para.pos_y = rx_buf[1];
      }
      if (rx_mark[2]) {
        draw_circle_para.radius = rx_buf[2];
      }
      if (rx_mark[3]) {
        if (rx_buf[3])
          draw_circle_para.mode = 1;
        else
          draw_circle_para.mode = 0;
      }
      OLED_DrawCircle(draw_circle_para.pos_x, draw_circle_para.pos_y, draw_circle_para.radius, draw_circle_para.mode);
    }
    else if (rx_data[0] == 0x60) {
      if (rx_data[1] <= 3) {
        invert_status = rx_data[1];
        OLED_DisplayTurn(invert_status);
      }
    }
    else if (rx_data[0] == 0x70) {
      if ((rx_data[1] & 1) == 1) {
        display_on_off_status = 1;
        OLED_DisPlay_On();
      } else {
        display_on_off_status = 0;
        OLED_DisPlay_Off();
      }
    }
    else if ((rx_data[0] >= 0x80) && (rx_data[0] <= 0x82)) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x80+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x80+i] = 1;     
      }    
      if (rx_mark[0] && rx_mark[1]) {
        buffer_index = (rx_buf[0] | (rx_buf[1] << 8));
      }
      if (buffer_index == 0) {
        memset(string_buffer, 0, sizeof(string_buffer));
      }
      if (rx_mark[2]) {
        if (buffer_index < 64) {
          string_buffer[buffer_index] = rx_buf[2];
        }
      }
    }    
    else if ((rx_data[0] >= 0x90) && (rx_data[0] <= 0x92)) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x90+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x90+i] = 1;     
      }    
      if (rx_mark[0] && rx_mark[1]) {
        buffer_index = (rx_buf[0] | (rx_buf[1] << 8));
      }
      if (buffer_index == 0) {
        memset(picture_buffer, 0, sizeof(picture_buffer));
      }      
      if (rx_mark[2]) {
        if (buffer_index < 1088) {
          picture_buffer[buffer_index] = rx_buf[2];
        }
      }
    } 
    else if (rx_data[0] == 0xA0) {
      if ((rx_data[1] & 1) == 1) {
        color_invert_status = 1;
        OLED_ColorTurn(1);
      } else {
        color_invert_status = 0;
        OLED_ColorTurn(0);
      }
    }       
    else if ((rx_data[0] >= 0xB0) && (rx_data[0] <= 0xB4)) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0xB0+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0xB0+i] = 1;     
      }    
      if (rx_mark[0]) {
        draw_picture_para.pos_x = rx_buf[0];
      }
      if (rx_mark[1]) {
        draw_picture_para.pos_y = rx_buf[1];
      }
      if (rx_mark[2]) {
        draw_picture_para.size_x = rx_buf[2];
      }
      if (rx_mark[3]) {
        draw_picture_para.size_y = rx_buf[3];
      }
      if (rx_mark[4]) {
        draw_picture_para.mode = rx_buf[4];
      }
      OLED_ShowPicture(draw_picture_para.pos_x, draw_picture_para.pos_y, 
      draw_picture_para.size_x, draw_picture_para.size_y, picture_buffer, draw_picture_para.mode);
    }  
    else if ((rx_data[0] >= 0xC0) && (rx_data[0] <= 0xC3))
    {
      if (len <= 5) {
        for(int i = 0; i < len - 1; i++) {
          rx_buf[rx_data[0]-0xC0+i] = rx_data[1+i];
          rx_mark[rx_data[0]-0xC0+i] = 1;     
        }
        if (rx_mark[0] && rx_mark[1]) {
          buzz_freq = (rx_buf[0] | (rx_buf[1] << 8));
        }
        if (rx_mark[2]) {
          buzz_duty = rx_buf[2];
        }
        if (rx_mark[3]) {
          
          buzz_enable = rx_buf[3];
          if (buzz_enable) {
            set_pwm(buzz_freq, buzz_duty); 
            LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH4);
            LL_TIM_EnableCounter(TIM3);
            LL_TIM_EnableAllOutputs(TIM3);
            buzz_manual_flag = 1;
          } else {
            LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH4);
            LL_TIM_DisableCounter(TIM3);
            LL_TIM_DisableAllOutputs(TIM3);
            buzz_manual_flag = 0;
          }
        }
      }
    }                 
  } else if (len == 1) {
    if (rx_data[0] == 0xD0)
    {
      key_l_status = read_button_status(KEY_L_Pin);
      i2c1_set_send_data((uint8_t *)&key_l_status, 1);
    }     
    else if (rx_data[0] == 0xD1)
    {
      key_r_status = read_button_status(KEY_R_Pin);
      i2c1_set_send_data((uint8_t *)&key_r_status, 1);
    }
    else if (rx_data[0] == 0xFE)
    {
      i2c1_set_send_data((uint8_t *)&fm_version, 1);
    }         
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
  IAP_Set();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  // OLED_DisPlay_Off();
  // HAL_Delay(1000);
  OLED_ColorTurn(0);//0正常显示，1 反色显示
	OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
  // set_i2c_slave_address(0x3D);
  // i2c1_it_enable(); 
  HAL_I2C_EnableListen_IT(&hi2c1);
  // string_buffer[0] = 'h';
  // string_buffer[1] = 'e';
  // string_buffer[2] = 'l';
  // string_buffer[3] = 'l';
  // string_buffer[4] = 'o';
  // OLED_DisplayTurn(0);
  // OLED_ColorTurn(0);
  // OLED_DisPlay_On();
  // OLED_Clear();
  // OLED_Refresh();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (oled_clear_flag) {
      // LL_I2C_DisableIT_ADDR(I2C1);
      OLED_Clear();
      oled_clear_flag = 0;
      // LL_I2C_EnableIT_ADDR(I2C1);
    } else if (oled_set_string_flag){
      OLED_ShowString(draw_text_para.pos_x, draw_text_para.pos_y, string_buffer, draw_text_para.size, draw_text_para.mode);
      oled_set_string_flag = 0;
    } else if (oled_show_flag) {
      // LL_I2C_DisableIT_ADDR(I2C1);
      OLED_Refresh();
      oled_show_flag = 0;
      // LL_I2C_EnableIT_ADDR(I2C1);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
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
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0x3D<<1;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 48-LL_TIM_IC_FILTER_FDIV1_N2;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 250;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration
  PB1   ------> TIM3_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|RESET_Pin|SCK_Pin|DC_Pin
                          |MOSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY_L_Pin KEY_R_Pin */
  GPIO_InitStruct.Pin = KEY_L_Pin|KEY_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_Pin SCK_Pin DC_Pin MOSI_Pin */
  GPIO_InitStruct.Pin = RESET_Pin|SCK_Pin|DC_Pin|MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
