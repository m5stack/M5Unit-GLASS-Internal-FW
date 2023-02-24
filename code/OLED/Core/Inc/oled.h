#ifndef __OLED_H
#define __OLED_H 

// #include "sys.h"
#include "stdlib.h"	
#include "main.h"

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

//-----------------测试LED端口定义---------------- 

// #define LED_ON GPIO_ResetBits(GPIOC,GPIO_Pin_12)
// #define LED_OFF GPIO_SetBits(GPIOC,GPIO_Pin_12)

//-----------------OLED端口定义---------------- 

// #define OLED_SCL_Clr() HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET)
// #define OLED_SCL_Set() HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET)
#define OLED_SCL_Clr() GPIOA->BRR = SCK_Pin
#define OLED_SCL_Set() GPIOA->BSRR = SCK_Pin

// #define OLED_SDA_Clr() HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET)
// #define OLED_SDA_Set() HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET)
#define OLED_SDA_Clr() GPIOA->BRR = MOSI_Pin
#define OLED_SDA_Set() GPIOA->BSRR = MOSI_Pin

// #define OLED_RES_Clr() HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET)
// #define OLED_RES_Set() HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET)
#define OLED_RES_Clr() GPIOA->BRR = RESET_Pin
#define OLED_RES_Set() GPIOA->BSRR = RESET_Pin

// #define OLED_DC_Clr()  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET)
// #define OLED_DC_Set()  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET)
#define OLED_DC_Clr()  GPIOA->BRR = DC_Pin
#define OLED_DC_Set()  GPIOA->BSRR = DC_Pin

// #define OLED_CS_Clr()  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
// #define OLED_CS_Set()  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)
#define OLED_CS_Clr()  GPIOA->BRR = CS_Pin
#define OLED_CS_Set()  GPIOA->BSRR = CS_Pin


#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

void OLED_ClearPoint(u8 x,u8 y);
void OLED_ColorTurn(u8 i);
void OLED_DisplayTurn(u8 i);
void OLED_WR_Byte(u8 dat,u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);
void OLED_DrawCircle(u8 x,u8 y,u8 r, u8 mode);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,u8 BMP[],u8 mode);
void OLED_Init(void);

#endif

