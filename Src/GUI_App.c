  /**
  ******************************************************************************
  * @file    GUI_App.c
  * @author  MCD Application Team
  * @brief   Simple demo drawing "Hello world"  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright 漏 2018 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "GUI_App.h"
#include "GUI.h"

#include "DIALOG.h"
extern  WM_HWIN CreateWindow(void);  
  

void GRAPHICS_MainTask(void) {
  
  WM_HWIN h_GUI;

  /* 2- Create a Window using GUIBuilder */
  h_GUI=CreateWindow();
 
/* USER CODE BEGIN GRAPHICS_MainTask */
 /* User can implement his graphic application here */
  /* Hello Word example */
//    GUI_Clear();
//    GUI_SetColor(GUI_WHITE);
//    GUI_SetFont(&GUI_Font32_1);
//    GUI_DispStringAt("Hello world!", (LCD_GetXSize()-150)/2, (LCD_GetYSize()-20)/2);
  GUI_Clear();
  GUI_SetBkColor(GUI_BLACK);
  GUI_SetColor(GUI_WHITE);
  GUI_SetFont(&GUI_Font16_1);
  
  //extern int screen_protect;
  
  extern int counter;
  extern int close_flag;
  extern int compare_register1,compare_register2,compare_register3;
  extern int set_tem_low,set_tem_high,close_flag_range;
  extern int Temperature_now;
  extern int speed_mode;
  extern int mode_mode;
  extern int HZ,HX,TZ,TX;
  extern WM_HWIN hWin; 
  extern TIM_HandleTypeDef htim1;
  extern TIM_HandleTypeDef htim2;
  extern TIM_HandleTypeDef htim3;
  extern TIM_HandleTypeDef htim4;
  extern RTC_TimeTypeDef stimestructure;
  extern RTC_DateTypeDef sdatestructure;
  extern SPINBOX_Handle hdata;
  extern void Touchcheck(void);
  extern void timedate(void);
  extern void DHT11(void);
  extern void LED(void);
  extern void infrared(void);
  extern uint8_t ReadDisp_bmp();
  char *Speed[3]={"High","Middle","Low"};
  char *Mode[3]={"Cold","Warm","Wind"};
  while(1)
{
    if(counter<20)
    {
    WM_ShowWindow(h_GUI);
    /*触摸检测*/
    Touchcheck();


    /*显示工作模式*/
    GUI_Clear();
    
    /*显示姓名学号*/
    GUI_DispStringAt("17231126 GKH",360,0);
    GUI_DispStringAt("17211308 HGZ",360,15);
    
    /*显示当前温度湿度*/
    GUI_DispStringAt("Air conditional_temperature:", 0, 55);
    GUI_DispDecMin(Temperature_now);
    SPINBOX_SetValue(hdata,Temperature_now);
    GUI_DispStringAt("Speed:", 200, 55);
    GUI_DispString(Speed[speed_mode]);
    GUI_DispStringAt("Mode:", 300, 55);
    GUI_DispString(Mode[mode_mode]);
    

    /*显示时间*/
    timedate();
    GUI_DispStringAt("", 0, 0);
    GUI_DispDecMin(2020+sdatestructure.Year);
    GUI_DispString("-");
    GUI_DispDecMin(sdatestructure.Month);
    GUI_DispString("-");
    GUI_DispDecMin(sdatestructure.Date);
    GUI_DispString("  ");
    GUI_DispDecMin(stimestructure.Hours);
    GUI_DispString(":");
    GUI_DispDecMin(stimestructure.Minutes);
    GUI_DispString(":");
    GUI_DispDecMin(stimestructure.Seconds);
    

    /*温度测量*/
    if(__HAL_TIM_GET_COUNTER(&htim4) > 19999)
    {
      DHT11();
      __HAL_TIM_SET_COUNTER(&htim4, 0);
    }
    GUI_DispStringAt("Temperature_now:", 0, 25);
    GUI_DispDecMin(TZ);
    GUI_DispString(".");
    GUI_DispDecMin(TX);
    GUI_DispString(" OC  ");
    GUI_DispString("Humidity_now:");
    GUI_DispDecMin(HZ);
    GUI_DispString(".");
    GUI_DispDecMin(HX);
    GUI_DispString("%");

    
    
    /*开关机的实现函数*/
    if(close_flag == 0 && close_flag_range == 0 )
    {
      LED();
    }
    else if(close_flag_range == 1)//预约温度开关机函数
    {
      HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
      if(TZ <= set_tem_low )
      {
        mode_mode = 1;
        close_flag_range=0;
        HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);
      }
      if(TZ >= set_tem_high )
      {
        mode_mode = 0;
        close_flag_range=0;
        HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);
      }
    }
    else if(close_flag == 1)//关机
    {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
    
    /*延时100ms*/
    GUI_Delay(100);
  }
  if(counter>=20)//大于20s未操作即进入屏保程序
  {
    WM_HideWindow(h_GUI);
    Touchcheck();
    if(((counter/10)%2) == 0)
    ReadDisp_bmp("pb.bmp");//显示图片1
    else
    ReadDisp_bmp("pb2.bmp");//显示图片2
    /*延时1ms*/
    GUI_Delay(1);
  }
    
     
}
   
/* USER CODE END GRAPHICS_MainTask */
  while(1)
{
      GUI_Delay(100);
}
}

/*************************** End of file ****************************/
