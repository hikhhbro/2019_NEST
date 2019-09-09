/**
	|--------------------------------- Copyright --------------------------------|
	|                                                                            |
	|                      (C) Copyright 2019,海康平头哥,                         |
	|           1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China      |
	|                           All Rights Reserved                              |
	|                                                                            |
	|           By(GCU The wold of team | 华南理工大学广州学院机器人野狼队)         |
	|                    https://github.com/GCUWildwolfteam                      |
	|----------------------------------------------------------------------------|
	|--FileName    : hmi_usart.h                                                
	|--Version     : v1.0                                                            
	|--Author      : 海康平头哥                                                       
	|--Date        : 2019-08-07               
	|--Libsupports : 
	|--Description :                                                       
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|-----------------------------declaration of end-----------------------------|
 **/
#ifndef __HMI_USART_H 
#define __HMI_USART_H 
#include "bsp_usart.h"
/* -------------- 宏定义 ----------------- */
  #define PAGE_SIZE  11      //串口屏页面数
  #define BUFFER_DATA_HMI (10+HEAD_FRAME_LEN) //缓存数据
  #define LOCK_P	0xFF   //锁屏界面
  #define MAIN_P  0      //主界面
  #define P1_P    1     //基本要求1界面
  #define P2_P    2     //基本要求2界面
  #define P3_P    3     //基本要求3界面
  #define F1_P    4     //发挥部分1界面
  #define F2_P    5     //发挥部分2界面
  #define F3_P    6     //发挥部分3界面
  #define PAGINATION_ID ('p')//页面标识符
  #define STARTUP_ID    ('s')//启动发射标识符
  #define DISTANCE_ID   ('d')//距离标识符
  #define ANGEL_ID      ('a')//角度标识符
  #define VAL_END_ID    (';')//变量结束标识符 
  #define POINT_ID      ('.')//小数点标识符 
  #define FUHAO         ('-')
  #define START_FLAG 1  //启动标志位
/* -------------- 结构体 ----------------- */
	typedef struct hmiStrct
	{
    uint8_t status;
		uint8_t page;
    uint8_t data[BUFFER_DATA_HMI];
    uint8_t commot;
      uint8_t load;
    int16_t distance;
    float angle;
    fps_t fps;
	}hmiStrct;
  
	HAL_StatusTypeDef HmiUsartInit(hmiStrct* hmi);
  void HmiUsartParseData(hmiStrct* hmi);
#endif	// __HMI_USART_H
/*-----------------------------------file of end------------------------------*/


