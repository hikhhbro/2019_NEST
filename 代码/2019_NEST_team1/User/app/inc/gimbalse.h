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
	|--FileName    : gimbal.h                                                
	|--Version     : v1.0                                                            
	|--Author      : 海康平头哥                                                       
	|--Date        : 2019-01-26               
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
#ifndef __GIMBALL_H 
#define __GIMBALL_H 
#include "DataStructure.h" 
#include "gyro.h" 
#include "filter.h" 
#include "DJI_dbus.h" 
#include "Servo.h"
#include "leds_tip.h" 
#include "hmi_usart.h"
#include "pc_data.h" 
#define LINE_PRE  20
#define X 0
#define Y 1
#define PI 3.14159
#define  POLES 4096.0
#define  DISTANCE  100.0
#define TWO_PIONT    0xFF
#define MIN_DISTANCE 1999 //mm
#define MAX_DISTANCE  3001//mm
#define MID_DISTANCE  1928   //舵机线数
#define START_DIS_ROLL 1658// mm//水平角度
#define ERROR_DISTANCE 0    //错误距离
#define MIN_ANGLE (-32.0f) //度 //1848
#define MAX_ANGLE  (32.0f)//度   //2531  中间值2190           
#define ERROR_ANGLE (40.0f)//错误角度
extern float ex_real[2];
	void GimbalInitl(const dbusStruct* pRc_t);
  typedef struct
  {
    float pointx[LINE_PRE];
    float pointy[LINE_PRE];
    float pointx_t[100];
    float pointy_t[100];
    float a;
    float b;
    float l;
  }LineStruct;
 typedef struct
  {
    float pointx[LINE_PRE];
    float pointy[LINE_PRE];
    float a;
    float b;
  }CircleStruct;
  uint8_t CalLine(LineStruct* pLine_t,int16_t r_x,int16_t r_y,int16_t t_x,int16_t t_y,float k);
 float ServoToAngle(float val);
   int16_t MarkToServo(float val);
 void SetReal(int16_t dis ,float angle);
  float JSFWJ(float vx,float vy);
   float RadianToAngle(float alfa);
   int AngleToLen(float Angle);
  	void GimbalControl(const dbusStruct* pRc_t);
 int LenToAngle(int16_t Len);
int ChooseMode(const dbusStruct* pRc_t);
   void DrawTheCircle(float d);
  void ShowLocation(int speed);
   uint8_t CalLinelEEE(LineStruct* pLine_t,int16_t r_x,int16_t r_y,int16_t t_x,int16_t t_y);
  
  
uint8_t ConfirmTheLaunch(hmiStrct *phmi);
	 int16_t GetDistance(int16_t dis);
	float GetAngle(float angle);
int16_t AngleToServo(float val); 
float OpenmvToAngle(int16_t val); 
void Loading(hmiStrct *phmi) ; 
#endif // __GIMBAL_H
/*-----------------------------------file of end------------------------------*/






