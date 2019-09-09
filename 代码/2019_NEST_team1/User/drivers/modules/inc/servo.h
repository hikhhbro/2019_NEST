/**
|-------------------------------- Copyright -----------------------------------|
|                                                                              |
|                        (C) Copyright 2019, Daqo Lee,                         |
|                                                                              |
|                            By:GCU The wold of team                           |
|                         https://github.com/GCUWildwolfteam                   |
|------------------------------------------------------------------------------|
|--FileName    : servo.h                                                
|--Version     : v1.0                                                            
|--Author      : Daqo Lee                                                       
|--Date        : 2019-01-10               
|--Libsupports : STM32F4xx_DFP ( 2.9.0)
|--Description :                                                       
|---------------------------------declaration of end----------------------------|
 **/
#ifndef __SERVO_H 
#define __SERVO_H 
/*--------------------- I N C L U D E - F I L E S ----------------------------*/
#include "baseclass.h" 

/*-------------------------G L O B A L - T Y P E S----------------------------*/
typedef enum
{
  USART_1=1,
  USART_2=2,
  USART_3=3,
  UART_4 =4,
  UART_5 =5,
  USART_6=6,
  UART_7 =7,
  UART_8 =8,
  
}USARTx_e;
   typedef struct
{
  USARTx_e USART_x;
  uint16_t crc;
  uint8_t *pUSARTSendBuff;
}USARTSend_t;
typedef struct 
{
    uint8_t   ID;
    uint32_t  StartAddr;
    uint16_t  Length;
    uint8_t   Error;
    uint8_t*  p_ucTable;
} SyncBulkData_t;

typedef struct 
{
    int32_t ID;
    int32_t ModelNo;
    int32_t FirmVer;
} PingData_t;

typedef struct
{
    uint8_t ID;
    uint8_t Error;

    uint16_t targetAngle;
    uint16_t targetSpeed;
    uint16_t targetAcc;

    uint16_t tarMaxAngLimit;
    uint16_t tarMiniAngLimit;

    uint16_t realAngle;
    uint16_t realSpeed;
    uint16_t realAcc;

}DXL_t;

typedef struct
{
    uint8_t ID;
    uint8_t Error;

    uint16_t targetAngle;
    uint16_t targetSpeed;

    uint16_t tarMaxAngLimit;
    uint16_t tarMiniAngLimit;

    uint16_t realAngle;
    uint16_t realSpeed;

}SMS_t;

typedef struct
{
	uint8_t DxlBuff[20];
	uint8_t SmsBuff[20];
	
	DXL_t MX_64[8];
	SMS_t SM_60[8];
	
}DigitalServo_t;
/*------------------------G L O B A L - M A C R O S --------------------------*/
#define MAX_ID				(252)
#define BROADCAST_ID		(254)  //BroadCast ID

#define PING          0x01
#define READ          0x02
#define WRITE         0x03
#define REG_WRITE     0x04
#define ACTION        0x05
#define FACTORY_REST  0x06
#define REBOOT        0x08
#define STATUS        0x55
#define SYNC_READ     0x82
#define SYNC_WRITE    0x83
#define BULK_READ     0x92
#define BULK_WRITE    0x93 

#define SPEED         0x20
#define POSITION      0x1E
#define ACC           0x49

#define CW_ANGLE_LIMIT   0x06
#define CCW_ANGLE_LIMIT  0x08
/*----------------------G L O B A L - D E F I N E S---------------------------*/

extern DigitalServo_t DigitalServo;

/*-----------G L O B A L - F U N C T I O N S - P R O T O T Y P E S------------*/

void DXL2_setMassage(UART_HandleTypeDef *huart,uint8_t ID, uint16_t Length, 
                                     uint8_t Cmd, uint8_t *Data);
void DXL1_setMassage(UART_HandleTypeDef *huart,uint8_t ID, uint16_t Length,
                                     uint8_t Cmd, uint8_t *Data);
void DXL1_setTargetAngle(UART_HandleTypeDef *huart,uint8_t ID, uint8_t Cmd,  uint16_t Data);
void DXL1_getMassage(uint8_t *DXLBuffer);
void DXL1_setSyncTarAng(UART_HandleTypeDef *huart,uint8_t Num,...);
void DXL1_setSyncMsg(UART_HandleTypeDef *huart,uint8_t Addr,uint8_t Num,...);
void SMS1_setTargetAngle(UART_HandleTypeDef *huart,uint8_t ID, uint8_t Cmd, uint16_t Position);
#endif	// __SERVO_H
/*----------------------------------FILE OF END-------------------------------*/














 

