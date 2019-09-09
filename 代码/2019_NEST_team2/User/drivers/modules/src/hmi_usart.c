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
	|--FileName    : hmi_usart.c                                              
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
#include "hmi_usart.h" 
/**
	* @Data    2019-08-07 23:19
	* @brief   串口屏初始化
	* @param   串口结构体地址
	* @retval  HAL Status 
	*/
	HAL_StatusTypeDef HmiUsartInit(hmiStrct* hmi)
	{
      hmi->status = MOD_READ;
      if(hmi == NULL)
      {
        return HAL_ERROR;
      }
      memset(hmi->data,0,BUFFER_DATA_HMI);
      hmi->page = 0;
      hmi->commot = 0;
      hmi->angle = 0;
      hmi->distance = 0;
      if(UsartAndDMAInit(HMI_USART,BUFFER_DATA_HMI,ENABLE) != HAL_OK)
      {
        //报错机制
        return HAL_ERROR;
      }
      SetFpsAddress(hmi->fps);
      SET_BIT(hmi->status,INIT_OK);
      return HAL_OK;
	}
/**
	* @Data    2019-08-07 23:37
	* @brief   串口屏数据解析
	* @param   串口结构体地址
	* @retval  void
	*/
	void HmiUsartParseData(hmiStrct* hmi)
	{
    uint8_t j=0,q=0,po=0;
    int16_t sdsf = 0;
		if(UserUsartQueueRX(HMI_USART,hmi->data) == HAL_OK)
    {
      
      for(int i=DATA_LEN_BYTE_LEN;i<BUFFER_DATA_HMI;)
      {
         if(hmi->data[i] ==PAGINATION_ID)//页数标识符号
         {
           hmi->page = hmi->data[i+1]&0x0f;
           i = i+2;     
         }
         if(hmi->data[i] == DISTANCE_ID)//距离
         {
           for(j = i;hmi->data[j]!= VAL_END_ID;j++)
           {
             if(j-i>5)
             hmi->data[j]= VAL_END_ID; 
           } 
           j=j-i-1;
           hmi->distance = 0;
           for(;j>0;j--)
           {
             i++;
             hmi->distance +=  (hmi->data[i]&0x0f) * pow(10,(j-1));
           }
         }
         if(hmi->data[i] == ANGEL_ID)//角度
         {
           for(j = i;hmi->data[j]!= VAL_END_ID;j++)
           {
             if(j-i>5)
             hmi->data[j]= VAL_END_ID; 
           } 
          if(hmi->data[i+1] == FUHAO) 
          {
             i++;
            sdsf = 1;
          } 
           j=j-i-1;
           hmi->angle = 0;
           q=0;
           po =j;
           for(;j>0;j--)
           { 
             i++;             
             if(hmi->data[i]== POINT_ID)
             {              
               q = po - j;
               i++;                           
             }                                          
             hmi->angle +=  (hmi->data[i]&0x0f) * pow(10,(j-1));
             if(q!=0&&j==2)
               j=1;
           }
           if(q==1)
             hmi->angle = hmi->angle/ pow(10,q+1); 
           else
           {
               hmi->angle = hmi->angle/ pow(10,q); 
           }
           if(sdsf ==1)
           {
              hmi->angle = -hmi->angle;
           }
         
         }
         if(hmi->data[i] ==STARTUP_ID)//启动
         {
           hmi->commot =  START_FLAG;
           i++;
         }
        if(hmi->data[i] =='l')//启动
         {
           hmi->load =  1;
           i++;
         }
         else i++;
      }
    }
    else if(GetFps(hmi->fps) ==0)//没数据或严重掉帧
    {
    //  hmi->commot =0;
//      hmi->angle  =0;
//      hmi->distance = 0;      
    }
	}
/*-----------------------------------file of end------------------------------*/


