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
	|--FileName    : gimbal.c                                              
	|--Version     : v2.0                                                          
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
#include "gimbalse.h" 
#include <Math.h>
  extern osThreadId startGimbalTaskHandle;
  extern pcDataStruct pc_t;//小电脑数据
  extern  hmiStrct hmi_t;    //串口屏
  #define X_Z 2037
  #define Y_Z 1859
  uint8_t ex_end = 255;
  float ex_real[2] ={0};
  int real_z[2] ={0};
  int16_t ex_distance=0,ex_angle = 0;//距离的舵机线数,角度的舵机线数
  uint8_t dddd[10];
	/**
	* @Data    2019-01-27 17:09
	* @brief   云台初始化
	* @param  
	* @retval  void
	*/
     int mode=0;
	void GimbalInitl(const dbusStruct* pRc_t)
	{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
    SetReal(START_DIS_ROLL,0);//更新真实值
        /* -------- 挂起等待任务系统初始化 --------- */
	   vTaskSuspend(startGimbalTaskHandle);
	}
 /**
	* @Data    2019-01-27 17:09
	* @brief   云台控制
	* @param  
	* @retval  void
	*/
  int16_t r_distance =MIN_DISTANCE;//真实长度
  float  r_angle = MIN_ANGLE;//真实角度
  float angle_temp1 =0;
  int16_t flag1=0,flag2=0,flag3=0,flag4=0,flag5=0,flag6=0,flag7=0,flag8=5,flag9 =0,flag17 =0;
  float hos_an[10];
  int16_t hos_dis[10];
  float angle_c=2,angle_c1 = 30;
  int16_t xxx = 2060,yyy =1520;
  float temmmm=0;int16_t saf=0;
  int16_t dis_temp=0;
  float angle_temp = 0;
  float angle_temp2 = 0;
  int16_t dell = 100,dell1= 400;
  int16_t dd = 200;
  uint8_t end_id_f = 255;
  float roll = 0;
  int16_t delay1=50;
  float en_angle = 0.07;
  float   fp1 =       0.844,fp2 =862.3 ;
	void GimbalControl(const dbusStruct* pRc_t)
	{

    switch(hmi_t.page)
    {
      case MAIN_P:  //主屏幕
        angle_temp = 0;//清楚临时角度值
        dis_temp = 0;
        hmi_t.commot = 0;//清楚发射标志
        SetReal(START_DIS_ROLL,0);
        // DXL1_setSyncTarAng(&huart2,2,0x00,xxx,0x01,yyy);
        break;
      case P1_P: //	电磁炮能够将弹丸射出炮口 
        Loading(&hmi_t);   //装弹
        DXL1_setSyncTarAng(&huart2,2,0x00,xxx,0x01,yyy);        
        ConfirmTheLaunch(&hmi_t);//确认发送
        break;
      case P2_P: //	定距离发射         
        Loading(&hmi_t);   //装弹
        dis_temp = GetDistance(hmi_t.distance);//获取距离
        angle_temp = GetAngle(0);//获取角度
        if(dis_temp!= ERROR_DISTANCE&&hmi_t.commot==1)
        {
          SetReal(dis_temp,angle_temp);
          osDelay(800);
          ConfirmTheLaunch(&hmi_t);//确认发送
        }       
        printf("n0.val=%d%c%c%c",hmi_t.distance,end_id_f,end_id_f,end_id_f);
        break;
      case P3_P: //	定距离角度发射
        Loading(&hmi_t);   //装弹
        dis_temp = GetDistance(hmi_t.distance);//获取距离
        angle_temp = GetAngle(hmi_t.angle);//获取角度
        if(dis_temp!= ERROR_DISTANCE&&angle_temp != ERROR_ANGLE&&hmi_t.commot==1)
        {
           SetReal(dis_temp,angle_temp);
           osDelay(800);
           ConfirmTheLaunch(&hmi_t);//确认发送
        }
        printf("n0.val=%d%c%c%c",hmi_t.distance,end_id_f,end_id_f,end_id_f);
        printf("t8.txt=\"%.2f\"%c%c%c",hmi_t.angle,end_id_f,end_id_f,end_id_f);        
        break;
      case F1_P://指定任意位置发射
        if(hmi_t.commot ==1)  //启动
        {
          if(pc_t.shoot_commot ==1) //发现目标并追踪
          {
            osDelay(dell);
            angle_temp2 = GetAngle(r_angle - OpenmvToAngle(pc_t.yaw_target_angle));//获取角度
            SetReal(START_DIS_ROLL,angle_temp2); 
            osDelay(dell1);
            if((flag3<flag8) && (IS_CLOSE_INT(pc_t.yaw_target_angle,-angle_c,angle_c)))
            {
              flag3++;
            }
            else if(flag3 >=flag8-1)
            {
              dis_temp = GetDistance((int16_t)(pc_t.distance));//获取距离
              osDelay(1000);
              SetReal(dis_temp,angle_temp2); 
              osDelay(500);
              flag3=0;
              ConfirmTheLaunch(&hmi_t);//确认发送
              osDelay(500);
            }                                 
          }
          else
          {
            if(flag17==0)
            {
              if(angle_temp>=30)
              flag17=1;
              else angle_temp = angle_temp+en_angle;
            }
            if(flag17==1)
            {
              if(angle_temp<=-30)
              flag17=0;
              else angle_temp = angle_temp-en_angle;
            }
            angle_temp = GetAngle(angle_temp);//获取角度
            dis_temp = GetDistance(pc_t.distance);//获取距离 
            if(angle_temp != ERROR_ANGLE)
            {
              SetReal(r_distance,angle_temp);
            }
          }
        }
        else
        {
          osDelay(1000);
          SetReal(START_DIS_ROLL,0); 
          Loading(&hmi_t);   //装弹                 
        }

        break;
      case F2_P://寻找目标发射     
          if(hmi_t.commot == START_FLAG)
          {
            if(flag5==0)
            {
              if(angle_temp>=30)
              flag5=1;
              else angle_temp = angle_temp+en_angle;
            }
            if(flag5==1)
            {
              if(angle_temp<=-30)
              flag5=0;
              else angle_temp = angle_temp-en_angle;
            }
            angle_temp = GetAngle(angle_temp);//获取角度
            dis_temp = GetDistance(pc_t.distance);//获取距离 
            if(angle_temp != ERROR_ANGLE)
            {
              SetReal(r_distance,angle_temp);
            }
            if(pc_t.shoot_commot ==1)
            {
              SetReal(dis_temp,angle_temp);
              if(IS_CLOSE_INT(pc_t.yaw_target_angle,-angle_c1,angle_c1))
              {
                ConfirmTheLaunch(&hmi_t); 
                osDelay(500);
              }
            }           
          } 
          else
          {
            Loading(&hmi_t);   //装弹 
            SetReal(START_DIS_ROLL,-30); 
            angle_temp=-30;
          }
        roll =  (ex_distance/POLES )* 360.0f-133.59f;
        printf("n1.val=%d%c%c%c",hmi_t.distance,end_id_f,end_id_f,end_id_f);
        printf("t8.txt=\"%.2f\"%c%c%c",r_angle,end_id_f,end_id_f,end_id_f);
        printf("t5.txt=\"%.2f\"%c%c%c",roll,end_id_f,end_id_f,end_id_f);          
        break;
    case F3_P:  //其他
        Loading(&hmi_t);   //装弹 
        if(IS_OPEN_INT(hmi_t.distance,1999,2501))
        {
          dis_temp =  fp1*hmi_t.distance+fp2;
          SetReal(dis_temp,0); 
          osDelay(800);
        }
        ConfirmTheLaunch(&hmi_t); 
        printf("n0.val=%d%c%c%c",dis_temp,end_id_f,end_id_f,end_id_f);
     break;       
    }    
	}
/* -------------- 国赛----------------- */   
/**
	* @Data    2019-08-08 09:44
	* @brief   确认发射弹丸
	* @param   串口屏地址
	* @retval  void
	*/
  int16_t s_lim=5;
	uint8_t ConfirmTheLaunch(hmiStrct *phmi)   
	{
    if(phmi->commot == START_FLAG)
    {
      phmi->commot = 0;//清楚发射标志
      //添加发射信息
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  //高电平        
      osDelay(s_lim);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 
      return 1;       
    } 
    return 0;     
	}
 /**
	 * @Data    2019-08-08 09:55
	 * @brief   获取距离
	 * @param   串口屏地址
	 * @retval  正确距离
	 */
	 int16_t GetDistance(int16_t dis)
	 {
		 if(!IS_CLOSE_INT(dis,MIN_DISTANCE,MAX_DISTANCE))
     {
       //输入的距离不正确
       return START_DIS_ROLL;
     }
     else return (int16_t)dis;
	 }
/**
	* @Data    2019-08-08 09:57
	* @brief   获取角度
	* @param   串口屏地址
	* @retval  正确角度
	*/
	float GetAngle(float angle)
	{
     if(!IS_CLOSE_INT(angle,MIN_ANGLE,MAX_ANGLE))
     {
       //输入的距离不正确
       return ERROR_ANGLE;
     }
     else 
     {
       return angle;
     }
	}  
 /**
	* @Data    2019-01-28 11:40
	* @brief  设置真实值
	* @param  舵机的pitch 和paw
	* @retval  void
	*/
  float   P1 =0.19  ,P2 =1324, P21 =0.21 , P22 =1275 , ap1 = 0.000009252 ,ap2 =-0.07894 ,ap3 =224.7  ,ap4 = -211500 ;
 void SetReal(int16_t dis ,float angle)
 {
   //距离转换舵机线数
   //float  dis_a=0;
   r_distance = dis;
   if(dis <= 2500)
   {
      ex_distance =  (int16_t)(P1*r_distance + P2);
   }
   else  if(IS_OPEN_INT(dis,2500,2700))
   {
      ex_distance =  (int16_t)(P21*r_distance + P22);
   }
   else
   {
      ex_distance = ap1*pow(dis,3) + ap2*pow(dis,2) +ap3*dis + ap4;
   }
    //角度转换舵机线数
    r_angle = angle;  
    ex_angle = (int16_t)(((angle+181.05f) / 360.0f) * POLES);
    DXL1_setSyncTarAng(&huart2,2,0x00,ex_angle,0x01,ex_distance);
 }  
 /**
	* @Data    2019-01-28 11:40
	* @brief  摄像头数据转换
	* @param  舵机的pitch 和paw
	* @retval  void
	*/  
float OpenmvToAngle(int16_t val)
{
    return (val/5.0f);
}  
 /**
	* @Data    2019-01-28 11:40
	* @brief  装弹
	* @param  
	* @retval  void
	*/ 
int16_t loading_flag = 600;
int16_t up_load = 1850;
void Loading(hmiStrct *phmi) 
{
  if(phmi->load ==1)
  {
    phmi->load =0;     
    DXL1_setSyncTarAng(&huart2,2,0x00,ex_angle ,0x01,up_load);
    osDelay(loading_flag);
    SetReal(r_distance,r_angle);
     osDelay(100);
  }

}    
/* -------------- 校内第二次模拟----------------- */  
/**
	* @Data    2019-01-28 11:40
	* @brief   求直线
	* @param   void
	* @retval  void
	*/
 uint8_t CalLine(LineStruct* pLine_t,int16_t r_x,int16_t r_y,int16_t t_x,int16_t t_y,float k)
 {
   	float temp = 0;
   if(k==TWO_PIONT)
   {
      if ((r_x - t_x) == 0)
        return 1;
      pLine_t->a = (float)(r_y - t_y) / (float)(r_x - t_x);
      pLine_t->b = (float)r_y - pLine_t->a * r_x;
      pLine_t->l = sqrt(pow((t_x-r_x),2) + pow((t_y-r_y),2));
      temp = (float)ABS((t_x - r_x) ) / (float)(LINE_PRE-1);
      pLine_t->pointx[0] = r_x;
      pLine_t->pointy[0] = r_y;          
   }
   else
   {
     pLine_t->a=k;
     pLine_t->b=0;
     temp = 0.7;
      pLine_t->pointx[0] = 0;
      pLine_t->pointy[0] = 0; 
   }
  for (int i = 1;i <LINE_PRE;i++)
	{
		pLine_t->pointx[i]  = pLine_t->pointx[i-1] + temp;
		pLine_t->pointy[i] = pLine_t->a * pLine_t->pointx[i] + pLine_t->b;
	}
	return 0;
 }

 /**
	* @Data    2019-01-28 11:40
	* @brief  舵机线数转角度 
	* @param   舵机线数
	* @retval  角度
	*/
 float ServoToAngle(float val)
 {      
     return ((val / POLES) * 360.0f);
 }
  /**
	* @Data    2019-01-28 11:40
	* @brief  角度转舵机线数
	* @param   角度
	* @retval  舵机线数
	*/
 int16_t AngleToServo(float val)
 {      
     return ((val / POLES) * 360.0f);
 }
 /**
	* @Data    2019-01-28 11:40
	* @brief  角度转长度 
	* @param   长度
	* @retval  void
	*/ 
 int AngleToLen(float Angle)
 {
   return (int)(DISTANCE  *tan(Angle * (PI/180.0f)));
 } 
  /**
	* @Data    2019-01-28 11:40
	* @brief  长度转角度 
	* @param   长度
	* @retval  舵机线数
	*/
 int LenToAngle(int16_t Len)
 {
   float  angle=0;
   r_distance = Len;
    angle = atan (Len) * 180.0 / PI;
   return (int)((angle/360.0f)* POLES) ;
 } 
/**
	* @Data    2019-01-28 11:40
	* @brief  画圆
	* @param   直径
	* @retval  void
	*/
 int fdf=0,buchang=2;
 void DrawTheCircle(float d)
 {
   float rx =  (float)ex_real[X],ry = (float)ex_real[Y],u_y,u_x;
   fdf=0;
   for(int i = 0;i<360;)
  {
    fdf++;
     u_y = ry + sin(i * (PI/180.00)) * d;
     u_x =    rx + cos(i * (PI/180.00))*d;
    if(i<46&&i>19)
      u_x=u_x-1;
    if(i<70)
    {
       u_x=u_x-1;
    }
    if(i>85&&i<115) 
     u_y=u_y+1; 
    if(i==135)
      u_y=u_y-1; 
   if(i==300)
     u_y=u_y+1; 
     SetReal(u_x,u_y); 
     printf("cirs %d,%d,2,RED%c%c%c",(int)(ex_real[X]*0.559f)*10,(190-(int)(ex_real[Y]*0.559f)*10),ex_end,ex_end,ex_end);    
    HAL_Delay(200);
    i=i+5;
  }
 }   
/**
	* @Data    2019-01-28 11:40
	* @brief  到达指定点
	* @param   void
	* @retval  void
	*/
 void GoPiont(int t_x,int t_y)
 {

   //DXL1_setSyncTarAng(&huart2,2,0x00, MarkToServo(t_y),0x01, MarkToServo(t_x)); 
 }
 /**
	* @Data    2019-01-28 11:40
	* @brief  弧度转角度
	* @param   void
	* @retval  void
	*/
 float RadianToAngle(float alfa)
{

        float alfa1,alfa2;

         alfa=alfa*180.0f/PI;

         alfa1=floor(alfa)+floor((alfa-floor(alfa))*60.0f)/100.0f;

         alfa2=(alfa*60.0f-floor(alfa*60.0))*0.006f;

         alfa1+=alfa2;

         return(alfa1);

}
 /**
	* @Data    2019-01-28 11:40
	* @brief  计算弧度
	* @param   void
	* @retval  void
	*/
float JSFWJ(float vx,float vy)

{

         float ml;

         if (vx>0 && vy==0)

                            ml=0;

         else if (vx>0 && vy>0)

                            ml=RadianToAngle(atan(vy/vx));

         else if (vx>0 && vy<0)

                            ml=RadianToAngle(atan(vy/vx)+2*PI);

         else if (vx<0 && vy==0)

                            ml=RadianToAngle(PI);

         else if (vx<0 && vy>0)

                            ml=RadianToAngle(atan(vy/vx)+PI);

         else if (vx<0 && vy<0)

                            ml=RadianToAngle(atan(vy/vx)+PI);

                  
         else if (vx==0 && vy>0)

                            ml=RadianToAngle(PI/2);

         else

                            ml=RadianToAngle(PI*3/2);

         return ml;

}
// /**
//	* @Data    2019-01-28 11:40
//	* @brief  遥控选择模式
//	* @param   遥控指针
//	* @retval  int 返回模式0~4
//	*/
//    int mode_temp = 0 ;
//int ChooseMode(const dbusStruct* pRc_t)
//{
//      if(pRc_t->switch_left ==1)
//     mode = mode_temp;
//    if(pRc_t->switch_left ==3)
//    {
//      if(pRc_t ->switch_right ==1 &&flat_0 ==1)
//      {
//         mode_temp = (mode_temp+1)%6;
//        flat_0 = 0;
//      }
//    }
//     if(pRc_t ->switch_right ==3)
//      flat_0 = 1; 
//  if(pRc_t->switch_left ==2&&pRc_t->switch_right==2)
//  {
//    mode_temp =6;
//     mode =6;
//  }
//   return  mode;    
//}

// /**
//	* @Data    2019-01-28 11:40
//	* @brief  显示光电定位
//	* @param   刷新速度
//	* @retval  void
//	*/
//void ShowLocation(int speed)
//{
//     printf("cirs %d,%d,2,RED%c%c%c",(int)(ex_real[X]*0.559)*10,(190-(int)(ex_real[Y]*0.559)*10),ex_end,ex_end,ex_end); 
//    if(updatap==0)
//       flag1=0;   
//     if(flag1 >speed)
//     {
//       printf("page 0%c%c%c",ex_end,ex_end,ex_end);
//       flag1 =0;
//     }
//     else   flag1++;
//}
/**
	* @Data    2019-01-28 11:40
	* @brief   求直线
	* @param   void
	* @retval  void
	*/
 uint8_t CalLinelEEE(LineStruct* pLine_t,int16_t r_x,int16_t r_y,int16_t t_x,int16_t t_y)
 {
   	float temp = 0;

      if ((r_x - t_x) == 0)
        return 1;
      temp = (float)ABS((t_x - r_x) )/ (float)(100-1);
      pLine_t->pointx_t[0] = r_x;
      pLine_t->pointy_t[0] = t_y;
for (int ii = 1;ii <100;ii++)
	{
		pLine_t->pointx_t[ii]  = pLine_t->pointx_t[ii-1] + temp;
		pLine_t->pointy_t[ii] = t_y;
	}
	return 0;
 }
/*-----------------------------------file of end------------------------------*/
