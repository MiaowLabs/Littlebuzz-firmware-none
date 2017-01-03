/********************************************************************
作者：Songyimiao
建立日期: 20160411
版本：V1.0
喵呜实验室版权所有
/********************************************************************/
#include "includes.h"
 
/***************************************************************
** 作　  者: Songyibiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: main()
** 功能描述: 主函数            
** 输　  入:   
** 输　  出:   
** 备    注: 参考STC开源四轴 感谢刘同学
    //失控：TxBuf[0] 
	//油门：TxBuf[1]
    //Yaw：	TxBuf[4]
    //俯仰：TxBuf[2]
    //横滚：TxBuf[3]
********************喵呜实验室版权所有**************************
***************************************************************/
void main()
{
	DisableInterrupts();//禁止总中断
	DriversInit();//底层驱动初始化		
	MPU6050Init();//MPU6050初始化
	LEDRUN();//电机臂跑马灯 

	GetGyroRevise();//校正陀螺仪	
//	LED_GREEN = 1; //绿灯灭
	ParametersInit();//系统参数初始化
	SimpleKalman(0.3,1.0); //一维卡尔曼滤波初始化
	Delaynms(50);//延时50ms	
	
	IAPRead(); //若果检测到更新过数据就读出来，读取IAP数据

	while(NRF24L01_Check())//检测不到24L01
	{
		Delaynms(500);
		LED_RED=~LED_RED;  
		Delaynms(500);
		LED_RED=~LED_RED; 
	} 
	init_NRF24L01();
	SetRX_Mode();	 //接收模式

	Delaynms(10);	 
	RxBuf[1]=255; //油门通道
	RxBuf[2]=128; //俯仰
	RxBuf[3]=128; //横滚
	RxBuf[4]=128;//yaw
	
	LED2=0;	//尾灯
	LED1=0;	//尾灯
	TickSound();  //启动响声 用到T2
				
	EnableInterrupts();//允许总中断	 
									  
	while(1)
	{
	TxBuf[1]=(int)roll;
	TxBuf[2]=(int)pitch;	
	TxBuf[0]=(int)(g_fPower*10);	 //-128-127 
	  STC_ISP();	//ISP 下载不用冷启动 				   
	if(!SoftTimer){
		SoftTimer = 10;
		SetTX_Mode();
	  	nRF24L01_TxPacket(TxBuf);	//发送模式	
		Delaynms(1);
	  	SetRX_Mode();
	}
	  nRF24L01_RxPacket(RxBuf);
	  BatteryChecker();
	 
	if(RxBuf[5]==1)	   //开关右侧按钮-->校正加速度计
	  {
	    DisableInterrupts();//关闭中断，防止干扰到IAP写入
		IAP_Angle();
		RxBuf[5]=0;
		TickSound();
		EnableInterrupts(); 
	  }
	  if(RxBuf[6]==1)  //开关左侧按钮-->校正陀螺仪
	  {
	  	DisableInterrupts();//关闭中断，防止干扰到IAP写入
		IAP_Gyro();
		RxBuf[6]=0;
		TickSound();
	    EnableInterrupts();
	  }
	  
	  if(i>=10){
	   i=0;
	   if(RxBuf[8]==1)	  //y  微调按钮
	   {
		  RxBuf[8]=0;
		  g_fOffsety-=1;
		//TickSound();
	   }	
	   if(RxBuf[9]==1)
	   {
		  RxBuf[9]=0;
		  g_fOffsety+=1;
		//TickSound();
		  }	
		  if(RxBuf[10]==1)	  //x
		  {
		  	RxBuf[10]=0;
		  	g_fOffsetx-=1;
		//	TickSound();
		  }	
		  if(RxBuf[7]==1)
		  {
		  	RxBuf[7]=0;
		  	g_fOffsetx+=1;
		//	TickSound();
		  }				
		}

	  
#if 0//DEBUG_UART  //调试启用 预编译命令

	OutData[0] = 45;
  	OutData[1] = 90;  
   	OutData[2] = g_fPower ;
   	OutData[3] = yaw;  

   	OutPut_Data();		
	 	  
#endif	 		
					
	}
}
