/********************************************************************
作者：Songyimiao
建立日期: 20151129
版本：V2.0
喵呜实验室版权所有
/********************************************************************/
#include "includes.h"



/***************************************************************
** 作　  者: Songyibiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: Timer1_Update
** 功能描述: 100hz中断服务函数            
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/
void Timer1_Update(void) interrupt  3		//100hz
{  
//	short tst16 = g_iAccel_X_Axis;
	i++;

	g_Throttle=(float)(255 - RxBuf[1]);	//油门	RxBuf[1]:0-255

	if(g_Throttle>20)//如果油门大于80 即已起飞
	{
		if(RxBuf[0]==g_LastCountRunAway)//如果RxBuf[0]的数据没有收到 即矢联
		{
			g_CountRunAway++;  //状态标识+1
			if(g_CountRunAway>128){g_CountRunAway=128;}  //状态标识大于128即1秒没有收到数据，失控保护
		}
		else{g_CountRunAway=0;}
	}
	else{g_CountRunAway=0;} //收到信号退出失控保护
	if(g_CountRunAway==128){g_Throttle=30;RxBuf[1]=128;RxBuf[2]=128;} //触发失控保护 油门为1半少一点，缓慢下降，俯仰横滚方向舵归中

	
	g_LastCountRunAway=RxBuf[0];
	SampleInputVoltage();

	Angle_ax=(float)g_iAccel_X_Axis/8192;  //加速度处理
	Angle_ay=(float)g_iAccel_Y_Axis/8192;  //转换关系8192LSB/g
	Angle_az=(float)g_iAccel_Z_Axis/8192;  //加速度量程 +-4g/S
	Angle_gx -= g_fGyroXOffset;
	Angle_gy -= g_fGyroYOffset;
	Angle_gz -= g_fGyroZOffset;
	Angle_gx=(float)g_iGyro_X_Axis/65.5;   //陀螺仪处理
	Angle_gy=(float)g_iGyro_Y_Axis/65.5;   //陀螺仪量程 
	Angle_gz=(float)g_iGyro_Z_Axis/65.5;   //转换关系65.5LSB/度
	
//***********************************四元数***********************************
	IMUupdate(Angle_gx*0.0174533,Angle_gy*0.0174533,Angle_gz*0.0174533,Angle_ax*0.0174533,Angle_ay*0.0174533,Angle_az*0.0174533);
	//*0.0174533为PI/180 目的是将角度转弧度

	if(123<TxBuf[2]<133)
	{
	TxBuf[2]=128;
	}
	if(123<TxBuf[3]<133)
	{
	TxBuf[3]=128;
	}
	if(113<TxBuf[4]<143)
	{
	TxBuf[4]=128;
	}

	AttitudeControl();
							  
	if(g_Throttle>=20)
	{PWM(PWM3,PWM2,PWM1,PWM4);}	 //入口2345-   PWM3/PWM2/PWM1/PWM4
	else 
	{PWM(1000,1000,1000,1000);}	  
	
/*	
 	UART1SendByte(0x9c);
 	UART1SendByte(0x00);
 	UART1SendByte(tst16&0xf);
 	UART1SendByte(tst16>>8);*/							  
#if 0//DEBUG_UART  //调试启用 预编译命令

   	OutData[0] = Angle_gx;
  	OutData[1] = Angle_gy;
   	OutData[2] = pitch;//对应Angle_gy
   	OutData[3] = roll; //对应Angle_gx 对应硬件Y轴  
//	OutData[0] = g_fOffsetx;
 // 	OutData[1] = g_fOffsety;   	
//	OutData[2] = g_fPower;
//  	OutData[3] = yaw; 
   	OutPut_Data();		
		 	  
#endif	 		
	
  		g_ucLEDCount++;
   		if(g_ucLEDCount >=125) //LED1灯1秒交替闪烁
   		{
			g_ucLEDCount=0;
	 		LED_GREEN = ~LED_GREEN;
   		}
			 
}


				 