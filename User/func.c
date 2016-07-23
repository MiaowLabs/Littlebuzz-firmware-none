/********************************************************************
作者：Songyimiao
建立日期: 20160411
版本：V1.0
喵呜实验室版权所有
/********************************************************************/
#include "includes.h"
float g_Throttle; //油门
char th;
float code g_Th[3]={4,3.5,4};		//分段油门
int PWM1=0,PWM2=0,PWM3=0,PWM4=0;

int g_LastCountRunAway,g_CountRunAway;
int MotorOut1,MotorOut2,MotorOut3,MotorOut4;
double pitch, yaw, roll;
double Angle_ax, Angle_ay, Angle_az;
double Angle_gx, Angle_gy, Angle_gz;
unsigned char g_fPower;
int g_fGyroXOffset,g_fGyroYOffset,g_fGyroZOffset;
float g_fOffsetx=0,g_fOffsety=0;
unsigned int xdata g_uiStartCount;
unsigned char xdata g_ucLEDCount;
int i;
/******角度控制参数******/
int   g_iAccel_X_Axis,g_iAccel_Y_Axis,g_iAccel_Z_Axis ;	//加速度X轴数据
int   g_iGyro_X_Axis,g_iGyro_Y_Axis,g_iGyro_Z_Axis  ;	//陀螺仪Y轴数据

long int  g_liAccSum;
long int  g_liGyroSum;
float g_fCarAngle;         			//车模倾角
float g_fGyroAngleSpeed;			//角速度      			
float g_fGyroscopeAngleIntegral;	//角速度积分值
float g_fGravityAngle;				//加速度初步计算得到的倾角
int g_iGyroOffset;
/******速度控制参数******/
int   g_iLeftMotorPulse;
int   g_iRightMotorPulse;
int   g_iLeftMotorPulseSigma;
int   g_iRightMotorPulseSigma;
float g_fCarSpeed;
float g_fCarSpeedOld;
float g_fCarPosition;
unsigned char g_ucSpeedControlPeriod ;
unsigned char g_ucSpeedControlCount ;

float g_fXAngleRemote,g_fYAngleRemote,g_fZAngleRemote;
float g_fXAngleCtrOut,g_fYAngleCtrOut,g_fZAngleCtrOut;
float g_fXAngleError,g_fYAngleError;
float g_fXP,g_fXI,g_fXD;
float g_fYP,g_fYI,g_fYD;							 
float g_fXDeriv,g_fXDeltaOld,g_fYDeriv,g_fYDeltaOld;
float Angle_Rx,Angle_Ry,Angle_gy_R;
float Out_PID_X,g_fXAngleError_In,g_fXAngleErrorIntegral_IN;
float Out_PID_Y,g_fYAngleError_In,g_fYAngleErrorIntegral_IN;
float g_fXAngleErrorIntegral,g_fYAngleErrorIntegral;
int g_x,g_y,g_z;

float code g_fcXAngle_P_Out=6.0; //
float code g_fcXAngle_I_Out=0.01; //
float code g_fcXAngle_P_In=1.0; //	  max1.2
float code g_fcXAngle_I_In=0.01; //
float code g_fcXAngle_D_In=4.5; //	  max9

float code g_fcYAngle_P_Out=6.0; //
float code g_fcYAngle_I_Out=0.01; //
float code g_fcYAngle_P_In=1.0; //
float code g_fcYAngle_I_In=0.01; //
float code g_fcYAngle_D_In=4.5; //

float code g_fcZAngle_P=5.0;//3
float code g_fcZAngle_D=4.0; //3
float Anglezlate;
float g_fYAngleErrorIntegral_IN;
float gyro_y_Last,gyro_x_Last,gyro_z_Last;   //储存上一次角速度数据
unsigned char g_COMThrottle;
	   
/******蓝牙控制参数******/
float xdata g_fBluetoothSpeed;
float xdata g_fBluetoothDirection;

/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: DriversInit
** 功能描述: 底层驱动初始化            
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/
void DriversInit(void)
{

	GPIOInit();
  	Timer1Init();
	PWMInit();
	Uart1Init();

}

void ParametersInit()
{
	g_fPower = 0;
	g_fGyroXOffset=g_fGyroYOffset=g_fGyroZOffset=0;	
}

/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: DataSynthesis
** 功能描述: 数据合成函数            
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/
int DataSynthesis(unsigned char REG_Address)	
{
	char idata uiHighByte; /*高八位*/
	char idata ucLowByte; /*低八位*/

	uiHighByte = Single_ReadI2C(REG_Address)  ;
	ucLowByte  = Single_ReadI2C(REG_Address+1);

	return ((uiHighByte << 8) + ucLowByte);   /*返回合成数据*/
}

/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: SampleInputVoltage
** 功能描述: MPU6050采样函数            
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/
void SampleInputVoltage(void)
{	

	g_iGyro_X_Axis   = DataSynthesis(GYRO_XOUT_H) ; //陀螺仪X轴
	g_iGyro_Y_Axis   = DataSynthesis(GYRO_YOUT_H) ; //陀螺仪Y轴
	g_iGyro_Z_Axis   = DataSynthesis(GYRO_ZOUT_H) ; //陀螺仪Z轴
    g_iAccel_X_Axis  = DataSynthesis(ACCEL_XOUT_H); //加速度X轴		
	g_iAccel_Y_Axis  = DataSynthesis(ACCEL_YOUT_H);	//加速度Y轴
	g_iAccel_Z_Axis  = DataSynthesis(ACCEL_ZOUT_H);	//加速度Z轴
}

/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: GyroRevise
** 功能描述: 陀螺仪校正函数            
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/
void GetGyroRevise()
{
	long int tempsumx,tempsumy,tempsumz;
	int temp;
	tempsumx=0;
	tempsumy=0;
	tempsumz=0;
	for(temp=0;temp<100;temp++)
	{
		tempsumx += DataSynthesis(GYRO_XOUT_H) ;
		tempsumy += DataSynthesis(GYRO_YOUT_H) ;
		tempsumz += DataSynthesis(GYRO_ZOUT_H) ;
	}
	g_fGyroXOffset = tempsumx/100;
	g_fGyroYOffset = tempsumy/100;
	g_fGyroZOffset = tempsumz/100;
}



void LEDRUN()
{
	LED0=0;
	Delaynms(300);
	LED1=0;
	Delaynms(300);
	LED2=0;
	Delaynms(300);
	LED3=0;
	Delaynms(300);
	LED3=1;
	Delaynms(300);
	LED2=1;
	Delaynms(300);
	LED1=1;
	Delaynms(300);
	LED0=1;	
}






/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 20160415
** 函数名称: BatteryChecker
** 功能描述: 电量检测（若电量不足，将亮起红灯）           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/
void BatteryChecker()
{

	g_fPower = GetADCResult();	 				//参考电压5.02V 检测max4.2V min3.7V
//	g_fPower = g_fPower / 206 * 4200;	 		//3.7/5.02*256=188	   4.2/5.02*256=214
	if(g_fPower <= 180)						
	{
		LED_RED=0;
	}
	else
	{
	 	LED_RED=1;
	}

}

void TickSound(void)
{
	PWMCKS=0x10;         
	T2L = 0xEB;	
	T2H = 0xFF;
	PWM(960,960,960,960);
	Delaynms(100); //校准完毕滴一声
    PWM(1000,1000,1000,1000);	
    PWMCKS=0x00;
	T2L = 0xD5;			//设定定时初值
	T2H = 0xFF;			//设定定时初值			
}

void AttitudeControl()
{
	g_fZAngleRemote= ((float)TxBuf[4]-128)*1.5;
	Angle_gz=g_fZAngleRemote-Angle_gz; 
	g_fZAngleCtrOut=Angle_gz*g_fcZAngle_P+(Angle_gz-Anglezlate)*g_fcZAngle_D;

	Anglezlate=Angle_gz;

//X轴 
	g_fXAngleRemote = ((float)RxBuf[2]- 128)/7;	 //max 128/7=18
//外环
	g_fXAngleError = g_fXAngleRemote - roll + g_fOffsetx; //ROLL对应硬件X轴

	if(g_Throttle>20)
	{
  		g_fXAngleErrorIntegral+=g_fXAngleError;//外环积分(油门小于某个值时不积分)
	}
	else
	{
		g_fXAngleErrorIntegral=0; //油门小于定值时清除积分值
	}
	if(g_fXAngleErrorIntegral>500){g_fXAngleErrorIntegral=500;}
	else if(g_fXAngleErrorIntegral<-500){g_fXAngleErrorIntegral=-500;}//积分限幅
	Out_PID_X=g_fXAngleError*g_fcXAngle_P_Out+g_fXAngleErrorIntegral*g_fcXAngle_I_Out;//外环PI
//内环
	g_fXAngleError_In = Out_PID_X - Angle_gx - g_fOffsetx ;

	if(g_Throttle>20)
	{
  		g_fXAngleErrorIntegral_IN += g_fXAngleError_In;//(油门小于某个值时不积分)
	}
	else
	{
		g_fXAngleErrorIntegral_IN =0; //油门小于定值时清除积分值
	}
	if(g_fXAngleErrorIntegral_IN>500){g_fXAngleErrorIntegral_IN=500;}
	else if(g_fXAngleErrorIntegral_IN<(-500)){g_fXAngleErrorIntegral_IN=(-500);}//积分限幅

	g_fXAngleCtrOut=g_fXAngleError_In*g_fcXAngle_P_In+g_fXAngleErrorIntegral_IN*g_fcXAngle_I_In-(Angle_gx-gyro_x_Last)*g_fcXAngle_D_In;//内环PID
	gyro_x_Last = Angle_gx;
	
  	if(g_fXAngleCtrOut>  1000) {g_fXAngleCtrOut =  1000; }  //输出量限幅
	if(g_fXAngleCtrOut<(-1000)){g_fXAngleCtrOut =(-1000);}
	
	MotorOut2= (int)(g_Throttle * 4 - g_fXAngleCtrOut + g_fZAngleCtrOut );	//255*4=1020
	MotorOut4= (int)(g_Throttle * 4 + g_fXAngleCtrOut + g_fZAngleCtrOut); 

	PWM2 = (1000 - MotorOut2 );	   
	if(PWM2>1000){PWM2=1000;}
	else if(PWM2<0){PWM2=0;}
	
	PWM4 = (1000 - MotorOut4 );
	if(PWM4>1000){PWM4=1000;}
	else if(PWM4<0){PWM4=0;}

		
  	
//Y轴部分
	g_fYAngleRemote = ((float)RxBuf[3]- 128)/7;	 //max18
//外环
	g_fYAngleError = g_fYAngleRemote - pitch + g_fOffsety ; //ROLL对应硬件X轴

	if(g_Throttle>20)
	{
  		g_fYAngleErrorIntegral+=g_fYAngleError;//外环积分(油门小于某个值时不积分)
	}
	else
	{
		g_fYAngleErrorIntegral=0; //油门小于定值时清除积分值
	}
	if(g_fYAngleErrorIntegral>500){g_fYAngleErrorIntegral=500;}
	else if(g_fYAngleErrorIntegral<-500){g_fYAngleErrorIntegral=-500;}//积分限幅
	Out_PID_Y=g_fYAngleError*g_fcXAngle_P_Out+g_fYAngleErrorIntegral*g_fcYAngle_I_Out;//外环PI
//内环
	g_fYAngleError_In = Out_PID_Y - Angle_gy ;

	if(g_Throttle>20)
	{
  		g_fYAngleErrorIntegral_IN += g_fYAngleError_In;//(油门小于某个值时不积分)
	}
	else
	{
		g_fYAngleErrorIntegral_IN =0; //油门小于定值时清除积分值
	}
	if(g_fYAngleErrorIntegral_IN>500){g_fYAngleErrorIntegral_IN=500;}
	else if(g_fYAngleErrorIntegral_IN<(-500)){g_fYAngleErrorIntegral_IN=(-500);}//积分限幅

	g_fYAngleCtrOut=g_fYAngleError_In*g_fcYAngle_P_In+g_fYAngleErrorIntegral_IN*g_fcYAngle_I_In-(Angle_gy-gyro_y_Last)*g_fcYAngle_D_In;//内环PID
	gyro_y_Last = Angle_gy;
	
    if(g_fYAngleCtrOut>1000){g_fYAngleCtrOut=1000;}  //输出量限幅
	if(g_fYAngleCtrOut<-1000){g_fYAngleCtrOut=-1000;}
	
	MotorOut1= (int)g_Throttle * 4 - g_fYAngleCtrOut - g_fZAngleCtrOut;
	MotorOut3= (int)g_Throttle * 4 + g_fYAngleCtrOut - g_fZAngleCtrOut;

	PWM1=(1000 - MotorOut1 );	  
	if(PWM1>1000){PWM1=1000;}
	else if(PWM1<0){PWM1=0;}
	
	PWM3=(1000 - MotorOut3 );
	if(PWM3>1000){PWM3=1000;}
	else if(PWM3<0){PWM3=0;}
}