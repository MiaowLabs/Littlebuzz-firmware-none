
#include <uart.h>   

/***************************************************************
** 函数名称: Uart1Init
** 功能描述: UART1初始化函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
***************************************************************/
void Uart1Init(void)		//115200bps@20.000MHz
{
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x04;		//定时器2时钟为Fosc,即1T
	T2L = 0xC5;		//设定定时初值
	T2H = 0xFF;		//设定定时初值
	AUXR |= 0x01;		//串口1选择定时器2为波特率发生器
	AUXR |= 0x10;		//启动定时器2
//	TI = 1;
}



/***************************************************************
** 函数名称: UART1ReceiveByte
** 功能描述: UART1接收函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
***************************************************************/
unsigned char UART1ReceiveByte(void)
{
    unsigned char xdata ucRxd1; 
    /*if(RI == 1)					     
    {
      RI = 0;
      ucRxd1 = SBUF;
      return(ucRxd1);		 
    }
    //return 0;	  */
	while(RI==0);
	RI = 0;
	ucRxd1 = SBUF;
	return(ucRxd1);
}

/***************************************************************
** 函数名称: UART2ReceiveByte
** 功能描述: UART2接收函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
***************************************************************/
unsigned char UART2ReceiveByte(void)
{
    unsigned char xdata ucRxd2; /*
	if ((S2CON & 0x01) == 1)  
    {  
      S2CON &= 0xFE;  
      ucRxd2 = S2BUF;
	  return(ucRxd2);  
    }     		 	   */
	while((S2CON & 0x01) == 0);
	S2CON &= 0xFE;
	ucRxd2 = S2BUF;
	return(ucRxd2);
}

/***************************************************************
** 函数名称: UART1SendByte
** 功能描述: UART1发送函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
***************************************************************/
void UART1SendByte(unsigned char TxD1)  
{   
    SBUF=TxD1;  
    while(TI == 0);//等待发送完成 
    //while(!TI);//等待发送完成 
    TI=0;   
}  

/***************************************************************
** 函数名称: UART2SendByte
** 功能描述: UART2发送函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
***************************************************************/
void UART2SendByte(unsigned char TxD2)  
{   
    S2BUF=TxD2;  
    while ((S2CON & 0x02) == 0); //等待发送完成 
    S2CON &= 0xFD;    
}   

void Send(int dataa,int datab,int datac,int datad,int datae,int dataf)
{
	unsigned char sum=0;
	//ES = 1;  //打开串口中断
	UART1SendByte(0xAA);   //帧头
	UART1SendByte(0xAA);
	UART1SendByte(0x02);   //功能字
	UART1SendByte(12);   //发送的数据长度
	UART1SendByte(dataa);     //低8位
	UART1SendByte(dataa>>8);  //高8位
	UART1SendByte(datab);
	UART1SendByte(datab>>8);
	UART1SendByte(datac);
	UART1SendByte(datac>>8);
	UART1SendByte(datad);
	UART1SendByte(datad>>8);
	UART1SendByte(datae);
	UART1SendByte(datae>>8);
	UART1SendByte(dataf);
	UART1SendByte(dataf>>8);		
	sum+=0xAA;sum+=0xAA;sum+=0x02;sum+=12;
	sum+=dataa>>8;sum+=dataa;sum+=datab>>8;sum+=datab;sum+=datac>>8;sum+=datac;
	sum+=datad>>8;sum+=datad;sum+=datae>>8;sum+=datae;sum+=dataf>>8;sum+=dataf;
	UART1SendByte(sum);  //校验和
	//ES = 0;   //关闭串口中断
}