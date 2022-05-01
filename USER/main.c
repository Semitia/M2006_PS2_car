#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "can.h" 
#include "pid.h"
#include "pstwo.h"
#include "math.h"

//改给mini了

//手柄接口初始化    输入  DI->PA0 
//                  输出  DO->PA1    CS->PA2  CLK->PA3
//串口1以波特率115200输出接收到的数据
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY,i=0;
char button[17][10]={"none","SELECT","L3","R3","START","UP","RIGHT","DOWN","LEFT","L2","R2","L1","R1","GREEN","RED","BLUE","PINK"};
//                 对应musk     0     1    2      3     4      5       6     7      8   9    10   11     12     13    14     15     16

int main(void)
{	 
	float fix[5]={0};//0,1,2 : kp,ki,kd
	u8 n_fix=0; 
	u16 goal_speed=0;
  int GOAL_SPEED[4]={0};
	int Y[4]={0,0,3000,-3000};     //纵向
	int X[4]={0,3465,-1731,-1731}; //横向
	int W[4]={0,2000,2000,2000};   //顺时针旋转
	u8 len;
	u8 key;
	u8 i=0,t=0,j=0;
	u8 cnt=0;
	u8 canbuf[8];
	u8 res;
	u8 mode_des=0;//1:已处于位置模式 0:速度模式，可以把坐标清0

	
	//Stm32_Clock_Init(9);
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200
	LED_Init();		  		//初始化与LED连接的硬件接口
	LCD_Init();			   	//初始化LCD	
	KEY_Init();				//按键初始化		 	
   
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,2,CAN_Mode_Normal);//CAN初始Normal,波特率1M  
	delay_ms(1000);                 //=====延时等待初始化稳定
	PS2_Init();											//=====ps2驱动端口初始化
	PS2_SetInit();		 							//=====ps2配置初始化,配置“红绿灯模式”，并选择是否可以修改
	delay_ms(1000);
	
 	while(1)
	{
		/*if(USART_RX_STA&0x8000)
		{			
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			goal_speed=0;
			for(j=0;j<len;j++)
			{goal_speed=goal_speed*10+(USART_RX_BUF[j]-'0');}
			//error_sum=0;
			printf("goal_speed:%d\r\n",goal_speed);
			USART_RX_STA=0;
		}
		GOAL_SPEED[1]=goal_speed;
		GOAL_SPEED[2]=goal_speed;
		GOAL_SPEED[3]=goal_speed;
		
		
		if(!mode)
		{
			//set_speed(goal_speed,fix[0],fix[1],fix[2]);
			//set_speed(goal_speed,num);
			//set_speed(GOAL_SPEED);
		}
		*/
		key=KEY_Scan(0);

	 if(key==WKUP_PRES)//WK_UP按下，改变要修改的pid参数
		{	   
			//mode=!mode;
			n_fix=(n_fix+1)%3;
			
			if(n_fix==0) printf("P\r\n\r\n");
			else if(n_fix==1) printf("I\r\n\r\n");
			else printf("D\r\n\r\n");
			//delay_ms(1000);
		}		 
		else if(key==KEY0)//参数值减少
		{
			fix[n_fix]-=0.1;
			printf("%f\r\n",fix[n_fix]);
			//delay_ms(1000);
		}
		else if(key==KEY1)//参数值增加
		{
			fix[n_fix]+=0.1;
			printf("%f\r\n",fix[n_fix]);
			//delay_ms(1000);
		}
		
		/*key=Can_Receive_Msg(canbuf);
		if(key)//接收到有数据
		{			
			LCD_Fill(60,270,130,310,WHITE);//清除之前的显示
 			for(i=0;i<key;i++)
			{									    
				if(i<4)LCD_ShowxNum(60+i*32,270,canbuf[i],3,16,0X80);	//显示数据
				else LCD_ShowxNum(60+(i-4)*32,290,canbuf[i],3,16,0X80);	//显示数据
				//printf("%d ",canbuf[i]);
 			}
			printf("机械角度:%d  转速:%d  实际电流:%d  温度:%d",calcu(canbuf[0],canbuf[1]),calcu(canbuf[2],canbuf[3]),calcu(canbuf[4],canbuf[5]),canbuf[6]);
			printf("\r\n");
		}*/
		
		
		t++; 
		if(t==5)
	{	
			u8 k;
			int length;
			double tx,ty;
			PS2_KEY=PS2_DataKey();	
			
			if((Handkey&(1<<(MASK[8]-1)))==0)//L2 速度模式
		{	
			mode_des=0;
			for(k=1;k<=3;k++) {GOAL_SPEED[k]=0;}
			PS2_LX=PS2_AnologData(PSS_LX)-128;    
			PS2_LY=127-PS2_AnologData(PSS_LY);
			PS2_RX=PS2_AnologData(PSS_RX)-128;
			PS2_RY=127-PS2_AnologData(PSS_RY);
			
			if(abs(PS2_LX)>abs(PS2_LY)) {length=abs(PS2_LX);}
			else {length=abs(PS2_LY);}
			if(length==0) {tx=0,ty=0;}
			else
			{
				tx=(double)length*(PS2_LX/sqrt(PS2_LX*PS2_LX+PS2_LY*PS2_LY))/200;
				ty=(double)length*(PS2_LY/sqrt(PS2_LX*PS2_LX+PS2_LY*PS2_LY))/200;
			}
			//printf("%d  %.3f  ",length,sqrt(PS2_LX*PS2_LX+PS2_LY*PS2_LY));
			//printf("%.3f  %.3f\r\n",tx,ty);
			
			GOAL_SPEED[1]+=tx*X[1],GOAL_SPEED[2]+=tx*X[2],GOAL_SPEED[3]+=tx*X[3];
			GOAL_SPEED[1]+=ty*Y[1],GOAL_SPEED[2]+=ty*Y[2],GOAL_SPEED[3]+=ty*Y[3];
			
			/*
			for(i=0;i<16;i++)
			{	    
				if((Handkey&(1<<(MASK[i]-1)))==0)//例如：mask[0]=1,是select(button[1])的序号，但实际其高低情况存储在handkey第0位
				{printf("%s",button[i+1]);}
				
			}
			*/
			
			if((Handkey&(1<<(MASK[4]-1)))==0) //up
			{		GOAL_SPEED[1]+=Y[1],GOAL_SPEED[2]+=Y[2],GOAL_SPEED[3]+=Y[3];	}
			if((Handkey&(1<<(MASK[6]-1)))==0) //down
			{		GOAL_SPEED[1]-=Y[1],GOAL_SPEED[2]-=Y[2],GOAL_SPEED[3]-=Y[3];	}
			if((Handkey&(1<<(MASK[7]-1)))==0) //left
			{		GOAL_SPEED[1]-=X[1],GOAL_SPEED[2]-=X[2],GOAL_SPEED[3]-=X[3];	}
			if((Handkey&(1<<(MASK[5]-1)))==0) //right
			{		GOAL_SPEED[1]+=X[1],GOAL_SPEED[2]+=X[2],GOAL_SPEED[3]+=X[3];	}
			if((Handkey&(1<<(MASK[11]-1)))==0) //R1
			{		GOAL_SPEED[1]+=W[1],GOAL_SPEED[2]+=W[2],GOAL_SPEED[3]+=W[3];	}
			if((Handkey&(1<<(MASK[10]-1)))==0) //L1
			{		GOAL_SPEED[1]-=W[1],GOAL_SPEED[2]-=W[2],GOAL_SPEED[3]-=W[3];	}
			

			

			set_speed(GOAL_SPEED);
			/*printf("PS2_LX:%d, ",PS2_LX);
			printf("PS2_LY:%d, ",PS2_LY);
		  printf("PS2_RX:%d, ",PS2_RX);
			printf("PS2_RY:%d, ",PS2_RY);
			printf("PS2_KEY:%d;",PS2_KEY);*/
			
			//printf("\r\n");
			//printf("%d %d %d\r\n",GOAL_SPEED[1],GOAL_SPEED[2],GOAL_SPEED[3]);
			//delay_ms(10);
		}
		if((Handkey&(1<<(MASK[9]-1)))==0)//L2 位置模式
		{
			
			if((Handkey&(1<<(MASK[12]-1)))==0)
			{set_destination(0,400,mode_des);}
			if((Handkey&(1<<(MASK[13]-1)))==0)
			{set_destination(400,0,mode_des);}
			if((Handkey&(1<<(MASK[14]-1)))==0)
			{set_destination(0,-400,mode_des);}
			if((Handkey&(1<<(MASK[15]-1)))==0)
			{set_destination(-400,0,mode_des);}	
			
			mode_des=1;
			
		}
		
			t=0;
	}	
		
		
		
		
		
		
		
		
		
		delay_ms(1);
	}
}



