#include "pid.h"
#include "sys.h"
#include "can.h"
#include "usart.h"

u8 key1;
int Y[4]={0,0,3000,-3000};     //纵向
int X[4]={0,3465,-1731,-1731}; //横向
int W[4]={0,2000,2000,2000};   //顺时针旋转
uint8_t TX_buf[8]={0};         //发送
uint8_t RX_buf[8];             //接受
u8 flag;
u32 IDD;                       //接收到的信号ID
int sign=1;
	//double KD;
u32 IID[4]={512,513,514,515};
double tem_speed[4];           //当前速度
double error[4];               //误差
double last_error[4];          //上次误差
double error_sum[4];           //误差和
double d_error[4];             //误差之差
u16 last_speed[4]={0};         //上次目标速度
int16_t current[4];            //控制电流
u16 angle[4];                  //转子角度
u16 last_angle[4];
int r[4];                    //三个轮子转过距离
float pos_X;                  //小车横向位置
float pos_Y;                  //小车纵向位置
float pos_W;                  //小车自旋角度
float error_X,error_Y,last_error_X,last_error_Y,error_sum_X,error_sum_Y,d_error_X,d_error_Y;


u16 calcu(u8 a,u8 b)
{
	return (a<<8)+b;
}

void show(u8 t)
{
	//printf("转速:%.0f  输出电流:%d  比例电流:%.0f  微分电流:%.0f  积分电流:%.0f  P:%.0f  D:%.0f  I:%.0f\r\n",
	//		   tem_speed,   current,      k_P*error,   k_D*d_error,   k_I*error_sum,  error, d_error, error_sum);
	u8 I_D[3];
	I_D[0]=IDD>>8;
	I_D[1]=(IDD%256)>>4;
	I_D[2]=IDD%512;

	printf("ID:0x%d%d%d  转速:%.0f  输出电流:%d  \r\n",I_D[0],I_D[1],I_D[2],tem_speed[t],   current[t]);
	
}

//void set_speed(u16 goal_speed,float pp,float ii,float dd)
//void set_speed(u16 goal_speed,u8 num)	
void set_speed(int goal_speed[4])
{
	float KP,KI,KD;
	u8 num;

  for(num=1;num<=3;num++)
{
	//printf("目标转速:%d\r\n",goal_speed[num]);
	if(goal_speed[num]!=last_speed[num])
	{error_sum[num]=0;}
	last_speed[num]=goal_speed[num];
	while(1)
  {
		IDD=Can_Receive_Msg(RX_buf);
		if(IDD==IID[num]) {break;}
	}
	//if(key1)//接收到有数据
	//{
		//if(calcu(RX_buf[2],RX_buf[3])>=20000) {tem_speed=0;}
		//else {tem_speed=calcu(RX_buf[2],RX_buf[3]);}
	
	if(RX_buf[2]&0x80)
	{
		//sign=-1;
		//RX_buf[2]&=0x7F;
		tem_speed[num]=-1*(65536-calcu(RX_buf[2],RX_buf[3]));
	}
	else 
	{
		//sign=1;
		tem_speed[num]=calcu(RX_buf[2],RX_buf[3]);
	}
	
	//M2006与M3508转向相反，乘以-1
	//tem_speed[num]*=-1;
	
	
	//}
		//printf("buf[2]:%d  ",RX_buf[2]);
		//printf("buf[3]:%d  ",RX_buf[3]);
		//printf("%.0f",tem_speed);
		//printf("\r\n");
		
	error[num]=goal_speed[num]-tem_speed[num];
	error_sum[num]+=error[num];
	d_error[num]=error[num]-last_error[num];


	//KD=(double)k_D*(1+(6000-error)/6000);
	
	/*
	KP=k_P+pp;
	KI=k_I+ii;
	KD=k_D+(dd/20);
	current=(KP*error)+(KI*error_sum)+(KD*d_error);
	*/
	last_error[num]=error[num];
	current[num]=(k_P*error[num])+(k_I*error_sum[num])+(k_D*d_error[num]);
	if(current[num]>8000) {current[num]=8000;}
	if(current[num]<-8000) {current[num]=-8000;}
	//show(num);
	//printf("%d  ",calcu(RX_buf[0],RX_buf[1]));
/**/
}
	printf("\r\n");
	TX_buf[1]=current[1]&0x00FF;      //LOW 8
	TX_buf[0]=current[1]>>8;          //HIGH 8
	TX_buf[3]=current[2]&0x00FF;      //LOW 8
	TX_buf[2]=current[2]>>8;          //HIGH 8
	TX_buf[5]=current[3]&0x00FF;      //LOW 8
	TX_buf[4]=current[3]>>8;          //HIGH 8

	//TX_buf[1]=(500)&0x00FF;      //LOW 8
	//TX_buf[0]=(500)>>8;
	flag=Can_Send_Msg(TX_buf,8);
	/*if(flag) 
	{printf("Failed\r\n");}	//fail
	else 
	{printf("OK\r\n");}     //success*/
	

	
	return;
}
//转子8192转一圈，减速比1:36；1706.7~1mm
void set_destination(int des_X,int des_Y,u8 mode)//目的坐标/mm
{
	u8 num;
	int a,b,c,trans[4]={0};
	float motor_speed[4]={0};
	float speed_X=0,speed_Y=0;
	
	if(mode==0)
	{
		u8 m=0;
		for(m=0;m<=3;m++) {r[m]=0;}
		pos_X=0, pos_Y=0, pos_W=0; 
	}
	
  for(num=1;num<=3;num++)
	{
		while(1)
		{
			IDD=Can_Receive_Msg(RX_buf);
			if(IDD==IID[num]) {break;}
		}
		angle[num]=calcu(RX_buf[0],RX_buf[1]);
		if(RX_buf[2]&0x80)//反转
		{
			if(angle[num]>last_angle[num]) {r[num]--;}
		}
		else 
		{
			if(angle[num]<last_angle[num]) {r[num]++;}
		}
		last_angle[num]=angle[num];
	}	
	a=r[1]*8192+angle[1];
	b=r[2]*8192+angle[2];
	c=r[3]*8192+angle[3];
	pos_Y=(float)(b-c)/2/1706.7;
	pos_X=(float)(a+a-b-c)/3.464/1706.7;
	pos_W=(float)(a+b+c)/3/1706.7;
	
	//printf("%d  %d\r\n",r[3],angle[3]);
	//printf("%d  %d  %d\r\n",a,b,c);
	printf("%.1f  %.1f\r\n",pos_X,pos_Y);
	
	error_X=des_X-pos_X, error_Y=des_Y-pos_Y;
	error_sum_X+=error_X, error_sum_Y+=error_Y;
	d_error_X=error_X-last_error_X, d_error_Y=error_Y-last_error_Y;
	last_error_X=error_X, last_error_Y=error_Y;
	speed_X=((Sk_p*error_X)+(Sk_i*error_sum_X)+(Sk_d*d_error_X))/1000;
	speed_Y=((Sk_p*error_Y)+(Sk_i*error_sum_Y)+(Sk_d*d_error_Y))/1000;
	motor_speed[1]+=speed_X*X[1], motor_speed[2]+=speed_X*X[2], motor_speed[3]+=speed_X*X[3];
	motor_speed[1]+=speed_Y*Y[1], motor_speed[2]+=speed_Y*Y[2], motor_speed[3]+=speed_Y*Y[3];
	trans[1]=(int)motor_speed[1], trans[2]=(int)motor_speed[2], trans[3]=(int)motor_speed[3];
	printf("speed_X:%.0f  motor[1]:%.0f\r\n",speed_X,motor_speed[1]);
	//printf("%.0f  %.0f  %.0f\r\n",motor_speed[1],motor_speed[2],motor_speed[3]);
	//printf("%d  %d  %d\r\n",trans[1],trans[2],trans[3]);
	set_speed(trans);
}





