#ifndef __PID_H
#define __PID_H
#include "sys.h"
#include "stdlib.h"


/*
	M3508较好的参数(空载)
	k_P 3.5
	k_I 0.01
	k_D 1.5
*/

/*
	M2006较好参数(空载)
	k_P 0.8
	k_I 0.01
	k_D 0.5
*/


#define k_P 0.8
#define k_I 0.01
#define k_D 0.5

#define Sk_p 5
#define Sk_i 0.0005
#define Sk_d 2

extern u16 calcu(u8 a,u8 b);
extern void show(u8 t);
extern void set_speed(int goal_speed[4]);
extern void set_destination(int des_X,int des_Y,u8 mode);
#endif

