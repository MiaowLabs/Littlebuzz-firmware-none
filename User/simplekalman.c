#include "simplekalman.h"
#include "math.h"

float k=0;  //Kalman增益
float p=0;  //预测误差协方差
float q=0;  //预测误差
float r=0;  //传感器测量误差
float x=0;  //返回值

void SimpleKalman(float q1,float r1){
	q=q1;
	r=r1;	
	p = sqrt(q1*q1+r1*r1);
}

float UpdateSimpleKalman(float value){
	p+=q;
	k=p/(p+r);
	x+=k*(value-x);
	p*=(1-k);

	return x;
}