

#ifndef  _ADRC_H
#define _ADRC_H

typedef struct{
	
		//参数区，这11个就是需要用户整定的参数
	/****************TD**********/
	float r ;//快速跟踪因子
	float h;//系统调用步长,时间尺度,0.01代表100hz
	float N0;//TD滤波因子
	/**************ESO**********/
	float b;//系统系数
	float delta   ;//delta为fal（e，alpha，delta）函数的线性区间宽度
	float belta01 ;//扩张状态观测器反馈增益1
	float belta02 ;//扩张状态观测器反馈增益2
	float belta03 ;//扩张状态观测器反馈增益3
			
	/**************NLSEF*******/
	float alpha1 ;//   一般 0<alpha1<1<alpha2
	float alpha2 ;//
	float belta1 ;//跟踪输入信号增益
	float belta2 ;//跟踪微分信号增益

		//中间变量区，不需要用户管理以及赋值
	/****************TD*******************/
	float x1;//跟踪输入
	float x2 ;//跟踪输入的微分
	/****************ESO******************/
	float e ;//误差
	float z1 ;//跟踪反馈值
	float z2 ;//跟踪反馈值的而微分
	float z3 ;//跟踪系统的扰动（总扰动）
	/**************NLSEF******************/
	float out;//输出值
}adrc_t;

float sign(float x);
float fhan(float x1,float x2,float r,float h);
float fal(float e,float alpha,float delta);
float ADRC_Contrl(adrc_t* ADRC_INPUT,float v,float y);
extern adrc_t adrc_spd;
#endif
