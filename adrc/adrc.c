/**
  ******************************************************************************
  * @file    adrc.c
  * @author  jiuden
  * @version V1.0.0
  * @date    2021-7-18
  * @brief   自抗干扰控制ADRC算法，非线性控制版。
  ******************************************************************************
  */


#include "adrc.h"
#include "math.h"
float show[3];
adrc_t adrc_spd =
{

    //参数区，这11个就是需要用户整定的参数
    /****************TD**********/
    .r = 100,//快速跟踪因子
    .h = 0.005,//系统调用步长,时间尺度,0.01代表100hz
    .N0 = 0.015,//TD滤波因子
    /**************ESO**********/
    .b = 1, //系统系数
    .delta = 0.04, //delta为fal（e，alpha，delta）函数的线性区间宽度
    .belta01 = 100,//扩张状态观测器反馈增益1
    .belta02 = 1000,//扩张状态观测器反馈增益2
    .belta03 = 2000,//扩张状态观测器反馈增益3

    /**************NLSEF*******/
    .alpha1 = 0.5,//   一般 0<alpha1<1<alpha2
    .alpha2 = 1.5,//
    .belta1 = 300,//跟踪输入信号增益
    .belta2 = 10,//跟踪微分信号增益

    //中间变量区，不需要用户管理以及赋值
    /****************TD*******************/
    .x1 = 0,//跟踪输入
    .x2 = 0, //跟踪输入的微分
    /****************ESO******************/
    .e = 0,//误差
    .z1 = 0,//跟踪反馈值
    .z2 = 0,//跟踪反馈值的而微分
    .z3 = 0,//跟踪系统的扰动（总扰动）
    /**************NLSEF******************/
    .out = 0//输出值

};



/*****************************fhan函数*********************************/
float fhan(float x1, float x2, float r, float h)
{
    /*****************************第一套************************/
//  float d    = 0,
//        a    = 0,
//		  a0   = 0,
//		  a1   = 0,
//		  a2   = 0,
//		  y    = 0,
//		  fhan = 0;
//
//  d    =  r*h*h;
//  a0   =  h*x2;
//  y    =  x1+a0;
//  a1   =  sqrtf(d*(d+8*fabsf(y)));
//  a2   =  a0 + sign(y)*(a1 - d)/2.0;
//  a    =  (a0+y)*(sign(y+d)-sign(y-d))/2.0  + a2*(1-(sign(y+d)-sign(y-d))/2.0);
//  fhan = -r*(a/d)*(sign(y+d)-sign(y-d))/2.0 - r*sign(a)*(1-(sign(a+d)-sign(a-d))/2.0);
    /******************************第二套**********************/
    float deltaa  = 0,
          deltaa0 = 0,
          y       = 0,
          a0      = 0,
          a       = 0,
          fhan    = 0;

    deltaa = r * h;
    deltaa0 = deltaa * h;
    y = x1 + x2 * h;
    a0 = sqrtf(deltaa * deltaa + 8 * r * fabsf(y));

    if(fabsf(y) <= deltaa0)
        a = x2 + y / h;
    else
        a = x2 + 0.5 * (a0 - deltaa) * sign(y);

    if(fabsf(a) <= deltaa)
        fhan = -r * a / deltaa;
    else
        fhan = -r * sign(a);

    return fhan;
}
/************************************sign函数***************************/
float sign(float x)
{
    if(x > 0)
        return (float)1;
    else if(x < 0)
        return (float) -1;
    else
        return (float)0;
}
/*******************************fal函数**********************************/
float fal(float e, float alpha, float delta)
{
    float result = 0, fabsf_e = 0;

    fabsf_e = fabsf(e);

    if(delta >= fabsf_e)
        result = e / powf(delta, 1.0 - alpha);
    else //if(delta<fabsf_e)
        result = powf(fabsf_e, alpha) * sign(e);

    return result;
}

/******************
**ADRC控制器  v目标值，y反馈值*********
********************/
float ADRC_Contrl(adrc_t* ADRC_INPUT, float v, float y)
{
    float u0 = 0,
          e1 = 0,
          e2 = 0;
    /******************************TD****************************************/
    ADRC_INPUT->x1 = ADRC_INPUT->x1 + ADRC_INPUT->h * ADRC_INPUT->x2;
    ADRC_INPUT->x2 = ADRC_INPUT->x2 + ADRC_INPUT->h * fhan(ADRC_INPUT->x1 - v, ADRC_INPUT->x2, ADRC_INPUT->r, ADRC_INPUT->N0);
    /******************************ESO***************************************/
    ADRC_INPUT->e = ADRC_INPUT->z1 - y;
    ADRC_INPUT->z1 = ADRC_INPUT->z1 + ADRC_INPUT->h * (ADRC_INPUT->z2 - ADRC_INPUT->belta01 * ADRC_INPUT->e);
    ADRC_INPUT->z2 = ADRC_INPUT->z2 + ADRC_INPUT->h * (ADRC_INPUT->z3 - ADRC_INPUT->belta02 * fal(ADRC_INPUT->e, 0.5, ADRC_INPUT->delta) + ADRC_INPUT->b * ADRC_INPUT->out);
    ADRC_INPUT->z3 = ADRC_INPUT->z3 + ADRC_INPUT->h * (-ADRC_INPUT->belta03 * fal(ADRC_INPUT->e, 0.25, ADRC_INPUT->delta));
    /******************限幅，ADRC正常的话不会达到限幅条件********************/
//  if(ADRC_INPUT->z1>=30000) ADRC_INPUT->z1=30000;
//  if(ADRC_INPUT->z1<=-30000) ADRC_INPUT->z1 = -30000;
//  if(ADRC_INPUT->z2>=30000) ADRC_INPUT->z2=30000;
//  if(ADRC_INPUT->z2<=-30000) ADRC_INPUT->z2 = -30000;
//  if(ADRC_INPUT->z3>=30000) ADRC_INPUT->z3=30000;
//  if(ADRC_INPUT->z3<=-30000) ADRC_INPUT->z3 = -30000;
    /******************************NLSEF*************************************/
    e1 = ADRC_INPUT->x1 - ADRC_INPUT->z1;
    e2 = ADRC_INPUT->x2 - ADRC_INPUT->z2;
    show[0] = e1;
    u0 = ADRC_INPUT->belta1 * fal(e1, ADRC_INPUT->alpha1, ADRC_INPUT->delta) + ADRC_INPUT->belta2 * fal(e2, ADRC_INPUT->alpha2, ADRC_INPUT->delta); //其中0<alpha1<1<alpha2
    show[1] = u0;
    show[2] = e2;
// ADRC_INPUT->out = u0 - ADRC_INPUT->z3/ADRC_INPUT->b;
    ADRC_INPUT->out = u0 ;
    return -ADRC_INPUT->out;
}



