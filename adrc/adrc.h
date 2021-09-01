

#ifndef  _ADRC_H
#define _ADRC_H

typedef struct{
	
		//����������11��������Ҫ�û������Ĳ���
	/****************TD**********/
	float r ;//���ٸ�������
	float h;//ϵͳ���ò���,ʱ��߶�,0.01����100hz
	float N0;//TD�˲�����
	/**************ESO**********/
	float b;//ϵͳϵ��
	float delta   ;//deltaΪfal��e��alpha��delta������������������
	float belta01 ;//����״̬�۲�����������1
	float belta02 ;//����״̬�۲�����������2
	float belta03 ;//����״̬�۲�����������3
			
	/**************NLSEF*******/
	float alpha1 ;//   һ�� 0<alpha1<1<alpha2
	float alpha2 ;//
	float belta1 ;//���������ź�����
	float belta2 ;//����΢���ź�����

		//�м������������Ҫ�û������Լ���ֵ
	/****************TD*******************/
	float x1;//��������
	float x2 ;//���������΢��
	/****************ESO******************/
	float e ;//���
	float z1 ;//���ٷ���ֵ
	float z2 ;//���ٷ���ֵ�Ķ�΢��
	float z3 ;//����ϵͳ���Ŷ������Ŷ���
	/**************NLSEF******************/
	float out;//���ֵ
}adrc_t;

float sign(float x);
float fhan(float x1,float x2,float r,float h);
float fal(float e,float alpha,float delta);
float ADRC_Contrl(adrc_t* ADRC_INPUT,float v,float y);
extern adrc_t adrc_spd;
#endif
