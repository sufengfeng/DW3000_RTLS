#include "kalman.h"
#include "instance.h"
#include <math.h>

const float Q=0.018;     // Q （偏差）为高斯白噪声 不随时间变化
const float R=0.542;     // R  (偏差) 为高斯白噪声 不随时间变化


float kalman_filter_pdoa_x(float kalman_val, int tag_num)
{
	static float kalman[MAX_TAG_LIST_SIZE][2]={0};//x_last;p_last
	float kg;          // kg 为 kalman filter 用于计算 最优值
	float x_mid;       // 当前的预测值  
	float x_now;       // 当前的最优值
	float p_mid;       // 当前的协方差
	float p_now;       // 当前的 最优 协方差 

	if((kalman[tag_num][0] == 0)&&(kalman[tag_num][1] == 0))
	{
		kalman[tag_num][0] = kalman_val;
		kalman[tag_num][1] = kalman_val;
	}

	x_mid=kalman[tag_num][0];  
	p_mid=kalman[tag_num][1]+Q;
	kg=p_mid/(p_mid+R);
	x_now=x_mid+kg*(kalman_val-x_mid);
	p_now=(1-kg)*p_mid;
	
	kalman[tag_num][1] = p_now;  //更新covariance值
	kalman[tag_num][0] = x_now;  //更新系统状态值
	
	return x_now;
}
float kalman_filter_pdoa_y(float kalman_val, int tag_num)
{
	static float kalman[MAX_TAG_LIST_SIZE][2]={0};//x_last;p_last
	float kg;          // kg 为 kalman filter 用于计算 最优值
	float x_mid;       // 当前的预测值  
	float x_now;       // 当前的最优值
	float p_mid;       // 当前的协方差
	float p_now;       // 当前的 最优 协方差 

	if((kalman[tag_num][0] == 0)&&(kalman[tag_num][1] == 0))
	{
		kalman[tag_num][0] = kalman_val;
		kalman[tag_num][1] = kalman_val;
	}

	x_mid=kalman[tag_num][0];  
	p_mid=kalman[tag_num][1]+Q;
	kg=p_mid/(p_mid+R);
	x_now=x_mid+kg*(kalman_val-x_mid);
	p_now=(1-kg)*p_mid;
	
	kalman[tag_num][1] = p_now;  //更新covariance值
	kalman[tag_num][0] = x_now;  //更新系统状态值
	
	return x_now;
}
float kalman_filter_pdoa_aoa(float kalman_val, int tag_num)
{
	static float kalman[MAX_TAG_LIST_SIZE][2]={0};//x_last;p_last
	float kg;          // kg 为 kalman filter 用于计算 最优值
	float x_mid;       // 当前的预测值  
	float x_now;       // 当前的最优值
	float p_mid;       // 当前的协方差
	float p_now;       // 当前的 最优 协方差 

	if((kalman[tag_num][0] == 0)&&(kalman[tag_num][1] == 0))
	{
		kalman[tag_num][0] = kalman_val;
		kalman[tag_num][1] = kalman_val;
	}

	x_mid=kalman[tag_num][0];  
	p_mid=kalman[tag_num][1]+Q;
	kg=p_mid/(p_mid+R);
	x_now=x_mid+kg*(kalman_val-x_mid);
	p_now=(1-kg)*p_mid;
	
	kalman[tag_num][1] = p_now;  //更新covariance值
	kalman[tag_num][0] = x_now;  //更新系统状态值
	
	return x_now;
}

float kalman_filter(int kalman_val, int anc_num, int tag_num)
{
	static float kalman[MAX_AHCHOR_NUMBER][MAX_TAG_LIST_SIZE][2]={0};//anc tag x_last;p_last
	float kg;          // kg 为 kalman filter 用于计算 最优值
	float x_mid;       // 当前的预测值
	float x_now;       // 当前的最优值
	float p_mid;       // 当前的协方差
	float p_now;       // 当前的 最优 协方差
	
	if(kalman[anc_num][tag_num][0] == 0)//第一次得到测距值，未设置初值
	{
		kalman[anc_num][tag_num][0] = kalman_val;
		kalman[anc_num][tag_num][1] = kalman_val;
	}
	
	//x_mid=x_last;
	x_mid = kalman[anc_num][tag_num][0];
	
	//p_mid=p_last+Q;
	p_mid = kalman[anc_num][tag_num][1]+Q;

	kg = p_mid/(p_mid+R);
	x_now = x_mid+kg*(kalman_val-x_mid);
	p_now = (1-kg)*p_mid;
	
	//p_last = p_now;  //更新covariance值
	kalman[anc_num][tag_num][1] = p_now;
	//x_last = x_now;  //更新系统状态值
	kalman[anc_num][tag_num][0] = x_now;

	return x_now;
}


