#ifndef  __kalman_H_
#define  __kalman_H_

float kalman_filter(int kalman_val, int anc_num, int tag_num);
float kalman_filter_pdoa_x(float kalman_val, int tag_num);
float kalman_filter_pdoa_y(float kalman_val, int tag_num);
float kalman_filter_pdoa_aoa(float kalman_val, int tag_num);

#endif 