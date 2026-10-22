#ifndef USER_LIB_H
#define USER_LIB_H

void bubble_sort(float arr[], int n);
float median_filter(float input, float* buffer, int size);  
void StateChange(bool input, bool* lastInput, bool* state, int mode);
void first_order_filter_calc(float x, float *y, float *last_y, float alpha);
float Max_Output(float num,float max);
float Float_Math_Constrain(float *x, float Min, float Max);
float power_calculate(float K_0, float K_1, float K_2, float A, float Current, float Omega);
float Float_Math_Abs(float x);
void normalizeAngle(float *angle);
float Math_Modulus_Normalization(float x, float modulus);
float Get_target_angle(float x, float y, float Now_angle, float *vel);
#endif