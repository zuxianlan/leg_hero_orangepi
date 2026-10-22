/**
  ******************************************************************************
  * @file           : VMC&LQR_Calc.c
  * @author         : gagami
  * @brief          : None
  * @attention      : None
  * @date           : 2025/8/7
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "VMC&LQR_Calc.h"

#include <cmath>

#include "leg_position.h"
#include "leg_speed.h"
#include "leg_convert.h"


/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

/**
 * @brief 
 * @param
 * @return
 */
void class_vmc_leg::VMC_Calc_1()
{
    static float pitch, d_pitch;
    pitch = -INS->pitch;
    d_pitch = -INS->gyro[0];

    leg_position(phi1, phi4, &L0, &phi0);
    leg_speed(d_phi1, d_phi4, phi1, phi4, &d_L0, &d_phi0);

    alpha = phi0 - std::numbers::pi / 2.0;

    theta = alpha - pitch;
    d_theta = (d_phi0 - d_pitch);

    last_phi0 = phi0;

    dd_L0 = (d_L0 - last_d_L0) / dt;

    last_d_L0 = d_L0;
    last_L0 = L0;

    dd_theta = (d_theta - last_d_theta) / dt;
    last_d_theta = d_theta;
}

/**
 * @brief 计算扭矩命令
 * @param
 * @return
 */
void class_vmc_leg::VMC_Calc_2()
{
    leg_convert(F0, Tp, phi1, phi4, torque_set);
    float tmp = 45.0f;
    if(torque_set[0] > tmp)         {torque_set[0] = tmp;}
    else if(torque_set[0] < -tmp)   {torque_set[0] = -tmp;}

    if(torque_set[1] > tmp)         {torque_set[1] = tmp;}
    else if(torque_set[1] < -tmp)   {torque_set[1] = -tmp;}
}

void Fitting_K_Calc(float (*fitting_k)[10],float (*p)[6],float L_l,float L_r)
{
	static unsigned short int i = 0;
	static unsigned short int j = 0;
	
	for(i=0;i<=3;i++)
	{
		for(j=0;j<=9;j++)
		{
			fitting_k[i][j] = p[i*10+j][0] + p[i*10+j][1]*L_l + p[i*10+j][2]*L_r + p[i*10+j][3]*(L_l*L_l) + p[i*10+j][4]*L_l*L_r + p[i*10+j][5]*(L_r*L_r);
		}
	}
}

/**
 * @brief 地面检测
 */
uint8_t class_vmc_leg::ground_detection_L()
{

  
    FN = -F0 * cosf(theta) - Tp * sinf(theta) / L0; 
    
    if (FN > 20)
    {
        return 1;
    }
    else
    {
        return 0;
    }
    return 0;
}

uint8_t class_vmc_leg::ground_detection_R()
{

    FN = -F0 * cosf(theta) - Tp * sinf(theta) / L0; 
    if (FN < -20)
    {
        return 1;
    }
    else
    {
        return 0;
        
    }

    return 0;
}
