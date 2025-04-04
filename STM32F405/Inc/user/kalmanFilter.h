#pragma once
#ifndef __KALMANFILTER_H__
#define __KALMANFILTER_H__

#include "stm32f4xx_hal.h"
#include <functional>
#include <vector>
#include <cmath>
#include "stdlib.h"
#include "./Inc/user/label.h"
#include "./Inc/user/martix.h"
#include "arm_math.h"
/*
* 
*来自王工开源
* kalman滤波
*
*/



#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
//#define Matrix_Init mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

typedef struct kf_t
{
    float* FilteredValue;//估计值
    float* MeasuredVector;//测量值
    float* ControlVector;//控制输入

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t* MeasurementMap;      // 量测与状态的关系 how measurement relates to the state
    float* MeasurementDegree;     // 测量值对应H矩阵元素值 elements of each measurement in H
    float* MatR_DiagonalElements; // 量测方差 variance for each measurement
    float* StateMinVariance;      // 最小方差 避免方差过度收敛 suppress filter excessive convergence
    uint8_t* temp;

    // 配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;
    
    // definiion of struct mat: rows & cols & pointer to vars
    mat xhat;      // x(k|k) 后验估计
    mat xhatminus; // x(k|k-1) 先验估计
    mat u;         // control vector u 控制向量
    mat z;         // measurement vector z 量测向量
    mat P;         // covariance matrix P(k|k) 后验协方差矩阵
    mat Pminus;    // covariance matrix P(k|k-1) 先验协方差矩阵
    mat F, FT, temp_F;     // state transition matrix F FT 状态转移矩阵
    mat B;         // control matrix B 控制矩阵
    mat H, HT;     // measurement matrix H 量测矩阵
    mat Q;         // process noise covariance matrix Q 过程噪声矩阵
    mat R;         // measurement noise covariance matrix R 量测噪声矩阵
    mat K;         // kalman gain  K 卡尔曼增益
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    // 用户定义函数,可以替换或扩展基准KF的功能
    void (*User_Func0_f)(struct kf_t* kf);
    void (*User_Func1_f)(struct kf_t* kf);
    void (*User_Func2_f)(struct kf_t* kf);
    void (*User_Func3_f)(struct kf_t* kf);
    void (*User_Func4_f)(struct kf_t* kf);
    void (*User_Func5_f)(struct kf_t* kf);
    void (*User_Func6_f)(struct kf_t* kf);

    // 矩阵存储空间指针
    float* xhat_data, * xhatminus_data;
    float* u_data;
    float* z_data;
    float* P_data, * Pminus_data;
    float* F_data, * FT_data, * temp_F_data;
    float* B_data;
    float* H_data, * HT_data;
    float* Q_data;
    float* R_data;
    float* K_data;
    float* S_data, * temp_matrix_data, * temp_matrix_data1, * temp_vector_data, * temp_vector_data1;
} KalmanFilter_t;

class KALMANFILTER
{
public:

    uint16_t sizeof_float, sizeof_double;
    uint32_t stop_time;
    KalmanFilter_t kf;
    
    void TaskInit(std::vector<float> _P, std::vector<float> _F, std::vector<float> _Q,
        std::vector<float> _R, std::vector<float> _H,std::vector<float> _Z,
        std::vector<float> _measurement_reference, std::vector<float> _state_min_variance, 
        std::vector<float>_measurement_degree, std::vector<float> _mat_R_diagonal_elements,
        uint8_t xhatSize, uint8_t uSize, uint8_t zSize, uint8_t autoAdjust, std::function<void()>fun);
   /* void TaskInit(float _P[], float _F[], float _Q[],
        float* _measurement_reference, float* _state_min_variance, float* _measurement_degree, float* _mat_R_diagonal_elements,
        uint8_t xhatSize, uint8_t uSize, uint8_t zSize);*/
    void TaskUpdate();

    std::function<void()> StateUpdate;

    void SensorUpdate(std::vector<float> sensorValue);

    void H_K_R_Adjustment(KalmanFilter_t* kf);   

    void Kalman_Filter_Init(KalmanFilter_t* kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
    void Kalman_Filter_Measure(KalmanFilter_t* kf);
    void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t* kf);
    void Kalman_Filter_PminusUpdate(KalmanFilter_t* kf);
    void Kalman_Filter_SetK(KalmanFilter_t* kf);
    void Kalman_Filter_xhatUpdate(KalmanFilter_t* kf);
    void Kalman_Filter_P_Update(KalmanFilter_t* kf);
    float* Kalman_Filter_Update(KalmanFilter_t* kf);
};

extern KALMANFILTER speedKalmanFilter, IMUKalmanFilter;

#endif // !__KALMANFILTER_H__
