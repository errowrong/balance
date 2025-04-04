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
*����������Դ
* kalman�˲�
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
    float* FilteredValue;//����ֵ
    float* MeasuredVector;//����ֵ
    float* ControlVector;//��������

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t* MeasurementMap;      // ������״̬�Ĺ�ϵ how measurement relates to the state
    float* MeasurementDegree;     // ����ֵ��ӦH����Ԫ��ֵ elements of each measurement in H
    float* MatR_DiagonalElements; // ���ⷽ�� variance for each measurement
    float* StateMinVariance;      // ��С���� ���ⷽ��������� suppress filter excessive convergence
    uint8_t* temp;

    // ����û����庯��ʹ��,��Ϊ��־λ�����ж��Ƿ�Ҫ������׼KF����������е�����һ��
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;
    
    // definiion of struct mat: rows & cols & pointer to vars
    mat xhat;      // x(k|k) �������
    mat xhatminus; // x(k|k-1) �������
    mat u;         // control vector u ��������
    mat z;         // measurement vector z ��������
    mat P;         // covariance matrix P(k|k) ����Э�������
    mat Pminus;    // covariance matrix P(k|k-1) ����Э�������
    mat F, FT, temp_F;     // state transition matrix F FT ״̬ת�ƾ���
    mat B;         // control matrix B ���ƾ���
    mat H, HT;     // measurement matrix H �������
    mat Q;         // process noise covariance matrix Q ������������
    mat R;         // measurement noise covariance matrix R ������������
    mat K;         // kalman gain  K ����������
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    // �û����庯��,�����滻����չ��׼KF�Ĺ���
    void (*User_Func0_f)(struct kf_t* kf);
    void (*User_Func1_f)(struct kf_t* kf);
    void (*User_Func2_f)(struct kf_t* kf);
    void (*User_Func3_f)(struct kf_t* kf);
    void (*User_Func4_f)(struct kf_t* kf);
    void (*User_Func5_f)(struct kf_t* kf);
    void (*User_Func6_f)(struct kf_t* kf);

    // ����洢�ռ�ָ��
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
