#ifndef __KALMAN_H__
#define __KALMAN_H__

class Kalman
{
public:
	float X_last; //��һʱ�̵����Ž��  X(k-|k-1)
	float X_mid;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
	float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
	float P_mid;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
	float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
	float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
	float kg;     //kalman����
	float A;      //ϵͳ����
	float B;
	float Q;
	float R;
	float H;
	/**
		* @name   kalmanCreate
		* @brief  ����һ���������˲���
		* @param  p:  �˲���
		*         T_Q:ϵͳ����Э����
		*         T_R:��������Э����
		*
		* @retval none
		* @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
		*		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
		*/
	Kalman(float T_Q, float T_R)
	{
		X_last = (float)0;
		P_last = 0;
		Q = T_Q;
		R = T_R;
		A = 1;
		B = 0;
		H = 1;
		X_mid = X_last;
	}

	float Filter(float dat)
	{
		X_mid = A * X_last;                   //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
		P_mid = A * P_last + Q;               //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
		kg = P_mid / (P_mid + R);           //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
		X_now = X_mid + kg * (dat - X_mid);   //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
		P_now = (1 - kg) * P_mid;             //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
		P_last = P_now;                     //״̬����
		X_last = X_now;
		return X_now;						//���Ԥ����x(k|k)
	}
};

#endif // !__KALMAN_H__