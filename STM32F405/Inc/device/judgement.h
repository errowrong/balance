#pragma once
#include "./Inc/basic/usart.h"
#include "./Inc/user/CRC.h"
#include "string.h"

#define BUFSIZE 100
#define DMA_RX_SIZE 100
#define DMA_TX_SIZE 150
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


/****************************��ʼ��־*********************/
#define UI_SOF 0xA5
/****************************CMD_ID����********************/
#define UI_CMD_Robo_Exchange 0x0301    
/****************************����ID����********************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************�췽������ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************����������ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************�췽������ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************����������ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************ɾ������***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************ͼ�����ò���__ͼ�β���********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************ͼ�����ò���__ͼ������********************/
#define UI_Graph_Line 0         //ֱ��
#define UI_Graph_Rectangle 1    //����
#define UI_Graph_Circle 2       //��Բ
#define UI_Graph_Ellipse 3      //��Բ
#define UI_Graph_Arc 4          //Բ��
#define UI_Graph_Float 5        //������
#define UI_Graph_Int 6          //����
#define UI_Graph_Char 7         //�ַ���
/***************************ͼ�����ò���__ͼ����ɫ********************/
#define UI_Color_Main 0         //������ɫ
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //�Ϻ�ɫ
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //��ɫ
#define UI_Color_Black 7
#define UI_Color_White 8

class Judgement
{
public:
	bool ready = false;
	bool powerheatready = false;
	bool online = false;
	bool graphInit = false;

	bool capState;

	float prebulletspd = 0;
	uint32_t id_blue = 256;
	uint32_t id_red = 256;

	uint8_t baseRFID;//��������
	uint8_t highlandRFID;//�ߵ�����
	uint8_t energyRFID;//������������
	uint8_t feipoRFID;//��������
	uint8_t outpostRFID;//ǰ��վ����
	uint8_t resourseRFID;//��Դ������
	int32_t nBullet = 0;
	int32_t count = 0;

	//uint8_t mode;
	float voltage;

	void Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base);
	void GetData(void);
	void SendData(void);
	void BuffData();
	void Decode(uint8_t* m_frame);

	void DisplayPantilePosition(int16_t deltaAngle);
	void DisplayCapState(uint8_t capState);
	void DisplpayMode(uint8_t mode);
	void DisplayFireMode(uint8_t fireMode);
	void DisplayCapVoltage(float capVoltage);
	void DisplayLegPosition(float phi1, float phi2, float phi3, float phi4, bool leftOrRight);

	void DisplayStaticUI();
	void DisplayDynamicUI();

	struct {
		uint16_t CmdID;

		//��ϸ�����뷭�ġ�RoboMaster 2021 ����ϵͳ����Э�鸽¼ V1.0��20210203����
		//cmd:0x0001 ����Ƶ�ʣ�1Hz
		struct {
			uint8_t game_type : 4;//��������
			uint8_t game_progress : 4;//��ǰ�����׶�
			uint16_t stage_remain_time;//��ǰ�׶�ʣ��ʱ��
			uint64_t SyncTimeStamp;
		} game_status_t;

		//cmd:0x0002 ��������ʱ����
		struct {
			uint8_t winner;//0��ƽ�֣�1���췽ʤ����2������ʤ��
		} game_result_t;

		//cmd:0x0003 ����Ƶ�ʣ�1Hz
		struct {
			uint16_t red_1_robot_HP;//��1Ӣ�ۻ�����Ѫ��
			uint16_t red_2_robot_HP;//��2���̻�����Ѫ��
			uint16_t red_3_robot_HP;//��3����������Ѫ��
			uint16_t red_4_robot_HP;//��4����������Ѫ��
			uint16_t red_5_robot_HP;//��5����������Ѫ��
			uint16_t red_7_robot_HP;//��7�ڱ�������Ѫ��
			uint16_t red_outpost_HP;//�췽ǰ��վѪ��
			uint16_t red_base_HP;//�췽����Ѫ��
			uint16_t blue_1_robot_HP;//��1Ӣ�ۻ�����Ѫ��
			uint16_t blue_2_robot_HP;//��2���̻�����Ѫ��
			uint16_t blue_3_robot_HP;//��3����������Ѫ��
			uint16_t blue_4_robot_HP;//��4����������Ѫ��
			uint16_t blue_5_robot_HP;//��5����������Ѫ��
			uint16_t blue_7_robot_HP;//��7�ڱ�������Ѫ��
			uint16_t blue_outpost_HP;//����ǰ��վѪ��
			uint16_t blue_base_HP;//��������Ѫ��
		}  game_robot_HP_t;

		//�����¼����� cmd:0x0101 ����Ƶ�ʣ��¼��ı����
		struct
		{
			uint32_t event_data;
		}event_data_t;

		//����Ƶ�ʣ����ڷ������
		struct {
			uint8_t dart_belong;//������ڵĶ��飻1���췽���ڣ�2����������
			uint16_t stage_remaining_time;//����ʱʣ�����ʱ��
		} ext_dart_status_t;

		//����վ������ʶ cmd:0x0102������Ƶ�ʣ������ı����, ���ͷ�Χ������������
		struct {
			uint8_t reserved;
			uint8_t supply_robot_id;
			uint8_t supply_projectile_step;
			uint8_t supply_projectile_num;
		} ext_supply_projectile_action_t;

		//���о�����Ϣ cmd_id:0x0104 ����Ƶ�ʣ����淢������
		struct {
			uint8_t level;
			uint8_t foul_robot_id;
			uint8_t count;
		} referee_warning_t;

		//���ڷ���ڵ���ʱ cmd_id:0x0105 ����Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ������������
		struct {
			uint8_t dart_remaining_time;
			uint16_t dart_info;
		}dart_dart_info_t;

		//����������״̬ cmd:0x0201������Ƶ�ʣ�10Hz
		struct {
			uint8_t robot_id;//������id
			uint8_t robot_level;//�����˵ȼ�
			uint16_t current_HP;//������ʣ��Ѫ��
			uint16_t maximum_HP;//����������Ѫ��
			uint16_t shooter_barrel_cooling_value;//������ǹ������ÿ����ȴֵ
			uint16_t shooter_barrel_heat_limit;//������ǹ����������
			uint16_t chassis_power_limit{};//�����˵��̹�������
			uint8_t power_management_gimbal_output ;//gimbal���Ƿ������
			uint8_t power_management_chassis_output ;//chassis���Ƿ������
			uint8_t power_management_shooter_output;//shooter���Ƿ������
		}robot_status_t;

		//ʵʱ������������ cmd:0x0202������Ƶ�ʣ�50Hz
		struct {
			uint16_t chassis_voltage;//���������ѹ ��λ����
			uint16_t chassis_current;//����������� ��λ����
			float chassis_power;//����������� ��λ��
			uint16_t chassis_power_buffer;//���̹��ʻ���
			uint16_t shooter_17mm_1_barrel_heat;//1��17mmǹ������
			uint16_t shooter_17mm_2_barrel_heat;//2��17mmǹ������
			uint16_t shooter_42mm_barrel_heat;//42mmǹ������
		}power_heat_data_t;

		//������λ�� cmd:0x0203������Ƶ�ʣ�10Hz
		struct {
			float x;//λ��x����
			float y;//λ��y����
			float angle;//�������˲���ģ��ĳ��򣬵�λ���ȡ�����Ϊ 0 ��
		}robot_pos_t;

		//���������� cmd:0x0204 ����Ƶ�ʣ�1Hz
		struct {
			uint8_t recovery_buff;//�����˻�Ѫ���棨�ٷֱȣ�ֵΪ 10 ��ʾÿ��ָ�Ѫ�����޵� 10%��
			uint8_t cooling_buff;//������ǹ����ȴ���ʣ�ֱ��ֵ��ֵΪ 5 ��ʾ 5 ����ȴ
			uint8_t defence_buff;//�����˷������棨�ٷֱȣ�ֵΪ 50 ��ʾ 50 % �������棩
			uint8_t vulnerability_buff;	//�����˸��������棨�ٷֱȣ�ֵΪ 30 ��ʾ - 30 % �������棩
			uint16_t attack_buff;//�����˹������棨�ٷֱȣ�ֵΪ 50 ��ʾ 50 % �������棩
		}buff_t;

		//���л���������״̬ cmd:0x0205 ����Ƶ�ʣ�10Hz
		struct
		{
			uint8_t airforce_status;
			uint8_t time_remain;
		}air_support_data_t;

		//�˺�״̬ cmd:0x0206 ����Ƶ�ʣ��˺���������
		struct
		{
			uint8_t armor_id;
			uint8_t HP_deduction_reason;
		}hurt_data_t;

		//ʵʱ�����Ϣ cmd:0x0207 ����Ƶ�ʣ��������
		struct {
			uint8_t bullet_type;
			uint8_t shooter_number;
			uint8_t bullet_freq;//�ӵ���Ƶ ��λHz
			float bullet_speed;//�ӵ����� ��λm/s
		}shoot_data_t;

		//�ӵ�ʣ�෢������ʣ������ cmd:0x0208 ����Ƶ�ʣ�10Hz ���ڷ��ͣ����л����˷���
		struct
		{
			uint16_t projectile_allowance_17mm;
			uint16_t projectile_allowance_42mm;
			uint16_t remaining_gold_coin;
		}projectile_allowance_t;

		//������ RFID ״̬ cmd0:x0209 ����Ƶ�ʣ�1Hz�����ͷ�Χ����һ������
		struct
		{
			uint32_t rfid_status;
		}rfid_status_t;

		//���ڻ����˿ͻ���ָ������ cmd:0x020A ����Ƶ�ʣ�10Hz�����ͷ�Χ����һ������
		struct
		{
			uint8_t dart_launch_opening_status;//��ǰ���ڷ���վ��״̬
			uint8_t reserved;
			uint16_t target_change_time;//�л�����Ŀ��ʱ�ı���ʣ��ʱ��
			uint16_t latest_launch_cmd_time;//���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ�䣬��λ���룬��ʼֵΪ 0��
		}dart_client_cmd_t;

		//���������� cmd:0x020B
		struct
		{
			float hero_x;
			float hero_y;
			float engineer_x;
			float engineer_y;
			float standard_3_x;
			float standard_3_y;
			float standard_4_x;
			float standard_4_y;
			float standard_5_x;
			float standard_5_y;
		}ground_robot_position_t;

		//��������˱��״̬ cmd:0x020C
		struct
		{
			uint8_t mark_hero_progress;
			uint8_t mark_engineer_progress;
			uint8_t mark_standard_3_progress;
			uint8_t mark_standard_4_progress;
			uint8_t mark_standard_5_progress;
			uint8_t mark_sentry_progress;
		}radar_mark_data_t;

		//Զ�̶һ����輰�������� cmd:0x020D
		struct
		{
			uint32_t sentry_info;
		} sentry_info_t;

		//˫������״̬ cmd:0x020E
		struct
		{
			uint8_t radar_info;
		} radar_info_t;

	}data;

#pragma pack(1)
	//-----------------------------------------------------------
	typedef __packed struct
	{
		uint8_t delete_type;
		uint8_t layer;
	}interaction_layer_delete_t;

	typedef  __packed  struct {
		uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
	}robot_interaction_data_t;
	typedef  __packed  struct {
		uint8_t data[15];
	} robot_interactive_data_t;
	typedef __packed struct
	{
		uint8_t figure_name[3];
		uint32_t operate_tpye : 3;
		uint32_t figure_tpye : 3;
		uint32_t layer : 4;
		uint32_t color : 4;
		uint32_t start_angle : 9;
		uint32_t end_angle : 9;
		uint32_t width : 10;
		uint32_t start_x : 11;
		uint32_t start_y : 11;
		uint32_t radius : 10;
		uint32_t end_x : 11;
		uint32_t end_y : 11;
	} graphic_data_struct_t;
	typedef struct
	{
		uint8_t figure_name[3];
		uint32_t operate_tpye : 3;
		uint32_t figure_tpye : 3;
		uint32_t layer : 4;
		uint32_t color : 4;
		uint32_t start_angle : 9;
		uint32_t end_angle : 9;
		uint32_t width : 10;
		uint32_t start_x : 11;
		uint32_t start_y : 11;
		uint32_t radius : 10;
		uint32_t end_x : 11;
		uint32_t end_y : 11;
	} float_data_struct_t;
	typedef struct
	{
		graphic_data_struct_t Graph_Control;
		uint8_t show_Data[30] = {};
	} string_data_struct_t;                  //��ӡ�ַ�������

	//-----------------------------------------------------------

	typedef __packed  struct
	{
		uint8_t  sof = 0xA5;//����֡��ʼ�ֽڣ��̶�ֵΪ0x05
		uint16_t data_length; //������data�ĳ���
		uint8_t  seq;  //��֡ͷ
		uint8_t  crc8;  //֡ͷCRC8У��

	} frame_header_t;
	typedef __packed  struct
	{
		frame_header_t   						txFrameHeader;
		uint16_t								CMD;//������id
		robot_interaction_data_t				txID;
		uint16_t		 						FrameTail;
	}CommunatianData_graphic_t;


#pragma pack()


private:

	uint16_t robotId = UI_Data_RobotID_BStandard3;
	uint16_t clientId = UI_Data_CilentID_BStandard3;

	uint8_t m_uartrx[DMA_RX_SIZE] = { 0 };
	uint8_t m_uarttx[DMA_TX_SIZE] = { 0 };
	uint8_t m_frame[DMA_RX_SIZE] = { 0 };
	uint8_t m_FIFO[BUFSIZE] = { 0 };
	uint8_t* m_whand = m_FIFO;
	uint8_t* m_rhand = m_FIFO;
	uint32_t m_readnum = 0;
	uint32_t m_leftsize = 0;

	uint8_t UI_seq{};

	UART* m_uart;

	union _4bytefloat
	{
		uint8_t b[4];
		float f;
	};
	float u32_to_float(uint8_t* chReceive)
	{
		union _4bytefloat x;
		memcpy(x.b, chReceive, sizeof(float));
		return x.f;
	}

	BaseType_t pd_Rx = false;
	QueueHandle_t* queueHandler = NULL;
	bool Transmit(uint32_t read_size, uint8_t* plate);


	void LineDraw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, \
		uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, \
		uint32_t Start_y, uint32_t End_x, uint32_t End_y);
	void Rectangle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, \
		uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, \
		uint32_t Start_y, uint32_t End_x, uint32_t End_y);
	void Circle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, \
		uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);
	void Arc_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, \
		uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, \
		uint32_t Start_y, uint32_t x_Length, uint32_t y_Length);
	void Float_Draw(float_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, 
		uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, \
		uint32_t Start_y, float Graph_Float);
	void Char_Draw(string_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, 
		uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, \
		uint32_t Start_y, char* Char_Data);


	void UI_ReFresh(int cnt, graphic_data_struct_t* imagedata);
	void UI_ReFresh(int cnt, float_data_struct_t* floatdata);
	void Char_ReFresh(string_data_struct_t* string_Data);

	void UIDelete(uint8_t deleteOperator, uint8_t deleteLayer);


};

extern "C" Judgement judgement;

