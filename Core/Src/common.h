/*******************************************************************************
 * Title   : common.h
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 12/10
 *******************************************************************************
 * Description: Sensor and function definitions
 *
 * Version History:
 *  1.0 - converted to hal library
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "adapter.h"
#include "cmsis_os.h"

// Blinking hz for modes
#define NormalMode 500
#define AutoMode 125
#define PoleSpeed 6.5
#define BluePickMore 190
#define RedPickMore 230

// Speed
#define type1 4.7
#define BlueType2 8.15
#define RedType2 8.2
#define BlueType3 11.75
#define RedType3 11.7
#define RedOppoType2 12.2
#define BlueOppoType2 12.3

// Duty
#define type1Duty 0.17
#define BlueType2Duty 0.275
#define RedType2Duty 0.27
#define BlueType3Duty 0.375
#define RedType3Duty  0.375
#define BlueOppoType2Duty  0.39
#define RedOppoType2Duty   0.39

#define SAMPLE_TIME 0.005f

#define IP1  		HAL_GPIO_ReadPin(IP1_PIN)
#define IP2  		HAL_GPIO_ReadPin(IP2_PIN)
#define IP3  		HAL_GPIO_ReadPin(IP3_PIN)
#define IP4			HAL_GPIO_ReadPin(IP4_PIN)
#define IP5 		HAL_GPIO_ReadPin(IP5_PIN)
#define IP6 		HAL_GPIO_ReadPin(IP6_PIN)
#define IP7		    HAL_GPIO_ReadPin(IP7_PIN)
#define IP8	 		HAL_GPIO_ReadPin(IP8_PIN)
#define IP9  		HAL_GPIO_ReadPin(IP9_PIN)
#define IP10    	HAL_GPIO_ReadPin(IP10_PIN)
#define IP11  		HAL_GPIO_ReadPin(IP11_PIN)
#define IP12 		HAL_GPIO_ReadPin(IP12_PIN)
#define IP13  		HAL_GPIO_ReadPin(IP13_PIN)
#define IP14 		HAL_GPIO_ReadPin(IP14_PIN)
#define IP15		HAL_GPIO_ReadPin(IP15_PIN)

//ANALOG PIN//
#define IP16	HAL_GPIO_ReadPin(IP16_Analog1_PIN)
#define IP17	HAL_GPIO_ReadPin(IP17_Analog2_PIN)
#define	IP18   HAL_GPIO_ReadPin(IP18_Analog3_PIN)
#define IP19	HAL_GPIO_ReadPin(IP19_Analog4_PIN)
#define IP20	HAL_GPIO_ReadPin(IP20_Analog5_PIN)
#define IP21	HAL_GPIO_ReadPin(IP21_Analog6_PIN)

// Sensors
#define In_Pick			IP1 == 0
#define In_Load			IP2 == 0
#define In_ShotReady 	IP3 == 0
#define In_ShotDone		IP4 == 0
#define In_Pitch45		IP5 == 0
#define In_Pitch70		IP6 == 0
#define In_LS_Left_1	IP7 == 0
#define In_LS_Left_2	IP8 == 0
#define In_LS_Right_1	IP9 == 0
#define In_LS_Right_2	IP10 == 0
#define In_LS_Shot_1	IP11 == 0
#define In_LS_Shot_2	IP12 == 0
#define In_Pick_0		IP13 == 0

// Actuator
#define cylinder_load		SR.cast[1].bit6 = 1;  // BDC1
#define cylinder_retract	SR.cast[1].bit6 = 0;
#define pitch_up			WriteBDC(&BDC3, -19999);
#define pitch_down			WriteBDC(&BDC3, 19999);
#define pitch_stop			WriteBDC(&BDC3, 0);
#define push_shoot			WriteBDC(&BDC4, 19999);
#define push_return			WriteBDC(&BDC4, -19999);
#define push_stop			WriteBDC(&BDC4, 0);
#define open_servo			ServoSetPulse(&servo_ring, 2300); // BDC7
#define close_servo			ServoSetPulse(&servo_ring, 850);
#define load_adjust_servo	ServoSetPulse(&servo_ring, 1100);
#define adjust_servo		ServoSetPulse(&servo_ring, 950);
#define pick_up				WriteBDC(&BDC2, 19999);
#define pick_down			WriteBDC(&BDC2, -19999);
#define pick_stop			WriteBDC(&BDC2, 0);
#define pick_manual(x)		WriteBDC(&BDC2, x);

#define Mux1		 MUX.mux_data.bit0
#define Mux2		 MUX.mux_data.bit1
#define Mux3		 MUX.mux_data.bit2
#define Mux4		 MUX.mux_data.bit3
#define Mux5		 MUX.mux_data.bit4
#define Mux6		 MUX.mux_data.bit5
#define Mux7		 MUX.mux_data.bit6
#define Mux8		 MUX.mux_data.bit7

osThreadId_t MainTaskHandle;
osThreadId_t NaviTaskHandle;
osThreadId_t PitchTaskHandle;
osThreadId_t CheckingTaskHandle;
osThreadId_t LaserNavigateTaskHandle;
osThreadId_t EmergencyTaskHandle;
osThreadId_t SecondaryTaskHandle;
osThreadId_t TuneTaskHandle;
osThreadId_t FlywheelPitchPIDTaskHandle;
osThreadId_t FlywheelYawPIDTaskHandle;
osThreadId_t TestTaskHandle;

osSemaphoreId_t TuneSemaphore;

typedef union{
	uint16_t flags;
	struct{
		//Least significant 16 bits can be cleared all at once by
		//sys.flags = 0 for example during emergency
		unsigned manual       :1;
		unsigned tunePid      :1;
		unsigned pp_ready	  :1;
		unsigned pp_start     :1;
		unsigned rns_busy     :1;
		unsigned ros_test_start  :1;
		unsigned ros_path_start  :1;
		unsigned ros_stop		  :1;
		unsigned vel_ready        :1;
		unsigned stop        :1;
		unsigned pitch_cali	      :1;
		unsigned set_pitch		  :1;
		unsigned load_start		  :1;
		unsigned pick_start       :1;
		unsigned pick_adjust       :1;
		unsigned flag15       :1;

		//Most significant 16 bits are not clear
		unsigned flag16       :1;
		unsigned flag17       :1;
		unsigned flag18	      :1;
		unsigned flag19       :1;
		unsigned flag20       :1;
		unsigned flag21       :1;
		unsigned flag22       :1;
		unsigned flag23		  :1;
		unsigned flag24       :1;
		unsigned flag25       :1;
		unsigned flag26	      :1;
		unsigned flag27		  :1;
		unsigned flag28		  :1;
		unsigned flag29       :1;
		unsigned flag30       :1;
		unsigned flag31       :1;

		//A flag can use more than 1 bit
		//Example : Combine flag30 and flag31
		//unsigned flag29     :1;
		//unsigned flag30     :2;
		//the value of sys.flag30 range from 0 to 3 then overflow to 0 again and vice versa
		//the value of flag29 is not affected when flag30 overflow
	};
}sys_t;

sys_t sys;

enum
{
	NORMAL,
	AUTO,
	TUNE
};

typedef enum
{
	PITCH45,
	PITCH70
}PITCH_t;

PITCH_t Robot_Pitch;

void RNS_config(CAN_HandleTypeDef* hcanx);
void set(void);
void manual_mode(void);
void enq(void);
void setTargetPitch(float target);
void NormalControl();
void RobotStart();
void Checking();
void LoadRing(void);
void AutoLoadRing(void);
void ResetCoordinate(void);
void ResetPickEnc(void);
void StopAutoPP(void);
void Auto();
void setPick(int32_t target);
void CheckPickEnc(void);
void CheckLoad();
void CheckShoot();
void CheckPick0();
void flywheelStop();
void flywheelAct();
void AdjustRings(void);
void flywheelPID(float speed);
void Shot();
void CheckPitch();
void tune(void);
void setPitch(PITCH_t target_pitch);
void Await(uint32_t ticks);

#define NUM_INT_UPDATE		1
#define NUM_FLOAT_UPDATE	1

void ILI9341_Init_List(void);
void ILI9341_Update_List(void);
void calcSpeed(uint32_t);
void registerPath(float path[][7], int numPoint, ...);

struct{
	UART_HandleTypeDef* huart;
	uint8_t buffer[10];
	char lagoriColor;
	uint16_t lagoriHeight;
	uint16_t lagoriWidth;
	uint32_t lagoriArea;
}esp32;
#endif /* INC_COMMON_H_ */
