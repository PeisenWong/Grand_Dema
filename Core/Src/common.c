/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "common.h"
#include "math.h"

float pickVG[7] = {1.0, 1.0, 19999.0, 1.0, 1.0, 0.001, 60.0};;

void set(void) {
	Initialize();
	PSxInitDMA(&ps4, &hi2c1);
	TIMxInit(&htim6, 5000, 84);
	MODNInit(MODN_FWD_OMNI, 3.5, 0.5, 2.0, 0.1);
	LidarInit(&huart4, DISCONTINUOUS, &lidar);
	RNS_config(&hcan1);
	ServoxInit(&servo_ring, &htim5, IP15_PIN, TIM_CHANNEL_2); // BDC7
	ServoInitPulseLimit(&servo_ring, 500, 2500);
	ServoSetPulse(&servo_ring, 2300);

	PPInit(fwd_omni, &fXPos, &fYPos, &fyaw, &pp);
	PP_SetZ(-180, &pp);
	PP_PIDPathSet(1.0, 0.5, 0.5, &pp);
	PP_PIDEndSet(0.5, 0.1, 0.7, &pp);
	PP_PIDZSet(0.5, 0.05, 0.2, 5.5, &pp);
	PP_SetCrv_Points(10, &pp);

	R6091U_Init(&IMU, &huart3);

	PIDSourceInit(&pickErr, &pickU, &pick_PID);
	PIDDelayInit(&pick_PID);
	PIDGainInit(0.005, pickVG[0], pickVG[1], pickVG[2], pickVG[3], pickVG[4], pickVG[5], pickVG[6], &pick_PID);

	float fXPosGain[3] = {0.8, 0.4, 0.2};
	float fYPosGain[3] = {0.8, 0.4, 0.2};

	/* X & Y position ABT */
	ABTInit(SAMPLE_TIME, fXPosGain[0], fXPosGain[1], fXPosGain[2], &fXEncData, &fXPos, &fXVel, &fXAcc, &x_data);
	ABTEstimateInit(&x_data);

	ABTInit(SAMPLE_TIME, fYPosGain[0], fYPosGain[1], fYPosGain[2], &fYEncData, &fYPos, &fYVel, &fYAcc, &y_data);
	ABTEstimateInit(&y_data);

	VESCInit(35000, 7, 0.15, 111, &vesc1);
	VESCInit(35000, 7, 0.15, 112, &vesc2);
}

void RNS_config(CAN_HandleTypeDef *hcanx) {
	RNSInit(hcanx, &rns);
	//0b11100101
	//Encoder dcba(0-swap, 1-keep)  BDC dcba(0-keep, 1-swap) //0x00 0x00 0x
	RNSSet(&rns, RNS_DEVICE_CONFIG, (float) 0b00001101, (float) fwd_omni,
			(float) roboconPID);
	RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.05 / 4000.0 * 3.142, 2.0,
			0.05 / 4000.0 * 3.142, 2.0); //1.0 for nonswap , 2.0 for swap
	RNSSet(&rns, RNS_F_KCD_PTD, (float) 5816.0 / 5181.0, //remember to put floating point 5175/5174 wrong
	(float) (3.142 * 0.126 * 2 / 581.6));
	RNSSet(&rns, RNS_B_KCD_PTD, (float) 5191.0 / 5183.0,
			(float) (3.142 * 0.126 * 2 / 519.1));

	RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / 6.88, 19999.0);
	RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 8.66, 19999.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 7.91, 19999.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 7.79, 19999.0);

	RNSSet(&rns, RNS_F_LEFT_VEL_PID, 1.0, 1.5, 0.001);   //
	RNSSet(&rns, RNS_F_RIGHT_VEL_PID, 1.0, 1.5, 0.001);   //
	RNSSet(&rns, RNS_B_LEFT_VEL_PID, 1.0, 1.5, 0.001);   //
	RNSSet(&rns, RNS_B_RIGHT_VEL_PID, 1.0, 1.5, 0.001);   //

	RNSSet(&rns, RNS_PPInit); //Path Planning
	RNSSet(&rns, RNS_PPPathPID, 1.0, 0.5, 0.5);
	RNSSet(&rns, RNS_PPEndPID, 0.5, 0.1, 0.7);
	RNSSet(&rns, RNS_PPZPID, 0.5, 0.05, 0.2, 5.5);
	RNSSet(&rns, RNS_PPSetCRV_PTS, 10.0); // Change No. of Points in the Curved Path
}

void manual_mode(void) {

	while (sys.manual) {

		if (ps4.button == R1) {
			setSpeedMODN(4.0);
		} else if (ps4.button == L1) {
			setSpeedMODN(2.0);
		} else {
			setSpeedMODN(3.0);
		}
		realMODN(&ps4, &rns);
	}

}

void RobotStart()
{
	before = 0;
	mode = 1;
	servo_close_once = 0;
	shoot_done = 0;
	shoot_start = 0;
	vesc_duty = 0;
	pick_start = 0;
	Robot_Pitch = PITCH45;
	set_pitch = 0;
	picked_left = 0;
	picked_right = 0;
	cylinder_retract;
	set_pick_enc = 0;
	pick_tol = 100;
	vel_adjust = 0;
	reload = 0;
	load_stop_once = 0;
	load_start = 0;
	wait_load = 0;
	wheel = 1;
	loaded = 0;
}

void NormalControl()
{
	// Cylinder
	if (ps4.button == OPTION)
	{
		while (ps4.button == OPTION);
//		setPick(0);
		static int counter = 0;
		counter++;

		if(counter == 1)
		{
			vesc_duty = 0.405;
		}
		else if(counter == 2)
		{
			vesc_duty = 0.295;
		}
		else if(counter == 3)
		{
			vesc_duty = 0.175;
			counter = 0;
		}
	}

	// Cylinder
	if (ps4.button == TRIANGLE)
	{
		while (ps4.button == TRIANGLE);
//		static int counter = 0;
//		if (counter == 0)
//		{
//			counter++;
//			cylinder_retract; // BDC1
//		}
//		else
//		{
//			cylinder_load;
//			counter = 0;
//		}
//		LidarSendIns(NEAR, &lidar);
//		ResetCoordinate();
		pick_right = 1;
	}

	// Servo
	if(ps4.button == SQUARE)
	{
		while(ps4.button == SQUARE);
		if(!pp.pp_start)
		{
			static int counter = 0;

			if(counter == 0)
			{
				counter++;
				close_servo;
			}
			else
			{
				open_servo;
				counter = 0;
			}
		}
	}

	// Shoot
	if(ps4.button == CROSS)
	{
		while(ps4.button == CROSS);
		push_shoot;
		shoot_start = 1;
		wait_load = 1;
	}

	// Fly wheel
	if(ps4.button == CIRCLE)
	{
		while(ps4.button == CIRCLE);
		static int counter = 0;

		if(counter == 0)
		{
			counter++;
			vesc_duty = 0.3;
		}
		else
		{
			vesc_duty = 0;
			VESCStop(&vesc1);
			VESCStop(&vesc2);
			counter = 0;
		}
	}

	if(ps4.button == RIGHT)
	{
		while(ps4.button == RIGHT);
		LoadRing();
	}

	if(ps4.button == LEFT)
	{
		while(ps4.button == LEFT);
//		if(!picked_left)
//			pick_left = 1;
//		else
//			pick_right = 1;

		pick_left = 1;

	}

	if (HAL_GetTick() - before >= NormalMode) {
		led9= !led9;
		before = HAL_GetTick();
	}
}

void Auto() {
	if(ps4.button == OPTION)
	{
		while(ps4.button == OPTION);
//		if(lidar.AdjEnb)
//			lidar.AdjEnb = 0;
//		else
//			lidar.AdjEnb = 1;
//		ResetCoordinate();
		ResetPickEnc();
	}

	// Shoot
	if(ps4.button == CROSS)
	{
		while(ps4.button == CROSS);
		push_shoot;
		shoot_start = 1;
		wait_load = 1;
	}

	if(ps4.button == SQUARE)
	{
		while(ps4.button == SQUARE);
//		PP_stop(&pp);
	}

	if(ps4.button == CIRCLE)
	{
		while(ps4.button == CIRCLE);
		static int counter = 0;

		if(counter == 0)
		{
			setPick(2200);

			float pick_left_point[1][7] = {{5.0, -4.3, 0, 0, 0, 0, 0}};
			PP_start(pick_left_point, 1, &pp);
			while(pp.pp_start)
			{
				if(ps4.button == SQUARE)
				{
					while(ps4.button == SQUARE);
					PP_stop(&pp);
				}
			}
			pick_left = 1;
			counter++;
		}
		else
		{
			counter = 0;
			pick_right = 1;
		}
	}

	if(ps4.button == TRIANGLE)
	{
		while(ps4.button == TRIANGLE);
		AutoLoadRing();
	}

	if (HAL_GetTick() - before >= AutoMode)
	{
		led9= !led9;
		before = HAL_GetTick();
	}
}

void setPitch(PITCH_t target_pitch)
{
	switch(target_pitch)
	{
		case PITCH45:
			switch(Robot_Pitch)
			{
				case PITCH70:
					pitch_up;
					set_pitch = 1;
					break;

				default:
					break;
			}
			break;

		case PITCH70:
			switch(Robot_Pitch)
			{
				case PITCH45:
					pitch_down;
					set_pitch = 1;
					break;
				default:
					break;
			}
			break;

		default:
			break;
	}
}

void setPick(int32_t target)
{
	pick_target_enc = target;
	set_pick_enc = 1;
}

void CheckPickEnc(void)
{
	if(set_pick_enc)
	{
		pickErr = pick_target_enc - pick_enc;
		PID(&pick_PID);
		pick_manual((int32_t)pickU);

		if(abs(pickErr) < pick_tol)
		{
			set_pick_enc = 0;
			pick_stop;
		}
	}
}

void CheckPitch()
{
	if(set_pitch)
	{
		switch(Robot_Pitch)
		{
			// Setting to pitch 70 deg
			case PITCH45:
				if(In_Pitch70)
				{
					pitch_stop;
					Robot_Pitch = PITCH70;
					set_pitch = 0;
				}
				break;

			// Setting to pitch 45 deg
			case PITCH70:
				if(In_Pitch45)
				{
					pitch_stop;
					Robot_Pitch = PITCH45;
					set_pitch = 0;
				}
				break;
		}
	}
}

void CheckPick()
{
	if(pick_left)
	{
		// Stick to fence
		float stick_fence_point[1][7] = {{2.0, 0.0, -5, 0, 0, 0, 0}};
		PP_start(stick_fence_point, 1, &pp);
		while(pp.pp_start)
		{
			if(ps4.button == SQUARE)
			{
				while(ps4.button == SQUARE);
				PP_stop(&pp);
			}

			if(In_LS_Left_1 && In_LS_Left_2)
				PP_stop(&pp);
		}
		pick_left = 0;
		picked_left = 1;
		setPick(0);
		float pick_left_point[1][7] = {{0.65, -10, 0, pp.real_z, 0, 0, 0}};
		PP_start(pick_left_point, 1, &pp);
		while(pp.pp_start)
		{
			if(ps4.button == SQUARE)
			{
				while(ps4.button == SQUARE);
				PP_stop(&pp);
				picked_left = 0;
			}

			if(In_Pick && pp.real_x <= -4.7)
				PP_stop(&pp);
		}
		if(picked_left)
		{
			LoadRing();
			lidar.pos_counter = CENTER_1;
			osDelay(500);
			close_servo;
		}
	}

	if(pick_right)
	{
		// Stick to fence
		setPick(0);
		open_servo
		pick_right = 0;
		float stick_fence_right[1][7] = {{2.0, 5, 0, pp.real_z, 0, 0, 0}};
		PP_start(stick_fence_right, 1, &pp);
		while(pp.pp_start)
		{
			if(ps4.button == SQUARE)
			{
				while(ps4.button == SQUARE);
				PP_stop(&pp);
			}

			if(In_LS_Left_1 && In_LS_Left_2)
				PP_stop(&pp);
		}

		picked_right = 1;
		float pick_right_point[1][7] = {{0.65, 0, -10, pp.real_z, 0, 0, 0}};
		PP_start(pick_right_point, 1, &pp);
		while(pp.pp_start)
		{
			if(ps4.button == SQUARE)
			{
				while(ps4.button == SQUARE);
				PP_stop(&pp);
				picked_right = 0;
			}

			if(In_Pick)
				PP_stop(&pp);
		}

		if(picked_right)
			LoadRing();
	}
}

void CheckShoot()
{
	if(In_ShotReady)
		led2 = 1;
	else
		led2 = 0;

	if(In_ShotDone)
		led3 = 1;
	else
		led3 = 0;

	if(shoot_start && In_ShotDone)
	{
		shoot_done = 1;
		shoot_start = 0;
		push_return;
	}
	else if(In_ShotDone)
	{
		push_return;
		shoot_done = 1;
	}

	// Done Shooting
	if(shoot_done && In_ShotReady && wait_load)
	{
		wait_load = 0;
		shoot_done = 0;
		push_stop;
		AutoLoadRing();
	}
	else if(shoot_done && In_ShotReady)
	{
		shoot_done = 0;
		push_stop;
	}
}

void CheckLoad()
{
//	if(load_start && In_Load)
//	{
//		load_start = 0;
//		pick_stop;
//		AutoLoadRing();
//	}
//	else if(load_start)
//	{
//		pick_up;
//	}

	if(load_start || reload)
	{
		if(pick_enc >= 8000 && !load_stop_once)
		{
			pick_stop;
			osDelay(200);
			pick_manual(5000);
			load_stop_once = 1;
		}
		if(pick_enc >= 17000)
		{
			load_start = 0;
			reload = 0;
			pick_stop;
		}
	}

	if(load_start && In_Load && pick_enc >= 9500)
	{
		load_start = 0;
		pick_stop;
		AutoLoadRing();
	}

	if(reload && In_Load)
	{
		reload = 0;
		pick_stop;
	}

	if(loaded >= 7)
	{
		adjust_servo;
	}

	if(loaded >= 10)
		loaded = 0;
}

void Checking()
{
	CheckShoot();
	CheckLoad();
	CheckPick();
	CheckPitch();
}

void LoadRing(void)
{
	load_start = 1;
	load_stop_once = 0;
	pick_up;
}

void StopAutoPP(void)
{
	PP_stop(&pp);
	RNSStop(&rns);
}

void AutoLoadRing(void)
{
	cylinder_load;
	osDelay(500);
	cylinder_retract;
	osDelay(500);
	loaded++;
	reload = 1;
	pick_manual(5000);
}

void ResetCoordinate(void)
{
	RNSSet(&rns, RNS_PP_Reset);
	PP_reset(&pp);
}

void ResetPickEnc(void)
{
	QEIReset(QEI1);
	pick_enc = 0;
}

void enq(void) {

//	RNSEnquire(RNS_VEL_BOTH, &rns);
//	a = rns.enq.enq_buffer[0].data;
//	b = rns.enq.enq_buffer[1].data;
//	c = rns.enq.enq_buffer[2].data;
//	d = rns.enq.enq_buffer[3].data;
//	sprintf((char*)debug, "A: %.2f B: %.2f C: %.2f D: %.2f\n", a, b, c, d);

//	RNSEnquire(RNS_POS_BOTH, &rns);
//	a = rns.enq.enq_buffer[0].data;
//	b = rns.enq.enq_buffer[1].data;
//	c = rns.enq.enq_buffer[2].data;
//	d = rns.enq.enq_buffer[3].data;

//	RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, &rns);
//	xpos = rns.enq.enq_buffer[0].data;
//	ypos = rns.enq.enq_buffer[1].data;
//	z = rns.enq.enq_buffer[2].data;
//	sprintf((char*)debug, "X: %.2f Y: %.2f Yaw: %.2f\n", xpos, ypos, z);

	RNSEnquire(RNS_XYZ_RAW, &rns);
	fXPos = rns.enq.enq_buffer[0].data;
	fYPos = rns.enq.enq_buffer[1].data;
	fyaw = rns.enq.enq_buffer[2].data;

//	sprintf((char*)debug, "A: %.2f B: %.2f C: %.2f D: %.2f X: %.2f Y: %.2f Z: %.2f Servo: %ld\n", a, b, c, d, fXPos, fYPos, fyaw, servo_ring.currentPulse);
//	sprintf((char*)debug, "X: %.2f Y: %.2f Yaw: %.2f\n", fXPos, fYPos, fyaw);
}

void tune(void)
{
	if(ps4.button == UP)
	{
		while(ps4.button == UP);
		tune_p = 1;
		tune_i = 0;
		tune_d = 0;
	}

	if(ps4.button == DOWN)
	{
		while(ps4.button == DOWN);
		tune_p = 0;
		tune_i = 1;
		tune_d = 0;
	}

	if(ps4.button == LEFT)
	{
		while(ps4.button == LEFT);
		tune_p = 0;
		tune_i = 0;
		tune_d = 1;
	}

	if(ps4.button == RIGHT)
	{
		while(ps4.button == RIGHT);
		wheel++;
		if(wheel > 5)
			wheel = 1;
	}

	if(tune_p)
	{
		switch(wheel)
		{
		case 1:
			AP += ps4.joyR_y * 0.0001;
			break;
		case 2:
			BP += ps4.joyR_y * 0.0001;
			break;
		case 3:
			CP += ps4.joyR_y * 0.0001;
			break;
		case 4:
			DP += ps4.joyR_y * 0.0001;
			break;
		}
	}
	else if(tune_i)
	{
		switch(wheel)
		{
		case 1:
			AI += ps4.joyR_y * 0.0001;
			break;
		case 2:
			BI += ps4.joyR_y * 0.0001;
			break;
		case 3:
			CI += ps4.joyR_y * 0.0001;
			break;
		case 4:
			DI += ps4.joyR_y * 0.0001;
			break;
		}
	}
	else
	{
		switch(wheel)
		{
		case 1:
			AD += ps4.joyR_y * 0.00001;
			break;
		case 2:
			BD += ps4.joyR_y * 0.00001;
			break;
		case 3:
			CD += ps4.joyR_y * 0.00001;
			break;
		case 4:
			DD += ps4.joyR_y * 0.00001;
			break;
		}
	}

	if(ps4.button == OPTION)
	{
		while(ps4.button == OPTION);
		switch(wheel)
		{
		case 1:
			RNSSet(&rns, RNS_F_LEFT_VEL_PID, AP, AI, AD);
			break;

		case 2:
			RNSSet(&rns, RNS_F_RIGHT_VEL_PID, BP, BI, BD);
			break;

		case 3:
			RNSSet(&rns, RNS_B_LEFT_VEL_PID, CP, CI, CD);
			break;

		case 4:
			RNSSet(&rns, RNS_B_RIGHT_VEL_PID, DP, DI, DD);
			break;
		}
	}
}

void Shot()
{
	push_shoot;
	shoot_start = 1;
	wait_load = 1;
}

void Await(uint32_t ticks) {
	counter = 0;
	while (counter < ticks);
}
