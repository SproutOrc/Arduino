/*
 * commandList.h
 *
 * Created: 2014.07.05. 10:44:54
 *  Author: F
 */ 


#ifndef COMMANDLIST_H_
#define COMMANDLIST_H_

//***********************Movement Control********************
#define Command_SetBehavior							"sa"
#define Command_RequestedSpeed_FromRC				"rs"
#define Command_RequestedDirection					"rd"
#define Command_RequestedGlobalDirection			"rgd"
#define Command_RequestedTurningRate				"rt"
#define Command_RequestedMaximumSpeed				"rms"
#define Command_BalanceStart						"start"
#define Command_Ready								"rr"

#define Command_PatrolSpeed							"rps"
#define Command_PatrolLoop							"rpl"

#define Command_SET_wp								"SETwp"
#define Command_SET_GOTOgxy							"GOTOgxy"
#define Command_SET_GOTOlxy							"GOTOlxy"

//***********************CSV*********************************
#define Command_Display_Comp_roll					"DISPcompr"
#define Command_Display_Comp_pitch					"DISPcompp"
#define Command_Display_Req_angle					"DISPreqa"
#define Command_Display_Req_speed					"DISPreqs"
#define Command_Display_Req_dir						"DISPreqd"
#define Command_Display_Acc_roll					"DISPaccr"
#define Command_Display_Acc_pitch					"DISPaccp"

#define Command_Display_Global_Dir					"DISPdir"
#define Command_Display_Global_POS					"DISPpos"
#define Command_Display_Local_vel					"DISPvel"
#define Command_Display_Encoder						"DISPenc"

#define Command_Display_M1_PWM						"DISPm1pwm"
#define Command_Display_M2_PWM						"DISPm2pwm"
#define Command_Display_M1_RPM						"DISPm1RPM"
#define Command_Display_M2_RPM						"DISPm2RPM"

#define Command_Display_LoopTime					"DISPlt"

//***********************Setup******************************

//#define Command_SET_AngleDataCalcType				"SETat"
#define Command_Set_CompGain						"SETcg"
#define Command_Set_Payload							"SETpl"


#define Command_SET_PositionKP						"SETpKP"

#define Command_Set_WheelvelKP						"SETwvKP"
#define Command_Set_WheelvelKI						"SETwvKI"
#define Command_Set_Wheelvel_wind_Umin				"SETwvUmin"
#define Command_Set_Wheelvel_wind_Umax				"SETwvUmax"
#define Command_Set_Wheelvel_wind_maxIchange		"SETwvIc"
#define Command_Set_Wheelvel_wind_Imax				"SETwvImax"
#define Command_Set_Wheelvel_wind_fastReset			"SETwvFR"
#define Command_Set_Wheelvel_wind_KC				"SETwvKC"

#define Command_Set_AngleKP							"SETaKP"
#define Command_Set_AngleKI							"SETaKI"
#define Command_Set_AngleKD							"SETaKD"
#define Command_Set_Angle_wind_Umin					"SETaUmin"
#define Command_Set_Angle_wind_Umax					"SETaUmax"
#define Command_Set_Angle_wind_maxIchange			"SETaIc"
#define Command_Set_Angle_wind_Imax					"SETaImax"
#define Command_Set_Angle_wind_fastReset			"SETaFR"
#define Command_Set_Angle_wind_KC					"SETaKC"

#define Command_SET_OrientKP						"SEToKP"
#define Command_SET_OrientKD						"SEToKD"
#define Command_SET_OrientKI						"SEToKI"
#define Command_Set_Orient_wind_Umin				"SEToUmin"
#define Command_Set_Orient_wind_Umax				"SEToUmax"
#define Command_Set_Orient_wind_maxIchange			"SEToIc"
#define Command_Set_Orient_wind_Imax				"SEToImax"
#define Command_Set_Orient_wind_fastReset			"SEToFR"
#define Command_Set_Orient_wind_KC					"SEToKC"

#define Command_SET_TurningKP						"SETtKP"
#define Command_SET_TurningKD						"SETtKD"
#define Command_SET_TurningKI						"SETtKI"
#define Command_Set_Turning_wind_Umin				"SETtUmin"
#define Command_Set_Turning_wind_Umax				"SETtUmax"
#define Command_Set_Turning_wind_maxIchange			"SETtIc"
#define Command_Set_Turning_wind_Imax				"SETtImax"
#define Command_Set_Turning_wind_fastReset			"SETtFR"
#define Command_Set_Turning_wind_KC					"SETtKC"

#define Command_SET_PWM_LPFgain						"SETpwmg"
#define Command_Set_vel_LPF_gain					"SETwvg"
#define Command_SET_Turning_LPFgain					"SETtg"
#define	Command_SET_reqvel_LPFgain					"SETrvg"

#define Command_CSV_ON								"CSV"
#define Command_CSV_RESET							"CSVreset"



#endif /* COMMANDLIST_H_ */