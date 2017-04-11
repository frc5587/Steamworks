package org.usfirst.frc.team5587.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	//PWM
	public static int LEFT_FRONT_MOTOR = 4;
	public static int LEFT_REAR_MOTOR = 1;
	public static int RIGHT_FRONT_MOTOR = 3;
	public static int RIGHT_REAR_MOTOR = 5;
	public static int SUZY_MOTOR = 6;
	public static int WINCH_MOTOR = 0;
	public static int ROLLER_MOTOR = 7;
	public static int SCREW_MOTOR = 2;
	public static int DROP_BOX_FOLDER = 8;
	public static int DROP_BOX_GATEKEEPER = 9;
	
	public static int STAR_R_PWM = 10;
	public static int STAR_G_PWM = 11;
	public static int STAR_B_PWM = 12;
	
	//DIO
	public static int LEFT_DRIVETRAIN_ENCODER_A = 0;
	public static int LEFT_DRIVETRAIN_ENCODER_B = 1;
	public static int RIGHT_DRIVETRAIN_ENCODER_A = 2;
	public static int RIGHT_DRIVETRAIN_ENCODER_B = 3;
	public static int GEAR_SWITCH = 4;
	public static int JOEY_A = 5;
	public static int JOEY_B = 6;
	public static int SUZY_ENC_A = 7;
	public static int SUZY_ENC_B = 8;
	public static int BOTTOM_TRIGGER = 9;
	
	//CAN
	public static int FLYWHEEL_MOTOR_CAN_ID = 1;
	public static int TURNTABLE_MOTOR_CAN_ID = 3;
	public static int ARTICULES_MOTOR_CAN_ID = 3;
	
	public static int ROLLER_MOTOR_CAN_ID = 2;
	
	//MXP
	public static Port NAVX_MXP = SPI.Port.kMXP;
	
	//Joysticks
	public static int DRIVER = 0;
	public static int CODRIVER = 1;
	
	//Driver Buttons
	//public static int UNGUZZLE_BUTTON = 3;
	public static int CLIMB_BUTTON = 6;
	public static int UNWIND_BUTTON = 5;
	public static int INVERT_BUTTON = 7;
	public static int BACK_OFF_BUTTON = 1;
	public static int TELE_OPERATED_BUTTON = 2;
	public static int VOMIT_BUTTON = 4;
	
	//Operator Buttons
	public static int HUNGER_BUTTON = 10;
	public static int DIGEST_BUTTON = 8;
	public static int DELIVER_BUTTON = 7;
	public static int REGRET_BUTTON = 11;
	public static int RESET_BUTTON = 9;
	public static int DUMB_DOWN_BUTTON = 12;
	public static int DUMB_UP_BUTTON = 1;
	
	
	
	
	//NetworkTable
	
	//PDP
	public static int WINCH_PDP_PORT = 0;
	
	
	/*
	 * Michael Morris
	 * 01/09/2017
	 * 
	 * I put everything in here with temporary values so that we can program
	 * using these values and cleanly change them whenever components are
	 * assigned permanent values.
	 */
}
