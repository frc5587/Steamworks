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
	public static int LEFT_FRONT_MOTOR = 1;
	public static int LEFT_REAR_MOTOR = 0;
	public static int RIGHT_FRONT_MOTOR = 2;
	public static int RIGHT_REAR_MOTOR = 3;
	public static int LAZY_SUSAN_MOTOR = 4;
	public static int WINCH_MOTOR = 5;
	public static int INTAKE_MOTOR = 6;
	public static int SCREW_MOTOR = 7;
	public static int FEEDER_SERVO = 8;
	public static int FLYWHEEL_PLACEHOLDER_PWM = 9;
	
	//DIO
	public static int LEFT_DRIVETRAIN_ENCODER_A = 0;
	public static int LEFT_DRIVETRAIN_ENCODER_B = 1;
	public static int RIGHT_DRIVETRAIN_ENCODER_A = 2;
	public static int RIGHT_DRIVETRAIN_ENCODER_B = 3;
	public static int ULTRASONIC_RANGEFINDER = 4;
	public static int WINCH_ENCODER_A = 5;
	public static int WINCH_ENCODER_B = 6;
	public static int SHOOTER_A = 7;
	public static int SHOOTER_B = 8;

	//CAN
	public static int FLYWHEEL_MOTOR_CAN_ID = 1;

	//Analog In
	public static int LAZY_SUSAN_POTENTIOMETER = 0;
	
	//MXP
	public static Port NAVX_MXP = SPI.Port.kMXP;
	
	//Joysticks
	public static int DRIVER = 0;
	public static int CODRIVER = 1;
	
	//Buttons
	public static int GUZZLE_BUTTON = 3;
	public static int INTAKE_BUTTON = 4;
	
	/*
	 * Michael Morris
	 * 01/09/2017
	 * 
	 * I put everything in here with temporary values so that we can program
	 * using these values and cleanly change them whenever components are
	 * assigned permanent values.
	 */
}