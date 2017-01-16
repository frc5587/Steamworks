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
	public static final int LEFT_FRONT_MOTOR = 0;
	public static final int LEFT_REAR_MOTOR = 1;
	public static final int RIGHT_FRONT_MOTOR = 2;
	public static final int RIGHT_REAR_MOTOR = 3;
	public static final int TURNTABLE_MOTOR = 4;
	public static final int WINCH_MOTOR = 5;
	public static final int INTAKE_MOTOR = 6;
	public static final int SCREW_MOTOR = 7;
	public static final int FEEDER_SERVO = 8;
	public static final int FLYWHEEL_PLACEHOLDER_PWM = 9;
	
	//DIO
	public static final int LEFT_DRIVETRAIN_ENCODER_A = 0;
	public static final int LEFT_DRIVETRAIN_ENCODER_B = 1;
	public static final int RIGHT_DRIVETRAIN_ENCODER_A = 2;
	public static final int RIGHT_DRIVETRAIN_ENCODER_B = 3;
	public static final int ULTRASONIC_RANGEFINDER = 4;
	public static final int WINCH_ENCODER_A = 5;
	public static final int WINCH_ENCODER_B = 6;

	//CAN
	public static final int FLYWHEEL_MOTOR_CAN_ID = 1;
	
	//MXP
	public static final Port NAVX_MXP = SPI.Port.kMXP;
	public static final Port GYRO_PORT = SPI.Port.kOnboardCS0;

	
	//Joysticks
	public static final int DRIVER = 0;
	public static final int CODRIVER = 1;
	
	/*
	 * Michael Morris
	 * 01/09/2017
	 * 
	 * I put everything in here with temporary values so that we can program
	 * using these values and cleanly change them whenever components are
	 * assigned permanent values.
	 */
}