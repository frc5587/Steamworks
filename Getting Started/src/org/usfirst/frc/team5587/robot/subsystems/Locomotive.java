package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This subsystem contains the motors in the drive train
 */
public class Locomotive extends Subsystem
{
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	//Provides a limitation to the maximum speed of the drive train (needs to be tested)
	private static final double SCALE_FACTOR_Y = 1.0;
	private static final double SCALE_FACTOR_X = 1.0;
	
	private RobotDrive driveTrain;
	
	//Creates a new DriveTrain object and initializes the RobotDrive driveTrain 
	public Locomotive()
	{
	
		driveTrain = new RobotDrive( new VictorSP( RobotMap.LEFT_FRONT_MOTOR),
								  	 new VictorSP (RobotMap.LEFT_REAR_MOTOR),
								  	 new VictorSP (RobotMap.RIGHT_FRONT_MOTOR),
								  	 new VictorSP (RobotMap.RIGHT_REAR_MOTOR));
		
	}
	
	/*
	 * Brings robot into motion based on numerical input
	 * 
	 * @param pwr The power level on which to run the drive train motors ( -1 <= pwr <= 1 )
	 * @param curve Tells the robot how much to curve: a curve less than 0 turns , a curve of 0 drives straight, a curve greater than 0 turns , a curve of plus or minus 1 turns the robot in place.  
	 */
	public void trot( double pwr, double curve)
	{
		driveTrain.drive( pwr , curve );
	}

	/*
	 * Allows the driver to take direct control over the movement of Robot
	 * 
	 * @param stick The joystick that will dictate Robot's movements
	 */
	public void canter( Joystick stick )
	{
		double xValue = stick.getX();
		double yValue = stick.getY();
		driveTrain.arcadeDrive( -yValue * SCALE_FACTOR_Y, -xValue * SCALE_FACTOR_X, false);
	}
	
	public void stop()
	{
		this.trot(0, 0);
	}
	
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}