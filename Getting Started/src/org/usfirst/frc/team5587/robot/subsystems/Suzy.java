package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This subsystem contains the motors in the drive train
 */
public class Suzy extends Subsystem
{
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	//Provides a limitation to the maximum speed of the drive train (needs to be tested)
	
	private VictorSP motor;
	private Encoder encoder;
	
	//Creates a new DriveTrain object and initializes the RobotDrive driveTrain 
	public Suzy()
	{
		motor = new VictorSP( RobotMap.SUZY_MOTOR );
		encoder = new Encoder( RobotMap.SUZY_ENC_A, RobotMap.SUZY_ENC_B );
		encoder.setDistancePerPulse( 360.0/1024.0*10.0 );
		
	}
	
	/*
	 * Brings robot into motion based on numerical input
	 * 
	 * @param pwr The power level on which to run the drive train motors ( -1 <= pwr <= 1 )
	 */
	public void set( double pwr)
	{
		motor.set( pwr );
	}
	public double get( )
	{
		return encoder.getDistance();
	}
	
	public void stop()
	{
		this.set(0.0);
	}
	
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}