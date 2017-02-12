package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.classes.ADXRS450Gyro;
import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This subsystem contains the motors in the drive train
 */
public class Suzy extends Subsystem
{
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	private static double DISTANCE_PER_PULSE = ( 360.0 / 1024.0 );
	
	private Spark motor;
	private Encoder encoder;
	private ADXRS450Gyro gyro;
	
	private boolean onTarget;
	
	
	//Creates a new DriveTrain object and initializes the RobotDrive driveTrain 
	public Suzy()
	{
		motor = new Spark( RobotMap.SUZY_MOTOR );
		encoder = new Encoder( RobotMap.SUZY_ENC_A, RobotMap.SUZY_ENC_B, false, EncodingType.k4X );
		encoder.setDistancePerPulse( DISTANCE_PER_PULSE );
		encoder.setReverseDirection( true );
		
		gyro = new ADXRS450Gyro();
		gyro.startThread();
		
		onTarget = false;
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
	public double getEncAngle( )
	{
		return encoder.getDistance();
	}
	
	public void stop()
	{
		set( 0.0 );
	}
	
	public void setOnTarget( boolean b )
	{
		onTarget = b;
	}
	
	public boolean onTarget()
	{
		return onTarget;
	}
	
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}