package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 *
 */
public class Lucio extends Subsystem
{

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	NetworkTable table;
	private static final String TABLE_NAME = "GRIP/preprocessed";
	
	private static final double kP = .1;
	private static final double kI = 0;
	private static final double kD = 0;
	
	private ADXRS450_Gyro gyro;
	private Spark sparky;
	private PIDController turntPID;
	
	public Lucio()
	{
		
		table = NetworkTable.getTable( TABLE_NAME );
		sparky = new Spark( RobotMap.TURNTABLE_MOTOR );
		
		gyro = new ADXRS450_Gyro( RobotMap.GYRO_PORT );
		gyro.setPIDSourceType( PIDSourceType.kDisplacement );
		
		turntPID = new PIDController( kP, kI, kD, gyro, sparky );
	}
	
	public double [] getXAngles()
	{
		return table.getNumberArray( "angleX", new double [] {} );
	}
	
	public double [] getYAngles()
	{
		return table.getNumberArray( "angleY", new double [] {} );
	}
	
	public void runPID( double targetPos )
	{
		turntPID.disable();
		turntPID.setSetpoint( targetPos );
		turntPID.enable();
	}
	
	public void calibrate()
	{
		gyro.calibrate();
	}
	
	public void reset()
	{
		gyro.reset();
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

