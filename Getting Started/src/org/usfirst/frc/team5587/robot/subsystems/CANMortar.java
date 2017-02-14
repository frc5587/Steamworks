package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CANMortar extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private static final int ENC_PULSE_PER_REVOLUTION = 12; 
	private static final int GEAR_RATIO = 4;
	private static final int FLYWHEEL_PULSE_PER_REVOLUTION = ENC_PULSE_PER_REVOLUTION * GEAR_RATIO;
	
	private static double kF = 0.1097,
						  		kP = .22,
								kI = 0.0,
								kD = 0.0;
								
	
	private CANTalon flywheel;
	
	public CANMortar()
	{	
		flywheel = new CANTalon( RobotMap.FLYWHEEL_MOTOR_CAN_ID );
		
		flywheel.setFeedbackDevice( FeedbackDevice.QuadEncoder );
		flywheel.configEncoderCodesPerRev( FLYWHEEL_PULSE_PER_REVOLUTION );
		
		flywheel.configNominalOutputVoltage( +0.0f, -0.0f );
		flywheel.configPeakOutputVoltage( +12.0f, -12.0f );
		
		flywheel.setProfile( 0 );
		flywheel.setF( kF );
		flywheel.setP( kP );
		flywheel.setI( kI );
		flywheel.setD( kD );
		
		SmartDashboard.putNumber( "Flywheel P: ", kP );
		SmartDashboard.putNumber( "Flywheel I: ", kI );
		SmartDashboard.putNumber( "Flywheel D: ", kD );
		SmartDashboard.putNumber( "Flywheel F: ", kF );
		
		flywheel.changeControlMode( TalonControlMode.Speed );
	}
	
	public void spin( double speed )
	{
		flywheel.set( speed );
	}
	
	public void stop()
	{
		flywheel.set( 0 );
	}
	
	public double rps()
	{
		return flywheel.getEncVelocity();
	}
	
	public void speedMode()
	{
		flywheel.changeControlMode( TalonControlMode.Speed );
	}
	
	public void throttleMode()
	{
		flywheel.changeControlMode( TalonControlMode.PercentVbus );
	}
	
	public void updatePID()
	{
		kP = SmartDashboard.getNumber( "Flywheel P: ", kP );
		kI = SmartDashboard.getNumber( "Flywheel I: ", kI );
		kD = SmartDashboard.getNumber( "Flywheel D: ", kD );
		kF = SmartDashboard.getNumber( "Flywheel F: ", kF );
		
		flywheel.setP( kP );
		flywheel.setI( kI );
		flywheel.setD( kD );
		flywheel.setF( kF );
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

