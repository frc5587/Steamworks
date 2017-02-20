package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CANSuzy extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private static final int ENCODER_PULSES_PER_REV = 4096;
	
	private CANTalon talon;
	
	private static double kP = 0.001,
							kI = 0,
							kD = 0,
							kF = 0;
	
	public CANSuzy()
	{
		talon = new CANTalon( RobotMap.TURNTABLE_MOTOR_CAN_ID );
		talon.setFeedbackDevice( FeedbackDevice.CtreMagEncoder_Absolute );
		talon.setInverted( false );
		talon.setVoltageRampRate( 0 );
		talon.changeControlMode( TalonControlMode.Position );
		talon.setPIDSourceType( PIDSourceType.kDisplacement );
		
		talon.configNominalOutputVoltage( +0.0f, -0.0f );
		talon.configPeakOutputVoltage( +12.0f, -12.0f );
		
		talon.setAllowableClosedLoopErr( 1 );
		
		talon.setP( kP );
		talon.setI( kI );
		talon.setD( kD );
		talon.setF( kF );
	}
	
	public void enable()
	{
		talon.enable();
	}
	
	public void disable()
	{
		talon.disableControl();
	}
	
	public void setRaw( int target )
	{
		talon.set( target );
	}
	
	public void setPosition( double degrees )
	{
		double target = ( degrees / 360.0 ) * ENCODER_PULSES_PER_REV;
		talon.setPosition( target );
	}
	
	public void setRelativePosition( double degrees )
	{
		double target = ( degrees / 360.0 ) * ENCODER_PULSES_PER_REV + getPosition();
		setPosition( target );
	}
	
	public double getPosition()
	{
		return talon.getEncPosition();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

