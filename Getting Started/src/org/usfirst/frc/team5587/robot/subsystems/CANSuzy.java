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
	public static final int ALLOWABLE_ERROR = 11;
	
	private CANTalon talon;
	
	private static double kP = 1.2,
							kI = 0,
							kD = 200,
							kF = 0;
	
	public CANSuzy()
	{
		talon = new CANTalon( RobotMap.TURNTABLE_MOTOR_CAN_ID );
		talon.setFeedbackDevice( FeedbackDevice.CtreMagEncoder_Relative );
		talon.setInverted( false );
		talon.setVoltageRampRate( 0 );
		talon.changeControlMode( TalonControlMode.Position );
		talon.setPIDSourceType( PIDSourceType.kDisplacement );
		
		talon.configNominalOutputVoltage( +0.0f, -0.0f );
		talon.configPeakOutputVoltage( +12.0f, -12.0f );
		
		talon.setAllowableClosedLoopErr( ALLOWABLE_ERROR );
		
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
	
	public void setRaw( double target )
	{
		talon.setSetpoint( target );
	}
	
	public void setDegrees( double degrees )
	{
		double target = ( degrees / 360.0 );
		talon.setSetpoint( target );
	}
	
	public void setRelativePosition( double degrees )
	{
		double target = ( degrees / 360.0 ) * ENCODER_PULSES_PER_REV + getPosition();
		talon.setSetpoint( target );
	}
	
	public void zeroPos()
	{
		talon.setPosition( 0 );
	}
	
	public double getPosition()
	{
		return talon.getEncPosition();
	}
	
	public double getError()
	{
		return talon.getError();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

