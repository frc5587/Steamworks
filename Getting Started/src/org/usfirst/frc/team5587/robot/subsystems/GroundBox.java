package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GroundBox extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private CANTalon articules;
	private VictorSP highRoller;
	private DigitalInput gearSwitch;

	private static double ROLL_POWER = 0.35;
	
	private static final double UP_POSITION = 70;
	private static final double DELIVER_POSITION = 45;
	private static final double DOWN_POSITION = 0;
	private static final int ERROR_MARGIN = 5;
	private static final int CURRENT_LIMIT = 40;
	
	private static double kP,
							kI,
							kD,
							kF;
	
	public GroundBox()
	{
		articules = new CANTalon( RobotMap.ARTICULES_MOTOR_CAN_ID );
		highRoller = new VictorSP( RobotMap.ROLLER_MOTOR );
		gearSwitch = new DigitalInput( RobotMap.GEAR_SWITCH );
		
		articules.changeControlMode( TalonControlMode.Position );
		articules.setFeedbackDevice( FeedbackDevice.CtreMagEncoder_Absolute );
		articules.enableZeroSensorPositionOnForwardLimit(true);
		articules.reverseOutput(false);
		
		articules.configNominalOutputVoltage( +0.0f, -0.0f );
		articules.configPeakOutputVoltage( +12.0f, -12.0f );
		
		articules.setP( kP );
		articules.setI( kI );
		articules.setD( kD );
		articules.setF( kF );
		
		articules.setAllowableClosedLoopErr(ERROR_MARGIN);
		articules.setCurrentLimit(CURRENT_LIMIT);
		articules.EnableCurrentLimit(true);
	}
	
	public double getPosition()
	{
		return articules.getPosition();
	}
	
	public double getDegrees()
	{
		return getPosition() * 360.0 / 4096.0;
	}
	
	public boolean hasGear()
	{
		return gearSwitch.get(); 
	}
	
	public void grindUp()
	{
		grindManual( UP_POSITION );
	}
	
	public void grindDown()
	{
		grindManual( DOWN_POSITION );
	}
	
	public void deliveryNotDigiorno()
	{
		articules.setSetpoint( DELIVER_POSITION );
	}
	
	public void grindManual(double pos)
	{
		articules.setSetpoint( pos/360 );
	}
	
	public void grindReset()
	{
		articules.setPosition(0);
	}
	public void stopGrind(){
		articules.disable();
	}
	
	public void succ()
	{
		highRoller.set( ROLL_POWER );
		
	}
	
	public void rollOut()
	{
		highRoller.set( -ROLL_POWER );
	}
	
	public void stopRolling()
	{
		highRoller.stopMotor();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}