package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Mortar extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	private static final double MAX_SPEED = 1500; //RPM
	
	private CANTalon flywheel;
	private VictorSP fly;
	private Encoder joey;
	
	public Mortar()
	{
		fly = new VictorSP( RobotMap.FLYWHEEL_PLACEHOLDER_PWM );
		joey = new Encoder( RobotMap.SHOOTER_A, RobotMap.SHOOTER_B, true, Encoder.EncodingType.k4X );
		joey.setDistancePerPulse( 1.0 / 12.0 );
//	{
//		flywheel = new CANTalon( RobotMap.FLYWHEEL_PLACEHOLDER_PWM );
//		
//		flywheel.setFeedbackDevice( FeedbackDevice.CtreMagEncoder_Relative );
//		
//		flywheel.configNominalOutputVoltage( +0.0f, -0.0f );
//		flywheel.configPeakOutputVoltage( +12.0f, -12.0f );
//		
//		flywheel.setProfile( 0 );
//		flywheel.setF( 0.1097 );
//		flywheel.setP( .22 );
//		flywheel.setI( 0 );
//		flywheel.setD( 0 );
//		
//		flywheel.changeControlMode( TalonControlMode.Speed );
	}
	
	public void spin( double percent )
	{
		fly.set( percent );
	}
	
	public void stop()
	{
		flywheel.set( 0 );
	}
	
	public double RPS()
	{
		return joey.getRate();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

