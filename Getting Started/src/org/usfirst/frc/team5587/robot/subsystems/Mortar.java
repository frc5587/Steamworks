package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Mortar extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private static final int ENC_PULSE_PER_REVOLUTION = 12; 
	private static final int GEAR_RATIO = 4;
	private static final int FLYWHEEL_PULSE_PER_REVOLUTION = ENC_PULSE_PER_REVOLUTION * GEAR_RATIO;
	
	private VictorSP fly;
	private Encoder joey;
	
	public Mortar()
	{
		fly = new VictorSP( RobotMap.FLYWHEEL_PLACEHOLDER_PWM );
		joey = new Encoder( RobotMap.JOEY_A, RobotMap.JOEY_B );
		joey.setDistancePerPulse( 1.0 / FLYWHEEL_PULSE_PER_REVOLUTION );
	}
	
	public void spin( double percent )
	{
		fly.set( percent );
	}
	
	public void stop()
	{
		spin( 0.0 );
	}
	
	public double rps()
	{
		return joey.getRate();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

