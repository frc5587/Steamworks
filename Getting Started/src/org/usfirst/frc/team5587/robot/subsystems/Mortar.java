package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Mortar extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	private static final double SPIN_POWER = 1.0;
	
	private TalonSRX flyWheel;
	
	public Mortar()
	{
		flyWheel = new TalonSRX( RobotMap.FLYWHEEL_PLACEHOLDER_PWM );
	}
	
	public void spin()
	{
		flyWheel.set( SPIN_POWER );
	}
	
	public void stop()
	{
		flyWheel.set( 0 );
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

