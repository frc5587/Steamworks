package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GasGuzzler extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private static final double INTAKE_POWER = 1.0;
	private VictorSP guzzle;
	
	
	
	public void move()
	{
		guzzle.set( INTAKE_POWER );
		guzzle = new VictorSP( RobotMap.INTAKE_MOTOR );
	}
	
	public void stop()
	{
		guzzle.set( 0 );
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

