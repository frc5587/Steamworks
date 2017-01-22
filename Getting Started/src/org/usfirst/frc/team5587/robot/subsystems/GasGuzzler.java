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
	private static final double INTAKE_POWER = .25;
	private static final VictorSP guzzle = new VictorSP( 3 );
	
	public GasGuzzler()
	{
		guzzle.enableDeadbandElimination( true );
	}
	
	public void move()
	{
		guzzle.set( INTAKE_POWER );
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

