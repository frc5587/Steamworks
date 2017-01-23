package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Archie extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private static final double SCREW_POWER = .5;
	private static final VictorSP screwIt = new VictorSP( RobotMap.SCREW_MOTOR );
	
	public void spin()
	{
		screwIt.set( SCREW_POWER );
	}
	
	public void stop()
	{
		screwIt.set( 0 );
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

