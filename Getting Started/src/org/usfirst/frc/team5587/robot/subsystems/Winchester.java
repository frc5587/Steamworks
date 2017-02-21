package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Winchester extends Subsystem{
	
	private final VictorSP winchMotor = new VictorSP(RobotMap.WINCH_MOTOR);
	
	public Winchester(){	
    }
	
	public void set(double output)
	{
		winchMotor.set( output );
	}
    
    public double getCurrent()
    {
    	return Robot.pdp.getCurrent( RobotMap.WINCH_PDP_PORT );
    }
    
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}