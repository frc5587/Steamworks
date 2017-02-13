package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class Winchester extends Subsystem{
	
	private final VictorSP winchMotor = new VictorSP(RobotMap.WINCH_MOTOR);
	public static final PowerDistributionPanel pdp = new PowerDistributionPanel();
	
	public Winchester() {
		System.out.println( "Winchester");
    }
	public void set(double output){
		winchMotor.set( output );
	}
    
    public double getCurrent(){
    	return pdp.getCurrent( RobotMap.WINCH_PDP_PORT );
    }
    
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}