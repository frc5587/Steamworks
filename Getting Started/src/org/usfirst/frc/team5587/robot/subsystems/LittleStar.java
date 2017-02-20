package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class LittleStar extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private PWM starR;
	private PWM starG;
	private PWM starB;
	
	public LittleStar()
	{
		starR = new PWM( RobotMap.STAR_R_PWM );
		starG = new PWM( RobotMap.STAR_G_PWM );
		starB = new PWM( RobotMap.STAR_B_PWM );
		
		starR.setPeriodMultiplier( PeriodMultiplier.k1X );
		starG.setPeriodMultiplier( PeriodMultiplier.k1X );
		starB.setPeriodMultiplier( PeriodMultiplier.k1X );
	}
	
	public void setRGB( double r, double g, double b )
	{
		int rVal = (int)(r*255);
		int gVal = (int)(g*255);
		int bVal = (int)(b*255);
		this.setRGB(rVal, gVal, bVal);
	}
	
	public void setRGB( int r, int g, int b )
	{
		starR.setRaw( r );
		starG.setRaw( g );
		starB.setRaw( b );
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

