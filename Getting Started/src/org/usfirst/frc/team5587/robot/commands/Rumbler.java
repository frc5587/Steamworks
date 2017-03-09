package org.usfirst.frc.team5587.robot.commands;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.GroundBox;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Rumbler extends Command {

	private boolean lastHas;
	private Timer timer;
	private GroundBox groundbox;
	
    public Rumbler() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	lastHas = false;
    	timer = new Timer();
    	groundbox = Robot.groundbox;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if( groundbox.hasGear() != lastHas )
    	{
    		timer.stop();
    		timer.reset();
    		timer.start();
    		if( groundbox.hasGear() )
    		{
    			Robot.oi.driver.setRumble( RumbleType.kLeftRumble, 1.0 );
    		}
    		else
    		{
    			Robot.oi.driver.setRumble( RumbleType.kRightRumble, 1.0 );
    		}
    	}
    	else if( timer.get() > .5 )
    	{
    		timer.stop();
    		Robot.oi.driver.setRumble( RumbleType.kLeftRumble, 0.0 );
    		Robot.oi.driver.setRumble( RumbleType.kRightRumble, 0.0 );
    	}
    	
    	lastHas = groundbox.hasGear();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
