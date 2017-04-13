package org.usfirst.frc.team5587.robot.commands.locomotive;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriverInput extends Command {

	private Locomotive loco;
	private boolean enabled;
	
    public DriverInput(boolean e) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	loco = Robot.loco;
    	enabled = e;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(enabled){
    		loco.enableDriverControl();
    	}
    	else if(!enabled){
    		loco.disableDriverControl();
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
