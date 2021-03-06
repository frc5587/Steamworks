package org.usfirst.frc.team5587.robot.commands.groundbox;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.GroundBox;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ResetEncArm extends Command {

	private GroundBox groundbox;
	
    public ResetEncArm() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	groundbox = Robot.groundbox;
    	requires(groundbox);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	groundbox.grindReset();
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
