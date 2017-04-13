package org.usfirst.frc.team5587.robot.commands.groundbox;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.GroundBox;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class Regret extends TimedCommand {

	private GroundBox groundbox;
    public Regret() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super(1);
    	requires( Robot.groundbox );
    	groundbox = Robot.groundbox;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	groundbox.enableAngleCompensation();
    	groundbox.updatePID();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	groundbox.pid(95);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	groundbox.stopGrind();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
