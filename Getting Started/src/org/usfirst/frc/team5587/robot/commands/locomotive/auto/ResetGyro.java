package org.usfirst.frc.team5587.robot.commands.locomotive.auto;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ResetGyro extends Command {

	private Locomotive loco; 
	private double offset;
	
    public ResetGyro(double o) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	loco = Robot.loco;
    	offset = o;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loco.setYaw(offset);
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
