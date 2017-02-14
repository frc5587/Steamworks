package org.usfirst.frc.team5587.robot.commands.locomotive.auto;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveMarch extends Command {

	private Locomotive loco;
	private double distance;
	
    public DriveMarch( double dist ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.loco );
    	loco = Robot.loco;
    	distance = dist;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loco.setDriveDistance( distance );
    	loco.enableDriveDistance();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	loco.proceedForwards( loco.driveRate );
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
