package org.usfirst.frc.team5587.robot.commands;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GyroRotate extends Command {

	private double rotateAngle;
	private Locomotive loco;
	
    public GyroRotate( double angle ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.loco );
    	loco = Robot.loco;
    	rotateAngle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loco.rotate( rotateAngle );
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return loco.isAtAngle();
    }

    // Called once after isFinished returns true
    protected void end() {
    	loco.disablePID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
