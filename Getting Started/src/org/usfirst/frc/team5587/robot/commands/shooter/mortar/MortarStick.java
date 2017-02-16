package org.usfirst.frc.team5587.robot.commands.shooter.mortar;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Mortar;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Used to drive the flywheel directly using the joystick.
 * Also prints out the readings on the encoder.
 */
public class MortarStick extends Command {

	private Joystick stick;
	private Mortar mortar;
	
    public MortarStick( Joystick j ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.mortar );
    	
    	stick = j;
    	mortar = Robot.mortar;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	mortar.spin( stick.getY() );
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end()
    {
    	mortar.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
