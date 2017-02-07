package org.usfirst.frc.team5587.robot.commands;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.LittleStar;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TwinkleTwinkle extends Command {

	private LittleStar orion;
	
	private double red, green, blue;
	
    public TwinkleTwinkle() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.orion );
    	
    	orion = Robot.orion;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	SmartDashboard.putNumber( "Red: ", 0 );
    	SmartDashboard.putNumber( "Green: ", 0 );
    	SmartDashboard.putNumber( "Blue: ", 0 );
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	red = SmartDashboard.getNumber( "Red: ", 0 );
    	green = SmartDashboard.getNumber( "Green: ", 0 );
    	blue = SmartDashboard.getNumber( "Blue: ", 0 );
    	
    	orion.setRGB( red, green, blue );
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
