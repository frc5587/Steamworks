package org.usfirst.frc.team5587.robot.commands.locomotive;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TankStick extends Command {

	Locomotive loco;
	Joystick stick;
	
    public TankStick( Joystick s ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.loco );
    	loco = Robot.loco;
    	stick = s;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loco.resetDistance();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	loco.tankDrive( stick );
    	loco.printEncoders();
    	loco.printCurrents();
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
