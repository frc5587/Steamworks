package org.usfirst.frc.team5587.robot.commands;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DutifulProgression extends Command {

	private double targetDistance;
	private Locomotive loco;
	
	/**
	 * DutifulProgression takes the robot along a straight line a given distance.
	 * 
	 * @param distance The distance, in feet, we want the robot to travel.
	 */
    public DutifulProgression( double distance ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.loco );
    	loco = Robot.loco;
    	targetDistance = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loco.resetDistance();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {
    	loco.proceedForwards();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return loco.getDistance() >= targetDistance;
    }

    // Called once after isFinished returns true
    protected void end() {
    	loco.halt();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
