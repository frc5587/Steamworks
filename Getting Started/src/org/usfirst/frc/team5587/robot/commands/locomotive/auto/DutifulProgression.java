package org.usfirst.frc.team5587.robot.commands.locomotive.auto;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DutifulProgression extends Command {

	private double targetDistance, sign;
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
    	sign = Math.signum( targetDistance );
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loco.resetDistance();
    	loco.zeroYaw();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {
    	loco.proceedForwards( sign * 0.3 );
    	loco.printEncoders();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if( sign > 1 )
    		return loco.getDistance() >= targetDistance;
    	else
    		return loco.getDistance() <= targetDistance;
    }

    // Called once after isFinished returns true
    protected void end() {
    	SmartDashboard.putNumber( "Passive Drift", loco.getYaw() );
    	loco.halt();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
