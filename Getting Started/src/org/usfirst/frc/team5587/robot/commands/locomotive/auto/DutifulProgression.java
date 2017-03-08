package org.usfirst.frc.team5587.robot.commands.locomotive.auto;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DutifulProgression extends Command {

	private static double kCorrection = 0.12;
	private double targetDistance, driftAngle, originalHeading, direction;
	
	private Locomotive loco;
	private double power = 0.3;
	
	/**
	 * DutifulProgression takes the robot along a straight line a given distance.
	 * 
	 * @param distance The distance, in inches, we want the robot to travel.
	 */
    public DutifulProgression( double distance ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.loco );
    	loco = Robot.loco;
    	targetDistance = distance; 
    	direction = Math.signum( targetDistance );
    	power *= direction;
    	SmartDashboard.putNumber( "Correction Coefficient", kCorrection );
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loco.resetDistance();
    	loco.zeroYaw();
    	originalHeading = loco.getYaw();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {
    	driftAngle = loco.getYaw() - originalHeading;
    	loco.printEncoders();
    	SmartDashboard.putNumber( "Passive Drift", driftAngle );
    	loco.keepPace( power, driftAngle * SmartDashboard.getNumber( "Correction Coefficient", kCorrection ) );
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if( direction > 0 )
    		return loco.getDistance() >= targetDistance;
    	else if( direction < 0 )
    		return loco.getDistance() <= targetDistance;
    	else
    		return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	loco.halt();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    /**
     * 1074, 57.5"
     * 1075, 59"
     * 1082, 60"
     * 1081, 58.5"
     * 1069, 61"
     * 
     * 1076.2, 59.2
     */
}
