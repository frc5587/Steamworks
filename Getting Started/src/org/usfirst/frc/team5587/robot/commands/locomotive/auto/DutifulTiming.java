package org.usfirst.frc.team5587.robot.commands.locomotive.auto;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DutifulTiming extends Command {

	private static double kCorrection = 0.12;
	private double targetDistance, driftAngle, originalHeading, direction, killTime;
	private Timer timer;
	
	private Locomotive loco;
	private double power = -0.4;
	
	/**
	 * DutifulProgression takes the robot along a straight line a given distance.
	 * 
	 * @param distance The distance, in inches, we want the robot to travel.
	 */
    public DutifulTiming( double time ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	//requires( Robot.loco );
    	loco = Robot.loco;
    	SmartDashboard.putNumber( "Correction Coefficient", kCorrection );
    	killTime = time;
    	timer = new Timer();
    }
    
    public DutifulTiming( double time, int dir ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	//requires( Robot.loco );
    	loco = Robot.loco;
    	SmartDashboard.putNumber( "Correction Coefficient", kCorrection );
    	killTime = time;
    	timer = new Timer();
    	power *= dir;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loco.resetDistance();
    	loco.zeroYaw();
    	loco.disableDriverControl();
    	originalHeading = loco.getYaw();
    	timer.start();
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
    	if( timer.get() > killTime )
    		return true;
    	else{
    		return false;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    	loco.halt();
    	loco.enableDriverControl();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
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
