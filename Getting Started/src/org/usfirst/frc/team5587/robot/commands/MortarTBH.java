package org.usfirst.frc.team5587.robot.commands;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;
import org.usfirst.frc.team5587.robot.subsystems.Mortar;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class MortarTBH extends Command {
	
	private static final double GAIN = .05;
	private static final double D_GAIN = 0.0;
	
	private double targetRate; //The target angle for the robot to rotate.
	private double rate; //The current Gyroscope reading
	
	private double output; //The PWM output
	private double h0; //The average between the last value of h0 and the last output value that caused a change in the sign of the error
	
	private double error, //The current error
					error1; //The error from the last instance
	
	private double sign; //The sign of the current error
	private double sign0; //The sign of the last error.
	
	private Mortar joey;

    public MortarTBH( double target ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.mortar );
    	joey = Robot.mortar;
    	targetRate = target;
    	h0 = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	SmartDashboard.putNumber( "GAIN: ", 0.05 );
    	SmartDashboard.putNumber( "D_Gain", 0.1 );
    	
    	output = 0;
    	rate = joey.RPS();
    	
    	error = targetRate - rate;
    	sign0 = Math.signum( error );
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {    	
    	rate = joey.RPS();
    	error = targetRate - rate;
    	
    	sign = Math.signum( error );
    	
    	if( sign == sign0 * -1.0 )
    	{
    		h0 = ( output + h0 ) / 2.0;
    		output = h0;
    	}
    	else
    	{
    		output += SmartDashboard.getNumber( "GAIN: ", 0.05 ) * error + SmartDashboard.getNumber( "D_Gain", 0.0 ) * ( error - error1 );
    	}
    	
    	if( output > .7 )
    		output = .7;
    	else if( output < -.7 )
    		output = -.7;
    	
    	SmartDashboard.putNumber( "Error: ", error );
    	SmartDashboard.putNumber( "Output: ", output );
    	SmartDashboard.putNumber( "H0: ", h0 );
    	SmartDashboard.putNumber( "Joey: ", joey.RPS() );
    	
    	sign0 = sign;
    	
    	error1 = error;
    	joey.spin( output );
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
