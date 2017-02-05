package org.usfirst.frc.team5587.robot.commands;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Retator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Etator extends Command {

	private static final double ERROR_MARGIN = 0.5;
	private static final double GAIN = .0003;
	private static final double D_GAIN = 0.01;
	
	private double rotateAngle; //The target angle for the robot to rotate.
	private double angle; //The current encoder angle reading
	
	private double output; //The PWM output
	private double h0; //The average between the last value of h0 and the last output value that caused a change in the sign of the error
	
	private double error, //The current error
					error1, //The error from the last instance
					error2, //The error from 2 instances ago
					error3, //The error from 3 instances ago
					error4; //The error from 4 instances ago
	
	private double sign; //The sign of the current error
	private double sign0; //The sign of the last error.
	
	private Retator retator;
	
    public Etator(double target) {
    	// Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.retator );
    	retator = Robot.retator;
    	rotateAngle = target;
    	h0 = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	output = 0;

    	angle = retator.get();
    	error = rotateAngle - angle;
    	sign0 = Math.signum( error );
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	angle = retator.get();
    	error = rotateAngle - angle;
    	
    	sign = Math.signum( error );
    	
    	if( error4 == error3
        	&& error3 == error2
        	&& error1 == error2
        	&& error1 == error
        	&& Math.abs( error ) < ERROR_MARGIN )
    	{
    		output = 0.0;
    	}
    	else
    	{
	    	if( sign == sign0 * -1.0 )
	    	{
	    		h0 = ( output + h0 ) / 2.0;
	    		output = -h0;
	    	}
	    	else
	    	{
	    		output += GAIN * error + D_GAIN * ( error - error1 );
	    	}
	    	
	    	if( output > 1.0 )
	    		output = 1.0;
	    	else if( output < -1.0 )
	    		output = -1.0;
	    	
	    	SmartDashboard.putNumber( "Turntable Error: ", error );
	    	SmartDashboard.putNumber( "Turntable Output: ", output );
	    	SmartDashboard.putNumber( "Turntable H0: ", h0 );
	    	
	    	sign0 = sign;
    	}
    	
    	error4 = error3;
    	error3 = error2;
    	error2 = error1;
    	error1 = error;
    	retator.set( -output );
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
