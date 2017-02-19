package org.usfirst.frc.team5587.robot.commands.locomotive.auto;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Gyrate extends Command {

	private static final double ERROR_MARGIN = 0.5;
	private static final double GAIN = 0.0002;
	private static final double D_GAIN = 0.01;
	
	private double rotateAngle; //The target angle for the robot to rotate.
	private double yaw; //The current Gyroscope reading
	
	private double output; //The PWM output
	private double h0; //The average between the last value of h0 and the last output value that caused a change in the sign of the error
	
	private double error, //The current error
					error1, //The error from the last instance
					error2, //The error from 2 instances ago
					error3, //The error from 3 instances ago
					error4; //The error from 4 instances ago
	
	private double sign; //The sign of the current error
	private double sign0; //The sign of the last error.
	
	private Locomotive loco;
	
    public Gyrate( double target ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.loco );
    	loco = Robot.loco;
    	rotateAngle = target;
    	h0 = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loco.zeroYaw();
    	output = 0;
    	
    	yaw = loco.getYaw();
    	error = rotateAngle - yaw;
    	sign0 = Math.signum( error );
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	yaw = loco.getYaw();
    	error = rotateAngle - yaw;
    	
    	sign = Math.signum( error );
    	
    	if( sign == sign0 * -1.0 )
    	{
    		h0 = ( output + h0 ) / 2.0;
    		output = -h0;
    	}
    	else
    	{
    		output += GAIN * error + D_GAIN * ( error1 - error2 );
    	}
    	
    	if( output > 1.0 )
    		output = 1.0;
    	else if( output < -1.0 )
    		output = -1.0;
    	
    	SmartDashboard.putNumber( "Error: ", error );
    	SmartDashboard.putNumber( "Output: ", output );
    	SmartDashboard.putNumber( "H0: ", h0 );
    	
    	sign0 = sign;
    	
    	error4 = error3;
    	error3 = error2;
    	error2 = error1;
    	error1 = error;
    	loco.rotate( -output );
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return withinMargin();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    private boolean withinMargin()
    {
    	return Math.abs( error4 ) < ERROR_MARGIN 
            	&& Math.abs( error3 ) < ERROR_MARGIN
            	&& Math.abs( error2 ) < ERROR_MARGIN 
            	&& Math.abs( error1 ) < ERROR_MARGIN
            	&& Math.abs( error ) < ERROR_MARGIN;
    }
}
