package org.usfirst.frc.team5587.robot.commands.shooter.mortar;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Mortar;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The premise of the TBH (Take Back Half) loop is to adjust output based on error.
 * h0 represents the last registered value that changed the error of the sign.
 * Output is a value that is constantly updated based on error.
 * Every time the loop is run, it checks the sign of the error. If the sign of the
 * error has changed since the last time the loop was run, the output is set to the
 * average between the last value of output and the last value of h0.
 * If the sign of the error has not changed, the product of the gain value and the
 * current error is added to the output.
 * With an appropriate gain, the TBH loop is stable and will always converge on the
 * setpoint. By adding a gain term that changes the output based on the change in
 * error, we are able to change how quickly the loop reaches the setpoint. 
 */
public class MortarTBH extends Command {
	
	private static final double GAIN = .05; //A constant that needs to be tested
	private static final double D_GAIN = 0.0; //A constant that needs to be tested
	private static final double D2_GAIN = 0.0;
	private static final double ERROR_MARGIN = .5; //How much we're willing to be off by.
	private static final double OUTPUT_CAP = 1.0; //The maximum power we want the motor to run (Will probably stay at 1)
	
	private double targetRate; //The target rate for the flywheel to spin
	private double rate; //The current encoder rate reading.
	
	private double output; //The PWM output
	private double h0; //The average between the last value of h0 and the last output value that caused a change in the sign of the error
	
	//Errors.
	private double error, //The current error. Used to calculate change in output based on current error.
					error1, //The error from the last instance of the loop. Used to calculate change in output based on derivative of error.
					error2, //The error from two loop instances ago. May be used to calculate change in output based on second derivative of error.
					error3, //The error from three loop instances ago
					error4; //The error from four loop instances ago
	
	private double sign; //The sign of the current error
	private double sign0; //The sign of the last error.
	
	private Mortar joey;

    public MortarTBH( double target )
    {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.mortar );
    	joey = Robot.mortar;
    	targetRate = target;
    	h0 = 0.0; //Setting h0 equal to the output we want to start at.
    }

    // Called just before this Command runs the first time
    protected void initialize()
    {
    	
    	output = h0; //Set output equal to whatever we set our initial output equal to.
    	rate = joey.rps(); //Read the encoder value from Joey
    	
    	error = targetRate - rate; //Get the first error
    	sign0 = Math.signum( error ); //Get the sign of the first error.
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {    	
    	rate = joey.rps(); //Update the encoder rate
    	
    	error = targetRate - rate; //Update the current error value
    	
    	sign = Math.signum( error ); //Get the sign of the current error.
    	
    	if( sign == sign0 * -1.0 ) //If the sign of the error has changed
    	{
    		h0 = ( output + h0 ) / 2.0; //Set h0 equal to the average of the last output and the last value of h0 (here's the take back half)
    		output = h0; //Set output equal to your new h0.
    	}
    	else //Else, change the current output.
    	{
    	}
    	
    	//Limiting the output to what is allowed
    	if( output > OUTPUT_CAP )
    		output = OUTPUT_CAP;
    	else if( output < -OUTPUT_CAP )
    		output = -OUTPUT_CAP;
    	
    	
    	//Update sign0 to be the current value of sign
    	sign0 = sign;
    	
    	//Update all the cached errors
    	error4 = error3;
    	error3 = error2;
    	error2 = error1;
    	error1 = error;
    	joey.spin( output );
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished()
    {
        return false;
    }

    // Called once after isFinished returns true
    protected void end()
    {
    	joey.spin( 0.0 );
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted()
    {
    	end();
    }   
    
    /**
     * Checks to see if the last five registered errors are within the acceptable margin of error.
     * This is to allow the loop to find a range it likes to settle in.
     * 
     * @return Whether the last five registered errors are within the acceptable margin of error.
     */
    private boolean withinMargin()
    {
    	return Math.abs( error4 ) < ERROR_MARGIN 
            	&& Math.abs( error3 ) < ERROR_MARGIN
            	&& Math.abs( error2 ) < ERROR_MARGIN 
            	&& Math.abs( error1 ) < ERROR_MARGIN
            	&& Math.abs( error ) < ERROR_MARGIN;
    }
}
