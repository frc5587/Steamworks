package org.usfirst.frc.team5587.robot.commands.locomotive;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Gyrate extends Command {

	private double rotateAngle;
	private double yaw;
	private static final double gain = .0003;
	private double output;
	private double h0;
	private double error;
	private double error1;
	private double error2;
	private double sign;
	private double sign0;
	
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
    		output += gain * error;
    	}
    	
    	if( output > 1.0 )
    		output = 1.0;
    	else if( output < -1.0 )
    		output = -1.0;
    	
    	SmartDashboard.putNumber( "Error: ", error );
    	SmartDashboard.putNumber( "Output: ", output );
    	SmartDashboard.putNumber( "H0: ", h0 );
    	
    	sign0 = sign;
    	error2 = error1;
    	error1 = error;
    	loco.rotate( -output );
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return error1 == error2 && error1 == error && Math.abs( error ) < 1.0;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println( "Finished bitch! ");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
