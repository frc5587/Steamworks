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

	private double rotateAngle, turnValue = .5;
	private double yaw;
	private double narrowLimit = 1.0, broadLimit = 4.0;
	private boolean broadened;
	private static final double gain = .00003;
	private double output;
	private double lastOutput;
	private double h0;
	private double error;
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
    	broadened = false;
    	output = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	yaw = loco.getYaw();
    	
    	if( broadened )
    	{
    		if( ( yaw < rotateAngle + broadLimit ) && ( yaw > rotateAngle - broadLimit ) )
    			loco.rotate( 0.0 );
    		else
    			broadened = false;
    	}
    	else
    	{
    		if( Math.abs( rotateAngle - yaw ) > 180.0 )
    			if( yaw > rotateAngle + narrowLimit )
    				loco.rotate( -turnValue );
    			else if( yaw < rotateAngle - narrowLimit )
    				loco.rotate( turnValue );
    		if( yaw > rotateAngle + 1 )
    			loco.rotate( turnValue );
    		else if( yaw < rotateAngle - 1 )
    			loco.rotate( -turnValue );
    		else
    			broadened = true;
    	}
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
