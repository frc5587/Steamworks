package org.usfirst.frc.team5587.robot.commands;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GyroThrottle extends Command {

	private double rotateAngle, turnValue = .5;
	private double yaw;
	private double narrowLimit = 1.0, broadLimit = 4.0;
	private boolean broadened;
	private Locomotive loco;
	
    public GyroThrottle() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.loco );
    	loco = Robot.loco;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loco.zeroYaw();
    	broadened = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

//    	throttleValue = stick.getThrottle();
//    	rotateAngle = throttleValue * -180.0;
//    	rotateAngle = (int)(rotateAngle);
    	rotateAngle = SmartDashboard.getNumber( "Manual Control", 10.0 );
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
