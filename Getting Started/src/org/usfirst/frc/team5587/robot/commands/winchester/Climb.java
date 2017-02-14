package org.usfirst.frc.team5587.robot.commands.winchester;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Winchester;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 *
 */
public class Climb extends Command {

	private NetworkTable table;
	private Winchester winch;
	
	private static final double KILL_CURRENT = 0.0;
	
    public Climb() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.winch );
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	table = NetworkTable.getTable( "Winch" );
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	winch.set( 1.0 );
    	table.putNumber( "Winch Current Draw", winch.getCurrent() );
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if( winch.getCurrent() >= KILL_CURRENT){
    		return true;
    	}
    	else{
    		return false;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    	winch.set( 0.0 );
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
