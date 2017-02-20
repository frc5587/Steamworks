package org.usfirst.frc.team5587.robot.commands;

import org.usfirst.frc.team5587.classes.NetworkTable;
import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.CANSuzy;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TestTurntableScale extends Command {

	private CANSuzy suzyQ;
	private NetworkTable table2;
	
    public TestTurntableScale() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.suzyCAN );
    	suzyQ = Robot.suzyCAN;
    	table2 = NetworkTable.getTable( "angle thingy" );
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	suzyQ.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	suzyQ.setRaw( (int)table2.getNumber( "PID Angle", 0.0 ) );
    	
    	table2.putNumber( "Encoder Position: ", suzyQ.getPosition() );//suzyQ.getEncAngle());
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
