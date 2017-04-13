package org.usfirst.frc.team5587.robot.commands.shooter.turntable;

import org.usfirst.frc.team5587.classes.NetworkTable;
import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.CANSuzy;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CANRelativEtator extends Command {

	private static final String NETWORKTABLES_TABLE_NAME = "/GRIP/postprocessed";
	private static final String NETWORKTABLES_ANGLE_NAME = "x angles";

	private NetworkTable table2;
	
	private double [] angles;
	private double rotateAngle; //The target angle for the robot to rotate.
	private double angle; //The current encoder angle reading
	
	private CANSuzy suzyQ;
	
    public CANRelativEtator()
    {
    	// Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.suzyCAN );
    	suzyQ = Robot.suzyCAN;
    	
    	table2 = NetworkTable.getTable( "angle thingy" );
    	table2.putNumber( "Relative PID Angle: ", 0.0 );
    }

    // Called just before this Command runs the first time
    protected void initialize()
    {
//    	table = NetworkTable.getTable( NETWORKTABLES_TABLE_NAME );
    	
    	suzyQ.updatePID();
    	suzyQ.setRelativePosition( table2.getNumber( "Relative PID Angle: ", 0.0 ) );
    	suzyQ.enable();
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {
    	SmartDashboard.putNumber( "Encoder Position: ", suzyQ.getPosition() );
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished()
    {
        return false;
    }

    // Called once after isFinished returns true
    protected void end()
    {
    	suzyQ.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted()
    {
    	end();
    }
}
