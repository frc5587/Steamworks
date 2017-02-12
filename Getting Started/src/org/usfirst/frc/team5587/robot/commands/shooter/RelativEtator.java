package org.usfirst.frc.team5587.robot.commands.shooter;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Suzy;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RelativEtator extends Command {
	
	private static final String NETWORKTABLES_TABLE_NAME = "/GRIP/postprocessed";
	private static final String NETWORKTABLES_ANGLE_NAME = "x angles";

	private NetworkTable table;
	private NetworkTable table2;
	
	private double [] angles;
	private double rotateAngle; //The target angle for the robot to rotate.
	private double angle; //The current encoder angle reading
	
	private Suzy suzyQ;
	
    public RelativEtator() {
    	// Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.suzyQ );
    	suzyQ = Robot.suzyQ;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//    	table = NetworkTable.getTable( NETWORKTABLES_TABLE_NAME );
    	table2 = NetworkTable.getTable( "angle thingy" );
    	
    	suzyQ.zeroEnc();
    	suzyQ.setUsingPID(true);
    	
    	angle = suzyQ.getEncAngle();
    	rotateAngle = angle + table2.getNumber( "Relative PID Angle", 3000.0 );

    	suzyQ.getController().setSetpoint( rotateAngle );
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	suzyQ.set(suzyQ.getRotateRate());
    	SmartDashboard.putNumber("encoder val", suzyQ.getEncAngle());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished(){
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	suzyQ.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	suzyQ.setUsingPID(false);
    	suzyQ.stop();
    }
}
