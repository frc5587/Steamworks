package org.usfirst.frc.team5587.robot.commands.groundbox;

import org.usfirst.frc.team5587.classes.NetworkTable;
import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.GroundBox;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GroundManual extends Command {

	private GroundBox groundbox;
	private NetworkTable table;
	
    public GroundManual() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.groundbox );
    	groundbox = Robot.groundbox;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	table = NetworkTable.getTable("arm");
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	groundbox.grindManual((int) SmartDashboard.getNumber("Box Angle", 0));
    	System.out.println("Running.");
    	table.putNumber("val", groundbox.getDegrees());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//groundbox.stopGrind();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
