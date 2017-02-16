package org.usfirst.frc.team5587.robot.commands.shooter.mortar;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.CANMortar;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CANMortarSpinAtRate extends Command {

	private CANMortar mortar;
	private double targetRate;
	
    public CANMortarSpinAtRate() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.mortarCAN );
    	mortar = Robot.mortarCAN;
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	mortar.speedMode();
    	mortar.updatePID();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	mortar.spin( targetRate );
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
