package org.usfirst.frc.team5587.robot.commands.shooter.mortar;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.CANMortar;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CANMortarPID extends Command {

	private CANMortar mortar;
	private double targetRate;
	
    public CANMortarPID() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.mortarCAN );
    	mortar = Robot.mortarCAN;
    	
    	SmartDashboard.putNumber( "CAN Target Rate: ", 0.0 );
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	mortar.speedMode();
    	mortar.updatePID();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	targetRate = SmartDashboard.getNumber( "CAN Target Rate: ", 0.0 );
    	
    	mortar.spin( targetRate );
    	SmartDashboard.putNumber( "Encoder RPS: ", mortar.rps() );
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