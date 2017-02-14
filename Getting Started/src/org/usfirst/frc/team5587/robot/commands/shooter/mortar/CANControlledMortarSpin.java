package org.usfirst.frc.team5587.robot.commands.shooter.mortar;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.CANMortar;
import org.usfirst.frc.team5587.robot.subsystems.Mortar;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CANControlledMortarSpin extends Command {

	private Joystick stick;
	private CANMortar mortar;
	
    public CANControlledMortarSpin( Joystick j ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		requires( Robot.mortarCAN );
		
		stick = j;
		mortar = Robot.mortarCAN;
		
		SmartDashboard.putNumber( "CAN Target Power Level: ", 0.0 );
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	mortar.throttleMode();
    	mortar.updatePID();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {
    	mortar.spin( SmartDashboard.getNumber( "CAN Target Power Level: ", 0.0 ) );
    	SmartDashboard.putNumber( "Encoder RPS", mortar.rps() );
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
