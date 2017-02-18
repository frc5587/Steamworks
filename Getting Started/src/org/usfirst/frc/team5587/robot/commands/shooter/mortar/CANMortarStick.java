package org.usfirst.frc.team5587.robot.commands.shooter.mortar;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.CANMortar;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CANMortarStick extends Command {

	private Joystick stick;
	private CANMortar mortar;
	
    public CANMortarStick( Joystick j ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		requires( Robot.mortarCAN );
		
		stick = j;
		mortar = Robot.mortarCAN;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	mortar.throttleMode();
    	mortar.updatePID();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {
    	mortar.spin( stick.getY() );
    	SmartDashboard.putNumber( "Encoder RPS: ", mortar.rpm() );
    	System.out.println( mortar.rpm() );
    	SmartDashboard.putNumber( "Encoder Distance", mortar.distance() );
    	SmartDashboard.putNumber( "Motor Output", mortar.output() );
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished()
    {
        return false;
    }

    // Called once after isFinished returns true
    protected void end()
    {
    	mortar.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted()
    {
    	end();
    }
}
