package org.usfirst.frc.team5587.robot.commands.shooter.mortar;

import org.usfirst.frc.team5587.classes.NetworkTable;
import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.CANMortar;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;

/**
 *
 */
public class CANMortarPID extends Command {

	private CANMortar mortar;
	private double targetRate;
	private NetworkTable table;
	
    public CANMortarPID() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.mortarCAN );
    	mortar = Robot.mortarCAN;
    	
    	SmartDashboard.putNumber( "CAN Target Rate: ", -4000.0 );
    	
    	SmartDashboard.putNumber("Velocity: ", 0.0);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	mortar.speedMode();
    	//mortar.updatePID();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	targetRate = SmartDashboard.getNumber( "CAN Target Rate: ", -4000.0 );
    	
    	SmartDashboard.putNumber("Ball Distance: ", ballDistance(75.0, SmartDashboard.getNumber("Velocity: ", 0.0)));
    	
    	mortar.spin( targetRate );
    	SmartDashboard.putNumber( "Encoder RPS: ", mortar.rps() );
    	SmartDashboard.putNumber( "Motor Output", mortar.output() );
    	
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
    
    private double ballDistance(double angle, double velocity) {
    	double distance = 0;
    	//gravity acceleration
    	double g = 9.80665;
    	//Set angle to radians for sin calc
    	angle = Math.toRadians(angle);
    	
    	distance = (Math.pow(velocity, 2)/g)*Math.toDegrees(Math.sin(2*angle));
    	
    	return distance;
    }
    
    private double velocityNeeded(double angle, double distance) {
    	double initialVelocity = 0;
    	//gravity acceleration
    	double g = 9.80665;
    	//Set angle to radians for sin calc
    	angle = Math.toRadians(angle);
    	
    	//Based on equation d = ((v^2)/g)*sin(2*angle)
    	initialVelocity = Math.sqrt((distance*g)/(Math.toDegrees(2*Math.sin(angle))));
    	
    	return initialVelocity;
    }
}
