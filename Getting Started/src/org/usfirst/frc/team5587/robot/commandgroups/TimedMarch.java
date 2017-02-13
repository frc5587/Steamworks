package org.usfirst.frc.team5587.robot.commandgroups;

import org.usfirst.frc.team5587.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TimedMarch extends Command {

	double timeTarget, power;
	Timer timer;
	
    public TimedMarch( double p, double t ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.loco );
    	
    	power = p;
    	timeTarget = t;
    	timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.loco.tankDrive( power, power );
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return timer.get() >= timeTarget;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.loco.halt();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
