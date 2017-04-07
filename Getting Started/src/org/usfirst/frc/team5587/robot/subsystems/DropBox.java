package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class DropBox extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private Servo servo;
	
	public DropBox() {
		servo = new Servo( RobotMap.DROP_BOX_SERVO );
		LiveWindow.addActuator( "DropBox", "Servo", servo );
	}
	
	public void drop() {
		servo.setAngle( 180.0 );
	}
	
	public void box() {
		servo.setAngle( 0.0 );
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

