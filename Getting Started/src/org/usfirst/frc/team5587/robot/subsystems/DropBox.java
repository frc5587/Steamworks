package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;
import org.usfirst.frc.team5587.robot.commands.SetDrop;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class DropBox extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private Servo folder;
	private Servo gateKeeper;
	
	public DropBox() {
		folder = new Servo( RobotMap.DROP_BOX_FOLDER );
		gateKeeper = new Servo( RobotMap.DROP_BOX_GATEKEEPER );
		LiveWindow.addActuator( "DropBox", "Servo", folder );
	}
	
	public void drop() {
		gateKeeper.set( 0.0 );
		folder.set( 0.0 );
	}
	
	public void box() {
		folder.set( 0.8 );
	}
	
	public void fold() {
		folder.set( 1.0 );
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}