package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Mortar extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private CANTalon flyWheel;
	private PIDController flyPID;
	
	double speedP = 1.2; 
	double speedI = 0.7;
	double speedD = 0.3; 
	
	public Mortar() {
		//Instantiate CANTalon 
		flyWheel = new CANTalon( RobotMap.FLYWHEEL_MOTOR_CAN_ID);
		//Instantiate the PID controller 
		flyPID = new PIDController(speedP, speedI, speedD, flyWheel, flyWheel);
		// TODO Auto-generated constructor stub
	}
	public void runPID(double targetPos){
		flyPID.disable();
		flyPID.setSetpoint(targetPos);
		flyPID.enable();
		
	}
    
	public void setPower (double pow){
		flyPID.disable();
		flyWheel.set(pow); 
	}
	public double getEncoderVal() {
		return flyWheel.getPosition();
	}
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

