package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Mortar extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private static final double DISTANCE_PER_PULSE = 1.0 / 12.0;
	private static final double MAX_SPEED = 4;
	
	
	private CANTalon flyWheel;
	private PIDController flyPID;
	
	double speedP = 1.2; 
	double speedI = 0.7;
	double speedD = 0.3; 
	
	public Mortar() {
		//Instantiate CANTalon 
		flyWheel = new CANTalon( RobotMap.FLYWHEEL_MOTOR_CAN_ID );
		//Instantiate the PID controller
		flyWheel.setPIDSourceType( PIDSourceType.kRate );
		flyPID = new PIDController( speedP, speedI, speedD, flyWheel, flyWheel );
		// TODO Auto-generated constructor stub
	}
	
	public void runPID( double targetSpeed ){
		flyPID.disable();
		targetSpeed= limit( targetSpeed );
		targetSpeed = targetSpeed / DISTANCE_PER_PULSE;
		flyPID.setSetpoint( targetSpeed );
		flyPID.enable();
	}
	
	private double limit( double targetSpeed ) {
		// TODO Auto-generated method stub
		return 0;
	}

	public void changeSpeed( double targetSpeed ){
		flyPID.setSetpoint( targetSpeed );
	}
    
	public void setPower( double pow ){
		flyPID.disable();
		flyWheel.set( pow ); 
	}
	
	public double getEncoderVal() {
		return flyWheel.getEncVelocity() * DISTANCE_PER_PULSE;
	}
	
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

