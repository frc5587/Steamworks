package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Winch extends Subsystem {
	
	private final VictorSP winchMotor = new VictorSP(RobotMap.WINCH_MOTOR);
	private boolean encDir = true; //Encoder Direction
	private final Encoder winchEnc = new Encoder(RobotMap.WINCH_ENCODER_A, RobotMap.WINCH_ENCODER_B, encDir, EncodingType.k4X);
	
	
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}
