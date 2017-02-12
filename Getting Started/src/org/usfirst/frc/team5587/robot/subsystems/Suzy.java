package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.classes.ADXRS450Gyro;
import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.RobotMap;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * This subsystem contains the motors in the drive train
 */
public class Suzy extends Subsystem implements PIDOutput
{
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	private static double DISTANCE_PER_PULSE = ( 360.0 / 1024.0 );
	
	private Spark motor;
	private Encoder encoder;
	private ADXRS450Gyro gyro;
	
	private boolean onTarget;
	
	PIDController turnController;
	private double rotateToAngleRate;
    private boolean usingPID;
    
	private static final double kP = 0.001;
	private static final double kI = 0.0;
	private static final double kD = 0.0;
	private static final double kF = 0.0;
	private static final double kToleranceDegrees = 20;
	
	//Creates a new DriveTrain object and initializes the RobotDrive driveTrain 
	public Suzy()
	{
		motor = new Spark( RobotMap.SUZY_MOTOR );
		encoder = new Encoder( RobotMap.SUZY_ENC_A, RobotMap.SUZY_ENC_B, false, EncodingType.k4X );
		encoder.setDistancePerPulse( DISTANCE_PER_PULSE );
		encoder.setReverseDirection( true );
		
		gyro = new ADXRS450Gyro();
		gyro.startThread();
		
		onTarget = false;
		
		//Begin 614 pid code
		turnController = new PIDController(kP, kI, kD, kF, encoder, this);
		//turnController.setInputRange(0.0f,  360.0f);
		turnController.setOutputRange(-0.4, 0.4);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        //turnController.setContinuous(true);

        /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
        /* tuning of the Turn Controller's P, I and D coefficients.            */
        /* Typically, only the P value needs to be modified.                   */
        LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
	}
	
	/*
	 * Brings robot into motion based on numerical input
	 * 
	 * @param pwr The power level on which to run the drive train motors ( -1 <= pwr <= 1 )
	 */
	public void set( double pwr)
	{
		motor.set( -pwr );
	}
	public double getEncAngle( )
	{
		return encoder.getDistance();
	}
	
	public void stop()
	{
		set( 0.0 );
	}
	
	public void setOnTarget( boolean b )
	{
		onTarget = b;
	}
	
	public boolean onTarget()
	{
		return onTarget;
	}
	public void setUsingPID(boolean set) {
		usingPID = set;
		if(set == true) {
			turnController.enable();
		} else {
			turnController.disable();
		}
	}
	public boolean getUsingPID() {
		return usingPID;
	}
	public double getRotateRate() {
		return rotateToAngleRate;
	}
	public PIDController getController() {
		return turnController;
	}

	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}
	
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}