package org.usfirst.frc.team5587.robot.subsystems;

import org.usfirst.frc.team5587.classes.NetworkTable;
import org.usfirst.frc.team5587.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GroundBox extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private CANTalon articules;
	private VictorSP highRoller;
	private DigitalInput gearSwitch;

	private static double ROLL_POWER = 0.35;
	
	private static final double UP_POSITION = 70;
	private static final double DELIVER_POSITION = 45;
	private static final double DOWN_POSITION = 0;
	private static final int ERROR_MARGIN = 5;
	private static final int CURRENT_LIMIT = 40;
	
	private double kP,
							kI,
							kD,
							kF;
	
	public GroundBox()
	{
		articules = new CANTalon( RobotMap.ARTICULES_MOTOR_CAN_ID );
		highRoller = new VictorSP( RobotMap.ROLLER_MOTOR );
		gearSwitch = new DigitalInput( RobotMap.GEAR_SWITCH );
		
		articules.changeControlMode( TalonControlMode.Position );
		articules.setFeedbackDevice( FeedbackDevice.CtreMagEncoder_Absolute );
		articules.enableZeroSensorPositionOnForwardLimit(true);
		articules.reverseOutput(true);
		
		articules.configNominalOutputVoltage( +0.0f, -0.0f );
		articules.configPeakOutputVoltage( +0.0f, -12.0f );
		
		articules.setP( kP );
		articules.setI( kI );
		articules.setD( kD );
		articules.setF( kF );
		
		articules.setAllowableClosedLoopErr(ERROR_MARGIN);
		articules.setCurrentLimit(CURRENT_LIMIT);
		articules.EnableCurrentLimit(true);
	}
	
	public double getPosition()
	{
		return articules.getPosition();
	}
	
	public double getDegrees()
	{
		return getPosition() * 360.0;
	}
	
	public boolean hasGear()
	{
		return gearSwitch.get(); 
	}
	
	public void grindUp()
	{
		grindManual( UP_POSITION );
	}
	
	public void grindDown()
	{
		grindManual( DOWN_POSITION );
	}
	
	public void deliveryNotDigiorno()
	{
		articules.setSetpoint( DELIVER_POSITION );
	}
	
	public void grindManual(double pos)
	{
		articules.setSetpoint( pos/360 );
	}
	
	public void grindReset()
	{
		articules.setPosition(0);
	}
	public void stopGrind(){
		articules.disableControl();
	}
	
	public void succ()
	{
		highRoller.set( ROLL_POWER );
		
	}
	
	public void rollOut()
	{
		highRoller.set( -ROLL_POWER );
	}
	
	public void stopRolling()
	{
		highRoller.stopMotor();
	}

	//Custom PID Section
	private NetworkTable armTable = NetworkTable.getTable("arm");
	
	private double error = 0;
	private double sumError = 0;
	private double deltaError = 0;
	private static double deadband = 1;
	private double pidOutput = 0;
	private double restAngle = 0; //this is the angle that the groundbox would naturally fall down to
	
	public void enableAngleCompensation(){
		articules.changeControlMode( TalonControlMode.Voltage );
	}
	
	public void updatePID(){
		kF = armTable.getNumber( "kF");
		kP = armTable.getNumber( "kP");
		armTable.putNumber("working", 1);
		System.out.println(armTable.getNumber( "kP" ));
		kI = armTable.getNumber( "kI");
		kD = armTable.getNumber( "kD");
		
		error = 0;
		sumError = 0;
		deltaError = 0;
	}
	
	public void pid( double setpoint ){
		armTable.putNumber("angle", getDegrees());
		double lastError = error;
		double radians = getDegrees()/180.0 * Math.PI;
		double restRadians = restAngle/180.0 * Math.PI;
		
		error = setpoint - getDegrees();
		sumError += error;
		
		if(lastError == 0.0)	//I'm tired and unsure if this is necessary, 
			deltaError = 0;		//but I want to make sure that there isn't any ridiculous damping out of the gate
		else
			deltaError = error-lastError;
		
			
		if(sumError*kI > 1){
			sumError = 1.0/kI;
		}
		
		if(Math.abs(error) > deadband){
			pidOutput = kP*error+ kI*sumError + kD*deltaError + kF*Math.sin(radians-restRadians);
		}
		articules.set(pidOutput*12); //multiplied by 12 for 12v output
		System.out.println("kP: " + kP);
		System.out.println("Output: " + pidOutput);
	}
	
	public void updateP( double kP ){
		articules.setP(kP);
	}
	public void updateI( double kI){
		articules.setI(kI);
	}
	public void updateD( double kD ){
		articules.setD(kD);
	}
	public void updateF( double kF ){
		articules.setF(kF);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}