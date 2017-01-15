package org.usfirst.frc.team5587.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team5587.robot.RobotMap;

/**
 * Drivetrain subsystem.
 *
 */
public class Locomotive extends Subsystem {
	
	//The distance covered by the wheels per one pulse registered on the encoder. ( Pi * diameter * pulses per revolution )
    private static final double DISTANCE_PER_PULSE = Math.PI * 6 / 1440;
    
    //The maximum speed we want our robot to move forward
    private static final double MAX_SPEED = 25;
    
    //The maximum speed we want our robot to turn
    private static final double MAX_TURN_SPEED = 12;
    
    //The Drive Train motors
    private VictorSP leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;
    
    //The Drive Train encoders
    private Encoder leftEncoder, rightEncoder;
    
    //PID constants
    private static final double speedP = 0.1, speedI = 0.0, speedD = 0.0;
    
    //The PID controllers for speed.
    private PIDController leftSpeedPID, rightSpeedPID;
   
    /**
     * Drivetrain constructor.
     */
    public Locomotive()
    {
    	//Instantiate motors
        leftFrontMotor = new VictorSP( RobotMap.LEFT_FRONT_MOTOR );
        leftRearMotor = new VictorSP( RobotMap.LEFT_REAR_MOTOR );
        rightFrontMotor = new VictorSP( RobotMap.RIGHT_FRONT_MOTOR );
        rightRearMotor = new VictorSP( RobotMap.RIGHT_REAR_MOTOR );
        
        //Instantiate encoders
        leftEncoder = new Encoder(RobotMap.LEFT_DRIVETRAIN_ENCODER_A,
                RobotMap.LEFT_DRIVETRAIN_ENCODER_B );
        rightEncoder = new Encoder(RobotMap.RIGHT_DRIVETRAIN_ENCODER_A,
                RobotMap.RIGHT_DRIVETRAIN_ENCODER_B );
        
        //Setup encoders
        leftEncoder.setDistancePerPulse( DISTANCE_PER_PULSE );
        rightEncoder.setDistancePerPulse( DISTANCE_PER_PULSE );
        leftEncoder.setReverseDirection( true );
        
        //Instantiate speed PID controllers
        leftSpeedPID = new PIDController( speedP, speedI, speedD, leftSpeedSource, leftSpeedOutput );
        rightSpeedPID = new PIDController( speedP, speedI, speedD, rightSpeedSource, rightSpeedOutput );
        
        //Setup speed PID controllers
        leftSpeedPID.setOutputRange( -1, 1 );
        rightSpeedPID.setOutputRange( -1, 1 );
    }

    /**
     * There is <b>no</b> default command.
     */
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    

    /**
     * Gets the robot's distance traveled since last reset. Only really works
     * when not turning.
     *
     * @return Distance in meters.
     */
    public double getDistance()
    {
        double leftDistance = leftEncoder.getDistance();
        double rightDistance = rightEncoder.getDistance();
        double averageDistance = leftDistance / 2 + rightDistance / 2;
        return averageDistance;
    }
    
    /**
     * Advance Dutifully implements single stick driving. This function lets you directly provide
     * joystick values from any source.
     * Modified from the method in RobotDrive, this uses speed values in place of power values.
     *
     * @param moveValue     The value to use for forwards/backwards (y-axis)
     * @param rotateValue   The value to use for the rotate right/left (x-axis)
     * @param squaredInputs if set, decreases the sensitivity at low speeds
     */
    public void advanceDutifully(double moveValue, double rotateValue, boolean squaredInputs)
    {
    	double leftMotorSpeed, rightMotorSpeed;
    	
    	if(squaredInputs)
    	{
    		// square the inputs (while preserving the sign) to increase fine control
    		// 	while permitting full power
    		if( moveValue >= 0.0 )
    		{	
    			moveValue = moveValue * moveValue;
    		}
    		else
    		{
    			moveValue = -( moveValue * moveValue );
    		}
    		if( rotateValue >= 0.0 )
    		{
    			rotateValue = rotateValue * rotateValue;
    		}
    		else
    		{
    			rotateValue = -( rotateValue * rotateValue );
    		}
    	}
    	
    	moveValue = moveValue * MAX_SPEED;
    	moveValue = limit( moveValue );
    	
        rotateValue = rotateValue * MAX_TURN_SPEED;

        if( moveValue > 0.0 )
        {
      	  if( rotateValue > 0.0 )
      	  {
      		  leftMotorSpeed = moveValue - rotateValue;
      		  rightMotorSpeed = Math.max( moveValue, rotateValue );
      	  }
      	  else
      	  {
      		  leftMotorSpeed = Math.max( moveValue, -rotateValue );
      		  rightMotorSpeed = moveValue + rotateValue;
      	  }
        }
        else
        {
      	  if( rotateValue > 0.0 )
      	  {
      		  leftMotorSpeed = -Math.max( -moveValue, rotateValue );
      		  rightMotorSpeed = moveValue + rotateValue;
      	  }
      	  else
      	  {
      		  leftMotorSpeed = moveValue - rotateValue;
      		  rightMotorSpeed = -Math.max( -moveValue, -rotateValue );
      	  }
        }
        
        leftSpeedPID.setSetpoint( leftMotorSpeed );
        rightSpeedPID.setSetpoint( rightMotorSpeed );
    }

    /**
     * Resets the robot's distance travelled.
     */
    public void resetDistance()
    {
        leftEncoder.reset();
        rightEncoder.reset();
    }
    
    /**
     * Limit speed values to +/- our defined maximum speed
     */
    protected static double limit( double speed)
    {
      if ( speed > MAX_SPEED ) {
        return MAX_SPEED;
      }
      if ( speed < -MAX_SPEED ) {
        return -MAX_SPEED;
      }
      return speed;
    }

    private final PIDSource leftSpeedSource = new PIDSource()
    {
    	public double pidGet()
    	{
            return leftEncoder.getRate();
        }

		@Override
		public void setPIDSourceType(PIDSourceType pidSource)
		{
			// TODO Auto-generated method stub
			leftEncoder.setPIDSourceType( PIDSourceType.kRate );
		}

		@Override
		public PIDSourceType getPIDSourceType()
		{
			// TODO Auto-generated method stub
			return leftEncoder.getPIDSourceType();
		}
    };
    
    private final PIDOutput leftSpeedOutput= new PIDOutput() 
    {
    	public void pidWrite( double output )
    	{
    		leftFrontMotor.set( output );
    		leftRearMotor.set( output );
    	}
    };
    
    private final PIDSource rightSpeedSource = new PIDSource()
    {
    	public double pidGet()
    	{
            return rightEncoder.getRate();
        }

		@Override
		public void setPIDSourceType(PIDSourceType pidSource)
		{
			// TODO Auto-generated method stub
			rightEncoder.setPIDSourceType( PIDSourceType.kRate );
		}

		@Override
		public PIDSourceType getPIDSourceType()
		{
			// TODO Auto-generated method stub
			return rightEncoder.getPIDSourceType();
		}
    };
    
    private final PIDOutput rightSpeedOutput = new PIDOutput()
    {
    	public void pidWrite( double output )
    	{
    		rightFrontMotor.set( output );
    		rightRearMotor.set( output );
    	}
    };
}