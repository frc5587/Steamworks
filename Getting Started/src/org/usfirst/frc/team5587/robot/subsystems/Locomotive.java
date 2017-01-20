package org.usfirst.frc.team5587.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team55587.classes.DualPIDController;
import org.usfirst.frc.team55587.classes.GyroPIDOutput;
import org.usfirst.frc.team5587.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

/**
 * Drivetrain subsystem.
 *
 */
public class Locomotive extends Subsystem {
	
	//The distance covered by the wheels per one pulse registered on the encoder. ( Pi * diameter * pulses per revolution )
    private static final double DISTANCE_PER_PULSE = Math.PI * 6 / 1440;
    
    //The maximum speed we want our robot to move forward
    private static final double MAX_SPEED = 25; //TODO: Determine appropriate maximum drive speed.
    
    //The maximum speed we want our robot to turn
    private static final double MAX_TURN_SPEED = 12; //TODO: Determine appropriate maximum turn speed.
    
    private static final double TURN_RADIUS = 145.64; //TODO: Check this value.
    private static final double WHEEL_BASE = 14; //TODO: Double check with Build Team on this value.
    public static final double AUTO_CURVE = Math.pow( Math.E, ( - TURN_RADIUS / WHEEL_BASE ) );
    public static final double AUTO_SPEED_LIMIT = .5; //TODO: Determine maximum autonomous power.
    
    //The Drive Train motors
    private VictorSP leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;
    
    //The Drive Train encoders
    private Encoder leftEncoder, rightEncoder;
    
    private AHRS gyro;
    private GyroPIDOutput gyroOutput;
    
    //PID constants
    private static final double [] leftSpeedConstants = { 0.1, 0.0, 0.0 }, //TODO: Tune PID Constants
    							   rightSpeedConstants = { 0.1, 0.0, 0.0 }, //TODO: Tune PID Constants
    							   leftDistConstants = { 0.1, 0.0, 0.0 }, //TODO: Tune PID Constants
    							   rightDistConstants = { 0.1, 0.0, 0.0 }; //TODO: Tune PID Constants
    private static final double gyroP = 0.1, gyroI = 0.0, gyroD = 0.0; //TODO: Tune PID Constants
    
    //The PID controllers for speed.
    private DualPIDController speedPID;
    private DualPIDController distPID;
    private PIDController gyroPID;
   
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
        
        //Setup Gyroscope
        try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            gyro = new AHRS( RobotMap.NAVX_MXP ); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        gyro.setPIDSourceType( PIDSourceType.kDisplacement );
        
        gyroOutput = new GyroPIDOutput( leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor );
        
        //Instantiate PID controllers
        speedPID = new DualPIDController( leftSpeedConstants, rightSpeedConstants,
        								  leftSpeedSource, rightSpeedSource,
        								  leftOutput, rightOutput );
        distPID = new DualPIDController( leftDistConstants, rightDistConstants,
        								 leftDistSource, rightDistSource,
        								 leftOutput, rightOutput );
        gyroPID = new PIDController( gyroP, gyroI, gyroD, gyroSource, gyroOutput );
        
        //Setup speed PID controllers
        speedPID.setOutputRange( -1.0, 1.0 );
        speedPID.setContinuous( true );
        
        distPID.setOutputRange( -AUTO_SPEED_LIMIT, AUTO_SPEED_LIMIT );
        distPID.setContinuous( true );
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
    public void keepPace(double moveValue, double rotateValue, boolean squaredInputs)
    {
    	double leftMotorSpeed, rightMotorSpeed;
    	disablePID();
    	
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
        
        speedPID.setLeft( leftMotorSpeed );
        speedPID.setRight( rightMotorSpeed );
        enableSpeed();
    }

    /**
     * Span Distance tells the robot to go a certain distance in a straight line.
     * @param arcLength The distance traveled by the center of the robot across the curve.
     * @param angle The angle, in radians, that the robot turns from the beginning to the end of the curve.
     */
    public void spanDistance( double arcLength )
    {
    	disablePID();
    	
    	distPID.setSetpoint( arcLength );
    	enableDistance();
    }
    
    /**
     * 
     */
    public void traverseCurve( double angle )
    {
    	disablePID();
    	gyroOutput.setCurve( angle );
    	
    	gyroPID.setSetpoint( angle );
    	gyroPID.enable();
    }
    
    /**
     * Resets the robot's distance travelled.
     */
    public void resetDistance()
    {
        leftEncoder.reset();
        rightEncoder.reset();
    }
    
    public void disablePID()
    {
    	speedPID.disable();
    	distPID.disable();
    	gyroPID.disable();
    }
    
    public void enableSpeed()
    {
    	speedPID.setPIDSourceType( PIDSourceType.kRate );
    	speedPID.enable();
    }
    
    public void enableDistance()
    {
    	distPID.setPIDSourceType( PIDSourceType.kDisplacement );
    	distPID.enable();
    }
    
    /**
     * Limit speed values to +/- our defined maximum speed
     */
    protected static double limit( double speed )
    {
      if ( speed > MAX_SPEED ) {
        return MAX_SPEED;
      }
      if ( speed < -MAX_SPEED ) {
        return -MAX_SPEED;
      }
      return speed;
    }
    
    //Creating all of the PIDSource objects necessary for operation.
    
    /**
     * PIDSource object for the left speed PID controller
     */
    private final PIDSource leftSpeedSource = new PIDSource()
    {
    	public double pidGet()
    	{
            return leftEncoder.getRate();
        }

		@Override
		public void setPIDSourceType(PIDSourceType pidSource)
		{
			leftEncoder.setPIDSourceType( PIDSourceType.kRate );
		}

		@Override
		public PIDSourceType getPIDSourceType()
		{
			return leftEncoder.getPIDSourceType();
		}
    };
    
    /**
     * PIDSource object for the right speed PID controller
     */
    private final PIDSource rightSpeedSource = new PIDSource()
    {
    	public double pidGet()
    	{
            return rightEncoder.getRate();
        }

		@Override
		public void setPIDSourceType(PIDSourceType pidSource)
		{
			rightEncoder.setPIDSourceType( PIDSourceType.kRate );
		}

		@Override
		public PIDSourceType getPIDSourceType()
		{
			return rightEncoder.getPIDSourceType();
		}
    };
    
    /**
     * PIDSource object for the left distance PID controller
     */
    private final PIDSource leftDistSource = new PIDSource()
    {

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			leftEncoder.setPIDSourceType( PIDSourceType.kDisplacement );
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			return leftEncoder.getDistance();
		}
    	
    };
    
    /**
     * PIDSource object for the right distance PID controller
     */
    private final PIDSource rightDistSource = new PIDSource()
    {

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			rightEncoder.setPIDSourceType( PIDSourceType.kDisplacement );
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			return rightEncoder.getDistance();
		}
    	
    };
    
    /**
     * PIDSource object for the gyroscope PID controller
     */
    private final PIDSource gyroSource = new PIDSource()
    {
    	
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			gyro.setPIDSourceType( PIDSourceType.kDisplacement );
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return gyro.getPIDSourceType();
		}
		
		@Override
		public double pidGet() {
			return gyro.getAngle();
		}
    	
    };
    
    /**
     * PIDOutput object for the left PID controllers.
     */
    private final PIDOutput leftOutput= new PIDOutput() 
    {
    	public void pidWrite( double output )
    	{
    		leftFrontMotor.set( output );
    		leftRearMotor.set( output );
    	}
    };
    
    /**
     * PIDOutput object for the rightPID controllers.
     */
    private final PIDOutput rightOutput = new PIDOutput()
    {
    	public void pidWrite( double output )
    	{
    		rightFrontMotor.set( output );
    		rightRearMotor.set( output );
    	}
    };
}