package org.usfirst.frc.team5587.classes;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team5587.classes.DualPIDController;
import org.usfirst.frc.team5587.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

/**
 * Drivetrain subsystem.
 *
 */
public class Locomotive extends Subsystem {
	
	//The distance covered by the wheels per one pulse registered on the encoder. ( Pi * diameter * pulses per revolution )
    private static final double DISTANCE_PER_PULSE = Math.PI * 6 / 1440;
    private static final double AUTO_OUTPUT_LIMIT = 0.5;
    
    
    //The Drive Train motors
    private VictorSP leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;
    
    //The Drive Train encoders
    private Encoder leftEncoder, rightEncoder;
    
    //PID constants
    private static final double [] leftDistConstants = { 0.1, 0.0, 0.0 }, //TODO: Tune PID Constants
    							   rightDistConstants = { 0.1, 0.0, 0.0 }; //TODO: Tune PID Constants
    
    public double leftRate, rightRate;
    
    //The PID controller for distance
    private DualPIDController distPID;
   
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
        
        distPID = new DualPIDController( leftDistConstants, rightDistConstants,
        								 leftDistSource, rightDistSource,
        								 leftOutput, rightOutput );
        
        distPID.setOutputRange( -AUTO_OUTPUT_LIMIT, AUTO_OUTPUT_LIMIT );
        distPID.setContinuous( false );
        
        leftRate = 0.0;
        rightRate = 0.0;
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
     * Span Distance tells the robot to go a certain distance in a straight line.
     * @param arcLength The distance traveled by the center of the robot across the curve.
     * @param angle The angle, in radians, that the robot turns from the beginning to the end of the curve.
     */
    public void tankDrive( double left, double right )
    {
    	
    }
    
    /**
     * Resets the robot's distance travelled.
     */
    public void resetDistance()
    {
        leftEncoder.reset();
        rightEncoder.reset();
    }
    
    public void enableDistance()
    {
    	distPID.setPIDSourceType( PIDSourceType.kDisplacement );
    	distPID.enable();
    }
    
    public void disableDistance()
    {
    	distPID.disable();
    }
    
    public boolean distanceOnTarget()
    {
    	return distPID.onTarget();
    }
    
    //Creating all of the PIDSource objects necessary for operation.
    
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