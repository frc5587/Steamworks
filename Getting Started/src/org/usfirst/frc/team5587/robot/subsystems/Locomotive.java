package org.usfirst.frc.team5587.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;

import org.usfirst.frc.team5587.classes.DualPIDController;
import org.usfirst.frc.team5587.classes.NetworkTable;
import org.usfirst.frc.team5587.robot.RobotMap;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import com.kauailabs.navx.frc.AHRS;

/**
 * Drivetrain subsystem.
 *
 */
public class Locomotive extends Subsystem {
	
	//The distance covered by the wheels per one pulse registered on the encoder. ( Pi * diameter * pulses per revolution )
    private static final double DISTANCE_PER_PULSE = 1;//0.05500836275785170042742984575358;
    private static final double AUTO_OUTPUT_LIMIT = 0.5;
    
    private static final double WHEEL_BASE = 14; //TODO: Double check with Build Team on this value.
    public static final double AUTO_SPEED_LIMIT = .5; //TODO: Determine maximum autonomous power.
    
    private static double Y_LIMIT = 1.0;
    private static final double X_LIMIT = 1.0;
    
    private static double LEFT_LIMIT = 1.0;
    private static double RIGHT_LIMIT = 1.0;
    
    //The Drive Train motors
    public VictorSP leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;
    
    //The Drive Train encoders
    private Encoder leftEncoder, rightEncoder;
    
    public AHRS gyro;
    
    private RobotDrive train;
    
    private NetworkTable table;

    //The PID controller for distance
    private DualPIDController tankPID;
    private PIDController drivePID;
   
    //PID constants
    private static final double [] leftDistConstants = { 0.1, 0.0, 0.0 }, //TODO: Tune PID Constants
    						 	   rightDistConstants = { 0.1, 0.0, 0.0 }; //TODO: Tune PID Constants
    private static final double kP = 0.1,
    							kI = 0.0,
    							kD = 0.0;
    
    public double leftRate, rightRate, driveRate;
    
    private double zeroYaw;
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
        
        train = new RobotDrive( leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor );
        
        //Instantiate encoders
        leftEncoder = new Encoder(RobotMap.LEFT_DRIVETRAIN_ENCODER_A,
                RobotMap.LEFT_DRIVETRAIN_ENCODER_B );
        rightEncoder = new Encoder(RobotMap.RIGHT_DRIVETRAIN_ENCODER_A,
                RobotMap.RIGHT_DRIVETRAIN_ENCODER_B );
        
        //Setup encoders
        leftEncoder.setDistancePerPulse( DISTANCE_PER_PULSE );
        rightEncoder.setDistancePerPulse( DISTANCE_PER_PULSE );
        leftEncoder.setReverseDirection( true );
        
        leftEncoder.setPIDSourceType( PIDSourceType.kDisplacement );
        rightEncoder.setPIDSourceType( PIDSourceType.kDisplacement );
        
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
        zeroYaw = gyro.getYaw();

        LiveWindow.addSensor( "Locomotive", "Gyroscope", gyro );
        LiveWindow.addSensor( "Locomotive", "Left Encoder", leftEncoder );
        LiveWindow.addSensor( "Locomotive", "Right Encoder", rightEncoder );
        
        drivePID = new PIDController( kP, kI, kD, driveSource, driveOutput );
        drivePID.setContinuous( false );
        drivePID.setOutputRange( -AUTO_OUTPUT_LIMIT, AUTO_OUTPUT_LIMIT );
        
        tankPID = new DualPIDController( leftDistConstants, rightDistConstants,
				 leftDistSource, rightDistSource,
				 leftOutput, rightOutput );

        tankPID.setOutputRange( -AUTO_OUTPUT_LIMIT, AUTO_OUTPUT_LIMIT );
        tankPID.setContinuous( false );

        leftRate = 0.0;
        rightRate = 0.0;
        
        table = NetworkTable.getTable( "Drivetrain" );
    }
	
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    /**
     * Advance Dutifully implements single stick driving. This function lets you directly provide
     * joystick values from any source.
     *
     * @param moveValue     The value to use for forwards/backwards (y-axis)
     * @param rotateValue   The value to use for the rotate right/left (x-axis)
     */
    public void keepPace( Joystick stick )
    {
    	keepPace( -stick.getY() * Y_LIMIT, -stick.getX() * X_LIMIT );
    }
    
    public void invert()
    {
    	Y_LIMIT *= -1.0;
    }
    
    public void keepPace( double y, double x )
    {
    	train.arcadeDrive( y, x, false );
    }

    /**
     * 
     */
    public void tankDrive( double left, double right )
    {
    	train.tankDrive( left * LEFT_LIMIT, right * RIGHT_LIMIT );
    }
    
    /**
     * 
     */
    public void proceedForwards( double speed )
    {
    	train.drive( speed, 0.0 );
    }
    
    /**
     * 
     */
    public void halt()
    {
    	train.arcadeDrive( 0.0, 0.0 );
    }
    
    /**
     * 
     */
    public void rotate( double power )
    {
    	train.arcadeDrive( 0.0, power );
    }
    
    /**
     * 
     * @param distance
     */
    public void setTankDistance( double distance )
    {
    	tankPID.setSetpoint( distance );
    }
    
    public void setDriveDistance( double distance )
    {
    	drivePID.setSetpoint( distance );
    }
    
    /**
     * Gets the robot's distance traveled since last reset. Only really works
     * when not turning.
     *
     * @return Distance in meters.
     */
    public double getDistance()
    {
        return ( rightEncoder.getDistance() + leftEncoder.getDistance() ) / 2.0;
    }
    
    /**
     * Resets the robot's distance travelled.
     */
    public void resetDistance()
    {
        leftEncoder.reset();
        rightEncoder.reset();
    }
    
    public void enableTankDistance()
    {
    	tankPID.setPIDSourceType( PIDSourceType.kDisplacement );
    	tankPID.enable();
    }
    
    public void enableDriveDistance()
    {
    	drivePID.enable();
    }
    
    public void disablePID()
    {
    	tankPID.disable();
    	drivePID.disable();
    }
    
    public boolean driveOnTarget()
    {
    	return drivePID.onTarget();
    }
    
    public boolean tankOnTarget()
    {
    	return tankPID.onTarget();
    }
    
    public void printEncoders()
    {
    	table.putNumber( "Left Distance", leftEncoder.getDistance() );
    	table.putNumber( "Right Distance", rightEncoder.getDistance() );
    	table.putNumber( "Left Speed", leftEncoder.getRate() );
    	table.putNumber( "Right Speed", rightEncoder.getRate() );
    }
    
    public double getYaw()
    {
    	return gyro.getYaw() - zeroYaw;
    }
    
    public void zeroYaw()
    {
    	zeroYaw = gyro.getYaw();
    }
    
    public boolean isCalibrating()
    {
    	return gyro.isCalibrating();
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
    		leftRate = output;
    	}
    };
    
    /**
     * PIDOutput object for the rightPID controllers.
     */
    private final PIDOutput rightOutput = new PIDOutput()
    {
    	public void pidWrite( double output )
    	{
    		rightRate = output;
    	}
    };
    
    private final PIDSource driveSource = new PIDSource()
    {
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			// TODO Auto-generated method stub
			leftEncoder.setPIDSourceType( pidSource );
			rightEncoder.setPIDSourceType( pidSource );
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			// TODO Auto-generated method stub
			return null;
		}

		@Override
		public double pidGet() {
			// TODO Auto-generated method stub
			return getDistance();
		}	 
    };
    
    private final PIDOutput driveOutput = new PIDOutput()
    {
		@Override
		public void pidWrite(double output) {
			// TODO Auto-generated method stub
			driveRate = output;
		}
    };
}