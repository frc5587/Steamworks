package org.usfirst.frc.team5587.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team55587.classes.PIDController;
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
    
    private static final double WHEEL_BASE = 14; //TODO: Double check with Build Team on this value.
    public static final double AUTO_SPEED_LIMIT = .5; //TODO: Determine maximum autonomous power.
    
    //The Drive Train motors
    public VictorSP leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;
    
    //The Drive Train encoders
    private Encoder leftEncoder, rightEncoder;
    
    private AHRS gyro;
    
    //The PID controllers for speed.
    private RobotDrive train;
   
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
        

        LiveWindow.addSensor( "GyroSensor", "Gyroscope", gyro );
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
     * Advance Dutifully implements single stick driving. This function lets you directly provide
     * joystick values from any source.
     *
     * @param moveValue     The value to use for forwards/backwards (y-axis)
     * @param rotateValue   The value to use for the rotate right/left (x-axis)
     */
    public void keepPace(double moveValue, double rotateValue )
    {
    	train.arcadeDrive( moveValue, rotateValue );
    }

    /**
     * Span Distance tells the robot to go a certain distance in a straight line.
     * @param pwr			  The power to set the motors to.
     * @param curve           The rate of turn, constant for different forward speeds. Set {@literal
     *                        curve < 0 for left turn or curve > 0 for right turn.} Set curve =
     *                        e^(-r/w) to get a turn radius r for wheelbase w of your robot.
     *                        Conversely, turn radius r = -ln(curve)*w for a given value of curve and
     *                        wheelbase w.
     */
    public void spanDistance( double pwr, double curve )
    {
    	train.drive( pwr * AUTO_SPEED_LIMIT, curve );
    }
    
    public void proceedForwards()
    {
    	train.drive( AUTO_SPEED_LIMIT, 0.0 );
    }
    
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
     * Resets the robot's distance travelled.
     */
    public void resetDistance()
    {
        leftEncoder.reset();
        rightEncoder.reset();
    }
    
    public double getYaw()
    {
    	return gyro.getYaw();
    }
    
    public void zeroYaw()
    {
    	gyro.zeroYaw();
    }
    
    public boolean isCalibrating()
    {
    	return gyro.isCalibrating();
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
    
}