package org.usfirst.frc.team5587.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;
import org.usfirst.frc.team5587.robot.RobotMap;

/**
 * Drivetrain subsystem.
 *
 * @author Edmund
 */
public class Locomotive extends Subsystem {

    private static final double DISTANCE_PER_PULSE = (24.0 / 42.0) * 3.0 * 4.0 * Math.PI / 128;
    //0.00467498906784195422390274312988;//0.05609986881410345068683291755856;//0.0146869113074526041414067839586;//0.17624293568943124969688;//0.0140249672;
    private static final double MAX_INCH_PER_SECOND = 13 * 12;
    public static final double SPEED_LIMIT = 0.9;
    private VictorSP leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;
    private Encoder leftEncoder, rightEncoder;
    private static final double speedP = 1.2, speedI = 0.07, speedD = 0.02;
    private static final double distP = 0.1, distI = 0, distD = 0;
    private PIDController arcadeController;

    /**
     * Drivetrain constructor.
     */
    public Locomotive() {
        leftFrontMotor = new VictorSP(RobotMap.LEFT_FRONT_MOTOR);
        leftRearMotor = new VictorSP( RobotMap.LEFT_REAR_MOTOR );
        rightFrontMotor = new VictorSP(RobotMap.RIGHT_FRONT_MOTOR);
        rightRearMotor = new VictorSP( RobotMap.RIGHT_REAR_MOTOR );
        leftEncoder = new Encoder(RobotMap.LEFT_DRIVETRAIN_ENCODER_A,
                RobotMap.LEFT_DRIVETRAIN_ENCODER_B );
        rightEncoder = new Encoder(RobotMap.RIGHT_DRIVETRAIN_ENCODER_A,
                RobotMap.RIGHT_DRIVETRAIN_ENCODER_B );
        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        leftEncoder.setSamplesToAverage(6);
        rightEncoder.setSamplesToAverage(6);
        leftEncoder.setReverseDirection(true);
        arcadeController = new PIDController(speedP, speedI, speedD,
                encoderArcadeSource, encoderArcadeOutput, 10);
        arcadeController.setInputRange(-1, 1);
        arcadeController.setOutputRange(-1, 1);

//        distanceController = new PIDController(distP, distI, distD,
//                encoderDistSource, encoderDistOutput);
    }

    /**
     * There is <b>no</b> default command.
     */
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    /**
     * Tank drive: makes the robot move.
     *
     * @param left Speed for the left motors.
     * @param right Speed for the right motors.
     */
    public void tankDrive( double left, double right ) {
        leftFrontMotor.set( left * SPEED_LIMIT );
        leftRearMotor.set( left * SPEED_LIMIT );
        rightFrontMotor.set( -right * SPEED_LIMIT );
        rightRearMotor.set( -right * SPEED_LIMIT ); 
    }
    double controllerValue;

    /**
     * Drives, actively compensating for drift.
     *
     * @param moveValue Speed for forward movement.
     * @param rotateValue Speed for turning. PID controlled.
     */
    public void arcadeControlledDrive(double moveValue, double rotateValue) {
        if (Math.abs(moveValue) < 0.01) {
            arcadeDrive(0, 0);
        } else {
            arcadeDrive(moveValue, 2 * rotateValue - getTurnValue());//rotateValue - getTurnValue() + rotateValue);
        }
//        if (!arcadeController.isEnable())
//          arcadeController.enable();
//        arcadeController.setSetpoint(rotateValue);
//        controllerValue = arcadeController.get();
//        //controllerValue = arcadeController.get() * 0.3 + controllerValue * 0.7;
//        if (Math.abs(moveValue) < 0.01) {
//            arcadeDrive(0, 0);
//        } else {
//            arcadeDrive(moveValue, rotateValue + controllerValue);// + arcadeController.get());
//        }
    }

    /**
     * Arcade drive: drives using a turn value and move value.
     *
     * @param moveValue The speed at which the robot will move forward and
     * backward.
     * @param rotateValue The speed at which the robot will turn. Right values
     * are right turns.
     */
    public void arcadeDrive(double moveValue, double rotateValue) {
        double leftMotorSpeed;
        double rightMotorSpeed;
        rotateValue *= -1;
        double theta = Math.atan2(moveValue, rotateValue);
        double r = Math.sqrt(
                (moveValue * moveValue) + (rotateValue * rotateValue));
        double sinTheta = Math.sin(theta);
        double cosTheta = Math.cos(theta);
        leftMotorSpeed = (sinTheta + cosTheta) * r;
        rightMotorSpeed = (sinTheta - cosTheta) * r;
//        leftMotorSpeed = (moveValue + rotateValue) / 2;
//        rightMotorSpeed = (moveValue - rotateValue) / 2;
        //Log.log("Left: " + leftMotorSpeed + " Right: " + rightMotorSpeed);
        tankDrive(leftMotorSpeed, rightMotorSpeed);
    }

    /**
     * Stops the robot. May make use of encoder-based PID, for now it just sets
     * speed to zero.
     */
    public void brake() {
        tankDrive(0, 0);
    }

    /**
     * Gets the robot's speed. Only really works when not turning.
     *
     * @return Speed in meters / second.
     */
    public double getSpeed() {
        double leftSpeed = leftEncoder.getRate();
        double rightSpeed = rightEncoder.getRate();
        double averageSpeed = leftSpeed / 2.0 + rightSpeed / 2.0;
        return averageSpeed;
    }

    /**
     * Converts encoder speeds to an arcade drive joystick x-value.
     *
     * @return
     */
    protected double getTurnValue() {
        double leftSpeed = leftEncoder.getRate() / MAX_INCH_PER_SECOND * 3;
        double rightSpeed = rightEncoder.getRate() / MAX_INCH_PER_SECOND * 3;
        double forwardSpeed = leftSpeed / 2.0 + rightSpeed / 2.0;
        double turnSpeed = (leftSpeed / 2.0 - rightSpeed / 2.0); //* 0.1 + lastTurnSpeed * 0.9;
        return turnSpeed;
    }

    /**
     * Gets the robot's distance traveled since last reset. Only really works
     * when not turning.
     *
     * @return Distance in meters.
     */
    public double getDistance() {
        double leftDistance = leftEncoder.getDistance();
        double rightDistance = rightEncoder.getDistance();
        double averageDistance = leftDistance / 2 + rightDistance / 2;
        return averageDistance;
    }

    /**
     * Resets the robot's distance travelled.
     */
    public void resetDistance() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    private final PIDSource encoderArcadeSource = new PIDSource() {
        public double pidGet() {
            return getTurnValue();
        }

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			// TODO Auto-generated method stub
			return null;
		}
    };
    private final PIDSource encoderDistSource = new PIDSource() {
        public double pidGet() {
            return getDistance() / MAX_INCH_PER_SECOND;
        }

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			// TODO Auto-generated method stub
			return null;
		}
    };
    private final PIDOutput encoderArcadeOutput = new PIDOutput() {
        public void pidWrite(double d) {
        	
        }
    };
    private final PIDOutput encoderDistOutput = new PIDOutput() {
        public void pidWrite(double d) {
        }
    };
}