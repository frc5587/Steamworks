package org.usfirst.frc.team5587.robot;

import org.usfirst.frc.team5587.robot.commandgroups.ReturnTrip;
import org.usfirst.frc.team5587.robot.commandgroups.TeleOp;
import org.usfirst.frc.team5587.robot.commands.locomotive.Gyrate;
import org.usfirst.frc.team5587.robot.subsystems.GasGuzzler;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;
import org.usfirst.frc.team5587.robot.subsystems.Mortar;
import org.usfirst.frc.team5587.robot.subsystems.Retator;
import org.usfirst.frc.team5587.robot.subsystems.Winchester;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	public static final GasGuzzler guzzler = new GasGuzzler();
	public static final Locomotive loco = new Locomotive();
	public static final Retator retator = new Retator();
	public static final Mortar mortar = new Mortar();
	public static final Winchester winch = new Winchester();

	private OI oi;
	private Command auto;
	private Command teleOp;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		SmartDashboard.putNumber( "Target: ", 0.0 );
		teleOp = new TeleOp( oi.driver, oi.codriver );
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {

		auto = new Gyrate( SmartDashboard.getNumber( "Target: ", 0.0 ) ); 
		if( auto != null )
			auto.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
        Scheduler.getInstance().run();
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		SmartDashboard.putNumber( "Manual Control", 0.0 );
		
		if( auto != null)
			auto.cancel();
		
		if( teleOp != null )
			teleOp.start();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		SmartDashboard.putNumber( "Gyro", loco.getYaw() );
		SmartDashboard.putNumber( "Throttle", oi.driver.getThrottle() * -180.0 );
		//SmartDashboard.putNumber( "Error", loco.gyroPID.getAvgError() );
		SmartDashboard.putNumber( "Motor Output", loco.leftFrontMotor.get() );
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
		SmartDashboard.putNumber( "Gyro", loco.getYaw() );
		//SmartDashboard.putNumber( "Error", loco.gyroPID.getAvgError() );
	}
	
	@Override
	public void disabledInit()
	{
		
	}
	
	@Override
	public void disabledPeriodic()
	{
		
	}
}
