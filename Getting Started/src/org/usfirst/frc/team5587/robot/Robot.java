package org.usfirst.frc.team5587.robot;

import org.usfirst.frc.team5587.robot.commands.BasicDrive;
import org.usfirst.frc.team5587.robot.commands.TeleOp;
import org.usfirst.frc.team5587.robot.subsystems.Archie;
import org.usfirst.frc.team5587.robot.subsystems.GasGuzzler;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;
import org.usfirst.frc.team5587.robot.subsystems.Mortar;
import org.usfirst.frc.team5587.robot.subsystems.Winchester;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	public static OI oi;
	public static final GasGuzzler guzzler = new GasGuzzler();
	public static final Locomotive locomotive = new Locomotive();
	public static final Archie screw = new Archie();
	public static final Mortar mortar = new Mortar();
	public static final Winchester winch = new Winchester();
	
	Command auto;
	Command teleop;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {

		oi = new OI();
		
		teleop = new TeleOp();
		auto = null;
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
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
		if( auto != null)
			auto.cancel();
		
		if( teleop != null )
			teleop.start();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
        Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
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
