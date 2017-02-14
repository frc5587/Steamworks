package org.usfirst.frc.team5587.robot;

import org.usfirst.frc.team5587.classes.NetworkTable;
import org.usfirst.frc.team5587.robot.commandgroups.TeleOp;
import org.usfirst.frc.team5587.robot.subsystems.GasGuzzler;
import org.usfirst.frc.team5587.robot.subsystems.LittleStar;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;
import org.usfirst.frc.team5587.robot.subsystems.Mortar;
import org.usfirst.frc.team5587.robot.subsystems.Suzy;
import org.usfirst.frc.team5587.robot.subsystems.Winchester;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
	public static final Suzy suzyQ = new Suzy();
	public static final Mortar mortar = new Mortar();
	public static final Winchester winch = new Winchester();
	public static final LittleStar orion = new LittleStar();

	private OI oi;
	private Command auto;
	private Command teleOp;
	SendableChooser<Command> autoChooser = new SendableChooser<>();
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		
		NetworkTable table = NetworkTable.getTable( "PID Tuning" );
    	table.putNumber("kP", 0.0 );
    	table.putNumber("kI", 0.0 );
    	table.putNumber("kD", 0.0 );
    	table.putNumber("kF", 0.0 );
    	table.putNumber("Relative PID Angle", 0.0 );
    	
		teleOp = new TeleOp( oi.driver, oi.codriver );
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
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
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
		SmartDashboard.putNumber( "Gyro", loco.getYaw() );
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
