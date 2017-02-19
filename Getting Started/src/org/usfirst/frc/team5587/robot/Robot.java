package org.usfirst.frc.team5587.robot;

import org.usfirst.frc.team5587.classes.IterativeRobot;
import org.usfirst.frc.team5587.classes.NetworkTable;
import org.usfirst.frc.team5587.robot.commandgroups.DriveRightDrive;
import org.usfirst.frc.team5587.robot.commandgroups.ReturnTrip;
import org.usfirst.frc.team5587.robot.commandgroups.TeleOp;
import org.usfirst.frc.team5587.robot.commands.ClearEncoder;
import org.usfirst.frc.team5587.robot.commands.locomotive.auto.DriveMarch;
import org.usfirst.frc.team5587.robot.commands.locomotive.auto.DutifulProgression;
import org.usfirst.frc.team5587.robot.commands.locomotive.auto.Gyrate;
import org.usfirst.frc.team5587.robot.commands.shooter.mortar.CANMortarPID;
import org.usfirst.frc.team5587.robot.subsystems.CANMortar;
import org.usfirst.frc.team5587.robot.subsystems.CANSuzy;
import org.usfirst.frc.team5587.robot.subsystems.GasGuzzler;
import org.usfirst.frc.team5587.robot.subsystems.LittleStar;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;
import org.usfirst.frc.team5587.robot.subsystems.Mortar;
import org.usfirst.frc.team5587.robot.subsystems.Suzy;
import org.usfirst.frc.team5587.robot.subsystems.Winchester;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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
	public static final CANSuzy suzyCAN = new CANSuzy();
	public static final Suzy suzyQ = new Suzy();
	public static final Mortar mortar = new Mortar();
	public static final CANMortar mortarCAN = new CANMortar();
	public static final Winchester winch = new Winchester();
	public static final LittleStar orion = new LittleStar();

	NetworkTable table;
	int counter;
	private OI oi;
	private Command auto;
	private Command teleOp;
	private CameraServer visualSensor;
	SendableChooser<Command> autoChooser = new SendableChooser<>();
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
    	oi = new OI();
    	table = NetworkTable.getTable( "Is This Thing On?" );
		//teleOp = new CANMortarPID();
    	visualSensor = CameraServer.getInstance();
    	visualSensor.startAutomaticCapture( "Visual Sensor", "cam0");
    	teleOp = new TeleOp( oi.driver, oi.codriver );
    	auto = new DriveRightDrive();
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
		counter = 0;
		if( teleOp != null )
			teleOp.start();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		table.putNumber( "Yes", counter );
		counter++;
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
