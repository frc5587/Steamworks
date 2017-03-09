package org.usfirst.frc.team5587.robot;

import org.usfirst.frc.team5587.classes.IterativeRobot;
import org.usfirst.frc.team5587.classes.NetworkTable;
import org.usfirst.frc.team5587.robot.commandgroups.LeftGearDelivery;
import org.usfirst.frc.team5587.robot.commandgroups.RightGearDelivery;
import org.usfirst.frc.team5587.robot.commandgroups.TeleOp;
import org.usfirst.frc.team5587.robot.commands.locomotive.auto.DutifulProgression;
import org.usfirst.frc.team5587.robot.subsystems.CANMortar;
import org.usfirst.frc.team5587.robot.subsystems.CANSuzy;
import org.usfirst.frc.team5587.robot.subsystems.GroundBox;
import org.usfirst.frc.team5587.robot.subsystems.LittleStar;
import org.usfirst.frc.team5587.robot.subsystems.Locomotive;
import org.usfirst.frc.team5587.robot.subsystems.Winchester;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
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

	public static final GroundBox groundbox = new GroundBox();
	public static final CANMortar mortarCAN = new CANMortar();
	public static final LittleStar orion = new LittleStar();
	public static final Locomotive loco = new Locomotive();
	public static final PowerDistributionPanel pdp = new PowerDistributionPanel();
	public static final CANSuzy suzyCAN = new CANSuzy();
	public static final Winchester winch = new Winchester();

	NetworkTable table;
	int counter;
	public static OI oi;
	private Command auto;
	private Command teleOp;
	private UsbCamera cam0, cam1;
	private CameraServer cam;
	private MjpegServer cam1Server, cam2Server;
	SendableChooser<Command> autoChooser = new SendableChooser<>();
	private boolean resBool;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
    	oi = new OI();
		//teleOp = new CANMortarPID();
    	
    	cam0 = new UsbCamera("cam0",0);
    	resBool = cam0.setResolution(32, 28);
    	cam1 = new UsbCamera( "cam1", 1 );
    	resBool = cam1.setResolution(32,28);
    	
    	cam1Server = new MjpegServer("cam0 feed",1181);
    	cam1Server.setSource(cam0);
    	
    	cam2Server = new MjpegServer("cam1 feed",1182);
    	cam2Server.setSource(cam1);
    	
    	autoChooser.addDefault("Do Nothing", null);
    	autoChooser.addObject("Left Gear Place", new LeftGearDelivery());
    	autoChooser.addObject("Right Gear Place", new RightGearDelivery());
    	autoChooser.addObject( "Front Gear Place", new DutifulProgression( -1000.0 ));//-1500.0 ) );
    	SmartDashboard.putData( "Auto Chooser", autoChooser );
    	
    	SmartDashboard.putBoolean( "time_running" , false);
    	
    	teleOp = new TeleOp( oi.driver, oi.codriver );
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		loco.zeroYaw();
		auto = autoChooser.getSelected();
		if( auto != null )
			auto.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    	SmartDashboard.putNumber( "Distance", loco.getDistance() );
    	SmartDashboard.putNumber("Gyro", loco.getYaw());
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
    	SmartDashboard.putBoolean( "time_running" , true);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		SmartDashboard.putNumber("Gyro", loco.getYaw());
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
    	SmartDashboard.putNumber( "winch_state", 1 );
	}
	
	@Override
	public void disabledPeriodic()
	{
		SmartDashboard.putNumber("Gyro", loco.getYaw());
    	SmartDashboard.putBoolean( "time_running" , false);
	}
}
