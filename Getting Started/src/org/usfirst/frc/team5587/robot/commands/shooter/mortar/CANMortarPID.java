package org.usfirst.frc.team5587.robot.commands.shooter.mortar;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team5587.classes.NetworkTable;
import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.CANMortar;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CANMortarPID extends Command {

	private CANMortar mortar;
	private double targetRate;
	private List<Double> data = new ArrayList<Double>();
	private List<Double> times = new ArrayList<Double>();
	private NetworkTable table;
	private int counter;
	private double startTime;
	private double prevRate;
	private boolean isRebooting;
	
    public CANMortarPID() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.mortarCAN );
    	mortar = Robot.mortarCAN;
    	
    	table = NetworkTable.getTable( "Output data" );
    	SmartDashboard.putNumber( "CAN Target Rate: ", -2500.0 );
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	mortar.speedMode();
    	//mortar.updatePID();
    	counter = 0;
    	prevRate = mortar.rps();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	targetRate = SmartDashboard.getNumber( "CAN Target Rate: ", -0.0 );
    	
    	mortar.spin( targetRate );	
    	SmartDashboard.putNumber( "Encoder RPS: ", mortar.rps() );
    	SmartDashboard.putNumber( "Speed: ", mortar.rpm() );
    	System.out.println(mortar.rpm());
    	SmartDashboard.putNumber( "Motor Output", mortar.output() );
    	
    	times.add( this.timeSinceInitialized() );
    	data.add( mortar.rps() );
    
    	if(isRebooting == true && prevRate < mortar.rps()) {
    		double newTime = this.timeSinceInitialized() - startTime;
    		table.putNumber("Final Time", newTime);
    		isRebooting = false;
    	}
    	
    	if(isRebooting == false && prevRate - mortar.rps() >= 1500) {
    		isRebooting = true;
    		startTime = this.timeSinceInitialized();
    	}
    	
    	prevRate = mortar.rps();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() { 	
    	data.add( mortar.rps() );
    	times.add( this.timeSinceInitialized() );
    	
    	System.out.println( "Time: " + times.get( counter ) + ", Rate: " + data.get( counter ) );
    	counter++;
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	double [] sendData = new double [ data.size() ];
    	double [] sendTime = new double [ times.size() ];
    	
    	for( int i = 0; i < data.size(); i++ )
    	{
    		sendData[ i ] = data.get( i );
    		sendTime[ i ] = times.get( i );
    	}
    	
    	table.putNumberArray( "Rate", sendData );
    	table.putNumberArray( "Times", sendTime );
    	
    	mortar.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
