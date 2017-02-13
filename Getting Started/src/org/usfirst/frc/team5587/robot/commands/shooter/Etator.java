package org.usfirst.frc.team5587.robot.commands.shooter;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.subsystems.Suzy;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Etator extends Command {

//	private static final double ERROR_MARGIN = 10.0;
//	private static final double GAIN = .0003;
//	private static final double D_GAIN = 0.0;
//	private static final double OUTPUT_CAP = 0.4;
//	
	private static final String NETWORKTABLES_TABLE_NAME = "/GRIP/postprocessed";
	private static final String NETWORKTABLES_ANGLE_NAME = "x angles";
//
	private NetworkTable table2;
	private double counter;
//	
	private double [] angles;
//	private double deltaAngle;
//	private double rotateAngle; //The target angle for the robot to rotate.
	private double angle; //The current encoder angle reading
//	
//	private double output; //The PWM output
//	private double h0; //The average between the last value of h0 and the last output value that caused a change in the sign of the error
//	
//	private double error, //The current error
//					error1, //The error from the last instance
//					error2, //The error from 2 instances ago
//					error3, //The error from 3 instances ago
//					error4; //The error from 4 instances ago
//	
//	private double sign; //The sign of the current error
//	private double sign0; //The sign of the last error.
//	
//	private boolean wait = false;
//	
	private Suzy suzyQ;
//	private AHRS loco;
	
    public Etator() {
    	// Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.suzyQ );
    	suzyQ = Robot.suzyQ;
    	counter = 0;
//    	h0 = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//    	output = 0;
//
//    	table = NetworkTable.getTable( NETWORKTABLES_TABLE_NAME );
    	System.out.println( "Etate." );
    	table2 = NetworkTable.getTable( "angle thingy" );
    	table2.putNumber("PID Angle", 0.0 );
//    	setNewAngle();
//    	
//    	angle = suzyQ.getEncAngle();
//    	
//    	error = deltaAngle;
//    	sign0 = Math.signum( error );
//    	SmartDashboard.putNumber("GAIN", GAIN);
//    	SmartDashboard.putNumber("D_GAIN", D_GAIN);
    	suzyQ.zeroEnc();
    	suzyQ.setUsingPID(true);
    	//suzyQ.getController().setSetpoint(suzyQ.getEncAngle());
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	angle = suzyQ.getEncAngle();
//    	error = rotateAngle - angle;
//    	
//    	sign = Math.signum( error );
//    	
//    	if( withinMargin() )
//    	{
//    		output = 0.0;
//    		
//    	}
//    	else
//    	{
//	    	if( sign == sign0 * -1.0 )
//	    	{
//	    		h0 = ( output + h0 ) / 2.0;
//	    		output = -h0;
//	    	}
//	    	else if( error > 10 && error < 10)
//	    	{
//	    		output = 0;
//	    	}
//	    	else
//	    	{
//	    		output += SmartDashboard.getNumber("GAIN", GAIN) * error + SmartDashboard.getNumber("D_GAIN", D_GAIN) * ( error - error1 );
//	    	}
//	    	
//	    	if( output > OUTPUT_CAP )
//	    		output = OUTPUT_CAP;
//	    	else if( output < -OUTPUT_CAP )
//	    		output = -OUTPUT_CAP;
//	    	
//	    	SmartDashboard.putNumber( "Turntable Error: ", error );
//	    	SmartDashboard.putNumber( "Turntable Output: ", output );
//	    	SmartDashboard.putNumber( "Turntable H0: ", h0 );
//	    	SmartDashboard.putNumber( "Encoder: ", suzyQ.getEncAngle() );
//	    	
//	    	sign0 = sign;
//    	}
//    	
//    	error4 = error3;
//    	error3 = error2;
//    	error2 = error1;
//    	error1 = error;
//    	suzyQ.set( -output );
    	
    	//614 PID
//    	angles = table.getNumberArray( NETWORKTABLES_ANGLE_NAME, new double [] { 0.0 } );
//    	if( angles.length > 0 )
//    		angle = angles[ 0 ];
//    	else
//    		angle = 0.0;
//    	
//    	if( angle == Double.NaN )
//    		angle = 0.0;
//    	
//    	suzyQ.getController().setSetpoint(angle);

    	suzyQ.getController().setSetpoint(table2.getNumber("PID Angle", 3000));
    	suzyQ.set(suzyQ.getRotateRate());
    	SmartDashboard.putNumber("encoder val", suzyQ.getEncAngle() );//suzyQ.getEncAngle());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished(){
//    	if( withinMargin() )//&& !loco.isMoving() )
//    	{
//    		System.out.println( "I'm here." );
//    		setNewAngle();
//    		//if( Math.abs( error - deltaAngle ) <= ERROR_MARGIN )
//    			//suzyQ.setOnTarget( true );
//    		//else
//    			//rotateAngle =  deltaAngle;
//    	}
//    	
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	suzyQ.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	suzyQ.setUsingPID(false);
    	suzyQ.stop();
    }
    
//    private void setNewAngle()
//    {
//    	angles = table.getNumberArray( NETWORKTABLES_ANGLE_NAME, new double [] { 0.0 } );
//    	if( angles.length > 0 )
//    		deltaAngle = angles[ 0 ];
//    	else
//    		deltaAngle = 0.0;
//    	
//    	if( deltaAngle == Double.NaN )
//    		deltaAngle = 0.0;
//    	
//    	rotateAngle = -deltaAngle + angle;
//    }
    
//    private boolean withinMargin()
//    {
//    	return //Math.abs( error4 ) < ERROR_MARGIN 
//            	//&& Math.abs( error3 ) < ERROR_MARGIN
//            	Math.abs( error2 ) < ERROR_MARGIN 
//            	&& Math.abs( error1 ) < ERROR_MARGIN
//            	&& Math.abs( error ) < ERROR_MARGIN;
//    }
}
