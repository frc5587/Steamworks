package org.usfirst.frc.team55587.classes;

import org.usfirst.frc.team5587.robot.subsystems.Locomotive;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.VictorSP;

public class GyroPIDOutput implements PIDOutput {

	private RobotDrive drivetrain;
	private double driveCurve;

	public GyroPIDOutput( VictorSP lf, VictorSP lr, VictorSP rf, VictorSP rr )
	{
		drivetrain = new RobotDrive( lf, lr, rf, rr );
		driveCurve = 0.0;
	}
	
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		output = limit( (-1.0f) * truncate( output ) );
		drivetrain.arcadeDrive( 0.0, output );
	}
	
	public double limit( double input )
	{
		if( input < 0.1 && input > -0.1 )
			return 0.0;
		else if( input < -1.0 )
			return -1.0;
		else if( input > 1.0 )
			return 1.0;
		else
			return input;
	}
	
	public static double truncate( double input )
	{
		return ( (int)( input * 10.0 ))/ 10.0;
	}
}
