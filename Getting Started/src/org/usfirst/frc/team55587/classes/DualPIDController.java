package org.usfirst.frc.team55587.classes;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class DualPIDController {

	private PIDController leftController, rightController;
	private PIDSource leftSource, rightSource;
	
	public DualPIDController( double [] kLefts,
							  double [] kRights,
							  PIDSource lSrc, PIDSource rSrc,
							  PIDOutput lOut, PIDOutput rOut )
	{
		leftSource = lSrc;
		rightSource = rSrc;
		
		leftController = new PIDController( kLefts[0], kLefts[1], kLefts[2], leftSource, lOut );
		rightController = new PIDController( kRights[0], kRights[1], kRights[2], rightSource, rOut );
	}
	
	public void setLeft( double setpoint )
	{
		leftController.setSetpoint( setpoint );
	}
	
	public void setRight( double setpoint )
	{
		rightController.setSetpoint( setpoint );
	}
	
	public void setSetpoint( double setpoint )
	{
		setLeft( setpoint );
		setRight( setpoint );
	}
	
	public void disable()
	{
		leftController.disable();
		rightController.disable();
	}
	
	public void enable()
	{
		leftController.enable();
		rightController.enable();
	}
	
	public void setOutputRange( double minimum, double maximum )
	{
		leftController.setOutputRange( minimum, maximum );
		rightController.setOutputRange( minimum, maximum );
	}
	
	public void setContinuous( boolean isContinuous )
	{
		leftController.setContinuous( isContinuous );
		rightController.setContinuous( isContinuous );
	}
	
	public void setPIDSourceType( PIDSourceType pidSource )
	{
		leftSource.setPIDSourceType( pidSource );
		rightSource.setPIDSourceType( pidSource );
	}
}
