package org.usfirst.team55587.classes;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class DualPIDController {

	private PIDController leftController, rightController;
	
	public DualPIDController( double [] kLefts,
							  double [] kRights,
							  PIDSource lSrc, PIDSource rSrc,
							  PIDOutput lOut, PIDOutput rOut )
	{
		leftController = new PIDController( kLefts[1], kLefts[2], kLefts[3], lSrc, lOut );
		rightController = new PIDController( kRights[1], kRights[2], kRights[3], rSrc, rOut );
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
}
