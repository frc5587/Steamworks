package org.usfirst.frc.team5587.classes;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class TableTenant {

	public NetworkTable shooterPIDTable;
	public NetworkTable turntablePIDTable;
	public NetworkTable visionTable;
	public NetworkTable winchTable;
	
	public TableTenant()
	{
		shooterPIDTable = NetworkTable.getTable( "Flywheel PID" );
		turntablePIDTable = NetworkTable.getTable( "Turntable PID" );
		visionTable = NetworkTable.getTable( "/GRIP/postprocessed" );
		winchTable = NetworkTable.getTable( "Winch" );
	}
}
