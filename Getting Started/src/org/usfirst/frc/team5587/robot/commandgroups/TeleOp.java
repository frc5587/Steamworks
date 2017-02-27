package org.usfirst.frc.team5587.robot.commandgroups;

import org.usfirst.frc.team5587.robot.Robot;
import org.usfirst.frc.team5587.robot.commands.locomotive.ForcedMarch;
import org.usfirst.frc.team5587.robot.commands.shooter.mortar.CANMortarStick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class TeleOp extends CommandGroup {

    public TeleOp( Joystick driver, Joystick codriver ) {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.
    	requires( Robot.mortar );
    	requires( Robot.loco );
    	
    	//addParallel( new TableMarch() );
    	addParallel( new ForcedMarch( driver ) );
    	addParallel( new CANMortarStick( codriver ) );

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
