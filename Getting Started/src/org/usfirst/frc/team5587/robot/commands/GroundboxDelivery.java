package org.usfirst.frc.team5587.robot.commands;

import org.usfirst.frc.team5587.robot.commands.groundbox.Regret;
import org.usfirst.frc.team5587.robot.commands.groundbox.Vomit;
import org.usfirst.frc.team5587.robot.commands.locomotive.auto.DutifulProgression;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class GroundboxDelivery extends CommandGroup {

    public GroundboxDelivery() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	
    	addSequential(new Regret());
    	addParallel(new Vomit());
    	addParallel(new DutifulProgression(-10, 8 ));
    }
}
