package org.usfirst.frc.team5587.robot.commandgroups;

import org.usfirst.frc.team5587.robot.commands.DropBalls;
import org.usfirst.frc.team5587.robot.commands.FoldIn;
import org.usfirst.frc.team5587.robot.commands.Wait;
import org.usfirst.frc.team5587.robot.commands.locomotive.auto.DutifulTiming;
import org.usfirst.frc.team5587.robot.commands.locomotive.auto.Gyrate;
import org.usfirst.frc.team5587.robot.commands.locomotive.auto.ResetGyro;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DropAutoBlue extends CommandGroup {

    public DropAutoBlue() {
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
    	
    	addSequential(new DropBalls());
    	addSequential(new Wait(4));
    	addSequential(new DutifulTiming(.5,-1));
    	addSequential(new FoldIn());
    	addSequential(new Gyrate(-45,3));
    	addSequential(new DutifulTiming(3,-1));
    }
}
