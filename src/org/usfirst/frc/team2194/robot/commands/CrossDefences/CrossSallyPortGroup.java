package org.usfirst.frc.team2194.robot.commands.CrossDefences;

import org.usfirst.frc.team2194.robot.commands.Manipulators.LiftOneRaise;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPositionIndependentSides;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CrossSallyPortGroup extends CommandGroup {
    
    public  CrossSallyPortGroup() {
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
    	addSequential (new CrossSallyPort());
    	addSequential(new DefenseEnded());

    }
}
