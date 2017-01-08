package org.usfirst.frc.team2194.robot.AutonomousShoot;

import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.Shooter.ShootPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoShootGroup extends CommandGroup {
    
    public  AutoShootGroup() {
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
    	addSequential(new CameraPosition());
    	addSequential(new TimeDelay(1));
    	addSequential(new SquareToGoalTrue());
    	addSequential(new TimeDelay(3));
    	addSequential(new SquareToGoalFalse());
    	addSequential(new ShootPosition());
//    	addSequential(new ShootBall());
    }
}
