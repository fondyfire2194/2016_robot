package org.usfirst.frc.team2194.robot.commands.CrossDefences;

import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.ChoiceSelections.SelectedSlotMove;
import org.usfirst.frc.team2194.robot.commands.Gyro.GyroOnOff;
import org.usfirst.frc.team2194.robot.commands.Gyro.ResetGyro;
import org.usfirst.frc.team2194.robot.commands.Gyro.UseYawComp;
import org.usfirst.frc.team2194.robot.commands.Manipulators.LiftOneLower;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotAutoDriveToLevel;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPositionIndependentSides;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CrossRampartsGroup extends CommandGroup {
    
    public  CrossRampartsGroup() {
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
    	addSequential(new CrossRamparts());
		addSequential(new GyroOnOff(true));
		addSequential(new ResetGyro());
		addSequential(new ResetEncoders());
		addSequential(new UseYawComp(true));
//		addSequential(new LiftOneLower());
		addSequential(new ResetEncoders());
		addSequential(new TimeDelay(.05));
		
		addSequential(new RobotAutoDriveToLevel(.75,true,15));//cross defense
    	addSequential(new DefenseEnded());//message only
    	addSequential(new SelectedSlotMove());//start move to goal
    }
}
