package org.usfirst.frc.team2194.robot.commands.CrossDefences;

import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.ChoiceSelections.SelectedSlotMove;
import org.usfirst.frc.team2194.robot.commands.Gyro.GyroOnOff;
import org.usfirst.frc.team2194.robot.commands.Gyro.ResetGyro;
import org.usfirst.frc.team2194.robot.commands.Gyro.UseYawComp;
import org.usfirst.frc.team2194.robot.commands.Manipulators.LiftOneLower;
import org.usfirst.frc.team2194.robot.commands.Manipulators.PortcullisChevalManipulatorLower;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPosition;
import org.usfirst.frc.team2194.robot.commands.Shooter.PositionAngleShooter;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CrossLowBarGroup extends CommandGroup {

	public CrossLowBarGroup() {
		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// A command group will require all of the subsystems that each member
		// would require.
		// e.g. if Command1 requires chassis, and Command2 requires arm,
		// a CommandGroup containing them would require both the chassis and the
		// arm.
		//make the slowest command second in a parallel / serial pair.
		//program does not wait on parallel line ending
		addSequential(new CrossLowBar());
		addSequential(new GyroOnOff(true));
		addSequential(new ResetGyro());
//		addSequential(new ResetEncoders());
		addSequential(new UseYawComp(true));
		addSequential(new LiftOneLower());

		addSequential(new ResetEncoders());
		addSequential(new TimeDelay(.05));
		
		addSequential(new PortcullisChevalManipulatorLower());
		addParallel(new PositionAngleShooter(169, .7,10));//lower shooter
		addSequential(new RobotPosition(65, .5, 1, true, 5)); // move to defense
		
//		addParallel(new PositionAngleShooter(169, .7,10));//lower shooter
//		addSequential(new PortcullisChevalManipulatorLower());
		

//		addSequential(new ResetEncoders());
		addSequential(new TimeDelay(.1));
		
		addParallel(new PositionAngleShooter(160, .7,10));//raise shooter
		addSequential(new RobotPosition(107, .5, 2, true, 5)); // move through defense
		
//		addSequential(new PositionAngleShooter(130, .7,10));//raise shooter
		
    	addSequential(new DefenseEnded());
    	addSequential(new SelectedSlotMove());//start move to goal

// move after through defense continues in slot move
	}
}
