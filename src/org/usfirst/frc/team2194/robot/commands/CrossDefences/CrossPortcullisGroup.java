package org.usfirst.frc.team2194.robot.commands.CrossDefences;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.ChoiceSelections.SelectedSlotMove;
import org.usfirst.frc.team2194.robot.commands.Gyro.GyroOnOff;
import org.usfirst.frc.team2194.robot.commands.Gyro.ResetGyro;
import org.usfirst.frc.team2194.robot.commands.Gyro.UseYawComp;
import org.usfirst.frc.team2194.robot.commands.Manipulators.LiftOneLower;
import org.usfirst.frc.team2194.robot.commands.Manipulators.PortcullisChevalManipulatorLower;
import org.usfirst.frc.team2194.robot.commands.Manipulators.PortcullisChevalManipulatorRaise;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.PLSPortcullis;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotAutoDriveToRampAngle;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotCrossPortcullis;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPosition;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPositionIndependentSides;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPositionToEye;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CrossPortcullisGroup extends CommandGroup {

	public CrossPortcullisGroup() {
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
		addSequential(new CrossPortcullis());
		addSequential(new GyroOnOff(true));
		addSequential(new ResetGyro());
		addSequential(new ResetEncoders());
		addSequential(new UseYawComp(true));

		addSequential(new RobotPositionToEye(50,.5, Robot.prefs.getDouble("Pcls Eye Dist", 2),.5,true,6));
//		addParallel(new PLSPortcullis());//looks for drive position and fires manipulators
		addSequential(new RobotCrossPortcullis(20,.5,3,true,10));//contains pls for actuators
    	addSequential(new DefenseEnded());
    	addSequential(new SelectedSlotMove());//start move to goal

	}
}
