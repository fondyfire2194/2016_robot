package org.usfirst.frc.team2194.robot.commands.CrossDefences;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.SetStuckDetect;
import org.usfirst.frc.team2194.robot.commands.ChoiceSelections.SelectedSlotMove;
import org.usfirst.frc.team2194.robot.commands.Gyro.GyroOnOff;
import org.usfirst.frc.team2194.robot.commands.Gyro.ResetGyro;
import org.usfirst.frc.team2194.robot.commands.Gyro.UseYawComp;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.PLSChevalDeFrise;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotCrossChevalDeFrise;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPositionToEye;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CrossChevalDeFriseGroup extends CommandGroup {

	public CrossChevalDeFriseGroup() {
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
		addSequential(new CrossChevalDeFrise());//message
		addSequential(new GyroOnOff(true));
		addSequential(new ResetGyro());
		addSequential(new ResetEncoders());
		addSequential(new UseYawComp(true));
		addParallel(new SetStuckDetect(true));

		addSequential(new RobotPositionToEye(50, .5, Robot.prefs.getDouble(
				"Chdf Eye Dist", 2), .5, true, 6));

		// addParallel(new PLSChevalDeFrise());//looks for drive position and
		// fires manipulators
		addSequential(new RobotCrossChevalDeFrise(20, .25, 2, true, 10));
		addSequential(new DefenseEnded());//message
		addSequential(new SelectedSlotMove());
	}
}
