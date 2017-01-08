package org.usfirst.frc.team2194.robot.commands.SlotMoves;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.Gyro.UseYawComp;
import org.usfirst.frc.team2194.robot.commands.Manipulators.PortcullisChevalManipulatorRaise;
import org.usfirst.frc.team2194.robot.commands.PIDSets.SetPIDMotors;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.ResetEncodersVerify;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotOrient;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPosition;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotVisionOrient;
import org.usfirst.frc.team2194.robot.commands.Shooter.PositionAngleShooter;
import org.usfirst.frc.team2194.robot.commands.Shooter.PositionAngleShooterToCamera;
import org.usfirst.frc.team2194.robot.commands.Shooter.ShootBall;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SlotMove1LeftGroup extends CommandGroup {

	public SlotMove1LeftGroup() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// command group starts after robot is through defense

		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		addSequential(new Slot1MoveLeft());
		addSequential(new PortcullisChevalManipulatorRaise());
		// addSequential(new ResetEncoders());

		addParallel(new PositionAngleShooter(145, .7, 10)); //was 140
		addSequential(new RobotPosition(202, .5, 2, true, 5));

		addSequential(new UseYawComp(false));

		addSequential(new SetPIDMotors(true));
		addSequential(new RobotOrient(50, .4, 5));
		addSequential(new SetPIDMotors(false));

		addSequential(new ResetEncodersVerify());

//		addParallel(new RobotVisionOrient(Robot.visionRotate.getActiveXTarget(), .25));
		addSequential(new PositionAngleShooterToCamera(.25, 4));

		// addSequential(new RobotAutoDriveToVisionTarget(.5, true, 5));
		addParallel(new PositionAngleShooterToCamera(.25, 4));
		addSequential(new ShootBall());

		addSequential(new SlotEnded());

	}
}
