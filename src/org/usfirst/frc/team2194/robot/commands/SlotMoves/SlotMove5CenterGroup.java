package org.usfirst.frc.team2194.robot.commands.SlotMoves;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.Gyro.UseYawComp;
import org.usfirst.frc.team2194.robot.commands.Manipulators.PortcullisChevalManipulatorRaise;
import org.usfirst.frc.team2194.robot.commands.PIDSets.SetPIDMotors;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.ResetEncodersVerify;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotAutoDriveToVisionTarget;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotOrient;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPosition;
import org.usfirst.frc.team2194.robot.commands.Shooter.PositionAngleShooter;
import org.usfirst.frc.team2194.robot.commands.Shooter.PositionAngleShooterToCamera;
import org.usfirst.frc.team2194.robot.commands.Shooter.ShootBall;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SlotMove5CenterGroup extends CommandGroup {

	public SlotMove5CenterGroup() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		addSequential(new Slot5MoveCenter());
		
//		addSequential(new PortcullisChevalManipulatorRaise());
		
		
		addSequential(new ResetEncodersVerify());
//		addSequential(new TimeDelay(.1));
		
//		addParallel(new PositionAngleShooter(140, .5,10)); //Ethan

		addSequential(new RobotPosition(30, .5, 1, true, 5)); // move to
																		// turn
																		// point
		addSequential(new UseYawComp(false));
		
		addSequential(new SetPIDMotors(true));
		addSequential(new RobotOrient(345, .5, 5));
		addSequential(new SetPIDMotors(false));
		
		
		addParallel(new PositionAngleShooterToCamera(.25,10));
		addSequential(new RobotAutoDriveToVisionTarget(.5, true, 5));
		addSequential(new ShootBall());

	   	addSequential(new SlotEnded());

	}
}
