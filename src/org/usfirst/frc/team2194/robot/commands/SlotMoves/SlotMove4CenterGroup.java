package org.usfirst.frc.team2194.robot.commands.SlotMoves;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.Gyro.UseYawComp;
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

public class SlotMove4CenterGroup extends CommandGroup {

	public SlotMove4CenterGroup() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		addSequential(new Slot4MoveCenter());
		
		
		
		addSequential(new PositionAngleShooter(140, .5,10));

		addSequential(new UseYawComp(false));

		addSequential(new SetPIDMotors(true));
		addSequential(new RobotOrient(340, .4, 5));
		addSequential(new SetPIDMotors(false));
		
		addSequential(new ResetEncodersVerify());
		
		addParallel(new RobotPosition(40, .5, 1, true, 5)); // move to
																		// turn
										// point
		
//		addSequential(new RobotOrient(40, .5, 5));
//		addSequential(new RobotRotateToFindTarget(50, .5, true, 5));
        
		addSequential(new PositionAngleShooterToCamera(.25, 4));
		
//		addSequential(new RobotAutoDriveToVisionTarget(.5, true, 5));
		addParallel(new PositionAngleShooterToCamera(.25, 4));
		addSequential(new ShootBall());

	   	addSequential(new SlotEnded());

	}
}
