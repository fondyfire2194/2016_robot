package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotOKCheck extends Command {
	private double leftDriveStartTime;
	private double rightDriveStartTime;
	private boolean leftStuckOn;
	private boolean rightStuckOn;
	private boolean collisionOn;

	public RobotOKCheck() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// checks for motion being commanded to drive motors
		// if it is then it checks that both encoder are not stopped
		// also checks motion using gyro
		// checks for collision using gyro jerk calculation (rate of change of
		// acceleration

	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.leftMotionCommanded = (Math.abs(RobotMap.driveLeftMotor1.get()) > .1);
		Robot.rightMotionCommanded = (Math.abs(RobotMap.driveRightMotor1.get()) > .1);

		if (leftDriveStartTime == 0 && Robot.leftMotionCommanded)
			leftDriveStartTime = Timer.getFPGATimestamp();
		if (rightDriveStartTime == 0 && Robot.rightMotionCommanded)
			rightDriveStartTime = Timer.getFPGATimestamp();

		Robot.leftSideStuck = Robot.leftMotionCommanded
				&& RobotMap.leftEncoder.getStopped()
				&& (Timer.getFPGATimestamp() - leftDriveStartTime) > 1;

		Robot.rightSideStuck = Robot.rightMotionCommanded
				&& (Timer.getFPGATimestamp() - rightDriveStartTime) > 1;

		if (Robot.leftSideStuck && !leftStuckOn) {
			Robot.leftStuckCounter++;
			leftStuckOn = true;
		}
		leftStuckOn = Robot.leftSideStuck;
		
		if (Robot.rightSideStuck && !rightStuckOn) {
			Robot.rightStuckCounter++;
			rightStuckOn = true;
		}
		rightStuckOn = Robot.rightSideStuck;
		
		if (!Robot.leftMotionCommanded)
			leftDriveStartTime = 0;
		if (!Robot.rightMotionCommanded)
			rightDriveStartTime = 0;

		if (Robot.gyroRotate.collisionDetect() && !collisionOn) {
			Robot.collisionCounter++;
			collisionOn = true;
		}
		collisionOn = Robot.gyroRotate.collisionDetect();

		if (Robot.checkCollision
				&& (Robot.leftMotionCommanded || Robot.rightMotionCommanded)
				&& Robot.gyroRotate.collisionDetect())
			Robot.collisionOccured = true;
		if (Robot.checkStuck
				&& (Robot.leftSideStuck || Robot.rightSideStuck || !Robot.gyroRotate
						.isMoving()))
			Robot.stuck = true;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
