package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotRotateToFindTarget extends Command {
	private double encoderCounts;
	private double mySpeed;
	private boolean myDirection;
	private double myTimeout;

	// side distances are in inches
	// side speeds are in per unit where .25 = 25%
	// inPositionband is in inches

	public RobotRotateToFindTarget(double distance, double speed,
			boolean direction, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		encoderCounts = distance * Robot.encoderCountsPerLoGearInch;
		mySpeed = speed;
		myDirection = direction;
		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.leftSideDrive.zeroIGain();
		Robot.rightSideDrive.zeroIGain();

		Robot.leftPositionLinear.setMaxOut(mySpeed);
		Robot.rightPositionLinear.setMaxOut(mySpeed);
		Robot.rightSideDrive.setMaxOut(mySpeed);
		Robot.leftSideDrive.setMaxOut(mySpeed);

		Robot.currentMaxSpeed = mySpeed;
		// Robot.gyroTarget = Robot.gyroRotate.getGyroAngle();
		if (myDirection) {
			Robot.leftPositionLinear.setSetpoint(encoderCounts);
			Robot.rightPositionLinear.setSetpoint(-encoderCounts);
		} else {
			Robot.leftPositionLinear.setSetpoint(-encoderCounts);
			Robot.rightPositionLinear.setSetpoint(encoderCounts);
		}
		Robot.leftPositionLinear.enable();
		Robot.rightPositionLinear.enable();
		Robot.isPositioning = true;
		Robot.isSamePositioning = true;
		setTimeout(myTimeout);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || Robot.visionRotate.targetMidScreenX();
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.leftPositionLinear.getPIDController().reset();
		Robot.rightPositionLinear.getPIDController().reset();

		Robot.isPositioning = false;
		Robot.isSamePositioning = false;
		Robot.leftPositionLinear.setMaxOut(1); // Ethan added this block of code
		Robot.rightPositionLinear.setMaxOut(1);
		Robot.rightSideDrive.setMaxOut(1);
		Robot.leftSideDrive.setMaxOut(1);

		Robot.leftSideDrive.setPIDF(Robot.prefs.getDouble("Left Kp", .0003),
				Robot.prefs.getDouble("Left Ki", 0), 0, 0);
		Robot.rightSideDrive.setPIDF(Robot.prefs.getDouble("Right Kp", .0003),
				Robot.prefs.getDouble("Right Ki", 0), 0, 0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
