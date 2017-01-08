package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotPositionToEye extends Command {
	private double distanceEncoderCounts;
	private double mySpeed;
	private double eyeDistanceToMoveEncoderCounts;
	private double inPositionBandEncoderCounts;
	private boolean myDisableWhenDone;
	private double myTimeout;
	private double rampIncrement;
	private boolean eyeSeen;
	private double targetLeftPosition;
	private double targetRightPosition;

	// side distances are in inches
	// side speeds are in per unit where .25 = 25%
	// inPositionband is in inches

	public RobotPositionToEye(double distance, double speed,
			double eyeDistanceToMove, double inPositionBand,
			boolean disableWhenDone, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		distanceEncoderCounts = distance * Robot.encoderCountsPerLoGearInch;
		mySpeed = speed;
		eyeDistanceToMoveEncoderCounts = eyeDistanceToMove
				* Robot.encoderCountsPerLoGearInch;
		inPositionBandEncoderCounts = inPositionBand
				* Robot.encoderCountsPerLoGearInch;
		myDisableWhenDone = disableWhenDone;
		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.leftSideDrive.zeroIGain();
		Robot.rightSideDrive.zeroIGain();
		rampIncrement = mySpeed / 50;
		Robot.currentMaxSpeed = 0;

		Robot.leftPositionLinear.setSetpoint(distanceEncoderCounts);
		Robot.rightPositionLinear.setSetpoint(distanceEncoderCounts);
		Robot.leftPositionLinear.enable();
		Robot.rightPositionLinear.enable();
		Robot.isPositioning = true;
		Robot.isSamePositioning = true;
		setTimeout(myTimeout);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.currentMaxSpeed = Robot.currentMaxSpeed + rampIncrement;
		if (Robot.currentMaxSpeed > mySpeed)
			Robot.currentMaxSpeed = mySpeed;
		Robot.leftPositionLinear.setMaxOut(Robot.currentMaxSpeed);
		Robot.rightPositionLinear.setMaxOut(Robot.currentMaxSpeed);
		Robot.rightSideDrive.setMaxOut(Robot.currentMaxSpeed);
		Robot.leftSideDrive.setMaxOut(Robot.currentMaxSpeed);

		if (eyeDistanceToMoveEncoderCounts !=0 && !eyeSeen && RobotMap.rampEdgeDetect.get()) {
			targetLeftPosition = RobotMap.leftEncoder.getDistance()
					+ eyeDistanceToMoveEncoderCounts;
			targetRightPosition = RobotMap.rightEncoder.getDistance()
					+ eyeDistanceToMoveEncoderCounts;
			Robot.leftPositionLinear.setSetpoint(targetLeftPosition);
			Robot.rightPositionLinear.setSetpoint(targetRightPosition);
			eyeSeen = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut()
				|| (Robot.leftPositionLinear
						.inPosition(inPositionBandEncoderCounts) && Robot.rightPositionLinear
						.inPosition(inPositionBandEncoderCounts));
	}

	// Called once after isFinished returns true
	protected void end() {
		if (myDisableWhenDone) {
			Robot.leftPositionLinear.disablePID();
			Robot.rightPositionLinear.disablePID();
		}
		Robot.isPositioning = false;
		Robot.isSamePositioning = false;
		eyeSeen = false;
		Robot.leftPositionLinear.setMaxOut(1); // Ethan added this block of code
		Robot.rightPositionLinear.setMaxOut(1);
		Robot.rightSideDrive.setMaxOut(1);
		Robot.leftSideDrive.setMaxOut(1);

		Robot.leftSideDrive.setPIDF(Robot.prefs.getDouble("Left Kp", .0003),
				Robot.prefs.getDouble("Left Ki", 0), 0, 0);
		Robot.rightSideDrive.setPIDF(Robot.prefs.getDouble("Right Kp", .0003),
				Robot.prefs.getDouble("Right Ki", 0), 0, 0);

		// Robot.leftSideDrive.setSetpoint(0.0);
		// Robot.leftSideDrive.enable();
		// Robot.rightSideDrive.setSetpoint(0.0);
		// Robot.rightSideDrive.enable();

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
