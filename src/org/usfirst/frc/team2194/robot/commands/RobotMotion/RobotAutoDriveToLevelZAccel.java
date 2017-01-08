package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RobotAutoDriveToLevelZAccel extends Command {
	private double mySpeed;
	private double myTimeout;
	private double myAccelTime;
	private double myDecelTime;
	private boolean defenseSeen;
	private double levelSeenStartTime;
	private double rampIncrement;
	private double rampDecrement;
	private boolean startSlowdown;

	public RobotAutoDriveToLevelZAccel(double speed, double accelTime, double decelTime, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		mySpeed = speed;
		myTimeout = timeout;
		myAccelTime = Math.max(accelTime, .01);
		myDecelTime = Math.max(decelTime, .01);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.leftSideDrive.zeroIGain();
		Robot.rightSideDrive.zeroIGain();

		Robot.rightSideDrive.setMaxOut(mySpeed);
		Robot.leftSideDrive.setMaxOut(mySpeed);
		Robot.isAutoDriving = true;
		Robot.currentMaxSpeed = mySpeed;
		setTimeout(myTimeout);
		rampIncrement = mySpeed / (50 * myAccelTime);
		rampDecrement = mySpeed / (50 * myDecelTime);

		Robot.currentMaxSpeed = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		// SmartDashboard.putBoolean("Defense Seen", defenseSeen);
		// SmartDashboard.putBoolean("Start Slowdown", startSlowdown);
		// SmartDashboard.putNumber("Seen Time", levelSeenStartTime);
		if (!startSlowdown) {
			Robot.currentMaxSpeed = Robot.currentMaxSpeed + rampIncrement;
			if (mySpeed > 0 && Robot.currentMaxSpeed > mySpeed || mySpeed < 0 && Robot.currentMaxSpeed < mySpeed)
				Robot.currentMaxSpeed = mySpeed;
		}
		if (!defenseSeen && (Math.abs(Robot.gyroRotate.getPitch()) > 8)) {
			defenseSeen = true;
		}
		if (levelSeenStartTime == 0 && defenseSeen && Math.abs(Robot.gyroRotate.getZAcceleration()) < 1
				&& Robot.gyroRotate.getPitch() < -7)
			levelSeenStartTime = Timer.getFPGATimestamp();

		// if (levelSeenStartTime != 0
		// && defenseSeen
		// && (Math.abs(Robot.gyroRotate.getZAcceleration()) > .05 || Math
		// .abs(Robot.gyroRotate.getPitch()) > 7))
		// levelSeenStartTime = 0;

		if (defenseSeen && levelSeenStartTime != 0 && (Timer.getFPGATimestamp() - .1) > levelSeenStartTime)
			startSlowdown = true;

		if (startSlowdown) {
			Robot.currentMaxSpeed = Robot.currentMaxSpeed - rampDecrement;
			if (mySpeed > 0 && Robot.currentMaxSpeed < 0 || mySpeed < 0 && Robot.currentMaxSpeed > 0)
				Robot.currentMaxSpeed = 0;
		}
	}

	protected boolean isFinished() {
		return isTimedOut() || ((startSlowdown && Robot.currentMaxSpeed <= 0));
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.isAutoDriving = false;
		defenseSeen = false;
		startSlowdown = false;
		levelSeenStartTime = 0;
		// Robot.leftPositionLinear.setMaxOut(1); // Ethan added this block of
		// code
		// Robot.rightPositionLinear.setMaxOut(1);
		Robot.rightSideDrive.setMaxOut(1);
		Robot.leftSideDrive.setMaxOut(1);

		Robot.leftSideDrive.setPIDF(Robot.prefs.getDouble("Left Kp", .0003), Robot.prefs.getDouble("Left Ki", 0), 0, 0);
		Robot.rightSideDrive.setPIDF(Robot.prefs.getDouble("Right Kp", .0003), Robot.prefs.getDouble("Right Ki", 0), 0,
				0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
