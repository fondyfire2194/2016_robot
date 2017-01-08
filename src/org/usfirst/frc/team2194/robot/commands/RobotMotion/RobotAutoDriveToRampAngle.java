package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotAutoDriveToRampAngle extends Command {
	private double mySpeed;
	private double myTimeout;
	private double myAngle;

	public RobotAutoDriveToRampAngle(double speed, double angle, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		mySpeed = speed;
		myTimeout = timeout;
		myAngle = angle;
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
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || Math.abs(Robot.gyroRotate.getPitch()) >= myAngle;

	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.leftSideDrive.setPIDF(Robot.prefs.getDouble("Left Kp", .0003),
				Robot.prefs.getDouble("Left Ki", 0), 0, 0);
		Robot.rightSideDrive.setPIDF(Robot.prefs.getDouble("Right Kp", .0003),
				Robot.prefs.getDouble("Right Ki", 0), 0, 0);

		Robot.isAutoDriving = false;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
