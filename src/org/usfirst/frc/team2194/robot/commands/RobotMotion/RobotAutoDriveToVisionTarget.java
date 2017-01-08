package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotAutoDriveToVisionTarget extends Command {
	private double mySpeed;
	private double myTimeout;

	public RobotAutoDriveToVisionTarget(double speed,
			boolean disableWhenDone, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		mySpeed = speed;
		myTimeout = timeout;
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
		return isTimedOut()
				|| 	Robot.visionRotate.onTarget();
			
		}

	// Called once after isFinished returns true
	protected void end() {
		Robot.currentMaxSpeed = 0;
		Robot.isAutoDriving = false;
		Robot.leftSideDrive.setPIDF(Robot.prefs.getDouble("Left Kp", .0003), Robot.prefs.getDouble("Left Ki",0),0,0);
        Robot.rightSideDrive.setPIDF(Robot.prefs.getDouble("Right Kp", .0003), Robot.prefs.getDouble("Right Ki",0),0,0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
