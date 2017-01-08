package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RobotAutoDriveToLevel extends Command {
	private double mySpeed;
	private double myTimeout;
	private boolean defenseSeen;
	private double levelSeenStartTime;
	private double rampIncrement;
	private boolean lookForLevel;

	public RobotAutoDriveToLevel(double speed, boolean disableWhenDone, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		mySpeed = speed;
		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
		lookForLevel = false;
		
		Robot.leftSideDrive.zeroIGain();
		Robot.rightSideDrive.zeroIGain();

		Robot.rightSideDrive.setMaxOut(mySpeed);
		Robot.leftSideDrive.setMaxOut(mySpeed);
		Robot.isAutoDriving = true;
		Robot.currentMaxSpeed = mySpeed;
		setTimeout(myTimeout);
		rampIncrement = mySpeed / 50;
		Robot.currentMaxSpeed = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		SmartDashboard.putBoolean("D seen", defenseSeen);
		Robot.currentMaxSpeed = Robot.currentMaxSpeed + rampIncrement;
		if (mySpeed > 0 && Robot.currentMaxSpeed > mySpeed || mySpeed < 0 && Robot.currentMaxSpeed < mySpeed)
			Robot.currentMaxSpeed = mySpeed;

		if (!defenseSeen && (Math.abs(Robot.gyroRotate.getPitch()) > 4)) {
			defenseSeen = true;
		}
		if ((levelSeenStartTime == 0 && defenseSeen && Math.abs(Robot.gyroRotate.getPitch()) <= 3)) {
			levelSeenStartTime = Timer.getFPGATimestamp();
		}
		if (levelSeenStartTime != 0 && (Math.abs(Robot.gyroRotate.getPitch()) > 3)) {
			levelSeenStartTime = 0;
		}
		
		if (Robot.gyroRotate.getPitch() <= -10){
			lookForLevel = true;
		}
	}

	protected boolean isFinished() {
		return isTimedOut() || (//Robot.gyroRotate.getPitch() <= -10 && 
				defenseSeen //&& (Timer.getFPGATimestamp() - levelSeenStartTime) > .25 
				&& lookForLevel && Robot.gyroRotate.getPitch() >= -2);
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.isAutoDriving = false;
		defenseSeen = false;
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
