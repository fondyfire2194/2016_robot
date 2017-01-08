package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotPositionIndependentSides extends Command {
	private double leftSideDistanceEncoderCounts;
	private double myLeftSideSpeed;
	private double rightSideDistanceEncoderCounts;
	private double myRightSideSpeed;
	private double inPositionBandEncoderCounts;
	private boolean myDisableWhenDone;
	private double myTimeout;

	// side distances are in inches
	// side speeds are in per unit where .25 = 25%
	// inPositionband is in inches

	public RobotPositionIndependentSides(double leftSideDistance, double leftSideSpeed, double rightSideDistance, double rightSideSpeed,
			double inPositionBand, boolean disableWhenDone, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		leftSideDistanceEncoderCounts = leftSideDistance * Robot.encoderCountsPerLoGearInch;
		myLeftSideSpeed = leftSideSpeed;
		rightSideDistanceEncoderCounts = (rightSideDistance * Robot.encoderCountsPerLoGearInch
				* Robot.wheelDiameterRatio);
		myRightSideSpeed = rightSideSpeed;
		inPositionBandEncoderCounts = inPositionBand * Robot.encoderCountsPerLoGearInch;
		myDisableWhenDone = disableWhenDone;
		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
//		Robot.imu.zeroYaw();
		Robot.leftSideDrive.zeroIGain();
		Robot.rightSideDrive.zeroIGain();
        
		Robot.leftPositionLinear.setMaxOut(myLeftSideSpeed);
		Robot.rightPositionLinear.setMaxOut(myRightSideSpeed);
		Robot.rightSideDrive.setMaxOut(myRightSideSpeed);
		Robot.leftSideDrive.setMaxOut(myLeftSideSpeed);

		Robot.currentMaxSpeed = myLeftSideSpeed;
		// Robot.gyroTarget = Robot.gyroRotate.getGyroAngle();
		Robot.leftPositionLinear.setSetpoint(leftSideDistanceEncoderCounts);
		Robot.rightPositionLinear.setSetpoint(rightSideDistanceEncoderCounts);
		Robot.leftPositionLinear.enable();
		Robot.rightPositionLinear.enable();
		Robot.isPositioning = true;
		if (myLeftSideSpeed == myRightSideSpeed & leftSideDistanceEncoderCounts == rightSideDistanceEncoderCounts)
			Robot.isSamePositioning = true;
		setTimeout(myTimeout);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || (Robot.leftPositionLinear.inPosition(inPositionBandEncoderCounts)
				&& Robot.rightPositionLinear.inPosition(inPositionBandEncoderCounts));
	}

	// Called once after isFinished returns true
	protected void end() {
		if (myDisableWhenDone) {
			Robot.leftPositionLinear.disablePID();
			Robot.rightPositionLinear.disablePID();
		}
		Robot.isPositioning = false;
		Robot.isSamePositioning = false;
		Robot.leftPositionLinear.setMaxOut(1); // Ethan added this block of code
		Robot.rightPositionLinear.setMaxOut(1);
		Robot.rightSideDrive.setMaxOut(1);
		Robot.leftSideDrive.setMaxOut(1);
		
		Robot.leftSideDrive.setPIDF(Robot.prefs.getDouble("Left Kp", .0003), Robot.prefs.getDouble("Left Ki",0),0,0);
        Robot.rightSideDrive.setPIDF(Robot.prefs.getDouble("Right Kp", .0003), Robot.prefs.getDouble("Right Ki",0),0,0);
        
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
