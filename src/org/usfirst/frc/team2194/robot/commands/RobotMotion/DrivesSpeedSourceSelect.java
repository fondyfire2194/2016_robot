package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DrivesSpeedSourceSelect extends Command {
	public static int modeChosen;
	private double moveValueY1;
	private double moveValueY1Squared;
	private double moveValueX1;
	private double twistValue1;
	private double moveValueY2;
	private double moveValueY2Squared;
	private double moveValueX2;
	private double twistValue2;
	private double gyroYawComp;
	private double gyroRollComp;
	private double visionComp;
	private double leftMotorSpeed;
	private double rightMotorSpeed;

	public DrivesSpeedSourceSelect() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis)
		// requires(Robot.leftSideDrive);
		// requires(Robot.rightSideDrive);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		gyroYawComp = Robot.gyroRotate.getGyroYawComp();
		gyroRollComp = Robot.gyroRotate.getGyroRollComp();
		visionComp = Robot.visionRotate.getVisionComp();

		if (!RobotMap.leftEncoder.getDirection())
			gyroRollComp = -gyroRollComp;

		if (!Robot.gyroRotate.gyroState || !Robot.useYawComp)
			gyroYawComp = 0;
		if (!Robot.gyroRotate.gyroState || !Robot.useRollComp)
			gyroRollComp = 0;

		if (!Robot.useGoalVision) {
			visionComp = 0;
		}
		moveValueX1 = Robot.oi.joystick1.getX();
		moveValueY1 = Robot.oi.joystick1.getY();
		moveValueY1Squared = moveValueY1 * moveValueY1 * moveValueY1;
		// if (moveValueY1 < 0) {
		// moveValueY1Squared = -moveValueY1Squared;
		// }

		twistValue1 = (Robot.oi.joystick1.getTwist() * 0.25);

		moveValueX2 = Robot.oi.joystick2.getX();
		moveValueY2 = Robot.oi.joystick2.getY();
		moveValueY2Squared = moveValueY2 * moveValueY2 * moveValueY2;
		// if (moveValueY2 < 0) {
		// moveValueY2Squared = -moveValueY2Squared;
		// }
		if (Robot.usePID)
			twistValue2 = (Robot.oi.joystick2.getTwist()
					* Robot.oi.joystick2.getTwist()
					* Robot.oi.joystick2.getTwist() * .25);//0.9); // was
		else
			twistValue2 = (Robot.oi.joystick2.getTwist()
					* Robot.oi.joystick2.getTwist() * Robot.oi.joystick2
					.getTwist() * .5);//

		if (Math.abs(moveValueY2Squared) < .01)
			moveValueY2Squared = 0;
		if (Math.abs(twistValue2) < .001)
			twistValue2 = 0;
		if (Math.abs(moveValueY1Squared) < .05)
			moveValueY1Squared = 0;
		if (Math.abs(twistValue1) < .1)
			twistValue1 = 0;

		modeChosen = 1; // tank drive

		if (Robot.useSingleJoystick) {// arcade drive
			modeChosen = 3;
		}
		if (Robot.isPositioning || Robot.isSamePositioning) {
			modeChosen = 5;
		}
		if (Robot.isOrienting) {
			modeChosen = 7;
		}
		if (Robot.isVisionOrienting) {
			modeChosen = 9;
		}
		if (Robot.isPitchPositioning) {
			modeChosen = 11;
		}
		if (Robot.isAutoDriving) {
			modeChosen = 13;
		}
		if (Robot.usePID) {// PID speed loop
			modeChosen++;
		}
		if (Robot.collisionOccured || Robot.stuck)
			modeChosen = 15;
		SmartDashboard.putNumber("Driving Mode", modeChosen);
		SmartDashboard.putNumber("Gyro Yaw Comp", gyroYawComp);
		SmartDashboard.putNumber("Gyro Roll Comp", gyroRollComp);
		SmartDashboard.putNumber("Vision Comp Value", visionComp);
		switch (modeChosen) {

		case 1: // tank drive no PID speed control
			Robot.leftSideDrive.setSetpoint(0);
			Robot.leftSideDrive.getPIDController().reset();
			leftMotorSpeed = -moveValueY1Squared - visionComp;
			if (leftMotorSpeed > 1)
				leftMotorSpeed = 1;
			if (leftMotorSpeed < -1)
				leftMotorSpeed = -1;
			Robot.leftSideDrive.runLeftMotor(leftMotorSpeed);

			Robot.rightSideDrive.setSetpoint(0);
			Robot.rightSideDrive.getPIDController().reset();
			rightMotorSpeed = -moveValueY2Squared + visionComp;
			if (rightMotorSpeed > 1)
				rightMotorSpeed = 1;
			if (rightMotorSpeed < -1)
				rightMotorSpeed = -1;
			Robot.rightSideDrive.runRightMotor(rightMotorSpeed);
			break;
		case 2: // tank drive PID speed control on
			Robot.rightSideDrive.setMaxOut(1);
			Robot.leftSideDrive.setMaxOut(1);
			Robot.leftSideDrive.enable();
			leftMotorSpeed = -moveValueY1Squared - visionComp;
			if (leftMotorSpeed > 1)
				leftMotorSpeed = 1;
			if (leftMotorSpeed < -1)
				leftMotorSpeed = -1;
			Robot.leftSideDrive.setSetpoint(leftMotorSpeed
					* Robot.maxEncoderCountsPerSecond);

			Robot.rightSideDrive.enable();
			rightMotorSpeed = -moveValueY2Squared + visionComp;
			if (rightMotorSpeed > 1)
				rightMotorSpeed = 1;
			if (rightMotorSpeed < -1)
				rightMotorSpeed = -1;
			Robot.rightSideDrive.setSetpoint(rightMotorSpeed
					* Robot.maxEncoderCountsPerSecond);
			break;
		case 3: // arcade drive from left joystick no PID speed control
			if (Math.abs(moveValueY2Squared) == 0) {
				moveValueY2Squared = 0;
				gyroYawComp = 0;
				gyroRollComp = 0;
				visionComp = 0;
			}
			Robot.leftSideDrive.setSetpoint(0);
			Robot.leftSideDrive.getPIDController().reset();
			leftMotorSpeed = -moveValueY2Squared + twistValue2 - gyroYawComp
					- gyroRollComp - visionComp;
			if (leftMotorSpeed > 1)
				leftMotorSpeed = 1;
			if (leftMotorSpeed < -1)
				leftMotorSpeed = -1;
			Robot.leftSideDrive.runLeftMotor(leftMotorSpeed);
			Robot.rightSideDrive.setSetpoint(0);
			Robot.rightSideDrive.getPIDController().reset();

			rightMotorSpeed = -moveValueY2Squared - twistValue2 + gyroYawComp
					+ gyroRollComp + visionComp;
			if (rightMotorSpeed > 1)
				rightMotorSpeed = 1;
			if (rightMotorSpeed < -1)
				rightMotorSpeed = -1;
			Robot.rightSideDrive.runRightMotor(rightMotorSpeed);
			break;
		case 4: // arcade drive left joystick with PID speed control
			Robot.currentMaxSpeed = 1;
			Robot.leftPositionLinear.setMaxOut(1); // Ethan added this block of
													// code
			Robot.rightPositionLinear.setMaxOut(1);
			Robot.rightSideDrive.setMaxOut(1);
			Robot.leftSideDrive.setMaxOut(1);

			if (Math.abs(moveValueY2Squared) == 0) {
				moveValueY2Squared = 0;
				gyroYawComp = 0;
				gyroRollComp = 0;
				// visionComp = 0;
			}
			leftMotorSpeed = -moveValueY2Squared + twistValue2 - gyroYawComp
					- gyroRollComp - visionComp;
			if (leftMotorSpeed > 1)
				leftMotorSpeed = 1;
			if (leftMotorSpeed < -1)
				leftMotorSpeed = -1;
			Robot.leftSideDrive.setSetpoint(leftMotorSpeed
					* Robot.maxEncoderCountsPerSecond);
			Robot.leftSideDrive.enable();

			rightMotorSpeed = -moveValueY2Squared - twistValue2 + gyroYawComp
					+ gyroRollComp + visionComp;
			if (rightMotorSpeed > 1)
				rightMotorSpeed = 1;
			if (rightMotorSpeed < -1)
				rightMotorSpeed = -1;
			Robot.rightSideDrive.setSetpoint(rightMotorSpeed
					* Robot.maxEncoderCountsPerSecond);
			Robot.rightSideDrive.enable();
			break;

		case 5:// position from command with no PID Speed
			if (!Robot.isSamePositioning) {
				gyroRollComp = 0;
				gyroYawComp = 0;
			}
			Robot.leftSideDrive.setSetpoint(0);
			Robot.leftSideDrive.getPIDController().reset();
			Robot.leftSideDrive.runLeftMotor(Robot.positionLeftOutput
					- gyroRollComp - gyroYawComp);
			Robot.rightSideDrive.setSetpoint(0);
			Robot.rightSideDrive.getPIDController().reset();
			Robot.rightSideDrive.runRightMotor(Robot.positionRightOutput
					+ gyroRollComp + gyroYawComp);
			Robot.rightPositionLinear.setMaxOut(Robot.currentMaxSpeed);
			Robot.leftPositionLinear.setMaxOut(Robot.currentMaxSpeed);

			break;
		case 6: // position from command with PID speed
			if (!Robot.isSamePositioning) {
				gyroRollComp = 0;
				gyroYawComp = 0;
			}
			Robot.leftSideDrive.setSetpoint((Robot.positionLeftOutput
					- gyroYawComp - gyroRollComp)
					* Robot.maxEncoderCountsPerSecond);
			Robot.leftSideDrive.enable();
			Robot.rightSideDrive.setSetpoint((Robot.positionRightOutput
					+ gyroYawComp + gyroRollComp)
					* Robot.maxEncoderCountsPerSecond);
			Robot.rightSideDrive.enable();
			Robot.rightSideDrive.setMaxOut(Robot.currentMaxSpeed);
			Robot.leftSideDrive.setMaxOut(Robot.currentMaxSpeed);

			break;
		case 7:// orienting to gyro angle with no speed PID
			Robot.leftSideDrive.setSetpoint(0);
			Robot.leftSideDrive.getPIDController().reset();
			Robot.rightSideDrive.setSetpoint(0);
			Robot.rightSideDrive.getPIDController().reset();
			Robot.leftSideDrive.runLeftMotor(Robot.positionLeftOutput);
			Robot.rightSideDrive.runRightMotor(-Robot.positionLeftOutput);

		case 8:// orient to gyro angle with speed PID
			Robot.leftSideDrive.setSetpoint(Robot.positionLeftOutput
					* Robot.maxEncoderCountsPerSecond);
			Robot.rightSideDrive.setSetpoint(-Robot.positionLeftOutput
					* Robot.maxEncoderCountsPerSecond);
			Robot.leftSideDrive.enable();
			Robot.rightSideDrive.enable();
			break;

		case 9:// orienting to vision position with no speed PID
			Robot.leftSideDrive.setSetpoint(0);
			Robot.leftSideDrive.getPIDController().reset();
			Robot.rightSideDrive.setSetpoint(0);
			Robot.rightSideDrive.getPIDController().reset();
			Robot.leftSideDrive.runLeftMotor(Robot.positionLeftOutput);
			Robot.rightSideDrive.runRightMotor(-Robot.positionLeftOutput);

		case 10:// orient to vision position with speed PID
			Robot.leftSideDrive.setSetpoint(Robot.positionLeftOutput
					* Robot.maxEncoderCountsPerSecond);
			Robot.rightSideDrive.setSetpoint(-Robot.positionLeftOutput
					* Robot.maxEncoderCountsPerSecond);
			Robot.leftSideDrive.enable();
			Robot.rightSideDrive.enable();
			break;

		case 11:// position to pitch angle with no PID Speed
			Robot.leftSideDrive.setSetpoint(0);
			Robot.leftSideDrive.getPIDController().reset();
			Robot.leftSideDrive.runLeftMotor(Robot.positionLeftOutput
					- gyroRollComp - gyroYawComp);
			Robot.rightSideDrive.setSetpoint(0);
			Robot.rightSideDrive.getPIDController().reset();
			Robot.rightSideDrive.runRightMotor(Robot.positionLeftOutput
					+ gyroRollComp + gyroYawComp);
			Robot.rightPositionLinear.setMaxOut(Robot.currentMaxSpeed);
			Robot.leftPositionLinear.setMaxOut(Robot.currentMaxSpeed);
			break;

		case 12: // position to pitch angle with PID speed
			Robot.leftSideDrive.setSetpoint((Robot.positionLeftOutput
					- gyroYawComp - gyroRollComp)
					* Robot.maxEncoderCountsPerSecond);
			Robot.leftSideDrive.enable();
			Robot.rightSideDrive.setSetpoint((Robot.positionLeftOutput
					+ gyroYawComp + gyroRollComp)
					* Robot.maxEncoderCountsPerSecond);
			Robot.rightSideDrive.enable();
			Robot.rightSideDrive.setMaxOut(Robot.currentMaxSpeed);
			Robot.leftSideDrive.setMaxOut(Robot.currentMaxSpeed);

			break;

		case 13: // auto drive no PID speed control
			Robot.leftSideDrive.setSetpoint(0);
			Robot.leftSideDrive.getPIDController().reset();
			Robot.leftSideDrive.runLeftMotor(Robot.currentMaxSpeed
					- gyroYawComp - gyroRollComp - visionComp);
			Robot.rightSideDrive.setSetpoint(0);
			Robot.rightSideDrive.getPIDController().reset();
			Robot.rightSideDrive.runRightMotor(Robot.currentMaxSpeed
					- twistValue2 + gyroYawComp + gyroRollComp + visionComp);
			break;
		case 14: // auto drive with PID speed control
			Robot.rightSideDrive.setMaxOut(1);
			Robot.leftSideDrive.setMaxOut(1);
			Robot.leftSideDrive.setSetpoint((Robot.currentMaxSpeed
					- gyroYawComp - gyroRollComp - visionComp)
					* Robot.maxEncoderCountsPerSecond);
			Robot.leftSideDrive.enable();
			Robot.rightSideDrive.setSetpoint((Robot.currentMaxSpeed
					+ gyroYawComp + gyroRollComp + visionComp)
					* Robot.maxEncoderCountsPerSecond);
			Robot.rightSideDrive.enable();
			break;

		case 15:
			Robot.rightSideDrive.disable();
			Robot.leftSideDrive.disable();
			RobotMap.driveRightMotor1.set(0);
			RobotMap.driveLeftMotor1.set(0);

			break;
		}
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
