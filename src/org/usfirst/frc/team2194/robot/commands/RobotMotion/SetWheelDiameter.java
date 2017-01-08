package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SetWheelDiameter extends Command {

	public SetWheelDiameter() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveLeftWheelDiameter = SmartDashboard
				.getNumber("Left Wheel Diameter");
		Robot.driveRightWheelDiameter = SmartDashboard
				.getNumber("Right Wheel Diameter");
		Robot.wheelDiameterRatio = Robot.driveLeftWheelDiameter
				/ Robot.driveRightWheelDiameter;
		Robot.encoderCountsPerLoGearInch = (Robot.encoderCountsPerMotorRev
				* Robot.loGearMotorToWheelRatio) / (Robot.driveLeftWheelDiameter
				* 3.1417);
		Robot.encoderCountsPerHiGearInch = (Robot.encoderCountsPerMotorRev
				* Robot.hiGearMotorToWheelRatio) / (Robot.driveLeftWheelDiameter
				* 3.1417);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
