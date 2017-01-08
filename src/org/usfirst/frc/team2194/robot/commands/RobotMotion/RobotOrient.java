/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2194.robot.Robot;

/**
 *
 * @author John
 */
public class RobotOrient extends Command {
	private double mySpeed;
	private double myAngle;
	private double myTimeout;

	public RobotOrient(double angle, double speed, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		 requires(Robot.gyroRotate); //Ethan added requires back
		// requires(Robot.leftSideDrive);
		// requires(Robot.rightSideDrive);

		mySpeed = speed;
		myAngle = angle;
		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.gyroRotate.setMaxOut(mySpeed);
		Robot.rightSideDrive.setOutputRange(-mySpeed, mySpeed);
		Robot.leftSideDrive.setOutputRange(-mySpeed, mySpeed);
		Robot.gyroRotate.setSetpoint(myAngle);
		Robot.gyroRotate.enablePID();
		Robot.isOrienting = true;
		Robot.motionSeen = false;
		setTimeout(myTimeout);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.gyroRotate.setMaxOut(mySpeed);

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || Robot.gyroRotate.inPosition();

	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.gyroRotate.getPIDController().reset();
		Robot.isOrienting = false;
		Robot.motionSeen = false;
		Robot.rightSideDrive.setOutputRange(-1, 1); // Ethan
		Robot.leftSideDrive.setOutputRange(-1, 1); // Ethan

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
