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
public class RobotVisionOrient extends Command {
	private double speed;
	private double xPosition;

	public RobotVisionOrient(double xPosition, double speed) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.gyroRotate); //Ethan added requires back
		// requires(Robot.leftSideDrive);
		// requires(Robot.rightSideDrive);
		requires(Robot.visionRotate);

		this.speed = speed;
		this.xPosition = xPosition;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.visionRotate.setMaxOut(speed);
		Robot.rightSideDrive.setOutputRange(-speed, speed);
		Robot.leftSideDrive.setOutputRange(-speed, speed);
		Robot.visionRotate.setSetpoint(xPosition);
		Robot.visionRotate.enablePID();
		Robot.isVisionOrienting = true;
		setTimeout(600);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return (isTimedOut() || Robot.visionRotate.onTargetX())
				|| Robot.xCameraVal == 999;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.gyroRotate.disablePID();
		Robot.isVisionOrienting = false;
		Robot.rightSideDrive.setOutputRange(-1, 1); // Ethan
		Robot.leftSideDrive.setOutputRange(-1, 1); // Ethan
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
