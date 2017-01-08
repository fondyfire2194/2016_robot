/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2194.robot.RobotMap;

/**
 *
 * @author John
 */
public class ResetEncodersVerify extends Command {

	public ResetEncodersVerify() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		RobotMap.leftEncoder.reset();
		RobotMap.rightEncoder.reset();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		RobotMap.leftEncoder.reset();
		RobotMap.rightEncoder.reset();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// return
		// RobotMap.leftEncoder.getStopped()&&RobotMap.rightEncoder.getStopped();
		return (RobotMap.leftEncoder.getDistance() == 0 && RobotMap.rightEncoder
				.getDistance() == 0);
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
