package org.usfirst.frc.team2194.robot.commands.Manipulators;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PortcullisChevalManipulatorRaiseAtPitch extends Command {
	private double myPitch;

	public PortcullisChevalManipulatorRaiseAtPitch(double pitch) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myPitch = pitch;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
setTimeout(5);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (Robot.gyroRotate.getPitch() > myPitch)
			Robot.manipulators.portcullisChevalManipulatorRaise();

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return (Robot.gyroRotate.getPitch() > myPitch + 1) || isTimedOut();
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
