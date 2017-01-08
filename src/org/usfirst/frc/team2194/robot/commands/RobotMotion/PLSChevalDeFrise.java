package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PLSChevalDeFrise extends Command {

	private double on1EncoderCounts;
	private double currentPosition;
	private boolean on1Dn;
	private boolean on2Dn;

	public PLSChevalDeFrise() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		on1EncoderCounts = Robot.prefs.getDouble("Chdf ManDn", 3)
				* Robot.encoderCountsPerLoGearInch;
		setTimeout(10);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		currentPosition = RobotMap.leftEncoder.getDistance();
		if (!on1Dn && (currentPosition > on1EncoderCounts)) {
			Robot.manipulators.portcullisChevalManipulatorLower();
			on1Dn = true;
		}
		if (!on2Dn && (Robot.gyroRotate.getPitch() > Robot.prefs.getDouble("Chdf ManUp", 3))) {
			Robot.manipulators.portcullisChevalManipulatorRaise();
			on2Dn = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || on2Dn;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.manipulators.portcullisChevalManipulatorRaise();
		on1Dn = false;
		on2Dn = false;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
